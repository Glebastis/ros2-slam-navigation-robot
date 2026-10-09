#!/usr/bin/env python3
"""Объединяет несколько LaserScan (по одному на каждую камеру) в один скан в общем кадре.

Зачем: одна depth-камера даёт слишком узкий скан (85.9°), и Karto-матчер «сползает» вдоль
плоской стены. Две камеры (например, спереди и сзади) дают две полосы обзора, и положение
вдоль стены перестаёт быть неопределённым.

Важная деталь: складывать сообщения LaserScan «в лоб» нельзя — дальности в каждом скане
измерены от **своей** камеры, и у скана одно начало координат. Поэтому узел:

  1. разворачивает лучи каждого входного скана в точки в кадре его камеры
     (это корректно, если входной узел depthimage_to_laserscan публикует скан с
     output_frame = camera_link этой камеры — так он и настроен);
  2. переводит точки в общий кадр (по умолчанию footprint_link) через TF;
  3. раскладывает их по общему угловому шагу, оставляя минимальную дальность в секторе.

На выходе — один скан, в котором дальности честно измерены от начала общего кадра
(поэтому его frame_id можно указывать без вранья, в отличие от «подписи» в
depthimage_to_laserscan).

Параметры (ROS, можно задавать из launch или через --ros-args -p):
    inputs    (string[])  входные топики, по умолчанию ['/scan_front', '/scan_rear']
    output    (string)    выходной топик, по умолчанию '/scan'
    frame     (string)    общий кадр, по умолчанию 'footprint_link'
    bins      (int)       число угловых секторов (720 = 0.5°)
    range_min / range_max (double)
    max_skew  (double)    максимальная рассинхронизация камер, с

Запуск вручную:
    ros2 run my_robot_bringup merge_scans.py --ros-args \
        -p inputs:="['/scan_front', '/scan_rear']" -p output:=/scan
"""
import math
import re
import sys
from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def stamp_of(scan):
    return scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9


class ScanMerger(Node):
    def __init__(self):
        super().__init__('merge_scans')
        raw_inputs = self.declare_parameter('inputs', ['/scan_front', '/scan_rear']).value
        if isinstance(raw_inputs, str):          # на случай, если пришло строкой
            raw_inputs = [t.strip("'\"") for t in
                          re.split(r'[,\s]+', raw_inputs.strip("[]'\" ")) if t.strip("'\"")]
        inputs = list(raw_inputs)
        output = self.declare_parameter('output', '/scan').value
        frame = self.declare_parameter('frame', 'footprint_link').value
        bins = int(self.declare_parameter('bins', 720).value)
        range_min = float(self.declare_parameter('range_min', 0.05).value)
        range_max = float(self.declare_parameter('range_max', 5.5).value)
        max_skew = float(self.declare_parameter('max_skew', 0.05).value)
        self.frame = frame
        self.bins = bins
        self.range_min = range_min
        self.range_max = range_max
        self.max_skew = max_skew          # максимальная рассинхронизация камер, с
        self.step = 2.0 * math.pi / bins
        self.buf = {t: deque(maxlen=6) for t in inputs}
        self.last_signature = None
        self.dropped = 0
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        for t in inputs:
            self.create_subscription(
                LaserScan, t, lambda m, t=t: self.on_scan(m, t), qos)
        self.pub = self.create_publisher(LaserScan, output, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.warned = set()
        self.get_logger().info(
            f'сливаю {inputs} -> {output} в кадре {frame}, {bins} секторов, '
            f'макс. рассинхрон камер {max_skew * 1000:.0f} мс')

    def on_scan(self, msg, topic):
        """Берём самый свежий скан и к нему — ближайшие по времени сканы других камер.

        Важно: если камеры идут с разной частотой (в симуляции на software-рендере это
        норма), нельзя склеивать свежий скан одной камеры с устаревшим от другой — при
        повороте это даёт перекос внутри одного скана и матчер начинает «сползать».
        """
        self.buf[topic].append(msg)
        t_new = stamp_of(msg)
        chosen = {topic: msg}
        for other, buf in self.buf.items():
            if other == topic or not buf:
                continue
            best = min(buf, key=lambda s: abs(stamp_of(s) - t_new))
            if abs(stamp_of(best) - t_new) <= self.max_skew:
                chosen[other] = best
        signature = tuple(sorted((t, round(stamp_of(s), 3)) for t, s in chosen.items()))
        if signature == self.last_signature:
            return
        self.last_signature = signature
        if len(chosen) < 2:
            self.dropped += 1
        self.publish_merged(chosen)

    def publish_merged(self, scans):
        acc = np.full(self.bins, np.inf)      # inf -> «нет данных», в NaN переведём в конце
        stamp = None
        stamp_sec = None
        for scan in scans.values():
            ts = stamp_of(scan)
            if stamp_sec is None or ts > stamp_sec:
                stamp_sec = ts
                stamp = scan.header.stamp      # метка = самый свежий из склеенных
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.frame, scan.header.frame_id, Time(), Duration(seconds=0.2))
            except Exception as exc:                       # noqa: BLE001
                key = scan.header.frame_id
                if key not in self.warned:
                    self.warned.add(key)
                    self.get_logger().warn(
                        f'нет TF {key} -> {self.frame}: {exc}; скан пропущен')
                continue

            r = np.asarray(scan.ranges, dtype=np.float64)
            ang = scan.angle_min + np.arange(r.size) * scan.angle_increment
            ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
            if not ok.any():
                continue
            x = r[ok] * np.cos(ang[ok])
            y = r[ok] * np.sin(ang[ok])
            yaw = yaw_of(tf.transform.rotation)
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            X = tx + x * math.cos(yaw) - y * math.sin(yaw)
            Y = ty + x * math.sin(yaw) + y * math.cos(yaw)
            R = np.hypot(X, Y)
            A = np.arctan2(Y, X)
            idx = np.floor((A + math.pi) / self.step).astype(int) % self.bins
            good = (R >= self.range_min) & (R <= self.range_max)
            np.minimum.at(acc, idx[good], R[good])

        if stamp is None:
            return
        out = LaserScan()
        out.header.stamp = stamp
        out.header.frame_id = self.frame
        out.angle_min = -math.pi
        out.angle_increment = self.step
        out.angle_max = -math.pi + self.step * (self.bins - 1)
        out.range_min = float(self.range_min)
        out.range_max = float(self.range_max)
        out.scan_time = 0.1
        acc[np.isinf(acc)] = np.nan
        out.ranges = acc.tolist()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
