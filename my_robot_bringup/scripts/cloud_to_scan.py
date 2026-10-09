#!/usr/bin/env python3
"""Скан из облака точек depth-камеры: полоса по высоте вместо одной строки кадра.

Зачем. Штатный путь «depthimage_to_laserscan» берёт одну строку изображения глубины. Это
даёт три врождённые проблемы:

  * полоса строк вокруг горизонта смотрит в пол, а плагин оставляет ближайший пиксель
    по столбцу - скан превращается в «кольцо по полу» вокруг робота (1.19-1.90 м) и комната
    исчезает;
  * одна строка = «лазерная плоскость», которая «плавает» при любом тангаже робота;
  * в кадре 640x480 огромная часть измерений просто выбрасывается.

Этот узел работает с облаком точек (XYZ) и берёт из него **полосу по высоте**, то есть
препятствия в заданном диапазоне высот над полом. Пол исключается по построению, а не
«удачно выбранной строкой», и используются все точки полосы.

Скан публикуется в общем кадре (по умолчанию footprint_link), дальности честно
пересчитываются от начала этого кадра - то же правило, что и в merge_scans.py.

Параметры (ROS):
    inputs       (string[]) облака точек, по умолчанию ['/depth_camera/points']
    output       (string)    выходной топик, по умолчанию '/scan_band'
    frame        (string)    общий кадр, по умолчанию 'footprint_link'
    min_height   (double)    нижняя граница полосы над началом кадра, м (0.15)
    max_height   (double)    верхняя граница полосы, м (0.8)
    bins         (int)       число угловых секторов (720 = 0.5 град)
    range_min / range_max (double)
    max_skew     (double)    максимальная рассинхронизация облаков, с
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
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')
        raw = self.declare_parameter('inputs', ['/depth_camera/points']).value
        if isinstance(raw, str):
            raw = [t.strip("'\"") for t in re.split(r'[,\s]+', raw.strip("[]'\" ")) if t.strip("'\"")]
        self.inputs = list(raw)
        self.output = self.declare_parameter('output', '/scan_band').value
        self.frame = self.declare_parameter('frame', 'footprint_link').value
        self.min_height = float(self.declare_parameter('min_height', 0.15).value)
        self.max_height = float(self.declare_parameter('max_height', 0.8).value)
        self.bins = int(self.declare_parameter('bins', 720).value)
        self.range_min = float(self.declare_parameter('range_min', 0.2).value)
        self.range_max = float(self.declare_parameter('range_max', 6.0).value)
        self.max_skew = float(self.declare_parameter('max_skew', 0.05).value)
        self.step = 2.0 * math.pi / self.bins

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        for t in self.inputs:
            self.create_subscription(PointCloud2, t, lambda m, t=t: self.on_cloud(m, t), qos)
        self.pub = self.create_publisher(LaserScan, self.output, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.buf = {t: deque(maxlen=3) for t in self.inputs}
        self.last_signature = None
        self.warned = set()
        self.get_logger().info(
            f'полоса {self.min_height:.2f}-{self.max_height:.2f} м, облака {self.inputs} '
            f'-> {self.output} в кадре {self.frame}, {self.bins} секторов')

    def on_cloud(self, msg, topic):
        self.buf[topic].append(msg)

        # синхронность источников: облака новее max_skew не склеиваем
        t_new = stamp_of(msg)
        chosen = {topic: msg}
        for other, buf in self.buf.items():
            if other == topic or not buf:
                continue
            best = min(buf, key=lambda m: abs(stamp_of(m) - t_new))
            if abs(stamp_of(best) - t_new) <= self.max_skew:
                chosen[other] = best
        signature = tuple(sorted((t, round(stamp_of(m), 3)) for t, m in chosen.items()))
        if signature == self.last_signature:
            return
        self.last_signature = signature
        self.publish_scan(chosen)

    def publish_scan(self, clouds):
        acc = np.full(self.bins, np.inf)
        stamp_sec = None
        stamp = None
        for cloud in clouds.values():
            ts = stamp_of(cloud)
            if stamp_sec is None or ts > stamp_sec:
                stamp_sec, stamp = ts, cloud.header.stamp
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.frame, cloud.header.frame_id, Time(), Duration(seconds=0.2))
            except Exception as exc:                       # noqa: BLE001
                key = cloud.header.frame_id
                if key not in self.warned:
                    self.warned.add(key)
                    self.get_logger().warn(f'нет TF {key} -> {self.frame}: {exc}')
                continue

            pts = point_cloud2.read_points_numpy(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True)
            if pts.size == 0:
                continue
            p = pts.reshape(-1, 3).astype(np.float64)
            # в общий кадр
            yaw = yaw_of(tf.transform.rotation)
            cy, sy = math.cos(yaw), math.sin(yaw)
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tz = tf.transform.translation.z
            X = tx + p[:, 0] * cy - p[:, 1] * sy
            Y = ty + p[:, 0] * sy + p[:, 1] * cy
            Z = tz + p[:, 2]                      # высота точки над началом общего кадра
            band = (Z >= self.min_height) & (Z <= self.max_height)
            if not band.any():
                continue
            X, Y = X[band], Y[band]
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
    node = CloudToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
