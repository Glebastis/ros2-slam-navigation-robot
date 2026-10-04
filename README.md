# ros2-slam-navigation-robot

Автономный мобильный робот на ROS 2. Дифференциальный привод, depth-камера (Orbbec Astra), ультразвуковые датчики, GPS. SLAM, навигация, симуляция в Gazebo. Целевое железо — Raspberry Pi 5.

## Стек

- **ROS 2 Humble** (Ubuntu 22.04)
- **Gazebo Classic** + `gazebo_ros_pkgs`
- **slam_toolbox** — SLAM в режиме mapping
- **Nav2** — планирование пути и управление
- **depthimage_to_laserscan** — генерация `/scan` из depth-камеры
- **robot_localization** — слияние одометрии и GPS (план)
- **Docker** — воспроизводимая сборка для Raspberry Pi 5

## Железо

| Компонент | Модель |
|---|---|
| Вычислитель | Raspberry Pi 5 |
| Камера | Orbbec Astra (depth) |
| Дальномеры | HC-SR04 × 3 (front, left, right) |
| GPS | NMEA-модуль (UART) |
| Привод | Дифференциальный, 2 ведущих колеса + кастер |

## Структура репозитория

├── my_robot_bringup  
│   ├── CMakeLists.txt  
│   ├── config  
│   │   └── slam_params.yaml  
│   ├── launch  
│   │   └── my_robot_gazebo.launch.xml  
│   ├── package.xml  
│   ├── rviz  
│   │   └── urdf_config.rviz  
│   └── worlds  
│       └── test_world.world  
├── my_robot_description  
│   ├── CMakeLists.txt  
│   ├── config  
│   │   └── config.rviz  
│   ├── launch  
│   │   ├── display.launch.py  
│   │   └── display.launch.xml  
│   ├── package.xml  
│   └── urdf  
│       ├── camera.xacro  
│       ├── common_properties.xacro  
│       ├── gps_module.xacro  
│       ├── mobile_base_gazebo.xacro  
│       ├── mobile_base.xacro  
│       ├── my_robot.urdf.xacro  
│       └── sonic_radar.xacro  
├── Dockerfile  
├── requirements.txt  
├── docker-compose.yaml  
└── README.md  


## Быстрый старт (симуляция)

```bash
# 1. Сборка
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 2. Запуск Gazebo + SLAM + RViz
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

В RViz добавьте дисплеи:
- `Map` → `/map` (Durability: Transient Local)
- `LaserScan` → `/scan`
- `TF` → все фреймы
- `InteractiveMarkers` → `/slam_toolbox/update`

## Управление

```bash
# Движение вперёд
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}"

# Поворот на месте
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.3}}"
```

## Топики

| Топик | Тип | Источник |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | depthimage_to_laserscan |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox |
| `/odom` | `nav_msgs/Odometry` | diff_drive_controller |
| `/cmd_vel` | `geometry_msgs/Twist` | внешний |
| `/depth_camera/depth/image_raw` | `sensor_msgs/Image` | depth-камера |
| `/depth_camera/points` | `sensor_msgs/PointCloud2` | depth-камера |
| `/front_ultrasonic/range` | `sensor_msgs/Range` | HC-SR04 (front) |
| `/left_ultrasonic/range` | `sensor_msgs/Range` | HC-SR04 (left) |
| `/right_ultrasonic/range` | `sensor_msgs/Range` | HC-SR04 (right) |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS-модуль |

## TF-дерево

```
map  
└── odom  
    └── footprint_link  
        └── base_link  
            ├── camera_link  
            │   └── camera_link_optical  
            ├── front_ultrasonic_link  
            ├── left_ultrasonic_link  
            ├── right_ultrasonic_link  
            ├── gps_link  
            ├── left_wheel_link  
            ├── right_wheel_link  
            └── caster_wheel_link  
```

## Известные ограничения

- **`/scan` из depth-камеры имеет узкое поле зрения (~60°).** Это затрудняет автоматический loop closure. Для стабильного SLAM рекомендуется увеличить `horizontal_fov` в `camera.xacro` до 1.5–1.9 рад.  
- **Интерактивный режим slam_toolbox не создаёт маркеры** в текущей сборке, несмотря на `enable_interactive_mode: true`. Ручное замыкание петли недоступно.  
- **`debug_logging: true` не даёт DEBUG-логов от slam_toolbox** — это известная особенность пакета. Используйте `--ros-args --log-level debug` (и фильтруйте по имени логгера).  
- **GPS в симуляции** публикует `NavSatFix` без слияния с одометрией. `robot_localization` пока не подключён.  

## TODO

### Сделано ✅

- [x] URDF/Xacro с модульной структурой (base, camera, ultrasonic, GPS)  
- [x] Дифференциальный привод в Gazebo (`libgazebo_ros_diff_drive.so`)  
- [x] Depth-камера (`libgazebo_ros_camera.so`) с optical frame  
- [x] Ультразвуковые датчики (3 шт.) через `libgazebo_ros_ray_sensor.so`  
- [x] GPS через `libgazebo_ros_gps_sensor.so`  
- [x] Генерация `/scan` из depth-камеры (`depthimage_to_laserscan`)  
- [x] Запуск `slam_toolbox` в режиме mapping, карта строится  
- [x] Топики датчиков публикуются, TF-цепочка корректна  

### В работе 🚧

- [ ] **Стабильный loop closure.** Сейчас карта «затирается» при повторном прохождении. Причина — узкий FOV скана. План: увеличить `horizontal_fov` до 1.5–1.9 рад, снизить `loop_match_minimum_response_fine` до 0.30, `loop_match_minimum_chain_size` до 3, `minimum_travel_heading` до 0.25.  
- [ ] **Диагностика scan matching.** Проверить, не «телепортируется» ли поза робота при поворотах. Если да — временно отключить `use_scan_matching` и сравнить поведение.  

### Не начато ❌

- [ ] **Nav2.** Настроить `nav2_params.yaml`, запустить `navigation_launch.py`, отправить `NavigateToPose` через RViz.  
- [ ] **Costmap-слои.** Подключить ультразвук и GPS в costmap'ы Nav2.  
- [ ] **robot_localization.** Слияние одометрии колёс и GPS в единый `/odom` (EKF).  
- [ ] **Docker.** Dockerfile с ROS 2 Humble и всеми зависимостями для Raspberry Pi 5.  
- [ ] **Реальное железо.** Сборка, отладка электроники, полевые испытания.  
- [ ] **Orbbec Astra на реальном железе.** Запуск `orbbec_camera` через OrbbecSDK_ROS2.  
- [ ] **NMEA-драйвер GPS.** Парсинг NMEA-сообщений через `nmea_navsat_driver`.  
- [ ] **Twist mux.** Приоритизация команд от Nav2, телеопа и аварийной остановки.  
- [ ] **Тесты.** Юнит-тесты для узлов, integration-тесты в Gazebo.  
- [ ] **Документация.** Описание API, схемы подключения, калибровки.  

## Лицензия

MIT
