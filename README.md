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

