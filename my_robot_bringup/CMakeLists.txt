cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz worlds config
  DESTINATION share/${PROJECT_NAME}/
)

# узел слияния сканов с нескольких depth-камер (см. SLAM_FIX_NOTES.md §8.3)
# и узел сборки скана из облака точек полосой по высоте (§9.2)
install(PROGRAMS
  scripts/merge_scans.py
  scripts/cloud_to_scan.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
