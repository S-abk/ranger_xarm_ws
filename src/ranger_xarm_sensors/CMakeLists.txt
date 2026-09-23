cmake_minimum_required(VERSION 3.8)
project(ranger_xarm_sensors)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ouster_points_viz src/ouster_points_viz.cpp)
ament_target_dependencies(ouster_points_viz rclcpp sensor_msgs)

install(TARGETS
  ouster_points_viz
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/rplidar_scan_check.py
  scripts/rplidar_watchdog.py
  scripts/d435_check.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
