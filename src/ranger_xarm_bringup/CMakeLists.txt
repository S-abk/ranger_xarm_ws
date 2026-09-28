cmake_minimum_required(VERSION 3.8)
project(ranger_xarm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_deadman.py
  scripts/imu_yaw_bias_corrector.py
  scripts/lidar_odometry_relay.py
  scripts/lidar_terrain_filter.py
  scripts/arm_pose.py
  scripts/ranger_mode_arbiter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
