cmake_minimum_required(VERSION 3.8)
project(ranger_xarm_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gripper_action_proxy.py
  scripts/trusted_manipulation_volume_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
