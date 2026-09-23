cmake_minimum_required(VERSION 3.8)
project(ranger_xarm_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
