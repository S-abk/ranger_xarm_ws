cmake_minimum_required(VERSION 3.8)
project(ranger_xarm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch meshes
  DESTINATION share/${PROJECT_NAME}
)
# CC-BY 4.0 text for the printable mount meshes, installed beside them.
install(FILES LICENSE-CC-BY-4.0
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
