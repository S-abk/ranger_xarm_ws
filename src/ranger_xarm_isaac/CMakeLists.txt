cmake_minimum_required(VERSION 3.8)
project(ranger_xarm_isaac)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Scripts must be listed here individually or they are not installed to
# lib/${PROJECT_NAME} and the launch files cannot find them.
install(PROGRAMS
  scripts/urdf_to_usd.py
  scripts/isaac_bringup.py
  scripts/joint_state_effort_filter.py
  scripts/joint_command_merger.py
  scripts/isaac_sensors.py
  scripts/isaac_contacts.py
  scripts/isaac_scenes.py
  scripts/isaac_contact_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
