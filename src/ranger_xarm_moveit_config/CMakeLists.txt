cmake_minimum_required(VERSION 3.8)
project(ranger_xarm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# The semantic description for a driven base: the shipped one plus exactly a planar
# virtual joint odom -> base_footprint, generated on every build, never edited by hand.
set(MOBILE_BASE_SRDF ${CMAKE_CURRENT_BINARY_DIR}/ranger_xarm_mobile_base.srdf)
add_custom_command(
  OUTPUT ${MOBILE_BASE_SRDF}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_mobile_base_srdf.py
          ${CMAKE_CURRENT_SOURCE_DIR}/config/ranger_xarm.srdf ${MOBILE_BASE_SRDF}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/make_mobile_base_srdf.py
          ${CMAKE_CURRENT_SOURCE_DIR}/config/ranger_xarm.srdf
  COMMENT "Generating ranger_xarm_mobile_base.srdf"
)
add_custom_target(mobile_base_srdf ALL DEPENDS ${MOBILE_BASE_SRDF})
install(FILES ${MOBILE_BASE_SRDF} DESTINATION share/${PROJECT_NAME}/config)

install(PROGRAMS
  scripts/gripper_action_proxy.py
  scripts/trusted_manipulation_volume_filter.py
  scripts/make_mobile_base_srdf.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mobile_base_srdf test/test_mobile_base_srdf.py
    ENV MOBILE_BASE_SRDF=${MOBILE_BASE_SRDF}
    TIMEOUT 60)
endif()

ament_package()
