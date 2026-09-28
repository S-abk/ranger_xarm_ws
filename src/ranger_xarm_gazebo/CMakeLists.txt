cmake_minimum_required(VERSION 3.8)
project(ranger_xarm_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config models
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/ranger_4wis_controller.py
  scripts/wheel_odometry.py
  scripts/gz_clock_relay.py
  scripts/arc_probe.py
  scripts/crab_probe.py
  scripts/crab_timeline.py
  scripts/make_surface_worlds.py
  scripts/run_surface_sweep.sh
  scripts/reset_pose.sh
  scripts/ekf_score.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
