#!/bin/bash
# Bring up gz on one surface world, start the EKF, score N trials, tear down.
# Each trial teleports the base back to a fixed start so the trials are
# independent rather than resuming wherever the last one got stuck.
# NB: no `set -u` here -- the ROS setup files read unset trace variables
# (AMENT_TRACE_SETUP_FILES, COLCON_TRACE) and would abort the script.
WS=${WS:-$HOME/ranger_xarm_ws}
S=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORLD=$1; N=${2:-5}
LOG=${LOG_DIR:-$(mktemp -d)}   # launch logs; override with LOG_DIR=
source $WS/install/setup.bash >/dev/null 2>&1

nohup ros2 launch ranger_xarm_gazebo sim.launch.py drive_base:=true sensors:=true \
  wheel_drive:=${WHEEL_DRIVE:-effort} \
  headless:=true odom_tf:=false world:=$WS/src/ranger_xarm_gazebo/worlds/$WORLD.sdf \
  > $LOG/${WORLD}_sim.log 2>&1 &
for i in $(seq 1 60); do
  grep -q "Successfully switched controllers" $LOG/${WORLD}_sim.log 2>/dev/null && break
  sleep 5
done
sleep 15

nohup ros2 launch ranger_xarm_bringup ekf_odom_imu.launch.py use_sim_time:=true \
  > $LOG/${WORLD}_ekf.log 2>&1 &
sleep 30

echo "########## $WORLD, wheel_drive=${WHEEL_DRIVE:-effort} (launch logs in $LOG) ##########"
for i in $(seq 1 $N); do
  echo "=== trial $i ==="
  $S/reset_pose.sh >/dev/null 2>&1
  sleep 8
  timeout 200 python3 $S/ekf_score.py 2>&1 | tail -4
done

pkill -f "ekf_odom_imu.launch"; pkill -f "sim.launch.py"; sleep 3
for pat in "sim.launch.py" "gz sim" "parameter_bridge" "robot_state_publisher" \
           "static_transform_publisher" "gz_clock_relay.py" "ekf_node" \
           "wheel_odometry.py" "ranger_4wis_controller.py" \
           "imu_yaw_bias_corrector.py"; do
  pkill -9 -f "$pat"
done
sleep 5
# Every kill -9 leaves a Fast DDS shared-memory segment behind, and stale
# ones serve ghost endpoints to the next run -- once, a controller_manager
# that answered for a controller the live one had never loaded.
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
# Count by command line, not process name: the gz server's comm is "ruby",
# so a check for a process called gz reports zero while one is running --
# which is how a stale server once survived teardown for hours.
left=$(pgrep -f "gz sim|robot_state_publisher|parameter_bridge|ekf_node|gz_clock_relay" \
       | grep -vx "$$" | wc -l)
echo "teardown: $left sim procs left"
