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

echo "########## $WORLD (launch logs in $LOG) ##########"
for i in $(seq 1 $N); do
  echo "=== trial $i ==="
  $S/reset_pose.sh >/dev/null 2>&1
  sleep 8
  timeout 200 python3 $S/ekf_score.py 2>&1 | tail -4
done

pkill -f "ekf_odom_imu.launch"; pkill -f "sim.launch.py"; sleep 3
pkill -9 -f "sim.launch.py"; pkill -9 -f "gz sim"
pkill -9 -f "/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge"
pkill -9 -f "/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher"
pkill -9 -f "ekf_node"; pkill -9 -f "wheel_odometry.py"
pkill -9 -f "ranger_4wis_controller.py"; pkill -9 -f "imu_yaw_bias_corrector.py"
sleep 5
echo "teardown: $(ps -eo comm= | grep -cE '^(gz|robot_state_pub|parameter_brid|ekf_node)$') sim procs left"
