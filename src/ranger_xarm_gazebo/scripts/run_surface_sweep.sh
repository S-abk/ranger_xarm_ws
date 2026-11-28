#!/bin/bash
# Bring up gz on one surface world, start the EKF, score N trials, tear down.
# Each trial teleports the base back to a fixed start so the trials are
# independent rather than resuming wherever the last one got stuck.
# NB: no `set -u` here -- the ROS setup files read unset trace variables
# (AMENT_TRACE_SETUP_FILES, COLCON_TRACE) and would abort the script.
S=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
WS=${WS:-$(cd "$S/../../.." && pwd)}   # the workspace this script's source lives in; override with WS=
WORLD=$1; N=${2:-5}
LOG=${LOG_DIR:-$(mktemp -d)}   # launch logs; override with LOG_DIR=
source $WS/install/setup.bash >/dev/null 2>&1

# Each launch runs in a process group of its own, so the teardown below reaches the
# processes this script started and nothing else on the machine.
own_group() { [ "$(ps -o pgid= -p "$1" 2>/dev/null | tr -d ' ')" = "$1" ]; }
setsid nohup ros2 launch ranger_xarm_gazebo sim.launch.py drive_base:=true sensors:=true \
  wheel_drive:=${WHEEL_DRIVE:-velocity} \
  headless:=true odom_tf:=false world:=$WS/src/ranger_xarm_gazebo/worlds/$WORLD.sdf \
  > $LOG/${WORLD}_sim.log 2>&1 &
SIM=$!
for i in $(seq 1 60); do
  grep -q "Successfully switched controllers" $LOG/${WORLD}_sim.log 2>/dev/null && break
  sleep 5
done
sleep 15

setsid nohup ros2 launch ranger_xarm_bringup ekf_odom_imu.launch.py use_sim_time:=true \
  > $LOG/${WORLD}_ekf.log 2>&1 &
EKF=$!
sleep 30

echo "########## $WORLD, wheel_drive=${WHEEL_DRIVE:-velocity} (launch logs in $LOG) ##########"
for i in $(seq 1 $N); do
  echo "=== trial $i ==="
  $S/reset_pose.sh >/dev/null 2>&1
  sleep 8
  timeout 200 python3 $S/ekf_score.py 2>&1 | tail -4
done

# Teardown: interrupt the two process groups this script started, give them 20 s
# to exit, then kill what is left of them. A group is signalled only while its
# leader is still the process recorded at launch and still leads that group.
# No other process is signalled and no shared-memory file is deleted: those may
# belong to other ROS processes on the machine.
for g in $EKF $SIM; do own_group $g && kill -INT -- -$g 2>/dev/null; done
for i in $(seq 1 20); do
  alive=; for g in $EKF $SIM; do pgrep -g $g >/dev/null && alive=1; done
  [ -z "$alive" ] && break
  sleep 1
done
for g in $EKF $SIM; do own_group $g && kill -KILL -- -$g 2>/dev/null; done
sleep 2
left=0
for g in $EKF $SIM; do
  left=$((left + $(pgrep -g $g | wc -l)))
done
echo "teardown: $left sim procs left"
