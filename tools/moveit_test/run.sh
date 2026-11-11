#!/bin/bash
# MoveIt test in simulation: bring up the simulator with the arm welded to the
# world, its controllers, move_group (ranger_xarm_isaac moveit.launch.py, as
# shipped) and the true-model reference checker, run moveit_test.py, shut down.
#
#   tools/moveit_test/run.sh [gz|isaac] [moveit_test.py args...]   (default gz)
#
# Needs the workspace built (colcon build --packages-up-to ranger_xarm_gazebo
# ranger_xarm_isaac); isaac also needs ISAACSIM_PYTHON_EXE (top-level README).
# Runs on its own ROS domain (MOVEIT_TEST_DOMAIN_ID, default 43) with discovery
# kept to this machine, and gz on its own partition. Logs and results go to
# $LOG_DIR (default: a fresh temporary directory, printed at the end).
# Exit status: 0 all passed, 1 a check failed, 2 the setup never came up.

HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
WS=${WS:-$(cd "$HERE/../.." && pwd)}
SIM=${1:-gz}; [ $# -gt 0 ] && shift
case $SIM in gz|isaac) ;; *) echo "usage: $0 [gz|isaac] [moveit_test.py args...]"; exit 2;; esac
LOG_DIR=${LOG_DIR:-$(mktemp -d -t ranger_xarm_moveit_test.XXXX)}
mkdir -p "$LOG_DIR"

export ROS_DOMAIN_ID=${MOVEIT_TEST_DOMAIN_ID:-43} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=ranger_xarm_moveit_test_$$
if [ "$SIM" = isaac ] && [ ! -x "$ISAACSIM_PYTHON_EXE" ]; then
  echo "set ISAACSIM_PYTHON_EXE to Isaac Sim's python"; exit 2
fi
# The ROS setup files read unset variables, so no `set -u` before this.
source "$WS/install/setup.bash" || { echo "build the workspace first: $WS/install"; exit 2; }

GROUPS_=()
stop() {
  for g in "${GROUPS_[@]}"; do kill -INT -- -"$g" 2>/dev/null; done
  for _ in $(seq 1 15); do
    alive=; for g in "${GROUPS_[@]}"; do kill -0 -- -"$g" 2>/dev/null && alive=1; done
    [ -z "$alive" ] && break; sleep 1
  done
  for g in "${GROUPS_[@]}"; do kill -9 -- -"$g" 2>/dev/null; done
  pkill -9 -f "gz sim.*$GZ_PARTITION" 2>/dev/null   # gz can outlive its launch
  GROUPS_=()
}
trap 'stop; ros2 daemon stop >/dev/null 2>&1' EXIT
start() { local log=$1; shift; setsid "$@" > "$LOG_DIR/$log" 2>&1 & GROUPS_+=($!); }
# wait_for <seconds> <what> <log> <shell test>
wait_for() {
  local n=$1 what=$2 log=$3; shift 3
  for i in $(seq 1 "$n"); do
    eval "$*" > /dev/null 2>&1 && { echo "$what after ${i} s"; return 0; }
    sleep 1
  done
  echo "FAIL  $what: never came up; tail of $LOG_DIR/$log:"; tail -20 "$LOG_DIR/$log"; exit 2
}

if ros2 node list --no-daemon 2>/dev/null | grep -qE '^/(move_group|controller_manager)$'; then
  echo "ROS_DOMAIN_ID $ROS_DOMAIN_ID already has a move_group or controller_manager; set MOVEIT_TEST_DOMAIN_ID"
  exit 2
fi

READY="ros2 control list_controllers | grep -q 'xarm_xarm_gripper_traj_controller .*active'"
if [ "$SIM" = gz ]; then
  echo "=== gz: sim.launch.py headless:=true"
  start sim.log ros2 launch ranger_xarm_gazebo sim.launch.py headless:=true
  wait_for 180 "controllers active" sim.log "$READY"
else
  # Without the large shared-memory profile most Ouster scans are dropped.
  export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix ranger_xarm_bringup)/share/ranger_xarm_bringup/config/fastdds_large_shm.xml
  P=$(ros2 pkg prefix ranger_xarm_isaac)
  USD=$LOG_DIR/usd/ranger_xarm.usd   # generated here, so the one in install/ is left alone
  echo "=== isaac: generating the USD (arm welded to the world)"
  "$ISAACSIM_PYTHON_EXE" "$P/lib/ranger_xarm_isaac/urdf_to_usd.py" --force --output "$USD" \
      > "$LOG_DIR/urdf_to_usd.log" 2>&1 && [ -f "$USD" ] \
      || { echo "FAIL  USD generation; tail of $LOG_DIR/urdf_to_usd.log:"; tail -20 "$LOG_DIR/urdf_to_usd.log"; exit 2; }
  echo "=== isaac: isaac_bringup.py --headless"
  start isaac.log "$ISAACSIM_PYTHON_EXE" "$P/lib/ranger_xarm_isaac/isaac_bringup.py" --headless --usd "$USD"
  wait_for 300 "Isaac publishing /clock" isaac.log "timeout 5 ros2 topic echo --once /clock"
  start control.log ros2 launch ranger_xarm_isaac control.launch.py
  wait_for 180 "controllers active" control.log "$READY"
fi

echo "=== move_group (moveit.launch.py) and the reference checker (/ref)"
start move_group.log ros2 launch ranger_xarm_isaac moveit.launch.py
start ref_move_group.log ros2 launch "$HERE/ref_move_group.launch.py"
wait_for 120 "move_group up" move_group.log "ros2 action list | grep -qx /move_action"
wait_for 120 "reference checker up" ref_move_group.log "ros2 service list | grep -qx /ref/check_state_validity"
sleep 3

python3 "$HERE/moveit_test.py" --out "$LOG_DIR" "$@"; rc=$?
stop
echo "logs and results: $LOG_DIR"
case $rc in
  0) echo "moveit test ($SIM) PASSED";;
  1) echo "moveit test ($SIM) FAILED";;
  *) echo "moveit test ($SIM) could not run (exit $rc)";;
esac
exit $rc
