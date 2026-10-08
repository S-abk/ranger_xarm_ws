#!/bin/bash
# Smoke-test the gz bringup: launch it headless, run probe.py, shut it down.
#
#   tools/smoke_test/gazebo.sh [arm|base|all]      (default all)
#
#   arm   sim.launch.py as it ships: base welded to the world; the arm moves
#   base  drive_base:=true sensors:=true; drive, sensors and the arm
#
# Needs the workspace built (colcon build --packages-up-to ranger_xarm_gazebo).
# Runs on its own ROS domain (SMOKE_DOMAIN_ID, default 42) with discovery
# kept to this machine, and on its own gz partition, so it neither sees nor
# disturbs anything else running. Launch logs go to $LOG_DIR (default: a
# fresh temporary directory, printed at the end).
# Exit status: 0 all passed, 1 a check failed, 2 the simulation never came up.

HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
WS=${WS:-$(cd "$HERE/../.." && pwd)}
WHICH=${1:-all}
case $WHICH in arm|base|all) ;; *) echo "usage: $0 [arm|base|all]"; exit 2;; esac
LOG_DIR=${LOG_DIR:-$(mktemp -d -t ranger_xarm_smoke.XXXX)}

export ROS_DOMAIN_ID=${SMOKE_DOMAIN_ID:-42} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=ranger_xarm_smoke_$$
# The ROS setup files read unset variables, so no `set -u` before this.
source "$WS/install/setup.bash" || { echo "build the workspace first: $WS/install"; exit 2; }

PG=
stop() {
  [ -z "$PG" ] && return
  kill -INT -- -"$PG" 2>/dev/null
  for _ in $(seq 1 10); do kill -0 -- -"$PG" 2>/dev/null || break; sleep 1; done
  kill -9 -- -"$PG" 2>/dev/null
  pkill -9 -f "gz sim.*$GZ_PARTITION" 2>/dev/null   # gz can outlive its launch
  PG=
}
trap 'stop; ros2 daemon stop >/dev/null 2>&1' EXIT

# scenario <name> <last controller spawned> <probe args...> -- <sim.launch.py args...>
scenario() {
  local name=$1 ready=$2; shift 2
  local probe=(); while [ "$1" != "--" ]; do probe+=("$1"); shift; done; shift
  local log=$LOG_DIR/gazebo_$name.log
  echo "=== gazebo $name: sim.launch.py headless:=true $*"
  setsid ros2 launch ranger_xarm_gazebo sim.launch.py headless:=true "$@" > "$log" 2>&1 &
  PG=$!
  local up=
  for i in $(seq 1 120); do
    # Controllers spawn one after another, so the last one active means all are.
    ros2 control list_controllers 2>/dev/null | grep -q "$ready .*active" && { up=$i; break; }
    kill -0 "$PG" 2>/dev/null || break
    sleep 1
  done
  if [ -z "$up" ]; then
    echo "FAIL  controllers never came up; tail of $log:"; tail -20 "$log"; stop; return 2
  fi
  echo "controllers active after ${up} s"
  python3 "$HERE/probe.py" "${probe[@]}"; local rc=$?
  stop
  return $rc
}

rc=0
case $WHICH in arm|all) scenario arm xarm_xarm_gripper_traj_controller --arm -- || rc=$?;; esac
case $WHICH in base|all)
  scenario base ranger_wheel_controller --drive --sensors --arm -- drive_base:=true sensors:=true || { r=$?; [ $rc -eq 0 ] && rc=$r; };;
esac
echo "logs: $LOG_DIR"
[ $rc -eq 0 ] && echo "gazebo smoke test PASSED" || echo "gazebo smoke test FAILED"
exit $rc
