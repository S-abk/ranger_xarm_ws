#!/bin/bash
# Obstacle-map check: the lidar obstacle map with the base driven in Gazebo.
#
#   tools/smoke_test/obstacle_map.sh [mobile|welded|free ...]   (default: all three)
#
#   mobile  moveit.launch.py drive_base:=true obstacle_map:=true -- the shipped design:
#           MoveIt plans in odom (planar virtual joint), the map stays put in the world.
#           Must pass T1-T6 and plan arm-only trajectories that execute.
#   welded  moveit.launch.py obstacle_map:=true (drive_base left false) beside a driven
#           base: MoveIt's planning frame, world, is not in the TF tree, so no map builds.
#           Negative control: must fail T3 (it plans into the obstacle).
#   free    moveit.launch.py drive_base:=true obstacle_map:=true planning_frame:=base_footprint:
#           the selectable alternative, MoveIt plans on the base. Negative control for this
#           check: it must fail T6 (its map moves with the robot and smears in motion).
# Tolerances and sequence: obstacle_probe.py. About three and a half minutes per variant.
#
# Same isolation as gazebo.sh: its own ROS domain (SMOKE_DOMAIN_ID, default 42), discovery
# kept to this machine, its own gz partition. SMOKE_GPU=nvidia renders on the NVIDIA card
# (sim.launch.py gpu:=). Exit 0: as expected; 1: not; 2: the simulation never came up;
# 3: inconclusive (the box was not seen, or odometry drifted).

HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
WS=${WS:-$(cd "$HERE/../.." && pwd)}
VARIANTS=${*:-mobile welded free}
for v in $VARIANTS; do case $v in mobile|welded|free) ;; *) echo "usage: $0 [mobile|welded|free ...]"; exit 2;; esac; done
LOG_DIR=${LOG_DIR:-$(mktemp -d -t ranger_xarm_obstacle.XXXX)}
mkdir -p "$LOG_DIR"

export ROS_DOMAIN_ID=${SMOKE_DOMAIN_ID:-42} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=ranger_xarm_obstacle_$$
source "$WS/install/setup.bash" || { echo "build the workspace first: $WS/install"; exit 2; }

PG= MG=
stop() {
  [ -z "$PG" ] && return
  for g in $MG $PG; do kill -INT -- -"$g" 2>/dev/null; done
  for _ in $(seq 1 10); do
    alive=; for g in $MG $PG; do kill -0 -- -"$g" 2>/dev/null && alive=1; done
    [ -z "$alive" ] && break; sleep 1
  done
  for g in $MG $PG; do kill -9 -- -"$g" 2>/dev/null; done
  pkill -9 -f "gz sim.*$GZ_PARTITION" 2>/dev/null
  PG= MG=
}
trap 'stop; ros2 daemon stop >/dev/null 2>&1' EXIT

rc=0
for v in $VARIANTS; do
  echo "=== obstacle map, $v"
  setsid ros2 launch ranger_xarm_gazebo sim.launch.py headless:=true drive_base:=true sensors:=true \
    gpu:="${SMOKE_GPU:-default}" > "$LOG_DIR/${v}_sim.log" 2>&1 &
  PG=$!
  up=
  for i in $(seq 1 180); do
    # timeout: if gz-sim dies at start-up the call can block for good instead of failing
    timeout 10 ros2 control list_controllers 2>/dev/null | grep -q "ranger_wheel_controller .*active" && { up=$i; break; }
    kill -0 "$PG" 2>/dev/null || break
    sleep 1
  done
  if [ -z "$up" ]; then
    echo "FAIL  controllers never came up; tail of $LOG_DIR/${v}_sim.log:"; tail -20 "$LOG_DIR/${v}_sim.log"
    stop; rc=2; continue
  fi
  case $v in
    mobile) setsid ros2 launch ranger_xarm_isaac moveit.launch.py drive_base:=true obstacle_map:=true \
              > "$LOG_DIR/${v}_moveit.log" 2>&1 & expect=pass ;;
    welded) setsid ros2 launch ranger_xarm_isaac moveit.launch.py obstacle_map:=true \
              > "$LOG_DIR/${v}_moveit.log" 2>&1 & expect=fail-T3 ;;
    free)   setsid ros2 launch ranger_xarm_isaac moveit.launch.py drive_base:=true obstacle_map:=true \
              planning_frame:=base_footprint > "$LOG_DIR/${v}_moveit.log" 2>&1 & expect=fail-T6 ;;
  esac
  MG=$!
  timeout 900 python3 "$HERE/obstacle_probe.py" "$v" "$LOG_DIR" --expect $expect | tee "$LOG_DIR/${v}.out"
  r=${PIPESTATUS[0]}
  (timeout 20 ros2 run tf2_tools view_frames -o "$LOG_DIR/${v}_frames" > /dev/null 2>&1)
  stop; sleep 3
  if [ $r -ne 0 ]; then [ $rc -eq 0 ] || [ $rc -eq 3 ] && rc=$r; fi
done
echo "logs: $LOG_DIR"
case $rc in
  0) echo "obstacle-map check PASSED";;
  3) echo "obstacle-map check INCONCLUSIVE";;
  *) echo "obstacle-map check FAILED";;
esac
exit $rc
