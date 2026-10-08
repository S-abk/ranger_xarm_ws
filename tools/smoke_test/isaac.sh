#!/bin/bash
# Smoke-test the Isaac Sim bringup: generate the wheeled USD, start Isaac
# headless, start control.launch.py drive_base:=true, run probe.py with
# every check (drive, sensors, arm), shut it all down.
#
#   tools/smoke_test/isaac.sh
#
# Needs ISAACSIM_PYTHON_EXE (see the top-level README) and the workspace
# built with colcon build --packages-up-to ranger_xarm_isaac. The USD goes
# to the log directory, so the one in install/ is left alone. Isolation,
# logs and exit status are as for gazebo.sh.

HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
WS=${WS:-$(cd "$HERE/../.." && pwd)}
LOG_DIR=${LOG_DIR:-$(mktemp -d -t ranger_xarm_smoke.XXXX)}

[ -x "$ISAACSIM_PYTHON_EXE" ] || { echo "set ISAACSIM_PYTHON_EXE to Isaac Sim's python"; exit 2; }
export ROS_DOMAIN_ID=${SMOKE_DOMAIN_ID:-42} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source "$WS/install/setup.bash" || { echo "build the workspace first: $WS/install"; exit 2; }
# Without the large shared-memory profile most Ouster scans are dropped;
# every process here inherits it, Isaac included.
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix ranger_xarm_bringup)/share/ranger_xarm_bringup/config/fastdds_large_shm.xml
P=$(ros2 pkg prefix ranger_xarm_isaac)
USD=$LOG_DIR/usd/ranger_xarm_wheeled.usd

IG= CG=
stop() {
  for g in $CG $IG; do kill -INT -- -"$g" 2>/dev/null; done
  for _ in $(seq 1 10); do
    alive=; for g in $CG $IG; do kill -0 -- -"$g" 2>/dev/null && alive=1; done
    [ -z "$alive" ] && break; sleep 1
  done
  for g in $CG $IG; do kill -9 -- -"$g" 2>/dev/null; done
  IG= CG=
}
trap 'stop; ros2 daemon stop >/dev/null 2>&1' EXIT
fail_setup() { echo "FAIL  $1; tail of $2:"; tail -20 "$2"; echo "isaac smoke test FAILED"; exit 2; }

echo "=== isaac: generating the wheeled USD"
"$ISAACSIM_PYTHON_EXE" "$P/lib/ranger_xarm_isaac/urdf_to_usd.py" --force --output "$USD" \
    use_wheels:=true fix_base_to_world:=false wheels_command_interface:=velocity \
    > "$LOG_DIR/urdf_to_usd.log" 2>&1 && [ -f "$USD" ] || fail_setup "USD generation" "$LOG_DIR/urdf_to_usd.log"

echo "=== isaac: isaac_bringup.py --headless"
setsid "$ISAACSIM_PYTHON_EXE" "$P/lib/ranger_xarm_isaac/isaac_bringup.py" --headless --usd "$USD" \
    > "$LOG_DIR/isaac.log" 2>&1 &
IG=$!
up=
for i in $(seq 1 150); do
  timeout 5 ros2 topic echo --once /clock > /dev/null 2>&1 && { up=$i; break; }
  kill -0 "$IG" 2>/dev/null || break
  sleep 1
done
[ -n "$up" ] || fail_setup "Isaac never published /clock" "$LOG_DIR/isaac.log"
echo "Isaac publishing /clock"

echo "=== isaac: control.launch.py drive_base:=true"
setsid ros2 launch ranger_xarm_isaac control.launch.py drive_base:=true > "$LOG_DIR/control.log" 2>&1 &
CG=$!
up=
for i in $(seq 1 120); do
  # Controllers spawn one after another; the wheels' is last.
  ros2 control list_controllers 2>/dev/null | grep -q "ranger_wheel_controller .*active" && { up=$i; break; }
  kill -0 "$CG" 2>/dev/null || break
  sleep 1
done
[ -n "$up" ] || fail_setup "controllers never came up" "$LOG_DIR/control.log"
echo "controllers active after ${up} s"

python3 "$HERE/probe.py" --drive --sensors --arm; rc=$?
stop
echo "logs: $LOG_DIR"
[ $rc -eq 0 ] && echo "isaac smoke test PASSED" || echo "isaac smoke test FAILED"
exit $rc
