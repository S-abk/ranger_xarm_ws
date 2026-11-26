#!/bin/bash
# Runs inside the container that run.sh starts, as root, on a plain Ubuntu
# 24.04: installs ROS 2 Jazzy (ros-base) the documented way, then runs the
# README's own code blocks, extracted from the clone being tested, so the
# proof cannot drift from what the README says.
#
# A README block is the first ```bash fence after a line
#   <!-- fresh_clone: NAME -->
# It runs as written, with these substitutions only: `<this repo>` becomes
# the repository under test; every launch line (and its continuation lines)
# becomes a fixed no-op, its text saved beside the block, never run (a
# simulator would open a window and never return, and a physical-robot
# launch must never start here); `sudo rosdep init` becomes a no-op when
# rosdep is already initialised (the second pass). `sudo` is a no-op shim
# because the container runs as root, and apt answers yes. A block that
# still names a launch file, a ros2 run, gz sim or rviz2 anywhere after
# that is refused, not run.
#
# Env: REPO (clone URL or path), REF, DRIVERS (1 to run the hardware blocks),
# OUT (log directory, bind-mounted).

# Container only. run.sh sets FRESH_CLONE_CONTAINER=1 and Docker creates
# /.dockerenv; on a host this exits before doing anything (it would
# otherwise install packages as root).
if [ "$FRESH_CLONE_CONTAINER" != 1 ] || [ ! -e /.dockerenv ] || [ "$(id -u)" != 0 ]; then
  echo "walkthrough.sh runs only inside the container that run.sh starts" >&2
  exit 2
fi
export DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC
OUT=${OUT:-/out}
# Hand the logs back to the host user, however this ends.
trap '[ -n "$HOST_UID" ] && chown -R "$HOST_UID:${HOST_GID:-$HOST_UID}" "$OUT"' EXIT
STEPS=$OUT/steps.tsv
: > "$STEPS"

step() {  # step ID DESCRIPTION EXPECT COMMAND   (EXPECT: pass or fail)
  local id=$1 desc=$2 expect=$3 cmd=$4
  echo "=== [$id] $desc" | tee -a "$OUT/walkthrough.log"
  local t0=$SECONDS
  timeout -k 60 5400 bash -c "$cmd" >> "$OUT/walkthrough.log" 2>&1
  local rc=$? verdict
  if { [ "$expect" = pass ] && [ $rc -eq 0 ]; } || { [ "$expect" = fail ] && [ $rc -ne 0 ]; }; then
    verdict=OK; else verdict=UNEXPECTED; fi
  printf '%s\t%s\trc=%s\texpected %s\t%s\t%ss\n' "$id" "$desc" "$rc" "$expect" "$verdict" \
    "$((SECONDS - t0))" >> "$STEPS"
  echo "    rc=$rc ($verdict)" | tee -a "$OUT/walkthrough.log"
}

readme_block() {  # readme_block README NAME  -> the block's commands on stdout
  awk -v tag="<!-- fresh_clone: $2 -->" '
    $0 == tag {armed=1; next}
    armed && /^```bash/ {inblock=1; next}
    inblock && /^```/ {exit}
    inblock {print}' "$1"
}

run_block() {  # run_block ID NAME EXPECT [HOME]
  local id=$1 name=$2 expect=$3 home=${4:-/root}
  local src=$OUT/readme_$name.sh
  readme_block "$README" "$name" > "$src.orig"
  if [ ! -s "$src.orig" ]; then
    printf '%s\tREADME block %s\tmissing\texpected %s\tUNEXPECTED\t0s\n' "$id" "$name" "$expect" >> "$STEPS"
    return
  fi
  local inited=0
  [ -e /etc/ros/rosdep/sources.list.d/20-default.list ] && inited=1
  {
    echo 'set -ex'
    # A launch line, with any continuation lines after it, becomes a fixed
    # no-op; its text goes to $src.skipped and nothing of it is executed.
    awk -v repo="$REPO" -v inited=$inited -v skipped="$src.skipped" '
      cont { print > skipped; cont = /\\$/ && !/#/; next }
      /^[[:space:]]*ros2[[:space:]]+launch[[:space:]]/ {
        print > skipped; print ": launch line skipped"; cont = /\\$/ && !/#/; next }
      inited && /^[[:space:]]*sudo[[:space:]]+rosdep[[:space:]]+init/ {
        print ": rosdep already initialised"; next }
      { gsub(/<this repo>/, repo); print }' "$src.orig"
  } > "$src"
  # Anything still naming a launch file or a GUI, anywhere on a line, is
  # refused rather than run.
  if grep -qE 'ros2[[:space:]]+(launch|run)|launch\.(py|xml)|gz[[:space:]]+sim|rviz2' "$src"; then
    printf '%s\tREADME block %s\trefused: starts a launch file\texpected %s\tUNEXPECTED\t0s\n' \
      "$id" "$name" "$expect" >> "$STEPS"
    return
  fi
  step "$id" "README block '$name' (HOME=$home)" "$expect" "HOME=$home; cd \$HOME; source $src"
}

# --- ROS 2 Jazzy, ros-base, per the official Ubuntu deb install -------------
step R1 "base tools for the ROS install" pass \
  'apt-get update && apt-get install -y locales curl software-properties-common && locale-gen en_US.UTF-8 && add-apt-repository -y universe'
step R2 "ros2-apt-source" pass \
  'V=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | cut -d\" -f4) && curl -fsSL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && apt-get install -y /tmp/ros2-apt-source.deb && apt-get update'
step R3 "ros-jazzy-ros-base" pass 'apt-get install -y ros-jazzy-ros-base'
export LANG=en_US.UTF-8
printf '#!/bin/sh\nexec "$@"\n' > /usr/local/bin/sudo && chmod +x /usr/local/bin/sudo
echo 'APT::Get::Assume-Yes "true";' > /etc/apt/apt.conf.d/90-assume-yes
# Build parallelism for the 8 GB limit, without editing the README's lines.
mkdir -p /root/.colcon && printf 'build:\n  parallel-workers: 4\n' > /root/.colcon/defaults.yaml
export MAKEFLAGS=-j4 COLCON_HOME=/root/.colcon
dpkg-query -W -f='${Package}\t${Version}\n' | sort > "$OUT/dpkg_ros.tsv"

# git is what fetches the README, so it comes first; the README's own apt
# line installs it too.
apt-get install -y git > /dev/null
git config --global --add safe.directory '*'
git clone -q "$REPO" /tmp/readme_src && git -C /tmp/readme_src checkout -q "$REF" || exit 3
README=/tmp/readme_src/README.md
git -C /tmp/readme_src log --oneline -1 > "$OUT/commit.txt"
apt-get purge -y --auto-remove git > /dev/null

# --- Simulation: the quick start, then the smoke tests ----------------------
run_block S1 sim pass
dpkg-query -W -f='${Package}\t${Version}\n' | sort > "$OUT/dpkg_sim.tsv"
WS=/root/ranger_xarm_ws
step S2 "commit under test" pass "git -C $WS checkout -q $REF && git -C $WS log --oneline -1"
for s in arm base; do
  step "S3-$s" "smoke $s" pass \
    "source /opt/ros/jazzy/setup.bash && cd $WS && SMOKE_DOMAIN_ID=81 LOG_DIR=$OUT/smoke_$s tools/smoke_test/gazebo.sh $s"
done
step S4 "no hardware driver was installed" pass \
  "! grep -E '^ros-jazzy-(ouster-ros|realsense2-camera|librealsense2|sllidar-ros2|ranger-base)\b' $OUT/dpkg_sim.tsv"
# Packages of the hardware path that the simulation install did bring in,
# for the record: robot_localization through ranger_xarm_isaac ->
# ranger_xarm_bringup, moveit_ros_perception through
# ranger_xarm_moveit_config. (kiss_icp is built from source, not a deb.)
step S4b "hardware-path packages the simulation install brought in (record)" pass \
  "grep -E '^ros-jazzy-(robot-localization|moveit-ros-perception)|^libpcl' $OUT/dpkg_sim.tsv | tee $OUT/sim_hardware_path_packages.txt; true"
step S5 "check sim" pass \
  "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && python3 $WS/tools/deps/deps.py check sim"
# The state a hardware user is in before installing drivers: every package
# of this workspace built, no driver present. The simulation build lacks
# only ranger_xarm_sensors, which builds without its drivers (run-time
# dependencies only; its one test is not built here).
step S5b "build ranger_xarm_sensors, drivers absent" pass \
  "source /opt/ros/jazzy/setup.bash && cd $WS && colcon build --packages-up-to ranger_xarm_sensors --cmake-args -DBUILD_TESTING=OFF"
# Exit 1 with a MISSING list (not a crash), and every absent driver named.
step S6 "check hardware, no drivers: exit 1 and names each missing driver" pass \
  "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash
   python3 $WS/tools/deps/deps.py check hardware > $OUT/check_hardware_without_drivers.txt 2>&1; rc=\$?
   cat $OUT/check_hardware_without_drivers.txt; [ \$rc -eq 1 ] || exit 1
   grep -q '^MISSING, ' $OUT/check_hardware_without_drivers.txt || exit 1
   for p in ranger_base ranger_msgs sllidar_ros2 ouster_ros realsense2_camera; do
     grep -q \"package \$p \" $OUT/check_hardware_without_drivers.txt || { echo \"not named: \$p\"; exit 1; }
   done"
# What moveit_ros_perception (MoveIt's point-cloud octomap plugin, needed by
# real_execution.launch.py) adds to a simulation install: itself and the
# packages nothing else needs.
step S7 "what moveit_ros_perception adds" pass \
  "apt-get -s purge --auto-remove ros-jazzy-moveit-ros-perception | grep '^Purg' > $OUT/perception_adds.txt; wc -l < $OUT/perception_adds.txt"

# --- Hardware: (a) a driver workspace, then (b) one workspace ---------------
if [ "$DRIVERS" = 1 ]; then
  run_block H1 hardware pass
  awk '/=== \[H1\]/,/rc=/' "$OUT/walkthrough.log" | grep -A40 'deps.py check hardware' \
    > "$OUT/check_hardware_with_drivers.txt"
  dpkg-query -W -f='${Package}\t${Version}\n' | sort > "$OUT/dpkg_hardware.tsv"
  # (b) in its own HOME, so the driver workspace from (a) is not on its path
  mkdir -p /root/b
  run_block H2 sim pass /root/b
  run_block H3 hardware_single pass /root/b
fi
echo "done" >> "$OUT/walkthrough.log"
