# ranger_xarm_ws

ROS 2 Jazzy workspace for a **Ranger Mini V3 omnidirectional base + UFACTORY
xArm 6** mobile manipulator, with an Ouster OS0 lidar, two Intel RealSense cameras (a D455 on the gantry, a D435 on the wrist)
and an RPLIDAR A1 on a sensor gantry.

## Hardware configuration

`main` targets this hardware:

- **Gantry:** an Intel RealSense **D455** on the 140 mm PETG gantry support.
  Printable mesh: [`gantry_support_140.stl`](src/ranger_xarm_description/meshes/visual/gantry_support_140.stl).
- **Wrist:** an Intel RealSense **D435** on the PETG wrist plate, between the
  xArm flange and the gripper. Printable mesh: [`wrist_d435_plate.stl`](src/ranger_xarm_description/meshes/visual/wrist_d435_plate.stl).

Both meshes are exact and print-ready, in mm, positioned in their link frames,
so re-orient them in the slicer. Each camera needs its own serial at bringup
(see [Running the physical robot](#running-the-physical-robot)). What changed,
and every renamed frame, topic and launch argument, is in
[`docs/MIGRATION.md`](docs/MIGRATION.md).

### Still on the gantry-D435 robot?

The previous configuration, one D435 on the 110 mm gantry support, is the
branch `legacy/gantry-d435`, maintained for fixes only. The tag
`legacy-gantry-d435` is its exact last commit before `main` moved. To switch:

```bash
cd ~/ranger_xarm_ws
git fetch origin
git checkout legacy/gantry-d435
vcs import --recursive src < ranger_xarm.repos
rm -rf build install log
colcon build
```

It gives you the robot and nothing else: description, MoveIt configuration,
Gazebo and NVIDIA Isaac Sim simulation, sensor and base bringup. There is no application in here.
Build yours as a package that depends on these.

That is a rule for the history too. On 2026-10-08 every commit was
rewritten to remove application- and site-specific content (application
names and configuration, network addresses, a sensor's identifying
metadata), so nothing here, past or present, assumes a particular
deployment. If you cloned before then, re-clone, or run
`git fetch origin && git reset --hard origin/<branch>` on each branch;
your old commits do not share history with the new ones.

The **same URDF drives simulation and hardware.** Simulation changes xacro
arguments (the ros2_control plugin, the weld to the world, and optionally
the wheels and simulated sensors), not the model, so a fix made in
simulation is a fix on the robot and there is no second description to
drift out of step.

---

New to this robot? **`docs/SITE_SETUP.md`** is the hands-on onboarding guide:
the site-specific values you must supply, host prerequisites, the ordered
bring-up rungs, and what a healthy system actually measures.

## Quick start (simulation, no hardware)

On Ubuntu 24.04 with ROS 2 Jazzy installed and nothing else:

<!-- fresh_clone: sim -->
```bash
sudo apt install ros-dev-tools   # git, compilers, colcon, rosdep, vcstool
sudo rosdep init        # once per machine; it refuses if already done, which is fine
rosdep update
source /opt/ros/jazzy/setup.bash

git clone <this repo> ~/ranger_xarm_ws
cd ~/ranger_xarm_ws
vcs import --recursive src < ranger_xarm.repos   # xarm_ros2 and two others, pinned
PKGS="ranger_xarm_gazebo ranger_xarm_isaac"
rosdep install -y --ignore-src --from-paths $(colcon list --packages-up-to $PKGS --paths-only) \
    --skip-keys "ranger_base ranger_msgs"

colcon build --packages-up-to $PKGS
source install/setup.bash

ros2 launch ranger_xarm_gazebo sim.launch.py
```

You should get a gz window with the platform on a ground plane, a populated
`/joint_states`, and `joint_state_broadcaster` plus `xarm_xarm6_traj_controller`
active. `tools/smoke_test/gazebo.sh` checks all of that headless.

This builds the simulation path only and installs no hardware driver.
`ranger_xarm_isaac` is in it because MoveIt against gz (and the smoke test)
uses its `moveit.launch.py`; for gz alone, `PKGS=ranger_xarm_gazebo` is
enough. `ranger_xarm_isaac` depends on `ranger_xarm_bringup` for a Fast DDS
profile file, and the bring-up package names the base driver, which
simulation never starts: hence the `--skip-keys`. Three packages of the
hardware path do come in: `robot_localization` from apt and `kiss_icp`
built from source, because the bring-up package names them for its EKF and
lidar odometry, and `moveit_ros_perception` from apt, because the MoveIt
configuration names the point-cloud octomap plugin the real robot uses. The real robot's
drivers are a separate step: [Running the physical robot](#running-the-physical-robot).
A plain `colcon build` builds everything, including the hardware packages,
which need their drivers only at run time.

Useful arguments:

| Argument | Default | |
| --- | --- | --- |
| `world` | `empty_ground.sdf` | absolute path to your own `.sdf` |
| `headless` | `false` | no gz GUI, for CI or a headless box |
| `add_gripper` | `true` | include the xArm gripper |
| `start_rviz` | `false` | |
| `drive_base` | `false` | add the 4WIS wheels and drive the base from `/cmd_vel`. Off by default: without it the base is welded to the world, which is what the arm-only workflows expect |

Driving the base:

```bash
ros2 launch ranger_xarm_gazebo sim.launch.py drive_base:=true
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

One `/cmd_vel` twist covers every mode the platform has: `linear.x` drives,
`linear.y` crabs, `angular.z` spins in place, and any mix arcs. There is no
mode switch, because four-wheel independent steering does not need one.

**The gripper relies on a fork of xarm_ros2.** Its finger linkage holds
together in gz only with one commit that declares the follower joints to
ros2_control as mimic joints; `ranger_xarm.repos` pins a fork that carries
it until upstream
[xarm_ros2#180](https://github.com/xArm-Developer/xarm_ros2/pull/180)
merges. If you point `ranger_xarm.repos` elsewhere, check the fingers in gz,
not in RViz: the TF tree looks correct either way, because
`robot_state_publisher` derives the mimic joints kinematically. See
`docs/TODO.md`.

Look at the model without a simulator at all:

```bash
ros2 launch ranger_xarm_description display.launch.py
```

Plan without any physics, using MoveIt's mock controllers — faster than gz when
you only care about IK, collision and planning:

```bash
ros2 launch ranger_xarm_moveit_config planning_demo.launch.py   # plan only
ros2 launch ranger_xarm_moveit_config fake_execution.launch.py  # mock execution
```

## Licensing

Each package carries its own licence in its `package.xml`:

| Package | Licence |
| --- | --- |
| `ranger_xarm_description` | BSD-3-Clause (except the meshes listed below) |
| `ranger_xarm_moveit_config` | BSD-3-Clause |
| `ranger_xarm_bringup` | BSD-3-Clause |
| `ranger_xarm_sensors` | BSD-3-Clause |
| `ranger_xarm_gazebo` | Apache-2.0 |
| `ranger_xarm_isaac` | Apache-2.0 |

Meshes with their own licence, in `src/ranger_xarm_description/meshes/visual/`:

- `gantry_support_140.stl` and `wrist_d435_plate.stl`, the printable camera
  mounts: CC-BY 4.0 (full text in
  [`src/ranger_xarm_description/LICENSE-CC-BY-4.0`](src/ranger_xarm_description/LICENSE-CC-BY-4.0)). Attribution:
  Shuaib Olanrewaju. Give this credit when you share or adapt the meshes.
- `realsense_d455.stl`: Apache-2.0, copied unchanged from Intel RealSense's
  `realsense2_description` (`meshes/d455.stl`).

Upstream sources pulled by `vcs import` (`ranger_xarm.repos`) keep their own
licences.

## Packages

| Package | What it is | Needs hardware drivers |
| --- | --- | --- |
| `ranger_xarm_description` | URDF/xacro and CAD meshes. `base_link` is the CAD reference; `base_footprint` is a separate frame at the lowest wheel-tread point — they are not interchangeable. | no |
| `ranger_xarm_moveit_config` | MoveIt 2. Planning group `xarm6` spans `xarm_link_base -> xarm_link_tcp`; `xarm_gripper` is separate. | no |
| `ranger_xarm_gazebo` | gz simulation: world, spawn, `gz_ros2_control`. | no |
| `ranger_xarm_isaac` | Isaac Sim: xacro-to-USD conversion, the simulator bridge, sensors, ros2_control and MoveIt against it. | no (needs Isaac Sim) |
| `ranger_xarm_sensors` | `robot.launch.py`, the physical robot composed in one file. | yes |
| `ranger_xarm_bringup` | CAN driver bringup, `odom -> base_footprint`, cmd_vel deadman. | yes |

## Isaac Sim

`ranger_xarm_isaac` runs the same robot in NVIDIA Isaac Sim (verified on
5.1.0). The USD is generated from the xacro, never
committed; the arm, gripper and 4WIS base are driven through ros2_control's
topic-based hardware interface (`topic_based_hardware_interfaces`, pulled by
`ranger_xarm.repos`), and the Ouster OS0 (with its IMU), RPLIDAR A1M8, gantry
D455 and wrist D435 publish on the same topics and frames as the real drivers. Details and
pitfalls: `src/ranger_xarm_isaac/README.md`.

Isaac Sim's scripts run under Isaac's own Python, not the system one. Point
`ISAACSIM_PYTHON_EXE` at it once (the variable NVIDIA's own install docs use):
for a standalone (workstation) install that is `python.sh` in the folder you
unpacked it to; for a pip install (`pip install isaacsim`) it is the
virtual environment's `python`.

```bash
export ISAACSIM_PATH=/path/to/isaac-sim              # where you installed it
export ISAACSIM_PYTHON_EXE=$ISAACSIM_PATH/python.sh  # pip install: the venv's python

source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ranger_xarm_isaac
source install/setup.bash
P=$(ros2 pkg prefix ranger_xarm_isaac)
USD=$P/share/ranger_xarm_isaac/usd/ranger_xarm_wheeled.usd

# The Ouster cloud needs the large shared-memory Fast DDS profile. Export it
# in EVERY shell below (Isaac's included) or most scans are dropped.
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix ranger_xarm_bringup)/share/ranger_xarm_bringup/config/fastdds_large_shm.xml

# 1. Generate the USD, with the wheeled base (Isaac's own interpreter)
$ISAACSIM_PYTHON_EXE $P/lib/ranger_xarm_isaac/urdf_to_usd.py --force --output $USD \
    use_wheels:=true fix_base_to_world:=false wheels_command_interface:=velocity

# 2. Start Isaac: robot on a flat ground plane, sensors, clock
$ISAACSIM_PYTHON_EXE $P/lib/ranger_xarm_isaac/isaac_bringup.py --usd $USD
#    optionally in an environment: --scene hospital --spawn 2.5,0,0

# 3. ros2_control, the 4WIS controller, wheel odometry
ros2 launch ranger_xarm_isaac control.launch.py drive_base:=true

# 4. Drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 5. MoveIt for the arm (optional)
ros2 launch ranger_xarm_isaac moveit.launch.py start_rviz:=true
```

`/cmd_vel` is executed the way the real Ranger's AgileX driver executes it:
`linear.y` crabs (all wheels parallel), a turn tighter than the minimum radius
spins in place, anything else is dual Ackermann, and the base stops 0.5 s
after the last command. `/odom` is wheel odometry; `/ground_truth/odom` is
Isaac's chassis pose and `/ground_truth/contacts` what the robot touches. For
the arm alone, run `urdf_to_usd.py` with no arguments and start
`isaac_bringup.py` and `control.launch.py` without `--usd` / `drive_base`.

**The suspension is on by default, and its numbers are estimates.** Each
wheel corner rides on a sprung, damped prismatic joint (`ranger_wheels.xacro`),
but AgileX publishes no suspension travel, spring rate or damping for the
Ranger Mini 3.0, so the values are estimated, not measured: 15 mm of droop and
25 mm of bump about the static ride height, a 20 kN/m spring with 12.5 mm of
static sag, and 570 N s/m of damping. Behaviour that depends on how the
wheels load and unload — side slopes, crests, one wheel lifting — is only as
good as those numbers; replace them with measurements from your robot before
trusting it there. To run with a rigid chassis instead, pass
`use_suspension:=false` to both `urdf_to_usd.py` and `control.launch.py`
(the USD and the controllers must agree on which joints exist).

## Running the physical robot

### Hardware dependencies

The two hardware packages need drivers that simulation does not, so they
are a separate step from the quick start:

| Package | For | Comes from |
| --- | --- | --- |
| `ranger_base`, `ranger_msgs` | the Ranger CAN driver | source, `ranger_xarm_hardware.repos` (a fork: the vendor's branch does not support the Mini V3) |
| `ugv_sdk` | the vendor CAN SDK `ranger_base` builds on | source, `ranger_xarm_hardware.repos` |
| `sllidar_ros2` | RPLIDAR A1 | source, `ranger_xarm_hardware.repos` |
| `ouster_ros` | Ouster OS0 | apt through rosdep; see the version pin in [`docs/SITE_SETUP.md`](docs/SITE_SETUP.md) |
| `realsense2_camera` | RealSense D455 and D435 | apt through rosdep |
| `robot_localization` | the EKF (`ekf_odom_imu.launch.py`) | apt through rosdep |
| `moveit_ros_perception` | MoveIt's octomap from the Ouster (`real_execution.launch.py`) | apt through rosdep |
| `kiss_icp` | lidar odometry (`lidar_odometry:=true`) | source, `ranger_xarm.repos` (already fetched) |

The pins in `ranger_xarm_hardware.repos` are provisional: they are the
commits this workspace was built against, not yet confirmed against a
robot's own driver workspaces.

Install them **one** of two ways, never both on the same machine; with
both, two copies of a driver are on the path and whichever comes first
wins.

**a) A separate driver workspace, sourced before this one** (what the
robot's computer does):

<!-- fresh_clone: hardware -->
```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ranger_drivers_ws/src && cd ~/ranger_drivers_ws
vcs import --recursive src < ~/ranger_xarm_ws/ranger_xarm_hardware.repos
rosdep install -y --ignore-src --from-paths src --skip-keys "catkin ament_python"
colcon build
source install/setup.bash          # in every shell, before this workspace's

cd ~/ranger_xarm_ws
rosdep install -y --ignore-src --from-paths src
colcon build
source install/setup.bash
python3 tools/deps/deps.py check hardware
```

**b) Everything in this workspace**, on a machine with no driver
workspace:

<!-- fresh_clone: hardware_single -->
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ranger_xarm_ws
vcs import --recursive src < ranger_xarm_hardware.repos
rosdep install -y --ignore-src --from-paths src --skip-keys "catkin ament_python"
colcon build
source install/setup.bash
python3 tools/deps/deps.py check hardware
```

The two skipped keys are declared by the drivers' own manifests and need
nothing installed (`ugv_sdk` names its ROS 1 build tool; one unused package
of the base driver names its build type). The last line,
[`tools/deps/deps.py check hardware`](tools/deps/README.md), reads the
installed launch files and manifests and lists every package, program,
file or plugin the hardware launch files need that is missing, with what
needs it, and which workspace each driver was found in. Run it before the
first launch on a new machine and after changing the workspace chain. It
starts nothing.

### Launching

```bash
ros2 launch ranger_xarm_sensors robot.launch.py robot_ip:=<xarm controller ip> \
    ouster_sensor_hostname:=<ouster host> \
    gantry_d455_serial_no:=<digits> wrist_d435_serial_no:=<digits>
```

Each RealSense needs its own serial (digits, from `rs-enumerate-devices`) and has
no default, so the launch can never bind whichever camera enumerates first.
Details: `src/ranger_xarm_sensors/README.md`.

Every sensor is an `enable_*` argument, so a subset works without editing
anything. A camera that is off needs no serial:

```bash
ros2 launch ranger_xarm_sensors robot.launch.py robot_ip:=<ip> \
    enable_ouster:=false enable_rplidar:=false enable_base:=false \
    enable_gantry_d455:=false enable_wrist_d435:=false
```

Staged bringup, which is how to debug a bad start rather than an alternative
set of options to pick from:

`planning_demo` (plan only) → `fake_execution` (mock control) →
`real_hardware_check` (arm connected, nothing commanded) → `real_execution`
(arm will move) → `robot.launch.py` (everything).

### Two things that will bite you

**Exactly one node may own `odom -> base_footprint`.** The Ranger driver does by
default; `publish_odom_tf:=false` hands it to the gyro-fused EKF in
`ranger_xarm_bringup/ekf_odom_imu.launch.py`. With two owners TF silently takes
whichever message arrived last, and the symptom looks like a bad sensor rather
than a configuration error.

The same EKF can also fuse KISS-ICP lidar odometry (`lidar_odometry:=true`,
off by default; KISS-ICP is built from source via `ranger_xarm.repos`). It
needs the full-rate cloud, which a 2 - 3 MB Ouster message only gets between
processes with a larger Fast DDS shared-memory segment: export
`ranger_xarm_bringup/config/fastdds_large_shm.xml` as
`FASTRTPS_DEFAULT_PROFILES_FILE` in the driver's shell and the EKF's. Without
it, loopback UDP silently drops scans. See the launch file's docstring.

**`/ouster/points` needs enlarged UDP receive buffers** or the kernel drops
about half the frames — it reads as a ~5 Hz sensor instead of 10 Hz, with
`RcvbufErrors` climbing in `/proc/net/snmp`:

```bash
sudo sysctl -w net.core.rmem_max=33554432 net.core.rmem_default=33554432
```

Persist it in `/etc/sysctl.d/` once you have confirmed it helps.

## Safety invariants

These are not style preferences. Each one is here because violating it has a
specific, physical failure mode, and they are easy to lose when this stack is
extended.

1. **Physical motion is opt-in and layered.** A subsystem being *present* is
   independent of it being *allowed to move hardware*. Every new motion
   capability defaults its execution-enable parameter to `false`.
2. **A MoveIt trajectory is executed only when the planning result is exactly
   `SUCCESS`.** Not "no error", not a partial plan.
3. **Absence of sensor returns is not free space.** An occluded region reads
   identically to an empty one. Anything that clears volume for planning must
   justify it from a positive observation.
4. **Never enlarge a trusted manipulation volume to make a plan succeed.** That
   volume exists to suppress nuisance points; widening it to clear an obstacle
   deletes the obstacle instead of avoiding it.
5. **One publisher per command topic.** Two nodes publishing the same goal is a
   race whose loser is invisible.
6. **Re-plan after a policy change.** Any change to the collision or keepout
   policy invalidates plans made under the old one; recovery must be planned
   afresh, not resumed.

If you add autonomous motion, put it behind an explicit gate of the shape
*valid task → valid fresh trajectory → explicit arm → one-shot execute*, with
freshness and start-state agreement checked at the moment of execution rather
than at planning time.

## Conventions

- Almost no unit tests: validation here is launching nodes and driving the
  robot, simulated or real. The one `colcon test` target is
  `ranger_xarm_sensors`'s test of the two cameras' launch descriptions,
  which walks them without starting anything. `tools/smoke_test/` automates
  the simulated part: it brings the
  robot up, drives it, moves the arm and checks the sensors, in about two
  minutes. `tools/moveit_test/` runs a fixed set of MoveIt goals, including
  some close to the gantry, and checks every planned path against the true
  collision model.
- Custom packages use `ament_cmake` with Python scripts installed via
  `install(PROGRAMS ...)`. **A new script must be added to `CMakeLists.txt` or
  it will not be installed to `lib/<pkg>/`** and the launch file will fail to
  find it. Two scripts are deliberately not installed: they are tools run
  from the source tree, and no launch file uses them.
  `src/ranger_xarm_description/scripts/split_wheel_meshes.py` regenerates
  the wheel meshes, and `src/ranger_xarm_gazebo/scripts/make_outdoor_terrain.py`
  the outdoor terrain model (its docstring gives the Blender command line;
  `split_wheel_meshes.py --help` lists its options).
- `tools/fresh_clone/run.sh` proves that a commit goes from `git clone` to a
  built, smoke-tested simulation on a bare Ubuntu 24.04 with ROS 2 Jazzy, by
  running this README's own code blocks in a container (the blocks marked
  `<!-- fresh_clone: ... -->`; keep the markers when editing them). Run it,
  with `--drivers`, before merging any change to those blocks or to either
  `.repos` file.
  `tools/deps/deps.py report` lists where every external package comes from.
- `build/`, `install/` and `log/` are colcon output and are gitignored. Never
  edit or commit into them.
