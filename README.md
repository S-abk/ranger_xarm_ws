# ranger_xarm_ws

ROS 2 Jazzy workspace for a **Ranger Mini V3 omnidirectional base + UFACTORY
xArm 6** mobile manipulator, with an Ouster OS0 lidar, an Intel RealSense D435
and an RPLIDAR A1 on a sensor gantry.

It gives you the robot and nothing else: description, MoveIt configuration,
Gazebo simulation, sensor and base bringup. There is no application in here.
Build yours as a package that depends on these.

The **same URDF drives simulation and hardware.** Simulation changes three
xacro arguments, not the model, so a fix made in simulation is a fix on the
robot and there is no second description to drift out of step.

---

New to this robot? **`docs/SITE_SETUP.md`** is the hands-on onboarding guide:
the site-specific values you must supply, host prerequisites, the ordered
bring-up rungs, and what a healthy system actually measures.

## Quick start (simulation, no hardware)

```bash
sudo apt install python3-colcon-common-extensions python3-vcstool
source /opt/ros/jazzy/setup.bash

git clone <this repo> ~/ranger_xarm_ws
cd ~/ranger_xarm_ws
vcs import --recursive src < ranger_xarm.repos   # xarm_ros2 at a pinned commit
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-up-to ranger_xarm_gazebo
source install/setup.bash

ros2 launch ranger_xarm_gazebo sim.launch.py
```

You should get a gz window with the platform on a ground plane, a populated
`/joint_states`, and `joint_state_broadcaster` plus `xarm_xarm6_traj_controller`
active. `--packages-up-to` matters: it builds the simulation path only, and
skips the packages that need physical-sensor drivers.

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

**The gripper needs a patch that is not in this repo.** It lives in a file
`vcs import` pulls into `src/xarm_ros2/`, which is gitignored here, so a fresh
clone does not have it and the finger linkage comes apart in simulation
(the TF tree still looks correct — `robot_state_publisher` derives the mimic
joints kinematically — so check the fingers in gz, not in RViz). See
`docs/TODO.md`; upstream as
[xarm_ros2#180](https://github.com/xArm-Developer/xarm_ros2/pull/180).

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

## Packages

| Package | What it is | Needs hardware drivers |
| --- | --- | --- |
| `ranger_xarm_description` | URDF/xacro and CAD meshes. `base_link` is the CAD reference; `base_footprint` is a separate frame at the lowest wheel-tread point — they are not interchangeable. | no |
| `ranger_xarm_moveit_config` | MoveIt 2. Planning group `xarm6` spans `xarm_link_base -> xarm_link_tcp`; `xarm_gripper` is separate. | no |
| `ranger_xarm_gazebo` | gz simulation: world, spawn, `gz_ros2_control`. | no |
| `ranger_xarm_sensors` | `robot.launch.py`, the physical robot composed in one file. | yes |
| `ranger_xarm_bringup` | CAN driver bringup, `odom -> base_footprint`, cmd_vel deadman. | yes |

## Running the physical robot

The two hardware packages need drivers that are not in `ranger_xarm.repos`,
because which ones you need depends on which sensors you have:

- `ranger_base`, `ranger_msgs` — AgileX Ranger CAN driver
- `ouster_ros` — Ouster OS0
- `realsense2_camera` — Intel RealSense D435
- `sllidar_ros2` — RPLIDAR A1

With those in the workspace:

```bash
colcon build
ros2 launch ranger_xarm_sensors robot.launch.py robot_ip:=<xarm controller ip>
```

Every sensor is an `enable_*` argument, so a subset works without editing
anything:

```bash
ros2 launch ranger_xarm_sensors robot.launch.py robot_ip:=<ip> \
    enable_ouster:=false enable_rplidar:=false enable_base:=false
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

- No unit or integration test suites: validation here is launching nodes and
  driving the robot, simulated or real. `colcon test` will report nothing
  meaningful.
- Custom packages use `ament_cmake` with Python scripts installed via
  `install(PROGRAMS ...)`. **A new script must be added to `CMakeLists.txt` or
  it will not be installed to `lib/<pkg>/`** and the launch file will fail to
  find it.
- `build/`, `install/` and `log/` are colcon output and are gitignored. Never
  edit or commit into them.
