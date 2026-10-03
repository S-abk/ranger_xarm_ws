# ranger_xarm_isaac

NVIDIA Isaac Sim bring-up for the platform: the same xacro the real robot and
the gz simulation use, converted to USD, with the arm, the gripper, the 4WIS
base and the sensors bridged to ROS 2 the way the real drivers expose them.
Nothing in this package is needed to run the physical robot.

Verified against Isaac Sim 5.1.0. `$ISAACSIM_PYTHON_EXE` below is Isaac Sim's own
interpreter: `python.sh` in a standalone install, the venv's `python` for a pip
install (see the top-level README).

```bash
# Source the workspace FIRST. Isaac's interpreter does not do it for you,
# and the scripts need ament to find the description package.
source /opt/ros/jazzy/setup.bash && source install/setup.bash
P=$(ros2 pkg prefix ranger_xarm_isaac)

# 1. convert the xacro to USD (Isaac's interpreter, not the system one)
$ISAACSIM_PYTHON_EXE $P/lib/ranger_xarm_isaac/urdf_to_usd.py

# 2. Isaac itself (owns physics and the clock)
$ISAACSIM_PYTHON_EXE $P/lib/ranger_xarm_isaac/isaac_bringup.py

# 3. ros2_control against it
ros2 launch ranger_xarm_isaac control.launch.py

# 4. MoveIt on top (optional)
ros2 launch ranger_xarm_isaac moveit.launch.py start_rviz:=true
```

That is the arm on a base welded to the world. For the driving base see
[The 4WIS base](#the-4wis-base). `description.launch.py` publishes
`robot_description` and TF without any control stack, for looking at the
model.

The conversion produces one articulation root and 12 movable joints on the
arm-only model (arm 1-6, `drive_joint`, and the 5 gripper followers), at
roughly 34 MB.

## Streaming (a remote machine)

To run the simulation on a more capable machine and watch it from this one
(e.g. over a VPN), run everything there, ROS included, and stream only
Isaac's viewport:

```bash
# on the remote machine
export ISAACSIM_STREAM_ADDRESS=<REMOTE_IP>   # its address on the VPN
$ISAACSIM_PYTHON_EXE $P/lib/ranger_xarm_isaac/isaac_bringup.py --stream   # plus the usual arguments
```

`--stream` runs Isaac headless with its UI kept and the viewport served over
WebRTC (`omni.services.livestream.nvcf` in Isaac Sim 5.1; the extension name
is version specific). Connect with NVIDIA's *Isaac Sim WebRTC Streaming
Client* (the 1.1.x AppImage for 5.1) to `ISAACSIM_STREAM_ADDRESS`. It needs
TCP 49100 (signalling) and UDP 47998 (video) to reach the remote machine.
There is no authentication, so keep it on the VPN.

Keep the ROS graph on the remote machine (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
there): the raw sensor streams (the Ouster cloud, the RealSense colour and depth images)
come to roughly 95 MB/s, far more than a VPN carries well. RViz is better
run on the remote machine too, behind a remote desktop, than fed over the
tunnel.

## Two traps that cost real time

**Isaac's Python is not your Python.** Isaac 5.1 ships CPython 3.11 and
`python.sh` exports `PYTHONHOME`/`PYTHONPATH` pointing into it. Anything the
script shells out to, `xacro` included, has a system shebang, so it starts
3.12 and then loads Isaac's 3.11 standard library:

```
AssertionError: SRE module mismatch
```

which tells you nothing. `urdf_to_usd.py` strips those variables for the
subprocess; see `ros_clean_env()`.

**Isaac cannot resolve `package://`, and does not say so.** The importer is
not ROS-aware. Given `package://ranger_xarm_description/meshes/...` it
imports the robot with every mesh silently missing: no error, a USD that
loads fine and contains no geometry. The tell is size. Converted with the
meshes dropped it was 36 kB against 19 MB of source STL; resolved, it is
34 MB. `resolve_package_uris()` rewrites the URIs in the temporary expanded
copy only, so the description keeps the `package://` form that ROS and gz
both want.

## The rule this package exists to not break

`ranger_xarm_description`'s xacro is the single source of truth, the same as
it is for gz and for hardware. **The USD is a build artifact.** It is written
to `share/ranger_xarm_isaac/usd/`, it is gitignored, and it is regenerated
from the xacro. It is never edited and never committed.

NVIDIA's ROS 2 reference architecture makes the same argument: one model,
defined in URDF, as the common origin for both ROS 2 and Isaac. A committed
USD is precisely the "second description to drift out of step" the workspace
README warns about, and it is the worst kind, because nothing fails when it
goes stale. It just quietly stops matching the robot.

## Conversion is lazy

`urdf_to_usd.py` skips the conversion when the USD is newer than every xacro
and mesh under `ranger_xarm_description`. Re-running it is cheap.

It does **not** detect changed xacro *arguments*, since those leave no mtime.
After changing e.g. `add_gripper`, pass `--force`.

```bash
$ISAACSIM_PYTHON_EXE .../urdf_to_usd.py --force add_gripper:=false
$ISAACSIM_PYTHON_EXE .../urdf_to_usd.py --fix-base      # weld to world
```

Other options: `--wheel-mu` (tyre friction, default 1.2),
`--wheel-contact-offset` / `--wheel-rest-offset` (PhysX contact skin on the
tyres), `--velocity-drive-joints` (which joints get velocity drives). Run
with `--help` for details.

The USD is written into the package's **install** share directory, so
`rm -rf build/ranger_xarm_isaac install/ranger_xarm_isaac` takes it with
it, and a plain `colcon build` does not put it back. Re-run
`urdf_to_usd.py`. A sim already running keeps its loaded copy, so the
first sign is usually a conversion or a launch finding no file while the
robot on screen is still fine.

Build the package without `--symlink-install` unless the whole workspace
uses it; mixing the two leaves the scripts non-executable and launch
fails with `executable '...' not found on the libexec directory`.

`merge_fixed_joints` is off deliberately. Most of this platform is fixed
joints (sensors, gantry, arm pedestal) and merging them collapses exactly the
frames the rest of the stack addresses by name.

What the converter adds on top of the importer, because the importer cannot
read it from the URDF:

- **Drives on the gripper's mimic joints.** The importer gives every joint a
  drive *except* those with a `<mimic>` tag, and USD has no mimic coupling,
  so the followers would hang wherever contact left them. Each gets a copy of
  the drive of the joint it mimics.
- **Velocity drives on the wheels** (stiffness 0, damping carried over from
  the stiffness the importer chose). A stiff position drive would hold each
  wheel at a fixed angle and fight the speed command.
- **Suspension springs.** Each corner's prismatic `*_suspension_joint` gets
  a force drive with the xacro's spring stiffness, its damping and the
  spring's reference as target, i.e. a linear spring-damper. The importer
  alone would make it a stiff position drive towards 0: a rigid suspension.
- **Tyre friction.** `ranger_wheels.xacro` states the grip only in gz
  extension tags, so the imported tyres would fall back to PhysX's default
  0.5. A physics material with `--wheel-mu` is bound to them instead.
- **A `defaultPrim`.** Without it, referencing the USD resolves to nothing
  and Isaac reports the far less obvious "Prim ... is not an articulation".

## How the halves meet

`isaac_bringup.py` is the simulator half; `control.launch.py` is the ROS
half. Nothing above the hardware interface differs from the gz path: the
same xacro, the same `joint_trajectory_controller`s, the same MoveIt
config. Only the ros2_control plugin underneath changes, which is the seam
`xarm_ros2_control_plugin` already exists to move: under gz it is
`gz_ros2_control/GazeboSimSystem`, on hardware
`uf_robot_hardware/UFRobotSystemHardware`, and here
`joint_state_topic_hardware_interface/JointStateTopicSystem`
(from `topic_based_hardware_interfaces`, pulled by `ranger_xarm.repos`).

```
Isaac  --isaac_joint_states-->  joint_state_effort_filter  --robot_joint_states-->    ros2_control
Isaac  <--isaac_joint_commands--  joint_command_merger  <--robot_joint_commands--  ros2_control
```

The articulation is ticked by an `OnImpulseEvent` fired once per physics
step, so joint states and commands stay in lockstep with physics rather
than with the renderer. Isaac also publishes `/clock`, and everything on the
ROS side runs on sim time.

- `joint_state_effort_filter.py` drops the effort field Isaac always fills:
  the arm description declares no effort state interface, and the hardware
  interface throws rather than skipping it.
- `joint_command_merger.py` merges the per-component command messages into
  one. See below.

### Three things the gripper needed, none of them obvious

Commanding `drive_joint` to 0.849 originally left Isaac reporting 0.0085
with the followers scattered and asymmetric. Three separate faults, each
hiding the next.

**The hardware components share one command topic.** The arm and the
gripper are separate `ros2_control` blocks and both fall back to the
interface's default topic (the vendored xarm xacros accept no `<param>` in
their `<hardware>` block), so the topic carries two different messages in
alternation and Isaac only ever sees half the robot:

```
x101  [drive_joint, left_finger, left_inner_knuckle, ...]
x100  [joint1 .. joint6]
```

`joint_command_merger.py` keeps the latest command per joint name and
republishes the union.

**The messages are ragged.** `name` gets one entry per joint, but
`position`/`velocity`/`effort` are packed *independently*, each carrying a
value only for the joints that declare that command interface
(`JointStateTopicSystem::write`). They are not parallel with the names, so
indexing them by a name's index is wrong whenever a component is not
uniform. The merger reads each joint's command interfaces from the
`<ros2_control>` blocks in `/robot_description` and unpacks the arrays the
way `write()` packed them. It derives the gripper followers from the URDF's
own `<mimic>` tags, also read off `/robot_description` rather than
hardcoded.

On the base this stops being cosmetic. Steering takes `position` and the
wheels take `velocity`, interleaved per corner:

```
name     = [FL_steer, FL_wheel, FR_steer, FR_wheel, RL_..., RR_...]   (8)
position = [FL_steer, FR_steer, RL_steer, RR_steer]                   (4)
velocity = [FL_wheel, FR_wheel, RL_wheel, RR_wheel]                   (4)
```

Indexed positionally, the four wheel *speeds* land on the first four
*names*: the steer joints are told to rotate at the wheel rate and the rear
joints are never commanded. From outside that looks like a robot that
creeps, yaws at random and drives on its front wheels only.

**The follower joints had no drive.** This was the real one; see the mimic
drives above.

**Check the fingers in Isaac, not in TF.** `robot_state_publisher` derives
mimic joints kinematically from the URDF, so `/tf` and `/joint_states` show
a correct symmetric gripper regardless of what physics is doing.
`/isaac_joint_states` is Isaac's own articulation state and is the only
honest source.

## MoveIt

`moveit.launch.py` runs move_group with the planning configuration reused
unchanged from `ranger_xarm_moveit_config`. It starts move_group ONLY:
`robot_state_publisher` and the controllers already exist, and second
copies would put two publishers on `/robot_description` and two controller
managers on the same joints.

Only the controller map is local (`config/moveit_controllers_isaac.yaml`).
The existing maps name the unprefixed `xarm6_traj_controller` that
`fake_execution` spawns, and on hardware a `xarm_gripper` GripperCommand
proxy forwarding to UFACTORY's native server. Neither exists here:
`control.launch.py` reuses xarm's own controller yaml rewriter, which
applies the `xarm_` prefix, and the gripper is a plain trajectory
controller because there is no UFACTORY driver to proxy to.

Plan and execute to a joint goal, read back from Isaac's articulation:

```
target   j1=-0.600 j2=-0.300 j3=-0.700 j4=+0.200 j5=+0.800 j6=+0.100
Isaac    j1=-0.599 j2=-0.292 j3=-0.707 j4=+0.207 j5=+0.803 j6=+0.100
```

Collision checking is live: a goal folded into the pedestal is refused
before planning starts, naming the pair.

## The 4WIS base

Build a wheeled USD and launch with `drive_base:=true`:

```bash
P=$(ros2 pkg prefix ranger_xarm_isaac)
USD=$P/share/ranger_xarm_isaac/usd/ranger_xarm_wheeled.usd
$ISAACSIM_PYTHON_EXE $P/lib/ranger_xarm_isaac/urdf_to_usd.py --force --output $USD \
    use_wheels:=true fix_base_to_world:=false wheels_command_interface:=velocity
$ISAACSIM_PYTHON_EXE $P/lib/ranger_xarm_isaac/isaac_bringup.py --usd $USD
ros2 launch ranger_xarm_isaac control.launch.py drive_base:=true
```

`/cmd_vel` -> `ranger_4wis_controller.py` -> `ranger_steer_controller` /
`ranger_wheel_controller` -> ros2_control -> Isaac. The controller is the
one `ranger_xarm_gazebo` uses, run with `command_mode: velocity`: Isaac's
wheel joints are velocity drives, i.e. torque-limited speed sources, so it
sends wheel speeds (with its acceleration limit, integral speed correction
and parking brake) rather than closing a torque loop itself.

It executes a twist the way the real Ranger's AgileX driver does, in one
of its steering modes: `linear.y` non-zero crabs with all four wheels
parallel (`angular.z` ignored); a turn tighter than the minimum radius
spins in place (`linear.x` ignored); anything else is dual Ackermann. With
no command, or none for `cmd_timeout` (0.5 s), the wheels stop and the
knuckles return to straight ahead. Drive it from a keyboard with

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # ros-jazzy-teleop-twist-keyboard
```

(the shifted keys send `linear.y`, i.e. crab) or by publishing `/cmd_vel`
directly.

Odometry:

- `/odom` and `odom -> base_footprint` come from `wheel_odometry.py`, dead
  reckoning from the wheel encoders, which is what the real platform can
  know. `odom_tf:=false` hands the TF edge to something else; two
  publishers on one edge is not reported as an error, TF just returns
  whichever arrived last.
- `/ground_truth/odom` (frames `ground_truth_odom` -> `base_link_ground_truth`)
  is `IsaacComputeOdometry`, the chassis pose read out of the physics
  engine. It never slips or drifts; use it to score, never to drive.

Each corner rides on the xacro's sprung suspension joint, reported in the
joint states like any other. Its travel, spring and damping are estimates
(see `ranger_wheels.xacro`).

Steering reaches its target in ~1.25 s of sim time and the wheel speeds
are gated on knuckle convergence, so a mode change has a transient; a test
that measures from t=0 will bake it in.

### Measure in sim time, not wall clock

Isaac advances sim time at its own rate, which is not real time. A test
that divides by `time.time()` reports the base over- or undershooting its
commanded velocity by that ratio, consistently, across speeds and in both
translation and rotation, which reads convincingly like a wheel-radius
error. It is not one. Distance per wheel revolution, in which the time base
cancels, gives an effective rolling radius matching the collision sphere to
five digits. Take every interval from the message header stamps.

### Tuning flags

`isaac_bringup.py` takes a few flags about the robot's physics:

| Flag | |
| --- | --- |
| `--physics-dt S` | physics step; Isaac defaults to 1/60 s, the gz worlds run 1 ms. Rendering tracks it |
| `--wheel-mu MU` | re-bind tyre friction at load time |
| `--wheel-max-torque NM` | wheel drive torque limit (default 22 N m, the Ranger Mini datasheet figure; 0 keeps the USD's) |
| `--steer-damping D` | steering drive damping (default 6.6, about critical; the converter's 0.05 rings) |
| `--wheel-damping D`, `--wheel-drive-type` | wheel drive damping and type |
| `--print-drives` | print the drive gains and limits PhysX actually runs with |
| `--no-sensors` | skip the lidars, IMU and camera (each costs a render product per frame) |
| `--no-contacts` | skip contact reporting |
| `--headless` | no GUI |

## Scenes

The robot loads onto a flat ground plane. `--scene` references an
environment on top of it: `hospital`, `office`, `warehouse`,
`warehouse_small`, `simple_room` (NVIDIA's sample environments, referenced
from the Isaac assets server rather than copied here, since they are not
redistributable) or any USD path or URL. `--spawn x,y,yaw_deg` places the
robot; sample scenes do not keep the origin clear (in the hospital it is on
the west wall, which the robot's rear then sits in; (2.5, 0) is open floor).

```bash
$ISAACSIM_PYTHON_EXE $P/lib/ranger_xarm_isaac/isaac_bringup.py --usd $USD \
    --scene hospital --spawn 2.5,0,0
```

On load `isaac_scenes.py` reports the scene's units, extent and how many of
its geometry prims collide. A prop that renders without a collider is seen
by the lidars and driven through by the robot. The hospital: metres, Z up,
2036 of 2059 geometry prims collide, ~150 s to load.

The RTX lidar can see through thin walls: at 0.1 m it did so in bursts of
frames, a few hundred points each well beyond the wall. Walls of 0.2 m did
not. Keep that in mind for your own environments.

## Collision ground truth

`isaac_bringup.py` reports what the robot actually touches (`--no-contacts`
to turn it off): every robot rigid body gets PhysX's contact-report API,
and after each physics step any contact between a robot body and something
that is neither the robot nor the ground plane is sent on: walls,
furniture, and wheels against any of them. `isaac_contact_relay.py`
(started by `control.launch.py`) publishes it on `/ground_truth/contacts`,
one `std_msgs/String` of JSON per physics step while touching:

    {"t": 70.9, "contacts": [{"link": "ranger_cad_link", "other": "/Environment/...",
      "force": 812.4, "depth": 0.0021, "p": [0.03, 4.0, 0.31]}]}

and an empty list when it ends; each new contact is also logged as a
warning. The first message of a contact can carry no point and 0 N (PhysX
reports the pair a step before its contact data).

It goes through UDP (`--contacts-port`, 47811) because Isaac runs Python
3.11 and the system's Jazzy rclpy is built for 3.12. The bridge's bundled
rclpy aborts on the system's message libraries, and replacing them for the
whole process would swap the bridge's Fast DDS too.

## Sensors

The description carries the sensor links, meshes and CAD-derived mount
poses, but no `<sensor>` tags, so without this nothing produces a reading.
`isaac_sensors.py` fills that in. On by default; `--no-sensors` skips them.

Topics and frames match what the REAL drivers publish, because
`ranger_xarm_sensors` already defines that contract:

```
Ouster OS0     /ouster/points                       os_lidar
               /ouster/imu                          os_imu
RPLIDAR A1M8   /scan                                laser_frame
RealSense      /camera/d435/color/image_raw         d435_color_optical_frame
               /camera/d435/depth/image_rect_raw    d435_depth_optical_frame
               /camera/d435/color/camera_info
```

**Export the large-SHM Fast DDS profile.** The 128 x 1024 Ouster cloud is
2 - 3 MB, more than Fast DDS's default shared-memory segment, so it falls
back to loopback UDP and most scans are dropped (1.7 of 10 Hz arrived).
Export `ranger_xarm_bringup/config/fastdds_large_shm.xml` in Isaac's shell
and in every consumer's, before starting them:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix ranger_xarm_bringup)/share/ranger_xarm_bringup/config/fastdds_large_shm.xml
```

The sensor-internal frames are deliberately **not** in the URDF. On
hardware they belong to the drivers, which publish them from the sensor's
own metadata and calibration, and that split is why `d435_mount_link`
exists in the description and nothing below it does. In Isaac,
`control.launch.py` publishes them statically the way the drivers do:
`os_sensor -> os_lidar` with the Ouster's OS-series offset (36.18 mm up,
yawed 180 deg), and `d435_link` -> the two optical frames. Isaac's own
transform tree names frames after prims, so it cannot supply them.

The IMU reads `(0, 0, +9.810)` at rest. `readGravity` is on because a
real accelerometer at rest reports gravity; with it off the published
acceleration is zero and any attitude filter downstream reads free fall.

### Two traps in Isaac's RTX lidar

**A config name that does not exist does not fail.** Isaac 5.x picks RTX
lidars out of a fixed table of USD assets with variant sets
(`SUPPORTED_LIDAR_CONFIGS`). A bare JSON in `profileBaseFolder` is only
consulted for the deprecated camera-prim path. Naming something not in
the table silently falls back to a generic rotary lidar, and the first
symptom is unrelated: a 32-beam elevation fan where a single-plane
scanner should be, and `IsaacComputeRTXLidarFlatScan` refusing to run
because the prim "is not a 2D Lidar".

There is no A1M8 in that table, and the SLAMTEC entry is an **S2E** --
30 m and 10 Hz against the A1M8's 12 m and 5.5 Hz. So the A1M8 starts from
`Example_Rotary_2D`, which is genuinely single-plane, and
`config/lidar/RPLIDAR_A1M8.json` is written onto its
`omni:sensor:Core:*` attributes. `scanRateBaseHz` is integral on the
prim, so the 5.5 Hz default becomes 6 Hz; the A1M8 is adjustable 2-10 Hz
so that is a setting it genuinely has, just not its default one.

**No-return is -1, not inf.** Below `range_min`, so consumers following
the LaserScan contract discard it correctly, but anything testing
`isinf()` will read it as a real measurement a metre behind the sensor.

The A1M8 sits on the front deck, so the platform's own superstructure
blocks part of its rear view; returns at ~0.4 m behind it are the robot.

### Known simplifications

- One render product feeds both colour and depth, so they are
  pixel-identical and share the colour FOV (69 deg, verified 69.0 x 42.5
  against the D435 spec). On hardware depth is wider at 87 deg and sits
  a baseline away with its own intrinsics. Splitting them is two render
  products and twice the GPU cost.
- The Ouster mount offsets are the OS-series family defaults (lidar
  origin 36.18 mm above the sensor origin, yawed 180). A specific unit's
  values come from its own metadata JSON, which this workspace
  deliberately does not commit.
- Isaac's OS0 profile is 0.5-75 m; the real OS0 is roughly 0.3-50 m.
- Anything printed after `SimulationApp` starts can disappear into Kit's
  logging, so `urdf_to_usd.py`'s "wrote ..." may not show.
