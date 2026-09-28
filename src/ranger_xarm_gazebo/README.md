# ranger_xarm_gazebo

Gazebo (gz) simulation of the platform.

```bash
ros2 launch ranger_xarm_gazebo sim.launch.py
ros2 launch ranger_xarm_gazebo sim.launch.py headless:=true
ros2 launch ranger_xarm_gazebo sim.launch.py world:=/abs/path/to/your.sdf
```

## How it hooks up

`ranger_xarm_description` carries **one** URDF, used by both simulation and
hardware. Simulation flips three xacro arguments and changes nothing else:

| Argument | Hardware | Simulation |
| --- | --- | --- |
| `xarm_ros2_control_plugin` | `uf_robot_hardware/UFRobotSystemHardware` | `gz_ros2_control/GazeboSimSystem` |
| `xarm_load_gazebo_plugin` | `false` | `true` |
| `xarm_ros2_control_params` | *(empty)* | generated controller yaml |

All three default to the hardware values, so nothing changes for the real
robot unless a launch file asks. Keeping one description is the point: a
separate simulation URDF drifts, and the drift is invisible until a plan that
worked in simulation collides on the robot.

The controller parameters are produced at launch from
`xarm_controller/config/xarm6_controllers.yaml` through upstream's
`generate_ros2_control_params_temp_file`, which rewrites it for our `xarm_`
prefix and stamps `use_sim_time`. That helper is reused rather than forked so
the config cannot silently age against the upstream it came from.

## Two things worth knowing before you edit this

**The model is spawned from the `/robot_description` topic, not from a file.**
So gz gets byte-for-byte what `robot_state_publisher` is using. Spawning from
a separately expanded file is how the TF tree and the physics model drift
apart.

**Controller spawning is chained to the spawn finishing.** The
`controller_manager` the spawners talk to is created by the plugin *inside the
spawned model*, so it does not exist until gz has the model. Starting the
spawners in parallel is a race that usually works and occasionally does not.

## Worlds

`worlds/empty_ground.sdf` is a ground plane and a sun, deliberately. This
workspace is the platform, not an application. Put your scenery in your own
package and pass `world:=`.

`worlds/sensor_test.sdf` is that plus three boxes at known positions. It
exists to check the sensors against arithmetic rather than by eye, not as
scenery; see **Sensors** below.


## Sensors

Off by default; `sensors:=true` adds them. Each rendering sensor costs a
render pass every frame, and anything consuming this description for MoveIt,
RViz or the real robot wants the frames without them.

```bash
ros2 launch ranger_xarm_gazebo sim.launch.py drive_base:=true sensors:=true
```

Topics and frame ids match what the **real** drivers in
`ranger_xarm_sensors` publish, so nothing downstream can tell which it is
talking to. `ranger_xarm_isaac` publishes the same set, so the two
simulators are interchangeable too.

```
Ouster OS0     /ouster/points                       os_lidar
               /ouster/imu                          os_imu
RPLIDAR A1M8   /scan                                laser_frame
RealSense      /camera/d435/color/image_raw         d435_color_optical_frame
               /camera/d435/depth/image_rect_raw    d435_depth_optical_frame
               /camera/d435/color/camera_info
```

The sensor-internal frames are deliberately **not** links in the URDF. On
hardware the drivers own them, publishing from the sensor's own metadata and
calibration, which is why `d435_mount_link` is in the description and
nothing below it is. Putting them in the URDF would collide with the
driver's transforms the moment a real sensor is plugged in, so gz names them
with `<gz_frame_id>` and the launch publishes the transforms instead.

### Checked against arithmetic

`worlds/sensor_test.sdf` puts boxes at known positions, because on a bare
plane a working sensor and a broken one look identical: the 2D scanner sees
only the robot and the 3D one sees only the floor.

```bash
ros2 launch ranger_xarm_gazebo sim.launch.py drive_base:=true sensors:=true \
    world:=$(ros2 pkg prefix ranger_xarm_gazebo)/share/ranger_xarm_gazebo/worlds/sensor_test.sdf
```

```
BoxA  predicted 2.452 m ahead      measured 2.452   0 mm
BoxC  predicted 2.800 m at -90     measured 2.800   0 mm
BoxB  predicted 2.593 m at +155    no return, occluded by the robot
Ouster height above floor          1.513 m vs 1.509 predicted, 4 mm
```

BoxB is correct rather than missing: the A1M8 sits on the front deck and the
platform's own superstructure blocks the rear-left. The positions match
`ranger_xarm_isaac`'s obstacle flag exactly, so the same predictions apply to
both and a disagreement between them is a real disagreement.

### Where gz and Isaac differ

- **No-return.** gz gives `inf`, which is the LaserScan convention. Isaac's
  RTX lidar gives `-1`. Both are below `range_min` so a correct consumer
  discards either, but code testing `isinf()` is only right on one of them.
- **Scan rate.** gz takes the A1M8's 5.5 Hz directly. Isaac's RTX scan rate
  attribute is integral, so it runs at 6 Hz there.
- **Ouster range.** The real OS0 is roughly 0.3 to 50 m, which is what gz is
  given here. Isaac's shipped OS0 profile says 0.5 to 75.

### Needed in the world

`gz-sim-imu-system`, which is separate from `gz-sim-sensors-system`. The
latter covers the rendering sensors; an `imu` sensor is simply never updated
without the former, and nothing complains. The topic just never appears.

## Odometry and ground truth are different topics

`/odom` is dead reckoning from the wheel encoders. `/ground_truth/odom` is
the simulator's own answer. They were the same topic until now, and in
Isaac's case `/odom` *was* `IsaacComputeOdometry` — the chassis pose read
out of the physics engine, published under odometry's name. Anything
downstream believed it had odometry when what it had was the answer, so a
state estimator fed from it could not be wrong and could not be tested.

`wheel_odometry.py` uses only what the real platform knows: steer angle
and differenced wheel position, solved by least squares across all four
corners (the inverse of the controller's IK). Wheel travel is differenced
from POSITION rather than read from the simulator's velocity estimate,
because a real encoder counts, and counting is what makes odometry drift
the way real odometry drifts.

```
                gz            Isaac
path length     5.902 m       5.422 m
position error  0.003 m       0.002 m
                0.06%         0.05% of distance
heading error   -0.09 deg     +0.62 deg
```

### Ground surface decides whether this is a test at all

On flat ground dead reckoning tracks ground truth to under a tenth of a
percent, which is not a compliment to the odometry: it means the floor
never lets the wheels slip. `scripts/make_surface_worlds.py` generates
three variants that break that, and they do not break it equally. Same
drive on each:

```
world           path      pos err    % of path   heading err
empty_ground    5.961 m   0.003 m      0.05%      -0.09 deg
low_friction    5.937 m   0.005 m      0.09%      -0.04 deg
mixed_surface   5.840 m   0.027 m      0.46%      +1.11 deg
rough_ground    4.506 m   1.123 m     24.92%     +29.75 deg
```

**Low friction alone does almost nothing.** mu 0.25, a sixth of the
default, moved the error from 0.05% to 0.09%. At 0.35 m/s the wheels
never demand enough traction to break grip, so uniformly slippery ground
is not a test of anything. Grip only matters if something asks for it.

**Losing contact is what destroys odometry.** 24 mm bumps produced 25%
position error and nearly 30 degrees of heading error. The mechanism is
visible in the path length: the robot covered 4.5 m where the other
worlds covered 5.9, while the wheels kept turning the whole time. A wheel
that is turning is not a wheel that is travelling, and an encoder cannot
tell the difference.

**Differential grip is the one to watch.** mixed_surface is only 0.46% of
distance, nine times the baseline but still small, yet it is the case
that produces heading error from a straight-line drive, because wheels on
the same axle are on different surfaces. Heading error is the one that
never washes out.

So: use `empty_ground` to check that an estimator is correct, and
`rough_ground` to check that it survives. A result from the flat world
says nothing about robustness.

**`rough_ground` is a lattice, and the robot can miss it.** Its rows sit
at y = 0 and +/-0.55 m and the wheels at y = +/-0.185 m, so a straight
start runs between them; on the scored drive a wheel was on a bump for a
quarter of the distance. `outdoor_terrain.sdf` is the realistic
alternative: a Blender-built mesh from scanned ground and scanned stones
(`models/outdoor_terrain`, `scripts/make_outdoor_terrain.py`), undulating
everywhere, with no pattern to drive around, and the same mesh is Isaac's
`--terrain`. `outdoor_terrain_room.sdf` adds walls for lidar odometry.

**Error is not the same as drift.** The gz numbers above are from the
empty world. The same legs run in `sensor_test.sdf` gave 1.94% and +2.29
deg, because the robot grazed a box and the wheels spun — which is
exactly what odometry is supposed to get wrong, and a reminder that a
single number here characterises the run, not the estimator.

### What a wrong wheel radius does

`wheel_radius` is a multiplier on every wheel displacement, and in a 4WIS
forward kinematic it scales the solved yaw as well as the solved
translation, because both come out of the same least-squares system. A
radius error is therefore a heading error too, and on a curved path the
heading error compounds the position error rather than adding to it.

Measured on the same 5.6 m path with 137.6 deg of turning:

```
r = 0.100036 (measured)   0.006 m   0.10% of path   -0.06 deg
r = 0.1026   (legacy)     0.418 m   7.41% of path   +3.47 deg
```

A 2.56% radius error produced a 7.41% position error. This is stated
because `wheel_radius` read 0.1026 in this workspace until it was
measured, and anything calibrated against odometry during that period
inherited it. That includes `ranger_xarm_bringup`'s EKF, whose gyro would
have disagreed with wheel heading by roughly this much on this kind of
path. Whether that actually mistuned it is **not established here** — the
point is that the mechanism exists and is now measurable, not that a
verdict has been reached.

## What this does not do

It does not start MoveIt. Bring the simulation up first, confirm the
controllers are active, then launch planning separately — when they start
together, a controller that failed to spawn looks exactly like a planning
failure.

It does not give you real perception. `sensors:=true` simulates the Ouster,
the D435 and the RPLIDAR (see below), but the returns are ideal geometry.
They do not have the return statistics of the physical sensors — grazing
incidence dropouts, retroreflector blooming, dust, sunlight on the depth
pair. A perception stack that works here has been shown to work against
clean geometry and nothing more.
