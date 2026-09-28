# SLAM and Nav2 on the 3D lidar

Status as of 2026-09-28: working in Isaac Sim (the primary simulator), with
known issues listed at the end. Everything here is robot-level (it reads the
same topics and frames the real drivers publish); only the numbers are from
simulation.

## Running it (Isaac)

```bash
# simulator + controllers (see ranger_xarm_isaac), e.g. the outdoor terrain in the room:
~/isaacsim/python.sh .../isaac_bringup.py --terrain --room --usd .../ranger_xarm_wheeled.usd
ros2 launch ranger_xarm_isaac control.launch.py drive_base:=true odom_tf:=false

# every shell: a 2-3 MB Ouster cloud needs the large shared-memory segment
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix ranger_xarm_bringup)/share/ranger_xarm_bringup/config/fastdds_large_shm.xml

ros2 run ranger_xarm_bringup arm_pose.py travel --ros-args -p use_sim_time:=true
ros2 launch ranger_xarm_bringup ekf_odom_imu.launch.py use_sim_time:=true   # odom -> base_footprint
ros2 launch ranger_xarm_bringup slam.launch.py                               # map -> odom (mapping)
# drive the site, then save it and navigate on it in localization mode:
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/path/site'}"
ros2 launch ranger_xarm_bringup slam.launch.py mode:=localization map_file:=/path/site map_start_pose:='[x, y, yaw]'
ros2 launch ranger_xarm_bringup navigation.launch.py                         # Nav2 + mode arbiter
```

Leave the EKF's `lidar_odometry` off here: see Known issues. Mapping mode
left running through long Nav2 sessions degrades the map (two rotated copies
of the room after an hour), so navigation runs on a saved map.

## The pieces, and why each is the way it is

**`lidar_terrain_filter.py`** turns the Ouster's cloud into what 2D SLAM and
the costmaps need, on uneven ground:

- *deskew*: a spinning lidar collects its 360 deg over 0.1 s; while turning at
  0.6 rad/s the two ends of a sweep disagree by 3.4 deg. Points are moved to
  the stamp time with the gyro's yaw rate and the odometry's velocity, timed
  by the Ouster's per-point `t` or, in Isaac, by firing order (Isaac stamps the
  END of the sweep: deskewed that way 98 % of a spinning scan's points lie
  within 7 cm of a stationary scan, against 81 % raw and 49 % the other way);
- *self-filter*: a box around the body, mast and parked arm;
- *level*: roll and pitch from a complementary filter on the IMU (the OS0's
  IMU is 6-axis and Isaac's orientation field is identity);
- *obstacles by height above the local ground*: a fixed height band sees the
  ground as a wall once the robot pitches (6 deg measured on the terrain puts a
  horizontal slice on the ground 14 m out). A point more than 15 cm above its
  0.25 m cell's lowest point is an obstacle: slopes and the 5 cm stones are
  not, walls, tanks and boulders are. Holes and drops are not detected.

It publishes `/lidar/obstacles` (costmaps, collision monitor) and
`/lidar/scan` (slam_toolbox), ~7-15 ms per cloud.

**slam_toolbox** (online async, `config/slam_toolbox.yaml`) on `/lidar/scan`.
It is a lifecycle node in Jazzy: started plainly it sits unconfigured and
publishes nothing, so `slam.launch.py` configures and activates it. With
ground-truth odometry the room maps cleanly, and with the wheels + gyro EKF
too (square walls, all four tanks) over one mapping loop.

**Steering modes.** The Ranger's wheels steer independently but move in
modes. AgileX's driver (`agilexrobotics/ranger_ros2`, `TwistCmdCallback`)
picks one per command: *parallel* if `linear.y != 0` (all four wheels at
one angle, `angular.z` ignored), *spinning* if `|vx|/|wz|` < 0.4764 m
(`linear.x` ignored, wheels on the diagonals), otherwise *dual Ackermann*
(front and rear mirrored about the lateral axis, steer `atan((l/2)/R)`
capped at 0.601 rad). So:

- `ranger_mode_arbiter.py` sits between Nav2 and the robot (`cmd_vel_raw`
  -> `cmd_vel`) and reduces each twist to one mode: parallel when `|vy|`
  outweighs `|wz| * 0.3 m`, else spinning or Ackermann by radius, with
  exact zeros so the driver makes the same choice. MPPI's vy is never
  exactly zero; sent raw, the real driver would stay in parallel mode and
  never turn. A mode change needs the new mode to dominate by 1.5x and the
  old one to have lasted 0.4 s, because it costs a knuckle swing.
- `ranger_4wis_controller.py` (`steer_modes: agilex`, the default) executes
  a twist the way the driver and firmware do, so in simulation the wheels
  hold the mode's geometry (measured: parallel within 1 deg on all four,
  Ackermann front/rear mirrored with the inner wheels steering more, spin at
  the ±53 deg diagonal). One assumption: the firmware realises the driver's
  steering angle as the INNER wheel's (the driver's own odometry reads it
  back that way), so an Ackermann turn is about R + track/2. The firmware is
  closed; check on the robot.

**Nav2** (`navigation.launch.py`, `config/nav2_params.yaml`): Smac 2D, MPPI,
behaviours, velocity smoother, collision monitor, BT navigator. Nav2's own
`navigation_launch.py` is not used: Jazzy's also starts the route and docking
servers, whose default configs expect files this robot lacks, and one node
failing to activate aborts the lot. The base is 4WIS, so MPPI samples vx,
vy and wz. MPPI's acceleration limits are Nav2's defaults (3 m/s^2): at the
controller's 1 m/s^2, MPPI clamped every command to 5 cm/s over a measured
speed that never caught up, and the robot crept at 3 cm/s.

**`FourWIS` motion model** (vendored `nav2_mppi_controller`, see its
`VENDORED.md`): each rollout step passes the command through Nav2's velocity
smoother, the steering-mode reduction above, a command delay, the knuckle
slew, the controller's ±90 deg fold, its common gate and its acceleration
limit. Knuckles are unit vectors, so a step needs no trigonometry. The
smoother matters: MPPI's sampling noise is independent per step, so a raw
sampled sequence changes direction every 0.05 s, and without the smoother
nearly every rollout went nowhere and the robot sat still (40 s at 1.5 cm/s
before a goal needing a 180 deg turn). On a held-out sequence it predicts
the base's velocity in the 0.8 s after a command change with a third less
error than Omni (0.141 vs 0.213 m/s) and yaw rate with half (0.092 vs 0.169
rad/s): Omni predicts turns the driver drops in parallel mode.

**`ranger_4wis_controller.py`**: when every wheel's commanded speed is below
3 cm/s the knuckles hold together and the wheels stop, and within 10 deg of
the ±90 deg fold a knuckle keeps its side. Planner corrections of a few cm/s
were swinging the knuckles back and forth by up to 90 deg while the base went
nowhere (seen in the Isaac GUI), and holding each knuckle on its own left the
wheels at mismatched angles.

## Measured (Isaac, outdoor terrain in the room, steering modes on)

A 6-leg course (open run, past the tanks, far corner with a 90 deg turn, a
pure sideways move, home) on a saved map in localization mode. The test
harness fits the saved map to the room's known walls for a fixed world ->
map transform and re-anchors the localization from ground truth before
each course (test-only; localization error stayed 0.1 - 0.2 m throughout).

| MPPI model | run 1 | run 2 | min clearance | knuckle travel |
|---|---|---|---|---|
| Omni | 224.1 s | 134.1 s | 0.52 - 0.60 m | 9,300 - 10,700 deg |
| FourWIS | 134.9 s | 92.7 s | 0.61 - 0.63 m | 8,600 - 9,200 deg |

All 6/6 in every run. FourWIS was faster in both pairs (~35 % on average),
kept more clearance and swung the knuckles less; the consistent gap is the
far-corner leg (FourWIS 21-22 s both times, Omni 66 and 112 s). Two runs
each: an indication, not a statistic. Earlier comparisons in this file's
history ran on a controller that executed any twist, which the real robot
does not, and are superseded.

## Planned: a test site for global planning

A seeded extension of `make_outdoor_terrain.py`, built for both simulators,
rather than NVIDIA's sample scenes (flat, indoor only, not redistributable):

- ~40 x 40 m of the scanned-ground terrain with gentle slopes, CC0 boulder
  fields and trees, and a simple building (doorways ~1.0 m for the 0.52 m
  wide base, a corridor, a dead-end room), so routes need detours, the right
  doorway, and to avoid traps longer than the local costmap sees;
- the generator exports the true occupancy grid, so SLAM maps are scored
  against it, and a goal suite scored on success, time, path length against
  the optimal path, and clearance;
- an optional 4WIS suite, places a car-like base cannot go:
  - an L-shaped slot whose turn is too tight to turn in, passable only by
    crabbing sideways;
  - a parallel-parking bay entered from the side;
  - lateral docking against a wall or station, holding heading;
  - a narrow dead end the base must leave by spinning in place or crabbing;
  - zig-zag gates that reward diagonal travel over turning.

NVIDIA's warehouse/hospital scenes stay useful as a quick flat, cluttered
indoor check.

## Known issues, next steps

- **Mode preferences, slopes**: nothing yet prefers a steering mode (e.g.
  Ackermann over crabbing), limits speed per mode, or accounts for slope
  and stability. Planned as MPPI critics and a traversability layer.
- **Firmware steering semantics**: verify on the robot that the Ackermann
  steering angle is realised as the inner wheel's (commanded vs measured
  yaw rate on an arc), and refit FourWIS's delay and acceleration there.
- **Nav2 startup under load**: two of seven starts hung (a lifecycle
  service timeout, once in planner_server's configure) at a real-time
  factor of ~0.3; the test harness retries.
- **KISS-ICP fusion under load**: with Isaac, SLAM and the filter all running
  (real-time factor 0.2 - 0.4) the lidar-fused EKF jumped 0.3 m at a time and
  the relay dropped its anchor 21 times in one loop; wheels + gyro stayed
  smooth. SLAM already corrects odometry with the lidar, so it stays off.
- **Isaac RTX lidar artefact**: in some frames a sector returns at ~0.7 of
  its true range (seen with one publisher and a stationary robot).
- **Near-field blind zone**: the OS0 sees at most 45 deg down from 1.51 m, so
  ground closer than ~1.5 m is outside its field of view, and the chassis and
  gantry shadow the ground 1-3 m out to the sides and rear (68 % of that ring
  is seen, whatever the arm does). The arm itself hides almost nothing at
  wall height (99 % of bearings see the room); `arm_pose.py travel` (joint 2
  at -35 deg) only cuts its self-returns from 31 to 11. The front RPLIDAR and
  the D435 cover the near field and are not in the costmaps yet.
- **Real-time factor**: 0.2 - 0.5 with the whole stack and the GUI; Nav2's
  control loop runs on wall time, so a slow simulation shortens its horizon.
