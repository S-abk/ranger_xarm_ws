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

ros2 launch ranger_xarm_bringup ekf_odom_imu.launch.py use_sim_time:=true   # odom -> base_footprint
ros2 launch ranger_xarm_bringup slam.launch.py                               # map -> odom
ros2 launch ranger_xarm_bringup navigation.launch.py                         # Nav2
```

Leave the EKF's `lidar_odometry` off here: see Known issues.

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

**Nav2** (`navigation.launch.py`, `config/nav2_params.yaml`): Smac 2D, MPPI,
behaviours, velocity smoother, collision monitor, BT navigator. Nav2's own
`navigation_launch.py` is not used: Jazzy's also starts the route and docking
servers, whose default configs expect files this robot lacks, and one node
failing to activate aborts the lot. The base is 4WIS, so MPPI samples vx,
vy and wz. MPPI's acceleration limits are Nav2's defaults (3 m/s^2): at the
controller's 1 m/s^2, MPPI clamped every command to 5 cm/s over a measured
speed that never caught up, and the robot crept at 3 cm/s.

**`FourWIS` motion model** (vendored `nav2_mppi_controller`, see its
`VENDORED.md`): MPPI's rollouts obey the steering knuckles: the command
delay, the knuckle slew, the controller's ±90 deg fold, its common gate and
its acceleration limit, as `ranger_4wis_controller.py` does them. Knuckles are
unit vectors, so a rollout step needs no trigonometry. Fitted to Isaac's step
response and checked on a held-out sequence, it predicts the base's velocity
in the 0.8 s after a command change with half Omni's error (0.092 vs 0.190
m/s; yaw rate 0.127 vs 0.186 rad/s).

**`ranger_4wis_controller.py`**: below 3 cm/s at a wheel the knuckle now holds
and the wheel drives the command's component along it, and within 10 deg of
the ±90 deg fold a knuckle keeps its side. Planner corrections of a few cm/s
were swinging the knuckles back and forth by up to 90 deg while the base went
nowhere (visible in the Isaac GUI).

## Measured (Isaac, outdoor terrain in the room, same 8-goal route)

| MPPI model | succeeded | total time | path | min clearance | command variation |
|---|---|---|---|---|---|
| Omni | 7/8 | 124 s | 28.4 m | 0.29 m | 27 |
| FourWIS | 6/8 | 345 s | 26.5 m | 0.65 m | 48 |

The goal both missed lay on a phantom wall in the map (below). FourWIS keeps
more than twice the clearance but hesitates: one leg past the tanks took
172 s and failed where Omni took 20 s. Its predictions are better; MPPI's
critics, tuned for Omni-style rollouts, do not yet turn that into better
driving.

## Known issues, next steps

- **FourWIS closed loop**: retune the critics for it (the goal and path
  critics see slower predicted progress), and look at the knuckle swings in
  the last second before a goal.
- **Map degrades during long navigation**: slam_toolbox kept mapping through
  the Nav2 runs and ended with two overlapping, rotated copies of the room.
  Map first, then navigate in slam_toolbox's localization mode on the saved
  map. Contributing: the Ouster sits behind the mast and arm, which hide
  about half of its view.
- **KISS-ICP fusion under load**: with Isaac, SLAM and the filter all running
  (real-time factor 0.2 - 0.4) the lidar-fused EKF jumped 0.3 m at a time and
  the relay dropped its anchor 21 times in one loop; wheels + gyro stayed
  smooth. SLAM already corrects odometry with the lidar, so it stays off.
- **Isaac RTX lidar artefact**: in some frames a sector returns at ~0.7 of
  its true range (seen with one publisher and a stationary robot).
- **Real-time factor**: 0.2 - 0.5 with the whole stack and the GUI; Nav2's
  control loop runs on wall time, so a slow simulation shortens its horizon.
