# TODO

Outstanding items found while bringing this workspace up in simulation.
Each one names the repo it applies to, because some belong to
`S-abk/Ranger_xarm` (private) or to upstream `xarm_ros2` rather than here.

---

## Revert the xarm_ros2 pin once #180 merges

**Repo:** this one · **File:** `ranger_xarm.repos`
**Upstream PR:** [xArm-Developer/xarm_ros2#180](https://github.com/xArm-Developer/xarm_ros2/pull/180)

`ranger_xarm.repos` points at `S-abk/xarm_ros2` @ `8d01a1b`, which is the
upstream jazzy commit `3dc2b5e` plus one commit declaring the gripper's five
follower joints to ros2_control with `mimic="true"`.

That is needed because the sim runs on dartsim, which has no mimic constraint
support (gazebosim/gz-physics#432), so without it the gripper linkage comes
apart. The fork exists only so a clean clone works today instead of whenever
the PR lands.

When #180 merges: set the url back to `xArm-Developer/xarm_ros2` and pin the
merged commit. Nothing else changes.

**Do not verify the gripper from TF.** `robot_state_publisher` derives mimic
joints kinematically from the URDF, so `/tf` reports a correct, symmetric
gripper whether or not physics is enforcing the linkage. Measure what gz
publishes instead:

```bash
gz topic -e -t /world/empty_ground/dynamic_pose/info -n 1 | grep -A6 xarm_left_finger
```

Closed and working, the fingers sit at y = -0.0269 / +0.0269, symmetric about
the centreline, 0.0538 m apart. Broken, they sit at -0.0269 / +0.0444 and
0.0713 m apart while TF still insists on 0.0540.

---
## Fix the `rviz` build break in `Ranger_xarm` (private)

**Repo:** `S-abk/Ranger_xarm` · **File:** `src/ranger_xarm_description/CMakeLists.txt`
**Severity:** breaks a clean clone; invisible on a machine that already has the directory.

`CMakeLists.txt` installs a directory that is not in the repository:

```cmake
install(DIRECTORY urdf config launch rviz meshes
  DESTINATION share/${PROJECT_NAME}
)
```

There is no `src/ranger_xarm_description/rviz/` in git, and `.gitignore` does not
exclude one, so `colcon build --packages-select ranger_xarm_description` fails:

```
CMake Error at cmake_install.cmake:46 (file):
  file INSTALL cannot find ".../src/ranger_xarm_description/rviz": No such
  file or directory.
```

It builds today only on machines that happen to have an untracked local `rviz/`
folder left over from earlier work. A fresh clone, a new workstation, or CI will
fail on the first build. The same defect was present in `ranger_xarm_ws` and is
already fixed there.

**Why `rviz` is the right thing to drop rather than create:** nothing references
an RViz config from this package. `display.launch.py` starts RViz with no `-d`
argument, so it loads the default config:

```python
Node(package='rviz2', executable='rviz2', output='screen'),
```

The only tracked `.rviz` files in the workspace live in
`ranger_xarm_moveit_config/rviz/` (`planning.rviz`, `hardware_check.rviz`), and
that package installs its own directory. So the entry is vestigial.

**Fix** — drop `rviz` from the install list:

```cmake
install(DIRECTORY urdf config launch meshes
  DESTINATION share/${PROJECT_NAME}
)
```

**Alternative, only if a description-local RViz config is actually wanted:**
create `src/ranger_xarm_description/rviz/`, commit a real `.rviz` file into it
(git does not track empty directories, so a `.gitkeep` would be needed
otherwise), and point `display.launch.py` at it with
`arguments=['-d', <path>]`. Do not do this just to satisfy the installer.

**Verify:**

```bash
rm -rf build/ranger_xarm_description install/ranger_xarm_description
colcon build --symlink-install --packages-select ranger_xarm_description
```

Confirm it is a genuinely clean check first — a stale untracked `rviz/` in the
source tree will mask the failure:

```bash
git status --ignored src/ranger_xarm_description
```


---

## Measure the loaded rolling radius on hardware

`wheel_radius` is 0.100036 m, measured off the tyre in
`ranger_mini_v3_cad.stl` and corroborated by AgileX's own `wheel_v3.dae`
(node matrix 0.1003675). That is the **unloaded** radius: CAD does not
sag. On the real platform each tyre carries roughly a quarter of ~100 kg
and deflects, so the rolling radius under load is smaller by an amount
nobody has measured.

Both simulators inherit the same optimism, because the tyre is a rigid
collision sphere in each, so neither will ever reveal it.

It matters in one direction. The kinematics use `omega = v / r`, so a
radius larger than the true rolling radius makes the platform run **slow**
by that ratio, and odometry built the same way over-reports distance.
That is opposite in sign to the 0.1026 error already corrected, so the
two partly cancelled.

Measuring it needs no instrumentation:

1. Mark a wheel and the floor, drive a straight line over a measured
   distance at the payload the robot actually carries.
2. Count wheel revolutions, or integrate `/dynamic_joint_states` position
   with unwrapping.
3. `r_eff = distance / total_radians`.

Deliberately a ratio of two measured quantities with no clock in it,
which is what makes it immune to the sim-time trap in
`ranger_xarm_isaac`'s README, and it is the same quantity that showed
0.1026 was wrong. Then set `wheel_radius` (and the `ranger_4wis_controller`
default) from the loaded figure, or expose it as a parameter if the
payload varies enough to matter.

## Resolved: the EKF heading fault was a two-parent TF tree

Kept because the failure mode is worth recognising, not because the bug
is still open.

`base_footprint` was enabled as a CHILD of `base_link`, while the EKF is
configured with `base_link_frame: base_footprint` and `publish_tf: true`
and therefore publishes `odom -> base_footprint`. The frame then had two
parents. tf2 allows exactly one, so the estimator's edge orphaned
robot_state_publisher's, and every lookup that had to cross the break
failed.

The IMU sits in `os_imu`, under `base_link`, on the far side of it.
`odom0` was unaffected because its `child_frame_id` is already
`base_footprint`, so it needs no lookup at all. The filter therefore
fused wheel velocity perfectly and never rotated:

```
                  pos err   % path    yaw err
wheel odometry     0.007 m   0.11%    +0.17 deg
EKF  (broken)      6.219 m  91.97%  -147.39 deg
EKF  (fixed)       0.093 m   1.39%    -0.01 deg
```

Fix: `base_footprint` is the ROOT, with `base_link` as its child, which
is the conventional layout for a mobile base and the one the EKF config
already assumed. The world weld moved to the root for the same reason.

What made this expensive to find:

- Nothing reports it. The filter runs, publishes at its configured rate,
  and its diagnostics say "functioning properly". The discard is visible
  only with `debug: true`, as "Could not transform measurement into
  base_footprint. Ignoring...".
- The transform resolves perfectly from any OTHER process, because the
  second parent only exists while the estimator is publishing. Every
  external check said the TF was fine.
- Half the inputs keep working, so the output looks plausible rather than
  absent.

The discriminator that found it, and the one to reach for next time: set
`base_link_frame` to the frame the sensor already hangs off. If the
measurement starts being fused, the target frame is the problem, not the
sensor, the data or the filter.

## Resolved: what the EKF buys across the four surfaces, and what it cannot

**Measured:** all four worlds, five trials each from a teleported fixed start,
`sensors:=true odom_tf:=false`. Ranges are across trials; flat ground is a
single run predating the harness fix below (no teleport, so unaffected).

| surface | wheel odom pos | wheel odom yaw | EKF pos | EKF yaw |
| --- | --- | --- | --- | --- |
| `empty_ground` | 0.10 % | +0.16 deg | 1.39 % | -0.01 deg |
| `low_friction` (mu 0.25) | 0.21 - 0.46 % | 0.4 - 0.9 deg | 0.04 - 0.28 % | <= 0.07 deg |
| `mixed_surface` (31 mu 0.15 patches) | 1.8 - 2.8 % | 0.3 - 4.4 deg | 1.4 - 2.2 % | <= 0.24 deg |
| `rough_ground` (101 bumps, 24 mm) | 42 - 237 % | 59 - 180 deg | 8.6 - 31 % | <= 0.4 deg |

Three things fall out of this.

Uniform low friction is not a hard surface for this base. A 4WIS platform
drives all four wheels at modest acceleration, so mu 0.25 barely slips and
both estimators stay under half a percent. Patchy friction is mildly harder
because the wheels lose grip one at a time; geometry is far harder than
friction of any kind.

Heading is where the filter always wins, and the margin widens exactly as the
surface gets worse: from parity on flat ground to two orders of magnitude on
the bumps. The cleanest single demonstration is `low_friction` trial 1, where
wheel odometry's heading collapsed to -69 deg and carried its position error to
4.84 m, while the EKF -- fed the same wheel velocities -- finished 3 mm from
ground truth. The wheel velocities were never the problem there; the yaw that
rotated them into the world frame was, and that is precisely the term the gyro
replaces.

Position is a different story, and flat ground is the one place the EKF is
(slightly) worse. It has no absolute position input anywhere, so on a surface
where dead reckoning is already near-perfect the filter can only add process
noise.

A per-segment probe on `rough_ground` says where the residual comes from.
Longitudinal traction over the bumps is
essentially perfect -- driving straight, the wheels report 0.350 m/s against a
ground truth of 0.350 m/s, and the filter accumulates 3 mm over six seconds.
The error is entirely lateral: during a steered arc the wheels claim
vx +0.281 while the base actually does vx +0.106, vy -0.124, and during a crab
the wheels claim vy +0.306 while the base does vx +0.160, vy +0.044.

The gyro observes the rotational half of that slip and corrects it, which is
why yaw holds to a fraction of a degree. Nothing in this filter observes the
translational half: `odom0_config` contributes vx and vy only and there is no
absolute position measurement anywhere in the graph. Closing that gap needs
lidar or visual odometry, not a better tuning of this EKF.

### Harness bug found while measuring this

The first four trials scored the EKF at 22 - 109 %, including a straight
segment where wheel and ground-truth velocity agreed to three decimal places
yet the score claimed 2.06 m of error. That was the scorer, not the filter.
Each estimator lives in its own world frame; teleporting the base to a fixed
start zeroes ground truth but leaves the filter integrating from the heading
it already held, so the two frames sit at an angle and a raw displacement
difference scores that angle as error. `ekf_score.py` and `segprobe.py` now
rotate the estimator displacement by the initial heading difference first. The
flat-ground numbers were taken without a teleport, so they were never affected.

### Benign: `Detected jump back in time` on rough ground

The heavier world drops the real-time factor to ~0.8 and `/clock` arrives
slightly out of order (about two inversions per eight seconds), which clears
the EKF's TF buffer a few times a second. It looks alarming in the log and it
is not blocking anything -- the gyro is still being fused, which is exactly
what the sub-degree yaw error demonstrates. Worth revisiting only if an
absolute-position source is added and starts dropping measurements.

## Open: the gz clock lags simulation under load, which invalidated the slip evidence

**Repos:** both · **Worlds:** `rough_ground.sdf`, `rough_rounded.sdf`, and
Isaac's `--rough-ground`.

### The rounded-bump test, and what it actually showed

`rough_rounded.sdf` is `rough_ground.sdf` with every box replaced by a
dome -- same 101 positions (both worlds now come from one
`rough_grid()`), same 24 mm height, same 0.16 m footprint, same mu 1.5,
no vertical face. It was built to test whether gz's apparent wheel slip
came from the tyre sphere striking a box edge.

The answer is that there was no slip to explain. Measured dt-free -- total
wheel rotation times radius, against ground-truth distance over the same
window -- gz does not slip on either shape:

| gz, straight run at 0.35 m/s | wheels rolled | base travelled | slip |
| --- | --- | --- | --- |
| boxes | 2.568 m | 2.562 m | 0.2 % |
| boxes | 2.582 m | 2.576 m | 0.2 % |
| domes | 2.506 m | 2.497 m | 0.4 % |

Isaac, measured the same way, does not slip either.

**The previously reported "+178 % gz slip against 0 % in Isaac" was an
artifact and is withdrawn.** So was the edge hypothesis it motivated.

### Root cause: the bridged /clock falls behind simulation

gz publishes `/clock` once per 1 ms physics step. `ros_gz_bridge` relays
every one, in order, without dropping any -- and under load it cannot keep
pace, so ROS time runs progressively slower than simulation time. Over a
10 s wall window:

| world | `/clock` (bridged) advanced | gz sim advanced | ROS time runs |
| --- | --- | --- | --- |
| boxes | 3.905 s | 8.240 s | 2.1x slow |
| domes | 2.197 s | 8.920 s | 4.1x slow |

Every ROS node on sim time uses that lagging clock: the controller_manager
stamps joint states with it, and `wheel_odometry.py` divides true wheel
travel by those compressed intervals, inflating its reported speed by the
lag factor. The apparent "slip" therefore tracked CPU load, not physics --
+178 % and +78-99 % on the same box world in different runs, +252-290 % on
domes, which are heavier to simulate. Ground truth and the bridged
sensors (IMU, lidar) carry gz's own stamps and are unaffected, which is
why the two disagreed.

### What this does and does not invalidate

- **Wheel-odometry pose is unaffected.** It integrates wheel travel, which
  needs no dt. Verified: 1056 joint-state messages, no repeated stamps,
  `/odom` distance 2.589 m against ground truth 2.592 m.
- **Wheel-odometry twist is inflated** by the lag factor on heavy worlds.
- **The gz EKF on heavy worlds is compromised.** `odom0_config` fuses
  only that twist (vx, vy), and the IMU it is fused with is stamped on
  gz's correct clock while the odometry is stamped on the lagging one.
  The rough-ground gz EKF figures (8.6-31 %) should be treated as suspect,
  and the "Detected jump back in time" warnings are likely the same
  fault. Flat-ground gz EKF (1.39 %) is probably unaffected at the lighter
  load, but that is not measured.
- **The gz/Isaac gap in wheel-odometry pose (42-237 % against
  0.43-4.47 %) is real and still unexplained.** It is not generated on
  straight segments; it comes from the steered arc and the crab, and
  mostly as heading error (59-180 deg in gz).

### Candidate for the remaining gap

The two sides do not drive the wheels the same way. Isaac uses an exact
velocity drive. gz runs `ranger_4wis_controller.py`'s PI speed loop
(`wheel_kp` 1.5, `wheel_ki` 4.0, `max_wheel_effort` 30) -- on the lagging
clock, so its integrator's dt is 2-4x too small. On a straight all four
wheels want the same speed and the loop's accuracy barely matters. In an
arc the inner and outer wheels need different speeds matched to the steer
geometry; if the loop tracks them poorly the wheels fight, the base
scrubs, and the least-squares heading goes wrong. Test: log commanded
against actual wheel speed per corner, and `wheel_odometry`'s
least-squares residual, through an arc on rough ground.

### Fixing the clock

Not done yet. Options: bridge `/clock` with a keep-last-1 QoS so a slow
consumer drops stale ticks instead of backlogging; have gz publish clock
at a lower rate than the physics step; or reduce load (the lag is
worst with rendering sensors on). Whatever is chosen, re-run the gz EKF
rough-ground scores afterwards -- the ones in this file predate it.

### Also noted, not fixed

`wheel_odometry.py` consumes each wheel position before its `dt <= 0`
early return, so a message with a repeated stamp would silently discard
that step's travel. Measured not to fire (0 repeats in 1056), but it
should integrate pose regardless and skip only the twist.

The per-segment "truth vx/vy" decomposition used in earlier probes
rotates ground-truth twist by ground-truth yaw, i.e. assumes it is
world-frame. That is unverified for either simulator; if either publishes
body-frame twist, the arc and crab slide figures are wrong. Rely on
distance and heading, which are frame-free.

### Eliminated earlier (still stands)

Geometry and collider identical; Isaac wheel friction 0.5 -> 1.2 no effect;
Isaac timestep 1/60 -> 1/240 s no effect; PhysX contact offset default ->
2 mm no effect.

### Environment faults (still stand)

A Gazebo server survived teardown for hours and served a second
`/controller_manager` from its in-process `gz_ros2_control` -- check
`pgrep -f "gz sim"`, since its process name is `ruby`. Stale Fast DDS
segments in `/dev/shm` compound it; clear them between runs. `grep` is
unreliable on USD crate files. The earlier 5-8x vertical-acceleration
comparison was confounded by mismatched IMU rates.

## Watch: the non-finite guard in wheel_odometry.py is untested in anger

**Repo:** this one · **File:** `src/ranger_xarm_gazebo/scripts/wheel_odometry.py`

The first Isaac rough-ground run scored NaN on every trial. PhysX emitted
a non-finite joint value while the base settled onto the bumps at spawn,
and one sample was enough to poison the integrator for the whole
session: NaN compares false against every bound, so it passed the
angle-wrap loop untouched, went through the least squares, and left x, y
and yaw NaN with nothing logged.

The guard drops such a sample and warns, throttled. It did not fire on
the rerun, so the underlying event is intermittent and the guard has
never been observed catching a live one. If a run ever reports
`non-finite joint data from ...`, that is the guard doing its job and
the sample count is worth recording here.

## Optional: purge the Ouster metadata from published history

**Repo:** this one · **File:** `192.168.1-metadata.json` (removed from the tree)

The Ouster driver writes `<sensor-ip>-metadata.json` into whatever directory
the launch was started from, so it landed at the workspace root and was
committed. It carries the lidar's `prod_sn`, `prod_pn`, `image_rev` and
`build_date`, and the filename encodes the sensor subnet.

The working tree and all future commits are handled: the file is deleted and
`*-metadata.json` is in `.gitignore`.

**Not done:** the blob is still reachable in the commit that introduced it, so
it remains visible on GitHub. Removing it means rewriting published history:

```bash
git filter-repo --path 192.168.1-metadata.json --invert-paths
git push --force-with-lease origin main
```

That breaks every existing clone, so it is a deliberate call rather than a
cleanup. Weigh it against what is actually exposed: an RFC1918 subnet and a
lidar serial. Deliberately deferred in favour of keeping history linear.
