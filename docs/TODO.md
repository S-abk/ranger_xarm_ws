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

## Resolved: what the EKF buys across the four gz surfaces

**Measured after the clock fix below**, all four worlds, five trials each
from a teleported fixed start, `sensors:=true odom_tf:=false`, 0
"jump back in time" warnings on any world (160 on rough ground before).
Reproduce with `run_surface_sweep.sh <world> 5`.

| surface | wheel odom pos | wheel odom yaw | EKF pos | EKF yaw |
| --- | --- | --- | --- | --- |
| `empty_ground` | 0.10 - 0.11 % | +0.16 - +0.18 deg | 0.09 - 0.14 % | <= 0.01 deg |
| `low_friction` (mu 0.25) | 0.42 - 0.45 % | +0.62 - +0.65 deg | 0.08 - 0.09 % | <= 0.05 deg |
| `mixed_surface` (31 mu 0.15 patches) | 2.2 - 3.9 % * | -4.6 - +0.4 deg * | 1.9 - 3.2 % * | <= 0.28 deg |
| `rough_ground` (101 bumps, 24 mm) | 22.9 - 47.0 % | +20.6 - +68.5 deg | 12.2 - 27.8 % | <= 0.87 deg |

\* `mixed_surface` trial 4 is excluded from the ranges and reported
separately: the base did not turn at all (net ground-truth heading
+0.0 deg against ~+138 deg in every other trial) while wheel odometry
believed it had turned +133 deg, giving 73.6 % wheel-odometry and 30.9 %
EKF error. The gyro was right (EKF yaw error -0.00 deg). Something
physical stopped the base following the arc -- likely a wheel caught on a
patch edge -- and it happened once in five.

**The EKF is now at least as good as wheel odometry on every surface.** The
earlier table put the flat-ground EKF at 1.39 % against wheel odometry's
0.10 %, and this file concluded that on a surface where dead reckoning is
already near-perfect the filter "can only add process noise". **That
conclusion is withdrawn**: it was the lagging clock, which inflated the
wheel twist that `odom0` fuses. With the clock fixed, flat ground is
0.09-0.14 % for the EKF, parity with wheel odometry, and low friction is
0.08-0.09 % against 0.42-0.45 %.

Heading is still where the filter earns its keep: sub-degree on every
surface, against wheel-odometry heading errors that reach 68 deg on the
bumps.

**These numbers are not directly comparable with the pre-fix ones.** The
clock fix changed how the robot drives, not only how it is measured:
`ranger_4wis_controller.py`'s PI wheel-speed loop ticks on a 10 ms ROS
timer with a fixed dt, so while the clock lagged it ran 2-4x less often
per simulated second. On rough
ground the base now consistently turns about 66 deg over a drive that
turns 138 deg on the flat, where before the net heading varied widely
between trials.

The EKF's residual position error on the bumps is translational. The gyro
observes rotation and corrects it, which is why yaw holds to a fraction
of a degree; nothing in this filter observes translation other than the
wheel twist, because `odom0_config` contributes vx and vy only and there
is no absolute position measurement anywhere in the graph. When the base
scrubs through an arc (see "gz under-rotates in arcs" below), the wheels
report motion the base did not make and the filter has no way to reject
it. Closing that gap needs lidar or visual odometry, not a better tuning
of this EKF.

An earlier per-segment probe here reported the arc and crab slide in
body-frame velocities. Those figures used the old inflated wheel twist
and an unverified world-frame assumption about ground-truth twist, and
have been removed.

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

### `Detected jump back in time` on rough ground was not benign

This file once called these warnings harmless. They were the symptom of
the lagging clock described below: 160 of them on rough ground before
the fix, none on any world after it.

## Resolved: the gz clock lagged simulation under load

**Repos:** both. **Fix:** `gz_clock_relay.py` replaces the stock clock
bridge in `sim.launch.py`; `wheel_odometry.py` computes twist from wheel
velocities.

### The fault

gz publishes `/clock` once per 1 ms physics step. The stock
`ros_gz_bridge` clock bridge relayed every tick, in order, never dropping
one, to every use_sim_time node -- 42 subscribers with the full stack up,
so ~42,000 deliveries a second. It sustained roughly a quarter of that
with a whole core pegged, and the shortfall accumulated as a backlog: ROS
time ran 2-4x slower than simulation on the rough-ground worlds (3.9 s
against 8.2 s of sim per 10 s wall on boxes, 2.2 s against 8.9 s on
domes). Everything stamped with ROS time inherited the lag; ground truth
and the bridged sensors, stamped by gz, did not.

Switching the bridge to the built-in `CLOCK` QoS profile (best-effort,
keep-last-1) did not help -- 0.57 against 0.74 -- because the cost was
the fan-out, not reliability.

### The fix, and the measurement

`gz_clock_relay.py` subscribes to gz `/clock` through gz-transport with a
250 Hz throttle, so excess ticks are dropped at the source and the ROS
clock always carries a current sim time. 250 Hz stays above the 150 Hz
controller_manager rate. Driving across the domes, the worst case:

| clock source | ROS clock rate / sim rate |
| --- | --- |
| stock bridge | 0.25 - 0.74 (load-dependent) |
| stock bridge, `CLOCK` QoS | 0.57 |
| `gz_clock_relay.py`, 250 Hz | 0.98 - 1.00 |

That alone was not enough for `wheel_odometry.py`. With 150 Hz joint
states stamped from a 250 Hz clock, each ~7 ms interval is recorded as
4 or 8 ms, and averaging travel/dt over that jitter still read 23-35 %
high -- the mean of the ratios exceeds the ratio of the means. The twist
now comes from the wheel *velocity* interface through the same least
squares as the pose, which needs no dt and so no clock: 0.350 m/s
against 0.350 m/s ground truth. Differencing stays as the fallback when
no velocity interface is present. Pose was never affected and is still
integrated from wheel travel.

The same change fixes the latent bug noted earlier: a repeated stamp
used to discard that step's wheel travel, because the position was
consumed before the `dt <= 0` early return. Pose now integrates
unconditionally and the fallback twist carries such travel into the next
interval.

### Earlier findings that stand

The "+178 % gz slip" was this artifact, and the edge hypothesis with it:
measured dt-free, gz does not slip on straights over boxes (0.2 %) or
domes (0.4 %). `rough_rounded.sdf` stays in the tree as a same-grid,
edgeless variant. Eliminated as causes of the gz/Isaac gap: geometry and
collider, Isaac wheel friction, Isaac timestep, PhysX contact offset.

## Diagnosed and fixed on rough ground: gz under-rotated in arcs because its wheel speed loop was too soft

**Probe:** `arc_probe.py` drives 6 s straight onto the terrain, then holds
the scored arc (vx 0.30 m/s, wz 0.40 rad/s) and logs, per corner, target
against actual wheel speed and steer angle, wheel effort against the
30 N m clamp, a rigid-body fit of the four wheel velocities, and
ground-truth yaw rate -- which is frame-independent, unlike planar twist.

Flat ground is the control and is exact: every wheel on target, efforts
about 0.2 N m, fit residual 0.0000 m/s, yaw rate 100 % of command, 137.6
deg over a 137.5 deg arc.

On `rough_ground`, three runs, all within a degree of each other:

| | result |
| --- | --- |
| steer angles | exactly on target at every corner |
| wheel efforts | 0.6 - 8.5 N m mean, at the 30 N m clamp 0 % of the time |
| wheel speeds | -27 % to +4 % against target |
| wheel rigid-body fit residual | 0.085 - 0.095 m/s (four wheels disagree) |
| yaw rate implied by the wheels | 0.356 - 0.371 rad/s |
| ground-truth yaw rate | **0.236 - 0.237 rad/s, 59 % of command** |
| heading over the arc | 82 - 84 deg against 137.5 |

So the knuckles are not being knocked off angle, and the drive is not
torque-limited -- it has 20+ N m of headroom it never uses. The wheels
simply miss their speeds under bump loads, stop agreeing on one body
motion, fight, and the base scrubs.

Whether that is cause or symptom was settled by stiffening only the
speed loop (`wheel_kp`, `wheel_ki` scaled together) on the live sim:

| gains | ground-truth yaw rate | heading over the arc | efforts at clamp |
| --- | --- | --- | --- |
| 1x (stock: kp 1.5, ki 4.0) | 59 % | 83 deg | 0 % |
| 3x | 91 % | 123 deg | 32 - 44 % |
| 5x | 98 % | 132 deg | 32 - 46 % |
| 10x | 101 - 105 % | 132 - 141 deg | 42 - 56 % |

Rotation tracks drive stiffness monotonically and reaches the full turn,
so **the cause is the drive, not the contact**: the bumps deliver the arc
once the wheels hold their speeds. This is the gz/Isaac rough-ground gap.
Isaac's velocity drive (damping 1e3) is effectively a stiff speed source
and never had the problem.

### The stiffened loops are not a fix

From 3x upwards the loop chatters: efforts sit at the clamp a third to
half the time with means near zero, and the fit residual rises to
0.43 - 0.53 m/s. The wheels reach their target speeds only on average.
The loop closes over DDS -- speed feedback from `/dynamic_joint_states`,
effort out through a topic, a 10 ms timer -- and the latency that adds
limits how stiff it can be. Whether the chatter is inherent to that
latency or excited by the bumps has not been tested (it would show up on
flat ground at 5x if inherent).

### The velocity drive: tried, and it closes the gap

`sim.launch.py wheel_drive:=velocity` (default still `effort`) sends wheel
speeds to dartsim's own joint velocity drive, capped by the 60 N m joint
effort limit, instead of closing a PI loop over DDS. The controller and
xacro pieces now match on both branches: `command_mode` in
`ranger_4wis_controller.py`, `wheels_command_interface` in the xacro,
both defaulting to effort, and the expanded default URDF is byte-identical
to before.

**The rigid-constraint failure does not occur on dartsim.** A commanded
spin in place yields 100 % of its yaw rate (170 deg against 172; the gap
is the ramp-up), where bullet-featherstone gave under 10 %.

Arc on rough ground, `arc_probe.py`:

| | effort, stock | effort, 5x gains | velocity drive |
| --- | --- | --- | --- |
| wheel speed error | -27 to +4 % | -4 to +9 % (chattering) | 0 % |
| wheel fit residual | 0.09 m/s | 0.49 m/s | 0.0005 - 0.001 m/s |
| yaw rate vs command | 59 % | 98 % | 98 % |
| heading over the arc | 83 deg | 132 deg | 134 - 135 deg of 137.5 |

Four-surface sweep, five trials each, 0 clock warnings, velocity
controller and velocity mode confirmed on every world:

| surface | effort: wheel odom / EKF | velocity: wheel odom / EKF |
| --- | --- | --- |
| `empty_ground` | 0.10 - 0.11 % / 0.09 - 0.14 % | 1.04 - 1.10 % / 0.14 - 0.20 % |
| `low_friction` | 0.42 - 0.45 % / 0.08 - 0.09 % | 0.38 - 0.40 % / 0.43 - 0.45 % |
| `mixed_surface` | 2.2 - 3.9 % / 1.9 - 3.2 %, 1 of 5 failed | 1.2 - 2.7 % / 0.11 - 0.64 %, **3 of 5 failed** |
| `rough_ground` | 22.9 - 47.0 % / 12.2 - 27.8 % | **3.5 - 8.4 % / 0.77 - 1.84 %** |

Rough ground is now comparable to Isaac (0.43 - 4.47 % / 0.54 - 2.89 %),
with net heading 125 - 136 deg instead of about 66. **The gz/Isaac
rough-ground gap was the drive.**

### Why the default stays effort: transition scrub

The velocity drive regresses elsewhere. On flat ground wheel odometry
carries a constant +1.33 deg heading error in every trial. Per segment,
heading being frame-free:

| segment | truth | wheel odom | error |
| --- | --- | --- | --- |
| straight | -0.00 | -0.00 | +0.00 |
| arc | +137.24 | +136.63 | -0.61 |
| crab | -0.28 | +1.75 | **+2.03** |
| straight | +0.17 | +0.07 | -0.09 |

It comes from the transitions, mostly into the crab, where the knuckles
sweep 0 -> 90 deg. A stiff velocity servo spins the wheels at full speed
throughout, so they push the base while still pointing the wrong way; the
base yaws slightly and the wheel kinematics, read from the measured steer
angles, stop agreeing. The effort loop's soft ramp-up had masked this.

The `mixed_surface` failures look related but are not diagnosed: three
of five trials came out deterministically identical (5.73 m path, +48 deg
net heading against +134, wheel odometry 49 %, EKF 24.6 %), a bimodal
outcome depending on small differences in how the base meets the 8 mm
low-grip patches. The effort drive had one such failure in five.

The `low_friction` EKF moving from 0.08 % to 0.44 %, now slightly worse
than wheel odometry, is also unexplained.

### Next: gate wheel speed on steering convergence

The standard 4WIS remedy: in velocity mode, scale each wheel's commanded
speed by how close its knuckle is to its target angle, e.g. by
max(0, cos(steer error)), so a wheel does not drive while it is still
turning. `ranger_4wis_controller.py` already subscribes to
`/dynamic_joint_states` and can read the steer positions there. Then
re-run the flat segment breakdown, the mixed-surface sweep, and the
rough-ground sweep; if the regressions go and rough ground holds, make
`velocity` the gz default.

### Environment faults (still stand)

A Gazebo server survived teardown for hours and served a second
`/controller_manager` from its in-process `gz_ros2_control` -- find gz
with `pgrep -f "gz sim"`, since its process name is `ruby`. Stale Fast DDS
segments in `/dev/shm` compound it. `run_surface_sweep.sh` now does both
between worlds. `grep` is unreliable on USD crate files. The earlier
5-8x vertical-acceleration comparison was confounded by mismatched IMU
rates. The per-segment "truth vx/vy" decomposition assumed world-frame
ground-truth twist, which is unverified for either simulator.

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

**Repo:** this one · **File:** `<sensor-ip>-metadata.json` (removed from the tree)

The Ouster driver writes `<sensor-ip>-metadata.json` into whatever directory
the launch was started from, so it landed at the workspace root and was
committed. It carries the lidar's `prod_sn`, `prod_pn`, `image_rev` and
`build_date`, and the filename encodes the sensor subnet.

The working tree and all future commits are handled: the file is deleted and
`*-metadata.json` is in `.gitignore`.

**Not done:** the blob is still reachable in the commit that introduced it, so
it remains visible on GitHub. Removing it means rewriting published history:

```bash
git filter-repo --path <sensor-ip>-metadata.json --invert-paths
git push --force-with-lease origin main
```

That breaks every existing clone, so it is a deliberate call rather than a
cleanup. Weigh it against what is actually exposed: an RFC1918 subnet and a
lidar serial. Deliberately deferred in favour of keeping history linear.
