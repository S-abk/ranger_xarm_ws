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

## Resolved: what the EKF buys on rough ground, and what it cannot

**Measured:** `rough_ground.sdf` (101 staggered 24 mm bumps), five trials from
a teleported fixed start, `sensors:=true odom_tf:=false`.

| estimator | pos err (% of path) | net yaw err |
| --- | --- | --- |
| wheel odometry | 42 - 237 % | 59 - 180 deg |
| EKF (odom + gyro) | 8.6 - 31 % | <= 0.4 deg |

On flat ground the two are indistinguishable (0.10 % vs 1.39 %), so rough
ground is the only place the filter earns its keep. It does, decisively, but
only on heading.

A per-segment probe says why. Longitudinal traction over the bumps is
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
