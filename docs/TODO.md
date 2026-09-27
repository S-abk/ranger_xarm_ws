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

## The EKF does not track heading against the simulator

Not a verdict on the EKF. A record of where the investigation stopped, so
the next person does not repeat the ruling-out.

Symptom: with gz on `empty_ground`, `sensors:=true`, `odom_tf:=false` and
`ranger_xarm_bringup/ekf_odom_imu.launch.py use_sim_time:=true`, over a
drive with 138.0 deg of real rotation:

```
wheel odometry    0.007 m   0.11% of path    +0.17 deg
EKF (odom+gyro)   5.799 m  86.76% of path  -138.00 deg
```

The EKF's orientation stays exactly identity and its reported yaw rate
stays exactly 0.0. `-138.00` against `+138.0` is not drift, it is the
filter never rotating at all. Wheel odometry on the same run is 0.11%, so
`/odom` is not the problem.

Ruled out, each checked directly:

- TF. `base_footprint -> os_imu` resolves.
- Frame id. The IMU publishes `os_imu`, which is that frame.
- Covariance. `angular_velocity_covariance[8]` is 1e-05, not the -1 that
  would mean "unavailable".
- Timestamps. Sim clock, IMU, corrector and odom stamps all agree, and
  `use_sim_time` is true on both the corrector and the EKF.
- Config indices. `imu0_config[11]` is vyaw and `odom0_config[6,7]` are
  vx, vy, which is the intended split: wheels for speed, gyro for
  turning.
- Gyro data. `/ouster/imu` reads 1.36 rad/s during a commanded spin, so
  the measurement exists.

Still unexplained: why a valid, correctly framed, correctly stamped vyaw
measurement with a sane covariance produces no yaw at all.

**Separately, and this one IS a defect.** The corrector seeds a
gyro-z bias of +0.3350 deg/s, measured on hardware, as its starting
value. The simulated IMU has exactly zero bias, so the corrector injects
a phantom -0.335 deg/s, and then its own sanity check refuses every
re-estimate:

```
seeded gyro-z bias +0.3350 deg/s
rejecting gyro bias jump of -0.3350 deg/s (limit 0.2005); robot may not
have been truly stationary
```

`max_bias_step` is 0.0035 rad/s, so the correction it needs (0.335) is
larger than the step it will accept (0.2005) and it can never converge.
The guard that protects it from a bad stationary sample on hardware locks
it onto the wrong value permanently against any IMU whose bias differs
from the seed by more than 0.2 deg/s. That is worth fixing regardless of
the yaw question, and it is a reason not to treat the seed as harmless.

Both of these were only findable once `/odom` stopped being ground truth.

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
