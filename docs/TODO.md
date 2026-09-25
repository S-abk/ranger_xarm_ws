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
