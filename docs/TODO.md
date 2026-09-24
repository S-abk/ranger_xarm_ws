# TODO

Outstanding items found while bringing this workspace up in simulation.
Each one names the repo it applies to, because some belong to
`S-abk/Ranger_xarm` (private) rather than here.

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

## Purge the leaked Ouster metadata from `ranger_xarm_ws` history (public)

**Repo:** this one · **File:** `192.168.1-metadata.json` (workspace root)

The Ouster driver writes `<sensor-ip>-metadata.json` into whatever directory the
launch was started from, so it landed at the workspace root and was committed in
`b1489de`. It contains the lidar's `prod_sn`, `prod_pn`, `image_rev` and
`build_date`, and the filename encodes the sensor subnet.

Working tree and future commits are handled: the file is deleted and
`*-metadata.json` is now in `.gitignore`, matching the pattern `Ranger_xarm`
already uses.

**Still outstanding:** deleting the file does *not* remove it from published
history — the blob is still reachable in `b1489de` on GitHub. Actually purging it
needs a history rewrite and force push:

```bash
git filter-repo --path 192.168.1-metadata.json --invert-paths
git push --force-with-lease origin main
```

That rewrites shared history and breaks every existing clone, so it is a
deliberate decision, not a cleanup. Weigh it against what is actually exposed: an
RFC1918 subnet and a lidar serial. If the repo has few or no other clones, doing
it is cheap; if not, it may not be worth the disruption.
