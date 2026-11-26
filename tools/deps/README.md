# deps

Where this workspace's packages come from, and which ones are missing
before you launch. One script, `deps.py`. It reads files only: launch files
are parsed, never run; nothing joins a ROS graph or opens a device.

## Before launching: `check`

With the environment you will launch from sourced (ROS, any driver
workspaces, then this one):

```bash
python3 tools/deps/deps.py check hardware   # robot.launch.py, base_bringup, ekf_odom_imu, real_hardware_check
python3 tools/deps/deps.py check sim        # sim.launch.py, display, planning_demo, fake_execution
python3 tools/deps/deps.py check isaac      # ranger_xarm_isaac control and moveit
python3 tools/deps/deps.py check ranger_xarm_bringup ekf_odom_imu.launch.py   # any one launch file
```

From each launch file it follows the launch files of this workspace that it
includes, and lists every package, executable and included file they name,
the run-time dependencies in the manifests of this workspace's packages
whose launch files it visited, and the pluginlib classes (ros2_control
hardware and controllers, MoveIt plugins) in their YAML and xacro
arguments. Every one missing from the sourced environment is printed with
what needs it, and the exit status is 1. A missing item that a launch
condition guards says so (`when IfCondition(lc('enable_rplidar'))`): a
subsystem you turn off does not need it at launch, although a manifest may
still name it.

It also prints which workspace each package outside the ROS install was
found in, and warns about any package present in more than one workspace
of the chain: whichever comes first on the path is used, and a stale copy
there has picked up the wrong package before.

A reference it cannot read statically (a package name or path built at
launch time) is printed as `not checked`.

## For maintainers: `report`

In a checkout with `ranger_xarm.repos` (and, for the hardware rows,
`ranger_xarm_hardware.repos`) imported into `src/`:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/deps/deps.py report
```

A table of every package that a manifest, launch file or xacro in `src/`
names and that is not in this repository: the `.repos` file and commit it is
pinned at (marked **PROVISIONAL** where the pin says so), or the apt package
rosdep resolves it to. It exits 1 if any package comes from nowhere. Its
rosdep column depends on the machine's rosdep sources.
