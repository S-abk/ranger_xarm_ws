# ranger_xarm_moveit_config — phase 3

Planning-only MoveIt 2 configuration for the validated Ranger Mini V3 + xArm6 robot description on ROS 2 Jazzy.

## Design choices

- The full Ranger/custom geometry stays inside `robot_description`, so MoveIt sees the chassis, pedestal, gantry, sensors, and default laptop as robot collision geometry.
- Planning group `xarm6` is the chain from `xarm_link_base` to `xarm_link_tcp`.
- The xArm gripper is a separate `xarm_gripper` group driven by `xarm_drive_joint`; its mimic joints remain passive.
- The initial phase intentionally sets `allow_trajectory_execution: false`. This lets us validate planning and collision avoidance without commanding hardware.
- OMPL RRTConnect is the intended first planner; PRM and RRT* are also exposed.
- KDL provides the first IK implementation. We can swap to a faster/analytic solver later without changing the robot description.

## Install/build

Place this package beside `ranger_xarm_description` in `~/ros2_ws/src`.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ranger_xarm_description ranger_xarm_moveit_config
source install/setup.bash
```

If MoveIt is not already installed, rosdep should pull the package dependencies. The typical binary install is also:

```bash
sudo apt update
sudo apt install ros-jazzy-moveit
```

## Launch the planning-only demo

Do not run the earlier `display.launch.py` at the same time; this launch owns its own robot_state_publisher, joint_state_publisher_gui, RViz, and move_group.

```bash
ros2 launch ranger_xarm_moveit_config planning_demo.launch.py
```

The RViz fixed frame is `base_footprint`, but MoveIt plans the xArm relative to its installed `xarm_link_base` chain while checking collisions against the entire combined model.

## First tests

1. Confirm RViz opens and the MotionPlanning display shows planning group `xarm6`.
2. Confirm the robot is not red/in collision at its initial state.
3. Drag the interactive goal marker to a reachable pose in front of the robot and press **Plan** (not Execute).
4. Try placing the goal behind/through the laptop or gantry. A valid plan must route around the obstacles or fail cleanly.
5. If the initial state is reported in collision, inspect the reported link pair before disabling anything. The SRDF only pre-disables known adjacent xArm pairs and fixed mounted-object overlaps.

## Useful diagnostics

```bash
ros2 topic echo /monitored_planning_scene --once
ros2 node info /move_group
ros2 param get /move_group robot_description_semantic
```

To inspect whether MoveIt regards the initial state as colliding, the RViz MotionPlanning display's **Scene Robot / Show Robot Collision** option is useful.

## Laptop variations

The default 15-inch laptop remains enabled. You can test another lid angle without editing the model:

```bash
ros2 launch ranger_xarm_moveit_config planning_demo.launch.py laptop_open_angle_deg:=100.0
```

Or temporarily remove the laptop from the robot collision model:

```bash
ros2 launch ranger_xarm_moveit_config planning_demo.launch.py use_laptop_collision:=false
```

## Next phase

Once collision-aware planning is validated, phase 4 should replace `allow_trajectory_execution: false` with UFACTORY's Jazzy ros2_control/trajectory controller path for the real xArm. The Ranger driver should remain a separate hardware/control stack that shares the same `base_link`/TF model rather than owning this MoveIt description.

---

## Phase 4A: fake trajectory execution

After Phase 3 planning has been validated, launch the ros2_control mock system:

```bash
ros2 launch ranger_xarm_moveit_config fake_execution.launch.py
```

This uses `mock_components/GenericSystem`, `joint_state_broadcaster`, and a
`JointTrajectoryController` named `xarm6_traj_controller`. MoveIt connects via
`FollowJointTrajectory` through `moveit_simple_controller_manager`.

Check:

```bash
ros2 control list_controllers
ros2 action list | grep follow_joint_trajectory
```

Then use **Plan** followed by **Execute** in RViz. The arm should animate and
`/joint_states` should change, while no physical robot is contacted.

## Phase 4B real hardware

Use `real_hardware_check.launch.py` first, then `real_execution.launch.py`. Both require `robot_ip:=...`. The real hardware plugin is `uf_robot_hardware/UFRobotSystemHardware`. See the bundle-level `PHASE4B_INSTALL.md` for the staged procedure.
