# ranger_xarm_moveit_config

MoveIt 2 configuration for the validated Ranger Mini V3 + xArm6 robot description on ROS 2 Jazzy: planning only (`planning_demo.launch.py`), mock execution (`fake_execution.launch.py`) and the real arm (`real_hardware_check.launch.py`, `real_execution.launch.py`).

## Design choices

- The full Ranger/custom geometry stays inside `robot_description`, so MoveIt sees the chassis, pedestal, gantry, sensors, and default laptop as robot collision geometry.
- Planning group `xarm6` is the chain from `xarm_link_base` to `xarm_link_tcp`.
- The xArm gripper is a separate `xarm_gripper` group driven by `xarm_drive_joint`; its mimic joints remain passive.
- `planning_demo.launch.py` sets `allow_trajectory_execution: false`, to validate planning and collision avoidance without commanding anything; `fake_execution.launch.py` and `real_execution.launch.py` set it `true`.
- OMPL RRTConnect is the intended first planner; PRM and RRT* are also exposed.
- KDL provides the first IK implementation. We can swap to a faster/analytic solver later without changing the robot description.

## Install/build

This package is built as part of the workspace: follow the quick start in the top-level `README.md`, which resolves its dependencies with rosdep. Note that `ros-jazzy-moveit` alone does not include `moveit_ros_perception`, which `real_execution.launch.py` needs for the lidar octomap; rosdep installs it from this package's manifest.

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

## Planning margin

Every MoveIt launch here (and `ranger_xarm_isaac moveit.launch.py`) gives `move_group` a description in which each fixed-structure collision box is grown by `planning_margin` per side: the chassis envelope, every `custom_structure_link` box, the RPLIDAR, Ouster and gantry D455 boxes, and the laptop. The default is `planning_margin:=0.005` (5 mm). The arm, gripper, wrist plate, wrist camera and wheels are not inflated, and visuals never change.

Why: MoveIt checks discrete states, so a planned path can graze the structure between checked states. In simulation, 2–5 % of planned paths near the gantry grazed by up to about 2 mm, and neither `longest_valid_segment_fraction` nor `totg.path_tolerance` removed this. `default_robot_padding` does not apply either: it pads robot-vs-world checks only, and the structure is part of this URDF.

`robot_state_publisher`, ros2_control, Gazebo and Isaac keep the true geometry (`planning_margin` defaults to 0 in the xacro). For that reason `move_group` no longer publishes `/robot_description`; RViz in these launches is given the planning description, so the collision display matches what is planned against.

```bash
ros2 launch ranger_xarm_moveit_config planning_demo.launch.py planning_margin:=0.0   # true geometry
```

## Laptop variations

The default 15-inch laptop remains enabled. You can test another lid angle without editing the model:

```bash
ros2 launch ranger_xarm_moveit_config planning_demo.launch.py laptop_open_angle_deg:=100.0
```

Or temporarily remove the laptop from the robot collision model:

```bash
ros2 launch ranger_xarm_moveit_config planning_demo.launch.py use_laptop_collision:=false
```

## Execution

`real_execution.launch.py` executes through UFACTORY's Jazzy ros2_control hardware interface and trajectory controllers for the real xArm. The Ranger driver remains a separate hardware/control stack that shares the same TF model rather than owning this MoveIt description.

---

## Fake trajectory execution

Once planning is validated, launch the ros2_control mock system:

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

## Real hardware

Use `real_hardware_check.launch.py` first, then `real_execution.launch.py`. Both require `robot_ip:=...`. The real hardware plugin is `uf_robot_hardware/UFRobotSystemHardware`. See `docs/SITE_SETUP.md` for the staged procedure.
