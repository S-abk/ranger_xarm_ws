# ranger_xarm_description — phase 2

ROS 2 Jazzy description for the Ranger Mini V3 + xArm6 assembly, generated from the updated Fusion STEP.

## Added in phase 2

- Full Ranger Mini CAD visual mesh.
- Full custom static structure visual mesh: pedestal, mounting plate, 8020 gantry, control/battery hardware, laptop stand, and sensor mounts.
- Actual CAD visual meshes for RPLIDAR A1, Ouster OS0 and RealSense D435.
- Conservative primitive collision boxes separate from high-detail visuals.
- Corrected xArm mounting height: `z = 0.391225 m`.
- Confirmed 180-degree laptop yaw correction.
- Default-on 15-inch laptop collision envelope.
- Launch file permanently uses `ParameterValue(..., value_type=str)` for `robot_description` on Jazzy.

## Replace your phase-1 package

Copy this directory to:

```bash
~/ros2_ws/src/ranger_xarm_description
```

Then rebuild:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rm -rf build/ranger_xarm_description install/ranger_xarm_description
colcon build --symlink-install --packages-select ranger_xarm_description
source install/setup.bash
```

Launch:

```bash
ros2 launch ranger_xarm_description display.launch.py use_xarm:=true
```

In RViz add `RobotModel` and `TF`, and keep Fixed Frame = `base_link`.

To inspect the visual model without conservative collision boxes:

```bash
ros2 launch ranger_xarm_description display.launch.py use_xarm:=true use_static_collision:=false
```

To inspect collision geometry, enable `Collision Enabled` under RobotModel in RViz.

## Mesh units

STEP/STL meshes are stored in millimetres and loaded with URDF mesh scale `0.001 0.001 0.001`.

## Important next validation

The static geometry should now provide the visual reference needed to check xArm yaw, RPLIDAR scan-frame position, OS0/D435 orientation, and the generic laptop envelope. Keep the calibration arguments in the main Xacro for small measured corrections instead of editing nominal CAD values.

## Ground / base_footprint

`base_link` is retained at the Ranger CAD/ROS reference used by all arm and sensor transforms. The exported Ranger visual reaches a lowest wheel-tread point at `z=-0.327028595 m`, so a fixed `base_footprint` frame is published there.

For RViz, set **Fixed Frame** to `base_footprint` (or set the Grid Z offset to `-0.327028595` while keeping `base_link`). Do not shift the robot geometry upward, because that would invalidate the already-validated arm and sensor transforms.
