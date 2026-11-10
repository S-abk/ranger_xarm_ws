# ranger_xarm_description

ROS 2 Jazzy description for the Ranger Mini V3 + xArm6 assembly, generated from the updated Fusion STEP.

## Contents

- Full Ranger Mini CAD visual mesh.
- Full custom static structure visual mesh: pedestal, mounting plate, 8020 gantry, control/battery hardware, laptop stand, and sensor mounts.
- Actual CAD visual meshes for RPLIDAR A1, Ouster OS0 and the RealSense D435 (wrist), plus the vendor RealSense D455 mesh (gantry).
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

## Cameras

Two RealSense units, each with the same frame pattern:

| camera_name | body | parent | `<name>_mount_link` | `<name>_link` |
|---|---|---|---|---|
| `gantry_d455` | D455, lens toward robot +X | `base_link` | CAD part frame of `D455_camera:1` at (-0.319000008, 0, 1.063025), rpy (pi/2, 0, pi/2) | REP-103 camera_link (X forward, Z up) |
| `wrist_d435` | standard D435 | `wrist_d435_plate_link` on `xarm_link_eef` | CAD part frame of the D435, 25.05 mm out along the tool axis and 75 mm off-axis | camera_link, X along the tool axis |

- `<name>_mount_link` is the CAD part frame and the mesh is drawn in it. `<name>_link` is a fixed permutation of it. robot_state_publisher owns both; the RealSense driver (`base_frame_id=link`) owns only the frames below `<name>_link`. See `ranger_xarm_sensors`.
- Printed parts never use `*_mount_link` names. The 140 mm PETG gantry support is part of `custom_structure_link` (`camera_lidar_mount_collision`, a conservative solid box, plus the exact visual mesh `gantry_support_140.stl`). The old 110 mm bracket has been removed from `custom_structure_cad.stl`. The wrist plate is `wrist_d435_plate_link` (exact visual mesh `wrist_d435_plate.stl`, collision = its 80 x 125 x 5.5 mm envelope).
- The wrist plate and camera are fixed children of the flange link, siblings of the upstream gripper. The plate adds no spacer, so flange->gripper and flange->TCP are unchanged, and everything rotates rigidly with joint 6.
- **Wrist roll (confirmed: A).** The CAD fixes the wrist geometry up to a 180 deg roll about the tool axis relative to the gripper root. `wrist_d435_roll_nom` in `calibration.xacro` is `0.0` (candidate A: camera on the gripper root's +X side), confirmed by SAO on 2026-11-10 by visual inspection. The live point-cloud check is still to come. The xacro arg `wrist_d435_roll` stays as an override for later recalibration (`0.0` = A, `3.141592653589793` = B, the -X side; empty = `wrist_d435_roll_nom`); `display.launch.py` passes it through.
- Masses: D455 0.116 kg and D435 0.075 kg nominal (D400 datasheet rev 021). The PETG parts, fasteners and cable carry no inertial because their mass is unknown. The xArm payload settings were not changed.

## Mesh sources and licences

- `realsense_d455.stl`: copied unchanged from `realsense2_description` (`meshes/d455.stl`, ROS 2 Jazzy package, Apache License 2.0, Intel/RealSense). Its frame matches the CAD part frame of the D455 to within 0.1 mm.
- `gantry_support_140.stl` and `wrist_d435_plate.stl` are the project's own PETG camera mounts, tessellated from the project's private CAD (OpenCASCADE BRepMesh, linear deflection 0.05 mm). They are **exact and print-ready**: holes included, each a single closed, manifold solid with outward normals and no open edges. They are in millimetres and positioned in their link frames (`custom_structure_link` = `base_link` for the support, the plate's CAD part frame for the plate), not on a print bed, so re-orient them in the slicer. They are covered by this package's licence (BSD-3-Clause, `package.xml`). The CAD itself is not in the repo.
- All other meshes are exported from the project CAD.

## Mesh units

STEP/STL meshes are stored in millimetres and loaded with URDF mesh scale `0.001 0.001 0.001`.

## Important next validation

The static geometry should now provide the visual reference needed to check xArm yaw, RPLIDAR scan-frame position, OS0/D435 orientation, and the generic laptop envelope. Keep the calibration arguments in the main Xacro for small measured corrections instead of editing nominal CAD values.

## Ground / base_footprint

`base_link` is retained at the Ranger CAD/ROS reference used by all arm and sensor transforms. The exported Ranger visual reaches a lowest wheel-tread point at `z=-0.327028595 m`, so a fixed `base_footprint` frame is published there.

For RViz, set **Fixed Frame** to `base_footprint` (or set the Grid Z offset to `-0.327028595` while keeping `base_link`). Do not shift the robot geometry upward, because that would invalidate the already-validated arm and sensor transforms.
