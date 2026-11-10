# Migrating from the gantry-D435 configuration

`main` now describes a robot with **two** RealSense cameras: a **D455 on the
gantry** (on a 140 mm PETG support) and a **D435 on the wrist** (on a PETG
plate between the xArm flange and the gripper). The previous configuration,
a single D435 on the 110 mm gantry support, lives on as:

- the tag **`legacy-gantry-d435`**: an exact snapshot of the last commit
  before the change (c2975e8);
- the branch **`legacy/gantry-d435`**: the same configuration, maintained for
  fixes only.

If you pull `main` and your robot still has the old hardware, read this first.

## 1. Hardware the new configuration needs

| Item | Where | Notes |
|---|---|---|
| Intel RealSense **D455** | gantry, on the 140 mm PETG support | replaces the gantry D435; the lens faces robot +X |
| 140 mm PETG gantry support | `custom_structure_link` | printable mesh: [`gantry_support_140.stl`](../src/ranger_xarm_description/meshes/visual/gantry_support_140.stl) |
| Intel RealSense **D435** (standard, not D435i) | wrist, on the PETG plate | rotates with joint 6 and looks out along the tool axis |
| PETG wrist plate | between flange and gripper; adds no spacer | printable mesh: [`wrist_d435_plate.stl`](../src/ranger_xarm_description/meshes/visual/wrist_d435_plate.stl) |
| xArm gripper | | the wrist plate and camera exist only with the gripper (`add_gripper`) |

The meshes are exact and print-ready, in mm, positioned in their link frames.
Re-orient them in the slicer. The wrist camera's roll about the tool axis is
candidate A (`wrist_d435_roll_nom = 0.0`), confirmed by visual inspection.
A live point-cloud check of both cameras' extrinsics is still to come.

## 2. Renamed and new names (old → new)

### TF frames, URDF links and joints

| Old | New |
|---|---|
| `d435_mount_link` | `gantry_d455_mount_link` |
| `d435_link` | `gantry_d455_link` |
| `d435_color_optical_frame`, `d435_depth_optical_frame` (and the other driver frames below `d435_link`) | `gantry_d455_color_optical_frame`, `gantry_d455_depth_optical_frame`, … |
| joint `base_to_d435_mount` | `base_to_gantry_d455_mount` |
| joint `d435_mount_to_link` | `gantry_d455_mount_to_link` |
| (new) | `wrist_d435_plate_link`, `wrist_d435_mount_link`, `wrist_d435_link`, `wrist_d435_*_optical_frame`; joints `flange_to_wrist_d435_plate`, `wrist_d435_plate_to_mount`, `wrist_d435_mount_to_link` |

The gantry camera's mount moved, from (−0.319950, 0, 1.061025) m to
(−0.319000, 0, 1.063025) m in `base_link`; the rotation is unchanged. In the
SRDF, the `custom_structure_link`–`d435_mount_link` disable entry is now
`…–gantry_d455_mount_link`, and the wrist plate and camera get entries for
the parts they are physically attached to.

### Topics

| Old | New |
|---|---|
| `/camera/d435/...` (real driver, Isaac, gz bridge remaps), e.g. `/camera/d435/color/image_raw`, `/camera/d435/depth/image_rect_raw`, `/camera/d435/color/camera_info` | `/camera/gantry_d455/...` and `/camera/wrist_d435/...` with the same suffixes |
| gz sensor topics `/camera/d435/image`, `/camera/d435/depth_image`, `/camera/d435/camera_info` | `/camera/gantry_d455/...` and `/camera/wrist_d435/...`, bridged and remapped as before |

### Launch files and arguments

| Old | New |
|---|---|
| `ranger_xarm_sensors/launch/d435.launch.py` | `ranger_xarm_sensors/launch/d400_camera.launch.py` (`camera_name`, `serial_no`, `device_type` are required; no defaults) |
| `robot.launch.py enable_d435:=…` | `enable_gantry_d455:=…` and `enable_wrist_d435:=…` (both default `true`) |
| `robot.launch.py d435_initial_reset:=…` | `gantry_d455_initial_reset:=…` and `wrist_d435_initial_reset:=…` |
| xacro `d435_dx`, `d435_dy`, `d435_dz` | `gantry_d455_dx`, `gantry_d455_dy`, `gantry_d455_dz` (the old names are no longer read) |
| (new) | xacro and `display.launch.py` arg `wrist_d435_roll` (empty = the model's value; only for recalibration) |

The Gazebo `sim.launch.py` and Isaac `control.launch.py` arguments are unchanged.

### Scripts

| Old | New |
|---|---|
| `ros2 run ranger_xarm_sensors d435_check.py` | `ros2 run ranger_xarm_sensors realsense_check.py --camera gantry_d455` (or `--camera wrist_d435`); `--camera` is required, `--timeout` defaults to 12 s, exit codes unchanged |

## 3. New required serial arguments

The old `robot.launch.py` took no camera serial and the driver bound the
first RealSense it found. Each camera now has its own **required** serial and
no default:

```bash
ros2 launch ranger_xarm_sensors robot.launch.py robot_ip:=<ip> \
    ouster_sensor_hostname:=<host> \
    gantry_d455_serial_no:=<digits> wrist_d435_serial_no:=<digits>
```

- Read the serials with `rs-enumerate-devices`.
- A missing or malformed serial, or the same serial for both cameras, stops
  the launch before anything starts, with an error naming the argument.
- A disabled camera (`enable_gantry_d455:=false` / `enable_wrist_d435:=false`)
  needs no serial.

## 4. What the smoke tests now expect

- `probe.py --sensors` requires the colour image of **both** cameras,
  `/camera/gantry_d455/...` and `/camera/wrist_d435/...`, each reported and
  passing separately. The single "D435 colour image" check is gone.
  `--no-wrist-camera` drops the wrist check for a model without the gripper.
- `probe.py --drive` drives each base segment for a fixed **2.4 s of sim
  time** (from `/clock`), not 4 s of wall time. The percentage of commanded
  travel therefore no longer depends on how fast the host runs the
  simulation. The pass thresholds are unchanged: at least half the commanded
  travel, under 15 cm off-axis. A 40 s wall-time cap per segment is only a
  safety limit. The legacy branch has the same change.
- `gazebo.sh` and `isaac.sh` are unchanged. In Isaac the second camera costs
  about 16 % of the simulation speed. See `tools/smoke_test/README.md`.

## 5. What happens if the old gantry D435 is still attached

This is read from the code; it has **not** been tested on hardware.

- **Without serials:** `robot.launch.py` refuses to start. Its error says
  `gantry_d455 is enabled but required launch argument 'gantry_d455_serial_no'
  was not provided`. The old "bind whatever is plugged in" behaviour is gone.
- **The old D435's serial passed as `gantry_d455_serial_no`:** the gantry
  camera is launched with `device_type` `d455`, a case-insensitive regex on
  the device name, on top of the serial. A D435 does not match it, so the
  driver finds no device. The installed `realsense2_camera` (4.58.4) logs
  *"The requested device with … is NOT found. Will Try again."* and keeps
  retrying, because `wait_for_device_timeout` is left at its default of −1.
  So it fails closed: no gantry image, no gantry depth, and no data published
  with the D455's extrinsics. It does **not** exit; the rest of the bringup
  keeps running without the gantry camera. Watch for that log line.
- **The old D435's serial passed as `wrist_d435_serial_no`:** the model
  filter **cannot** catch this. The wrist filter `d435(?!i)` accepts any
  standard D435, so the old gantry camera would be bound as the wrist camera
  and published with wrist extrinsics. The serial is the binding identity;
  pass the serial of the camera that is actually on the wrist.

## 6. Switching to the legacy line

If your robot still has the single gantry D435:

```bash
cd ~/ranger_xarm_ws
git fetch origin
git checkout legacy/gantry-d435
vcs import --recursive src < ranger_xarm.repos
rm -rf build install log
colcon build
```

Use the tag `legacy-gantry-d435` instead of the branch if you need the exact
last pre-change commit rather than the maintained line. To come back:
`git checkout main`, then run the same `vcs import`, clean and `colcon build`.
