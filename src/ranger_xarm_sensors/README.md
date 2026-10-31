# ranger_xarm_sensors

Onboard sensor bringup for the Ranger Mini + xArm6 platform.

Validated/target hardware:
- SLAMTEC RPLIDAR A1M8 -> `/scan`, frame `laser_frame`
- Intel RealSense D455 (gantry, lens facing +X) -> `/camera/gantry_d455/...`
- Intel RealSense D435 (xArm wrist, on the flange plate) -> `/camera/wrist_d435/...`
- Ouster OS0 -> later phase

The robot-level mount frames come from `ranger_xarm_description`. Sensor drivers own only their internal sensor frame trees.

## RealSense cameras

Both cameras start through `launch/d400_camera.launch.py`, which `robot.launch.py`
includes once per enabled camera. Per camera:

| | gantry | wrist |
| --- | --- | --- |
| `camera_name` | `gantry_d455` | `wrist_d435` |
| `device_type` | `d455` | `d435(?!i)` (a D435i does not match) |
| enable | `enable_gantry_d455` (default `true`) | `enable_wrist_d435` (default `true`) |
| serial argument | `gantry_d455_serial_no` | `wrist_d435_serial_no` |
| initial reset | `gantry_d455_initial_reset` | `wrist_d435_initial_reset` |

`color_profile` and `depth_profile` (default `640x480x15`) are shared. Topics are
`/camera/<camera_name>/color/image_raw`,
`/camera/<camera_name>/depth/image_rect_raw`, the matching `camera_info`, and so on.
Point cloud, depth alignment, infra and IMU streams are off. The operator RViz image
displays are off by default to keep remote bandwidth controlled.

### TF

`robot_state_publisher` owns `<camera_name>_mount_link` (the CAD part frame, Z out of
the lens) and `<camera_name>_link` (the semantic ROS camera frame: X forward, Y left,
Z up). The driver is launched with `camera_namespace=camera`, `camera_name=<name>` and
`base_frame_id=link`, so its root frame is `<camera_name>_link`, and it publishes only
the sensor and optical frames below it (`<camera_name>_color_optical_frame`,
`<camera_name>_depth_optical_frame`, ...). Each frame has one publisher. Do not root
the driver at `<camera_name>_mount_link`: that is the Z-forward CAD frame, and rooting
there would rotate every optical frame.

### Serial numbers: required, no default

Each camera has its own serial argument and it has **no default**. With two cameras on
one robot, "the first device found" could bind either one to the wrong frames, so the
launch never does that. If an enabled camera has no serial, or the serial is empty,
not all digits, or the same as the other camera's, `robot.launch.py` stops with an
error naming the argument **before** MoveIt, the arm or the base start.

```bash
ros2 launch ranger_xarm_sensors robot.launch.py robot_ip:=<ip> ouster_sensor_hostname:=<host> \
    gantry_d455_serial_no:=<digits> wrist_d435_serial_no:=<digits>
```

- With `enable_gantry_d455:=false` the gantry serial is not needed, and likewise for
  the wrist. Everything else still launches.
- The two serial arguments are not `DeclareLaunchArgument`s: one without a default
  would fail the whole launch even when its camera is off. They are read only for an
  enabled camera, so they do not appear in `--show-args`; this list is where they are
  documented.
- Digits only. A leading `_` is accepted and ignored. The launch always hands the
  driver `_<digits>`: ROS 2 would turn a bare all-digit parameter into an integer and
  lose a leading zero, and realsense2_camera 4.58.4 removes one leading `_`
  (`realsense_node_factory.cpp:337`).
- The serial is the binding identity. `device_type` is a secondary filter: a serial
  that belongs to the wrong model finds no device rather than the wrong one.

To find the serials, a person runs `rs-enumerate-devices` on the robot with both cameras
plugged in (it is not part of any launch or test). Use the `Serial Number` line, not
`Asic Serial Number`, and match each to its model name.

### Checking a camera

`ros2 run ranger_xarm_sensors realsense_check.py --camera gantry_d455` (or
`wrist_d435`) waits for the colour and depth images and CameraInfo and checks the
optical frame ids. It needs the camera running. USB load with both cameras at once is
untested.

### Tests

`colcon test --packages-select ranger_xarm_sensors` walks the launch descriptions
without starting a process (no hardware, no ROS graph): serial validation, the
`_<digits>` hand-off, the per-camera model filter, and that a camera that is off needs
no serial.

## Reduced Ouster preview cloud

`ouster_points_viz` subscribes to the full local `/ouster/points` cloud and
publishes `/ouster/points_viz` using deterministic point decimation plus a
publication-rate divisor.  The default unified-launch settings are:

- keep every 16th point
- publish every 2nd incoming cloud

For an OS0-128 at 1024x10 that measured about 63 MB/s on `/ouster/points`, the
nominal payload target is about 2 MB/s at about 5 Hz.  The full cloud remains
available onboard for autonomy; use the reduced topic for remote RViz.

