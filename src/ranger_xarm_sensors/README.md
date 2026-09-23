# ranger_xarm_sensors

Onboard sensor bringup for the Ranger Mini + xArm6 platform.

Validated/target hardware:
- SLAMTEC RPLIDAR A1M8 -> `/scan`, frame `laser_frame`
- Intel RealSense D435 -> `/camera/d435/...`, root frame `d435_mount_link`
- Ouster OS0 -> later phase

The robot-level mount frames come from `ranger_xarm_description`. Sensor drivers own only their internal sensor frame trees.

## D435 design

The RealSense driver is launched with `camera_name=d435` and `base_frame_id=mount_link`, so its root frame is exactly `d435_mount_link`, which already exists in the robot URDF at the CAD-derived pose. This avoids a duplicate robot-to-camera static transform.

Point cloud and depth alignment are intentionally disabled for the initial D435 validation. Color/depth are set to 640x480x15. Point cloud remains off, and the operator RViz image displays are disabled by default to keep remote bandwidth controlled.

## Phase 6C reduced Ouster preview cloud

`ouster_points_viz` subscribes to the full local `/ouster/points` cloud and
publishes `/ouster/points_viz` using deterministic point decimation plus a
publication-rate divisor.  The default unified-launch settings are:

- keep every 16th point
- publish every 2nd incoming cloud

For an OS0-128 at 1024x10 that measured about 63 MB/s on `/ouster/points`, the
nominal payload target is about 2 MB/s at about 5 Hz.  The full cloud remains
available onboard for autonomy; use the reduced topic for remote RViz.

## Optional hand skimmer

The unified Phase 6C launch can declare the manually clamped hand skimmer as a
MoveIt AttachedCollisionObject:

```bash
ros2 launch ranger_xarm_sensors robot.launch.py \
  robot_ip:=192.168.1.221 \
  skimmer_attached:=true
```

When enabled, the launch automatically uses `skimmer_gripper_position:=0.849`
for the ROS gripper model only. It does not command physical gripper motion.
CAD registration can be fine-tuned with the `skimmer_attach_*` arguments.
