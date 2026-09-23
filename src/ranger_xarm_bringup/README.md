# ranger_xarm_bringup — Phase 5 onboard package

Runs on the laptop mounted on the Ranger.

## Owns

- Ranger CAN driver
- dynamic `odom -> base_link`
- `robot_state_publisher`
- onboard `/cmd_vel_remote -> /cmd_vel` deadman relay

The Phase 5 launch intentionally does **not** run RViz or keyboard teleop onboard.

```bash
ros2 launch ranger_xarm_bringup base_bringup.launch.py can_device:=can0
```

Remote velocity commands should be sent to `/cmd_vel_remote`. The onboard relay publishes `/cmd_vel` at 50 Hz and commands zero if remote commands stop arriving for 0.35 s.

For this base-only validation `use_xarm:=false` is the default. Later combined bringup will use the validated physical xArm stack instead of inventing arm joint state.
