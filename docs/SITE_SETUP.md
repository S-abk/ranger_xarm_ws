# Site setup

Everything in this workspace that is specific to *your* robot and *your*
network, in one place. Nothing here is secret — the values below are
placeholders, and this file is meant to be handed to a new user directly.

Work top to bottom. The verification section at the end tells you what
"working" actually looks like, with real numbers, so you can tell a healthy
bring-up from a quiet failure.

---

## 1. Values you have to supply

| Placeholder | What it is | How to find it | Where it is used |
| --- | --- | --- | --- |
| `<XARM_IP>` | IP of the UFACTORY xArm **controller box**, not the arm | The controller's web UI, or your DHCP leases. `ping` it before launching anything | `robot_ip:=` on every hardware launch |
| `<OUSTER_HOST>` | Hostname or IP of the Ouster OS0 | Ouster web UI at `http://<OUSTER_HOST>`, or mDNS `os-<serial>.local` | `ouster_sensor_hostname:=` |
| `<HOST_IP>` | IP of **this computer** on the sensor network — where the lidar sends UDP | `ip -4 addr show <IFACE>` | `ouster_udp_dest:=` |
| `<IFACE>` | The NIC facing the robot network | `ip -brief addr` | DDS config, UDP buffers |
| `<CAN_DEV>` | CAN interface to the mobile base | `ip link show type can` | `can_device:=`, default `can0` |
| `<RPLIDAR_PORT>` | Serial device for the RPLIDAR A1 | `ls -l /dev/serial/by-id/` | `rplidar_serial_port:=` |

Record yours here when you set the robot up:

```
XARM_IP       = 
OUSTER_HOST   = 
HOST_IP       = 
IFACE         = 
CAN_DEV       = can0
RPLIDAR_PORT  = /dev/ttyUSB0
```

## 2. Host prerequisites

**UDP receive buffers.** Without these the kernel silently drops lidar packets
and the sensor reads as roughly half rate. Symptom: `RcvbufErrors` climbing in
`/proc/net/snmp`.

```bash
sudo sysctl -w net.core.rmem_max=33554432 net.core.rmem_default=33554432
# persist in /etc/sysctl.d/ once confirmed
```

**Real-time scheduling.** Without it `ros2_control` logs
`Could not enable FIFO RT scheduling policy` and you get timing jitter under
trajectory execution. Add your user to a `realtime` group with `rtprio`
limits in `/etc/security/limits.d/`.

**CAN interface**, if you are driving the base:

```bash
sudo ip link set <CAN_DEV> up type can bitrate 500000
ip -details link show <CAN_DEV>     # expect state ERROR-ACTIVE when idle
```

`ERROR-ACTIVE` is the healthy idle state, not a fault.

**Pin the Ouster driver version.** `ros-jazzy-ouster-ros` 0.15.1 aborts on
sensor firmware v3.1.0 with `Field 'WINDOW' not found in LidarScan`
(ouster-lidar/ouster-ros#577). 0.14.1 works. If you are on that firmware:

```bash
sudo apt-mark hold ros-jazzy-ouster-ros ros-jazzy-ouster-sensor-msgs
```

Older debs, when they have aged out of the pool, live at
`http://snapshots.ros.org/jazzy/<snapshot-date>/ubuntu/pool/main/r/`.

## 3. Bring-up, in order

Each rung is a real diagnostic step, not an alternative way to start. If a
rung fails, the next one will fail more confusingly.

| # | Command | Touches hardware? |
| --- | --- | --- |
| 1 | `ros2 launch ranger_xarm_description display.launch.py` | no |
| 2 | `ros2 launch ranger_xarm_moveit_config planning_demo.launch.py` | no |
| 3 | `ros2 launch ranger_xarm_moveit_config fake_execution.launch.py` | no |
| 4 | `ros2 launch ranger_xarm_moveit_config real_hardware_check.launch.py robot_ip:=<XARM_IP>` | reads the arm |
| 5 | `ros2 launch ranger_xarm_moveit_config real_execution.launch.py robot_ip:=<XARM_IP>` | **arm will move** |
| 6 | `ros2 launch ranger_xarm_sensors robot.launch.py robot_ip:=<XARM_IP> ...` | everything |

Full rung 6:

```bash
ros2 launch ranger_xarm_sensors robot.launch.py \
    robot_ip:=<XARM_IP> \
    ouster_sensor_hostname:=<OUSTER_HOST> \
    ouster_udp_dest:=<HOST_IP> \
    can_device:=<CAN_DEV> \
    rplidar_serial_port:=<RPLIDAR_PORT>
```

Every subsystem is an `enable_*` argument, so a partial robot needs no edits:

```bash
ros2 launch ranger_xarm_sensors robot.launch.py robot_ip:=<XARM_IP> \
    enable_base:=false enable_rplidar:=false enable_ouster:=false
```

## 4. Verification — what healthy looks like

Measured on a working system, so treat large deviations as real:

| Check | Command | Expected |
| --- | --- | --- |
| Controllers | `ros2 control list_controllers` | `joint_state_broadcaster` **and** `xarm6_traj_controller` both `active` |
| Arm state rate | `ros2 topic hz /joint_states` | ~150 Hz |
| Arm pose is real | `ros2 topic echo /joint_states --once` | matches where the arm physically is — all zeros usually means it is *not* reading the arm |
| Lidar | `ros2 topic hz /ouster/points` | 10 Hz standalone; 8–9 Hz under full load is contention, not loss |
| Lidar cloud | `ros2 topic echo /ouster/points --once --field width` | 1024 (and height 128 for a 128-beam unit) |
| Camera | `ros2 topic hz /camera/d435/depth/image_rect_raw` | ~15 Hz |
| Sensor mounting | `ros2 run tf2_ros tf2_echo base_footprint os_sensor` | resolves; z ≈ 1.47 m |
| Kernel not dropping | `grep -A1 ^Udp: /proc/net/snmp` | `RcvbufErrors` **not increasing** between samples |

## 5. Things that will catch you

**Exactly one node may own `odom -> base_footprint`.** The Ranger driver does
by default; `publish_odom_tf:=false` hands it to the EKF. With two owners TF
takes whichever arrived last and the symptom looks like a bad sensor.

**`real_hardware_check` validates less than it looks like.** It sets
`add_gripper:=false`, so `xarm_link_tcp` does not exist and MoveIt's actual
kinematic chain is never exercised. Passing that rung does not mean planning
will work.

**In `display.launch.py` there is no `base_footprint`.** It is published at
runtime by `robot.launch.py`, not defined in the URDF. Set RViz's Fixed Frame
to `base_link` or you get an empty view.

**The gripper's initial state is assumed, not measured** —
`xarm_drive_joint = 0.0`. If it is physically elsewhere, MoveIt's model is
wrong until something commands it.

**Killing leftover nodes:** `pkill -x` matches `comm`, which Linux truncates
to 15 characters, so `robot_state_publisher` is really `robot_state_pub` and
will not match. Every Python node reports `comm=python3`. Kill by PID from
`ps -eo pid,args | grep ranger_xarm_ws` instead.

## 6. Safety

Physical motion is opt-in and layered, and a subsystem being *present* is not
the same as it being allowed to move. Before rung 5, have the e-stop in hand
and clear space around the arm — `on_activate` runs `clean_error()`,
`motion_enable(true)`, `set_mode(SERVO)`, `set_state(START)`, so the brakes
release and the arm becomes live before anything is commanded.

The remaining invariants are in the top-level `README.md`. Read them before
adding autonomous motion.
