# Smoke tests

Bring the robot up in simulation, check it does the basic things, shut it
down. They answer "does the bringup work after this change", not "how well
does it track": a check passes at half the commanded travel, and a healthy
run gets 85-94 % (table below). Each drive segment lasts a fixed 2.4 s of *sim*
time (from `/clock`), so the percentage measures the base, not how fast the
host runs the simulation; a 40 s wall-time cap per segment is only a safety
limit.

```bash
colcon build --packages-up-to ranger_xarm_gazebo
tools/smoke_test/gazebo.sh          # or: gazebo.sh arm | gazebo.sh base
```

| Scenario | Launch | Checks |
| --- | --- | --- |
| `arm` | `sim.launch.py` as shipped (base welded to the world) | controllers active, `/joint_states`, `/clock`, arm reaches a pose |
| `base` | `drive_base:=true sensors:=true` | the above, plus Ouster cloud and IMU, `/scan`, the colour image of each camera (gantry D455 and wrist D435, reported separately, both required), and forward / crab / spin-in-place each moving the base along the commanded axis (by `/ground_truth/odom`, with `/odom` beside it) |

In Isaac Sim (about a minute and a half, plus Isaac's first-run shader
compile):

```bash
export ISAACSIM_PYTHON_EXE=...      # see the top-level README
colcon build --packages-up-to ranger_xarm_isaac
tools/smoke_test/isaac.sh
```

This generates the wheeled USD into the log directory, so the one in
`install/` is left alone. It then starts `isaac_bringup.py --headless` and
`control.launch.py drive_base:=true`, and runs the `base` checks.

The real-time factor depends on the machine and is information, not a pass
criterion. Measured values:

| Machine (gantry-D435 model) | Simulator | Real-time factor | Forward / crab / spin |
| --- | --- | --- | --- |
| RTX 4080 laptop | Isaac | about 0.65 | (measured with the old 4 s wall-time window) |
| RTX 2000 Ada laptop | Isaac | 0.29-0.31 | 93 / 85 / 88 % |
| RTX 2000 Ada laptop | Gazebo | 0.88 | 94 / 90 / 94 % |

On this configuration (gantry D455 and wrist D435):

| Machine | Simulator | Real-time factor | Forward / crab / spin |
| --- | --- | --- | --- |
| RTX 2000 Ada laptop | Isaac | 0.24-0.25 | 93 / 89 / 90 % |
| RTX 2000 Ada laptop | Gazebo | 0.76 | 94 / 89 / 95 % |

In Isaac the second camera costs about 16 % of the simulation speed: mean
real-time factor 0.296 with one camera and 0.248 with two, three runs each on
the same machine. Physics and rendering step together, so every extra render
product lengthens each step. Since the drive window is in sim time, this no
longer lowers the drive percentages.

Exit status 0 means every check passed, 1 means a check failed, and 2 means
the simulation never came up. One line per check is printed, and the launch
logs' directory is printed at the end. Each script takes
about two minutes.

Each run uses its own ROS domain (`SMOKE_DOMAIN_ID`, default 42), keeps
discovery on this machine and uses its own gz partition, so it can run
beside another simulation or a robot session without either seeing the
other.

Both cameras must publish for `--sensors` to pass; `--no-wrist-camera` drops
the wrist D435 check for a model without the gripper plate (`add_gripper:=false`).

`probe.py` is simulator-agnostic: point it at anything already running
(`probe.py --drive --sensors --arm`) and it runs the same checks.
