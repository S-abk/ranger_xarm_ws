# Smoke tests

Bring the robot up in simulation, check it does the basic things, shut it
down. They answer "does the bringup work after this change", not "how well
does it track": a check passes at half the commanded travel, and a healthy
run gets 80-96 %.

```bash
colcon build --packages-up-to ranger_xarm_gazebo
tools/smoke_test/gazebo.sh          # or: gazebo.sh arm | gazebo.sh base
```

| Scenario | Launch | Checks |
| --- | --- | --- |
| `arm` | `sim.launch.py` as shipped (base welded to the world) | controllers active, `/joint_states`, `/clock`, arm reaches a pose |
| `base` | `drive_base:=true sensors:=true` | the above, plus Ouster cloud and IMU, `/scan`, D435 image, and forward / crab / spin-in-place each moving the base along the commanded axis (by `/ground_truth/odom`, with `/odom` beside it) |

In Isaac Sim (about a minute and a half, plus Isaac's first-run shader
compile):

```bash
export ISAACSIM_PYTHON_EXE=...      # see the top-level README
colcon build --packages-up-to ranger_xarm_isaac
tools/smoke_test/isaac.sh
```

This generates the wheeled USD into the log directory, so the one in
`install/` is left alone. It then starts `isaac_bringup.py --headless` and
`control.launch.py drive_base:=true`, and runs the `base` checks. Expect
80-92 % here at a real-time factor of about 0.6.

Exit status 0 means every check passed, 1 means a check failed, and 2 means
the simulation never came up. One line per check is printed, and the launch
logs' directory is printed at the end. Each script takes
about two minutes.

Each run uses its own ROS domain (`SMOKE_DOMAIN_ID`, default 42), keeps
discovery on this machine and uses its own gz partition, so it can run
beside another simulation or a robot session without either seeing the
other.

`probe.py` is simulator-agnostic: point it at anything already running
(`probe.py --drive --sensors --arm`) and it runs the same checks.
