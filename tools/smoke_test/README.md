# Smoke tests

Bring the robot up in simulation, check it does the basic things, shut it
down. They answer "does the bringup work after this change", not "how well
does it track": a check passes at half the commanded travel, and a healthy
run gets 85-94 % (table below). Each drive segment lasts a fixed 2.4 s of *sim*
time (from `/clock`), so the percentage measures the base, not how fast the
host runs the simulation; a 40 s wall-time cap per segment is only a safety
limit.

```bash
colcon build --packages-up-to ranger_xarm_gazebo ranger_xarm_isaac
tools/smoke_test/gazebo.sh          # or: gazebo.sh arm | gazebo.sh base
```

`ranger_xarm_isaac` is needed even without Isaac: the `arm` scenario's MoveIt
checks use its `moveit.launch.py`, and fail with "no move_group" without it.
The README's quick start builds both.

| Scenario | Launch | Checks |
| --- | --- | --- |
| `arm` | `sim.launch.py` as shipped (base welded to the world), plus `ranger_xarm_isaac`'s `moveit.launch.py` | controllers active, `/joint_states`, `/clock`, arm reaches a pose, MoveIt plans and executes a joint goal and a pose goal |
| `base` | `drive_base:=true sensors:=true` | controllers active, `/joint_states`, `/clock` and the arm move as in `arm` (no MoveIt), plus Ouster cloud and IMU, `/scan`, the colour image of each camera (gantry D455 and wrist D435, reported separately, both required), and forward / crab / spin-in-place each moving the base along the commanded axis (by `/ground_truth/odom`, with `/odom` beside it) |

In Isaac Sim (about a minute and a half, plus Isaac's first-run shader
compile):

```bash
export ISAACSIM_PYTHON_EXE=...      # see the top-level README
colcon build --packages-up-to ranger_xarm_isaac
tools/smoke_test/isaac.sh
```

This generates the wheeled USD into the log directory, so the one in
`install/` is left alone. It then starts `isaac_bringup.py --headless` and
`control.launch.py drive_base:=true` and `moveit.launch.py`, and runs the
`base` checks plus the MoveIt ones.

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

## Obstacle map with the base driven

```bash
tools/smoke_test/obstacle_map.sh          # or: obstacle_map.sh mobile | welded | free
```

**What it does.** Gazebo comes up with `drive_base:=true sensors:=true`, plus MoveIt with the lidar obstacle map.
1. Before any obstacle: an arm goal J* to the robot's right (tool point about 0.6 m out) must plan.
2. A 0.20 m box is placed on J*'s tool point.
3. The base drives closed-loop on `/odom`: 1 m back, back to the start, 0.8 m left, back again.
4. While moving, the planning scene is stored every 0.5 s and decoded after the base stops.

**Tolerances** (`obstacle_probe.py`). They were fixed from the measurements that chose this design and are not loosened to make a run pass.

| | after each leg | tolerance |
| --- | --- | --- |
| T1 | box cells' centroid moved, at rest | ≤ 0.031 m |
| T2 | box cells vs the first map, at rest | ≥ 50 % |
| T3 | before the box: J* plans (up to 3 attempts; every failed attempt is printed with its start-state and J* contacts); J* with the box in the way: invalid with an `<octomap>` contact, plan refused; once moved on: valid | as stated |
| T4 | lidar points in the box; odometry drift; every leg reached, sampled in motion, every MoveIt query answered (else INCONCLUSIVE) | ≥ 100; ≤ 0.02 m |
| T5 | cells where the box would be if the map moved with the robot | ≤ 10 % |
| T6 | worst in-motion count of cells 0.1–1.5 m high off the box | ≤ 50 |

The `mobile` variant also checks that a planned trajectory names the six arm joints only and executes.

**Variants.**
- `mobile`: `moveit.launch.py drive_base:=true obstacle_map:=true`, the shipped design. Must pass everything.
- `welded`: `obstacle_map:=true` without `drive_base`, as before this change. MoveIt's planning frame is missing from TF, so no map builds. A negative control: it must fail T3.
- `free`: `moveit.launch.py drive_base:=true obstacle_map:=true planning_frame:=base_footprint`, the selectable alternative that plans on the base, so the map moves with the robot. A negative control: it must fail T6. This is expected of that choice while driving, not a fault in it.

If a control passes what it must fail, the check reports that it cannot tell, and fails.

**Running it.**
- About three and a half minutes per variant.
- `SMOKE_GPU=nvidia` renders on the NVIDIA card.
- `SMOKE_DOMAIN_ID` works as in `gazebo.sh`.
- Exit status: 0 as expected, 1 not, 2 no simulation, 3 inconclusive.

`probe.py` is simulator-agnostic: point it at anything already running
(`probe.py --drive --sensors --arm --moveit`) and it runs the same checks.
`--moveit` needs `move_group` running (`ranger_xarm_isaac moveit.launch.py`);
it plans and executes through the MoveGroup action, so the collision model
(wrist plate and D435, gantry D455 and its support), the SRDF and the
controller map are all exercised. A joint goal passes within 2 degrees, a
pose goal within 10 mm of the target TCP position.
