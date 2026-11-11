# MoveIt test

Plans and executes a fixed set of goals in simulation and checks each planned
path against the true collision model. The question it answers is "does
MoveIt still plan, execute and keep clear of the fixed structure after this
change". For "does the bringup work", use `tools/smoke_test`.

```bash
colcon build --packages-up-to ranger_xarm_gazebo ranger_xarm_isaac
tools/moveit_test/run.sh            # Gazebo (default)
tools/moveit_test/run.sh isaac      # Isaac Sim; needs ISAACSIM_PYTHON_EXE (top-level README)
```

`run.sh` starts these, then runs `moveit_test.py`:

- the simulator with the arm welded to the world;
- its controllers;
- `move_group` from `ranger_xarm_isaac moveit.launch.py` as shipped. This is
  the planning model, with the 5 mm planning margin.
- a reference checker, `ref_move_group.launch.py`. It is the same `move_group`
  configuration at `planning_margin:=0`, which is the true geometry. It runs
  in the `/ref` namespace and never plans or executes.

The test reads both robot descriptions back. It stops (exit 2) if they differ
in anything other than collision box sizes, and it prints the margin it
measured.

It takes about 5 minutes in Gazebo and about 18 in Isaac. Exit status 0 means
every check passed, 1 means a check failed, and 2 means the setup never came
up. There is one line per check. `results.json` and the launch logs go to
`$LOG_DIR`, which defaults to a new temporary directory and is printed at the
end. `moveit_test.py --help` lists its options; pass them after the simulator
name, for example `run.sh gz --only validity`.

## Checks

| Section | Checks |
| --- | --- |
| `validity` | Every SRDF named state (`home`, `hold_up`, gripper `open` and `closed`) is valid on both models. The wrist folded onto the arm is invalid on both. |
| `goals` | 15 goals run in sequence: named states, the gripper, three joint goals, three TCP pose goals, and a joint-6 sweep from −6.2 to +6.2 rad. Each must plan, execute and pass the dense check. |
| `gantry` | Four "near" goals put the wrist camera 1.4–2.8 cm from the gantry or its D455, with every arm link at least 5 mm from the structure. Each goal state must be valid on both models, and each is run as a joint goal and as a pose goal from home. Two "into" goals put the camera inside the gantry. They must be invalid and must not plan. |

**Dense check:** every planned path is sampled so that no joint moves more
than 0.01 rad between samples, and each sample is checked on both models.

- A sample in collision on the true model means the path crosses the real
  structure between the states MoveIt checked. That fails the goal.
- A sample in collision only on the planning model lies inside the margin but
  is still clear of the structure. It is counted, not failed.

**Retries:** a goal gets two planning requests (`--tries`). The "near" goals
sit in a narrow passage, and OMPL occasionally fails one on the first request.

**Returning home:** after every gantry goal, and after any aborted execution,
the arm returns home with MoveIt. A failed return is a failed check. If MoveIt
cannot plan from where the arm stopped, the test drives the arm controller
straight home so the next goal starts clean, and it records why. A typical
cause is an abort in Isaac that leaves the arm in collision or a hair past a
joint limit. `--no-reset` turns this off.

**Simulation only:** `moveit_test.py` sends motion goals, so it refuses to
run unless `/clock` is publishing and advancing. Don't point it at the robot.

## Measured

Measured on 2026-11-11 at the commit that added this tool, on an RTX 2000 Ada laptop:

| Simulator | Result | Failures |
| --- | --- | --- |
| Gazebo | 44/44 passed | none |
| Isaac | 39/44 passed | Three near-gantry pose goals (1.4, 1.8 and 2.8 cm) and two returns home |

The Isaac failures are all execution failures. In each one, 0 path samples
touched the true model:

- **The aborts:** the pose goals planned cleanly. Their long paths (11-13 s)
  aborted during execution because joint 2 fell more than 1 rad behind the
  reference, beyond the controller's state tolerance. The same goals run
  cleanly in Gazebo, so this is the arm's tracking in Isaac, not MoveIt.
- **The returns home:** two of the aborts left the arm in collision, inside
  the laptop envelope and the structure. MoveIt then couldn't plan home from
  there, and the test's controller reset brought the arm back.

## Changing the goals

The goals are joint values in `moveit_test.py`. A "near" goal must satisfy
both of these:

- It is valid with the planning margin and on the true model, which together
  guarantee at least the margin's clearance for every link.
- It sits at least 0.05 rad inside every joint limit. A goal on a limit
  invites a tracking overshoot past it, and MoveIt won't plan out of that.

If the planning margin or the structure changes, check those two conditions
again for every "near" goal.
