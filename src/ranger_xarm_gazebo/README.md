# ranger_xarm_gazebo

Gazebo (gz) simulation of the platform.

```bash
ros2 launch ranger_xarm_gazebo sim.launch.py
ros2 launch ranger_xarm_gazebo sim.launch.py headless:=true
ros2 launch ranger_xarm_gazebo sim.launch.py world:=/abs/path/to/your.sdf
```

## How it hooks up

`ranger_xarm_description` carries **one** URDF, used by both simulation and
hardware. Simulation flips three xacro arguments and changes nothing else:

| Argument | Hardware | Simulation |
| --- | --- | --- |
| `xarm_ros2_control_plugin` | `uf_robot_hardware/UFRobotSystemHardware` | `gz_ros2_control/GazeboSimSystem` |
| `xarm_load_gazebo_plugin` | `false` | `true` |
| `xarm_ros2_control_params` | *(empty)* | generated controller yaml |

All three default to the hardware values, so nothing changes for the real
robot unless a launch file asks. Keeping one description is the point: a
separate simulation URDF drifts, and the drift is invisible until a plan that
worked in simulation collides on the robot.

The controller parameters are produced at launch from
`xarm_controller/config/xarm6_controllers.yaml` through upstream's
`generate_ros2_control_params_temp_file`, which rewrites it for our `xarm_`
prefix and stamps `use_sim_time`. That helper is reused rather than forked so
the config cannot silently age against the upstream it came from.

## Two things worth knowing before you edit this

**The model is spawned from the `/robot_description` topic, not from a file.**
So gz gets byte-for-byte what `robot_state_publisher` is using. Spawning from
a separately expanded file is how the TF tree and the physics model drift
apart.

**Controller spawning is chained to the spawn finishing.** The
`controller_manager` the spawners talk to is created by the plugin *inside the
spawned model*, so it does not exist until gz has the model. Starting the
spawners in parallel is a race that usually works and occasionally does not.

## Worlds

`worlds/empty_ground.sdf` is a ground plane and a sun, deliberately. This
workspace is the platform, not an application. Put your scenery in your own
package and pass `world:=`.

## What this does not do

It does not start MoveIt. Bring the simulation up first, confirm the
controllers are active, then launch planning separately — when they start
together, a controller that failed to spawn looks exactly like a planning
failure.

It does not simulate the Ouster, the D435 or the RPLIDAR. Those are real
drivers in `ranger_xarm_sensors`. Adding them means gz sensor plugins in the
URDF plus `ros_gz_bridge` topics, and the point clouds will not have the
return statistics of the physical sensors — grazing-incidence dropouts in
particular. Do not treat simulated perception as evidence about real
perception.
