#!/usr/bin/env python3
"""Spawn the platform in gz and bring up ros2_control against it.

The URDF is the SAME file the real robot uses. Simulation changes only three
xacro arguments -- the ros2_control system plugin, the <gazebo> plugin block,
and the controller parameters -- so a model fix made for simulation is a fix
for hardware too, and there is no second description to drift.

    ros2 launch ranger_xarm_gazebo sim.launch.py
    ros2 launch ranger_xarm_gazebo sim.launch.py headless:=true
    ros2 launch ranger_xarm_gazebo sim.launch.py world:=/abs/path/to/your.sdf

This brings up joint control only. It does not start MoveIt: pair it with
ranger_xarm_moveit_config's planning launch once the controllers are up, so
the two can be debugged separately.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file

    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')
    add_gripper = LaunchConfiguration('add_gripper').perform(context)
    drive_base = LaunchConfiguration('drive_base').perform(context).lower() in ('true', '1', 'yes')
    prefix = 'xarm_'

    # The upstream controller config is written for a bare arm. This rewrites
    # it for our prefix and namespace and stamps use_sim_time, exactly as
    # xarm_gazebo does -- reusing their helper rather than keeping a forked
    # copy of the yaml that would silently age.
    controllers = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'),
                     'config', 'xarm6_controllers.yaml'),
        prefix=prefix,
        add_gripper=add_gripper.lower() in ('true', '1', 'yes'),
        ros_namespace='',
        update_rate=150,
        robot_type='xarm',
        use_sim_time=True,
    )

    # gz_ros2_control's write() picks ONE control law per joint, in fixed
    # priority: velocity beats position beats effort. The upstream xarm6
    # controller config claims both "position" and "velocity" command
    # interfaces for xarm6_traj_controller, so velocity always wins --
    # and once a trajectory finishes, joint_trajectory_controller's held
    # velocity output is a flat 0, with no feedback term at all. That
    # gives gravity a completely open loop: nothing is watching position
    # error, so xarm_joint2/3 sag a little further every cycle with
    # nothing to stop it, until the arm folds back into the sensor
    # gantry pedestal mounted right behind it.
    #
    # Dropping "velocity" here makes gz_ros2_control fall through to its
    # position branch, which closes a real proportional loop in velocity
    # space (target_vel = -gain * position_error * update_rate) -- gravity
    # sag now generates a restoring command instead of being silently
    # ignored. The stock gain is left alone on purpose: update_rate (150)
    # is baked into that product, so the effective gain is already ~15,
    # and raising the tunable on top of it just makes the joints hunt
    # around the setpoint instead of settling.
    with open(controllers, 'r') as f:
        controllers_yaml = yaml.safe_load(f)
    arm_controller = controllers_yaml['{}xarm6_traj_controller'.format(prefix)]
    arm_controller['ros__parameters']['command_interfaces'] = ['position']

    # gz_ros2_control hands ONE parameters file to the controller_manager it
    # embeds, so the base controllers have to be merged into the same file
    # the arm uses rather than passed separately.
    if drive_base:
        with open(os.path.join(
                get_package_share_directory('ranger_xarm_gazebo'),
                'config', 'base_controllers.yaml'), 'r') as f:
            base_yaml = yaml.safe_load(f)
        for name, cfg in base_yaml['controller_manager']['ros__parameters'].items():
            controllers_yaml['controller_manager']['ros__parameters'][name] = cfg
        for name in ('ranger_steer_controller', 'ranger_wheel_controller'):
            controllers_yaml[name] = base_yaml[name]

    # gz_ros2_control's hold_joints default (true) is deliberately left
    # alone. It writes a zero-velocity command to any actuated joint no
    # controller has claimed yet, which is the only thing holding the arm up
    # during the ~9 s between the model appearing in gz and the trajectory
    # controllers activating. Setting it false lets gravity take the arm over
    # in that window, and joint_trajectory_controller then holds whatever
    # pose it finds on activation: the arm comes up already collapsed, with
    # joint2 pinned against its 2.0944 limit. It was briefly set false while
    # chasing the base yaw problem on bullet-featherstone and never helped;
    # dartsim fixed that.
    with open(controllers, 'w') as f:
        yaml.dump(controllers_yaml, f, default_flow_style=False)

    xacro_file = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_description'), 'urdf',
        'ranger_xarm.urdf.xacro'])
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' xarm_ros2_control_plugin:=gz_ros2_control/GazeboSimSystem',
            ' xarm_load_gazebo_plugin:=true',
            ' xarm_ros2_control_params:=', controllers,
            ' add_gripper:=', add_gripper,
            # Wheels and the world weld are mutually exclusive: the weld is
            # what keeps a wheel-less base from being walked around by the
            # arm, and it would pin a wheeled one to the spot.
            ' use_wheels:=', 'true' if drive_base else 'false',
            ' fix_base_to_world:=', 'false' if drive_base else 'true',
        ]), value_type=str)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            # dartsim, gz-sim's default, for everything.
            #
            # bullet-featherstone is the obvious alternative and is the wrong
            # choice here: it implements a joint velocity command as a rigid
            # Bullet motor constraint that never disengages
            # (gazebosim/gz-sim#2729, gz-physics#713). Four wheels and four
            # steer joints so pinned over-determine the chassis, and it
            # simply refuses to yaw: a commanded 229 deg spin produced 0.5
            # deg. That is also why friction was irrelevant while debugging
            # it (mu 1.5 and mu 50 behaved identically; a wheel that cannot
            # slip does not care). The same model on dartsim spins in place
            # with zero drift.
            #
            # The reason dartsim used to be unusable here was the gripper:
            # it has no mimic constraint support (gz-physics#432, open), so
            # the finger linkage came apart. That is now handled a layer up,
            # by ros2_control's own mimic implementation, which
            # gz_ros2_control applies every cycle regardless of engine. See
            # xarm_gripper.ros2_control.xacro. The gz warning about mimic
            # constraints on startup is expected and harmless.
            'gz_args': ('-r -s -v 2 ' if headless else '-r -v 2 ')
                       + '--physics-engine gz-physics-dartsim-plugin '
                       + world,
        }.items(),
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # Spawned from the /robot_description topic rather than a file, so the
    # model gz receives is byte-for-byte the one robot_state_publisher is
    # using. Spawning from a separately expanded file is how the TF tree and
    # the physics model drift apart.
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'ranger_xarm', '-allow_renaming', 'true',
                   '-z', '0.33'],
    )

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
    )

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager',
                               '/controller_manager'],
                    output='screen')

    jsb = spawner('joint_state_broadcaster')
    arm = spawner('{}xarm6_traj_controller'.format(prefix))
    # add_gripper loads the gripper hardware component, so the controller that
    # claims it has to be spawned too. Without this the gripper joint is
    # simulated but unusable, and the only sign is a controller_manager line
    # saying the component registered no statistics -- not an error.
    gripper = spawner('{}xarm_gripper_traj_controller'.format(prefix))

    steer = spawner('ranger_steer_controller')
    wheels = spawner('ranger_wheel_controller')
    kinematics = Node(
        package='ranger_xarm_gazebo', executable='ranger_4wis_controller.py',
        output='screen', parameters=[{'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        parameters=[{'use_sim_time': True}],
    )

    after_jsb = [arm]
    if add_gripper.lower() in ('true', '1', 'yes'):
        after_jsb.append(gripper)
    if drive_base:
        after_jsb += [steer, wheels, kinematics]

    return [
        gz, rsp, clock_bridge, spawn,
        # Controllers are claimed only once the model exists in gz: the
        # controller_manager the spawners talk to is created by the plugin
        # inside the spawned model, so spawning them earlier is a race.
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb,
                                           on_exit=after_jsb)),
        rviz,
    ]


def gz_resource_path():
    """Every share directory on the ament prefix path, for gz to search.

    sdformat rewrites package:// to model:// when it converts the URDF,
    so gz resolves meshes by searching GZ_SIM_RESOURCE_PATH for a
    directory named after the package. A package only lands there if it
    exports a model path, and the vendored xarm_description does not:
    upstream has no reason to, since it targets the real arm rather than
    gz. The result is 112 "Unable to find file with URI
    [model://xarm_description/...]" errors and an arm with no visual
    meshes at all, while the ranger meshes load fine because
    ranger_xarm_description does carry the export. An invisible arm on a
    visible robot is a confusing first run for anyone cloning this.

    Rather than patch someone else's package.xml, hand gz every share
    directory the workspace already has. Anything overlaid is prepended,
    so an existing value still wins.
    """
    shares = [os.path.join(p, 'share')
              for p in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
              if p]
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing:
        shares.append(existing)
    # dict.fromkeys keeps first-seen order while dropping duplicates.
    return os.pathsep.join(dict.fromkeys(s for s in shares if s))


def generate_launch_description():
    default_world = os.path.join(
        get_package_share_directory('ranger_xarm_gazebo'),
        'worlds', 'empty_ground.sdf')
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path()),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Absolute path to a gz world file.'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run gz without its GUI.'),
        DeclareLaunchArgument('add_gripper', default_value='true',
                              description='Include the xArm gripper.'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument(
            'drive_base', default_value='false',
            description='Add the 4WIS wheels and drive the base from '
                        '/cmd_vel. Off by default: without it the base is '
                        'welded to the world, which is what the arm-only '
                        'workflows expect.'),
        OpaqueFunction(function=launch_setup),
    ])
