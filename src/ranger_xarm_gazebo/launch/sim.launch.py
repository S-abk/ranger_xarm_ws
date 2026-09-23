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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
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
        ]), value_type=str)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': ('-r -s -v 2 ' if headless else '-r -v 2 ') + world,
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

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        parameters=[{'use_sim_time': True}],
    )

    return [
        gz, rsp, clock_bridge, spawn,
        # Controllers are claimed only once the model exists in gz: the
        # controller_manager the spawners talk to is created by the plugin
        # inside the spawned model, so spawning them earlier is a race.
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb,
            on_exit=[arm] if add_gripper.lower() not in ('true', '1', 'yes')
            else [arm, gripper])),
        rviz,
    ]


def generate_launch_description():
    default_world = os.path.join(
        get_package_share_directory('ranger_xarm_gazebo'),
        'worlds', 'empty_ground.sdf')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Absolute path to a gz world file.'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run gz without its GUI.'),
        DeclareLaunchArgument('add_gripper', default_value='true',
                              description='Include the xArm gripper.'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
