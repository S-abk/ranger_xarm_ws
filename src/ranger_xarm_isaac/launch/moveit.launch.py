#!/usr/bin/env python3
"""Run MoveIt against a running Isaac + ros2_control stack.

Start the three in order:

    $ISAACSIM_PYTHON_EXE .../lib/ranger_xarm_isaac/isaac_bringup.py
    ros2 launch ranger_xarm_isaac control.launch.py
    ros2 launch ranger_xarm_isaac moveit.launch.py

This starts move_group ONLY. robot_state_publisher and the controllers
already exist, put there by control.launch.py, and starting second copies
would give two publishers on /robot_description and two controller
managers racing for the same joints.

Everything below move_group is the gz stack unchanged: MoveIt plans,
hands the trajectory to joint_trajectory_controller, and ros2_control
pushes it at Isaac over topics. That is the point of the seam; the
planner neither knows nor cares which simulator is underneath.

The planning configuration is reused wholesale from
ranger_xarm_moveit_config. Only the controller map is local, because the
controller names here carry xarm's prefix and the gripper is a trajectory
controller rather than the hardware GripperCommand proxy. See
config/moveit_controllers_isaac.yaml.
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PLUGIN = 'joint_state_topic_hardware_interface/JointStateTopicSystem'


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def launch_setup(context, *args, **kwargs):
    add_gripper = LaunchConfiguration('add_gripper').perform(context)
    planning_margin = LaunchConfiguration('planning_margin').perform(context)
    drive_base = _true(context, 'drive_base')
    obstacle_map = _true(context, 'obstacle_map')

    moveit_pkg = Path(get_package_share_directory('ranger_xarm_moveit_config'))
    isaac_pkg = Path(get_package_share_directory('ranger_xarm_isaac'))

    xacro_file = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_description'), 'urdf',
        'ranger_xarm.urdf.xacro'])

    # Must match control.launch.py exactly, except for planning_margin.
    # move_group builds its collision model from this, so a description
    # whose links or joints differ from the one the controllers are using
    # plans against a robot that is not the one moving. planning_margin
    # only grows the fixed-structure collision boxes (planning_margin.xacro
    # in ranger_xarm_description), so move_group keeps a few mm of
    # clearance that the discrete collision checks cannot guarantee;
    # robot_state_publisher, the controllers and the simulator keep the
    # true geometry. move_group does not publish this description.
    #
    # drive_base:=true matches the simulators' drive_base:=true (sim.launch.py,
    # control.launch.py): wheels on, base not welded, the description rooted at
    # base_footprint, whose pose wheel odometry publishes as odom -> base_footprint.
    # move_group then loads the driven-base semantic description (the shipped one
    # plus a planar virtual joint odom -> base_footprint) and plans in odom.
    drive_args = [' use_wheels:=true fix_base_to_world:=false'] if drive_base else []
    robot_description = {
        'robot_description': ParameterValue(Command([
            'xacro ', xacro_file,
            ' xarm_ros2_control_plugin:=', PLUGIN,
            ' add_gripper:=', add_gripper,
            ' xarm_load_gazebo_plugin:=false',
            ' planning_margin:=', planning_margin,
        ] + drive_args), value_type=str)
    }

    srdf_name = 'ranger_xarm_mobile_base.srdf' if drive_base else 'ranger_xarm.srdf'
    srdf = (moveit_pkg / 'config' / srdf_name).read_text(encoding='utf-8')
    joint_limits = _load_yaml(moveit_pkg / 'config' / 'joint_limits.yaml')

    # obstacle_map:=true: the lidar obstacle map, wired as on the real robot
    # (real_execution.launch.py): /ouster/points -> trusted-volume filter (zones
    # off, as shipped) -> /ouster/points_moveit -> move_group's octomap. Same
    # sensors_3d.yaml and trusted_manipulation_volume.yaml, unchanged. move_group
    # keeps the map in its planning frame (odom with drive_base:=true).
    sensors = [_load_yaml(moveit_pkg / 'config' / 'sensors_3d.yaml')] if obstacle_map else []
    trusted_volume_filter = Node(
        package='ranger_xarm_moveit_config',
        executable='trusted_manipulation_volume_filter.py',
        name='trusted_manipulation_volume_filter', output='screen',
        parameters=[{
            'config_file': str(moveit_pkg / 'config' / 'trusted_manipulation_volume.yaml'),
            'enabled_at_startup': False,
            'use_sim_time': True,
        }],
    )

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            {'robot_description_semantic': srdf},
            {'robot_description_kinematics':
                _load_yaml(moveit_pkg / 'config' / 'kinematics.yaml')},
            {'robot_description_planning':
                joint_limits.get('robot_description_planning', {})},
            {'planning_pipelines': ['ompl'],
             'default_planning_pipeline': 'ompl',
             'ompl': _load_yaml(moveit_pkg / 'config' / 'ompl_planning.yaml')},
            *sensors,
            {'planning_scene_monitor': {
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
            }},
            _load_yaml(isaac_pkg / 'config' / 'moveit_controllers_isaac.yaml'),
            {'trajectory_execution': {
                'allowed_execution_duration_scaling': 1.2,
                'allowed_goal_duration_margin': 0.5,
                'allowed_start_tolerance': 0.01,
            }},
            {'allow_trajectory_execution': True},
            {'use_sim_time': True},
        ],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        arguments=['-d', str(moveit_pkg / 'rviz' / 'planning.rviz')],
        parameters=[
            robot_description,
            {'robot_description_semantic': srdf},
            {'use_sim_time': True},
        ],
    )

    return [move_group, rviz] + ([trusted_volume_filter] if obstacle_map else [])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('add_gripper', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument(
            'drive_base', default_value='false',
            description='Set true when the simulator drives the base (sim.launch.py or '
                        'control.launch.py drive_base:=true): MoveIt then plans in odom.'),
        DeclareLaunchArgument(
            'obstacle_map', default_value='false',
            description='Feed the simulated Ouster cloud into MoveIt\'s obstacle map, through '
                        'the trusted-volume filter, as on the real robot. Off by default.'),
        DeclareLaunchArgument(
            'planning_margin', default_value='0.005',
            description='Per-side inflation (m) of fixed-structure collision boxes in '
                        "move_group's description only; 0 = true geometry."),
        OpaqueFunction(function=launch_setup),
    ])
