#!/usr/bin/env python3
"""A passive reference move_group on the TRUE collision model, for moveit_test.py.

    ros2 launch tools/moveit_test/ref_move_group.launch.py   (namespace: ref)

It builds move_group the way ranger_xarm_isaac moveit.launch.py does, from the
same installed files, but with planning_margin:=0 (the true fixed-structure
boxes), in namespace /ref, with /joint_states remapped in and trajectory
execution off. moveit_test.py checks each planned path, densely, against its
/ref/check_state_validity. It never plans or moves anything.

moveit_test.py compares the two descriptions and refuses to run if they differ
in anything other than collision box sizes.
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PLUGIN = 'joint_state_topic_hardware_interface/JointStateTopicSystem'


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context)
    moveit_pkg = Path(get_package_share_directory('ranger_xarm_moveit_config'))
    xacro_file = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_description'), 'urdf', 'ranger_xarm.urdf.xacro'])
    # Same xacro arguments as moveit.launch.py, with planning_margin 0.
    robot_description = {'robot_description': ParameterValue(Command([
        'xacro ', xacro_file,
        ' xarm_ros2_control_plugin:=', PLUGIN,
        ' add_gripper:=true',
        ' xarm_load_gazebo_plugin:=false',
        ' planning_margin:=0.0',
    ]), value_type=str)}
    joint_limits = _load_yaml(moveit_pkg / 'config' / 'joint_limits.yaml')
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        remappings=[('joint_states', '/joint_states')],
        parameters=[
            robot_description,
            {'robot_description_semantic':
                (moveit_pkg / 'config' / 'ranger_xarm.srdf').read_text(encoding='utf-8')},
            {'robot_description_kinematics': _load_yaml(moveit_pkg / 'config' / 'kinematics.yaml')},
            {'robot_description_planning': joint_limits.get('robot_description_planning', {})},
            {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
             'ompl': _load_yaml(moveit_pkg / 'config' / 'ompl_planning.yaml')},
            {'allow_trajectory_execution': False},
            {'use_sim_time': True},
        ],
    )
    return [GroupAction([PushRosNamespace(ns), move_group])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='ref'),
        OpaqueFunction(function=launch_setup),
    ])
