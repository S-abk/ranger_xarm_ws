#!/usr/bin/env python3
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    planning_margin = LaunchConfiguration('planning_margin')
    xarm_mount_yaw = LaunchConfiguration('xarm_mount_yaw')
    use_laptop_collision = LaunchConfiguration('use_laptop_collision')
    laptop_open_angle_deg = LaunchConfiguration('laptop_open_angle_deg')

    description_xacro = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_description'),
        'urdf',
        'ranger_xarm.urdf.xacro',
    ])

    description_cmd = [
        'xacro ', description_xacro,
        ' use_xarm:=true',
        ' use_static_geometry:=true',
        ' use_static_collision:=true',
        ' add_gripper:=true',
        ' xarm_mount_yaw:=', xarm_mount_yaw,
        ' use_laptop_collision:=', use_laptop_collision,
        ' laptop_open_angle_deg:=', laptop_open_angle_deg,
    ]
    robot_description_content = Command(description_cmd)
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    # move_group plans against the same description with every fixed-structure
    # collision box grown by planning_margin per side (see planning_margin.xacro in
    # ranger_xarm_description): MoveIt only checks discrete states, so paths can
    # graze the structure by ~1-2 mm between them. robot_state_publisher and
    # ros2_control keep the true geometry.
    planning_robot_description = {
        'robot_description': ParameterValue(
            Command(description_cmd + [' planning_margin:=', planning_margin]), value_type=str)
    }

    pkg = Path(get_package_share_directory('ranger_xarm_moveit_config'))
    srdf_text = (pkg / 'config' / 'ranger_xarm.srdf').read_text(encoding='utf-8')
    robot_description_semantic = {
        'robot_description_semantic': srdf_text
    }
    kinematics = _load_yaml(pkg / 'config' / 'kinematics.yaml')
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics
    }

    joint_limits_file = _load_yaml(pkg / 'config' / 'joint_limits.yaml')
    robot_description_planning = joint_limits_file.get('robot_description_planning', {})

    ompl = _load_yaml(pkg / 'config' / 'ompl_planning.yaml')
    planning_pipeline = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl,
    }

    planning_scene_monitor = {
        'planning_scene_monitor': {
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        }
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            planning_robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'robot_description_planning': robot_description_planning},
            planning_pipeline,
            planning_scene_monitor,
            {'allow_trajectory_execution': False},
            # Not published: /robot_description is robot_state_publisher's true
            # model; this node's description carries the planning margin.
            {'publish_robot_description': False},
            {'publish_robot_description_semantic': True},
            {'use_sim_time': False},
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[robot_description],
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_moveit_config'),
        'rviz',
        'planning.rviz',
    ])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            planning_robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'planning_margin', default_value='0.005',
            description='Per-side inflation (m) of fixed-structure collision boxes in '
                        "move_group's description only; 0 = true geometry."),
        DeclareLaunchArgument('xarm_mount_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_laptop_collision', default_value='true'),
        DeclareLaunchArgument('laptop_open_angle_deg', default_value='90.0'),
        robot_state_publisher,
        joint_state_publisher,
        move_group,
        rviz,
    ])
