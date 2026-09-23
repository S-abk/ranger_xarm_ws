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
    xarm_mount_yaw = LaunchConfiguration('xarm_mount_yaw')
    use_laptop_collision = LaunchConfiguration('use_laptop_collision')
    laptop_open_angle_deg = LaunchConfiguration('laptop_open_angle_deg')

    description_xacro = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_description'),
        'urdf',
        'ranger_xarm.urdf.xacro',
    ])

    # GenericSystem is the standard ros2_control mock component. It mirrors
    # controller commands into state interfaces, so this exercises the full
    # MoveIt -> FollowJointTrajectory -> ros2_control pipeline without hardware.
    robot_description_content = Command([
        'xacro ', description_xacro,
        ' use_xarm:=true',
        ' use_static_geometry:=true',
        ' use_static_collision:=true',
        ' add_gripper:=true',
        ' xarm_ros2_control_plugin:=mock_components/GenericSystem',
        ' xarm_mount_yaw:=', xarm_mount_yaw,
        ' use_laptop_collision:=', use_laptop_collision,
        ' laptop_open_angle_deg:=', laptop_open_angle_deg,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    pkg = Path(get_package_share_directory('ranger_xarm_moveit_config'))

    srdf_text = (pkg / 'config' / 'ranger_xarm.srdf').read_text(encoding='utf-8')
    robot_description_semantic = {'robot_description_semantic': srdf_text}
    robot_description_kinematics = {
        'robot_description_kinematics': _load_yaml(pkg / 'config' / 'kinematics.yaml')
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

    moveit_controllers = _load_yaml(pkg / 'config' / 'moveit_controllers.yaml')
    trajectory_execution = {
        'trajectory_execution': {
            'allowed_execution_duration_scaling': 1.2,
            'allowed_goal_duration_margin': 0.5,
            'allowed_start_tolerance': 0.01,
        }
    }

    controllers_file = str(pkg / 'config' / 'fake_ros2_controllers.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Jazzy controller_manager subscribes to robot_description. Remapping the
    # private controller-manager topic makes the source explicit.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[controllers_file],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )

    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'xarm6_traj_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'robot_description_planning': robot_description_planning},
            planning_pipeline,
            planning_scene_monitor,
            moveit_controllers,
            trajectory_execution,
            {'allow_trajectory_execution': True},
            {'moveit_manage_controllers': False},
            {'publish_robot_description': True},
            {'publish_robot_description_semantic': True},
            {'use_sim_time': False},
        ],
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
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('xarm_mount_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_laptop_collision', default_value='true'),
        DeclareLaunchArgument('laptop_open_angle_deg', default_value='90.0'),
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        trajectory_controller_spawner,
        move_group,
        rviz,
    ])
