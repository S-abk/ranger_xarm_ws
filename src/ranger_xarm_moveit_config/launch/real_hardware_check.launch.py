#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type')
    laptop_open_angle_deg = LaunchConfiguration('laptop_open_angle_deg')

    description_xacro = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_description'), 'urdf', 'ranger_xarm.urdf.xacro'
    ])

    robot_description_content = Command([
        'xacro ', description_xacro,
        ' use_xarm:=true',
        ' use_static_geometry:=true',
        ' use_static_collision:=true',
        ' add_gripper:=false',
        ' xarm_ros2_control_plugin:=uf_robot_hardware/UFRobotSystemHardware',
        ' xarm_robot_ip:=', robot_ip,
        ' xarm_report_type:=', report_type,
        ' xarm_velocity_control:=false',
        ' laptop_open_angle_deg:=', laptop_open_angle_deg,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    pkg = Path(get_package_share_directory('ranger_xarm_moveit_config'))
    controllers_file = str(pkg / 'config' / 'real_ros2_controllers.yaml')
    xarm_api_pkg = Path(get_package_share_directory('xarm_api'))
    xarm_params_file = str(xarm_api_pkg / 'config' / 'xarm_params.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[controllers_file, xarm_params_file],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
    )

    # Publish the physical state interfaces read by UFRobotSystemHardware.
    # No trajectory controller is loaded in this hardware-check launch.
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

    # Hardware-check mode intentionally omits the gripper.
    # The real gripper is validated separately in real_execution.launch.py via
    # gripper_action_proxy.py. This launch only verifies the six-axis arm state
    # path without exposing any command controller.


    rviz_config = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_moveit_config'), 'rviz', 'hardware_check.rviz'
    ])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[robot_description],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            description='Required IP address of the physical UFACTORY xArm controller.'
        ),
        DeclareLaunchArgument('report_type', default_value='normal'),
        DeclareLaunchArgument('laptop_open_angle_deg', default_value='90.0'),
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        rviz,
    ])
