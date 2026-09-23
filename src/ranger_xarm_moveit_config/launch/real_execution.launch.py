#!/usr/bin/env python3
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type')
    laptop_open_angle_deg = LaunchConfiguration('laptop_open_angle_deg')
    start_rviz = LaunchConfiguration('start_rviz')
    trusted_volume_enabled = LaunchConfiguration('trusted_volume_enabled')

    description_xacro = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_description'), 'urdf', 'ranger_xarm.urdf.xacro'
    ])

    robot_description_content = Command([
        'xacro ', description_xacro,
        ' use_xarm:=true',
        ' use_static_geometry:=true',
        ' use_static_collision:=true',
        ' add_gripper:=true',
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
    trusted_volume_config = str(
        pkg / 'config' / 'trusted_manipulation_volume.yaml'
    )
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

    sensors_3d = _load_yaml(
        pkg / 'config' / 'sensors_3d.yaml'
    )
    planning_scene_monitor = {
        'planning_scene_monitor': {
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        }
    }
    moveit_controllers = _load_yaml(pkg / 'config' / 'moveit_controllers_real.yaml')
    trajectory_execution = {
        'trajectory_execution': {
            'allowed_execution_duration_scaling': 1.5,
            'allowed_goal_duration_margin': 1.0,
            'allowed_start_tolerance': 0.01,
        }
    }
    controllers_file = str(pkg / 'config' / 'real_ros2_controllers.yaml')
    xarm_api_pkg = Path(get_package_share_directory('xarm_api'))
    xarm_params_file = str(xarm_api_pkg / 'config' / 'xarm_params.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[robot_description],
    )
    ros2_control_node = Node(
        package='controller_manager', executable='ros2_control_node',
        output='screen', parameters=[controllers_file, xarm_params_file],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
    )
    # Publish the physical xArm state interfaces read by UFRobotSystemHardware.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'joint_state_broadcaster', '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )
    gripper_action_proxy = Node(
        package='ranger_xarm_moveit_config', executable='gripper_action_proxy.py',
        name='xarm_gripper_action_proxy', output='screen',
        parameters=[{
            # MoveIt expects /xarm_gripper/gripper_action, but because our
            # combined model uses prefix=xarm_, UFACTORY exposes the native
            # server at /xarm_xarm_gripper/gripper_action.
            'proxy_action_name': '/xarm_gripper/gripper_action',
            'native_action_name': '/xarm_xarm_gripper/gripper_action',
            'joint_state_topic': '/joint_states',
            'joint_name': 'xarm_drive_joint',
            'initial_position': ParameterValue(
                LaunchConfiguration('gripper_initial_position'), value_type=float
            ),
            'publish_rate_hz': 10.0,
            'native_server_timeout_sec': 10.0,
        }],
    )

    trajectory_controller_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'xarm6_traj_controller', '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )
    
    trusted_volume_filter = Node(
        package='ranger_xarm_moveit_config',
        executable='trusted_manipulation_volume_filter.py',
        name='trusted_manipulation_volume_filter',
        output='screen',
        parameters=[{
            'config_file': trusted_volume_config,
            'enabled_at_startup': ParameterValue(
                trusted_volume_enabled,
                value_type=bool,
            ),
        }],
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

            # Ouster -> MoveIt occupancy map.
            sensors_3d,

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
        FindPackageShare('ranger_xarm_moveit_config'), 'rviz', 'planning.rviz'
    ])
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(start_rviz),
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            description='Required IP address of the physical UFACTORY xArm controller.'
        ),
        DeclareLaunchArgument('report_type', default_value='normal'),
        DeclareLaunchArgument('laptop_open_angle_deg', default_value='90.0'),
        DeclareLaunchArgument(
            'start_rviz', default_value='true',
            description='Start the local MoveIt RViz GUI. Unified onboard bringup sets this false.'
        ),
        DeclareLaunchArgument(
            'gripper_initial_position', default_value='0.0',
            description=(
                'Assumed physical xarm_drive_joint position at startup. '
                '0.0 rad = open, ~0.86 rad = closed. No startup motion is commanded.'
            ),
        ),
        DeclareLaunchArgument(
            'trusted_volume_enabled',
            default_value='false',
            description=(
                'Enable the robot-local trusted manipulation volume for '
                'MoveIt OctoMap perception.'
            ),
        ),
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        gripper_action_proxy,
        trajectory_controller_spawner,
        trusted_volume_filter,
        move_group,
        rviz,
        
    ])
