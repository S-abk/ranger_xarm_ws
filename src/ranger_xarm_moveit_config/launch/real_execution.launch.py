#!/usr/bin/env python3
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, EqualsSubstitution, FileContent, IfElseSubstitution,
                                  LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    planning_margin = LaunchConfiguration('planning_margin')
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type')
    laptop_open_angle_deg = LaunchConfiguration('laptop_open_angle_deg')
    start_rviz = LaunchConfiguration('start_rviz')
    trusted_volume_enabled = LaunchConfiguration('trusted_volume_enabled')
    fix_base_to_world = LaunchConfiguration('fix_base_to_world')
    planning_frame = LaunchConfiguration('planning_frame')

    description_xacro = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_description'), 'urdf', 'ranger_xarm.urdf.xacro'
    ])

    description_cmd = [
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
        ' fix_base_to_world:=', fix_base_to_world,
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
    trusted_volume_config = str(
        pkg / 'config' / 'trusted_manipulation_volume.yaml'
    )
    # Welded base: the shipped semantic description, planning frame world. Base pose published
    # by the driver or the filter (fix_base_to_world:=false):
    #   planning_frame:=odom (default) -- the generated one with a planar virtual joint
    #     odom -> base_footprint: MoveIt plans in odom and its obstacle map (kept in the
    #     planning frame) is kept in the world; it depends on odometry;
    #   planning_frame:=base_footprint -- the shipped one: MoveIt plans in base_footprint, the
    #     map moves with the robot and does not depend on odometry.
    # Either way the map was shown right only at rest, 2 s after the base stops, and arm
    # motion is planned and executed only then (workspace README, "Frames").
    shipped_srdf = str(pkg / 'config' / 'ranger_xarm.srdf')
    robot_description_semantic = {'robot_description_semantic': ParameterValue(FileContent(
        IfElseSubstitution(fix_base_to_world, shipped_srdf, IfElseSubstitution(
            EqualsSubstitution(planning_frame, 'odom'),
            str(pkg / 'config' / 'ranger_xarm_mobile_base.srdf'), shipped_srdf))), value_type=str)}
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
    # The teleop trajectory controller (ranger_xarm_teleop), loaded inactive: the teleop
    # stack's command mux is the only thing that ever activates it (strict switch, arm still).
    teleop_controller_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'teleop_arm_controller', '--inactive', '--controller-manager',
            '/controller_manager', '--controller-manager-timeout', '30',
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
            planning_robot_description,
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
            # Not published: /robot_description is robot_state_publisher's true
            # model; this node's description carries the planning margin.
            {'publish_robot_description': False},
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
            'fix_base_to_world', default_value='true',
            description=(
                'true: the base is welded to world (arm alone; nothing publishes the base pose). '
                'false: the base driver or the filter publishes odom -> base_footprint; the '
                'description is rooted at base_footprint and MoveIt plans in odom. '
                'ranger_xarm_sensors robot.launch.py sets this from enable_base.'
            ),
        ),
        DeclareLaunchArgument(
            'planning_frame', default_value='odom', choices=['odom', 'base_footprint'],
            description=(
                'With the base free (fix_base_to_world:=false): odom plans in the odometry frame '
                'through a planar virtual joint, so the obstacle map is kept in the world and '
                'depends on odometry; base_footprint plans on the base, so the map moves with the '
                'robot and does not depend on odometry. Either way, plan and move the arm only '
                'with the base at rest, 2 s after it stops. Ignored while the base is welded '
                '(plans in world).'
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
        teleop_controller_spawner,
        trusted_volume_filter,
        move_group,
        rviz,
        
    ])
