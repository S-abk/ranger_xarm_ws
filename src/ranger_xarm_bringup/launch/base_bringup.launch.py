from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_device = LaunchConfiguration('can_device')
    update_rate = LaunchConfiguration('update_rate')
    use_xarm = LaunchConfiguration('use_xarm')
    enable_deadman = LaunchConfiguration('enable_deadman')
    deadman_timeout = LaunchConfiguration('deadman_timeout')
    max_linear_x = LaunchConfiguration('max_linear_x')
    max_linear_y = LaunchConfiguration('max_linear_y')
    max_angular_z = LaunchConfiguration('max_angular_z')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_description'), 'urdf', 'ranger_xarm.urdf.xacro'
    ])
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_xarm:=', use_xarm,
        ' use_static_geometry:=true',
        ' use_static_collision:=true',
        ' use_laptop_collision:=true',
        ' laptop_open_angle_deg:=90.0',
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    ranger_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ranger_xarm_bringup'), 'launch', 'ranger_driver.launch.py'
        ])),
        launch_arguments={
            'can_device': can_device,
            'update_rate': update_rate,
            'publish_odom_tf': 'true',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'odom_topic': 'odom',
        }.items(),
    )

    deadman = Node(
        package='ranger_xarm_bringup',
        executable='cmd_vel_deadman.py',
        name='ranger_cmd_vel_deadman',
        output='screen',
        condition=IfCondition(enable_deadman),
        parameters=[{
            'input_topic': '/cmd_vel_remote',
            'output_topic': '/cmd_vel',
            'timeout_sec': ParameterValue(deadman_timeout, value_type=float),
            'publish_rate_hz': 50.0,
            'max_linear_x': ParameterValue(max_linear_x, value_type=float),
            'max_linear_y': ParameterValue(max_linear_y, value_type=float),
            'max_angular_z': ParameterValue(max_angular_z, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('can_device', default_value='can0'),
        DeclareLaunchArgument('update_rate', default_value='50'),
        DeclareLaunchArgument('use_xarm', default_value='false',
                              description='Base-only bringup defaults to no manipulator state.'),
        DeclareLaunchArgument('enable_deadman', default_value='true'),
        DeclareLaunchArgument('deadman_timeout', default_value='0.35'),
        DeclareLaunchArgument('max_linear_x', default_value='0.25'),
        DeclareLaunchArgument('max_linear_y', default_value='0.25'),
        DeclareLaunchArgument('max_angular_z', default_value='0.50'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0.327028595',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'base_link',
            ],
        ),
        ranger_driver,
        deadman,
    ])
