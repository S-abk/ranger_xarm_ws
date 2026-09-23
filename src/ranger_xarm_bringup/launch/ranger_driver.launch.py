from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    can_device = LaunchConfiguration('can_device')
    update_rate = LaunchConfiguration('update_rate')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    odom_topic = LaunchConfiguration('odom_topic')

    ranger = Node(
        package='ranger_base',
        executable='ranger_base_node',
        name='ranger_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
            'port_name': can_device,
            'robot_model': 'ranger_mini_v3',
            'simulated_robot': False,
            'update_rate': ParameterValue(update_rate, value_type=int),
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'odom_topic_name': odom_topic,
            'publish_odom_tf': ParameterValue(publish_odom_tf, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('can_device', default_value='can0'),
        DeclareLaunchArgument('update_rate', default_value='50'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        ranger,
    ])
