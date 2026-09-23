from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')

    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_a1',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port,
            'serial_baudrate': ParameterValue(serial_baudrate, value_type=int),
            'frame_id': frame_id,
            'inverted': ParameterValue(inverted, value_type=bool),
            'angle_compensate': ParameterValue(angle_compensate, value_type=bool),
        }],
        remappings=[('scan', '/scan')],
    )

    # The A1 has been stable with clean vendor-driver startup.
    # Keep recovery watchdog disabled by default to avoid interfering with startup.
    delayed_start = TimerAction(period=2.0, actions=[lidar])

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/rplidar_a1',
            description='RPLIDAR A1 serial device. Prefer /dev/serial/by-id/... or /dev/rplidar_a1.'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument(
            'frame_id', default_value='laser_frame',
            description='Must match the CAD-derived semantic laser frame in ranger_xarm_description.'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('angle_compensate', default_value='true'),
        delayed_start,
    ])
