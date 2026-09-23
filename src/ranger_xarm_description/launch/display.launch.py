from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_xarm = LaunchConfiguration('use_xarm')
    use_static_geometry = LaunchConfiguration('use_static_geometry')
    use_static_collision = LaunchConfiguration('use_static_collision')
    xarm_mount_yaw = LaunchConfiguration('xarm_mount_yaw')
    use_laptop_collision = LaunchConfiguration('use_laptop_collision')

    xacro_file = PathJoinSubstitution([FindPackageShare('ranger_xarm_description'), 'urdf', 'ranger_xarm.urdf.xacro'])
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_xarm:=', use_xarm,
        ' use_static_geometry:=', use_static_geometry,
        ' use_static_collision:=', use_static_collision,
        ' xarm_mount_yaw:=', xarm_mount_yaw,
        ' use_laptop_collision:=', use_laptop_collision,
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    return LaunchDescription([
        DeclareLaunchArgument('use_xarm', default_value='true'),
        DeclareLaunchArgument('use_static_geometry', default_value='true'),
        DeclareLaunchArgument('use_static_collision', default_value='true'),
        DeclareLaunchArgument('xarm_mount_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_laptop_collision', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[robot_description]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen'),
    ])
