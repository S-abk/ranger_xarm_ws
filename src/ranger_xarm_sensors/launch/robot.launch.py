#!/usr/bin/env python3
"""The physical robot: base, sensors and MoveIt, composed once.

Everything optional is an `enable_*` argument rather than a separate launch
file, so there is one file to read and one place a default can be wrong.

    ros2 launch ranger_xarm_sensors robot.launch.py robot_ip:=<arm ip> \
        ouster_sensor_hostname:=<ouster host> \
        gantry_d455_serial_no:=<digits> wrist_d435_serial_no:=<digits>
    ros2 launch ranger_xarm_sensors robot.launch.py robot_ip:=<arm ip> \
        enable_ouster:=false enable_gantry_d455:=false enable_wrist_d435:=false

Camera serials. Each RealSense has its own required serial argument and NO
default, so the launch can never bind "whichever camera enumerates first":

    gantry_d455_serial_no   required while enable_gantry_d455 is true
    wrist_d435_serial_no    required while enable_wrist_d435 is true

Digits only; a leading '_' is accepted and ignored. A missing, empty or
malformed serial, or the same serial for both cameras, stops the launch before
anything else starts. With a camera disabled its serial is not needed. Read the
serials with rs-enumerate-devices.

This launch talks to hardware. It does not execute any arm motion on its own:
MoveIt comes up able to plan, and executing a plan is a separate, explicit act.
Anything that moves the arm autonomously belongs in a downstream package that
includes this one, not in here -- see the workspace README.
"""
import re

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# (camera_name, device_type). device_type is a case-insensitive regex on the
# device name; 'd435(?!i)' keeps a D435i from answering for the wrist D435.
_CAMERAS = (
    ('gantry_d455', 'd455'),
    ('wrist_d435', 'd435(?!i)'),
)
_SERIAL = re.compile(r'[0-9]+', re.ASCII)


def _camera_serial(context, camera):
    """The validated digit serial for `camera`, or RuntimeError naming its argument.

    Same rule as normalize_serial in d400_camera.launch.py (digits only, one
    optional leading '_'), repeated here so a bad serial is caught while
    nothing has started yet; test_camera_launch.py checks the two agree.
    """
    arg = f'{camera}_serial_no'
    raw = context.launch_configurations.get(arg)
    if raw is None or not raw.strip():
        raise RuntimeError(
            f"{camera} is enabled but required launch argument '{arg}' was "
            f"not provided. Pass {arg}:=<digits> (rs-enumerate-devices lists "
            f"the serials) or disable the camera with enable_{camera}:=false. "
            "There is deliberately no default: the launch never binds the "
            "first camera it finds.")
    digits = raw.strip()
    if digits.startswith('_'):
        digits = digits[1:]
    if not _SERIAL.fullmatch(digits):
        raise RuntimeError(
            f"{arg} must be the camera serial number (digits only, a leading "
            f"'_' is allowed), got {raw!r}.")
    return digits


def realsense_cameras(context):
    """One scoped d400_camera include per enabled camera.

    Why an OpaqueFunction and not DeclareLaunchArgument + LaunchConfiguration:
    a DeclareLaunchArgument without a default makes the WHOLE launch fail when
    the argument is missing, even with that camera disabled, which would make
    enable_*:=false unusable. So the serial arguments are not declared; they
    are read from the launch configuration here, and only for a camera that is
    enabled. (They are therefore also absent from `ros2 launch --show-args`;
    the module docstring and the README list them.)
    """
    enabled = [(name, device_type) for name, device_type in _CAMERAS
               if IfCondition(LaunchConfiguration(f'enable_{name}')).evaluate(context)]
    serials = {name: _camera_serial(context, name) for name, _ in enabled}
    if len(set(serials.values())) < len(serials):
        raise RuntimeError(
            'gantry_d455_serial_no and wrist_d435_serial_no are the same '
            f'serial ({next(iter(serials.values()))}); two cameras cannot '
            'share one device.')

    def perform(name):
        return context.perform_substitution(LaunchConfiguration(name))

    groups = []
    for name, device_type in enabled:
        groups.append(GroupAction(
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('ranger_xarm_sensors'),
                    'launch', 'd400_camera.launch.py'])),
                # Plain strings, already resolved against this context. They
                # have to be launch_arguments of the include (launch checks
                # that an include is given its required arguments), and
                # forwarding=False below clears every configuration before the
                # include is evaluated, so a LaunchConfiguration here would
                # no longer exist by then.
                launch_arguments={
                    'camera_name': name,
                    'device_type': device_type,
                    'serial_no': serials[name],
                    'color_profile': perform('color_profile'),
                    'depth_profile': perform('depth_profile'),
                    'initial_reset': perform(f'{name}_initial_reset'),
                }.items(),
            )],
            # Scoped and NOT forwarding, which matters more than it looks. An
            # include inherits the parent's launch configurations, and the
            # RealSense launch turns every configuration it can see into a
            # node parameter -- so every argument declared here (can_device,
            # enable_base, max_linear_x, ...) arrived at the camera as an
            # unsupported parameter and produced ~20 lines of warning each
            # start. That noise is not cosmetic: it buried a driver crash
            # during bring-up and cost real time to see past.
            scoped=True, forwarding=False,
        ))
    return groups


def generate_launch_description():
    lc = LaunchConfiguration

    # The xArm + MoveIt stack owns the single robot_state_publisher for the
    # complete URDF. Nothing else here may start a second one.
    xarm_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ranger_xarm_moveit_config'),
            'launch', 'real_execution.launch.py'])),
        launch_arguments={
            'robot_ip': lc('robot_ip'),
            'report_type': lc('report_type'),
            'gripper_initial_position': lc('gripper_initial_position'),
            'laptop_open_angle_deg': lc('laptop_open_angle_deg'),
            'start_rviz': lc('start_rviz'),
        }.items(),
    )

    # Exactly one node may own odom -> base_footprint. The driver does by
    # default; set publish_odom_tf:=false to hand it to the gyro-fused EKF in
    # ranger_xarm_bringup/ekf_odom_imu.launch.py. Two owners is a silent
    # failure: TF takes whichever arrived last.
    ranger_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ranger_xarm_bringup'),
            'launch', 'ranger_driver.launch.py'])),
        condition=IfCondition(lc('enable_base')),
        launch_arguments={
            'can_device': lc('can_device'),
            'update_rate': lc('update_rate'),
            'publish_odom_tf': lc('publish_odom_tf'),
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'odom_topic': 'odom',
        }.items(),
    )

    # base_footprint is the lowest wheel-tread point; base_link is the CAD
    # reference. They are different frames -- do not collapse them to fix a
    # visual offset.
    base_footprint_to_base_link = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['--x', '0', '--y', '0', '--z', '0.327028595',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', 'base_link'],
    )

    # Relays /cmd_vel_remote to /cmd_vel and publishes zero after the timeout,
    # so a dropped teleop link stops the base instead of leaving it driving.
    deadman = Node(
        package='ranger_xarm_bringup', executable='cmd_vel_deadman.py',
        name='ranger_cmd_vel_deadman', output='screen',
        condition=IfCondition(lc('enable_deadman')),
        parameters=[{
            'input_topic': '/cmd_vel_remote',
            'output_topic': '/cmd_vel',
            'timeout_sec': ParameterValue(lc('deadman_timeout'), value_type=float),
            'publish_rate_hz': 50.0,
            'max_linear_x': ParameterValue(lc('max_linear_x'), value_type=float),
            'max_linear_y': ParameterValue(lc('max_linear_y'), value_type=float),
            'max_angular_z': ParameterValue(lc('max_angular_z'), value_type=float),
        }],
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ranger_xarm_sensors'),
            'launch', 'rplidar_a1.launch.py'])),
        condition=IfCondition(lc('enable_rplidar')),
        launch_arguments={
            'serial_port': lc('rplidar_serial_port'),
            'serial_baudrate': '115200',
            'frame_id': 'laser_frame',
            'inverted': 'false',
            'angle_compensate': 'true',
        }.items(),
    )

    # Ouster's own launch, so the driver keeps ownership of
    # os_sensor -> os_lidar/os_imu and its factory calibration. The URDF owns
    # only base_link -> os0_mount_link -> os_sensor.
    ouster = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ouster_ros'), 'launch', 'sensor.launch.xml'])),
        condition=IfCondition(lc('enable_ouster')),
        launch_arguments={
            'sensor_hostname': lc('ouster_sensor_hostname'),
            'udp_dest': lc('ouster_udp_dest'),
            'lidar_mode': lc('ouster_lidar_mode'),
            'timestamp_mode': lc('ouster_timestamp_mode'),
            'viz': 'false',
        }.items(),
    )

    # Decimated preview cloud for a remote viewer. Autonomy must always read
    # the full /ouster/points, never this.
    ouster_viz_cloud = Node(
        package='ranger_xarm_sensors', executable='ouster_points_viz',
        name='ouster_points_viz', output='screen',
        condition=IfCondition(lc('enable_ouster_viz_cloud')),
        parameters=[{
            'input_topic': '/ouster/points',
            'output_topic': '/ouster/points_viz',
            'point_stride': ParameterValue(lc('ouster_viz_point_stride'), value_type=int),
            'publish_every_nth': ParameterValue(lc('ouster_viz_publish_every_nth'), value_type=int),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip',
                              description='IP address of the xArm controller.'),
        DeclareLaunchArgument('report_type', default_value='normal'),
        DeclareLaunchArgument('gripper_initial_position', default_value='0.0'),
        DeclareLaunchArgument('laptop_open_angle_deg', default_value='90.0'),
        DeclareLaunchArgument('start_rviz', default_value='false'),

        DeclareLaunchArgument('enable_base', default_value='true'),
        DeclareLaunchArgument('can_device', default_value='can0'),
        DeclareLaunchArgument('update_rate', default_value='50'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        DeclareLaunchArgument('enable_deadman', default_value='true'),
        DeclareLaunchArgument('deadman_timeout', default_value='0.35'),
        DeclareLaunchArgument('max_linear_x', default_value='0.25'),
        DeclareLaunchArgument('max_linear_y', default_value='0.25'),
        DeclareLaunchArgument('max_angular_z', default_value='0.50'),

        DeclareLaunchArgument('enable_rplidar', default_value='true'),
        DeclareLaunchArgument('rplidar_serial_port', default_value='/dev/rplidar_a1'),
        # The camera serials (gantry_d455_serial_no, wrist_d435_serial_no) are
        # deliberately NOT declared here. See realsense_cameras().
        DeclareLaunchArgument('enable_gantry_d455', default_value='true'),
        DeclareLaunchArgument('enable_wrist_d435', default_value='true'),
        DeclareLaunchArgument('color_profile', default_value='640x480x15'),
        DeclareLaunchArgument('depth_profile', default_value='640x480x15'),
        DeclareLaunchArgument('gantry_d455_initial_reset', default_value='false'),
        DeclareLaunchArgument('wrist_d435_initial_reset', default_value='false'),

        DeclareLaunchArgument('enable_ouster', default_value='true'),
        DeclareLaunchArgument('ouster_sensor_hostname',
                              description='Hostname or IP of the Ouster '
                                          '(required with enable_ouster).'),
        DeclareLaunchArgument('ouster_udp_dest', default_value='',
                              description='Host IP the sensor sends to; '
                                          'empty lets ouster_ros detect it.'),
        DeclareLaunchArgument('ouster_lidar_mode', default_value='1024x10'),
        DeclareLaunchArgument('ouster_timestamp_mode', default_value='TIME_FROM_ROS_TIME'),
        DeclareLaunchArgument('enable_ouster_viz_cloud', default_value='true'),
        DeclareLaunchArgument('ouster_viz_point_stride', default_value='16'),
        DeclareLaunchArgument('ouster_viz_publish_every_nth', default_value='2'),

        # First, so a missing or malformed camera serial aborts the launch
        # before MoveIt, the arm or the base driver have been started. The
        # entities after it are not visited until this one has run.
        OpaqueFunction(function=realsense_cameras),

        xarm_moveit,
        ranger_driver,
        base_footprint_to_base_link,
        deadman,
        rplidar,
        ouster,
        ouster_viz_cloud,
    ])
