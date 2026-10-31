"""One Intel RealSense D400-series camera, bound to one specific device.

    ros2 launch ranger_xarm_sensors d400_camera.launch.py \
        camera_name:=gantry_d455 device_type:=d455 serial_no:=<digits>

Normally this is included by robot.launch.py, which passes all three. The
arguments below have no defaults on purpose:

* camera_name  topics become /camera/<camera_name>/..., frames <camera_name>_*.
* serial_no    digits only; a leading '_' is accepted and ignored. REQUIRED,
               and never defaulted: with two cameras on one robot, "the first
               device found" is a coin flip between them. The launch refuses
               to start rather than bind whichever enumerates first. The
               serial is the binding identity.
* device_type  the driver's model filter, a case-insensitive regex on the
               device name -- 'd455', or 'd435(?!i)' to exclude the D435i. A
               secondary guard on top of the serial: a serial that belongs to
               the wrong model finds no device instead of the wrong one.

The serial is handed to rs_launch.py as '_' + digits. ROS 2 turns a parameter
value made only of digits into an integer, and the driver's serial_no is a
string; realsense2_camera strips one leading '_' to get round that
(realsense_node_factory.cpp:337 in 4.58.4).
"""
import re

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_SERIAL = re.compile(r'[0-9]+', re.ASCII)
_NAME = re.compile(r'[A-Za-z][A-Za-z0-9_]*', re.ASCII)


def normalize_serial(raw, what='serial_no'):
    """Return the bare digit string for `raw`, or raise RuntimeError.

    Whitespace and one leading '_' are ignored. Anything else that is not
    entirely ASCII digits -- empty, 'abc', '123 456', '__123' -- is an error.
    Keep in step with robot.launch.py, which validates the same way earlier so
    that nothing has started by the time a bad serial is found.
    """
    digits = (raw or '').strip()
    if digits.startswith('_'):
        digits = digits[1:]
    if not _SERIAL.fullmatch(digits):
        raise RuntimeError(
            f'{what} must be the camera serial number (digits only, a leading '
            f"'_' is allowed), got {raw!r}. Refusing to start: the launch "
            'never binds the first device it finds. The serial is printed by '
            'rs-enumerate-devices.')
    return digits


def _launch_camera(context):
    def value(name):
        return context.perform_substitution(LaunchConfiguration(name)).strip()

    camera_name = value('camera_name')
    if not _NAME.fullmatch(camera_name):
        raise RuntimeError(
            f'camera_name must be an identifier such as gantry_d455, '
            f'got {camera_name!r}.')
    device_type = value('device_type')
    if not device_type:
        raise RuntimeError(
            f"camera '{camera_name}': device_type must not be empty "
            "(for example 'd455' or 'd435(?!i)').")
    serial = normalize_serial(
        context.perform_substitution(LaunchConfiguration('serial_no')),
        what=f"camera '{camera_name}': serial_no")

    # Scoped and non-forwarding. rs_launch.py turns every launch configuration
    # it can see into a node parameter, and this file's own arguments --
    # color_profile, depth_profile, camera_name -- are visible to it by
    # inheritance under those bare names, which the camera does not accept.
    # The result was "Parameter 'color_profile' is not supported" warnings on
    # every start, each printing the driver's whole ~80-entry parameter list.
    #
    # The profiles themselves were always applied correctly through the
    # properly-qualified names below; only the duplicate bare names were
    # rejected. Passing them as scope configurations rather than as inherited
    # ones keeps the mapping and drops the noise.
    #
    # Everything is resolved to plain strings above and passed as literals:
    # forwarding=False clears the configurations before the include is
    # evaluated, so a LaunchConfiguration here would no longer exist.
    realsense = GroupAction(
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
            ])),
            launch_arguments={
                # <camera_name>_mount_link is the physical CAD mesh frame and
                # <camera_name>_link the semantic ROS camera frame (X forward,
                # Y left, Z up); robot_state_publisher owns both. The driver is
                # rooted at <camera_name>_link (base_frame_id 'link' plus the
                # camera_name prefix) and publishes only the sensor and optical
                # frames below it. Rooting it at the Z-forward CAD frame would
                # rotate every optical frame.
                'camera_namespace': 'camera',
                'camera_name': camera_name,
                'base_frame_id': 'link',
                'device_type': device_type,
                'serial_no': '_' + serial,
                'enable_color': 'true',
                'enable_depth': 'true',

                # Motion/IR streams stay off: the D435 has no IMU, and nothing
                # consumes the D455's. Keeps USB/network load down with two
                # cameras on the robot.
                'enable_infra': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'enable_motion': 'false',

                'enable_sync': 'false',
                'align_depth.enable': 'false',
                'pointcloud.enable': 'false',

                # Driver owns only its internal camera TF tree. Its root is
                # <camera_name>_link, which robot_state_publisher connects to
                # the CAD mount frame.
                'publish_tf': 'true',
                'tf_publish_rate': '0.0',

                'diagnostics_period': '2.0',
            }.items(),
        )],
        scoped=True, forwarding=False,
        launch_configurations={
            'rgb_camera.color_profile': value('color_profile'),
            'depth_module.depth_profile': value('depth_profile'),
            'initial_reset': value('initial_reset'),
        },
    )
    return [realsense]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            description='Unique camera name, e.g. gantry_d455 or wrist_d435.'),
        DeclareLaunchArgument(
            'serial_no',
            description='Camera serial number, digits only (a leading _ is '
                        'allowed). Required: never defaulted.'),
        DeclareLaunchArgument(
            'device_type',
            description="Model filter regex: 'd455' or 'd435(?!i)'."),
        DeclareLaunchArgument(
            'color_profile', default_value='640x480x15',
            description='Color profile WIDTHxHEIGHTxFPS.'),
        DeclareLaunchArgument(
            'depth_profile', default_value='640x480x15',
            description='Depth profile WIDTHxHEIGHTxFPS.'),
        DeclareLaunchArgument(
            'initial_reset', default_value='false',
            description='Reset the RealSense device once before opening it.'),
        OpaqueFunction(function=_launch_camera),
    ])
