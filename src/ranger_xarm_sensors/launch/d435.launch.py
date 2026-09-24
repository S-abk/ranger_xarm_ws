from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    color_profile = LaunchConfiguration('color_profile')
    depth_profile = LaunchConfiguration('depth_profile')
    initial_reset = LaunchConfiguration('initial_reset')

    # Scoped and non-forwarding. rs_launch.py turns every launch configuration
    # it can see into a node parameter, and this file's own arguments --
    # color_profile, depth_profile -- are visible to it by inheritance under
    # those bare names, which the camera does not accept. The result was two
    # "Parameter 'color_profile' is not supported" warnings per start, each
    # printing the driver's whole ~80-entry parameter list.
    #
    # The profiles themselves were always applied correctly through the
    # properly-qualified names below; only the duplicate bare names were
    # rejected. Passing them as scope configurations rather than as inherited
    # ones keeps the mapping and drops the noise.
    realsense = GroupAction(
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
            ])),
            launch_arguments={
                # d435_mount_link is the physical CAD mesh frame.  The URDF adds a
                # semantic ROS camera frame d435_link (X forward, Y left, Z up).
                # RealSense owns only the internal sensor/optical frames below it.
                'camera_namespace': 'camera',
                'camera_name': 'd435',
                'base_frame_id': 'link',
                'device_type': 'd435(?!i)',
                'enable_color': 'true',
                'enable_depth': 'true',

                # This robot has a D435, not a D435i. Keep motion/IR streams off
                # for the initial integration and minimize USB/network load.
                'enable_infra': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'enable_motion': 'false',

                'enable_sync': 'false',
                'align_depth.enable': 'false',
                'pointcloud.enable': 'false',

                # Driver owns only its internal camera TF tree. Its root is d435_link,
                # which robot_state_publisher connects to the CAD mount frame.
                'publish_tf': 'true',
                'tf_publish_rate': '0.0',

                'diagnostics_period': '2.0',
            }.items(),
        )],
        scoped=True, forwarding=False,
        launch_configurations={
            'rgb_camera.color_profile': color_profile,
            'depth_module.depth_profile': depth_profile,
            'initial_reset': initial_reset,
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'color_profile', default_value='640x480x15',
            description='D435 color profile WIDTHxHEIGHTxFPS.'),
        DeclareLaunchArgument(
            'depth_profile', default_value='640x480x15',
            description='D435 depth profile WIDTHxHEIGHTxFPS.'),
        DeclareLaunchArgument(
            'initial_reset', default_value='false',
            description='Reset the RealSense device once before opening it.'),
        realsense,
    ])
