#!/usr/bin/env python3
"""Gyro-fused odometry: bias corrector + robot_localization EKF.

Run this ALONGSIDE the unified launch, with the Ranger driver's own TF turned
off so exactly one node owns odom -> base_footprint:

    ros2 launch ranger_xarm_sensors robot.launch.py \\
        robot_ip:=192.168.1.221 publish_odom_tf:=false
    ros2 launch ranger_xarm_bringup ekf_odom_imu.launch.py

Two publishers of the same transform is a silent failure, not a loud one: TF
consumers take whichever arrived last and the fusion quietly does nothing. The
same trap appeared offline, where rtabmap read the odometry pose from TF rather
than the odom topic and produced a byte-identical map until the stale TF was
excluded.

Requires ros-jazzy-robot-localization.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # fuse_attitude:=true also fuses roll/pitch, so terrain bumps are
    # compensated rather than discarded by two_d_mode.
    cfg = PythonExpression([
        "'ekf_odom_imu_3d.yaml' if '", LaunchConfiguration('fuse_attitude'),
        "'.lower() in ('true','1','yes') else 'ekf_odom_imu.yaml'"])
    params = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_bringup'), 'config', cfg])

    corrector = Node(
        package='ranger_xarm_bringup',
        executable='imu_yaw_bias_corrector.py',
        name='imu_yaw_bias_corrector',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'imu_in': LaunchConfiguration('imu_in'),
            'imu_out': LaunchConfiguration('imu_out'),
            'odom_in': LaunchConfiguration('odom_in'),
            'initial_bias': LaunchConfiguration('initial_bias'),
            'publish_attitude': LaunchConfiguration('fuse_attitude'),
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name=PythonExpression([
            "'ekf_odom_imu_3d' if '", LaunchConfiguration('fuse_attitude'),
            "'.lower() in ('true','1','yes') else 'ekf_odom_imu'"]),
        output='screen',
        parameters=[params,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('odometry/filtered', LaunchConfiguration('output_topic'))],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true for bag replay. The EKF runs a fixed-rate '
                        'prediction step, so replaying without --clock and this '
                        'set makes it predict on wall-clock dt against '
                        'bag-stamped measurements, which breaks the filter '
                        'outright rather than degrading it.'),
        DeclareLaunchArgument(
            'fuse_attitude', default_value='false',
            description='also fuse roll/pitch to compensate terrain bumps. '
                        'Selects ekf_odom_imu_3d.yaml and makes the corrector '
                        'publish a levelled attitude. Default false keeps the '
                        'verified 2D behaviour.'),
        DeclareLaunchArgument('imu_in', default_value='/ouster/imu'),
        DeclareLaunchArgument('imu_out', default_value='/ouster/imu_corrected'),
        DeclareLaunchArgument('odom_in', default_value='/odom'),
        DeclareLaunchArgument(
            'initial_bias', default_value='0.005847',
            description='seed gyro-z bias, rad/s (+0.335 deg/s measured '
                        '2026-09-09 and 2026-09-12)'),
        DeclareLaunchArgument('output_topic', default_value='/odometry/filtered'),
        corrector,
        ekf,
    ])
