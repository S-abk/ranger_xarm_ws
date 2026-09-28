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

lidar_odometry:=true also runs KISS-ICP on the Ouster cloud and fuses its
pose as a second odometry source (odom1):

- KISS-ICP never publishes TF: the EKF owns odom -> base_footprint, and a
  second parent for that frame is what once broke the heading.
- lidar_odometry_relay.py re-anchors KISS-ICP's pose into the EKF's odom
  frame and publishes it only while the scene's geometry can constrain scan
  matching and each step agrees with the gyro-held EKF; after any
  interruption it re-anchors to the EKF's own pose, so the estimate never
  jumps and a KISS-ICP failure is never imported. See its docstring.
- The EKF fuses that pose's x, y and yaw as absolute measurements, each at
  its scan's time (smooth_lagged_data). Pose rather than velocity because
  KISS-ICP's per-scan registration errors cancel along a drive but its
  per-scan velocity is noisier than the wheels'; yaw too because the
  anchored positions carry KISS-ICP's heading, and leaving it out makes the
  EKF rotate itself to reconcile them with a drifting gyro.

KISS-ICP needs the full-rate cloud, and a 128 x 1024 Ouster cloud is too
big for Fast DDS's default shared memory: it falls back to UDP and loses
scans. Export config/fastdds_large_shm.xml as FASTRTPS_DEFAULT_PROFILES_FILE
in every process that publishes or consumes the cloud (see that file).

Measured in simulation (docs/TODO.md): the fused estimate follows KISS-ICP,
so it is as good as KISS-ICP is. On the outdoor terrain in gz that was
0.28 - 0.48 % of path against 0.15 - 1.25 % for wheels + gyro; in Isaac
1.2 - 1.3 % against 0.6 - 1.2 %, with heading error cut from 1 - 1.6 deg to
0.02 - 0.13 deg. On featureless ground the relay never trusts the lidar and
the result equals wheels + gyro exactly. What the relay cannot catch is a
slow scale error, since heading and each step still agree: in gz a
noise-free lidar at the full 10 Hz made KISS-ICP under-count distance by
up to 15 %, and the fused estimate inherited it. The gz Ouster now has the
1 cm range noise a real one has.

KISS-ICP is built from source; see ranger_xarm.repos.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    def ekf_and_lidar(context):
        truthy = lambda n: LaunchConfiguration(n).perform(context).lower() in ('true', '1', 'yes')
        num = lambda n: float(LaunchConfiguration(n).perform(context))
        use_sim_time = truthy('use_sim_time')
        extra = {'use_sim_time': use_sim_time}
        nodes = []
        if truthy('lidar_odometry'):
            # odom1 is only configured when the lidar chain is running, so a
            # stray /kiss/pose can never be fused by accident.
            extra.update({
                'odom1': '/kiss/pose',
                #      x     y     z      r      p      yaw
                'odom1_config': [True, True, False, False, False, True,
                                 False, False, False, False, False, False,
                                 False, False, False],
                'odom1_differential': False,
                'odom1_relative': False,
                'odom1_queue_size': 5,
                'odom1_nodelay': True,
                'odom1_pose_rejection_threshold': num('lidar_rejection_threshold'),
                # Each pose is applied at its scan's time, which is in the
                # filter's past by KISS-ICP's processing time; applied at the
                # current time instead it drags the estimate backwards.
                'smooth_lagged_data': True,
                'history_length': 2.0,
            })
            nodes.append(Node(
                package='kiss_icp',
                executable='kiss_icp_node',
                name='kiss_icp_node',
                output='screen',
                remappings=[('pointcloud_topic', LaunchConfiguration('lidar_topic'))],
                parameters=[{
                    'use_sim_time': use_sim_time,
                    # Report the pose of the EKF's base frame.
                    'base_frame': 'base_footprint',
                    'lidar_odom_frame': 'odom_lidar',
                    'publish_odom_tf': False,
                    'publish_debug_clouds': False,
                    # The Ouster sees its own mount and the gantry top within
                    # 0.75 m (measured in simulation with the arm parked);
                    # those points move with the robot and would pull ICP
                    # towards 'not moving'. A raised arm may reach further.
                    'data.min_range': num('lidar_min_range'),
                    'data.max_range': num('lidar_max_range'),
                    # Uses the Ouster's per-point 't' field; clouds without
                    # one (the simulators) are simply not deskewed.
                    'data.deskew': True,
                }],
            ))
            nodes.append(Node(
                package='ranger_xarm_bringup',
                executable='lidar_odometry_relay.py',
                name='lidar_odometry_relay',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'odom_in': '/kiss/odometry',
                    'odom_out': '/kiss/pose',
                    'ekf_odom': LaunchConfiguration('output_topic').perform(context),
                    'cloud': LaunchConfiguration('lidar_topic').perform(context),
                    'odom_frame': 'odom',
                    'base_frame': 'base_footprint',
                    'position_covariance': num('lidar_position_covariance'),
                    'yaw_covariance': num('lidar_yaw_covariance'),
                    'min_range': num('lidar_min_range'),
                    'max_range': num('lidar_max_range'),
                    'min_structure': num('lidar_min_structure'),
                }],
            ))
        nodes.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name=PythonExpression([
                "'ekf_odom_imu_3d' if '", LaunchConfiguration('fuse_attitude'),
                "'.lower() in ('true','1','yes') else 'ekf_odom_imu'"]),
            output='screen',
            parameters=[params, extra],
            remappings=[('odometry/filtered', LaunchConfiguration('output_topic'))],
        ))
        return nodes

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
        DeclareLaunchArgument(
            'lidar_odometry', default_value='false',
            description='also run KISS-ICP on the Ouster and fuse its pose '
                        '(odom1).'),
        DeclareLaunchArgument('lidar_topic', default_value='/ouster/points'),
        DeclareLaunchArgument('lidar_min_range', default_value='0.8'),
        DeclareLaunchArgument('lidar_max_range', default_value='30.0'),
        DeclareLaunchArgument(
            'lidar_position_covariance', default_value='1e-4',
            description='m^2 on the anchored lidar x and y.'),
        DeclareLaunchArgument(
            'lidar_yaw_covariance', default_value='1e-4',
            description='rad^2 on the anchored lidar yaw.'),
        DeclareLaunchArgument(
            'lidar_min_structure', default_value='5.0',
            description='smaller eigenvalue of the horizontal-normal '
                        'information matrix below which the scene cannot '
                        'constrain lidar odometry. See lidar_odometry_relay.py.'),
        DeclareLaunchArgument(
            'lidar_rejection_threshold', default_value='1e9',
            description='Mahalanobis gate on the lidar pose, off by default: '
                        'the relay already checks every KISS-ICP step against '
                        'the EKF and the scene structure, and re-anchors '
                        'instead of rejecting.'),
        corrector,
        OpaqueFunction(function=ekf_and_lidar),
    ])
