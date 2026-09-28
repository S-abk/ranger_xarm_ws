#!/usr/bin/env python3
"""3D-lidar SLAM: lidar_terrain_filter.py + slam_toolbox (online async).

Run alongside the robot (or simulator) and the EKF, which owns
odom -> base_footprint; slam_toolbox adds map -> odom:

    ros2 launch ranger_xarm_bringup ekf_odom_imu.launch.py lidar_odometry:=true
    ros2 launch ranger_xarm_bringup slam.launch.py
    # then navigation.launch.py

slam_toolbox is a 2D graph SLAM, and it gets a 2D scan made from the 3D
Ouster by lidar_terrain_filter.py: points more than 15 cm above the local
ground, levelled by IMU tilt, so slopes and pitch do not appear as walls.
The same node publishes the obstacle cloud Nav2's costmaps use.

slam_toolbox is a lifecycle node in Jazzy: started as a plain node it sits
unconfigured and publishes nothing, silently. It is configured and then
activated here, the way slam_toolbox's own launch files do it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share = get_package_share_directory('ranger_xarm_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', namespace='', output='screen',
        parameters=[LaunchConfiguration('slam_params'),
                    {'use_sim_time': use_sim_time, 'use_lifecycle_manager': False}])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('lidar_topic', default_value='/ouster/points'),
        DeclareLaunchArgument('imu_topic', default_value='/ouster/imu'),
        DeclareLaunchArgument(
            'slam_params', default_value=os.path.join(share, 'config', 'slam_toolbox.yaml')),
        Node(
            package='ranger_xarm_bringup', executable='lidar_terrain_filter.py',
            name='lidar_terrain_filter', output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'cloud_in': LaunchConfiguration('lidar_topic'),
                         'imu_in': LaunchConfiguration('imu_topic')}]),
        slam,
        EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                    transition_id=Transition.TRANSITION_CONFIGURE)),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
            entities=[LogInfo(msg='slam_toolbox configured; activating'),
                      EmitEvent(event=ChangeState(
                          lifecycle_node_matcher=matches_action(slam),
                          transition_id=Transition.TRANSITION_ACTIVATE))])),
    ])
