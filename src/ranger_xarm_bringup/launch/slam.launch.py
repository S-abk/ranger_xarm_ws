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

mode:=localization navigates on a saved map without changing it: map a
site once, save it with

    ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph "{filename: '/path/site'}"

then relaunch with mode:=localization map_file:=/path/site and
map_start_pose:='[x, y, yaw]' (the robot's pose in that map). Mapping mode
kept running through long Nav2 sessions ended with two rotated copies of
the room, so navigation tests use localization.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def slam_nodes(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    mode = LaunchConfiguration('mode').perform(context)
    params = {'use_sim_time': use_sim_time, 'use_lifecycle_manager': False}
    if mode == 'localization':
        start = [float(v) for v in
                 LaunchConfiguration('map_start_pose').perform(context).strip('[] ').split(',')]
        params.update({'mode': 'localization',
                       'map_file_name': LaunchConfiguration('map_file').perform(context),
                       'map_start_pose': start})
        executable = 'localization_slam_toolbox_node'
    elif mode == 'mapping':
        executable = 'async_slam_toolbox_node'
    else:
        raise RuntimeError(f"mode must be 'mapping' or 'localization', not {mode!r}")
    slam = LifecycleNode(
        package='slam_toolbox', executable=executable, name='slam_toolbox', namespace='',
        output='screen', parameters=[LaunchConfiguration('slam_params').perform(context), params])
    return [
        slam,
        EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                    transition_id=Transition.TRANSITION_CONFIGURE)),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
            entities=[LogInfo(msg=f'slam_toolbox ({mode}) configured; activating'),
                      EmitEvent(event=ChangeState(
                          lifecycle_node_matcher=matches_action(slam),
                          transition_id=Transition.TRANSITION_ACTIVATE))])),
    ]


def generate_launch_description():
    share = get_package_share_directory('ranger_xarm_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('lidar_topic', default_value='/ouster/points'),
        DeclareLaunchArgument('imu_topic', default_value='/ouster/imu'),
        DeclareLaunchArgument(
            'slam_params', default_value=os.path.join(share, 'config', 'slam_toolbox.yaml')),
        DeclareLaunchArgument('mode', default_value='mapping',
                              description="'mapping' or 'localization'"),
        DeclareLaunchArgument('map_file', default_value='',
                              description='serialized pose graph (no extension), localization'),
        DeclareLaunchArgument('map_start_pose', default_value='[0.0, 0.0, 0.0]',
                              description='robot pose in the saved map, localization'),
        Node(
            package='ranger_xarm_bringup', executable='lidar_terrain_filter.py',
            name='lidar_terrain_filter', output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'cloud_in': LaunchConfiguration('lidar_topic'),
                         'imu_in': LaunchConfiguration('imu_topic')}]),
        OpaqueFunction(function=slam_nodes),
    ])
