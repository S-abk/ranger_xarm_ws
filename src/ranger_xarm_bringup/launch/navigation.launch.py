#!/usr/bin/env python3
"""Nav2 for the Ranger Mini V3 (4WIS): planner, MPPI, behaviours, smoother,
collision monitor, BT navigator and waypoint follower.

    ros2 launch ranger_xarm_bringup navigation.launch.py

Needs map -> odom (slam.launch.py) and odom -> base_footprint (the EKF).
The command chain matches nav2_bringup's: controller and behaviours ->
cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> collision_monitor
-> cmd_vel, which ranger_4wis_controller.py consumes. nav2_bringup's own
navigation_launch.py is not used because Jazzy's also starts the route and
docking servers, whose default configs expect files this robot does not
have, and one lifecycle node failing to activate aborts the whole stack.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODES = [
    # (package, executable, name, extra remappings)
    ('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_smoother', 'smoother_server', 'smoother_server', []),
    ('nav2_planner', 'planner_server', 'planner_server', []),
    ('nav2_behaviors', 'behavior_server', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
     [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_collision_monitor', 'collision_monitor', 'collision_monitor', []),
    ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
    ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', []),
]


def generate_launch_description():
    share = get_package_share_directory('ranger_xarm_bringup')
    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nodes = [Node(package=pkg, executable=exe, name=name, output='screen',
                  parameters=[params, {'use_sim_time': use_sim_time}],
                  remappings=remap)
             for pkg, exe, name, remap in NODES]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        *nodes,
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': [n[2] for n in NODES]}]),
    ])
