#!/usr/bin/env python3
"""Publish robot_description and TF for the Isaac side.

Isaac is started separately, by its own interpreter, so this launch does
not start the simulator. It brings up the ROS half: the same xacro the gz
sim and the real robot use, expanded with the same kind of arguments, on
/robot_description.

That topic is the seam. Isaac's URDF importer can subscribe to a ROS 2
topic carrying the robot description rather than reading a file, which is
the same guarantee ranger_xarm_gazebo already relies on when it spawns
with `-topic robot_description`: what the simulator gets is byte for byte
what robot_state_publisher is using, so the TF tree and the simulated
model cannot drift apart.

    ros2 launch ranger_xarm_isaac description.launch.py
    ros2 launch ranger_xarm_isaac description.launch.py add_gripper:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    add_gripper = LaunchConfiguration('add_gripper').perform(context)
    fix_base = LaunchConfiguration('fix_base').perform(context)

    xacro_file = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_description'), 'urdf',
        'ranger_xarm.urdf.xacro'])

    # No ros2_control plugin argument here. Under gz the plugin is what
    # drives the joints; Isaac provides its own bridge instead, so the
    # description is expanded plain and the control layer is wired on the
    # Isaac side. Keeping that out of the xacro is what stops this
    # becoming a third variant of the model.
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' add_gripper:=', add_gripper,
            ' fix_base_to_world:=', fix_base,
        ]), value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # Isaac publishes joint states over its own ROS 2 bridge. Until that
    # is wired up there is nothing driving the movable joints, so the GUI
    # is here to supply them and keep TF complete rather than half a tree.
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('joint_gui')),
    )

    return [rsp, jsp]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('add_gripper', default_value='true',
                              description='Include the xArm gripper.'),
        DeclareLaunchArgument(
            'fix_base', default_value='true',
            description='Weld base_link to the world. True matches the '
                        'arm-only gz workflow; set false when the Isaac '
                        'side is driving a wheeled base.'),
        DeclareLaunchArgument(
            'joint_gui', default_value='false',
            description='Run joint_state_publisher_gui to supply joint '
                        'states until the Isaac bridge does.'),
        OpaqueFunction(function=launch_setup),
    ])
