#!/usr/bin/env python3
"""Bring up ros2_control against a running Isaac Sim.

Start Isaac first (it owns the physics and the clock):

    $ISAACSIM_PYTHON_EXE .../lib/ranger_xarm_isaac/isaac_bringup.py

then this:

    ros2 launch ranger_xarm_isaac control.launch.py

Nothing above the hardware interface differs from the gz path. The same
xacro, the same joint_trajectory_controllers, the same MoveIt config;
only the ros2_control plugin underneath changes, which is the seam
`xarm_ros2_control_plugin` already exists to move. Under gz it is
gz_ros2_control/GazeboSimSystem, on hardware it is
uf_robot_hardware/UFRobotSystemHardware, and here it is
joint_state_topic_hardware_interface/JointStateTopicSystem, which
exchanges joint states and commands with Isaac over two topics.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PLUGIN = 'joint_state_topic_hardware_interface/JointStateTopicSystem'


def serialise(actions):
    """Chain actions so each starts only once the previous has exited.

    Returns the event handlers; the first action is started by the caller.
    """
    return [RegisterEventHandler(OnProcessExit(target_action=prev,
                                               on_exit=[nxt]))
            for prev, nxt in zip(actions, actions[1:])]


def launch_setup(context, *args, **kwargs):
    from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file

    add_gripper = LaunchConfiguration('add_gripper').perform(context)
    drive_base = LaunchConfiguration('drive_base').perform(context).lower() in ('true', '1', 'yes')
    use_suspension = LaunchConfiguration('use_suspension').perform(context)
    odom_tf = LaunchConfiguration('odom_tf').perform(context).lower() in ('true', '1', 'yes')
    prefix = 'xarm_'

    # Reuse xarm's own controller yaml rewriter, exactly as the gz launch
    # does, rather than keeping a forked copy that ages silently.
    controllers = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'),
                     'config', 'xarm6_controllers.yaml'),
        prefix=prefix,
        add_gripper=add_gripper.lower() in ('true', '1', 'yes'),
        ros_namespace='',
        update_rate=150,
        robot_type='xarm',
        use_sim_time=True,
    )

    # Same reasoning as the gz launch: a joint that claims both position
    # and velocity gets driven by whichever the hardware prefers, and the
    # ambiguity is not worth the trouble. Isaac's articulation controller
    # takes position targets, so claim position only.
    with open(controllers, 'r') as f:
        controllers_yaml = yaml.safe_load(f)
    arm = controllers_yaml['{}xarm6_traj_controller'.format(prefix)]
    arm['ros__parameters']['command_interfaces'] = ['position']
    if drive_base:
        with open(os.path.join(
                get_package_share_directory('ranger_xarm_gazebo'),
                'config', 'base_controllers.yaml'), 'r') as f:
            base_yaml = yaml.safe_load(f)
        for name, cfg in base_yaml['controller_manager']['ros__parameters'].items():
            controllers_yaml['controller_manager']['ros__parameters'][name] = cfg
        for name in ('ranger_steer_controller', 'ranger_wheel_controller'):
            controllers_yaml[name] = base_yaml[name]
        # Isaac drives wheels from a velocity drive, so the wheel controller
        # forwards speeds rather than torques. The gz path uses effort.
        controllers_yaml['controller_manager']['ros__parameters'][
            'ranger_wheel_controller']['type'] = \
            'velocity_controllers/JointGroupVelocityController'

    with open(controllers, 'w') as f:
        yaml.dump(controllers_yaml, f, default_flow_style=False)

    xacro_file = PathJoinSubstitution([
        FindPackageShare('ranger_xarm_description'), 'urdf',
        'ranger_xarm.urdf.xacro'])
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' xarm_ros2_control_plugin:=', PLUGIN,
            ' add_gripper:=', add_gripper,
            # No gz plugin block here; Isaac is not gz.
            ' xarm_load_gazebo_plugin:=false',
            ' use_wheels:=', 'true' if drive_base else 'false',
            ' use_suspension:=', use_suspension,
            ' fix_base_to_world:=', 'false' if drive_base else 'true',
            ' wheels_ros2_control_plugin:=', PLUGIN,
            ' wheels_command_interface:=velocity',
        ]), value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # os_sensor -> os_lidar, published the way the real Ouster driver does
    # (OS-series default: 36.18 mm up, yawed 180 deg; isaac_sensors.py's
    # OUSTER_LIDAR_Z). Isaac's own transform tree names frames after prims,
    # and the lidar's prim is called 'sensor', so without this nothing
    # connects /ouster/points (frame os_lidar) to the robot, and anything
    # that looks the frame up falls back to the lidar's own, yawed 180 deg.
    ouster_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='os_lidar_static_tf', output='log',
        arguments=['--z', '0.03618', '--yaw', '3.141592653589793',
                   '--frame-id', 'os_sensor', '--child-frame-id', 'os_lidar'],
        parameters=[{'use_sim_time': True}],
    )

    # d435_link -> the optical frames, as the RealSense driver publishes them
    # (+Z forward, +X right, +Y down). Colour and depth share one render
    # product in Isaac, so they share one pose.
    d435_tfs = [Node(
        package='tf2_ros', executable='static_transform_publisher',
        name=f'{frame}_static_tf', output='log',
        arguments=['--roll', '-1.5707963267948966', '--yaw', '-1.5707963267948966',
                   '--frame-id', 'd435_link', '--child-frame-id', frame],
        parameters=[{'use_sim_time': True}],
    ) for frame in ('d435_color_optical_frame', 'd435_depth_optical_frame')]

    # Isaac publishes /clock through the bridge, so everything here runs
    # on sim time and stays aligned with the physics step.
    control_node = Node(
        package='controller_manager', executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True},
                    controllers],
    )

    def spawner(name):
        # Longer than the defaults on purpose. The controller_manager is
        # waiting on Isaac's first joint states before its hardware reads
        # successfully, and Isaac is slow to that point.
        #
        # These are wall-clock timeouts, so they also have to cover the
        # case where Isaac is running far below real time: at
        # --physics-dt 0.001 the 30 s service-call timeout expires, the
        # spawner retries a load_controller the manager has already
        # serviced, and the retry fails to configure. The controller is
        # then simply absent, the wheels never steer, and the run looks
        # like broken kinematics rather than a timeout.
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '180',
                               '--service-call-timeout', '120',
                               '--switch-timeout', '120'],
                    output='screen')

    # Isaac fills the effort field; the arm description declares no effort
    # state interface, and the hardware interface throws rather than
    # skipping it. See the node's docstring.
    effort_filter = Node(
        package='ranger_xarm_isaac', executable='joint_state_effort_filter.py',
        output='screen', parameters=[{'use_sim_time': True}],
    )

    # The arm and gripper components share one command topic and take
    # turns on it, so Isaac only ever sees half the robot. Merge by joint
    # name and republish the union. See the node's docstring.
    command_merger = Node(
        package='ranger_xarm_isaac', executable='joint_command_merger.py',
        output='screen', parameters=[{'use_sim_time': True}],
    )

    # Collision ground truth from Isaac's PhysX contact reports, which
    # isaac_bringup.py sends over UDP (see isaac_contacts.py).
    contact_relay = Node(
        package='ranger_xarm_isaac', executable='isaac_contact_relay.py',
        output='screen', parameters=[{'use_sim_time': True}],
    )

    jsb = spawner('joint_state_broadcaster')
    arm_ctrl = spawner('{}xarm6_traj_controller'.format(prefix))
    gripper_ctrl = spawner('{}xarm_gripper_traj_controller'.format(prefix))

    # Spawners only: serialise() chains on process EXIT, so a
    # long-running node in this list stalls the chain and everything
    # behind it never starts.
    after_jsb = [arm_ctrl]
    if add_gripper.lower() in ('true', '1', 'yes'):
        after_jsb.append(gripper_ctrl)
    base_nodes = []
    if drive_base:
        after_jsb += [
            spawner('ranger_steer_controller'),
            spawner('ranger_wheel_controller'),
        ]
        base_nodes = [
            Node(package='ranger_xarm_gazebo',
                 executable='ranger_4wis_controller.py', output='screen',
                 # The steering convergence gate runs here as in gz, at its
                 # default (common mode): it removes Isaac's real crab
                 # scrub (+2.5 deg of unwanted yaw down to ~0). The wheel
                 # acceleration limit also runs at its default (1.0 m/s^2),
                 # as in gz; in Isaac it halved flat-ground wheel odometry
                 # error (0.17-0.25 % of path to 0.08-0.12 %).
                 parameters=[{'use_sim_time': True,
                              'command_mode': 'velocity'}]),
            # Dead reckoning from the wheel encoders, which is what the
            # real platform can know. Isaac's own chassis pose is ground
            # truth and publishes on /ground_truth/odom instead.
            Node(package='ranger_xarm_gazebo',
                 executable='wheel_odometry.py', output='screen',
                 parameters=[{'use_sim_time': True, 'publish_tf': odom_tf}]),
        ]

    return [
        rsp, ouster_tf, *d435_tfs, effort_filter, command_merger, contact_relay,
        control_node, jsb,
        # One spawner at a time. Firing them together on the same event had
        # four of them calling load_controller and configure on a
        # single-threaded controller_manager at once, and they interfered:
        # timed-out calls were retried against a manager that had already
        # serviced them ("Controller already loaded, skipping
        # load_controller" then "Failed to configure controller"), or a
        # configure arrived before its own load ("Could not configure
        # controller ... because no controller with this name exists").
        # Which controllers survived varied between runs.
        #
        # The failure is quiet where it matters: the launch carries on and
        # the missing controller is simply absent. With
        # ranger_steer_controller gone the wheels never leave 0 deg, so a
        # crab command drives the robot straight and a spin barely rotates
        # it, which reads as broken kinematics rather than a startup race.
        *serialise([jsb] + after_jsb),
        *base_nodes,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('add_gripper', default_value='true',
                              description='Include the xArm gripper.'),
        DeclareLaunchArgument('use_suspension', default_value='true',
                              description='Sprung wheel corners (estimated '
                                          'values, see ranger_wheels.xacro). '
                                          'Must match the USD: generate it '
                                          'with the same use_suspension.'),
        DeclareLaunchArgument('drive_base', default_value='false',
                              description='Add the 4WIS wheels and drive the '
                                          'base from /cmd_vel.'),
        DeclareLaunchArgument('odom_tf', default_value='true',
                              description='Let wheel odometry publish '
                                          'odom -> base_footprint. Set false '
                                          'when another node owns that edge. '
                                          'Two publishers on one edge is not '
                                          'reported as an error; TF simply '
                                          'returns whichever arrived last.'),
        OpaqueFunction(function=launch_setup),
    ])
