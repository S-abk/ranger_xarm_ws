#!/usr/bin/env python3
"""Load the platform into Isaac and bridge its joints to ROS 2.

Run under Isaac's interpreter, with the workspace sourced:

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    $ISAACSIM_PYTHON_EXE $(ros2 pkg prefix ranger_xarm_isaac)/lib/\
ranger_xarm_isaac/isaac_bringup.py

This is the simulator half. The ROS half (controller_manager, the
trajectory controllers, MoveIt) comes up separately via
control.launch.py, and the two meet at two topics:

    isaac_joint_states    Isaac  -> ros2_control    (measured)
    isaac_joint_commands  ros2_control -> Isaac     (commanded)

The hardware interface itself uses its OWN default topic names,
robot_joint_states and robot_joint_commands: the xarm ros2_control xacro
is vendored and offers no way to add a <param> to its <hardware> block,
so the topics cannot be overridden from the description. Two small
relays in control.launch.py sit between the pairs
(joint_state_effort_filter.py and joint_command_merger.py; see their
docstrings for why each is needed).

Which means nothing above the hardware interface changes between gz and
Isaac. MoveIt still talks to joint_trajectory_controller, which still
talks to ros2_control; only the plugin underneath it differs, the same
way `xarm_ros2_control_plugin` already swaps between the real arm and
gz. The graph below is NVIDIA's own MoveIt sample topology
(standalone_examples/api/isaacsim.ros2.bridge/moveit.py), not an
invention.

The articulation is ticked by an OnImpulseEvent fired once per step from
the loop at the bottom, rather than OnPlaybackTick, so publish and
subscribe stay in lockstep with physics instead of with the renderer.
"""
import argparse
import os
import sys

try:
    from ament_index_python.packages import get_package_share_directory
except ModuleNotFoundError:
    sys.exit(
        "ament_index_python not found.\n"
        "Isaac's python.sh does not source your ROS workspace. Source it\n"
        "first, in the same shell:\n"
        "    source /opt/ros/jazzy/setup.bash && source install/setup.bash"
    )

ap = argparse.ArgumentParser()
ap.add_argument('--usd', default=None, help='robot USD (default: the generated one)')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--stage-path', default='/ranger_xarm')
ap.add_argument('--commands-topic', default='isaac_joint_commands')
ap.add_argument('--states-topic', default='isaac_joint_states')
ap.add_argument('--no-sensors', action='store_true',
                help='skip the lidars, IMU and camera. They each cost a render '
                     'product every frame, so turn them off when only the '
                     'control loop matters.')
ap.add_argument('--physics-dt', type=float, default=None,
                help='physics timestep in seconds. Isaac defaults to 1/60, '
                     'which is 16.7x coarser than the 1 ms the gz worlds use; '
                     'pass 0.001 to compare like with like.')
ap.add_argument('--wheel-mu', type=float, default=None,
                help="friction for the tyre colliders. The URDF importer "
                     "cannot carry the xacro's gz-only friction tags, so "
                     "without this the wheels run on PhysX's default 0.5 "
                     "while the gz side of the same xacro gets 1.2.")
ap.add_argument('--scene', default=None,
                help='an environment on top of the ground plane: hospital, office, '
                     'warehouse, warehouse_small, simple_room (NVIDIA samples, referenced from '
                     'the Isaac assets server, see isaac_scenes.py) or a USD path / URL.')
ap.add_argument('--spawn', default=None,
                help='x,y,yaw_deg for the robot (world frame); default the origin. Sample '
                     'scenes do not keep the origin clear.')
ap.add_argument('--no-contacts', action='store_true',
                help='do not report what the robot touches (isaac_contacts.py; '
                     'published as /ground_truth/contacts by isaac_contact_relay.py).')
ap.add_argument('--contacts-port', type=int, default=47811)
ap.add_argument('--wheel-damping', type=float, default=0.0,
                help='damping of the wheel velocity drives, USD units (per degree). '
                     '0 keeps the converter\'s 625. Raising it (10000, or a force '
                     'drive at 1500) did NOT stop a parked robot rolling back down an '
                     'incline (~20 mm/s either way); ranger_4wis_controller.py\'s '
                     'parking brake does.')
ap.add_argument('--wheel-max-torque', type=float, default=22.0,
                help='torque limit of each wheel drive, N m. The Ranger Mini 2.0 '
                     'datasheet gives 22 N m per drive motor (the 3.0 manual lists '
                     '350 W and a 1:4.428 reduction but no torque; 350 W at the 2 m/s '
                     'top speed is ~17.5 N m at the wheel). The URDF\'s 60 N m is a gz '
                     'setting. 0 keeps the USD\'s.')
ap.add_argument('--wheel-drive-type', choices=('keep', 'force', 'acceleration'), default='keep',
                help='drive type for the wheel joints; the converter gives acceleration '
                     'drives, whose torque scales with the joint\'s effective inertia.')
ap.add_argument('--steer-damping', type=float, default=6.6,
                help='damping of the steering position drives, USD units (per degree; '
                     'PhysX reads 57.3x that per radian). The converter gives '
                     'stiffness 625 with damping 0.05, a damping ratio of ~0.008 on '
                     'these acceleration drives: the knuckles rang at every target '
                     'change. 6.6 is about critical. 0 keeps the USD\'s.')
ap.add_argument('--print-drives', action='store_true',
                help='after the first physics steps, print the wheel and steering '
                     'drive gains and effort limits PhysX actually runs with.')
args = ap.parse_args()

try:
    isaac_share = get_package_share_directory('ranger_xarm_isaac')
except Exception as exc:
    sys.exit(f'cannot locate ranger_xarm_isaac ({exc}); is the workspace sourced?')

usd_path = args.usd or os.path.join(isaac_share, 'usd', 'ranger_xarm.usd')
if not os.path.exists(usd_path):
    sys.exit(
        f'no USD at {usd_path}\n'
        'Generate it first:\n'
        f'    $ISAACSIM_PYTHON_EXE {isaac_share}/../../lib/ranger_xarm_isaac/urdf_to_usd.py'
    )

from isaacsim import SimulationApp  # noqa: E402  (must precede any omni import)

simulation_app = SimulationApp({
    'renderer': 'RaytracedLighting',
    'headless': args.headless,
})

import omni.graph.core as og  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.utils import extensions, prims, stage  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, Usd, UsdLux, UsdPhysics  # noqa: E402

extensions.enable_extension('isaacsim.ros2.bridge')
simulation_app.update()

if args.physics_dt is not None:
    # rendering_dt has to track physics_dt, not stay at 1/60. The
    # OmniGraph that carries joint commands in and joint states out ticks
    # on OnPlaybackTick, i.e. per rendered frame. Leaving rendering at
    # 1/60 while physics runs at 1 kHz gives the articulation 16 physics
    # steps per command update, and the base simply thrashes: ground
    # truth stops moving while the wheel joints report tens of rad/s.
    simulation_context = SimulationContext(
        stage_units_in_meters=1.0,
        physics_dt=args.physics_dt, rendering_dt=args.physics_dt)
else:
    simulation_context = SimulationContext(stage_units_in_meters=1.0)
print(f'[ranger_xarm_isaac] physics dt '
      f'{simulation_context.get_physics_dt():.6f} s '
      f'({1.0 / simulation_context.get_physics_dt():.0f} Hz)', flush=True)

prims.create_prim(args.stage_path, 'Xform', usd_path=usd_path)
simulation_app.update()
if args.scene or args.spawn:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import isaac_scenes
if args.spawn:
    _sx, _sy, _syaw = (float(v) for v in args.spawn.split(','))
    isaac_scenes.place_robot(stage.get_current_stage(), args.stage_path, _sx, _sy, _syaw)
    print(f'[ranger_xarm_isaac] robot at ({_sx}, {_sy}), yaw {_syaw} deg', flush=True)


def find_articulation_root(stage_obj, under):
    """Locate the articulation beneath the referenced robot.

    The URDF importer nests it (here at <root>/world/world) rather than
    putting it on the top prim the way Isaac's own sample robots do, so
    pointing the graph at the reference path gives the unhelpful
    "Prim ... is not an articulation". Search for it instead of hardcoding
    a path that moves whenever the description's root changes.
    """
    root = stage_obj.GetPrimAtPath(under)
    if not root or not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    return None


articulation_path = find_articulation_root(stage.get_current_stage(), args.stage_path)
if articulation_path is None:
    print(f'[ranger_xarm_isaac] no articulation found under {args.stage_path}. '
          'Regenerate the USD with urdf_to_usd.py; an older one may lack a '
          'defaultPrim, which makes the reference resolve to nothing.',
          flush=True)
    simulation_app.close()
    sys.exit(1)
print(f'[ranger_xarm_isaac] articulation: {articulation_path}', flush=True)

# The converted USD carries the robot only. Ground and light belong to
# the scene, the same split ranger_xarm_gazebo keeps between the model
# and empty_ground.sdf.
current_stage = stage.get_current_stage()
PhysicsSchemaTools.addGroundPlane(
    current_stage, '/groundPlane', 'Z', 100.0,
    Gf.Vec3f(0.0, 0.0, 0.0), Gf.Vec3f(0.6))
light = UsdLux.DistantLight.Define(current_stage, '/DistantLight')
light.CreateIntensityAttr(1500)

# Tyre friction is physics, not sensing, so it goes in before the
# --no-sensors guard: it has to apply with the render products switched
# off.
if args.wheel_mu is not None:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import isaac_sensors as _friction
    _friction.apply_wheel_friction(current_stage, args.wheel_mu)

if args.scene:
    _url = isaac_scenes.scene_url(args.scene)
    print(f'[ranger_xarm_isaac] scene {args.scene}: {_url}', flush=True)
    isaac_scenes.add_scene(current_stage, _url)
    for _ in range(20):                      # let the references resolve
        simulation_app.update()
    isaac_scenes.report(current_stage)

simulation_app.update()

og.Controller.edit(
    {'graph_path': '/ActionGraph', 'evaluator_name': 'execution'},
    {
        og.Controller.Keys.CREATE_NODES: [
            ('OnImpulseEvent', 'omni.graph.action.OnImpulseEvent'),
            ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
            ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
            ('PublishJointState', 'isaacsim.ros2.bridge.ROS2PublishJointState'),
            ('SubscribeJointState', 'isaacsim.ros2.bridge.ROS2SubscribeJointState'),
            ('ArticulationController', 'isaacsim.core.nodes.IsaacArticulationController'),
            ('PublishClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
            # A mobile base needs its pose out. Isaac publishes joint
            # states but nothing about where the chassis actually is, so
            # without this there is no way to tell a base that drove two
            # metres from one that spun its wheels in place.
            ('ComputeOdometry', 'isaacsim.core.nodes.IsaacComputeOdometry'),
            ('PublishOdometry', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
        ],
        og.Controller.Keys.CONNECT: [
            ('OnImpulseEvent.outputs:execOut', 'PublishJointState.inputs:execIn'),
            ('OnImpulseEvent.outputs:execOut', 'SubscribeJointState.inputs:execIn'),
            ('OnImpulseEvent.outputs:execOut', 'PublishClock.inputs:execIn'),
            ('OnImpulseEvent.outputs:execOut', 'ArticulationController.inputs:execIn'),
            ('Context.outputs:context', 'PublishJointState.inputs:context'),
            ('Context.outputs:context', 'SubscribeJointState.inputs:context'),
            ('Context.outputs:context', 'PublishClock.inputs:context'),
            ('ReadSimTime.outputs:simulationTime', 'PublishJointState.inputs:timeStamp'),
            ('ReadSimTime.outputs:simulationTime', 'PublishClock.inputs:timeStamp'),
            ('SubscribeJointState.outputs:jointNames',
             'ArticulationController.inputs:jointNames'),
            ('SubscribeJointState.outputs:positionCommand',
             'ArticulationController.inputs:positionCommand'),
            ('SubscribeJointState.outputs:velocityCommand',
             'ArticulationController.inputs:velocityCommand'),
            ('SubscribeJointState.outputs:effortCommand',
             'ArticulationController.inputs:effortCommand'),
            ('OnImpulseEvent.outputs:execOut', 'ComputeOdometry.inputs:execIn'),
            ('ComputeOdometry.outputs:execOut', 'PublishOdometry.inputs:execIn'),
            ('Context.outputs:context', 'PublishOdometry.inputs:context'),
            ('ReadSimTime.outputs:simulationTime', 'PublishOdometry.inputs:timeStamp'),
            ('ComputeOdometry.outputs:position', 'PublishOdometry.inputs:position'),
            ('ComputeOdometry.outputs:orientation', 'PublishOdometry.inputs:orientation'),
            ('ComputeOdometry.outputs:linearVelocity',
             'PublishOdometry.inputs:linearVelocity'),
            ('ComputeOdometry.outputs:angularVelocity',
             'PublishOdometry.inputs:angularVelocity'),
        ],
        og.Controller.Keys.SET_VALUES: [
            ('ArticulationController.inputs:robotPath', articulation_path),
            ('PublishJointState.inputs:topicName', args.states_topic),
            ('SubscribeJointState.inputs:topicName', args.commands_topic),
            ('PublishJointState.inputs:targetPrim', [usdrt.Sdf.Path(articulation_path)]),
            ('ComputeOdometry.inputs:chassisPrim', [usdrt.Sdf.Path(articulation_path)]),
            # GROUND TRUTH, and named as such. IsaacComputeOdometry reads
            # the chassis pose out of the physics engine, so it never
            # slips, never drifts and never accumulates heading error.
            # This used to be published as /odom, which meant anything
            # downstream believed it had odometry when what it had was
            # the answer. Dead reckoning from the wheel encoders is
            # wheel_odometry.py, and that is what owns /odom now.
            ('PublishOdometry.inputs:topicName', 'ground_truth/odom'),
            ('PublishOdometry.inputs:odomFrameId', 'ground_truth_odom'),
            ('PublishOdometry.inputs:chassisFrameId', 'base_link_ground_truth'),
        ],
    },
)

# Sensors. Kept in their own module and their own graph: they publish
# per rendered frame rather than per controller tick, and the control
# bridge has to keep working when they are switched off.
if not args.no_sensors:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import isaac_sensors as sensors

    sensors.register_config_dir(
        os.path.join(isaac_share, 'config', 'lidar'))
    current_stage = stage.get_current_stage()

    # No translation: NVIDIA's Ouster asset already puts its sensor prim
    # OUSTER_LIDAR_Z above its own root. Passing the offset here as well
    # put the scan origin 72 mm above os_sensor instead of 36 mm.
    ouster = sensors.add_rtx_lidar(
        current_stage, 'os_sensor', 'os_lidar', sensors.OUSTER_CONFIG,
        yaw_deg=180.0)
    ouster_imu = sensors.add_imu(current_stage, 'os_sensor', 'os_imu')
    rplidar = sensors.add_rtx_lidar(
        current_stage, 'laser_frame', 'rplidar', sensors.RPLIDAR_BASE_CONFIG,
        profile_json=os.path.join(isaac_share, 'config', 'lidar',
                                  'RPLIDAR_A1M8.json'))
    d435 = sensors.add_camera(current_stage, 'd435_link', 'd435_camera')
    simulation_app.update()
    sensors.build_sensor_graph(current_stage, ouster, ouster_imu, rplidar,
                               d435, articulation_path)
    simulation_app.update()

# Drive damping (see --wheel-damping, --steer-damping). Before physics
# starts, like the contact reports below.
if args.wheel_damping > 0 or args.steer_damping > 0 or args.wheel_max_torque > 0:
    _n = {'wheel': 0, 'steer': 0}
    for _prim in Usd.PrimRange(stage.get_current_stage().GetPrimAtPath(args.stage_path)):
        _name = _prim.GetName()
        _kind = ('wheel' if _name.endswith('_wheel_joint') else
                 'steer' if _name.endswith('_steer_joint') else None)
        _value = args.wheel_damping if _kind == 'wheel' else args.steer_damping
        if _kind is None:
            continue
        _drive = UsdPhysics.DriveAPI.Get(_prim, 'angular')
        if _drive:
            if _value > 0:
                _drive.CreateDampingAttr(float(_value))
            if _kind == 'wheel' and args.wheel_drive_type != 'keep':
                _drive.CreateTypeAttr(args.wheel_drive_type)
            if _kind == 'wheel' and args.wheel_max_torque > 0:
                _drive.CreateMaxForceAttr(float(args.wheel_max_torque))
            _n[_kind] += 1
    print(f'[ranger_xarm_isaac] drive damping: wheels {args.wheel_damping:g} '
          f'({_n["wheel"]}, type {args.wheel_drive_type}, max {args.wheel_max_torque:g} N m), '
          f'steering {args.steer_damping:g} ({_n["steer"]})', flush=True)

# Collision ground truth. Applied before physics starts: the contact
# report API is read when the bodies are created.
contacts = None
if not args.no_contacts:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import isaac_contacts
    contacts = isaac_contacts.ContactReporter(
        stage.get_current_stage(), args.stage_path, port=args.contacts_port)

simulation_app.update()
simulation_context.initialize_physics()
simulation_context.play()
physics_dt = simulation_context.get_physics_dt()

print(f'[ranger_xarm_isaac] bridging {args.states_topic} / {args.commands_topic}',
      flush=True)

_steps = 0
while simulation_app.is_running():
    simulation_context.step(render=True)
    _steps += 1
    if args.print_drives and _steps == 30:
        from isaacsim.core.prims import Articulation as _Art
        _a = _Art(articulation_path)
        _a.initialize()
        _kp, _kd = _a.get_gains()
        _fmax = _a.get_max_efforts()
        for _k, _dn in enumerate(_a.dof_names):
            if 'wheel' in _dn or 'steer' in _dn:
                print(f'[drives] {_dn:26s} stiffness {float(_kp[0][_k]):10.3f} damping '
                      f'{float(_kd[0][_k]):10.3f} max effort {float(_fmax[0][_k]):8.2f}', flush=True)
    if contacts is not None:
        contacts.step(simulation_context.current_time, physics_dt)
    # Fire the graph once per physics step; without this the publish and
    # subscribe nodes never execute and the bridge is silently dead.
    og.Controller.set(
        og.Controller.attribute('/ActionGraph/OnImpulseEvent.state:enableImpulse'), True)

simulation_context.stop()
simulation_app.close()
