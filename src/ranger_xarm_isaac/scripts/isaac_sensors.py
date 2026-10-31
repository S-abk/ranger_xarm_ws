#!/usr/bin/env python3
"""Attach the platform's sensors to the robot in Isaac and bridge them.

The description already carries the sensor links, their meshes and their
CAD-derived mount poses, but it carries no <sensor> tags at all, so
neither simulator produced a single reading. This module fills that in
for Isaac: an Ouster OS0 and its built-in IMU on the gantry, an RPLIDAR
A1M8 on the front deck, a RealSense D455 on the gantry and a RealSense D435
on the xArm wrist (it moves with joint 6).

The topics and frame ids deliberately match what the REAL drivers
publish, because ranger_xarm_sensors already defines that contract and
everything downstream is written against it. Same topic, same frame, so
nothing above this layer can tell which one it is talking to.

    Ouster OS0    /ouster/points      os_lidar
                  /ouster/imu         os_imu
    RPLIDAR A1M8  /scan               laser_frame
    Gantry D455   /camera/gantry_d455/color/image_raw       gantry_d455_color_optical_frame
                  /camera/gantry_d455/depth/image_rect_raw  gantry_d455_depth_optical_frame
                  .../camera_info
    Wrist D435    /camera/wrist_d435/color/image_raw        wrist_d435_color_optical_frame
                  /camera/wrist_d435/depth/image_rect_raw   wrist_d435_depth_optical_frame
                  .../camera_info

One consequence is worth stating, because it looks like an omission. The
sensor-internal frames (os_lidar, os_imu, the optical frames) are NOT
added to the URDF. On hardware they belong to the drivers: the Ouster
publishes them from its own metadata, the RealSense from its own
calibration, and ranger_xarm_sensors is built around that split, which is
why the *_mount_link frames exist in the description and nothing below
them does.
Putting them in the URDF would duplicate the driver's transforms the
moment the real sensor is plugged in. So Isaac publishes them too, as a
static transform tree hanging off the mount links, which is the same
thing the driver does.

The Ouster mounting offsets are the OS-series defaults: the lidar origin
sits 36.18 mm above the sensor origin and is yawed 180 degrees. The
authoritative values for a specific unit come out of that sensor's own
metadata JSON, which is exactly the file this workspace deliberately does
not commit, so these are the family defaults rather than this unit's
calibration. For a simulator that is the right trade; for comparing
against a recording from the real sensor it is not, and the metadata
should be read instead.
"""

import math

import omni.kit.commands
import omni.graph.core as og
from pxr import Gf, Sdf, UsdGeom

# Ouster OS-series: lidar frame is above the sensor origin and yawed 180.
# NVIDIA's asset carries this offset itself, and control.launch.py publishes
# os_sensor -> os_lidar with it, as the real driver does.
OUSTER_LIDAR_Z = 0.03618
# Isaac ships genuine Ouster profiles; this is the OS0 at 128 channels,
# 10 Hz, 1024 azimuth steps, which is the platform's configuration.
OUSTER_CONFIG = 'OS0_REV7_128ch10hz1024res'
# There is no A1M8 among Isaac's lidars, so the A1M8 is built by taking a
# genuine 2D prim and writing this platform's numbers onto it. See
# _apply_profile for why it cannot simply be named.
RPLIDAR_BASE_CONFIG = 'Example_Rotary_2D'

# RealSense depth stream size. The colour stream runs at 1280x720 (D435) or
# up to 1280x800 (D455) on hardware, but both are rendered here and each
# render product costs GPU every frame, so they are matched at the depth
# resolution instead. Used for both cameras.
CAMERA_WIDTH, CAMERA_HEIGHT = 848, 480


def register_config_dir(path):
    """Let the RTX lidar find configs shipped by this package.

    Isaac resolves a lidar config NAME by searching the folders in
    /app/sensors/nv/lidar/profileBaseFolder. Ours is not one of them
    until it is appended, and the failure if it is missing is a lidar
    that creates successfully and then returns nothing.
    """
    import carb
    settings = carb.settings.get_settings()
    key = '/app/sensors/nv/lidar/profileBaseFolder'
    folders = list(settings.get(key) or [])
    if path not in folders:
        folders.append(path)
        settings.set(key, folders)
    return folders


def _find_link(stage, name):
    """The prim for a URDF link, wherever the importer put it."""
    for prim in stage.Traverse():
        if prim.GetName() == name:
            return prim.GetPath().pathString
    return None


# JSON profile key -> USD attribute on an OmniLidar prim.
_PROFILE_ATTRS = {
    'nearRangeM': 'omni:sensor:Core:nearRangeM',
    'farRangeM': 'omni:sensor:Core:farRangeM',
    'rangeAccuracyM': 'omni:sensor:Core:rangeAccuracyM',
    'rangeResolutionM': 'omni:sensor:Core:rangeResolutionM',
    'scanRateBaseHz': 'omni:sensor:Core:scanRateBaseHz',
    'reportRateBaseHz': 'omni:sensor:Core:reportRateBaseHz',
    'azimuthErrorMean': 'omni:sensor:Core:azimuthErrorMean',
    'azimuthErrorStd': 'omni:sensor:Core:azimuthErrorStd',
    'elevationErrorMean': 'omni:sensor:Core:elevationErrorMean',
    'elevationErrorStd': 'omni:sensor:Core:elevationErrorStd',
}


def _apply_profile(prim, profile):
    """Write a JSON lidar profile onto an OmniLidar prim.

    Isaac 5.x configures RTX lidars from USD assets carrying
    omni:sensor:Core:* attributes, selected by name out of a fixed table
    of supported configs. A bare JSON file in profileBaseFolder is only
    consulted for the DEPRECATED camera-prim path. So naming a config
    that is not in that table does not fail: it silently falls back to a
    generic rotary lidar, and the first sign is a 32-beam elevation fan
    where a single-plane scanner should be, with
    IsaacComputeRTXLidarFlatScan refusing to run because the prim "is not
    a 2D Lidar".

    Hence this: start from a supported prim that is genuinely the right
    SHAPE of sensor, then write the real numbers over it.
    """
    applied, missing = [], []
    for key, attr_name in _PROFILE_ATTRS.items():
        if key not in profile:
            continue
        attr = prim.GetAttribute(attr_name)
        if not attr:
            missing.append(attr_name)
            continue
        current = attr.Get()
        value = profile[key]
        # scanRateBaseHz and reportRateBaseHz are integral on the prim.
        # The A1M8's 5.5 Hz is therefore not representable; it is an
        # adjustable 2-10 Hz sensor so a whole number is still a setting
        # it genuinely has, but it is not the 5.5 Hz default.
        if isinstance(current, int) and not isinstance(current, bool):
            value = int(round(float(value)))
        else:
            value = float(value)
        attr.Set(value)
        applied.append(f'{key}={value}')
    if missing:
        print(f'[sensors]   no such attribute: {missing}', flush=True)
    print(f'[sensors]   profile: {", ".join(applied)}', flush=True)


def add_rtx_lidar(stage, link_name, child, config, translate=None, yaw_deg=0.0,
                  profile_json=None):
    """Create an RTX lidar as a child of a URDF link's prim."""
    parent = _find_link(stage, link_name)
    if parent is None:
        print(f'[sensors] no prim for link {link_name}; skipping {child}', flush=True)
        return None
    _, prim = omni.kit.commands.execute(
        'IsaacSensorCreateRtxLidar',
        path=child,
        parent=parent,
        config=config,
        translation=Gf.Vec3d(*(translate or (0.0, 0.0, 0.0))),
        orientation=Gf.Quatd(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_deg).GetQuat()),
    )
    print(f'[sensors] {config} -> {prim.GetPath()}', flush=True)
    if profile_json:
        import json
        with open(profile_json) as f:
            _apply_profile(prim, json.load(f)['profile'])
    return prim.GetPath().pathString


def add_imu(stage, link_name, child, translate=None):
    parent = _find_link(stage, link_name)
    if parent is None:
        print(f'[sensors] no prim for link {link_name}; skipping {child}', flush=True)
        return None
    _, prim = omni.kit.commands.execute(
        'IsaacSensorCreateImuSensor',
        path=child,
        parent=parent,
        translation=Gf.Vec3d(*(translate or (0.0, 0.0, 0.0))),
        orientation=Gf.Quatd(1, 0, 0, 0),
    )
    print(f'[sensors] imu -> {prim.GetPath()}', flush=True)
    return prim.GetPath().pathString


# Colour-stream intrinsics per camera model (D400 datasheet rev 021).
# focal = (aperture/2) / tan(HFOV/2), with Isaac's default 20.955 mm
# horizontal aperture for both, so only the focal length differs.
#   D435: 69 deg horizontal FOV (42 vertical)  -> 15.245 mm
#   D455: 90 deg horizontal FOV (65 vertical)  -> 10.4775 mm
# At the 848x480 render size the vertical FOV follows from the aspect
# ratio (D435 42.5, D455 58.6); the D455's datasheet 65 deg is its 16:10
# mode. Isaac's clip range stays at the add_camera default for both; the
# minimum-Z figures (D435 0.105 m, D455 0.18 m) are modelled in the gz
# sim only.
CAMERA_APERTURE = 20.955
D435_APERTURE = CAMERA_APERTURE
D435_FOCAL = 15.245
D455_APERTURE = CAMERA_APERTURE
D455_FOCAL = (D455_APERTURE / 2.0) / math.tan(math.radians(90.0 / 2.0))
CAMERA_MODELS = {
    'd435': {'horizontal_aperture': D435_APERTURE, 'focal_length': D435_FOCAL},
    'd455': {'horizontal_aperture': D455_APERTURE, 'focal_length': D455_FOCAL},
}


def add_camera(stage, link_name, child, model='d435',
               horizontal_aperture=None, focal_length=None):
    """A camera on a link, looking along the link's +X.

    `model` ('d435' or 'd455') selects the intrinsics from CAMERA_MODELS;
    explicit horizontal_aperture / focal_length override it. The camera
    prim is created as a CHILD of the link's prim, so on a link that moves
    with the articulation (the wrist D435 on the xArm flange) the camera
    follows it through the physics-driven link transform. The link is
    found by name (see _find_link): with merge_fixed_joints off, the
    massless wrist_d435_link and gantry_d455_link exist as prims.

    USD cameras look down their own -Z with +Y up, so the prim is turned
    +90 deg about X and then -90 about Z: -Z goes to the link's +X (forward,
    where the real camera points) and +Y to its +Z (up). The rotation used to
    be composed the other way round, which aimed the camera out of the
    robot's right side with its top edge forward.

    The prim is NOT the optical frame. Isaac's transform tree names frames
    after prims, and a USD camera's axes are 180 deg about X from a ROS
    optical frame's (+Z forward, +Y down), so a prim called
    <name>_color_optical_frame published a wrong frame under the right name.
    The images carry the optical frame ids, and control.launch.py publishes
    <name>_link -> the optical frames statically, as the RealSense driver
    does. The camera prims are therefore NOT given to the sensor transform
    tree in build_sensor_graph: a moving link there would be a second
    publisher of a frame robot_state_publisher already owns.

    One render product feeds both the colour and the depth stream, which
    is a simplification worth knowing about. On the real D435 they are
    different sensors: the depth pair is wider (87 deg against 69; on the
    D455 87 against 90, so there depth is slightly narrower) and
    sits a short baseline away from the colour imager, so the two have
    their own intrinsics and their own extrinsics. Here they are
    pixel-identical and share the colour FOV, which means depth is
    NARROWER in simulation than on hardware and perfectly registered to
    colour in a way the real pair never is. Splitting them is two render
    products and twice the per-frame GPU cost; worth doing if anything
    downstream depends on the disparity or on the peripheral depth.
    """
    parent = _find_link(stage, link_name)
    if parent is None:
        print(f'[sensors] no prim for link {link_name}; skipping {child}', flush=True)
        return None
    intrinsics = CAMERA_MODELS[model]
    if horizontal_aperture is None:
        horizontal_aperture = intrinsics['horizontal_aperture']
    if focal_length is None:
        focal_length = intrinsics['focal_length']
    path = f'{parent}/{child}'
    cam = UsdGeom.Camera.Define(stage, path)
    # Gf composes left to right: the X turn is applied first.
    rot = (Gf.Rotation(Gf.Vec3d(1, 0, 0), 90.0)
           * Gf.Rotation(Gf.Vec3d(0, 0, 1), -90.0))
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddOrientOp().Set(Gf.Quatf(rot.GetQuat()))
    cam.CreateFocalLengthAttr(focal_length)
    cam.CreateHorizontalApertureAttr(horizontal_aperture)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    print(f'[sensors] camera ({model}, f={focal_length:g} mm) -> {path}',
          flush=True)
    return path


def build_sensor_graph(stage, ouster, ouster_imu, rplidar, cameras, robot_root):
    """One OmniGraph carrying every sensor publisher.

    Separate from the control ActionGraph on purpose. That one is driven
    by OnImpulseEvent so joint states and commands step with the
    controller; sensors want to run per rendered frame instead, which is
    what OnPlaybackTick gives, and mixing the two would tie the lidar
    rate to the controller's.

    Render products are created inside the graph with
    IsaacCreateRenderProduct rather than through the replicator API,
    which is how NVIDIA's own RTX Lidar graph does it and keeps the
    render product's lifetime tied to the graph rather than to a python
    reference nobody holds.

    `cameras` is a list of (name, prim_path) pairs, e.g.
    ('gantry_d455', '/.../gantry_d455_camera'); entries whose prim is None
    (link not in the model) are skipped. Each camera gets its own render
    product, colour/depth/camera_info publishers, topics
    camera/<name>/{color/image_raw,depth/image_rect_raw,color/camera_info}
    and frame ids <name>_{color,depth}_optical_frame.
    """
    cameras = [(n, p) for n, p in (cameras or []) if p]
    nodes = [
        ('OnTick', 'omni.graph.action.OnPlaybackTick'),
        ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
        ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
    ]
    connect = []
    values = []

    if ouster:
        nodes += [('OusterRP', 'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                  ('OusterPub', 'isaacsim.ros2.bridge.ROS2RtxLidarHelper')]
        connect += [('OnTick.outputs:tick', 'OusterRP.inputs:execIn'),
                    ('OusterRP.outputs:execOut', 'OusterPub.inputs:execIn'),
                    ('OusterRP.outputs:renderProductPath',
                     'OusterPub.inputs:renderProductPath'),
                    ('Context.outputs:context', 'OusterPub.inputs:context')]
        values += [('OusterRP.inputs:cameraPrim', [Sdf.Path(ouster)]),
                   ('OusterPub.inputs:topicName', 'ouster/points'),
                   ('OusterPub.inputs:frameId', 'os_lidar'),
                   ('OusterPub.inputs:type', 'point_cloud'),
                   # A full revolution per message. Without it the helper
                   # emits whatever partial sweep the frame happened to
                   # cover, so message size and coverage vary with render
                   # rate and a consumer sees a rotating wedge.
                   ('OusterPub.inputs:fullScan', True)]

    if rplidar:
        nodes += [('RpRP', 'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                  ('RpPub', 'isaacsim.ros2.bridge.ROS2RtxLidarHelper')]
        connect += [('OnTick.outputs:tick', 'RpRP.inputs:execIn'),
                    ('RpRP.outputs:execOut', 'RpPub.inputs:execIn'),
                    ('RpRP.outputs:renderProductPath',
                     'RpPub.inputs:renderProductPath'),
                    ('Context.outputs:context', 'RpPub.inputs:context')]
        values += [('RpRP.inputs:cameraPrim', [Sdf.Path(rplidar)]),
                   ('RpPub.inputs:topicName', 'scan'),
                   ('RpPub.inputs:frameId', 'laser_frame'),
                   # laser_scan, not point_cloud: the A1 is a single
                   # emitter in a plane, and the real driver publishes
                   # sensor_msgs/LaserScan on /scan.
                   # Beams that hit nothing come back as -1, not inf.
                   # That is below range_min, so consumers that follow
                   # the LaserScan contract discard them correctly, but
                   # anything that assumes the ROS convention of inf for
                   # "no return" and tests with isinf() will treat them
                   # as real measurements one metre behind the sensor.
                   ('RpPub.inputs:type', 'laser_scan'),
                   ('RpPub.inputs:fullScan', True)]

    if ouster_imu:
        nodes += [('ImuRead', 'isaacsim.sensors.physics.IsaacReadIMU'),
                  ('ImuPub', 'isaacsim.ros2.bridge.ROS2PublishImu')]
        connect += [('OnTick.outputs:tick', 'ImuRead.inputs:execIn'),
                    ('ImuRead.outputs:execOut', 'ImuPub.inputs:execIn'),
                    ('ImuRead.outputs:angVel', 'ImuPub.inputs:angularVelocity'),
                    ('ImuRead.outputs:linAcc', 'ImuPub.inputs:linearAcceleration'),
                    ('ImuRead.outputs:orientation', 'ImuPub.inputs:orientation'),
                    ('SimTime.outputs:simulationTime', 'ImuPub.inputs:timeStamp'),
                    ('Context.outputs:context', 'ImuPub.inputs:context')]
        values += [('ImuRead.inputs:imuPrim', [Sdf.Path(ouster_imu)]),
                   # The real Ouster IMU reports gravity, as every
                   # accelerometer at rest does. Without this the
                   # published acceleration is zero when stationary,
                   # which any attitude filter downstream will read as
                   # free fall.
                   ('ImuRead.inputs:readGravity', True),
                   ('ImuPub.inputs:topicName', 'ouster/imu'),
                   ('ImuPub.inputs:frameId', 'os_imu')]

    for name, prim in cameras:
        rp, rgb, dep, info = (f'{name}_{k}' for k in ('rp', 'rgb', 'depth', 'info'))
        nodes += [(rp, 'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                  (rgb, 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                  (dep, 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                  (info, 'isaacsim.ros2.bridge.ROS2CameraInfoHelper')]
        connect += [('OnTick.outputs:tick', f'{rp}.inputs:execIn'),
                    (f'{rp}.outputs:execOut', f'{rgb}.inputs:execIn'),
                    (f'{rp}.outputs:execOut', f'{dep}.inputs:execIn'),
                    (f'{rp}.outputs:execOut', f'{info}.inputs:execIn')]
        for n in (rgb, dep, info):
            connect += [(f'{rp}.outputs:renderProductPath',
                         f'{n}.inputs:renderProductPath'),
                        ('Context.outputs:context', f'{n}.inputs:context')]
        values += [(f'{rp}.inputs:cameraPrim', [Sdf.Path(prim)]),
                   (f'{rp}.inputs:width', CAMERA_WIDTH),
                   (f'{rp}.inputs:height', CAMERA_HEIGHT),
                   (f'{rgb}.inputs:topicName', f'camera/{name}/color/image_raw'),
                   (f'{rgb}.inputs:frameId', f'{name}_color_optical_frame'),
                   (f'{rgb}.inputs:type', 'rgb'),
                   (f'{dep}.inputs:topicName',
                    f'camera/{name}/depth/image_rect_raw'),
                   (f'{dep}.inputs:frameId', f'{name}_depth_optical_frame'),
                   (f'{dep}.inputs:type', 'depth'),
                   (f'{info}.inputs:topicName', f'camera/{name}/color/camera_info'),
                   (f'{info}.inputs:frameId', f'{name}_color_optical_frame')]

    # The sensor-internal frames, published the way a driver would. See
    # the module docstring for why these are not in the URDF. The camera
    # prims are left out on purpose: their optical frames come from the
    # static transforms in control.launch.py, and this tree names frames
    # after prims, so a camera on a moving link would add a second
    # publisher for a frame robot_state_publisher already owns.
    targets = [Sdf.Path(p) for p in (ouster, ouster_imu, rplidar) if p]
    if targets:
        nodes += [('SensorTf', 'isaacsim.ros2.bridge.ROS2PublishTransformTree')]
        connect += [('OnTick.outputs:tick', 'SensorTf.inputs:execIn'),
                    ('SimTime.outputs:simulationTime', 'SensorTf.inputs:timeStamp'),
                    ('Context.outputs:context', 'SensorTf.inputs:context')]
        values += [('SensorTf.inputs:parentPrim', [Sdf.Path(robot_root)]),
                   ('SensorTf.inputs:targetPrims', targets)]

    og.Controller.edit(
        {'graph_path': '/SensorGraph', 'evaluator_name': 'execution'},
        {
            og.Controller.Keys.CREATE_NODES: nodes,
            og.Controller.Keys.CONNECT: connect,
            og.Controller.Keys.SET_VALUES: values,
        },
    )
    print(f'[sensors] sensor graph: {len(nodes)} nodes', flush=True)


def apply_wheel_friction(stage, mu=1.2, root='/WheelPhysics'):
    """Give the tyre colliders the friction the xacro asks for.

    The URDF importer cannot carry it. `ranger_wheels.xacro` declares
    rubber-on-floor grip twice, once as `<ode><mu>1.5</mu></ode>` inside
    the tyre `<collision>` and once as `<mu1>1.2</mu1>` in a
    `<gazebo reference>` block, and both spellings are gz extensions that
    mean nothing to a URDF reader. So the imported spheres arrive with no
    physics material at all and silently fall back to PhysX's default of
    0.5, less than half the intended grip, while the gz side of the same
    xacro gets 1.2.

    Returns the number of colliders bound, so a caller can tell the
    difference between "applied" and "found nothing to apply it to" --
    the prim paths come from the importer and would change without
    warning if its naming ever did.
    """
    from pxr import UsdPhysics, UsdShade

    UsdGeom.Scope.Define(stage, root)
    material = UsdShade.Material.Define(stage, f'{root}/TyreMaterial')
    phys = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    phys.CreateStaticFrictionAttr().Set(mu)
    phys.CreateDynamicFrictionAttr().Set(mu)
    phys.CreateRestitutionAttr().Set(0.0)

    n = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if 'tyre' not in path or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose='physics')
        n += 1
    print(f'[sensors] wheel friction: mu {mu} on {n} tyre colliders', flush=True)
    return n
