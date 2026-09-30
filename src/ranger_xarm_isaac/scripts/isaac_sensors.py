#!/usr/bin/env python3
"""Attach the platform's sensors to the robot in Isaac and bridge them.

The description already carries the sensor links, their meshes and their
CAD-derived mount poses, but it carries no <sensor> tags at all, so
neither simulator produced a single reading. This module fills that in
for Isaac: an Ouster OS0 and its built-in IMU on the gantry, an RPLIDAR
A1M8 on the front deck, and a RealSense D435.

The topics and frame ids deliberately match what the REAL drivers
publish, because ranger_xarm_sensors already defines that contract and
everything downstream is written against it. Same topic, same frame, so
nothing above this layer can tell which one it is talking to.

    Ouster OS0    /ouster/points      os_lidar
                  /ouster/imu         os_imu
    RPLIDAR A1M8  /scan               laser_frame
    RealSense     /camera/d435/color/image_raw        d435_color_optical_frame
                  /camera/d435/depth/image_rect_raw   d435_depth_optical_frame
                  .../camera_info

One consequence is worth stating, because it looks like an omission. The
sensor-internal frames (os_lidar, os_imu, the optical frames) are NOT
added to the URDF. On hardware they belong to the drivers: the Ouster
publishes them from its own metadata, the RealSense from its own
calibration, and ranger_xarm_sensors is built around that split, which is
why d435_mount_link exists in the description and nothing below it does.
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

# D435 depth stream. The colour stream runs at 1280x720 on hardware, but
# both are rendered here and each render product costs GPU every frame,
# so they are matched at the depth resolution instead.
D435_WIDTH, D435_HEIGHT = 848, 480


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


# D435 colour stream: 69 deg horizontal FOV. focal = (aperture/2) /
# tan(FOV/2) for Isaac's default 20.955 mm aperture.
D435_APERTURE = 20.955
D435_FOCAL = 15.245


def add_camera(stage, link_name, child, horizontal_aperture=D435_APERTURE,
               focal_length=D435_FOCAL):
    """A camera on a link, looking along the link's +X.

    USD cameras look down their own -Z with +Y up, so the prim is turned
    +90 deg about X and then -90 about Z: -Z goes to the link's +X (forward,
    where the real D435 points) and +Y to its +Z (up). The rotation used to
    be composed the other way round, which aimed the camera out of the
    robot's right side with its top edge forward.

    The prim is NOT the optical frame. Isaac's transform tree names frames
    after prims, and a USD camera's axes are 180 deg about X from a ROS
    optical frame's (+Z forward, +Y down), so a prim called
    d435_color_optical_frame published a wrong frame under the right name.
    The images carry the optical frame ids, and control.launch.py publishes
    d435_link -> the optical frames statically, as the RealSense driver does.

    One render product feeds both the colour and the depth stream, which
    is a simplification worth knowing about. On the real D435 they are
    different sensors: the depth pair is wider (87 deg against 69) and
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
    print(f'[sensors] camera -> {path}', flush=True)
    return path


def build_sensor_graph(stage, ouster, ouster_imu, rplidar, d435, robot_root):
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
    """
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

    if d435:
        nodes += [('CamRP', 'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                  ('CamRgb', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                  ('CamDepth', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                  ('CamInfo', 'isaacsim.ros2.bridge.ROS2CameraInfoHelper')]
        connect += [('OnTick.outputs:tick', 'CamRP.inputs:execIn'),
                    ('CamRP.outputs:execOut', 'CamRgb.inputs:execIn'),
                    ('CamRP.outputs:execOut', 'CamDepth.inputs:execIn'),
                    ('CamRP.outputs:execOut', 'CamInfo.inputs:execIn')]
        for n in ('CamRgb', 'CamDepth', 'CamInfo'):
            connect += [('CamRP.outputs:renderProductPath',
                         f'{n}.inputs:renderProductPath'),
                        ('Context.outputs:context', f'{n}.inputs:context')]
        values += [('CamRP.inputs:cameraPrim', [Sdf.Path(d435)]),
                   ('CamRP.inputs:width', D435_WIDTH),
                   ('CamRP.inputs:height', D435_HEIGHT),
                   ('CamRgb.inputs:topicName', 'camera/d435/color/image_raw'),
                   ('CamRgb.inputs:frameId', 'd435_color_optical_frame'),
                   ('CamRgb.inputs:type', 'rgb'),
                   ('CamDepth.inputs:topicName',
                    'camera/d435/depth/image_rect_raw'),
                   ('CamDepth.inputs:frameId', 'd435_depth_optical_frame'),
                   ('CamDepth.inputs:type', 'depth'),
                   ('CamInfo.inputs:topicName', 'camera/d435/color/camera_info'),
                   ('CamInfo.inputs:frameId', 'd435_color_optical_frame')]

    # The sensor-internal frames, published the way a driver would. See
    # the module docstring for why these are not in the URDF.
    targets = [Sdf.Path(p) for p in (ouster, ouster_imu, rplidar, d435) if p]
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
