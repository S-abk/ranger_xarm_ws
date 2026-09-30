"""Scenes for isaac_bringup.py: NVIDIA's sample environments (or any USD)
and the robot's spawn pose.

The environments are referenced from NVIDIA's asset server, not copied
into this repository: they are not redistributable. Names map to paths
under the Isaac assets root (isaacsim.storage.native), and anything with
a scheme or a slash is taken as a USD path or URL as is.

report() prints what was loaded: units, up axis, extent, and how many of
its geometry prims carry a collider. A prop that renders but has no
collider is seen by the lidars and driven through by the robot, so a low
share is worth knowing about before trusting a scene.
"""
from pxr import Gf, Usd, UsdGeom, UsdPhysics

SCENES = {
    'hospital': '/Isaac/Environments/Hospital/hospital.usd',
    'office': '/Isaac/Environments/Office/office.usd',
    'warehouse': '/Isaac/Environments/Simple_Warehouse/full_warehouse.usd',
    'warehouse_small': '/Isaac/Environments/Simple_Warehouse/warehouse.usd',
    'simple_room': '/Isaac/Environments/Simple_Room/simple_room.usd',
}
ASSETS_FALLBACK = 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1'


def scene_url(name):
    if '://' in name or '/' in name or name.endswith(('.usd', '.usda', '.usdc')):
        return name
    if name not in SCENES:
        raise SystemExit(f'unknown scene {name!r}; one of {sorted(SCENES)} or a USD path/URL')
    root = None
    try:
        from isaacsim.storage.native import get_assets_root_path
        root = get_assets_root_path()
    except Exception as exc:  # the storage extension may not be loaded
        print(f'[isaac_scenes] assets root lookup failed ({exc}); using {ASSETS_FALLBACK}', flush=True)
    return (root or ASSETS_FALLBACK) + SCENES[name]


def add_scene(stage, url, path='/Environment'):
    prim = stage.DefinePrim(path, 'Xform')
    prim.GetReferences().AddReference(url)
    return prim


def place_robot(stage, path, x, y, yaw_deg, z=0.0):
    """Put the robot's root at (x, y, z) facing yaw_deg (before physics starts)."""
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddRotateZOp().Set(float(yaw_deg))


def report(stage, path='/Environment'):
    """Units, extent, floor height and how much of the scene can be collided with."""
    root = stage.GetPrimAtPath(path)
    meshes = collide = 0
    for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if p.IsA(UsdGeom.Gprim):
            meshes += 1
            q, hit = p, False
            while q and q != root.GetParent():
                if q.HasAPI(UsdPhysics.CollisionAPI):
                    hit = True
                    break
                q = q.GetParent()
            collide += hit
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render']).ComputeWorldBound(root)
    r = box.ComputeAlignedRange()
    lo, hi = r.GetMin(), r.GetMax()
    print(f'[isaac_scenes] {path}: metersPerUnit {UsdGeom.GetStageMetersPerUnit(stage)}, '
          f'upAxis {UsdGeom.GetStageUpAxis(stage)}, extent x {lo[0]:.2f}..{hi[0]:.2f} '
          f'y {lo[1]:.2f}..{hi[1]:.2f} z {lo[2]:.2f}..{hi[2]:.2f} m; '
          f'{meshes} geometry prims, {collide} of them collide ({100.0 * collide / max(1, meshes):.0f} %)',
          flush=True)
    return (lo[0], lo[1], lo[2]), (hi[0], hi[1], hi[2])
