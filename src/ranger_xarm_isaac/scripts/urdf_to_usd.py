#!/usr/bin/env python3
"""Convert the platform's xacro into a USD, lazily.

Run under Isaac's interpreter, not the system one; it needs the Kit
runtime that only that interpreter sets up:

    $ISAACSIM_PYTHON_EXE $(ros2 pkg prefix ranger_xarm_isaac)/lib/\
ranger_xarm_isaac/urdf_to_usd.py

The USD is a BUILD ARTIFACT and is gitignored. It is never the source of
truth: ranger_xarm_description's xacro is, exactly as it is for gz and
for hardware. NVIDIA's own ROS 2 reference architecture argues the same
point, that one URDF should be the common origin for both ROS 2 and
Isaac. A committed USD is the "second description that drifts" this
workspace is built to avoid, and it drifts silently, because nothing
fails when it goes stale.

Conversion is skipped when the USD is newer than every xacro and mesh it
was built from, so re-running this is cheap. Pass --force to rebuild
anyway, which is what you want after changing xacro ARGUMENTS rather
than xacro files, since those do not show up as mtimes.
"""
import argparse
import os
import re
import subprocess
import sys

try:
    from ament_index_python.packages import get_package_share_directory
except ModuleNotFoundError:
    sys.exit(
        "ament_index_python not found.\n"
        "Isaac's python.sh does not source your ROS workspace, so source it\n"
        "first, in the same shell:\n"
        "    source /opt/ros/jazzy/setup.bash && source install/setup.bash\n"
        "then re-run this with the Isaac Sim interpreter ($ISAACSIM_PYTHON_EXE: python.sh in a standalone install)."
    )


def newest_source_mtime(pkg_share):
    """Latest mtime across everything the URDF is assembled from."""
    newest = 0.0
    for root, _, files in os.walk(pkg_share):
        for f in files:
            if f.endswith(('.xacro', '.stl', '.dae', '.obj', '.yaml')):
                newest = max(newest, os.path.getmtime(os.path.join(root, f)))
    return newest


def ros_clean_env():
    """Environment for running a ROS tool from inside Isaac's interpreter.

    Isaac ships its own Python (3.11 in 5.1) and python.sh exports
    PYTHONHOME/PYTHONPATH pointing at it. A subprocess like xacro has a
    system shebang, so it starts CPython 3.12 but then finds Isaac's 3.11
    standard library and dies on an assert deep in the re module:

        AssertionError: SRE module mismatch

    which says nothing about the actual cause. Drop PYTHONHOME entirely
    and strip the isaacsim entries out of PYTHONPATH, keeping the ROS
    ones so xacro can still import itself.
    """
    env = dict(os.environ)
    env.pop('PYTHONHOME', None)
    if 'PYTHONPATH' in env:
        kept = [p for p in env['PYTHONPATH'].split(os.pathsep)
                if p and 'isaacsim' not in p and '/kit/' not in p]
        env['PYTHONPATH'] = os.pathsep.join(kept)
    return env


def resolve_package_uris(urdf_text):
    """Rewrite package:// mesh URIs to absolute paths.

    Isaac's URDF importer is not ROS-aware and cannot resolve
    package://. Worse, it does not complain: it imports the robot with
    the geometry silently missing, so the USD looks fine, loads fine, and
    contains no meshes at all. The giveaway is a converted robot that is
    a few tens of kB when the source meshes are megabytes.

    Only the temporary expanded copy is rewritten. The description keeps
    its package:// URIs, which is what ROS and gz both want.
    """
    def sub(m):
        pkg, rest = m.group(1), m.group(2)
        try:
            share = get_package_share_directory(pkg)
        except Exception:
            return m.group(0)
        return f'{share}/{rest}'

    return re.sub(r'package://([^/]+)/([^"\']+)', sub, urdf_text)


def expand_xacro(xacro_file, xacro_args):
    """Expand to a URDF string, the same way the launch files do."""
    cmd = ['xacro', xacro_file] + list(xacro_args)
    out = subprocess.run(cmd, capture_output=True, text=True, env=ros_clean_env())
    if out.returncode != 0:
        sys.exit(f'xacro failed:\n{out.stderr}')
    return out.stdout


def _add_mimic_drives(out_stage, urdf_text, UsdPhysics):
    """Give the URDF's mimic joints a drive, since the importer does not.

    The importer applies PhysicsDriveAPI to every joint EXCEPT those with
    a <mimic> tag, presumably on the assumption that something else will
    couple them. Nothing does: USD has no mimic concept, so the follower
    joints arrive with no drive at all and a position target sent to them
    is silently ignored. They then hang off the gripper and settle
    wherever contact and gravity leave them, which looks like a gripper
    that half closes and is not left/right symmetric.

    Each follower is given the same drive as the joint it mimics, so it
    responds identically to the commands ranger_xarm_isaac's merger
    derives for it.
    """
    import re as _re

    followers = {}
    for m in _re.finditer(
            r'<joint[^>]*name="([^"]+)"[^>]*>(.*?)</joint>', urdf_text, _re.S):
        name, body = m.group(1), m.group(2)
        mim = _re.search(r'<mimic[^>]*joint="([^"]+)"', body)
        if mim:
            followers[name] = mim.group(1)
    if not followers:
        return

    by_name = {}
    for prim in out_stage.Traverse():
        if 'Joint' in str(prim.GetTypeName()):
            by_name[prim.GetName()] = prim

    added = []
    for follower, source in followers.items():
        fp, sp = by_name.get(follower), by_name.get(source)
        if fp is None or sp is None:
            continue
        src_drive = UsdPhysics.DriveAPI.Get(sp, 'angular')
        if not src_drive:
            continue
        dst = UsdPhysics.DriveAPI.Apply(fp, 'angular')
        for attr in ('Stiffness', 'Damping', 'MaxForce'):
            src_attr = getattr(src_drive, f'Get{attr}Attr')()
            if src_attr and src_attr.Get() is not None:
                getattr(dst, f'Create{attr}Attr')(src_attr.Get())
        dst.CreateTypeAttr('force')
        added.append(follower)

    if added:
        print(f'added drives to {len(added)} mimic joints: {", ".join(added)}')


def _make_velocity_drives(out_stage, match, UsdPhysics):
    """Convert matching joints from a position drive to a velocity drive.

    The importer gives every joint a position drive with stiffness. That
    is right for the arm and the steering, and wrong for the wheels: a
    stiff position drive fights a speed command, holding the wheel at
    whatever angle it was rather than spinning it. A velocity drive is
    stiffness 0 with damping, which is a torque-limited speed source.

    Damping is carried over from the stiffness the importer chose, so the
    wheels stay in proportion with the rest of the model rather than
    picking up an unrelated magic number.
    """
    if not match:
        return
    changed = []
    for prim in out_stage.Traverse():
        if 'Joint' not in str(prim.GetTypeName()):
            continue
        if match not in prim.GetName():
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, 'angular')
        if not drive:
            continue
        stiff_attr = drive.GetStiffnessAttr()
        stiffness = stiff_attr.Get() if stiff_attr else 0.0
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(float(stiffness or 625.0))
        changed.append(prim.GetName())
    if changed:
        print(f'velocity drives on {len(changed)} joints: {", ".join(changed)}')


def _make_suspension_springs(out_stage, urdf_text, UsdPhysics):
    """Spring and damp the suspension joints, from the URDF's own numbers.

    ranger_wheels.xacro gives each corner a prismatic *_suspension_joint
    with <dynamics damping>, and <gazebo reference> tags springStiffness
    and springReference for gz. The importer knows none of that: it gives a
    prismatic joint a stiff position drive towards 0, which is a rigid
    suspension again. So each one becomes a force drive with the spring's
    stiffness, the joint's damping and the spring's reference as its
    target, which is a linear spring-damper. maxForce is left generous: the
    spring, not the drive, is what should bound the force.
    """
    import re as _re
    springs = {}
    for name, body in _re.findall(
            r'<gazebo[^>]*reference="([^"]*_suspension_joint)"[^>]*>(.*?)</gazebo>', urdf_text, _re.S):
        k = _re.search(r'<springStiffness>\s*([-0-9.eE+]+)', body)
        ref = _re.search(r'<springReference>\s*([-0-9.eE+]+)', body)
        if k and ref:
            springs[name] = [float(k.group(1)), float(ref.group(1)), 0.0]
    for name, body in _re.findall(r'<joint[^>]*name="([^"]*_suspension_joint)"[^>]*>(.*?)</joint>',
                                  urdf_text, _re.S):
        d = _re.search(r'<dynamics[^>]*damping="([-0-9.eE+]+)"', body)
        if name in springs and d:
            springs[name][2] = float(d.group(1))
    if not springs:
        return
    done = []
    for prim in out_stage.Traverse():
        if prim.GetName() not in springs:
            continue
        k, ref, c = springs[prim.GetName()]
        drive = UsdPhysics.DriveAPI.Apply(prim, 'linear')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(k)
        drive.CreateDampingAttr(c)
        drive.CreateTargetPositionAttr(ref)
        drive.CreateMaxForceAttr(1.0e5)
        done.append(prim.GetName())
    print(f'suspension springs on {len(done)} joints (k, reference, damping per joint): '
          + ', '.join(f'{n} {springs[n][0]:g} N/m {springs[n][1]:g} m {springs[n][2]:g} N s/m'
                      for n in done[:1]) + (' ...' if len(done) > 1 else ''))


def _set_wheel_friction(out_usd, mu, Usd, UsdPhysics):
    """Bind a physics material to the tyre colliders.

    The importer cannot carry this from the xacro. `ranger_wheels.xacro`
    states rubber-on-floor grip twice -- `<ode><mu>1.5</mu></ode>` inside
    the tyre `<collision>`, and `<mu1>1.2</mu1>` in a `<gazebo reference>`
    block -- and both spellings are gz extensions that a URDF reader is
    entitled to ignore. The spheres therefore arrive with no physics
    material and fall back to PhysX's default 0.5, less than half the
    grip the same xacro gives the gz side. Nothing warns about it; the
    robot simply drives with the wrong tyres.
    """
    if not mu:
        return
    from pxr import UsdShade
    # The colliders are not on the top-level stage. The importer splits
    # its output into <name>.usd plus configuration/<name>_base.usd and
    # configuration/<name>_physics.usd, and traversing the top layer
    # reaches none of the collision prims -- it returns zero, which reads
    # exactly like "no wheels" rather than "wrong stage". Open the physics
    # layer directly.
    stem = os.path.splitext(os.path.basename(out_usd))[0]
    phys_usd = os.path.join(os.path.dirname(out_usd), 'configuration',
                            f'{stem}_physics.usd')
    if not os.path.exists(phys_usd):
        print(f'  WARNING: no physics layer at {phys_usd}; '
              'wheels keep PhysX defaults (0.5)')
        return
    stage = Usd.Stage.Open(phys_usd)
    material = UsdShade.Material.Define(stage, '/TyrePhysics/TyreMaterial')
    phys = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    phys.CreateStaticFrictionAttr().Set(mu)
    phys.CreateDynamicFrictionAttr().Set(mu)
    phys.CreateRestitutionAttr().Set(0.0)

    bound = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if 'tyre' not in path or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose='physics')
        bound.append(prim.GetName())
    # Say the count out loud. The prim paths come from the importer and
    # would change without warning if its naming did, and a silent zero
    # here looks exactly like a working run.
    print(f'wheel friction: mu {mu} bound to {len(bound)} tyre colliders')
    if not bound:
        print('  WARNING: no tyre colliders matched; wheels keep PhysX '
              'defaults (0.5)')
    stage.GetRootLayer().Save()


def _set_wheel_contact_offsets(out_usd, contact, rest, Usd, UsdPhysics):
    """Author PhysX contact/rest offsets on the tyre colliders.

    These have no dartsim equivalent, so they are one place the two
    simulators can disagree over small obstacles. The importer leaves them
    unset, so they fall back to the scene default -- a contact offset of a
    couple of centimetres, which generates contacts well before touch and
    holds them after. dartsim has no such skin.

    Same sub-layer caveat as the friction: the colliders are not on the
    top-level stage.
    """
    if contact is None and rest is None:
        return
    from pxr import PhysxSchema
    stem = os.path.splitext(os.path.basename(out_usd))[0]
    phys_usd = os.path.join(os.path.dirname(out_usd), 'configuration',
                            f'{stem}_physics.usd')
    if not os.path.exists(phys_usd):
        print(f'  WARNING: no physics layer at {phys_usd}; offsets not set')
        return
    stage = Usd.Stage.Open(phys_usd)
    n = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if 'tyre' not in path or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        px = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        if rest is not None:
            px.CreateRestOffsetAttr().Set(rest)
        if contact is not None:
            px.CreateContactOffsetAttr().Set(contact)
        n += 1
    print(f'wheel contact offsets: contact={contact} rest={rest} on {n} colliders')
    if not n:
        print('  WARNING: no tyre colliders matched; offsets left at scene default')
    stage.GetRootLayer().Save()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', default=None,
                    help='destination .usd (default: <pkg_share>/usd/ranger_xarm.usd)')
    ap.add_argument('--force', action='store_true',
                    help='convert even if the USD looks up to date')
    ap.add_argument('--fix-base', action='store_true',
                    help='weld the base to the world, matching the gz sim default')
    ap.add_argument('--wheel-mu', type=float, default=1.2,
                    help='friction for the tyre colliders (default 1.2, the '
                         "value ranger_wheels.xacro asks for). The importer "
                         'cannot read it from the xacro, so it is applied '
                         'here; 0 leaves PhysX defaults in place.')
    ap.add_argument('--wheel-contact-offset', type=float, default=None,
                    help='PhysX contact offset for the tyre colliders, in '
                         'metres. Left unset they take the scene default, '
                         'a couple of centimetres, so contacts are '
                         'generated early and held. Set small '
                         '(e.g. 0.002) to make contact closer to the strict '
                         'rigid contact dartsim gives the gz side.')
    ap.add_argument('--wheel-rest-offset', type=float, default=None,
                    help='PhysX rest offset for the tyre colliders, in '
                         'metres. Pair with --wheel-contact-offset; PhysX '
                         'requires contact offset > rest offset.')
    ap.add_argument('--velocity-drive-joints', default='_wheel_joint',
                    help='substring; joints matching it get a velocity drive '
                         '(stiffness 0) instead of a position drive')
    ap.add_argument('xacro_args', nargs='*',
                    help='passed through to xacro, e.g. add_gripper:=true')
    args = ap.parse_args()

    try:
        desc_share = get_package_share_directory('ranger_xarm_description')
        isaac_share = get_package_share_directory('ranger_xarm_isaac')
    except Exception as exc:
        sys.exit(
            f'cannot locate the workspace packages ({exc}).\n'
            "Isaac's python.sh does not source your ROS workspace. Source it\n"
            'first, in the same shell:\n'
            '    source /opt/ros/jazzy/setup.bash && source install/setup.bash\n'
            'then re-run this with the Isaac Sim interpreter ($ISAACSIM_PYTHON_EXE: python.sh in a standalone install).'
        )

    xacro_file = os.path.join(desc_share, 'urdf', 'ranger_xarm.urdf.xacro')
    out_usd = args.output or os.path.join(isaac_share, 'usd', 'ranger_xarm.usd')
    os.makedirs(os.path.dirname(out_usd), exist_ok=True)

    if not args.force and os.path.exists(out_usd):
        if os.path.getmtime(out_usd) > newest_source_mtime(desc_share):
            print(f'up to date, skipping: {out_usd}')
            print('  (--force to rebuild; needed after changing xacro args)')
            return

    urdf_text = resolve_package_uris(expand_xacro(xacro_file, args.xacro_args))
    # The importer wants a file it can resolve mesh paths relative to, so
    # the expanded URDF is written beside the description share tree
    # rather than into a temp dir; package:// resolution depends on it.
    tmp_urdf = os.path.join(os.path.dirname(out_usd), 'ranger_xarm.expanded.urdf')
    with open(tmp_urdf, 'w') as f:
        f.write(urdf_text)

    # Kit has to be up before any omni import resolves.
    from isaacsim import SimulationApp
    kit = SimulationApp({'headless': True})

    import omni.kit.commands

    status, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
    # merge_fixed_joints stays False on purpose. The platform is mostly
    # fixed joints (sensors, gantry, pedestal) and merging them collapses
    # exactly the frames the rest of the stack refers to by name.
    cfg.merge_fixed_joints = False
    cfg.import_inertia_tensor = True
    cfg.fix_base = args.fix_base
    cfg.distance_scale = 1.0
    cfg.convex_decomp = False

    # Joint drives. Without these the importer leaves joints weakly driven
    # or undriven, and a position target sent by the articulation
    # controller is simply ignored: the joint reports whatever physics
    # left it at. The arm masks this because its links are heavy and its
    # targets barely move; the gripper does not, and its follower joints
    # sit wherever contact puts them.
    # 1 = position drive.
    cfg.set_default_drive_type(1)
    cfg.set_default_drive_strength(1e5)
    cfg.set_default_position_drive_damping(1e3)

    robot = omni.kit.commands.execute('URDFParseFile', urdf_path=tmp_urdf,
                                      import_config=cfg)[1]
    prim_path = omni.kit.commands.execute(
        'URDFImportRobot',
        urdf_path=tmp_urdf,
        urdf_robot=robot,
        import_config=cfg,
        dest_path=out_usd,
        get_articulation_root=True,
    )[1]

    # The importer writes the USD without a defaultPrim. Referencing such a
    # layer resolves to nothing, and Isaac then reports the far less obvious
    # "Prim /ranger_xarm is not an articulation" rather than anything about a
    # missing prim. Set it so the layer can be referenced by path alone.
    from pxr import Usd, UsdPhysics
    out_stage = Usd.Stage.Open(out_usd)
    _add_mimic_drives(out_stage, urdf_text, UsdPhysics)
    _make_velocity_drives(out_stage, args.velocity_drive_joints, UsdPhysics)
    _make_suspension_springs(out_stage, urdf_text, UsdPhysics)
    _set_wheel_friction(out_usd, args.wheel_mu, Usd, UsdPhysics)
    _set_wheel_contact_offsets(out_usd, args.wheel_contact_offset,
                               args.wheel_rest_offset, Usd, UsdPhysics)
    roots = list(out_stage.GetPseudoRoot().GetChildren())
    if roots and not out_stage.HasDefaultPrim():
        out_stage.SetDefaultPrim(roots[0])
        out_stage.GetRootLayer().Save()
        print(f'set defaultPrim: {roots[0].GetPath()}')
    # The drive, friction and spring edits above are opinions in the root
    # layer; save them whether or not the defaultPrim needed setting.
    out_stage.GetRootLayer().Save()

    print(f'wrote {out_usd}')
    print(f'articulation root: {prim_path}')
    kit.close()


if __name__ == '__main__':
    main()
