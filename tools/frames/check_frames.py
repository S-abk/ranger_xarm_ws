#!/usr/bin/env python3
"""Frame check: one parent per frame, one source per transform, in every bring-up.

    python3 tools/frames/check_frames.py

Run from the workspace root with the workspace sourced (xacro expands the installed
description). Starts no ROS node and touches no hardware.

For each bring-up combination below, the check expands the description that combination's
robot_state_publisher loads (one transform per joint), adds the other transform sources
the combination starts (base driver, filter, simulated odometry, sensor drivers, static
publishers), and fails on
  - a child frame with more than one parent,
  - one parent -> child transform from more than one source,
  - a MoveIt planning frame that is not in the robot's transform tree
    (tf2 keeps the moving parent when a fixed and a moving one compete, so a welded
    `world` is cut off as soon as odometry arrives).
It also checks that the driven-base semantic description is the shipped one plus exactly
the planar virtual joint, and that every row's evidence (a pattern in the launch file it
was read from) still matches, so the table cannot drift from the launch files unnoticed.

The table is data read from the launch files, not inferred from them: when a launch file
changes what it starts or passes, its row here must change with it (the evidence check
fails until it does).
"""
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

WS = Path(__file__).resolve().parents[2]
SRC = WS / 'src'

# ---- descriptions, with the xacro arguments each launch file passes ---------------------
REAL = ('use_xarm:=true use_static_geometry:=true use_static_collision:=true '
        'xarm_ros2_control_plugin:=uf_robot_hardware/UFRobotSystemHardware '
        'xarm_robot_ip:=0.0.0.0 xarm_report_type:=normal xarm_velocity_control:=false '
        'laptop_open_angle_deg:=90.0')
ISAAC_MG = ('xarm_ros2_control_plugin:=joint_state_topic_hardware_interface/JointStateTopicSystem '
            'add_gripper:=true xarm_load_gazebo_plugin:=false planning_margin:=0.005')
ISAAC_CTRL = ('xarm_ros2_control_plugin:=joint_state_topic_hardware_interface/JointStateTopicSystem '
              'add_gripper:=true xarm_load_gazebo_plugin:=false use_suspension:=true '
              'wheels_ros2_control_plugin:=joint_state_topic_hardware_interface/JointStateTopicSystem '
              'wheels_command_interface:=velocity')
SIM = ('xarm_ros2_control_plugin:=gz_ros2_control/GazeboSimSystem xarm_load_gazebo_plugin:=true '
       'xarm_ros2_control_params:=/CONTROLLERS.yaml add_gripper:=true wheels_command_interface:=velocity')
MOCK = ('use_xarm:=true use_static_geometry:=true use_static_collision:=true add_gripper:=true '
        'xarm_mount_yaw:=0.0 use_laptop_collision:=true laptop_open_angle_deg:=90.0')
DESC = {
    'real_execution': REAL + ' add_gripper:=true fix_base_to_world:=true',
    'real_execution_base': REAL + ' add_gripper:=true fix_base_to_world:=false',
    'real_hardware_check': REAL + ' add_gripper:=false fix_base_to_world:=true',
    'base_bringup': ('use_xarm:=false use_static_geometry:=true use_static_collision:=true '
                     'use_laptop_collision:=true laptop_open_angle_deg:=90.0 fix_base_to_world:=false'),
    'sim_arm': SIM + ' use_wheels:=false fix_base_to_world:=true gz_sensors:=false',
    'sim_drive': SIM + ' use_wheels:=true fix_base_to_world:=false gz_sensors:=true',
    'isaac_mg': ISAAC_MG,
    'isaac_mg_drive': ISAAC_MG + ' use_wheels:=true fix_base_to_world:=false',
    'isaac_ctrl_arm': ISAAC_CTRL + ' use_wheels:=false fix_base_to_world:=true',
    'isaac_ctrl_drive': ISAAC_CTRL + ' use_wheels:=true fix_base_to_world:=false',
    'fake_execution': MOCK + ' xarm_ros2_control_plugin:=mock_components/GenericSystem',
    'planning_demo': MOCK,
    'display': ('use_xarm:=true use_static_geometry:=true use_static_collision:=true '
                'xarm_mount_yaw:=0.0 use_laptop_collision:=true'),
}

# ---- other transform sources: (parent, child, moving) ------------------------------------
DRIVER = [('odom', 'base_footprint', True)]       # ranger_base, base_frame base_footprint
FILTER = [('odom', 'base_footprint', True)]       # robot_localization, ekf_odom_imu.yaml
SIM_ODOM = [('odom', 'base_footprint', True)]     # ranger_xarm_gazebo wheel_odometry.py
OUSTER = [('os_sensor', 'os_lidar', False), ('os_sensor', 'os_imu', False)]
REALSENSE = [(f'{c}_link', f'{c}_{k}', False) for c in ('gantry_d455', 'wrist_d435')
             for k in ('color_frame', 'depth_frame', 'color_optical_frame', 'depth_optical_frame')]
SIM_STATIC = OUSTER + [(f'{c}_link', f'{c}_{k}_optical_frame', False)
                       for c in ('gantry_d455', 'wrist_d435') for k in ('color', 'depth')]
ISAAC_STATIC = OUSTER[:1] + SIM_STATIC[2:]

L = 'src/'
# Evidence: (file, pattern that must match) or (file, pattern, False) for one that must NOT.
ROBOT = L + 'ranger_xarm_sensors/launch/robot.launch.py'
REALX = L + 'ranger_xarm_moveit_config/launch/real_execution.launch.py'
HWC = L + 'ranger_xarm_moveit_config/launch/real_hardware_check.launch.py'
BASEB = L + 'ranger_xarm_bringup/launch/base_bringup.launch.py'
MGL = L + 'ranger_xarm_isaac/launch/moveit.launch.py'
E_ROBOT = [(ROBOT, r"'fix_base_to_world': IfElseSubstitution\(lc\('enable_base'\), 'false', 'true'\)"),
           (ROBOT, r"'base_frame': 'base_footprint'")]
E_REAL = [(REALX, r"'fix_base_to_world', default_value='true'"),
          (REALX, r"' fix_base_to_world:=', fix_base_to_world,"),
          (REALX, r"shipped_srdf = str\(pkg / 'config' / 'ranger_xarm\.srdf'\)"),
          (REALX, r"IfElseSubstitution\(fix_base_to_world, shipped_srdf, IfElseSubstitution\(\s*"
                  r"EqualsSubstitution\(planning_frame, 'odom'\),\s*"
                  r"str\(pkg / 'config' / 'ranger_xarm_mobile_base\.srdf'\), shipped_srdf\)\)"),
          (REALX, r"'planning_frame', default_value='odom'")]
E_ROBOT_PF = [(ROBOT, r"'planning_frame': lc\('planning_frame'\)"),
              (ROBOT, r"'planning_frame', default_value='odom'")]
E_BASE = [(BASEB, r"' fix_base_to_world:=false'"), (BASEB, r"'base_frame': 'base_footprint'")]
E_BASE_TF = [(BASEB, r"'publish_odom_tf': LaunchConfiguration\('publish_odom_tf'\)")]
E_DRIVER = [(L + 'ranger_xarm_bringup/launch/ranger_driver.launch.py',
             r"'base_frame', default_value='base_footprint'")]
E_HWC = [(HWC, r"'fix_base_to_world', default_value='true'"),
         (HWC, r"' fix_base_to_world:=', LaunchConfiguration\('fix_base_to_world'\),")]
E_SIM = [(L + 'ranger_xarm_gazebo/launch/sim.launch.py',
          r"' use_wheels:=', 'true' if drive_base else 'false',"),
         (L + 'ranger_xarm_gazebo/launch/sim.launch.py',
          r"' fix_base_to_world:=', 'false' if drive_base else 'true',")]
E_MG = [(MGL, r"'ranger_xarm_mobile_base_wheels\.srdf' if drive_base and planning_frame == 'odom'\s*"
               r"else 'ranger_xarm\.srdf'"),
        (MGL, r"'planning_frame', default_value='odom'"),
        (MGL, r"\[' use_wheels:=true fix_base_to_world:=false'\] if drive_base else \[\]")]
E_CTRL = [(L + 'ranger_xarm_isaac/launch/control.launch.py',
           r"' fix_base_to_world:=', 'false' if drive_base else 'true',")]
# The mock and display launches rely on the description's default (welded).
E_MOCK = [(L + 'ranger_xarm_moveit_config/launch/fake_execution.launch.py', r'fix_base_to_world', False),
          (L + 'ranger_xarm_moveit_config/launch/planning_demo.launch.py', r'fix_base_to_world', False),
          (L + 'ranger_xarm_description/launch/display.launch.py', r'fix_base_to_world', False)]

# name: (descriptions run by robot_state_publishers, [(source label, edges)],
#        move_group description or None, move_group uses the driven-base SRDF,
#        status, evidence)
#   status: 'supported'   -- must be clean
#           'unsupported' -- documented as unsupported; reported, never fails the check
#           'pending'     -- clean once another repository changes (named in the row); reported
ROWS = {
    'R1 real_execution (arm only)':
        (['real_execution'], [], 'real_execution', False, 'supported', E_REAL),
    'R2 real_hardware_check (arm only)':
        (['real_hardware_check'], [], None, False, 'supported', E_HWC),
    'R3 base_bringup (base only, driver owns the pose)':
        (['base_bringup'], [('driver', DRIVER)], None, False, 'supported', E_BASE),
    'R4 base_bringup publish_odom_tf:=false + ekf_odom_imu (filter owns the pose)':
        (['base_bringup'], [('filter', FILTER)], None, False, 'supported', E_BASE + E_BASE_TF),
    'R5 robot.launch.py defaults (arm + base + sensors; driver owns the pose)':
        (['real_execution_base'], [('driver', DRIVER), ('ouster driver', OUSTER),
                                   ('realsense driver', REALSENSE)],
         'real_execution_base', True, 'supported', E_ROBOT + E_REAL),
    'R6 robot.launch.py publish_odom_tf:=false + ekf_odom_imu (filter owns the pose)':
        (['real_execution_base'], [('filter', FILTER), ('ouster driver', OUSTER),
                                   ('realsense driver', REALSENSE)],
         'real_execution_base', True, 'supported', E_ROBOT + E_REAL),
    'R6b robot.launch.py planning_frame:=base_footprint (MoveIt plans on the base)':
        (['real_execution_base'], [('driver', DRIVER), ('ouster driver', OUSTER),
                                   ('realsense driver', REALSENSE)],
         'real_execution_base', False, 'supported', E_ROBOT + E_ROBOT_PF + E_REAL),
    'R7 robot.launch.py enable_base:=false (arm + sensors; teleop hardware path)':
        (['real_execution'], [('ouster driver', OUSTER), ('realsense driver', REALSENSE)],
         'real_execution', False, 'supported', E_ROBOT + E_REAL),
    'R8 real_execution fix_base_to_world:=false + ranger_driver (building blocks)':
        (['real_execution_base'], [('driver', DRIVER)], 'real_execution_base', True, 'supported',
         E_REAL + E_DRIVER),
    'R9 real_execution + base_bringup, two launches (README: unsupported, use robot.launch.py)':
        (['real_execution', 'base_bringup'], [('driver', DRIVER)], 'real_execution', False,
         'unsupported', E_BASE),
    'R10 robot.launch.py publish_odom_tf:=false without the filter (README: start the filter)':
        (['real_execution_base'], [('ouster driver', OUSTER), ('realsense driver', REALSENSE)],
         'real_execution_base', True, 'unsupported', E_ROBOT),
    'G1 sim.launch.py + moveit.launch.py (arm; base welded)':
        (['sim_arm'], [], 'isaac_mg', False, 'supported', E_SIM + E_MG),
    'G2 sim.launch.py drive_base:=true sensors:=true + moveit.launch.py drive_base:=true':
        (['sim_drive'], [('wheel_odometry', SIM_ODOM), ('sim static', SIM_STATIC)],
         'isaac_mg_drive', True, 'supported', E_SIM + E_MG),
    'G2b sim.launch.py drive_base:=true + moveit.launch.py drive_base:=true planning_frame:=base_footprint':
        (['sim_drive'], [('wheel_odometry', SIM_ODOM), ('sim static', SIM_STATIC)],
         'isaac_mg_drive', False, 'supported', E_SIM + E_MG),
    'G3 sim.launch.py drive_base:=true odom_tf:=false (no MoveIt)':
        (['sim_drive'], [('sim static', SIM_STATIC)], None, False, 'supported', E_SIM),
    'I1 Isaac control.launch.py + moveit.launch.py (arm)':
        (['isaac_ctrl_arm'], [('isaac static', ISAAC_STATIC)], 'isaac_mg', False, 'supported',
         E_CTRL + E_MG),
    'I2 Isaac control.launch.py drive_base:=true + moveit.launch.py drive_base:=true':
        (['isaac_ctrl_drive'], [('wheel_odometry', SIM_ODOM), ('isaac static', ISAAC_STATIC)],
         'isaac_mg_drive', True, 'supported', E_CTRL + E_MG),
    'M1 fake_execution': (['fake_execution'], [], 'fake_execution', False, 'supported', E_MOCK[:1]),
    'M2 planning_demo': (['planning_demo'], [], 'planning_demo', False, 'supported', E_MOCK[1:2]),
    'M3 display': (['display'], [], None, False, 'supported', E_MOCK[2:]),
    'T1 teleop simulation base:=true as of its main (MoveIt included without drive_base:=true)':
        (['sim_drive'], [('wheel_odometry', SIM_ODOM), ('sim static', SIM_STATIC)],
         'isaac_mg', False, 'pending', E_MG),
}
# Not covered: Isaac's own sensor transform tree (isaac_sensors.py) beside control.launch.py's
# os_sensor -> os_lidar static transform -- the prim names decide it; to confirm in Isaac.

# Every launch file: no static publisher of base_footprint -> base_link (the description owns it).
NO_STATIC_BF_BL = [L + 'ranger_xarm_sensors/launch/robot.launch.py',
                   L + 'ranger_xarm_bringup/launch/base_bringup.launch.py']


def expand(name, cache={}):
    if name not in cache:
        prefix = subprocess.run(['ros2', 'pkg', 'prefix', 'ranger_xarm_description'],
                                capture_output=True, text=True, check=True).stdout.strip()
        xacro_file = Path(prefix) / 'share/ranger_xarm_description/urdf/ranger_xarm.urdf.xacro'
        out = subprocess.run(['xacro', str(xacro_file)] + DESC[name].split(),
                             capture_output=True, text=True)
        if out.returncode != 0:
            raise SystemExit(f'xacro failed for {name}: {out.stderr.strip()[-400:]}')
        root = ET.fromstring(out.stdout)
        cache[name] = [(j.find('parent').get('link'), j.find('child').get('link'),
                        j.get('type') != 'fixed') for j in root.findall('joint')]
    return cache[name]


def analyse(descs, extras, mg, mobile):
    edges = defaultdict(list)
    for d in descs:
        for p, c, moving in expand(d):
            edges[(p, c)].append((f'robot_state_publisher[{d}]', moving))
    for label, es in extras:
        for p, c, moving in es:
            edges[(p, c)].append((label, moving))
    parents = defaultdict(set)
    for p, c in edges:
        parents[c].add(p)
    two_parents = {c: sorted(ps) for c, ps in parents.items() if len(ps) > 1}
    two_sources = {f'{p}->{c}': [s for s, _ in v] for (p, c), v in edges.items() if len(v) > 1}
    kept = {}
    for (p, c), v in edges.items():
        moving = any(m for _, m in v)
        if c not in kept or (moving and not kept[c][1]):
            kept[c] = (p, moving)

    def root(f):
        seen = set()
        while f in kept and f not in seen:
            seen.add(f)
            f = kept[f][0]
        return f
    planning = None
    if mg:
        if mobile:   # the planar virtual joint's parent frame, read from the generated SRDF
            vj = ET.parse(installed_config() / 'ranger_xarm_mobile_base.srdf').getroot().find('virtual_joint')
            planning = vj.get('parent_frame')
        else:
            mg_edges = expand(mg)
            planning = sorted({p for p, _, _ in mg_edges} - {c for _, c, _ in mg_edges})[0]
    frames = set(kept) | {p for p, _ in kept.values()}
    in_tf = None if planning is None else (planning in frames and root(planning) == root('base_link'))
    roots = sorted({p for p, _ in kept.values()} - set(kept))
    return two_parents, two_sources, roots, planning, in_tf


def installed_config():
    share = subprocess.run(['ros2', 'pkg', 'prefix', 'ranger_xarm_moveit_config'],
                           capture_output=True, text=True, check=True).stdout.strip()
    return Path(share) / 'share/ranger_xarm_moveit_config/config'


def srdf_check():
    cfg = installed_config()
    script = SRC / 'ranger_xarm_moveit_config/scripts/make_mobile_base_srdf.py'
    shipped_src = (SRC / 'ranger_xarm_moveit_config/config/ranger_xarm.srdf').read_bytes()
    problems = []
    if (cfg / 'ranger_xarm.srdf').read_bytes() != shipped_src:
        problems.append('installed ranger_xarm.srdf differs from the source (rebuild)')
    for mode, parent, child in (('--check', 'ranger_xarm.srdf', 'ranger_xarm_mobile_base.srdf'),
                                ('--check-wheels', 'ranger_xarm_mobile_base.srdf',
                                 'ranger_xarm_mobile_base_wheels.srdf')):
        r = subprocess.run([sys.executable, str(script), mode, str(cfg / parent), str(cfg / child)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            problems.append(r.stdout.strip() or r.stderr.strip())
    return problems


def main():
    failures = []
    for ev in {e for row in ROWS.values() for e in row[5]}:
        f, pattern, must = ev if len(ev) == 3 else (*ev, True)
        if bool(re.search(pattern, (WS / f).read_text())) != must:
            failures.append(f'evidence no longer holds: {f}: /{pattern}/ must '
                            f'{"" if must else "not "}match (update the table)')
    for f in NO_STATIC_BF_BL:
        text = (WS / f).read_text()
        if re.search(r"static_transform_publisher", text) and re.search(r"'base_link'", text):
            failures.append(f'{f}: starts a static publisher next to base_link; '
                            'base_footprint -> base_link belongs to the description')
    failures += [f'semantic description: {p}' for p in srdf_check()]

    print('| combination | status | two parents | two sources | TF root | planning frame | in TF |')
    print('|---|---|---|---|---|---|---|')
    for name, (descs, extras, mg, mobile, status, _) in ROWS.items():
        tp, ts, roots, planning, in_tf = analyse(descs, extras, mg, mobile)
        bad = bool(tp or ts) or in_tf is False
        fmt = lambda d: '; '.join(f'{k}: {", ".join(v)}' for k, v in d.items()) if d else '-'
        ts_text = f'{len(ts)} edges' if len(ts) > 4 else fmt(ts)
        verdict = ('FAULT' if bad else 'ok') + ('' if status == 'supported' else f' ({status})')
        print(f'| {name} | {verdict} | {fmt(tp)} | {ts_text} | {", ".join(roots)} | '
              f'{planning or "-"} | {"-" if in_tf is None else ("yes" if in_tf else "NO")} |')
        if bad and status == 'supported':
            failures.append(f'{name}: two parents {tp or "-"}, two sources {list(ts) or "-"}, '
                            f'planning frame {planning} in TF: {in_tf}')
        if not bad and status != 'supported':
            failures.append(f'{name}: marked {status} but now clean: update its status')
    print()
    for f in failures:
        print(f'FAIL {f}')
    print('frame check PASSED' if not failures else 'frame check FAILED')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
