#!/usr/bin/env python3
"""MoveIt test in simulation: state validity, regular and near-gantry goals, and a
dense check of every planned path against the true collision model.

    moveit_test.py [--out DIR] [--only validity,goals,gantry] [--tries N]

Needs, on the current ROS domain: a simulation publishing /clock, its controllers,
move_group (ranger_xarm_isaac moveit.launch.py, which plans with the planning
margin) and the reference checker (ref_move_group.launch.py: the same model at
margin 0, in /ref). run.sh starts all of them; see README.md.

Every planned path is interpolated so no joint moves more than --step rad
between samples, and each sample is checked on /ref. A sample invalid there
means the path passes through the real structure between the states MoveIt
checked: that fails the goal. A sample invalid only on move_group's model
is inside the margin but clear of the real structure; it is reported, not failed.

Exit status: 0 every check passed, 1 a check failed, 2 the setup was missing
or wrong. Results go to DIR/results.json.
"""
import argparse
import json
import math
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint, RobotState)
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f'xarm_joint{i}' for i in range(1, 7)]
CODES = {v: k for k, v in MoveItErrorCodes.__dict__.items() if k.isupper() and isinstance(v, int)}
ARM_CONTROLLER = '/xarm_xarm6_traj_controller/follow_joint_trajectory'


def P(*q):
    return dict(zip(ARM, q))


HOME = P(0, 0, 0, 0, 0, 0)
# Joint limits from the URDF (xarm6), for the out-of-bounds note when a reset is needed.
LIMITS = dict(zip(ARM, [(-2 * math.pi, 2 * math.pi), (-2.059, 2.0944), (-3.927, 0.19198),
                        (-2 * math.pi, 2 * math.pi), (-1.69297, math.pi), (-2 * math.pi, 2 * math.pi)]))
# Wrist folded back onto the arm: the wrist camera hits xarm_link1/the base. Must be invalid.
FOLDED = P(0, 0, 0, 0, 1.2, -2.5)

# Regular goals, run as a sequence (each starts where the previous one ended).
# kind 'joint': joint goal; 'pose': TCP pose goal at the FK of these joints; 'gripper'.
GOALS = [
    ('named xarm6/hold_up', 'joint', P(0, 0, 0, 0, -1.5708, 0)),
    ('named xarm6/home', 'joint', HOME),
    ('named xarm_gripper/closed', 'gripper', {'xarm_drive_joint': 0.849}),
    ('named xarm_gripper/open', 'gripper', {'xarm_drive_joint': 0.0}),
    ('joint A', 'joint', P(0.5, -0.3, -0.8, 0, 0.8, 0)),
    ('joint B', 'joint', P(-1.0, 0.2, -0.6, 0.5, 0.6, 1.0)),
    ('joint C', 'joint', P(0.8, 0.4, -1.2, -0.6, 1.2, -0.8)),
    ('pose forward', 'pose', P(0, 0.6, -0.9, 0, 0.3, 0)),
    ('pose down toward deck', 'pose', P(0, 1.1, -1.0, 0, -0.1, 0)),
    ('pose side (+Y)', 'pose', P(1.5708, 0.4, -0.8, 0, 0.4, 0)),
    ('j6 sweep: start', 'joint', P(0, 0.3, -0.6, 0, 0.3, 0)),
    ('j6 sweep: to -6.2', 'joint', P(0, 0.3, -0.6, 0, 0.3, -6.2)),
    ('j6 sweep: to +6.2', 'joint', P(0, 0.3, -0.6, 0, 0.3, 6.2)),
    ('j6 sweep: back to 0', 'joint', P(0, 0.3, -0.6, 0, 0.3, 0)),
    ('return home', 'joint', HOME),
]

# Wrist camera close to the gantry. "near" goals: valid with a 5 mm planning margin AND on the
# true model, so every arm link is at least 5 mm from the fixed structure; at least 0.05 rad
# inside every joint limit. Distance = wrist camera or its plate to the nearest gantry or
# gantry-D455 collision box, true model. Each is tried as a joint goal and as a pose goal,
# from home, and the arm returns home after each.
NEAR = [
    ('near: 1.4 cm to D455', P(0.0, -0.046, -3.851, -0.325, -1.53, -0.024)),
    ('near: 1.8 cm to D455', P(0.219, -1.083, -1.593, -1.294, -1.485, -0.033)),
    ('near: 2.7 cm to D455', P(-0.35, -1.177, -1.319, 0.049, -1.583, 0.927)),
    ('near: 2.8 cm to support', P(-0.336, 0.158, -3.849, -0.439, -0.944, 1.299)),
]
# "into" goals: the camera inside the gantry support or D455 box. Must be invalid and must
# not plan.
INTO = [
    ('into: camera in support and D455', P(0.145, -0.022, -3.88, -0.296, -1.527, -2.738)),
    ('into: camera in support', P(0.101, -0.48, -3.907, 0.99, 2.562, 1.25)),
]

ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
ap.add_argument('--out', help='results directory (default: a new temporary directory)')
ap.add_argument('--only', default='validity,goals,gantry', help='comma-separated sections')
ap.add_argument('--ref', default='/ref', help='namespace of the reference (true-model) move_group')
ap.add_argument('--vel', type=float, default=0.3, help='velocity and acceleration scaling')
ap.add_argument('--step', type=float, default=0.01, help='dense check: largest joint step, rad')
ap.add_argument('--tries', type=int, default=2, help='planning requests per goal before it fails')
ap.add_argument('--plan-time', type=float, default=10.0, help='allowed planning time per request, s')
ap.add_argument('--no-reset', action='store_true',
                help='never drive the arm controller home directly when MoveIt cannot')
args = ap.parse_args()
OUT = Path(args.out or tempfile.mkdtemp(prefix='moveit_test.'))
OUT.mkdir(parents=True, exist_ok=True)
SECTIONS = set(args.only.split(','))

rclpy.init()
node = Node('moveit_test')
joints = {}
clock = {'t': None}


def _on_js(m):
    for n, p in zip(m.name, m.position):
        joints[n] = p


def _on_clock(m):
    clock['t'] = m.clock.sec + m.clock.nanosec * 1e-9


node.create_subscription(JointState, '/joint_states', _on_js, 10)
# best effort: matches /clock publishers of either reliability (the gz bridge's is best effort)
node.create_subscription(Clock, '/clock', _on_clock, qos_profile_sensor_data)
validity = node.create_client(GetStateValidity, '/check_state_validity')
ref_validity = node.create_client(GetStateValidity, args.ref + '/check_state_validity')
fk_cli = node.create_client(GetPositionFK, '/compute_fk')
params = node.create_client(GetParameters, '/move_group/get_parameters')
ref_params = node.create_client(GetParameters, args.ref + '/move_group/get_parameters')
move = ActionClient(node, MoveGroup, '/move_action')
execute = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
arm_ctl = ActionClient(node, FollowJointTrajectory, ARM_CONTROLLER)


def setup_fail(msg):
    print('SETUP', msg)
    sys.exit(2)


for c in (validity, ref_validity, fk_cli, params, ref_params):
    if not c.wait_for_service(timeout_sec=60):
        setup_fail(f'service missing: {c.srv_name}')
for a, name in ((move, '/move_action'), (execute, '/execute_trajectory')):
    if not a.wait_for_server(timeout_sec=60):
        setup_fail(f'action missing: {name}')


def spin_wall(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def spin_sim(sec, cap=60.0):
    """Wait `sec` of simulation time (at most `cap` wall seconds)."""
    spin_wall(0.05)
    t0, w0 = clock['t'], time.time()
    while time.time() - w0 < cap:
        rclpy.spin_once(node, timeout_sec=0.02)
        if clock['t'] - t0 >= sec:
            return


# This tool commands motion, so it runs only against a simulation: /clock must be advancing.
spin_wall(3.0)
if clock['t'] is None:
    setup_fail('no /clock: this test runs only against a simulation')
t0 = clock['t']
spin_wall(2.0)
if clock['t'] <= t0:
    setup_fail('/clock is not advancing')
while not all(j in joints for j in ARM):
    spin_wall(0.5)


def wait(fut, timeout):
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


def get_param(cli, name):
    req = GetParameters.Request()
    req.names = [name]
    v = wait(cli.call_async(req), 30).values[0]
    return {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value}.get(v.type)


# ------------------------------------------------------------------- the two models
def boxes(urdf):
    out = {}
    for link in ET.fromstring(urdf).iter('link'):
        for i, c in enumerate(link.findall('collision')):
            b = c.find('geometry/box')
            if b is not None:
                out[f"{link.get('name')}/{c.get('name') or i}"] = [float(v) for v in b.get('size').split()]
    return out


def without_box_sizes(urdf):
    root = ET.fromstring(urdf)
    for b in root.iter('box'):
        b.set('size', '')
    for e in root.iter():
        e.tail = None
        e.text = (e.text or '').strip() or None
    return ET.tostring(root)


plan_urdf = get_param(params, 'robot_description')
true_urdf = get_param(ref_params, 'robot_description')
if not plan_urdf or not true_urdf:
    setup_fail('could not read robot_description from move_group and the reference')
if without_box_sizes(plan_urdf) != without_box_sizes(true_urdf):
    setup_fail('the reference model differs from move_group\'s in more than collision box sizes')
bp, bt = boxes(plan_urdf), boxes(true_urdf)
grown = {k: {round((a - b) / 2, 6) for a, b in zip(bp[k], bt[k])} for k in bp if bp[k] != bt[k]}
per_side = sorted(set().union(*grown.values())) if grown else [0.0]
if any(x < 0 for x in per_side):
    setup_fail(f'the reference has larger boxes than move_group: {per_side}')
results = {
    'started': time.strftime('%Y-%m-%dT%H:%M:%S'),
    'margin': {'per_side_m': per_side, 'boxes': sorted(grown)},
    'config': {k: get_param(params, k) for k in (
        'ompl.xarm6.longest_valid_segment_fraction',
        'robot_description_planning.default_robot_padding')},
    'checks': [],
}
print(f'move_group planning margin {per_side} m per side on {len(grown)} boxes; '
      f'config {results["config"]}', flush=True)
(OUT / 'move_group.urdf').write_text(plan_urdf)
(OUT / 'true.urdf').write_text(true_urdf)


def report(name, ok, detail, **rec):
    results['checks'].append({'check': name, 'passed': bool(ok), 'detail': detail, **rec})
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)


# ------------------------------------------------------------------- helpers
def state_msg(q):
    rs = RobotState()
    rs.is_diff = True
    rs.joint_state.name = list(q)
    rs.joint_state.position = [float(v) for v in q.values()]
    return rs


def check(q, cli=None, group='xarm6'):
    """(valid, contacts) for joint values q."""
    req = GetStateValidity.Request()
    req.robot_state = state_msg(q)
    req.group_name = group
    r = wait((cli or validity).call_async(req), 30)
    pairs = sorted({'<->'.join(sorted((c.contact_body_1, c.contact_body_2))) for c in r.contacts})
    return r.valid, pairs


def fk(q, link='xarm_link_tcp'):
    req = GetPositionFK.Request()
    req.header.frame_id = 'base_link'
    req.fk_link_names = [link]
    req.robot_state = state_msg(q)
    return wait(fk_cli.call_async(req), 30).pose_stamped[0].pose


def joint_goal(q, tol=0.001):
    c = Constraints()
    for n, v in q.items():
        jc = JointConstraint()
        jc.joint_name, jc.position = n, float(v)
        jc.tolerance_above = jc.tolerance_below = tol
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    return c


def pose_goal(pose, link='xarm_link_tcp', pos_tol=0.005, rot_tol=0.02):
    c = Constraints()
    pc = PositionConstraint()
    pc.header.frame_id, pc.link_name, pc.weight = 'base_link', link, 1.0
    s = SolidPrimitive()
    s.type, s.dimensions = SolidPrimitive.SPHERE, [pos_tol]
    p = Pose()
    p.position, p.orientation.w = pose.position, 1.0
    pc.constraint_region.primitives.append(s)
    pc.constraint_region.primitive_poses.append(p)
    oc = OrientationConstraint()
    oc.header.frame_id, oc.link_name, oc.orientation, oc.weight = 'base_link', link, pose.orientation, 1.0
    oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = rot_tol
    c.position_constraints.append(pc)
    c.orientation_constraints.append(oc)
    return c


def dense_check(jt):
    """Interpolate the path (no joint moves more than --step between samples) and check every
    sample on both models. Returns (samples, invalid on move_group, invalid on the true model)."""
    pts = [list(p.positions) for p in jt.points]
    qs = []
    for a, b in zip(pts, pts[1:]):
        k = max(1, math.ceil(max(abs(y - x) for x, y in zip(a, b)) / args.step))
        qs += [[x + (y - x) * i / k for x, y in zip(a, b)] for i in range(k)]
    qs += pts[-1:]
    margin_only, real = 0, []
    for q in qs:
        jq = dict(zip(jt.joint_names, q))
        ok, _ = check(jq)
        rok, rcs = check(jq, ref_validity)
        if not rok:
            real.append({'q': [round(x, 4) for x in q], 'contacts': rcs})
        elif not ok:
            margin_only += 1
    return len(qs), margin_only, real


def plan(constraints, group):
    """Up to --tries planning requests from the current state. Returns (result, record)."""
    rec = {'requests': []}
    for _ in range(args.tries):
        g = MoveGroup.Goal()
        r = MotionPlanRequest()
        r.group_name, r.num_planning_attempts, r.allowed_planning_time = group, 5, args.plan_time
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = args.vel
        r.start_state.is_diff = True
        r.goal_constraints = [constraints]
        g.request = r
        g.planning_options.plan_only = True   # so planning_time is reported
        h = wait(move.send_goal_async(g), 30)
        if h is None or not h.accepted:
            rec['requests'].append({'code': 'goal rejected'})
            continue
        res = wait(h.get_result_async(), args.plan_time * 3 + 60).result
        ok = res.error_code.val == MoveItErrorCodes.SUCCESS and len(res.planned_trajectory.joint_trajectory.points) > 0
        rec['requests'].append({'code': CODES.get(res.error_code.val, res.error_code.val),
                                'planning_time_s': round(res.planning_time, 3)})
        if ok:
            return res, rec
    return None, rec


def settle(cap=20.0, tol=5e-4, window=0.5):
    """Wait until no arm joint moves more than tol over `window` sim seconds (at most cap)."""
    t0 = clock['t']
    prev = [joints[j] for j in ARM]
    while clock['t'] - t0 < cap:
        spin_sim(window, cap=10.0)
        cur = [joints[j] for j in ARM]
        if max(abs(a - b) for a, b in zip(cur, prev)) < tol:
            return
        prev = cur


def run(constraints, group='xarm6'):
    """Plan, dense-check, execute. Returns a record."""
    res, rec = plan(constraints, group)
    rec['planned'] = res is not None
    rec['executed'] = False
    if res is None:
        return rec
    jt = res.planned_trajectory.joint_trajectory
    rec['points'] = len(jt.points)
    d = jt.points[-1].time_from_start
    rec['duration_s'] = round(d.sec + d.nanosec * 1e-9, 3)
    rec['dense_samples'], rec['inside_margin'], real = dense_check(jt)
    rec['through_structure'] = len(real)
    rec['through_structure_samples'] = real[:5]
    eg = ExecuteTrajectory.Goal()
    eg.trajectory = res.planned_trajectory
    h = wait(execute.send_goal_async(eg), 30)
    if h is None or not h.accepted:
        rec['exec_code'] = 'goal rejected'
    else:
        code = wait(h.get_result_async(), 600).result.error_code.val
        rec['exec_code'] = CODES.get(code, code)
        rec['executed'] = code == MoveItErrorCodes.SUCCESS
    if not rec['executed']:
        settle()   # an aborted arm may still be coming to rest
    spin_sim(1.0)
    last = dict(zip(jt.joint_names, jt.points[-1].positions))
    rec['final_err_deg'] = round(math.degrees(max(abs(joints[n] - v) for n, v in last.items())), 3)
    return rec


def path_ok(rec):
    return rec['planned'] and rec['executed'] and rec['through_structure'] == 0


def describe(rec):
    if not rec['planned']:
        return f"no plan ({', '.join(str(r['code']) for r in rec['requests'])})"
    t = rec['requests'][-1]['planning_time_s']
    s = (f"planned in {t} s (request {len(rec['requests'])}), "
         f"{'executed' if rec['executed'] else 'execution ' + str(rec['exec_code'])}, "
         f"{rec['dense_samples']} samples: {rec['through_structure']} through the structure, "
         f"{rec['inside_margin']} inside the margin only")
    return s


def away_deg():
    return math.degrees(max(abs(joints[j] - HOME[j]) for j in ARM))


def reset_home(speed=0.3):
    """Simulation only: drive the arm controller straight home, slowly. Used when MoveIt cannot
    plan home, e.g. after an abort left the arm in collision or a hair past a joint limit."""
    if not arm_ctl.wait_for_server(timeout_sec=10):
        return 'arm controller action missing'
    sec = max(3.0, max(abs(joints[j]) for j in ARM) / speed)
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = ARM
    pt = JointTrajectoryPoint()
    pt.positions = [float(HOME[j]) for j in ARM]
    pt.time_from_start = Duration(sec=int(sec), nanosec=int(sec % 1 * 1e9))
    g.trajectory.points = [pt]
    h = wait(arm_ctl.send_goal_async(g), 20)
    code = wait(h.get_result_async(), 3 * sec + 60).result.error_code if h and h.accepted else 'rejected'
    settle()
    return f'controller result {code}'


def go_home(name, tol_deg=0.5, tries=3):
    """Bring the arm home with MoveIt (a failed home return is a failed check); if MoveIt cannot,
    reset it through the controller so the next goal starts from home."""
    if away_deg() <= tol_deg:
        return
    recs = []
    for _ in range(tries):
        recs.append(run(joint_goal(HOME)))
        if recs[-1]['executed'] and away_deg() <= tol_deg:
            break
    ok = recs[-1]['executed'] and away_deg() <= tol_deg and all(r.get('through_structure', 0) == 0 for r in recs)
    detail = f"{len(recs)} attempt(s); last: {describe(recs[-1])}"
    extra = {}
    if away_deg() > tol_deg:
        valid, cs = check({j: joints[j] for j in ARM})
        why = cs + [f'{j} out of bounds ({joints[j]:.4f})' for j, (lo, hi) in LIMITS.items()
                    if not lo <= joints[j] <= hi]
        if args.no_reset:
            detail += f'; arm left {away_deg():.1f} deg from home ({why})'
        else:
            extra['reset'] = {'why': why, 'result': reset_home()}
            detail += f"; reset through the controller ({why or 'MoveIt failed from a valid state'}), " \
                      f'now {away_deg():.2f} deg from home'
    report(f'{name}: back home', ok, detail, runs=recs, **extra)


# ------------------------------------------------------------------- validity
if 'validity' in SECTIONS:
    srdf = get_param(params, 'robot_description_semantic')
    for gs in ET.fromstring(srdf).findall('group_state'):
        q = {j.get('name'): float(j.get('value')) for j in gs.findall('joint')}
        if gs.get('group') != 'xarm6':
            q.update(HOME)   # a gripper state, with the arm at home
        ok, cs = check(q, group=gs.get('group'))
        rok, rcs = check(q, ref_validity, group=gs.get('group'))
        report(f"valid: {gs.get('group')}/{gs.get('name')}", ok and rok,
               f'move_group {ok} {cs}, true model {rok} {rcs}')
    ok, cs = check(FOLDED)
    rok, rcs = check(FOLDED, ref_validity)
    report('invalid: folded wrist (j5 1.2, j6 -2.5)', not ok and not rok,
           f'move_group {ok} {cs}, true model {rok} {rcs}')

# ------------------------------------------------------------------- regular goals
if 'goals' in SECTIONS:
    go_home('start')
    for name, kind, q in GOALS:
        if kind == 'gripper':
            rec = run(joint_goal(q), group='xarm_gripper')
        elif kind == 'pose':
            rec = run(pose_goal(fk(q)))
        else:
            rec = run(joint_goal(q))
        report(f'goal: {name} [{kind}]', path_ok(rec), describe(rec), run=rec)
        if kind != 'gripper' and not rec['executed'] and rec['planned']:
            go_home(f'goal: {name}')   # an abort leaves the arm somewhere unplanned

# ------------------------------------------------------------------- near-gantry goals
if 'gantry' in SECTIONS:
    go_home('start')
    for name, q in NEAR:
        ok, cs = check(q)
        rok, rcs = check(q, ref_validity)
        report(f'{name}: goal state valid', ok and rok, f'move_group {ok} {cs}, true model {rok} {rcs}')
        target = fk(q)
        for kind, cons in (('joint', joint_goal(q)), ('pose', pose_goal(target))):
            rec = run(cons)
            report(f'{name} [{kind}]', path_ok(rec), describe(rec), run=rec)
            go_home(f'{name} [{kind}]')
    for name, q in INTO:
        ok, cs = check(q)
        rok, rcs = check(q, ref_validity)
        report(f'{name}: goal state invalid', not ok and not rok, f'move_group {ok} {cs}, true model {rok} {rcs}')
        res, rec = plan(joint_goal(q), 'xarm6')
        report(f'{name}: no plan', res is None,
               ', '.join(str(r['code']) for r in rec['requests']) + ('' if res is None else ' (it planned)'), run=rec)
        if res is not None:
            go_home(name)

# ------------------------------------------------------------------- summary
checks = results['checks']
failed = [c['check'] for c in checks if not c['passed']]
results.update(finished=time.strftime('%Y-%m-%dT%H:%M:%S'), passed=len(checks) - len(failed),
               failed=failed, total=len(checks))
(OUT / 'results.json').write_text(json.dumps(results, indent=1))
print(f'{len(checks) - len(failed)}/{len(checks)} passed; results: {OUT / "results.json"}')
for f in failed:
    print('  failed:', f)
node.destroy_node()
rclpy.shutdown()
sys.exit(1 if failed else 0)
