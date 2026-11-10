#!/usr/bin/env python3
"""Check a running simulation of the robot, whichever simulator it is.

    probe.py [--drive] [--sensors] [--arm] [--moveit]

Always: /joint_states arrives and /clock advances. --sensors: the Ouster
cloud and IMU, the RPLIDAR scan and the colour images of BOTH cameras
(gantry D455 and wrist D435) publish; each is reported and must pass
separately. --no-wrist-camera drops the wrist camera for a model without
the gripper plate (the wrist camera is mounted on it).
--drive: forward, crab and spin-in-place commands on /cmd_vel each move the
base along the commanded axis, measured by /ground_truth/odom, and /odom is
reported beside it. --arm: the arm's trajectory controller reaches a pose.
--moveit: move_group (ranger_xarm_isaac moveit.launch.py) plans and executes
one joint goal and one pose goal for xarm6 through the MoveGroup action, so
the collision model, SRDF and controller map are exercised end to end.

Prints one PASS/FAIL line per check; exits 0 only if every check passed.
The thresholds are for "the bringup works", not for tracking accuracy: at
least half the commanded travel, under 15 cm off-axis. Each drive segment
lasts a fixed DRIVE_SIM_SEC of *sim* time (from /clock), so the percentage
measures the base, not how fast the host runs the simulation; WALL_CAP_SEC
is only a safety limit. The shortfall from 100 % is the steering and
acceleration ramp.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, JointState, LaserScan, PointCloud2
from trajectory_msgs.msg import JointTrajectoryPoint

ap = argparse.ArgumentParser()
ap.add_argument('--drive', action='store_true')
ap.add_argument('--sensors', action='store_true')
ap.add_argument('--arm', action='store_true')
ap.add_argument('--moveit', action='store_true')
ap.add_argument('--no-wrist-camera', action='store_true',
                help='model has no gripper plate, so no wrist D435')
ap.add_argument('--arm-controller', default='/xarm_xarm6_traj_controller')
args = ap.parse_args()

rclpy.init()
node = Node('smoke_test_probe')
results = []


def check(name, ok, detail=''):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)


last, counts = {}, {}


def keep(key):
    def cb(msg):
        last[key] = msg
        counts[key] = counts.get(key, 0) + 1
    return cb


subs = [
    node.create_subscription(JointState, '/joint_states', keep('js'), 10),
    node.create_subscription(Clock, '/clock', keep('clock'), qos_profile_sensor_data),
    node.create_subscription(Odometry, '/odom', keep('odom'), 10),
    node.create_subscription(Odometry, '/ground_truth/odom', keep('truth'), 10),
]
if args.sensors:
    # Isaac, the real driver and the gz bridge's remapped topics all use
    # /camera/<name>/color/image_raw; the unremapped gz name
    # /camera/<name>/image is listened to as well. Each camera has its own key.
    cameras = {'gantry_d455': 'Gantry D455 colour image'}
    if not args.no_wrist_camera:
        cameras['wrist_d435'] = 'Wrist D435 colour image'
    for cam in cameras:
        for topic in (f'/camera/{cam}/color/image_raw', f'/camera/{cam}/image'):
            subs.append(node.create_subscription(
                Image, topic, keep(f'rgb_{cam}'), qos_profile_sensor_data))
    subs += [
        node.create_subscription(PointCloud2, '/ouster/points', keep('ouster'), qos_profile_sensor_data),
        node.create_subscription(Imu, '/ouster/imu', keep('imu'), qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/scan', keep('scan'), qos_profile_sensor_data),
    ]
cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)


def spin(sec, twist=None):
    end = time.time() + sec
    while time.time() < end:
        if twist is not None:
            cmd_pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)


DRIVE_SIM_SEC = 2.4    # sim seconds per drive segment (the old 4 s wall at RTF 0.6)
SETTLE_SIM_SEC = 1.5   # sim seconds of zero command between segments
WALL_CAP_SEC = 40.0    # safety limit per segment; reached only below RTF ~0.06


def spin_sim(sim_sec, twist=None, cap=WALL_CAP_SEC):
    """Spin until sim_sec of /clock time has passed (or cap wall seconds). Returns True if capped."""
    start_wall = time.time()
    start = sim_time()
    while True:
        if twist is not None:
            cmd_pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)
        now = sim_time()
        if start is None:
            start = now
        elif now is not None and now - start >= sim_sec:
            return False
        if time.time() - start_wall > cap:
            return True


def sim_time():
    c = last.get('clock')
    return c.clock.sec + c.clock.nanosec * 1e-9 if c else None


def rates(sec):
    counts.clear()
    start = time.time()
    spin(sec)
    return {k: v / (time.time() - start) for k, v in counts.items()}


# --- always
start = time.time()
while 'js' not in last and time.time() - start < 30.0:
    spin(0.2)
first = time.time() - start
hz = rates(5.0)
js = last.get('js')
check('/joint_states', js is not None and hz.get('js', 0) > 5,
      f"{len(js.name)} joints, first after {first:.1f} s, {hz.get('js', 0):.0f} Hz"
      if js else 'nothing in 30 s')

t0, w0 = sim_time(), time.time()
spin(3.0)
t1 = sim_time()
if t0 is not None and t1 is not None:
    check('/clock advancing', t1 > t0, f"real-time factor {(t1 - t0) / (time.time() - w0):.2f}")
else:
    check('/clock advancing', False, 'no /clock')

# --- sensors
if args.sensors:
    hz = rates(5.0)
    for key, name in [('ouster', '/ouster/points'), ('imu', '/ouster/imu'),
                      ('scan', '/scan')] + [(f'rgb_{c}', n) for c, n in cameras.items()]:
        check(name, hz.get(key, 0) > 0.5, f"{hz.get(key, 0):.1f} Hz")
    cloud = last.get('ouster')
    if cloud is not None:
        check('Ouster cloud non-empty', cloud.width * cloud.height > 0,
              f"{cloud.width}x{cloud.height} in {cloud.header.frame_id}")


# --- base
def pose(key):
    m = last.get(key)
    if m is None:
        return None
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def in_start_frame(a, b):
    """b - a, expressed in a's body frame: (forward, left, yaw)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))


if args.drive:
    spin(1.0)
    check('/odom', 'odom' in last)
    check('/ground_truth/odom', 'truth' in last)
    for name, twist_xyw, axis in [('forward 0.3 m/s', (0.3, 0.0, 0.0), 0),
                                  ('crab 0.3 m/s', (0.0, 0.3, 0.0), 1),
                                  ('spin 0.5 rad/s', (0.0, 0.0, 0.5), 2)]:
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = twist_xyw
        a_truth, a_odom, s0 = pose('truth'), pose('odom'), sim_time()
        capped = spin_sim(DRIVE_SIM_SEC, tw)
        b_truth, b_odom, s1 = pose('truth'), pose('odom'), sim_time()
        spin_sim(SETTLE_SIM_SEC, Twist())       # stop and settle before the next
        if None in (a_truth, b_truth, a_odom, b_odom, s0, s1):
            check(name, False, 'no odometry or clock')
            continue
        truth, odom = in_start_frame(a_truth, b_truth), in_start_frame(a_odom, b_odom)
        commanded = twist_xyw[axis] * (s1 - s0)
        frac = truth[axis] / commanded
        off_axis = max(abs(truth[i]) for i in (0, 1) if i != axis)
        check(name, frac > 0.5 and off_axis < 0.15 and not capped,
              f"truth {truth[0]:+.2f} m {truth[1]:+.2f} m {math.degrees(truth[2]):+.0f} deg "
              f"({100 * frac:.0f} % of commanded over {s1 - s0:.1f} s sim); "
              f"odom {odom[0]:+.2f} m {odom[1]:+.2f} m {math.degrees(odom[2]):+.0f} deg"
              + (f"; hit the {WALL_CAP_SEC:.0f} s wall cap" if capped else ""))


# --- arm
def move_arm(client, degrees, sec=3.0):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [f'xarm_joint{i}' for i in range(1, 7)]
    pt = JointTrajectoryPoint()
    pt.positions = [math.radians(d) for d in degrees]
    pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
    goal.trajectory.points = [pt]
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        return 'goal not accepted'
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=60.0)
    if res.result() is None:
        return 'no result in 60 s'
    return res.result().result.error_code


if args.arm:
    client = ActionClient(node, FollowJointTrajectory,
                          f'{args.arm_controller}/follow_joint_trajectory')
    if not client.wait_for_server(timeout_sec=20.0):
        check('arm to joint2 -35 deg', False, f'{args.arm_controller} has no action server')
    else:
        code = move_arm(client, [0, -35, 0, 0, 0, 0])
        spin(1.0)
        js = last['js']
        j2 = math.degrees(js.position[js.name.index('xarm_joint2')]) if 'xarm_joint2' in js.name else None
        check('arm to joint2 -35 deg', code == 0 and j2 is not None and abs(j2 + 35) < 2,
              f"result {code}, joint2 {j2:.1f} deg" if j2 is not None else f"result {code}, no xarm_joint2")
        move_arm(client, [0, 0, 0, 0, 0, 0])

# --- moveit
MOVEIT_JOINT_DEG = [20, -10, -40, 0, 50, 30]   # clear of the gantry, the deck and the camera's reach
MOVEIT_POSE_DEG = [-25, 20, -60, 0, 40, -20]    # its TCP pose becomes the pose goal


def moveit_goal(action, constraints):
    """Plan and execute through MoveGroup.

    Returns (MoveItErrorCodes value or text, planned point count, the plan's last arm point)."""
    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = 'xarm6'
    req.num_planning_attempts = 3
    req.allowed_planning_time = 10.0
    req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = 0.3
    req.start_state.is_diff = True
    req.goal_constraints = [constraints]
    goal.planning_options.plan_only = False
    fut = action.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        return 'goal not accepted', 0, None
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=120.0)
    if res.result() is None:
        return 'no result in 120 s', 0, None
    r = res.result().result
    jt = r.planned_trajectory.joint_trajectory
    end = None
    if jt.points and all(f'xarm_joint{i}' in jt.joint_names for i in range(1, 7)):
        end = [jt.points[-1].positions[jt.joint_names.index(f'xarm_joint{i}')] for i in range(1, 7)]
    return r.error_code.val, len(jt.points), end


def joint_constraints(degrees):
    c = Constraints()
    for i, d in enumerate(degrees):
        jc = JointConstraint()
        jc.joint_name, jc.position = f'xarm_joint{i + 1}', math.radians(d)
        jc.tolerance_above = jc.tolerance_below = 0.001
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    return c


def tcp_pose(fk, positions):
    """TCP pose in move_group's model frame (returned as a PoseStamped).

    The header is left empty on purpose: any other frame makes /compute_fk
    convert through live TF, and once the base has driven that includes
    odometry, so the pose no longer matches what a constraint in a robot
    link frame means to the planner."""
    req = GetPositionFK.Request()
    req.fk_link_names = ['xarm_link_tcp']
    req.robot_state.is_diff = True
    req.robot_state.joint_state.name = [f'xarm_joint{i}' for i in range(1, 7)]
    req.robot_state.joint_state.position = [float(p) for p in positions]
    fut = fk.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    r = fut.result()
    return r.pose_stamped[0] if r is not None and r.pose_stamped else None


def pose_constraints(stamped):
    pose, frame = stamped.pose, stamped.header.frame_id
    c = Constraints()
    pc = PositionConstraint()
    pc.header.frame_id, pc.link_name, pc.weight = frame, 'xarm_link_tcp', 1.0
    sphere = SolidPrimitive()
    sphere.type, sphere.dimensions = SolidPrimitive.SPHERE, [0.005]
    centre = Pose()
    centre.position = pose.position
    centre.orientation.w = 1.0
    pc.constraint_region.primitives.append(sphere)
    pc.constraint_region.primitive_poses.append(centre)
    oc = OrientationConstraint()
    oc.header.frame_id, oc.link_name, oc.weight = frame, 'xarm_link_tcp', 1.0
    oc.orientation = pose.orientation
    oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 0.02
    c.position_constraints.append(pc)
    c.orientation_constraints.append(oc)
    return c


def arm_now():
    # Settle on sim time, not wall time: at a real-time factor of 0.2 a wall
    # second is a fifth of a simulated one.
    spin_sim(SETTLE_SIM_SEC)
    js = last['js']
    return [js.position[js.name.index(f'xarm_joint{i}')] for i in range(1, 7)]


if args.moveit:
    from geometry_msgs.msg import Pose
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes,
                                 OrientationConstraint, PositionConstraint)
    from moveit_msgs.srv import GetPositionFK
    from shape_msgs.msg import SolidPrimitive

    action = ActionClient(node, MoveGroup, '/move_action')
    fk = node.create_client(GetPositionFK, '/compute_fk')
    if not action.wait_for_server(timeout_sec=60.0) or not fk.wait_for_service(timeout_sec=10.0):
        check('MoveIt joint goal', False, 'no move_group (/move_action, /compute_fk)')
        check('MoveIt pose goal', False, 'no move_group')
    else:
        code, n, _ = moveit_goal(action, joint_constraints(MOVEIT_JOINT_DEG))
        q = arm_now()
        err = max(abs(math.degrees(a) - d) for a, d in zip(q, MOVEIT_JOINT_DEG))
        check('MoveIt joint goal', code == MoveItErrorCodes.SUCCESS and err < 2.0,
              f'result {code}, {n} points, worst joint {err:.2f} deg off')

        target = tcp_pose(fk, [math.radians(d) for d in MOVEIT_POSE_DEG])
        if target is None:
            check('MoveIt pose goal', False, '/compute_fk gave no pose')
        else:
            code, n, end = moveit_goal(action, pose_constraints(target))
            q = arm_now()
            got = tcp_pose(fk, q)
            off = 1000 * math.dist((got.pose.position.x, got.pose.position.y, got.pose.position.z),
                                   (target.pose.position.x, target.pose.position.y, target.pose.position.z)) if got else float('inf')
            jerr = (f', worst joint {max(abs(math.degrees(a - b)) for a, b in zip(q, end)):.2f} deg from the plan'
                    if end else '')
            check('MoveIt pose goal', code == MoveItErrorCodes.SUCCESS and off < 10.0,
                  f'result {code}, {n} points, TCP {off:.1f} mm from target{jerr}')
        moveit_goal(action, joint_constraints([0, 0, 0, 0, 0, 0]))

node.destroy_node()
rclpy.shutdown()
print(f"{sum(results)}/{len(results)} passed", flush=True)
sys.exit(0 if all(results) else 1)
