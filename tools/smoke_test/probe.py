#!/usr/bin/env python3
"""Check a running simulation of the robot, whichever simulator it is.

    probe.py [--drive] [--sensors] [--arm]

Always: /joint_states arrives and /clock advances. --sensors: the Ouster
cloud and IMU, the RPLIDAR scan and the D435 colour image publish.
--drive: forward, crab and spin-in-place commands on /cmd_vel each move the
base along the commanded axis, measured by /ground_truth/odom, and /odom is
reported beside it. --arm: the arm's trajectory controller reaches a pose.

Prints one PASS/FAIL line per check; exits 0 only if every check passed.
The thresholds are for "the bringup works", not for tracking accuracy: at
least half the commanded travel, under 15 cm off-axis. A healthy run gets
80-96 % (the shortfall is the steering and acceleration ramp).
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
    # gz bridges the camera to /camera/d435/image; Isaac and the real
    # driver use /camera/d435/color/image_raw.
    subs += [
        node.create_subscription(PointCloud2, '/ouster/points', keep('ouster'), qos_profile_sensor_data),
        node.create_subscription(Imu, '/ouster/imu', keep('imu'), qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/scan', keep('scan'), qos_profile_sensor_data),
        node.create_subscription(Image, '/camera/d435/color/image_raw', keep('rgb'), qos_profile_sensor_data),
        node.create_subscription(Image, '/camera/d435/image', keep('rgb'), qos_profile_sensor_data),
    ]
cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)


def spin(sec, twist=None):
    end = time.time() + sec
    while time.time() < end:
        if twist is not None:
            cmd_pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)


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
                      ('scan', '/scan'), ('rgb', 'D435 colour image')]:
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
        spin(4.0, tw)
        b_truth, b_odom, s1 = pose('truth'), pose('odom'), sim_time()
        spin(2.5, Twist())                      # stop and settle before the next
        if None in (a_truth, b_truth, a_odom, b_odom, s0, s1):
            check(name, False, 'no odometry or clock')
            continue
        truth, odom = in_start_frame(a_truth, b_truth), in_start_frame(a_odom, b_odom)
        commanded = twist_xyw[axis] * (s1 - s0)
        frac = truth[axis] / commanded
        off_axis = max(abs(truth[i]) for i in (0, 1) if i != axis)
        check(name, frac > 0.5 and off_axis < 0.15,
              f"truth {truth[0]:+.2f} m {truth[1]:+.2f} m {math.degrees(truth[2]):+.0f} deg "
              f"({100 * frac:.0f} % of commanded over {s1 - s0:.1f} s sim); "
              f"odom {odom[0]:+.2f} m {odom[1]:+.2f} m {math.degrees(odom[2]):+.0f} deg")


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

node.destroy_node()
rclpy.shutdown()
print(f"{sum(results)}/{len(results)} passed", flush=True)
sys.exit(0 if all(results) else 1)
