#!/usr/bin/env python3
"""When, during a crab driven through ranger_4wis_controller.py, does
wheel odometry's heading part company with ground truth?

Reproduces the scored drive's lead-up (arc, stop, crab, stop) through
/cmd_vel with the real 4WIS node, and logs heading from /odom and from
ground truth alongside the measured knuckle angles and wheel rates. It
then splits the crab into the knuckle sweep (until all four are within
1 deg of +/-90; with shortest-path steering the rears finish at -90) and the steady part, and says how much heading error each
contributed.

gz resets the pose first; in Isaac the reset call simply fails and the
drive starts wherever the base is, which is fine on flat ground.
"""
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')
R = 0.100036


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrapd(a):
    return (a + 180.0) % 360.0 - 180.0


class T(Node):
    def __init__(self):
        super().__init__('crab_timeline')
        self.o = self.g = None
        self.rec = False
        self.rows = []
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'o', m), 50)
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'g', m), 50)
        self.create_subscription(DynamicJointState, '/dynamic_joint_states', self.on_js, 50)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def t(self):
        s = self.g.header.stamp
        return s.sec + s.nanosec * 1e-9

    def on_js(self, m):
        if not (self.rec and self.o is not None and self.g is not None):
            return
        def iv(j, name):
            v = m.interface_values[m.joint_names.index(j)]
            return v.values[v.interface_names.index(name)]
        self.rows.append({
            't': self.t(),
            'oy': math.degrees(yaw(self.o.pose.pose.orientation)),
            'gy': math.degrees(yaw(self.g.pose.pose.orientation)),
            'st': [math.degrees(iv(f'{c}_steer_joint', 'position')) for c in CORNERS],
            'wr': [iv(f'{c}_wheel_joint', 'velocity') * R for c in CORNERS],
        })

    def hold(self, vx, vy, wz, sec):
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = float(vx), float(vy), float(wz)
        t0 = self.t()
        while self.t() - t0 < sec and rclpy.ok():
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.005)


def main():
    subprocess.run(['gz', 'service', '-s', '/world/empty_ground/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000', '--req',
                    'name: "ranger_xarm", position: {x: 0, y: 0, z: 0.15}, orientation: {w: 1}'],
                   capture_output=True, timeout=20)
    rclpy.init()
    n = T()
    t0 = time.time()
    while (n.o is None or n.g is None) and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.hold(0, 0, 0, 2.0)
    # arc: leaves the knuckles at the arc angles. Its length sets the world
    # heading the crab happens at (0.4 rad/s, so 3 s -> ~69 deg, 6 s -> ~137).
    arc_s = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    n.hold(0.30, 0, 0.40, arc_s)
    n.hold(0, 0, 0, 1.5)
    n.rec = True
    tc = n.t()
    n.hold(0, 0.30, 0, 4.0)         # crab
    te = n.t()
    n.hold(0, 0, 0, 1.5)
    n.rec = False
    rows = n.rows
    r0_heading = rows[0]['gy'] if rows else float('nan')

    def at(tt):
        return min(rows, key=lambda r: abs(r['t'] - tt))
    swept = next((r['t'] for r in rows if r['t'] >= tc and all(abs(abs(a) - 90) < 1.0 for a in r['st'])), te)

    print(f'arc {arc_s:.1f} s -> crab at world heading {r0_heading:+.1f} deg')
    print(f'{"t":>5s}  {"odom yaw":>8s} {"truth":>7s} {"err":>6s}   knuckles FL FR RL RR (deg)'
          f'        wheel speed FL FR RL RR (m/s)')
    r0 = rows[0]
    for k in range(0, 16):
        r = at(tc + 0.1 * k)
        do, dg = wrapd(r['oy'] - r0['oy']), wrapd(r['gy'] - r0['gy'])
        print(f'{r["t"] - tc:5.2f}  {do:+8.2f} {dg:+7.2f} {do - dg:+6.2f}   '
              + ' '.join(f'{a:+6.1f}' for a in r['st']) + '    '
              + ' '.join(f'{w:+.3f}' for w in r['wr']))

    def seg(a, b):
        ra, rb = at(a), at(b)
        do, dg = wrapd(rb['oy'] - ra['oy']), wrapd(rb['gy'] - ra['gy'])
        return do, dg, do - dg
    for name, a, b in (('sweep (to all knuckles within 1 deg)', tc, swept),
                       ('steady crab', swept, te),
                       ('stop', te, rows[-1]['t'])):
        do, dg, e = seg(a, b)
        print(f'{name:38s} {b - a:5.2f} s  odom {do:+.2f}  truth {dg:+.2f}  error {e:+.2f} deg')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
