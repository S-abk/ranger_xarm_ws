#!/usr/bin/env python3
"""Where does wheel odometry's crab yaw come from?

Bypasses ranger_4wis_controller.py and drives the steer and wheel
controllers directly, so each phase of a crab can be isolated:

  sweep   knuckles swing to 90 deg with the wheels commanded to zero
  crab    wheels drive at 0.3 m/s with the knuckles held at 90 deg
  stop    wheels commanded to zero, knuckles still at 90 deg

and for each phase reports what wheel odometry (/odom) and ground truth
say the base did, the travel of each wheel, and the measured knuckle
angles. The sweep starts either from the arc's steer angles, which is
what precedes the crab in the scored drive (front knuckles then swing
66-75 deg, rear 105-114), or from 0 deg, where all four swing equally.

The 4WIS node is stopped for the duration, since it republishes both
commands at 100 Hz; restart the stack afterwards. Velocity wheel drive
only: wheel commands are sent as rad/s.
"""
import math
import os
import signal
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')
R = 0.100036
ARC_STEER = {'front_left': 23.6, 'front_right': 14.8,
             'rear_left': -23.6, 'rear_right': -14.8}


def stop_4wis():
    me = os.getpid()
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == me:
            continue
        try:
            cmd = open(f'/proc/{pid}/cmdline', 'rb').read().replace(b'\0', b' ').decode()
        except OSError:
            continue
        if 'lib/ranger_xarm_gazebo/ranger_4wis_controller.py' in cmd:
            os.kill(int(pid), signal.SIGKILL)
    time.sleep(1.0)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__('crab_probe')
        self.o = self.g = self.js = None
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'o', m), 20)
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'g', m), 20)
        self.create_subscription(DynamicJointState, '/dynamic_joint_states',
                                 lambda m: setattr(self, 'js', m), 20)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/ranger_steer_controller/commands', 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/ranger_wheel_controller/commands', 10)
        self.samples = []
        self.travel = {c: 0.0 for c in CORNERS}
        self._last_w = None

    def iface(self, joint, name):
        m = self.js
        iv = m.interface_values[m.joint_names.index(joint)]
        return iv.values[iv.interface_names.index(name)]

    def t(self):
        s = self.g.header.stamp
        return s.sec + s.nanosec * 1e-9

    def snap(self):
        o, g = self.o.pose.pose, self.g.pose.pose
        return {'o': (o.position.x, o.position.y, yaw(o.orientation)),
                'g': (g.position.x, g.position.y, yaw(g.orientation)),
                'w': dict(self.travel),
                's': {c: self.iface(f'{c}_steer_joint', 'position') for c in CORNERS}}

    def _accumulate(self):
        w = {c: self.iface(f'{c}_wheel_joint', 'position') for c in CORNERS}
        if self._last_w is not None:
            for c in CORNERS:
                self.travel[c] += unwrap(w[c] - self._last_w[c])
        self._last_w = w

    def hold(self, steer_deg, wheel_rate, seconds, record=False):
        st = Float64MultiArray(data=[math.radians(steer_deg[c]) for c in CORNERS])
        wh = Float64MultiArray(data=[float(wheel_rate)] * 4)
        t0 = self.t()
        while self.t() - t0 < seconds and rclpy.ok():
            self.steer_pub.publish(st)
            self.wheel_pub.publish(wh)
            rclpy.spin_once(self, timeout_sec=0.01)
            self._accumulate()
            if record:
                self.samples.append({c: math.degrees(self.iface(f'{c}_steer_joint', 'position'))
                                     for c in CORNERS})


def unwrap(d):
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def report(name, a, b):
    """Odometry and ground truth, both expressed in the frame of snapshot a."""
    def body_delta(p0, p1):
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        c, s = math.cos(-p0[2]), math.sin(-p0[2])
        return c * dx - s * dy, s * dx + c * dy, math.degrees(unwrap(p1[2] - p0[2]))
    ox, oy, oyaw = body_delta(a['o'], b['o'])
    gx, gy, gyaw = body_delta(a['g'], b['g'])
    travel = {c: (b['w'][c] - a['w'][c]) * R * 1000 for c in CORNERS}
    print(f'{name:6s} odom  dx {ox:+.3f} dy {oy:+.3f} m  dyaw {oyaw:+.2f} deg')
    print(f'{"":6s} truth dx {gx:+.3f} dy {gy:+.3f} m  dyaw {gyaw:+.2f} deg   '
          f'-> odom yaw error {oyaw - gyaw:+.2f} deg')
    print(f'{"":6s} wheel travel mm  ' +
          '  '.join(f'{c.replace("_", " ")[:9]:>9s} {travel[c]:+7.2f}' for c in CORNERS))


def main():
    start = sys.argv[1] if len(sys.argv) > 1 else 'arc'
    stop_4wis()
    rclpy.init()
    n = Probe()
    t0 = time.time()
    while (n.o is None or n.g is None or n.js is None) and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.1)
    begin = ARC_STEER if start == 'arc' else {c: 0.0 for c in CORNERS}
    crab = {c: 90.0 for c in CORNERS}
    n.hold(begin, 0.0, 3.0)
    a = n.snap()
    n.hold(crab, 0.0, 3.0)
    b = n.snap()
    n.hold(crab, 0.30 / R, 4.0, record=True)
    c = n.snap()
    n.hold(crab, 0.0, 1.5)
    d = n.snap()
    print(f'--- crab probe, sweep from {start} angles: '
          + ', '.join(f'{k.replace("_", " ")} {v:+.1f}' for k, v in begin.items()) + ' deg ---')
    report('sweep', a, b)
    report('crab', b, c)
    report('stop', c, d)
    ms = {k: statistics.mean(s[k] for s in n.samples) for k in CORNERS}
    print('measured knuckle angle during crab, deg: '
          + '  '.join(f'{k.replace("_", " ")} {v:+.3f}' for k, v in ms.items()))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
