#!/usr/bin/env python3
"""Arc-tracking probe: does the base fail to turn because the wheels do?

Drives straight onto the terrain, then holds the same arc the scored
drive uses (vx 0.30 m/s, wz 0.40 rad/s) and logs, per corner, what
ranger_4wis_controller.py asked for against what the joint did: wheel
speed, steer angle, and the effort it spent getting there against the
max_wheel_effort clamp. It also fits one rigid-body twist to the four
measured wheel velocities, which says whether the wheels agree with each
other, and compares the yaw rate that fit implies with ground truth.

That separates the explanations for gz under-rotating on rough ground:

  wheels miss their targets, efforts pinned at the clamp -> the drive
  steer angles knocked off target                         -> the knuckles
  wheels hit their targets, fit consistent, truth yaw low -> the contact:
      the base slides even though the drive did its job

Ground-truth yaw rate is compared rather than planar velocity because
yaw rate is the same in any frame, which sidesteps whether a simulator
publishes body- or world-frame twist.

Needs gz's set_pose service for the reset, so it is gz-only as written.
"""
import math
import statistics
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')
R, LX, LY = 0.100036, 0.2470, 0.1852
POS = {'front_left': (LX, LY), 'front_right': (LX, -LY),
       'rear_left': (-LX, LY), 'rear_right': (-LX, -LY)}
MAX_EFF = 30.0
ARC = (0.30, 0.0, 0.40)


def ik(vx, vy, wz):
    """ranger_4wis_controller.py's inverse kinematics, flip included."""
    out = {}
    for c in CORNERS:
        xi, yi = POS[c]
        vxi, vyi = vx - wz * yi, vy + wz * xi
        speed, angle = math.hypot(vxi, vyi), math.atan2(vyi, vxi)
        if angle > math.pi / 2:
            angle, speed = angle - math.pi, -speed
        elif angle < -math.pi / 2:
            angle, speed = angle + math.pi, -speed
        out[c] = (speed, angle)
    return out


# Least squares from per-wheel contact velocity to body twist: rows are
# [1, 0, -yi] and [0, 1, xi], the same model wheel_odometry.py uses.
A = np.array([row for c in CORNERS
              for row in ([1, 0, -POS[c][1]], [0, 1, POS[c][0]])], float)
A_PINV = np.linalg.pinv(A)


class Probe(Node):
    def __init__(self):
        super().__init__('arc_probe')
        self.js = self.gt = self.eff = self.steer_cmd = None
        self.rec = False
        self.rows = []
        self.create_subscription(DynamicJointState, '/dynamic_joint_states',
                                 self._on_js, 50)
        self.create_subscription(Odometry, '/ground_truth/odom',
                                 lambda m: setattr(self, 'gt', m), 50)
        self.create_subscription(Float64MultiArray, '/ranger_wheel_controller/commands',
                                 lambda m: setattr(self, 'eff', list(m.data)), 50)
        self.create_subscription(Float64MultiArray, '/ranger_steer_controller/commands',
                                 lambda m: setattr(self, 'steer_cmd', list(m.data)), 50)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    @staticmethod
    def _iface(msg, joint, name):
        if joint not in msg.joint_names:
            return None
        iv = msg.interface_values[msg.joint_names.index(joint)]
        return iv.values[iv.interface_names.index(name)] if name in iv.interface_names else None

    def _on_js(self, msg):
        self.js = msg
        if not (self.rec and self.gt is not None and self.eff is not None):
            return
        steer = {c: self._iface(msg, f'{c}_steer_joint', 'position') for c in CORNERS}
        rate = {c: self._iface(msg, f'{c}_wheel_joint', 'velocity') for c in CORNERS}
        if any(v is None for v in list(steer.values()) + list(rate.values())):
            return
        b = []
        for c in CORNERS:
            u = rate[c] * R
            b += [u * math.cos(steer[c]), u * math.sin(steer[c])]
        b = np.array(b)
        twist = A_PINV @ b
        resid = float(np.linalg.norm(A @ twist - b) / math.sqrt(len(b)))
        self.rows.append({'steer': steer, 'rate': rate, 'eff': list(self.eff),
                          'wz_wheels': float(twist[2]), 'resid': resid,
                          'wz_truth': self.gt.twist.twist.angular.z})

    def sim_t(self):
        s = self.gt.header.stamp
        return s.sec + s.nanosec * 1e-9

    def hold(self, vx, vy, wz, seconds):
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = float(vx), float(vy), float(wz)
        t0 = self.sim_t()
        while self.sim_t() - t0 < seconds and rclpy.ok():
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.01)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else 'arc'
    lead_in = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    subprocess.run(
        ['gz', 'service', '-s', '/world/empty_ground/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '3000', '--req',
         'name: "ranger_xarm", position: {x: 0, y: 0, z: 0.15}, orientation: {w: 1}'],
        capture_output=True, timeout=20)

    rclpy.init()
    n = Probe()
    t0 = time.time()
    while (n.gt is None or n.js is None) and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.hold(0, 0, 0, 2.0)                       # settle after the reset
    n.hold(0.35, 0, 0, lead_in)                # straight, onto the terrain
    p = n.gt.pose.pose.position
    y0 = yaw(n.gt.pose.pose.orientation)
    n.hold(*ARC, 1.0)                          # let the steering arrive
    n.rec = True
    ta = n.sim_t()
    n.hold(*ARC, 5.0)
    n.rec = False
    arc_t = n.sim_t() - ta
    n.hold(0, 0, 0, 0.5)
    dyaw = math.degrees(yaw(n.gt.pose.pose.orientation) - y0)
    dyaw = (dyaw + 180) % 360 - 180

    tgt = ik(*ARC)
    rows = n.rows
    print(f'--- {label}: arc started at x={p.x:+.2f} y={p.y:+.2f}, '
          f'{len(rows)} samples over {arc_t:.1f} s sim ---')
    print(f"{'corner':12s} {'speed tgt':>9s} {'actual':>8s} {'err':>7s} "
          f"{'steer tgt':>9s} {'actual':>8s} {'effort':>7s} {'at clamp':>8s}")
    for i, c in enumerate(CORNERS):
        sp = [r['rate'][c] * R for r in rows]
        st = [math.degrees(r['steer'][c]) for r in rows]
        ef = [r['eff'][i] for r in rows]
        clamp = sum(1 for e in ef if abs(e) >= MAX_EFF - 0.1) / len(ef)
        print(f'{c:12s} {tgt[c][0]:+9.3f} {statistics.mean(sp):+8.3f} '
              f'{100 * (statistics.mean(sp) - tgt[c][0]) / tgt[c][0]:+6.0f}% '
              f'{math.degrees(tgt[c][1]):+9.1f} {statistics.mean(st):+8.1f} '
              f'{statistics.mean(ef):+7.2f} {100 * clamp:7.0f}%')
    wzw = statistics.mean(r['wz_wheels'] for r in rows)
    wzt = statistics.mean(r['wz_truth'] for r in rows)
    res = statistics.mean(r['resid'] for r in rows)
    print(f'yaw rate  commanded {ARC[2]:+.3f} | wheels imply {wzw:+.3f} | '
          f'ground truth {wzt:+.3f} rad/s  ({100 * wzt / ARC[2]:.0f}% of command)')
    print(f'wheel rigid-body fit residual {res:.4f} m/s rms '
          f'(0 = the four wheels agree on one twist)')
    print(f'heading change over the {1.0 + arc_t:.1f} s arc {dyaw:+.1f} deg '
          f'(commanded {math.degrees(ARC[2] * (1.0 + arc_t)):+.1f})')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
