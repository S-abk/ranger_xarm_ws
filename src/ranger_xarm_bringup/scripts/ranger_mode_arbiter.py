#!/usr/bin/env python3
"""Reduce a twist to one of the Ranger's steering modes before it drives.

The Ranger Mini's wheels steer independently but move in modes (AgileX's
ranger_ros2 driver, TwistCmdCallback): parallel when linear.y != 0 (all
wheels at one angle; angular.z is ignored), spinning when |vx|/|wz| is
below the minimum turn radius (linear.x is ignored), otherwise dual
Ackermann (vx and a steering angle; linear.y must be exactly zero). A
planner's twist is none of those: MPPI's vy is almost never exactly zero,
so sent raw the driver would sit in parallel mode and never turn.

This node picks the mode the twist is mostly asking for and sends a twist
that is exactly that mode, so the driver (or ranger_4wis_controller.py in
simulation, which mirrors it) makes the same choice:

- parallel if the lateral speed outweighs the rotation, compared as
  |vy| against |wz| * lateral_ref (the speed the rotation gives a point
  lateral_ref from the centre): (vx, vy, 0);
- otherwise spinning if |vx|/|wz| < min_turn_radius: (0, 0, wz);
- otherwise dual Ackermann: (vx, 0, wz), wz limited to what the steering
  cap allows.

A mode change costs a knuckle swing, so the mode only changes when the new
one dominates by switch_margin and the current one has lasted mode_hold, or
when the base is stopped. Commands below the deadbands are sent as zero.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class RangerModeArbiter(Node):
    def __init__(self):
        super().__init__('ranger_mode_arbiter')
        p = self.declare_parameter
        p('cmd_in', 'cmd_vel_raw')
        p('cmd_out', 'cmd_vel')
        p('wheelbase', 0.494)               # AgileX Ranger Mini V3 (ranger_params.hpp)
        p('track', 0.364)
        p('min_turn_radius', 0.4764)
        p('max_steer_ackermann', 0.601)     # rad
        p('lateral_ref', 0.30)              # m
        p('switch_margin', 1.5)
        p('mode_hold', 0.4)                 # s
        p('linear_deadband', 0.02)          # m/s
        p('angular_deadband', 0.05)         # rad/s
        g = lambda n: self.get_parameter(n).value
        self.cfg = {n: float(g(n)) for n in (
            'wheelbase', 'track', 'min_turn_radius', 'max_steer_ackermann', 'lateral_ref',
            'switch_margin', 'mode_hold', 'linear_deadband', 'angular_deadband')}
        # The tightest dual-Ackermann turn: the driver steers atan((l/2)/R),
        # capped, and spins below min_turn_radius; for the Mini V3 the cap
        # (R = 0.36 m) lies inside that, so the threshold is the limit.
        c = self.cfg
        phi_i = min(c['max_steer_ackermann'], math.radians(40.0))
        self.r_ack_min = max(c['min_turn_radius'], (c['wheelbase'] / 2) / math.tan(phi_i))
        self.mode = 'stop'
        self.since = self.get_clock().now()
        self.pub = self.create_publisher(Twist, g('cmd_out'), 10)
        self.mode_pub = self.create_publisher(String, 'ranger_mode', 10)
        self.create_subscription(Twist, g('cmd_in'), self._on_cmd, 10)
        self.get_logger().info(
            f"{g('cmd_in')} -> {g('cmd_out')}: parallel / spinning / dual Ackermann "
            f'(min turn radius {c["min_turn_radius"]:g} m, tightest Ackermann '
            f'{self.r_ack_min:.3f} m)')

    def _wanted(self, vx, vy, wz):
        c = self.cfg
        lat, rot = abs(vy), abs(wz) * c['lateral_ref']
        if lat > rot:
            return 'parallel', lat, rot
        if abs(wz) > 1e-9 and abs(vx) / abs(wz) < c['min_turn_radius']:
            return 'spinning', rot, lat
        return 'ackermann', max(abs(vx), rot), lat

    def _on_cmd(self, msg):
        c = self.cfg
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        out = Twist()
        now = self.get_clock().now()
        if math.hypot(vx, vy) < c['linear_deadband'] and abs(wz) < c['angular_deadband']:
            self.pub.publish(out)                        # stop; the mode may change freely
            if self.mode != 'stop':
                self.mode, self.since = 'stop', now
            return
        wanted, strength, other = self._wanted(vx, vy, wz)
        held = (now - self.since).nanoseconds * 1e-9
        if (wanted != self.mode and self.mode != 'stop' and
                (held < c['mode_hold'] or strength < c['switch_margin'] * other)):
            wanted = self.mode                           # not decisive enough to swing the knuckles
        if wanted != self.mode:
            self.mode, self.since = wanted, now
            self.mode_pub.publish(String(data=wanted))
        if wanted == 'parallel':
            out.linear.x = vx
            out.linear.y = vy if vy != 0.0 else math.copysign(1e-3, vy)
        elif wanted == 'spinning':
            out.angular.z = wz
        else:
            out.linear.x = vx
            if abs(vx) > 1e-9:
                out.angular.z = math.copysign(min(abs(wz), abs(vx) / self.r_ack_min), wz)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = RangerModeArbiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
