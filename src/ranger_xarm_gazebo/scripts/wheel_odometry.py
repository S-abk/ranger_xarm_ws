#!/usr/bin/env python3
"""Dead-reckon the 4WIS base from its wheel encoders alone.

Both simulators were publishing /odom, and in Isaac's case that /odom was
IsaacComputeOdometry: the chassis pose read straight out of the physics
engine. That is ground truth wearing odometry's name. Anything consuming
it sees a robot that never slips, never drifts and never accumulates
heading error, which is the one thing odometry always does. Feeding it to
a state estimator proves nothing about the estimator.

So this node computes what the real platform can actually know, from the
same two quantities its encoders report:

    steer angle   delta_i   (joint position)
    wheel travel  d_theta_i (joint position, differenced)

and ground truth moves to /ground_truth/odom, where it can be used as a
reference instead of as an input.

Forward kinematics is the inverse of the controller's. Each wheel's
contact point moves at omega_i * r along its steer direction, and for a
rigid body the same point moves at v + w x r:

    vx - wz*yi = omega_i * r * cos(delta_i)
    vy + wz*xi = omega_i * r * sin(delta_i)

Four wheels give eight equations in three unknowns, so it is solved by
least squares rather than by picking a wheel. That matters on a real
platform: when the wheels disagree, which they do whenever anything
slips, the residual is the disagreement, and a least-squares solution
degrades gracefully where a single-wheel solution simply believes
whichever wheel it was given.

WHEEL TRAVEL IS DIFFERENCED FROM POSITION, NOT READ FROM VELOCITY. A real
encoder counts, and counting is what makes odometry drift the way real
odometry drifts. Reading the simulator's velocity estimate instead would
inherit the simulator's own filtering and hide exactly the error this
node exists to expose. Joint position wraps at +/-pi in Isaac, so it is
unwrapped.

On wheel_radius, read this before trusting any number that comes out.
The radius is a MULTIPLIER on every distance reported here. It was
0.1026 m in this workspace until it was measured against the CAD and the
manufacturer's own mesh and found to be 0.100036, a 2.6% error, and
anything previously calibrated against the old figure inherited it. The
parameter is deliberately exposed and deliberately not hidden behind a
default that looks authoritative.
"""
import math

import numpy as np
import rclpy
from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')


class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        # Geometry. Defaults match ranger_wheels.xacro; override to test
        # what a wrong radius does to the estimate, which is the whole
        # point of it being a parameter.
        self.declare_parameter('wheel_radius', 0.100036)
        self.declare_parameter('half_wheelbase', 0.2470)
        self.declare_parameter('half_track', 0.1852)

        self.declare_parameter('odom_frame', 'odom')
        # base_footprint, matching ranger_xarm_bringup's EKF config. The
        # two frames differ only in z, so planar dead reckoning is the
        # same either way, but the estimator localises base_footprint and
        # a comparison is easier to trust when both sides name the same
        # thing.
        self.declare_parameter('child_frame', 'base_footprint')
        # Turn this OFF whenever the EKF is running. The EKF is
        # configured with publish_tf: true and owns odom -> base_footprint;
        # two publishers on one edge is not an error anyone reports, it
        # just makes TF return whichever arrived last.
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_topic', 'odom')

        self.r = self.get_parameter('wheel_radius').value
        lx = self.get_parameter('half_wheelbase').value
        ly = self.get_parameter('half_track').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.pos = {
            'front_left': (lx, ly),
            'front_right': (lx, -ly),
            'rear_left': (-lx, ly),
            'rear_right': (-lx, -ly),
        }

        # The geometry half of the least-squares system never changes, so
        # it is built once. Rows alternate x, y per corner.
        rows = []
        for c in CORNERS:
            xi, yi = self.pos[c]
            rows.append([1.0, 0.0, -yi])
            rows.append([0.0, 1.0, xi])
        self.A = np.array(rows)
        self.A_pinv = np.linalg.pinv(self.A)

        self.x = self.y = self.yaw = 0.0
        self.last_wheel = {}
        self.last_stamp = None
        self.residual = 0.0

        self.pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self._on_joints, 20)

        self.get_logger().info(
            f'wheel odometry: r={self.r} half_wheelbase={lx} half_track={ly} '
            f'-> {self.odom_frame}/{self.child_frame}')

    @staticmethod
    def _iface(msg, joint, name):
        if joint not in msg.joint_names:
            return None
        iv = msg.interface_values[msg.joint_names.index(joint)]
        if name not in iv.interface_names:
            return None
        return iv.values[iv.interface_names.index(name)]

    def _on_joints(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        steer, travel = {}, {}
        for c in CORNERS:
            a = self._iface(msg, f'{c}_steer_joint', 'position')
            w = self._iface(msg, f'{c}_wheel_joint', 'position')
            if a is None or w is None:
                return
            steer[c] = a
            prev = self.last_wheel.get(c)
            self.last_wheel[c] = w
            if prev is None:
                travel[c] = 0.0
            else:
                d = w - prev
                # Continuous joints wrap; a real encoder does not lose
                # the turn, so neither should this.
                while d > math.pi:
                    d -= 2.0 * math.pi
                while d < -math.pi:
                    d += 2.0 * math.pi
                travel[c] = d

        if self.last_stamp is None:
            self.last_stamp = stamp
            return
        dt = stamp - self.last_stamp
        self.last_stamp = stamp
        if dt <= 0.0:
            return

        # Per-wheel contact displacement, then least squares for the body
        # twist. Working in displacement rather than velocity keeps this
        # an integration of counts.
        b = []
        for c in CORNERS:
            d = travel[c] * self.r
            b.append(d * math.cos(steer[c]))
            b.append(d * math.sin(steer[c]))
        b = np.array(b)
        dx_b, dy_b, dyaw = self.A_pinv @ b
        self.residual = float(np.linalg.norm(self.A @ np.array([dx_b, dy_b, dyaw]) - b))

        # Integrate in the odom frame. Midpoint heading rather than the
        # start heading: over a curved step the start-heading form biases
        # the path to the outside of every turn, and that bias accumulates
        # in one direction instead of averaging out.
        mid = self.yaw + 0.5 * dyaw
        self.x += dx_b * math.cos(mid) - dy_b * math.sin(mid)
        self.y += dx_b * math.sin(mid) + dy_b * math.cos(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))

        self._publish(msg.header.stamp, dx_b / dt, dy_b / dt, dyaw / dt)

    def _publish(self, stamp, vx, vy, wz):
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.child_frame
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = vx
        o.twist.twist.linear.y = vy
        o.twist.twist.angular.z = wz
        # Covariances are placeholders, NOT a characterisation. Dead
        # reckoning error grows with distance travelled and with how much
        # the wheels have been slipping, and neither is captured by a
        # constant. Anything fusing this should set its own, derived from
        # the drift this node is now able to demonstrate.
        for i, v in ((0, 1e-3), (7, 1e-3), (14, 1e6), (21, 1e6), (28, 1e6), (35, 1e-2)):
            o.pose.covariance[i] = v
            o.twist.covariance[i] = v
        self.pub.publish(o)

        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.child_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
