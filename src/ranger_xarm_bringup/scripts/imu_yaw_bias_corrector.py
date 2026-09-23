#!/usr/bin/env python3
"""Republish the Ouster IMU with gyro-z bias removed and a levelled attitude.

Also estimates roll/pitch by complementary-filtering the gyro against the
gravity vector, so the EKF can compensate terrain bumps. Measured on the tank
drive the base rolls/pitches with sd ~2.4 deg and ~20 deg peak to peak, which
projects to ~400 mm of vertical scatter at 10 m range; two_d_mode discards all
of it and builds every scan as though the robot were level.

The STATIC offset must be removed or this makes things worse. The mean attitude
over that drive is pitch -3.08 deg, roll +1.11 deg -- a mounting/definition
offset relative to base_link, not real average ground slope. Fusing it raw would
tilt the whole map by ~3 deg, against the 0.145 deg tilt the map has today
(measured against the tank water surfaces, which are physically level). So the
offset is calibrated from the same stationary windows used for the gyro bias,
and only deviations from it are published.

Yaw in the published orientation is meaningless (no absolute heading reference)
and is flagged with a huge covariance; the EKF config must not fuse it.

The raw gyro-z carries a bias of about +0.335 deg/s. Measured on three separate
occasions -- +0.337 (2026-09-09), +0.3348 and +0.334 (2026-09-12) -- so it is
stable to ~0.002 deg/s across days, but it is NOT negligible: left in, it
integrates to roughly 90 deg of phantom yaw over a 271 s run. robot_localization
does not estimate gyro bias, so it has to be removed upstream.

The bias is re-estimated whenever the base is stationary (judged from /odom
twist, which is reliable for "am I moving" even though its yaw is not). The
robot stops often enough in normal operation to keep this fresh. Until the first
stationary window completes, the seeded value is used.

Also republishes a realistic angular_velocity_covariance: the driver advertises
6e-4 (rad/s)^2, i.e. sigma = 1.4 deg/s, but the measured standing noise is
0.034 deg/s. Leaving the pessimistic value in would make the EKF under-weight
the one signal we actually trust.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class ImuYawBiasCorrector(Node):
    def __init__(self):
        super().__init__('imu_yaw_bias_corrector')
        self.declare_parameter('imu_in', '/ouster/imu')
        self.declare_parameter('imu_out', '/ouster/imu_corrected')
        self.declare_parameter('odom_in', '/odom')
        # seed: +0.335 deg/s
        self.declare_parameter('initial_bias', 0.005847)
        self.declare_parameter('stationary_linear', 0.01)     # m/s
        self.declare_parameter('stationary_angular', 0.01)    # rad/s
        self.declare_parameter('min_still_sec', 2.0)
        self.declare_parameter('settle_sec', 0.5)             # discard after stopping
        self.declare_parameter('bias_alpha', 0.25)            # EMA on new estimates
        self.declare_parameter('max_bias_step', 0.0035)       # rad/s, ~0.2 deg/s
        self.declare_parameter('yaw_rate_variance', 1.0e-5)   # (rad/s)^2
        self.declare_parameter('publish_attitude', True)
        self.declare_parameter('comp_alpha', 0.98)            # ~0.5 s at 100 Hz
        self.declare_parameter('rp_variance', 4.0e-4)         # (rad)^2, ~1.1 deg
        self.declare_parameter('initial_roll_offset', 0.0)
        self.declare_parameter('initial_pitch_offset', 0.0)

        self.bias = float(self.get_parameter('initial_bias').value)
        self.var = float(self.get_parameter('yaw_rate_variance').value)
        self.alpha = float(self.get_parameter('bias_alpha').value)
        self.max_step = float(self.get_parameter('max_bias_step').value)

        self.still_since = None
        self.samples = []
        self.n_updates = 0
        # complementary-filter attitude state and its static offset
        self.roll = None
        self.pitch = None
        self.t_prev = None
        self.roll_off = float(self.get_parameter('initial_roll_offset').value)
        self.pitch_off = float(self.get_parameter('initial_pitch_offset').value)
        self.rp_samples = []
        self.rp_updates = 0

        best = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                          reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(
            Imu, str(self.get_parameter('imu_out').value), best)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_in').value), self.imu_cb, best)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_in').value), self.odom_cb, best)
        self.get_logger().info(
            f'seeded gyro-z bias {math.degrees(self.bias):+.4f} deg/s; '
            're-estimating whenever the base is stationary')

    def odom_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        still = (abs(msg.twist.twist.linear.x)
                 < float(self.get_parameter('stationary_linear').value)
                 and abs(msg.twist.twist.angular.z)
                 < float(self.get_parameter('stationary_angular').value))
        if still:
            if self.still_since is None:
                self.still_since = t
                self.samples = []
                self.rp_samples = []
        else:
            self._maybe_update()
            self._maybe_update_rp()
            self.still_since = None
            self.samples = []
            self.rp_samples = []

    def _maybe_update_rp(self):
        """Calibrate the static roll/pitch offset from a stationary window."""
        if len(self.rp_samples) < 100:
            return
        a = np.array(self.rp_samples)
        r, p = float(np.median(a[:, 0])), float(np.median(a[:, 1]))
        if self.rp_updates == 0:
            # Snap on the first window. Unlike the gyro bias there is no
            # trustworthy seed for the mounting offset, and an EMA from zero
            # converges far too slowly -- a drive with two or three stationary
            # windows would leave most of the offset in, tilting the whole map.
            self.roll_off, self.pitch_off = r, p
        else:
            self.roll_off += self.alpha * (r - self.roll_off)
            self.pitch_off += self.alpha * (p - self.pitch_off)
        self.rp_updates += 1
        self.get_logger().info(
            f'static attitude offset -> roll {math.degrees(self.roll_off):+.3f} '
            f'pitch {math.degrees(self.pitch_off):+.3f} deg '
            f'(window {math.degrees(r):+.3f}/{math.degrees(p):+.3f}, '
            f'n={len(a)}, update #{self.rp_updates})')

    def _maybe_update(self):
        if self.still_since is None or not self.samples:
            return
        if len(self.samples) < 100:
            return
        new = sum(self.samples) / len(self.samples)
        step = new - self.bias
        if abs(step) > self.max_step:
            self.get_logger().warn(
                f'rejecting gyro bias jump of {math.degrees(step):+.4f} deg/s '
                f'(limit {math.degrees(self.max_step):.4f}); robot may not have '
                'been truly stationary')
            return
        self.bias += self.alpha * step
        self.n_updates += 1
        self.get_logger().info(
            f'gyro-z bias -> {math.degrees(self.bias):+.4f} deg/s '
            f'(window mean {math.degrees(new):+.4f}, n={len(self.samples)}, '
            f'update #{self.n_updates})')

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.still_since is not None:
            if t - self.still_since > float(self.get_parameter('settle_sec').value):
                self.samples.append(msg.angular_velocity.z)
            if (t - self.still_since
                    > float(self.get_parameter('min_still_sec').value) + 8.0):
                self._maybe_update()
                self.still_since = t          # roll the window so it keeps refreshing
                self.samples = []

        # complementary filter: gyro for the fast component, gravity to stop it
        # drifting. A pure accelerometer tilt is corrupted by linear
        # acceleration; a pure gyro integral drifts. Blending gets both.
        ax, ay, az = (msg.linear_acceleration.x, msg.linear_acceleration.y,
                      msg.linear_acceleration.z)
        r_acc = math.atan2(ay, az)
        p_acc = math.atan2(-ax, math.hypot(ay, az))
        if self.roll is None:
            self.roll, self.pitch = r_acc, p_acc
        elif self.t_prev is not None:
            dt = t - self.t_prev
            if 0.0 < dt < 0.5:
                k = float(self.get_parameter('comp_alpha').value)
                self.roll = k * (self.roll + msg.angular_velocity.x * dt) + (1 - k) * r_acc
                self.pitch = k * (self.pitch + msg.angular_velocity.y * dt) + (1 - k) * p_acc
        self.t_prev = t
        if self.still_since is not None and t - self.still_since > 0.5:
            self.rp_samples.append((r_acc, p_acc))

        out = Imu()
        out.header = msg.header
        out.angular_velocity.x = msg.angular_velocity.x
        out.angular_velocity.y = msg.angular_velocity.y
        out.angular_velocity.z = msg.angular_velocity.z - self.bias
        cov = list(msg.angular_velocity_covariance)
        cov[8] = self.var
        out.angular_velocity_covariance = cov
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        if bool(self.get_parameter('publish_attitude').value) and self.roll is not None:
            # roll/pitch relative to the calibrated static offset; yaw is
            # meaningless here, so give it a huge variance and never fuse it.
            r = self.roll - self.roll_off
            p_ = self.pitch - self.pitch_off
            cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
            cp, sp = math.cos(p_ * 0.5), math.sin(p_ * 0.5)
            out.orientation.w = cr * cp
            out.orientation.x = sr * cp
            out.orientation.y = cr * sp
            out.orientation.z = -sr * sp
            v = float(self.get_parameter('rp_variance').value)
            out.orientation_covariance = [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, 1e6]
        else:
            # the Ouster IMU has no absolute heading of its own
            out.orientation_covariance = [-1.0] + [0.0] * 8
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuYawBiasCorrector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # A launch-issued SIGINT raises ExternalShutdownException, not
        # KeyboardInterrupt. Catching only the latter made the node exit 1 on
        # every clean shutdown, which looks like a crash in the launch log and
        # would mask a real failure.
        pass
    finally:
        node.destroy_node()


main()
