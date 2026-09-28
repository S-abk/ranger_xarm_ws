#!/usr/bin/env python3
"""Re-anchor KISS-ICP's pose into the EKF's odom frame, and publish it only
while it can be believed.

**Pose, not velocity.** KISS-ICP's pose is accurate because each scan is
registered against a local map: the registration error of one scan does not
carry into the next, so over a drive the errors cancel. Its per-scan
velocity has none of that -- measured in simulation, about three times the
wheels' velocity noise -- and fusing velocity in the EKF threw the
cancellation away (the fused estimate was worse than the wheels alone).
Fused as an absolute position, KISS-ICP's drift is what the EKF inherits.

**Anchored to the EKF.** KISS-ICP's frame starts where the robot started and
is not the EKF's odom frame. On the first usable message, and again after
every interruption, the relay records the transform that places KISS-ICP's
current pose exactly on the EKF's pose at the same instant, and publishes
KISS-ICP's poses through it. At an anchor the innovation is zero, so the
estimate never jumps; from then on the EKF follows KISS-ICP's displacement.
An error KISS-ICP made while it was not believed is therefore never
imported: re-anchoring discards it.

**Believed only when the scene constrains it.** Scan matching can only
observe motion that the scene's geometry pins down. On a bare floor there is
nothing to constrain horizontal translation or yaw, and KISS-ICP, measured
in simulation, missed an entire 137 deg turn there; its own covariance is a
constant, so it cannot say when it is guessing. This node decides from the
cloud itself. Non-ground points are grouped in a coarse voxel grid, each
sufficiently populated voxel gets a surface normal by PCA, and the
horizontal parts of those normals are summed into a 2x2 information matrix.
Its smaller eigenvalue is how well the weakest horizontal direction is
constrained, in units of roughly one voxel's worth of surface facing that
way. A bare floor gives nothing; a single straight wall constrains only
across itself, however much of it is visible; a room, or a row of round
pillars, constrains both directions. Below the threshold, and for a hold-off
after recovering, nothing is published and the EKF runs on wheels and gyro.

**And only while it agrees with the gyro.** A registration can also fail in
a well-structured scene, and in simulation every such failure began as a
heading jump (16 - 48 deg in one step). Each step of KISS-ICP's pose is
compared with the EKF's over the same interval; between scans the EKF's
heading comes from the gyro, so a heading step that disagrees by more than
max_yaw_mismatch ends the anchor, and the next consistent step starts a new
one. The translation check is deliberately only for gross jumps: KISS-ICP's
transient position errors grow a few cm per scan and then snap back in one
step, so a tight tolerance fused the growth, rejected the snap-back, and
re-anchoring locked the error in -- measured, it made the fused estimate
worse than KISS-ICP alone.
"""
import bisect
import collections
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32
import tf2_ros


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def relative(a, b):
    """Planar pose b expressed in the frame of planar pose a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2]))


class LidarOdometryRelay(Node):
    def __init__(self):
        super().__init__('lidar_odometry_relay')
        p = self.declare_parameter
        p('odom_in', '/kiss/odometry')
        p('odom_out', '/kiss/pose')
        p('cloud', '/ouster/points')
        p('ekf_odom', '/odometry/filtered')  # the EKF this relay feeds
        p('odom_frame', 'odom')
        p('base_frame', 'base_footprint')
        p('position_covariance', 1e-4)      # m^2 on the published x and y
        p('yaw_covariance', 1e-4)           # rad^2 on the published yaw
        # deg per KISS-ICP step, against the EKF. Real registration failures
        # jumped 16 - 48 deg in one step; Isaac's gyro alone spikes 2 - 3 deg
        # in 0.1 s on bumps, and each false alarm re-anchors to the EKF's error.
        p('max_yaw_mismatch', 5.0)
        p('max_translation_mismatch', 0.5)  # m per KISS-ICP step, against the EKF
        p('max_age', 0.25)                  # s behind the EKF beyond which a pose is stale
        p('min_range', 0.8)
        p('max_range', 30.0)
        p('min_height', 0.3)                # above the floor and its bumps
        p('max_height', 2.5)
        p('voxel', 0.3)
        p('min_points_per_voxel', 5)
        p('max_normal_z', 0.5)              # 'horizontal' normals only
        p('min_structure', 5.0)             # smaller eigenvalue threshold
        p('holdoff', 1.0)                   # s of distrust after recovering
        p('period', 0.5)                    # s between structure evaluations
        g = lambda n: self.get_parameter(n).value
        self.cfg = {n: g(n) for n in (
            'odom_frame', 'base_frame', 'position_covariance', 'yaw_covariance',
            'max_yaw_mismatch', 'max_translation_mismatch', 'max_age', 'min_range', 'max_range', 'min_height',
            'max_height', 'voxel', 'min_points_per_voxel', 'max_normal_z',
            'min_structure', 'holdoff', 'period')}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cloud_to_base = {}                 # frame_id -> (R, t)
        self.structured = False                 # safe until proven otherwise
        self.ok_since = None
        self.last_eval = 0.0
        self.structure = 0.0
        self.prev = None                        # (t, kiss pose, ekf pose) of the last step
        self.anchor = None                      # (x, y, yaw): KISS-ICP frame -> odom
        # The EKF's recent poses, to read its estimate at each scan's time
        # without blocking: (t, x, y, yaw), 50 Hz for ~10 s.
        self.ekf_hist = collections.deque(maxlen=512)
        self.pending = collections.deque(maxlen=50)   # KISS-ICP poses ahead of the EKF

        self.pub = self.create_publisher(Odometry, g('odom_out'), 10)
        self.metric_pub = self.create_publisher(Float32, g('odom_out') + '/structure', 10)
        self.create_subscription(Odometry, g('odom_in'), self._on_kiss, 20)
        self.create_subscription(Odometry, g('ekf_odom'), self._on_ekf, 50)
        # Raw: a full Ouster cloud costs real time to deserialise in Python,
        # and only one every `period` is looked at. A relay that falls behind
        # hands the EKF stale poses, which drag the estimate backwards.
        self.create_subscription(PointCloud2, g('cloud'), self._on_cloud,
                                 qos_profile_sensor_data, raw=True)
        self.get_logger().info(
            f"{g('odom_in')} -> {g('odom_out')} in {self.cfg['odom_frame']}, gated by scene structure "
            f"(min {self.cfg['min_structure']:g}, hold-off {self.cfg['holdoff']:g} s)")

    # -- structure ---------------------------------------------------------

    def _transform(self, frame):
        if frame not in self.cloud_to_base:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.cfg['base_frame'], frame, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                return None
            tr = t.transform.translation
            self.cloud_to_base[frame] = (quat_to_matrix(t.transform.rotation),
                                         np.array([tr.x, tr.y, tr.z]))
        return self.cloud_to_base[frame]

    def _measure(self, msg):
        tf = self._transform(msg.header.frame_id)
        if tf is None:
            return None
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                             skip_nans=True).astype(np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)]
        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r > self.cfg['min_range']) & (r < self.cfg['max_range'])]
        R, t = tf
        pts = pts @ R.T + t
        pts = pts[(pts[:, 2] > self.cfg['min_height']) & (pts[:, 2] < self.cfg['max_height'])]
        if len(pts) < self.cfg['min_points_per_voxel']:
            return 0.0

        keys = np.floor(pts / self.cfg['voxel']).astype(np.int64)
        _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        inv = inv.ravel()
        k = len(counts)
        s1 = np.zeros((k, 3))
        s2 = np.zeros((k, 3, 3))
        np.add.at(s1, inv, pts)
        np.add.at(s2, inv, pts[:, :, None] * pts[:, None, :])
        keep = counts >= self.cfg['min_points_per_voxel']
        if not keep.any():
            return 0.0
        n = counts[keep][:, None]
        mu = s1[keep] / n
        cov = s2[keep] / n[:, :, None] - mu[:, :, None] * mu[:, None, :]
        _, vecs = np.linalg.eigh(cov)
        normals = vecs[:, :, 0]                 # smallest-eigenvalue direction
        horiz = np.abs(normals[:, 2]) < self.cfg['max_normal_z']
        if not horiz.any():
            return 0.0
        nh = normals[horiz, :2]
        info = nh.T @ nh                        # 2x2 translational information
        return float(np.linalg.eigvalsh(info)[0])

    def _on_cloud(self, raw):
        now = time.monotonic()
        if now - self.last_eval < self.cfg['period']:
            return
        self.last_eval = now
        m = self._measure(deserialize_message(raw, PointCloud2))
        if m is None:
            return
        self.structure = m
        self.metric_pub.publish(Float32(data=m))
        ok = m >= self.cfg['min_structure']
        if ok and self.ok_since is None:
            self.ok_since = now
        elif not ok:
            self.ok_since = None
        structured = ok and (now - self.ok_since) >= self.cfg['holdoff']
        if structured != self.structured:
            self.get_logger().info(
                f"lidar odometry {'trusted' if structured else 'DISTRUSTED'}: "
                f'structure {m:.1f} (threshold {self.cfg["min_structure"]:g})')
        self.structured = structured

    # -- relay -------------------------------------------------------------

    def _on_ekf(self, msg):
        p = msg.pose.pose
        self.ekf_hist.append((stamp_of(msg), p.position.x, p.position.y, yaw_of(p.orientation)))
        while self.pending and self.pending[0][0] <= self.ekf_hist[-1][0]:
            self._process(*self.pending.popleft())

    def _ekf_pose(self, t):
        """The EKF's pose at time t, interpolated; None if t is outside the history."""
        h = self.ekf_hist
        if not h or t < h[0][0] or t > h[-1][0]:
            return None
        i = bisect.bisect_left(h, (t,))
        if h[i][0] == t or i == 0:
            return h[i][1:]
        a, b = h[i - 1], h[i]
        f = (t - a[0]) / (b[0] - a[0])
        return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]),
                wrap(a[3] + f * wrap(b[3] - a[3])))

    def _drop(self, why):
        if self.anchor is not None:
            self.get_logger().info(f'lidar odometry anchor dropped: {why}')
        self.anchor = None

    def _on_kiss(self, msg):
        t = stamp_of(msg)
        kiss = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                yaw_of(msg.pose.pose.orientation))
        if self.ekf_hist and t <= self.ekf_hist[-1][0]:
            self._process(t, kiss, msg.header.stamp)
        else:
            self.pending.append((t, kiss, msg.header.stamp))   # EKF not there yet

    def _process(self, t, kiss, stamp):
        ekf = self._ekf_pose(t)
        if ekf is None:
            # Says nothing about KISS-ICP: skip this scan, keep the anchor,
            # and check the next step from the last one that had a pose.
            return
        prev, self.prev = self.prev, (t, kiss, ekf)
        if not self.structured:
            self._drop(f'structure {self.structure:.1f}')
            return
        if prev is None or t <= prev[0]:
            return                              # need a step to check first
        # The step, in the frame of its start, from each source.
        dk, de = relative(prev[1], kiss), relative(prev[2], ekf)
        dyaw = math.degrees(abs(wrap(dk[2] - de[2])))
        dpos = math.hypot(dk[0] - de[0], dk[1] - de[1])
        if dyaw > self.cfg['max_yaw_mismatch'] or dpos > self.cfg['max_translation_mismatch']:
            self._drop(f'{t - prev[0]:.2f} s step disagrees with the EKF by {dyaw:.1f} deg, '
                       f'{dpos:.3f} m (lidar {dk[0]:+.3f} {dk[1]:+.3f} m {math.degrees(dk[2]):+.1f} deg, '
                       f'EKF {de[0]:+.3f} {de[1]:+.3f} m {math.degrees(de[2]):+.1f} deg)')
            return
        if self.anchor is None:
            # odom <- kiss such that this KISS-ICP pose lands on the EKF's.
            a = wrap(ekf[2] - kiss[2])
            c, s = math.cos(a), math.sin(a)
            self.anchor = (ekf[0] - (c * kiss[0] - s * kiss[1]),
                           ekf[1] - (s * kiss[0] + c * kiss[1]), a)
            self.get_logger().info(
                f'lidar odometry anchored at ({ekf[0]:.2f}, {ekf[1]:.2f}, '
                f'{math.degrees(ekf[2]):.1f} deg)')
        ax, ay, a = self.anchor
        c, s = math.cos(a), math.sin(a)

        out = Odometry()
        if self.ekf_hist[-1][0] - t > self.cfg['max_age']:
            return                              # stale: the anchor stays valid, the pose does not
        out.header.stamp = stamp
        out.header.frame_id = self.cfg['odom_frame']
        out.child_frame_id = self.cfg['base_frame']
        out.pose.pose.position.x = ax + c * kiss[0] - s * kiss[1]
        out.pose.pose.position.y = ay + s * kiss[0] + c * kiss[1]
        yaw = wrap(kiss[2] + a)
        out.pose.pose.orientation.z = math.sin(0.5 * yaw)
        out.pose.pose.orientation.w = math.cos(0.5 * yaw)
        cp, cy = self.cfg['position_covariance'], self.cfg['yaw_covariance']
        cov = [0.0] * 36
        for i, v in ((0, cp), (7, cp), (14, 1e6), (21, 1e6), (28, 1e6), (35, cy)):
            cov[i] = v
        out.pose.covariance = cov
        tc = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            tc[i] = 1e6                         # twist is not for fusing
        out.twist.covariance = tc
        self.pub.publish(out)


def main():
    rclpy.init()
    node = LidarOdometryRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
