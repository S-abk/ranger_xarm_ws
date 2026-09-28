#!/usr/bin/env python3
"""Turn the Ouster's 3D cloud into what 2D SLAM and Nav2 need on uneven ground.

The platform is meant for indoor and outdoor work on uneven terrain, and
that breaks the usual shortcuts. A fixed height band sees the ground as a
wall as soon as the robot pitches: from a lidar 1.5 m up, a 6 deg pitch
(measured on the outdoor terrain) puts a horizontal slice on the ground
14 m out, and a 5 deg slope rises 44 cm within 5 m. So this node decides
what is an obstacle by height above the LOCAL ground instead:

0. Each point is deskewed: a spinning lidar takes a sweep to collect its
   360 deg, and at 0.6 rad/s the sweep's two ends disagree by 3.4 deg, which
   put walls in slam_toolbox's map twice. Points are moved into the base
   frame at the stamp time using the gyro's yaw rate and the odometry's
   velocity. Point times come from the Ouster driver's per-point 't' (ns
   after the stamp) when the cloud has it, and otherwise from the firing
   order: Isaac emits points in sweep order and stamps the END of the sweep
   (measured: aligning spinning scans with a stationary one, 98 % of points
   within 7 cm deskewed that way, 81 % not deskewed, 49 % if the stamp were
   the start).
1. Points inside a box around the robot's own body, mast and arm go.
2. The cloud is levelled with roll and pitch from a complementary filter
   on the Ouster IMU (gyro integrated, accelerometer as the long-term
   reference, gated while accelerating). The real OS0 IMU is 6-axis and
   Isaac's orientation field is identity, so neither can supply attitude.
   TF stays 2D (base_footprint is level in TF), and a cloud levelled here
   and published in base_footprint is consistent with it.
3. The levelled points are binned in a grid; a point more than
   min_obstacle_height above its cell's lowest point is an obstacle, up to
   max_obstacle_height. Slopes and the 5 cm stones stay traversable (a
   0.25 m cell on an 11 deg slope varies 5 cm), a wall or boulder does not,
   and the bottom min_obstacle_height of every obstacle goes unmarked.
   Negative obstacles (holes, drops) are not detected.

Outputs, both in base_footprint:
- obstacles_out: the obstacle points (PointCloud2), for Nav2's costmaps;
- scan_out: the obstacles projected to a 360 deg LaserScan, for
  slam_toolbox, nearest per angle bin as seen from base_footprint.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class LidarTerrainFilter(Node):
    def __init__(self):
        super().__init__('lidar_terrain_filter')
        p = self.declare_parameter
        p('cloud_in', '/ouster/points')
        p('imu_in', '/ouster/imu')
        p('odom_in', '/odometry/filtered')  # linear velocity, for deskewing
        p('deskew', True)
        p('scan_period', 0.1)               # s per sweep, when there is no 't' field
        p('base_frame', 'base_footprint')
        p('obstacles_out', '/lidar/obstacles')
        p('scan_out', '/lidar/scan')
        p('max_range', 25.0)                # m, horizontal
        p('cell', 0.25)                     # m, ground grid
        p('min_obstacle_height', 0.15)      # m above the cell's lowest point
        p('max_obstacle_height', 2.0)
        # The robot's own returns (gantry top, mast, parked arm), in
        # base_footprint; measured in Isaac within x -0.38..0.09,
        # y 0..0.09, z 1.17..1.51. A raised arm can leave the box.
        p('body_min', [-0.55, -0.36, -0.30])
        p('body_max', [0.45, 0.36, 1.90])
        p('scan_angle_increment', math.radians(0.5))
        p('scan_range_min', 0.30)
        p('tilt_time_constant', 1.0)        # s, accelerometer weight
        p('accel_gate', 1.0)                # m/s^2 off 1 g: skip accel correction
        g = lambda n: self.get_parameter(n).value
        self.cfg = {n: g(n) for n in (
            'base_frame', 'max_range', 'cell', 'min_obstacle_height',
            'max_obstacle_height', 'scan_angle_increment', 'scan_range_min',
            'tilt_time_constant', 'accel_gate', 'deskew', 'scan_period')}
        self.body_min = np.array(g('body_min'))
        self.body_max = np.array(g('body_max'))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.extrinsics = {}                # frame -> (R, t) into base_frame
        self.roll = self.pitch = 0.0
        self.tilt_ready = False
        self.last_imu_t = None
        self.yaw_rate = 0.0
        self.vel = (0.0, 0.0)
        self.stats = [0, 0.0, time.monotonic()]

        self.obs_pub = self.create_publisher(PointCloud2, g('obstacles_out'), 5)
        self.scan_pub = self.create_publisher(LaserScan, g('scan_out'), 5)
        self.create_subscription(Imu, g('imu_in'), self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_in'), self._on_odom, 10)
        self.create_subscription(PointCloud2, g('cloud_in'), self._on_cloud,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f"{g('cloud_in')} -> {g('obstacles_out')}, {g('scan_out')} "
            f"(obstacle > {self.cfg['min_obstacle_height']:g} m above local ground)")

    def _extrinsic(self, frame):
        if frame not in self.extrinsics:
            try:
                t = self.tf_buffer.lookup_transform(self.cfg['base_frame'], frame,
                                                    rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                return None
            tr = t.transform.translation
            self.extrinsics[frame] = (quat_to_matrix(t.transform.rotation),
                                      np.array([tr.x, tr.y, tr.z]))
        return self.extrinsics[frame]

    def _on_odom(self, msg):
        self.vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    # -- attitude --------------------------------------------------------

    def _on_imu(self, msg):
        ext = self._extrinsic(msg.header.frame_id)
        if ext is None:
            return
        R = ext[0]
        w = R @ np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                          msg.angular_velocity.z])
        self.yaw_rate = float(w[2])
        a = R @ np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        roll_a = math.atan2(a[1], a[2])
        pitch_a = math.atan2(-a[0], math.hypot(a[1], a[2]))
        if not self.tilt_ready:
            self.roll, self.pitch, self.tilt_ready = roll_a, pitch_a, True
            self.last_imu_t = t
            return
        dt = t - self.last_imu_t
        self.last_imu_t = t
        if not 0.0 < dt < 0.5:
            return
        # Body rates to Euler rates (ZYX), so tilt survives yawing.
        sr, cr = math.sin(self.roll), math.cos(self.roll)
        cp, tp = math.cos(self.pitch), math.tan(self.pitch)
        self.roll += (w[0] + sr * tp * w[1] + cr * tp * w[2]) * dt
        self.pitch += (cr * w[1] - sr * w[2]) * dt
        if abs(np.linalg.norm(a) - 9.80665) < self.cfg['accel_gate'] and cp > 0.5:
            k = dt / (self.cfg['tilt_time_constant'] + dt)
            self.roll += k * (roll_a - self.roll)
            self.pitch += k * (pitch_a - self.pitch)

    def _deskew(self, pts, raw, has_t):
        """Move every point into the base frame at the stamp time (2D)."""
        if has_t:
            dt = raw[:, 3] * 1e-9                     # after the stamp
        else:
            az = np.unwrap(np.arctan2(raw[:, 1], raw[:, 0]))
            span = az[-1] - az[0]
            if abs(span) < 1e-6:
                return pts
            dt = ((az - az[0]) / span - 1.0) * self.cfg['scan_period']   # before the stamp
        phi = self.yaw_rate * dt
        c, s = np.cos(phi), np.sin(phi)
        x = c * pts[:, 0] - s * pts[:, 1] + self.vel[0] * dt
        y = s * pts[:, 0] + c * pts[:, 1] + self.vel[1] * dt
        return np.column_stack([x, y, pts[:, 2]])

    def _level(self):
        sr, cr = math.sin(self.roll), math.cos(self.roll)
        sp, cp = math.sin(self.pitch), math.cos(self.pitch)
        # R_y(pitch) @ R_x(roll): base axes expressed in the level frame.
        return np.array([[cp, sp * sr, sp * cr],
                         [0.0, cr, -sr],
                         [-sp, cp * sr, cp * cr]])

    # -- cloud -----------------------------------------------------------

    def _on_cloud(self, msg):
        if not self.tilt_ready:
            return
        ext = self._extrinsic(msg.header.frame_id)
        if ext is None:
            return
        t0 = time.monotonic()
        names = [f.name for f in msg.fields]
        has_t = 't' in names
        raw = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z', 't') if has_t else ('x', 'y', 'z'),
            skip_nans=True).astype(np.float64)
        pts = raw[:, :3]
        R, t = ext
        pts = pts @ R.T + t
        if self.cfg['deskew'] and len(pts) > 1:
            pts = self._deskew(pts, raw, has_t)
        inside = np.all((pts > self.body_min) & (pts < self.body_max), axis=1)
        pts = pts[~inside] @ self._level().T
        c = self.cfg
        r = np.hypot(pts[:, 0], pts[:, 1])
        keep = (r < c['max_range']) & (r > 1e-3)
        pts, r = pts[keep], r[keep]

        n = int(math.ceil(2 * c['max_range'] / c['cell']))
        ix = ((pts[:, 0] + c['max_range']) / c['cell']).astype(np.int64)
        iy = ((pts[:, 1] + c['max_range']) / c['cell']).astype(np.int64)
        cell = np.clip(ix, 0, n - 1) * n + np.clip(iy, 0, n - 1)
        floor = np.full(n * n, np.inf)
        np.minimum.at(floor, cell, pts[:, 2])
        height = pts[:, 2] - floor[cell]
        obst = (height > c['min_obstacle_height']) & (height < c['max_obstacle_height'])
        o, ro = pts[obst], r[obst]

        header = msg.header
        header.frame_id = c['base_frame']
        self.obs_pub.publish(point_cloud2.create_cloud_xyz32(header, o.astype(np.float32)))

        inc = c['scan_angle_increment']
        bins = int(round(2 * math.pi / inc))
        ranges = np.full(bins, np.inf, dtype=np.float32)
        if len(o):
            b = ((np.arctan2(o[:, 1], o[:, 0]) + math.pi) / inc).astype(np.int64) % bins
            np.minimum.at(ranges, b, ro.astype(np.float32))
        scan = LaserScan()
        scan.header = header
        scan.angle_min = -math.pi
        scan.angle_max = -math.pi + (bins - 1) * inc
        scan.angle_increment = inc
        scan.scan_time = 0.1
        scan.range_min = c['scan_range_min']
        scan.range_max = c['max_range']
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)

        st = self.stats
        st[0] += 1
        st[1] += time.monotonic() - t0
        if time.monotonic() - st[2] > 30.0:
            self.get_logger().info(
                f'{st[0]} clouds, {1000 * st[1] / st[0]:.1f} ms each; '
                f'{len(o)} obstacle points of {len(pts)}; tilt roll '
                f'{math.degrees(self.roll):+.1f} pitch {math.degrees(self.pitch):+.1f} deg')
            self.stats = [0, 0.0, time.monotonic()]


def main():
    rclpy.init()
    node = LidarTerrainFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
