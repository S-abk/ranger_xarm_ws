#!/usr/bin/env python3

import math
import time
from copy import deepcopy

import numpy as np
import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Empty, SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_to_rotation_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        raise ValueError('zero-norm quaternion')

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


class TrustedManipulationVolumeFilter(Node):
    """
    Optional robot-local PointCloud2 exclusion filter.

    Disabled:
      /ouster/points -> /ouster/points_moveit unchanged

    Enabled:
      points whose XYZ position falls inside one or more configured
      axis-aligned boxes in `filter_frame` are removed before the cloud
      is sent to MoveIt's PointCloudOctomapUpdater.

    IMPORTANT:
      A trusted volume deliberately hides sensor returns. Any foreign
      obstacle inside an enabled trusted volume can also be hidden from
      MoveIt's OctoMap. Keep the boxes small and robot-owned.
    """

    def __init__(self):
        super().__init__('trusted_manipulation_volume_filter')

        self.declare_parameter('config_file', '')
        config_path = str(self.get_parameter('config_file').value).strip()
        if not config_path:
            raise RuntimeError('Required parameter "config_file" is empty.')

        cfg = self._load_yaml(config_path)

        self.filter_frame = self._required_string(cfg, 'filter_frame')
        self.input_topic = self._required_string(cfg, 'input_topic')
        self.output_topic = self._required_string(cfg, 'output_topic')

        self.enable_service_name = str(
            cfg.get('enable_service', '/skimmer/trusted_volume_enable')
        )
        self.clear_service_name = str(
            cfg.get('clear_octomap_service', '/clear_octomap')
        )
        self.clear_on_toggle = bool(
            cfg.get('clear_octomap_on_toggle', True)
        )
        self.rebuild_hold_sec = float(
            cfg.get('rebuild_hold_sec', 2.0)
        )
        self.max_filter_rate_hz = float(
            cfg.get('max_filter_rate_hz', 2.0)
        )
        self.tf_timeout_sec = float(
            cfg.get('tf_timeout_sec', 0.5)
        )
        self.marker_publish_period_sec = float(
            cfg.get('marker_publish_period_sec', 1.0)
        )

        self.declare_parameter(
            'enabled_at_startup',
            bool(cfg.get('enabled_at_startup', False)),
        )
        self.startup_enable_requested = bool(
            self.get_parameter('enabled_at_startup').value
        )

        # Always begin as ordinary pass-through perception. If launch-time
        # enabling was requested, advertise NOT READY from the first status
        # publication until the safe enable -> clear -> rebuild sequence
        # completes. This prevents downstream execution logic from accepting
        # the temporary startup map as authoritative.
        self.enabled = False
        self.ready = not self.startup_enable_requested

        self.zones = self._load_zones(cfg.get('zones', []))

        if self.rebuild_hold_sec < 0.0:
            raise ValueError('rebuild_hold_sec must be >= 0')
        if self.max_filter_rate_hz <= 0.0:
            raise ValueError('max_filter_rate_hz must be > 0')
        if self.tf_timeout_sec <= 0.0:
            raise ValueError('tf_timeout_sec must be > 0')
        if self.startup_enable_requested and not self.zones:
            raise RuntimeError(
                'enabled_at_startup=true but no trusted zones are configured.'
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._cloud_cb,
            qos_profile_sensor_data,
        )

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.enabled_pub = self.create_publisher(
            Bool,
            '/skimmer/trusted_volume/enabled',
            status_qos,
        )
        self.ready_pub = self.create_publisher(
            Bool,
            '/skimmer/trusted_volume/ready',
            status_qos,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/skimmer/trusted_volume/markers',
            status_qos,
        )

        self.clear_client = self.create_client(
            Empty,
            self.clear_service_name,
        )
        self.enable_srv = self.create_service(
            SetBool,
            self.enable_service_name,
            self._enable_cb,
        )

        self._rebuild_deadline = None
        self._last_filtered_time = -1.0
        self._last_stats_log = 0.0
        self._last_marker_publish = time.monotonic()

        # Launch-time enable is intentionally deferred until /clear_octomap is
        # available, so startup uses the same safe semantics as a service
        # toggle: enable -> clear map -> rebuild hold -> ready.
        self._startup_enable_pending = self.startup_enable_requested

        self.timer = self.create_timer(0.1, self._timer_cb)

        self._publish_status()
        self._publish_markers()

        self.get_logger().info(
            'Trusted manipulation volume filter ready. '
            f'Input={self.input_topic}, output={self.output_topic}, '
            f'frame={self.filter_frame}, zones={len(self.zones)}, '
            f'enabled={self.enabled}.'
        )

        for zone in self.zones:
            self.get_logger().info(
                f'Trusted zone "{zone["name"]}": '
                f'min={zone["min"].tolist()}, '
                f'max={zone["max"].tolist()}'
            )

        if not self.zones:
            self.get_logger().warning(
                'No trusted zones are configured. Clouds pass through, '
                'and enable requests will be refused.'
            )

        if self.startup_enable_requested:
            self.get_logger().warning(
                'Trusted-volume launch-time ENABLE requested. '
                'Starting temporarily in pass-through mode until '
                f'{self.clear_service_name} is available, then the OctoMap '
                'will be cleared/rebuilt before READY=true.'
            )
        else:
            self.get_logger().warning(
                'Trusted-volume filtering starts DISABLED. '
                'All Ouster points pass through unchanged.'
            )

    @staticmethod
    def _load_yaml(path):
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict):
            raise ValueError('Trusted-volume YAML must contain a mapping.')
        return data

    @staticmethod
    def _required_string(cfg, key):
        value = str(cfg.get(key, '')).strip()
        if not value:
            raise ValueError(f'Missing required YAML key "{key}".')
        return value

    @staticmethod
    def _vector3(value, label):
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(f'{label} must be a 3-element list.')
        result = np.asarray([float(v) for v in value], dtype=np.float64)
        if not np.all(np.isfinite(result)):
            raise ValueError(f'{label} contains non-finite values.')
        return result

    def _load_zones(self, raw_zones):
        if raw_zones is None:
            return []
        if not isinstance(raw_zones, list):
            raise ValueError('"zones" must be a list.')

        zones = []
        names = set()

        for index, item in enumerate(raw_zones):
            if not isinstance(item, dict):
                raise ValueError(f'zones[{index}] must be a mapping.')

            name = str(item.get('name', f'zone_{index}')).strip()
            if not name:
                raise ValueError(f'zones[{index}] has an empty name.')
            if name in names:
                raise ValueError(f'Duplicate trusted zone name "{name}".')
            names.add(name)

            minimum = self._vector3(
                item.get('min_xyz_m'),
                f'zones[{index}].min_xyz_m',
            )
            maximum = self._vector3(
                item.get('max_xyz_m'),
                f'zones[{index}].max_xyz_m',
            )

            if np.any(maximum <= minimum):
                raise ValueError(
                    f'Zone "{name}" requires max_xyz_m > min_xyz_m '
                    'on all axes.'
                )

            zones.append({
                'name': name,
                'min': minimum,
                'max': maximum,
            })

        return zones

    def _publish_status(self):
        enabled = Bool()
        enabled.data = bool(self.enabled)
        self.enabled_pub.publish(enabled)

        ready = Bool()
        ready.data = bool(self.ready)
        self.ready_pub.publish(ready)

    def _publish_markers(self):
        array = MarkerArray()

        for marker_id, zone in enumerate(self.zones):
            minimum = zone['min']
            maximum = zone['max']
            center = 0.5 * (minimum + maximum)
            size = maximum - minimum

            marker = Marker()
            marker.header.frame_id = self.filter_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'trusted_manipulation_volume'
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = float(size[0])
            marker.scale.y = float(size[1])
            marker.scale.z = float(size[2])

            if self.enabled:
                marker.color.r = 1.0
                marker.color.g = 0.55
                marker.color.b = 0.0
                marker.color.a = 0.35
            else:
                marker.color.r = 0.55
                marker.color.g = 0.55
                marker.color.b = 0.55
                marker.color.a = 0.25

            array.markers.append(marker)

        self.marker_pub.publish(array)

    @staticmethod
    def _field_offset(msg, name):
        for field in msg.fields:
            if field.name == name:
                return int(field.offset)
        raise ValueError(
            f'PointCloud2 is missing required "{name}" field.'
        )

    def _xyz_arrays(self, msg):
        x_offset = self._field_offset(msg, 'x')
        y_offset = self._field_offset(msg, 'y')
        z_offset = self._field_offset(msg, 'z')

        byte_order = '>' if msg.is_bigendian else '<'
        dtype = np.dtype(byte_order + 'f4')

        shape = (int(msg.height), int(msg.width))
        strides = (int(msg.row_step), int(msg.point_step))

        x = np.ndarray(
            shape=shape,
            dtype=dtype,
            buffer=msg.data,
            offset=x_offset,
            strides=strides,
        )
        y = np.ndarray(
            shape=shape,
            dtype=dtype,
            buffer=msg.data,
            offset=y_offset,
            strides=strides,
        )
        z = np.ndarray(
            shape=shape,
            dtype=dtype,
            buffer=msg.data,
            offset=z_offset,
            strides=strides,
        )

        return (
            np.asarray(x, dtype=np.float64).reshape(-1),
            np.asarray(y, dtype=np.float64).reshape(-1),
            np.asarray(z, dtype=np.float64).reshape(-1),
        )

    @staticmethod
    def _raw_point_rows(msg):
        rows = []
        packed_row_bytes = int(msg.width) * int(msg.point_step)
        raw = memoryview(msg.data)

        for row_index in range(int(msg.height)):
            start = row_index * int(msg.row_step)
            stop = start + packed_row_bytes
            row = np.frombuffer(
                raw[start:stop],
                dtype=np.uint8,
            ).reshape(int(msg.width), int(msg.point_step))
            rows.append(row)

        if not rows:
            return np.empty((0, int(msg.point_step)), dtype=np.uint8)
        if len(rows) == 1:
            return rows[0]
        return np.concatenate(rows, axis=0)

    def _lookup_sensor_to_filter(self, sensor_frame):
        transform = self.tf_buffer.lookup_transform(
            self.filter_frame,
            sensor_frame,
            Time(),
            timeout=Duration(seconds=self.tf_timeout_sec),
        )

        t = transform.transform.translation
        q = transform.transform.rotation

        rotation = quaternion_to_rotation_matrix(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )
        translation = np.array(
            [float(t.x), float(t.y), float(t.z)],
            dtype=np.float64,
        )
        return rotation, translation

    def _filter_cloud(self, msg):
        if not msg.header.frame_id:
            raise ValueError('PointCloud2 header.frame_id is empty.')

        x, y, z = self._xyz_arrays(msg)
        sensor_points = np.column_stack((x, y, z))
        finite = np.all(np.isfinite(sensor_points), axis=1)

        rotation, translation = self._lookup_sensor_to_filter(
            msg.header.frame_id
        )

        filter_points = np.empty_like(sensor_points)
        filter_points[:] = np.nan
        filter_points[finite] = (
            sensor_points[finite] @ rotation.T + translation
        )

        remove = np.zeros(sensor_points.shape[0], dtype=bool)

        for zone in self.zones:
            minimum = zone['min']
            maximum = zone['max']
            inside = finite & np.all(
                (filter_points >= minimum)
                & (filter_points <= maximum),
                axis=1,
            )
            remove |= inside

        keep = ~remove
        records = self._raw_point_rows(msg)

        if records.shape[0] != keep.shape[0]:
            raise ValueError(
                'PointCloud2 record count does not match XYZ count.'
            )

        kept_records = records[keep]

        output = PointCloud2()
        output.header = deepcopy(msg.header)
        output.height = 1
        output.width = int(kept_records.shape[0])
        output.fields = deepcopy(msg.fields)
        output.is_bigendian = bool(msg.is_bigendian)
        output.point_step = int(msg.point_step)
        output.row_step = int(output.point_step) * int(output.width)
        output.data = kept_records.tobytes()
        output.is_dense = bool(msg.is_dense)

        return output, int(np.count_nonzero(remove))

    def _cloud_cb(self, msg):
        if not self.enabled:
            self.cloud_pub.publish(msg)
            return

        now = time.monotonic()
        min_period = 1.0 / self.max_filter_rate_hz

        if (
            self._last_filtered_time >= 0.0
            and now - self._last_filtered_time < min_period
        ):
            return

        self._last_filtered_time = now

        try:
            filtered, removed_count = self._filter_cloud(msg)
        except (ValueError, TransformException) as exc:
            self.get_logger().error(
                f'Trusted-volume cloud filtering failed: {exc}'
            )
            return
        except Exception as exc:
            self.get_logger().error(
                f'Unexpected trusted-volume filtering error: {exc}'
            )
            return

        self.cloud_pub.publish(filtered)

        if now - self._last_stats_log >= 5.0:
            self._last_stats_log = now
            self.get_logger().info(
                f'Trusted-volume filter removed {removed_count} '
                f'of {msg.width * msg.height} points from latest cloud.'
            )

    def _begin_mode_change(self, requested):
        requested = bool(requested)

        if requested and not self.zones:
            return False, (
                'ENABLE REFUSED: no trusted manipulation zones '
                'are configured.'
            )

        if requested == self.enabled and self.ready:
            return True, (
                f'Trusted-volume filtering is already '
                f'{"enabled" if self.enabled else "disabled"}.'
            )

        if (
            self.clear_on_toggle
            and not self.clear_client.service_is_ready()
        ):
            return False, (
                f'MODE CHANGE REFUSED: {self.clear_service_name} '
                'is not ready, so the OctoMap cannot be rebuilt.'
            )

        self.enabled = requested
        self.ready = not self.clear_on_toggle
        self._rebuild_deadline = None
        self._last_filtered_time = -1.0

        self._publish_status()
        self._publish_markers()

        if self.clear_on_toggle:
            future = self.clear_client.call_async(Empty.Request())
            future.add_done_callback(self._clear_done_cb)
            return True, (
                f'Trusted-volume filtering '
                f'{"ENABLED" if requested else "DISABLED"}; '
                'OctoMap clear requested. Wait for '
                '/skimmer/trusted_volume/ready=true before planning.'
            )

        return True, (
            f'Trusted-volume filtering '
            f'{"ENABLED" if requested else "DISABLED"}.'
        )

    def _enable_cb(self, request, response):
        success, message = self._begin_mode_change(request.data)
        response.success = success
        response.message = message

        if success:
            self.get_logger().warning(message)
        else:
            self.get_logger().error(message)

        return response

    def _clear_done_cb(self, future):
        try:
            future.result()
        except Exception as exc:
            self.ready = False
            self._rebuild_deadline = None
            self._publish_status()
            self.get_logger().error(
                f'OctoMap clear failed after mode change: {exc}'
            )
            return

        self.ready = False
        self._rebuild_deadline = (
            time.monotonic() + self.rebuild_hold_sec
        )
        self._publish_status()

        self.get_logger().warning(
            f'OctoMap cleared. Rebuilding for '
            f'{self.rebuild_hold_sec:.1f}s before marking '
            'trusted-volume perception READY.'
        )

    def _timer_cb(self):
        now = time.monotonic()

        # Safe launch-time opt-in. Do not fail just because move_group has not
        # created /clear_octomap yet; wait until the service becomes ready.
        if self._startup_enable_pending:
            if self.clear_on_toggle:
                if self.clear_client.service_is_ready():
                    self._startup_enable_pending = False
                    success, message = self._begin_mode_change(True)
                    if success:
                        self.get_logger().warning(
                            'Launch-time trusted-volume transition: ' + message
                        )
                    else:
                        self.get_logger().error(
                            'Launch-time trusted-volume enable failed: '
                            + message
                        )
            else:
                self._startup_enable_pending = False
                success, message = self._begin_mode_change(True)
                if success:
                    self.get_logger().warning(
                        'Launch-time trusted-volume transition: ' + message
                    )
                else:
                    self.get_logger().error(
                        'Launch-time trusted-volume enable failed: ' + message
                    )

        # MarkerArray displays are often added after this node starts.
        # Republish periodically so late-joining RViz subscribers always
        # receive the trusted-volume visualization.
        if (
            now - self._last_marker_publish
            >= self.marker_publish_period_sec
        ):
            self._last_marker_publish = now
            self._publish_markers()

        if (
            self._rebuild_deadline is not None
            and now >= self._rebuild_deadline
        ):
            self._rebuild_deadline = None
            self.ready = True
            self._publish_status()
            self.get_logger().warning(
                'Trusted-volume perception state is READY.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrustedManipulationVolumeFilter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
