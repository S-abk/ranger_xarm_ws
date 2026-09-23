#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class RplidarWatchdog(Node):
    """Retry the SLLIDAR driver's start_motor service if /scan goes silent.

    The upstream driver keeps running even if its initial startScan() call fails,
    and exposes /start_motor, which retries motor + scan start.  This node uses
    that recovery path without touching the serial port directly.
    """

    def __init__(self):
        super().__init__('rplidar_watchdog')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('start_service', '/start_motor')
        self.declare_parameter('startup_grace', 4.0)
        self.declare_parameter('scan_timeout', 2.0)
        self.declare_parameter('retry_period', 3.0)
        self.declare_parameter('max_retries', 5)

        scan_topic = self.get_parameter('scan_topic').value
        start_service = self.get_parameter('start_service').value
        self.startup_grace = float(self.get_parameter('startup_grace').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.retry_period = float(self.get_parameter('retry_period').value)
        self.max_retries = int(self.get_parameter('max_retries').value)

        self.started = time.monotonic()
        self.last_scan = None
        self.last_retry = 0.0
        self.retries = 0
        self.pending = None
        self.last_service_warn = 0.0

        self.create_subscription(LaserScan, scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.start_client = self.create_client(Empty, start_service)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'Watching {scan_topic}; will retry {start_service} if scans do not start or go stale.')

    def _scan_cb(self, _msg):
        was_missing = self.last_scan is None
        self.last_scan = time.monotonic()
        if was_missing and self.retries:
            self.get_logger().info('RPLIDAR scan stream recovered.')
        self.retries = 0

    def _tick(self):
        now = time.monotonic()
        if now - self.started < self.startup_grace:
            return
        stale = self.last_scan is None or (now - self.last_scan) > self.scan_timeout
        if not stale:
            return
        if self.pending is not None and not self.pending.done():
            return
        if now - self.last_retry < self.retry_period:
            return
        if self.retries >= self.max_retries:
            return
        if not self.start_client.service_is_ready():
            if now - self.last_service_warn > 5.0:
                self.get_logger().warn('RPLIDAR /start_motor service not ready yet.')
                self.last_service_warn = now
            return

        self.retries += 1
        self.last_retry = now
        self.get_logger().warn(
            f'No fresh /scan; retrying RPLIDAR start ({self.retries}/{self.max_retries}).')
        self.pending = self.start_client.call_async(Empty.Request())


def main():
    rclpy.init()
    node = RplidarWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
