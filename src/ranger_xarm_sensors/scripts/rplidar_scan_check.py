#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanCheck(Node):
    def __init__(self):
        super().__init__('rplidar_scan_check')
        self.declare_parameter('topic', '/scan')
        topic = self.get_parameter('topic').value
        self.subscription = self.create_subscription(LaserScan, topic, self.cb, 10)
        self.get_logger().info(f'Waiting for one LaserScan on {topic} ...')

    def cb(self, msg):
        valid = []
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                valid.append((r, angle))

        if not valid:
            self.get_logger().error('Scan received, but it contains no finite in-range returns.')
            rclpy.shutdown()
            return

        rmin, amin = min(valid, key=lambda x: x[0])

        def sector(center_deg, half_deg=10.0):
            c = math.radians(center_deg)
            h = math.radians(half_deg)
            vals = [r for r, a in valid if abs(math.atan2(math.sin(a-c), math.cos(a-c))) <= h]
            return min(vals) if vals else float('nan')

        self.get_logger().info(
            'frame=%s samples=%d valid=%d angle=[%.1f, %.1f] deg nearest=%.3f m @ %.1f deg'
            % (
                msg.header.frame_id,
                len(msg.ranges),
                len(valid),
                math.degrees(msg.angle_min),
                math.degrees(msg.angle_max),
                rmin,
                math.degrees(amin),
            )
        )
        self.get_logger().info(
            'sector minima: front(0deg)=%.3f m  left(+90deg)=%.3f m  right(-90deg)=%.3f m  rear(180deg)=%.3f m'
            % (sector(0), sector(90), sector(-90), sector(180))
        )
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ScanCheck()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
