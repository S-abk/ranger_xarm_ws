#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelDeadman(Node):
    def __init__(self):
        super().__init__('ranger_cmd_vel_deadman')

        self.declare_parameter('input_topic', '/cmd_vel_remote')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('timeout_sec', 0.35)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('max_linear_x', 0.25)
        self.declare_parameter('max_linear_y', 0.25)
        self.declare_parameter('max_angular_z', 0.50)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.max_linear_x = abs(float(self.get_parameter('max_linear_x').value))
        self.max_linear_y = abs(float(self.get_parameter('max_linear_y').value))
        self.max_angular_z = abs(float(self.get_parameter('max_angular_z').value))

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.sub = self.create_subscription(Twist, input_topic, self._on_cmd, 10)
        self.latest = Twist()
        self.last_rx_ns = None
        self.was_timed_out = True

        period = 1.0 / max(rate, 1.0)
        self.timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'Deadman relay {input_topic} -> {output_topic}, '
            f'timeout={self.timeout_sec:.2f}s, rate={rate:.1f}Hz, '
            f'limits x={self.max_linear_x:.2f}m/s y={self.max_linear_y:.2f}m/s '
            f'yaw={self.max_angular_z:.2f}'
        )

    @staticmethod
    def _clamp(value, limit):
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))

    def _on_cmd(self, msg):
        cmd = Twist()
        cmd.linear.x = self._clamp(msg.linear.x, self.max_linear_x)
        cmd.linear.y = self._clamp(msg.linear.y, self.max_linear_y)
        cmd.angular.z = self._clamp(msg.angular.z, self.max_angular_z)
        self.latest = cmd
        self.last_rx_ns = self.get_clock().now().nanoseconds
        self.was_timed_out = False

    def _publish(self):
        now_ns = self.get_clock().now().nanoseconds
        stale = self.last_rx_ns is None or (now_ns - self.last_rx_ns) / 1e9 > self.timeout_sec
        if stale:
            self.pub.publish(Twist())
            if not self.was_timed_out:
                self.get_logger().warn('Remote cmd_vel timed out; commanding zero velocity')
                self.was_timed_out = True
        else:
            self.pub.publish(self.latest)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDeadman()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort stop command before shutdown.
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
