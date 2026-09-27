#!/usr/bin/env python3
"""Relay gz /clock to ROS at a bounded rate, dropping stale ticks.

Replaces the stock ros_gz_bridge clock bridge. gz publishes /clock once
per physics step, which is 1 kHz in these worlds, and every ROS node with
use_sim_time subscribes to it -- 42 of them with the full stack up. The
stock bridge relays every tick, in order, so it has to deliver ~42,000
messages a second. It managed about a quarter of that with a whole core
pegged, and because it never drops a tick, the shortfall accumulates as a
backlog: ROS time fell progressively behind simulation, 2-4x slow on the
rough-ground worlds. Everything stamped with ROS time inherited the lag,
and wheel_odometry.py, which divides true wheel travel by joint-state
stamp intervals, reported the wheels spinning 2-4x faster than they were.
Switching the bridge to the CLOCK QoS profile did not help; the cost is
the fan-out, not reliability.

gz-transport can throttle a subscription at the source, keeping the most
recent tick and discarding the rest. At the default 250 Hz the relay
delivers ~10k messages a second, and the ROS clock always carries a
current sim time instead of a queued one. 250 Hz is chosen to stay above
the 150 Hz controller_manager rate, so consecutive joint-state messages
still get distinct stamps.

Needs the gz-transport Python bindings (python3-gz-transport13), which
the Harmonic install provides; they are not a ROS package.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock as RosClock

try:
    from gz.transport13 import Node as GzNode, SubscribeOptions
    from gz.msgs10.clock_pb2 import Clock as GzClock
except ImportError as exc:
    sys.exit(f'gz-transport Python bindings not found ({exc}); '
             'install python3-gz-transport13')


class ClockRelay(Node):
    def __init__(self):
        super().__init__('gz_clock_relay')
        self.declare_parameter('rate_hz', 250.0)
        self.declare_parameter('gz_topic', '/clock')
        rate = float(self.get_parameter('rate_hz').value)
        topic = self.get_parameter('gz_topic').value

        # Best-effort, keep-last-1: the profile rclpy's TimeSource subscribes
        # with (rclcpp's ClockQoS is the same), so nothing is left unmatched
        # and a slow reader holds at most one stale tick.
        self.pub = self.create_publisher(
            RosClock, '/clock',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.gz = GzNode()
        opts = SubscribeOptions()
        opts.msgs_per_sec = int(rate)
        if not self.gz.subscribe(GzClock, topic, self._on_clock, opts):
            raise RuntimeError(f'could not subscribe to gz topic {topic}')
        self.get_logger().info(
            f'relaying gz {topic} -> /clock at up to {rate:.0f} Hz')

    def _on_clock(self, msg):
        # Called on gz-transport's thread. rclpy publishers are thread-safe,
        # and there is nothing else here to protect.
        out = RosClock()
        out.clock.sec = msg.sim.sec
        out.clock.nanosec = msg.sim.nsec
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ClockRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
