#!/usr/bin/env python3
"""Republish Isaac's joint states without the effort field.

Isaac's ROS2PublishJointState always fills effort. The topic-based
hardware interface then tries to write it into an effort STATE interface:

    if (!effort.empty() && isfinite(effort[i]))
        set_state(name + "/effort", ...)

and set_state throws when that interface was never declared, which takes
the whole hardware component down mid-read:

    The requested state interface not found: 'xarm_joint1/effort'
    Deactivating following hardware components as their read cycle
    resulted in an error

xarm6.ros2_control.xacro declares position and velocity and leaves
effort commented out, and that file is vendored, so the interface cannot
be added from here without forking someone else's description.

So the field is dropped in transit. Position and velocity are untouched.

Two better fixes, neither of which belongs in this scaffold:

  * Upstream the guard. The hardware interface should check that the
    interface exists before setting it, rather than assuming a publisher
    only sends fields the URDF declares. One conditional in
    joint_state_topic_hardware_interface.cpp.
  * Declare the effort state interface in the arm description. Correct,
    since Isaac genuinely reports effort and it is useful state, but it
    means touching a vendored file, and the real xArm hardware plugin
    shares those same joint blocks and does not report effort.

This node is a hop in the control path. It is cheap, but it is a hop,
and it should go away once either of the above lands.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class EffortFilter(Node):

    def __init__(self):
        super().__init__('joint_state_effort_filter')
        self.declare_parameter('input_topic', 'isaac_joint_states')
        self.declare_parameter('output_topic', 'robot_joint_states')
        src = self.get_parameter('input_topic').value
        dst = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(JointState, dst, 10)
        self.create_subscription(JointState, src, self._relay, 10)
        self.get_logger().info(f'stripping effort: {src} -> {dst}')

    def _relay(self, msg):
        msg.effort = []
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = EffortFilter()
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
