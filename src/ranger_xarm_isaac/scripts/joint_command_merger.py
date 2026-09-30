#!/usr/bin/env python3
"""Merge the per-component joint commands into one message for Isaac.

The arm and the gripper are separate ros2_control hardware components.
Each publishes its own commands, and with no way to give them separate
topics (the vendored xarm ros2_control xacros accept no <param> in their
<hardware> block) they both fall back to the same default. So the topic
carries two different messages in alternation:

    x101  [drive_joint, left_finger, left_inner_knuckle,
           right_outer_knuckle, right_finger, right_inner_knuckle]
    x100  [joint1 .. joint6]

Isaac's ArticulationController applies the joints named in whichever
message it last received. Half the robot is therefore being commanded at
any instant, and the two halves take turns. The arm mostly survives this
because its targets barely change once a trajectory finishes; the
gripper does not, and drive_joint commanded to 0.849 settles around
0.008.

This node keeps the latest command for every joint it has ever seen and
republishes the union, so Isaac receives the whole robot every time.
Merging by name also makes it irrelevant how many components there are
or which of them published last.

The messages are also RAGGED, which is the other half of the problem:

    names=6 positions=6 velocities=0   <- arm
    names=6 positions=1 velocities=0   <- gripper
    names=8 positions=4 velocities=4   <- base

name gets one entry per joint, but position/velocity/effort are packed
INDEPENDENTLY, each holding a value only for the joints that declare
that command interface (see JointStateTopicSystem::write). They are not
parallel arrays with the names, and indexing them by the name's index is
wrong whenever the component is not uniform.

The gripper shows the mild form: five mimic joints deliberately have no
command interface, so they are named with nothing behind them and the
single drive_joint number lands on whichever name comes first.

The base shows the damaging form. Steering takes position, the wheels
take velocity, and they interleave per corner:

    name     = [FL_steer, FL_wheel, FR_steer, FR_wheel, RL_..., RR_...]
    position = [FL_steer, FR_steer, RL_steer, RR_steer]
    velocity = [FL_wheel, FR_wheel, RL_wheel, RR_wheel]

Indexed positionally, the four wheel SPEEDS get applied to the first
four names, so the steer joints are told to spin at the wheel rate and
the rear four joints are never commanded at all. Visibly the robot
creeps, yaws at random, and drives on its front wheels only.

So the arrays are unpacked the way they were packed: each joint's
command interfaces are read from the <ros2_control> blocks in
/robot_description, and the walk over names consumes from an array only
for the joints that declare it.

Since the followers are not commanded by anyone, this node computes them
from the URDF's own <mimic> tags, read off /robot_description rather than
hardcoded, so a change to the gripper linkage does not silently desync
from a table in here.
"""
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class CommandMerger(Node):

    def __init__(self):
        super().__init__('joint_command_merger')
        self.declare_parameter('input_topic', 'robot_joint_commands')
        self.declare_parameter('output_topic', 'isaac_joint_commands')
        self.declare_parameter('publish_rate', 120.0)

        src = self.get_parameter('input_topic').value
        dst = self.get_parameter('output_topic').value
        rate = self.get_parameter('publish_rate').value

        self.position = {}
        self.velocity = {}
        self.mimics = {}      # follower -> (source, multiplier, offset)
        self.cmd_ifaces = {}  # joint -> set of command interface names
        self.warned_unmapped = False

        latching = QoSProfile(depth=1,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description',
                                 self._on_description, latching)

        self.pub = self.create_publisher(JointState, dst, 10)
        self.create_subscription(JointState, src, self._absorb, 10)
        self.create_timer(1.0 / rate, self._emit)
        self.get_logger().info(f'merging {src} -> {dst} at {rate:.0f} Hz')

    def _on_description(self, msg):
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as exc:
            self.get_logger().error(f'could not parse robot_description: {exc}')
            return
        found = {}
        for joint in root.findall('joint'):
            mimic = joint.find('mimic')
            if mimic is None:
                continue
            found[joint.get('name')] = (
                mimic.get('joint'),
                float(mimic.get('multiplier', 1.0)),
                float(mimic.get('offset', 0.0)),
            )
        self.mimics = found
        self.get_logger().info(
            f'{len(found)} mimic joints from robot_description: '
            + ', '.join(sorted(found)))

        # Which command interfaces each joint declares. This is what makes
        # the ragged arrays unpackable; without it the only options are
        # assuming every component is uniform (it is not) or hardcoding a
        # table that silently desyncs from the description.
        ifaces = {}
        for ctrl in root.findall('ros2_control'):
            for joint in ctrl.findall('joint'):
                ifaces[joint.get('name')] = {
                    ci.get('name') for ci in joint.findall('command_interface')}
        self.cmd_ifaces = ifaces
        self.get_logger().info(
            f'command interfaces for {len(ifaces)} joints: '
            + ', '.join(f'{j}[{"+".join(sorted(v)) or "none"}]'
                        for j, v in sorted(ifaces.items())))

    def _absorb(self, msg):
        if not self.cmd_ifaces:
            # Better to drop a command than to apply it to the wrong joint.
            # robot_description is latched and published by the same launch,
            # so this is a startup race of at most a few messages.
            if not self.warned_unmapped:
                self.warned_unmapped = True
                self.get_logger().warn(
                    'command received before robot_description; dropping '
                    'until the command interfaces are known')
            return

        # Walk the names in order, consuming each array only for the joints
        # that declare it, which is exactly how write() filled them.
        pi = vi = 0
        for name in msg.name:
            ifaces = self.cmd_ifaces.get(name)
            if ifaces is None:
                if not self.warned_unmapped:
                    self.warned_unmapped = True
                    self.get_logger().warn(
                        f"'{name}' is not in any ros2_control block; its "
                        'command cannot be placed and is dropped')
                continue
            if 'position' in ifaces and pi < len(msg.position):
                self.position[name] = msg.position[pi]
                pi += 1
            if 'velocity' in ifaces and vi < len(msg.velocity):
                self.velocity[name] = msg.velocity[vi]
                vi += 1

    def _emit(self):
        if not self.position and not self.velocity:
            return
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        merged = dict(self.position)
        # Nobody commands the followers, so derive them. Skipped when the
        # source joint has not been commanded yet, rather than assuming 0.
        for follower, (source, mult, off) in self.mimics.items():
            if source in merged:
                merged[follower] = merged[source] * mult + off

        # The union, not just the position-commanded joints. The wheels take
        # velocity and nothing else, so keying the output off positions drops
        # them from the message entirely and they never spin.
        out.name = list(merged.keys()) + [
            n for n in self.velocity if n not in merged]
        # Both arrays are emitted full length, because JointState is parallel
        # arrays and a short one silently misaligns with the names.
        #
        # A velocity-only joint therefore carries a position of 0.0, which it
        # was never commanded. That is inert rather than wrong only because
        # the wheels are converted to VELOCITY drives at import (stiffness 0,
        # see _make_velocity_drives in urdf_to_usd.py), and a drive with zero
        # stiffness ignores its position target. If a wheel ever kept a
        # position drive it would be fighting to return to angle zero while
        # being told to spin.
        out.position = [merged.get(n, 0.0) for n in out.name]
        out.velocity = [self.velocity.get(n, 0.0) for n in out.name]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CommandMerger()
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
