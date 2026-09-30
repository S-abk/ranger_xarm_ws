#!/usr/bin/env python3
"""Publish Isaac's collision ground truth on ROS: /ground_truth/contacts.

isaac_contacts.py, inside Isaac, sends one JSON datagram per physics step
while the robot touches anything that is not the ground (see its
docstring for the format, and for why it cannot publish itself). This
republishes each datagram unchanged as std_msgs/String, so a consumer
json.loads() msg.data; an empty contact list marks the end of a contact.
Each new contact is also logged here, so a collision shows up in the
launch output without anything subscribed.
"""
import json
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ContactRelay(Node):
    def __init__(self):
        super().__init__('isaac_contact_relay')
        port = self.declare_parameter('port', 47811).value
        self.pub = self.create_publisher(String, 'ground_truth/contacts', 50)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port))
        self.sock.setblocking(False)
        self.touching = set()
        self.create_timer(0.005, self._poll)
        self.get_logger().info(f'udp://127.0.0.1:{port} -> ground_truth/contacts')

    def _poll(self):
        while True:
            try:
                data = self.sock.recv(65536)
            except BlockingIOError:
                return
            self.pub.publish(String(data=data.decode()))
            try:
                msg = json.loads(data)
            except ValueError:
                continue
            now = set()
            for c in msg.get('contacts', []):
                key = (c.get('link'), c.get('other'))
                now.add(key)
                if key not in self.touching:
                    self.get_logger().warn(
                        f"t={msg.get('t', 0):.2f}: {key[0]} touches {key[1]} at "
                        f"{c.get('p')} ({c.get('force', 0):.0f} N)")
            self.touching = now


def main():
    rclpy.init()
    node = ContactRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
