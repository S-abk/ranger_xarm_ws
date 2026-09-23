#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class D435Check(Node):
    def __init__(self):
        super().__init__('d435_check')
        self.color = None
        self.depth = None
        self.color_info = None
        self.depth_info = None

        self.create_subscription(
            Image, '/camera/d435/color/image_raw', self._color_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/d435/depth/image_rect_raw', self._depth_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/d435/color/camera_info', self._color_info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/d435/depth/camera_info', self._depth_info_cb,
            qos_profile_sensor_data)

    def _color_cb(self, msg):
        if self.color is None:
            self.color = msg

    def _depth_cb(self, msg):
        if self.depth is None:
            self.depth = msg

    def _color_info_cb(self, msg):
        if self.color_info is None:
            self.color_info = msg

    def _depth_info_cb(self, msg):
        if self.depth_info is None:
            self.depth_info = msg

    @staticmethod
    def _image_desc(msg):
        return (
            f'{msg.width}x{msg.height} encoding={msg.encoding} '
            f'frame={msg.header.frame_id}'
        )


def main():
    rclpy.init()
    node = D435Check()
    node.get_logger().info('Waiting for D435 color/depth images and CameraInfo ...')

    deadline = time.monotonic() + 12.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if all((node.color, node.depth, node.color_info, node.depth_info)):
            break

    missing = []
    if node.color is None:
        missing.append('color image')
    if node.depth is None:
        missing.append('depth image')
    if node.color_info is None:
        missing.append('color camera_info')
    if node.depth_info is None:
        missing.append('depth camera_info')

    if missing:
        node.get_logger().error('Timed out waiting for: ' + ', '.join(missing))
        node.destroy_node()
        rclpy.shutdown()
        return 2

    node.get_logger().info('color: ' + node._image_desc(node.color))
    node.get_logger().info('depth: ' + node._image_desc(node.depth))
    node.get_logger().info(
        f'color CameraInfo frame={node.color_info.header.frame_id} '
        f'fx={node.color_info.k[0]:.2f} fy={node.color_info.k[4]:.2f}')
    node.get_logger().info(
        f'depth CameraInfo frame={node.depth_info.header.frame_id} '
        f'fx={node.depth_info.k[0]:.2f} fy={node.depth_info.k[4]:.2f}')

    expected_color = 'd435_color_optical_frame'
    expected_depth = 'd435_depth_optical_frame'
    ok = True
    if node.color.header.frame_id != expected_color:
        node.get_logger().warn(
            f'Expected color frame {expected_color}, got {node.color.header.frame_id}')
        ok = False
    if node.depth.header.frame_id != expected_depth:
        node.get_logger().warn(
            f'Expected depth frame {expected_depth}, got {node.depth.header.frame_id}')
        ok = False

    if ok:
        node.get_logger().info('D435 stream/frame sanity check PASSED')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
