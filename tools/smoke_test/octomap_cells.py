"""Occupied cells of MoveIt's obstacle map, read from /get_planning_scene.

move_group sends the octomap in octomap's full format (OcTree::writeData): per node, in
pre-order, a float32 log-odds value and one byte whose bit i says child i exists. A node
without children is a leaf, occupied when its log-odds is above 0 (MoveIt's occupancy
threshold 0.5). Child index bit 0 is x, bit 1 y, bit 2 z; the root cube is centred on the
origin with side resolution * 2**16. The binary format (2 bits per child) is read too.
"""
import math
import struct

import numpy as np
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

COMPONENTS = (PlanningSceneComponents.SCENE_SETTINGS | PlanningSceneComponents.ROBOT_STATE
              | PlanningSceneComponents.OCTOMAP | PlanningSceneComponents.TRANSFORMS)


def _walk(data, res, full):
    out_c, out_s = [], []
    pos = 0
    stack = [(np.zeros(3), res * 2 ** 16)]
    while stack:
        centre, size = stack.pop()
        q = size / 4
        children = []
        if full:
            (value,) = struct.unpack_from('<f', data, pos)
            bits = data[pos + 4]
            pos += 5
            if bits == 0:
                if value > 0.0:
                    out_c.append(centre)
                    out_s.append(size)
                continue
            for i in range(8):
                if bits >> i & 1:
                    off = np.array([q if i & 1 else -q, q if i & 2 else -q, q if i & 4 else -q])
                    children.append((centre + off, size / 2))
        else:
            b = data[pos], data[pos + 1]
            pos += 2
            for i in range(8):
                byte, j = b[i // 4], i % 4
                occ, free = (byte >> (2 * j)) & 1, (byte >> (2 * j + 1)) & 1
                off = np.array([q if i & 1 else -q, q if i & 2 else -q, q if i & 4 else -q])
                if occ and free:
                    children.append((centre + off, size / 2))
                elif occ:
                    out_c.append(centre + off)
                    out_s.append(size / 2)
        stack.extend(reversed(children))
    if pos != len(data):
        raise ValueError(f'octomap: decoded {pos} of {len(data)} bytes')
    return np.array(out_c).reshape(-1, 3), np.array(out_s)


def decode(msg):
    """octomap_msgs/Octomap -> (occupied leaf centres Nx3, sizes N), in the map's own frame."""
    data = bytes(np.asarray(msg.data, dtype=np.int8).view(np.uint8))
    if not data:
        return np.zeros((0, 3)), np.zeros(0)
    if msg.id != 'OcTree':
        raise ValueError(f'unexpected octomap type {msg.id!r}')
    return _walk(data, msg.resolution, full=not msg.binary)


def pose_rt(p):
    """geometry_msgs Pose or Transform -> (R 3x3, t 3)."""
    q = p.orientation if hasattr(p, 'orientation') else p.rotation
    v = p.position if hasattr(p, 'position') else p.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R, np.array([v.x, v.y, v.z])


def apply(rt, pts):
    return pts @ rt[0].T + rt[1]


def inv(rt):
    return rt[0].T, -rt[0].T @ rt[1]


def planar(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]), np.array([x, y, 0.0])


class SceneReader:
    def __init__(self, node):
        self.node = node
        self.cli = node.create_client(GetPlanningScene, '/get_planning_scene')

    def get(self, rclpy, timeout=5.0):
        req = GetPlanningScene.Request()
        req.components.components = COMPONENTS
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result().scene if fut.done() and fut.result() else None


def robot_relative_cells(scene):
    """Occupied cells in base_footprint coordinates as MoveIt holds them, and the map frame.

    The octomap is kept in the planning frame (move_group passes its planning frame as the
    map frame); the scene gives its origin there. With a planar virtual joint the planning
    frame is odom and the robot state carries the base pose, which takes the cells to the
    robot; without one the planning frame is the description's root.
    """
    om = scene.world.octomap
    cells, _ = decode(om.octomap)
    if len(cells) == 0:
        return cells, om.header.frame_id
    cells = apply(pose_rt(om.origin), cells)
    md = scene.robot_state.multi_dof_joint_state
    if md.joint_names:
        cells = apply(inv(pose_rt(md.transforms[0])), cells)
    return cells, om.header.frame_id
