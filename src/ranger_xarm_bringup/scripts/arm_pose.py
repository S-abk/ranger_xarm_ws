#!/usr/bin/env python3
"""Move the xArm to a named or given joint pose and exit.

    ros2 run ranger_xarm_bringup arm_pose.py travel
    ros2 run ranger_xarm_bringup arm_pose.py --joints 0 35 0 0 0 0   # degrees

'travel' is the pose for driving: joint 2 tilted back 35 deg. Measured in
Isaac, the arm hides almost nothing of the Ouster's wall-level view in any
of the poses tried (99 % of bearings see the room at 0, +/-30..40 deg), and
the near-field shadows to the sides and rear are the chassis and gantry's,
the same for every pose. What the tilt changes is self-returns: 31 points
within 1.2 m of the lidar at zero, 11 at -35 deg, ~240 at +35 deg. The
gripper stays inside the footprint (x +0.26 m) and clear of the lidar. It goes through the arm's
joint_trajectory_controller, so MoveIt and anything else using that
controller see the same state.
"""
import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

POSES = {                     # degrees, joint1..joint6
    'zero': [0, 0, 0, 0, 0, 0],
    'travel': [0, -35, 0, 0, 0, 0],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pose', nargs='?', default='travel', choices=sorted(POSES))
    ap.add_argument('--joints', type=float, nargs=6, metavar='DEG')
    ap.add_argument('--controller', default='/xarm_xarm6_traj_controller')
    ap.add_argument('--prefix', default='xarm_')
    ap.add_argument('--duration', type=float, default=4.0)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    target = args.joints if args.joints is not None else POSES[args.pose]

    rclpy.init(args=sys.argv)
    node = Node('arm_pose')
    client = ActionClient(node, FollowJointTrajectory,
                          f'{args.controller}/follow_joint_trajectory')
    if not client.wait_for_server(timeout_sec=20.0):
        node.get_logger().error(f'{args.controller} action server not available')
        sys.exit(1)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [f'{args.prefix}joint{i}' for i in range(1, 7)]
    pt = JointTrajectoryPoint()
    pt.positions = [math.radians(d) for d in target]
    sec = int(args.duration)
    pt.time_from_start = Duration(sec=sec, nanosec=int((args.duration - sec) * 1e9))
    goal.trajectory.points = [pt]
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        node.get_logger().error('trajectory rejected')
        sys.exit(1)
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=args.duration + 20.0)
    code = res.result().result.error_code if res.done() else 'timeout'
    node.get_logger().info(f'arm at {target} deg (error code {code})')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
