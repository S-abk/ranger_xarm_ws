#!/usr/bin/env python3
"""Proxy UFACTORY's prefixed gripper action into MoveIt's expected action name.

The combined robot uses the xArm prefix ``xarm_``.  UFACTORY therefore exposes
its native gripper action as ``/xarm_xarm_gripper/gripper_action`` while the
MoveIt controller configuration naturally resolves ``xarm_gripper`` +
``gripper_action`` to ``/xarm_gripper/gripper_action``.

This node provides that unprefixed proxy action and forwards goals to the native
UFACTORY server.  It also publishes ``xarm_drive_joint`` on ``/joint_states``.
At startup the joint position is an explicit *assumption* supplied by the launch
argument (default 0.0 rad = fully open).  During commanded gripper motion the
state is updated from native action feedback, and on successful completion it is
updated from the native result (or the commanded goal as a last resort).

No gripper motion is issued automatically at startup.
"""

import math

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand


class GripperActionProxy(Node):
    def __init__(self):
        super().__init__('xarm_gripper_action_proxy')

        self.declare_parameter('proxy_action_name', '/xarm_gripper/gripper_action')
        self.declare_parameter('native_action_name', '/xarm_xarm_gripper/gripper_action')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('joint_name', 'xarm_drive_joint')
        self.declare_parameter('initial_position', 0.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('native_server_timeout_sec', 10.0)
        # Keep the published MoveIt state slightly inside the UFACTORY URDF
        # gripper limit. The native action may report values at/just above
        # 0.85 rad near full close, which can make mimic joints fail strict
        # start-state bound checks due to floating-point/driver conversion.
        self.declare_parameter('model_state_min', 0.0)
        self.declare_parameter('model_state_max', 0.849)

        self.proxy_action_name = str(self.get_parameter('proxy_action_name').value)
        self.native_action_name = str(self.get_parameter('native_action_name').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.joint_name = str(self.get_parameter('joint_name').value)
        self.current_position = float(self.get_parameter('initial_position').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.native_server_timeout_sec = float(
            self.get_parameter('native_server_timeout_sec').value
        )
        self.model_state_min = float(self.get_parameter('model_state_min').value)
        self.model_state_max = float(self.get_parameter('model_state_max').value)

        if not math.isfinite(self.current_position):
            raise ValueError('initial_position must be finite')
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be > 0')
        if not self.model_state_min < self.model_state_max:
            raise ValueError('model_state_min must be < model_state_max')
        self.current_position = min(
            max(self.current_position, self.model_state_min), self.model_state_max
        )

        self._joint_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self._native_client = ActionClient(
            self, GripperCommand, self.native_action_name
        )
        self._active_proxy_goal = None
        self._active_native_goal = None
        self._native_feedback_seen = False

        self._proxy_server = ActionServer(
            self,
            GripperCommand,
            self.proxy_action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self._timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._publish_joint_state
        )
        self._publish_joint_state()

        self.get_logger().warning(
            'Initial gripper state is assumed: '
            f'{self.joint_name}={self.current_position:.4f} rad. '
            'No physical motion is commanded at startup.'
        )
        self.get_logger().info(
            f'Proxying {self.proxy_action_name} -> {self.native_action_name}; '
            f'publishing {self.joint_name} on {self.joint_state_topic}'
        )

    def _publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [float(self.current_position)]
        self._joint_pub.publish(msg)

    def _update_position(self, value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return
        if math.isfinite(value):
            # UFACTORY documents the gripper action through ~0.86 rad, while
            # its MoveIt/URDF model closes at 0.85 rad. Clamp only the state
            # reported to MoveIt, leaving the native physical goal untouched.
            self.current_position = min(
                max(value, self.model_state_min), self.model_state_max
            )
            self._publish_joint_state()

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            'Received MoveIt gripper goal: '
            f'position={goal_request.command.position:.4f}, '
            f'max_effort={goal_request.command.max_effort:.4f}'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        # Forward cancellation to the native UFACTORY action if a goal is active.
        # Do not await here: the action server cancel callback is synchronous, and
        # rclpy will process the returned future in the executor.
        if self._active_native_goal is not None:
            try:
                self._active_native_goal.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(
                    f'Unable to forward gripper cancellation to native server: {exc}'
                )
        return CancelResponse.ACCEPT

    def _native_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._native_feedback_seen = True
        self._update_position(feedback.position)

        if self._active_proxy_goal is not None:
            proxy_feedback = GripperCommand.Feedback()
            proxy_feedback.position = self.current_position
            proxy_feedback.effort = feedback.effort
            proxy_feedback.stalled = feedback.stalled
            proxy_feedback.reached_goal = feedback.reached_goal
            try:
                self._active_proxy_goal.publish_feedback(proxy_feedback)
            except Exception as exc:
                self.get_logger().debug(f'Unable to republish gripper feedback: {exc}')

    async def _execute_callback(self, proxy_goal_handle):
        self._active_proxy_goal = proxy_goal_handle
        self._native_feedback_seen = False

        # rclpy action execute callbacks are coroutines managed by the ROS
        # executor, not by an asyncio event loop.  Therefore do not use
        # asyncio.sleep() here.  wait_for_server() is sufficient for discovery.
        if not self._native_client.wait_for_server(
            timeout_sec=self.native_server_timeout_sec
        ):
            self.get_logger().error(
                f'Native gripper action server {self.native_action_name} '
                f'not available after {self.native_server_timeout_sec:.1f} s'
            )
            proxy_goal_handle.abort()
            self._active_proxy_goal = None
            return GripperCommand.Result()

        if proxy_goal_handle.is_cancel_requested:
            proxy_goal_handle.canceled()
            self._active_proxy_goal = None
            return GripperCommand.Result()

        native_goal = GripperCommand.Goal()
        native_goal.command.position = proxy_goal_handle.request.command.position
        native_goal.command.max_effort = proxy_goal_handle.request.command.max_effort

        send_future = self._native_client.send_goal_async(
            native_goal, feedback_callback=self._native_feedback_callback
        )
        native_goal_handle = await send_future
        if not native_goal_handle.accepted:
            self.get_logger().error('Native UFACTORY gripper rejected the goal')
            proxy_goal_handle.abort()
            self._active_proxy_goal = None
            return GripperCommand.Result()

        self._active_native_goal = native_goal_handle
        result_future = native_goal_handle.get_result_async()

        # rclpy Future objects are directly awaitable by the ROS executor.
        # Await the native result instead of polling with asyncio.sleep(), which
        # raises RuntimeError('no running event loop') under rclpy's executor.
        wrapped = await result_future

        if proxy_goal_handle.is_cancel_requested:
            proxy_goal_handle.canceled()
            self._active_native_goal = None
            self._active_proxy_goal = None
            return GripperCommand.Result()
        native_result = wrapped.result
        native_status = wrapped.status

        # UFACTORY's gripper action may return a terminal SUCCEEDED action status
        # while leaving GripperCommand.Result.reached_goal at its default false
        # value.  Mirror the action protocol status, not that optional result flag.
        native_succeeded = native_status == GoalStatus.STATUS_SUCCEEDED
        native_canceled = native_status == GoalStatus.STATUS_CANCELED

        # Feedback, when present, is the best running estimate.  On a successful
        # terminal status, however, the commanded virtual-joint target is the most
        # reliable final MoveIt state because UFACTORY can leave Result.position
        # at a default/non-virtual value.
        if native_succeeded:
            self._update_position(native_goal.command.position)
        elif self._native_feedback_seen:
            try:
                if math.isfinite(float(native_result.position)):
                    self._update_position(native_result.position)
            except (TypeError, ValueError):
                pass

        proxy_result = GripperCommand.Result()
        proxy_result.position = self.current_position
        proxy_result.effort = native_result.effort
        proxy_result.stalled = native_result.stalled
        proxy_result.reached_goal = native_succeeded

        if native_succeeded:
            self.get_logger().info(
                'Native UFACTORY gripper action completed with SUCCEEDED status'
            )
            proxy_goal_handle.succeed()
        elif native_canceled:
            self.get_logger().warning(
                'Native UFACTORY gripper action completed with CANCELED status'
            )
            proxy_goal_handle.canceled()
        else:
            self.get_logger().error(
                f'Native UFACTORY gripper action failed with status={native_status}'
            )
            proxy_goal_handle.abort()

        self._active_native_goal = None
        self._active_proxy_goal = None
        return proxy_result


def main(args=None):
    rclpy.init(args=args)
    node = GripperActionProxy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
