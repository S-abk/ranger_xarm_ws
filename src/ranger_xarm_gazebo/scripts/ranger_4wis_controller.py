#!/usr/bin/env python3
"""Turn /cmd_vel into four steering angles and four wheel speeds.

ros2_control ships steering_controllers_library, but every controller in it
(bicycle, tricycle, ackermann) assumes the rear axle is fixed. The Ranger
steers all four corners independently, which is what lets it crab and spin
in place, so none of them can express its kinematics and this node does the
inverse kinematics directly.

Each corner is treated as a point on a rigid body. For a body twist
(vx, vy, wz) the velocity at a wheel sitting at (xi, yi) is

    vxi = vx - wz * yi
    vyi = vy + wz * xi

which is just v + w x r. The wheel is then pointed along that vector and
spun at its magnitude:

    steer_i = atan2(vyi, vxi)
    omega_i = hypot(vxi, vyi) / wheel_radius

That single formula already covers every mode the platform advertises:
all four angles equal gives crab, angles tangent to the centre gives a spin
about the middle, and a front/rear pair gives Ackermann-ish turning. There
is no mode switch here because there does not need to be one.

Two details that are not obvious from the formula:

Steering is limited to +/-90 degrees, so a wheel asked to point backwards is
instead pointed forwards and spun in reverse. Without that flip, reversing
would make all four wheels swing a half turn through 90 degrees, and the
robot would scrub sideways through the transition.

When the commanded twist is ~0 the steer angle is undefined (atan2(0,0)), so
the last angle is held rather than recomputed. Otherwise the wheels would
snap to zero every time the robot stops, which drags the stationary robot
sideways.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray

CORNERS = ('front_left', 'front_right', 'rear_left', 'rear_right')


class Ranger4WIS(Node):

    def __init__(self):
        super().__init__('ranger_4wis_controller')

        self.declare_parameter('wheel_radius', 0.100036)
        self.declare_parameter('half_wheelbase', 0.2470)
        self.declare_parameter('half_track', 0.1852)
        self.declare_parameter('max_wheel_speed', 25.0)
        self.declare_parameter('cmd_timeout', 0.5)
        # Wheel speed loop. The wheels take torque, not velocity, so the
        # speed loop lives here (see the effort comment in ranger_wheels.xacro
        # for why velocity commands cannot be used with bullet-featherstone).
        self.declare_parameter('wheel_kp', 1.5)
        self.declare_parameter('wheel_ki', 4.0)
        self.declare_parameter('max_wheel_effort', 30.0)

        self.r = self.get_parameter('wheel_radius').value
        lx = self.get_parameter('half_wheelbase').value
        ly = self.get_parameter('half_track').value
        self.max_w = self.get_parameter('max_wheel_speed').value
        self.timeout = self.get_parameter('cmd_timeout').value
        self.kp = self.get_parameter('wheel_kp').value
        self.ki = self.get_parameter('wheel_ki').value
        self.max_eff = self.get_parameter('max_wheel_effort').value

        # Must match the corner order used for the controller's joint list.
        self.pos = {
            'front_left': (lx, ly),
            'front_right': (lx, -ly),
            'rear_left': (-lx, ly),
            'rear_right': (-lx, -ly),
        }
        self.last_steer = {c: 0.0 for c in CORNERS}
        self.wheel_vel = {c: 0.0 for c in CORNERS}
        self.integral = {c: 0.0 for c in CORNERS}

        self.steer_pub = self.create_publisher(
            Float64MultiArray, '/ranger_steer_controller/commands', 10)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/ranger_wheel_controller/commands', 10)

        self.twist = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        # Wheel speed feedback comes from /dynamic_joint_states, NOT
        # /joint_states. joint_state_broadcaster only puts the arm and
        # drive_joint on /joint_states here; the eight base joints appear
        # solely on the dynamic topic. Reading the wrong one fails silently:
        # every lookup misses, measured speed stays 0, the PI loop believes
        # the wheels are stalled and runs open loop. The visible symptom is
        # that releasing /cmd_vel leaves the base coasting for tens of
        # metres, because zeroing the command also zeroes an error that was
        # never real, so nothing ever brakes.
        self.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self._on_joints, 10)
        self.got_feedback = False
        self.dt = 0.01
        self.create_timer(self.dt, self._tick)
        # Say so rather than quietly driving open loop.
        self.create_timer(5.0, self._check_feedback)

        self.get_logger().info(
            f'4WIS ready: r={self.r} half_wheelbase={lx} half_track={ly}')

    def _on_cmd(self, msg):
        self.twist = msg
        self.last_cmd_time = self.get_clock().now()

    def _on_joints(self, msg):
        for c in CORNERS:
            name = f'{c}_wheel_joint'
            if name not in msg.joint_names:
                continue
            iv = msg.interface_values[msg.joint_names.index(name)]
            if 'velocity' in iv.interface_names:
                self.wheel_vel[c] = iv.values[iv.interface_names.index('velocity')]
                self.got_feedback = True

    def _check_feedback(self):
        if not self.got_feedback:
            self.get_logger().warn(
                'no wheel velocity on /dynamic_joint_states; the speed loop '
                'is running open loop and the base will not brake')

    def _tick(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        # A dropped publisher must coast to a stop, not keep driving.
        t = Twist() if age > self.timeout else self.twist
        vx, vy, wz = t.linear.x, t.linear.y, t.angular.z

        steers, speeds = [], []
        for c in CORNERS:
            xi, yi = self.pos[c]
            vxi = vx - wz * yi
            vyi = vy + wz * xi
            speed = math.hypot(vxi, vyi)

            if speed < 1e-6:
                angle = self.last_steer[c]
                omega = 0.0
            else:
                angle = math.atan2(vyi, vxi)
                omega = speed / self.r
                # Point forwards and spin backwards rather than steering past
                # +/-90 degrees.
                if angle > math.pi / 2.0:
                    angle -= math.pi
                    omega = -omega
                elif angle < -math.pi / 2.0:
                    angle += math.pi
                    omega = -omega
                self.last_steer[c] = angle

            omega = max(-self.max_w, min(self.max_w, omega))
            steers.append(angle)

            # PI on wheel speed -> torque. The integral is what holds a
            # steady cruise once the proportional error has shrunk, and it is
            # clamped so a stalled wheel (against a wall, say) cannot wind up
            # and then lurch when it comes free.
            err = omega - self.wheel_vel[c]
            self.integral[c] = max(-self.max_eff,
                                   min(self.max_eff,
                                       self.integral[c] + self.ki * err * self.dt))
            if abs(omega) < 1e-6 and abs(self.wheel_vel[c]) < 1e-3:
                self.integral[c] = 0.0
            effort = self.kp * err + self.integral[c]
            speeds.append(max(-self.max_eff, min(self.max_eff, effort)))

        self.steer_pub.publish(Float64MultiArray(data=steers))
        self.wheel_pub.publish(Float64MultiArray(data=speeds))


def main():
    rclpy.init()
    node = Ranger4WIS()
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
