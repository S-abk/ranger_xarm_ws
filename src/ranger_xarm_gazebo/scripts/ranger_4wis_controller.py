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

With steer_modes 'agilex' (the default) a twist is first reduced to the
one mode the real Ranger would execute for it -- parallel, spinning or dual
Ackermann, as AgileX's ranger_ros2 driver chooses -- so the four wheels
always hold one of those geometries, and with no command (a zero twist, or
none for cmd_timeout) the knuckles return to straight ahead, as the real
Ranger's do; every stop therefore costs the next move its knuckle swing.
Small non-zero commands are executed as the driver would execute them, so a
caller that wants the base to stay put should send a zero twist. In 'free'
mode, when every wheel's commanded speed is below steer_deadband the
knuckles instead hold their angles together and the wheels stop. Near the
+/-90 degree fold the knuckle keeps its side (fold_hysteresis_deg) rather
than swinging a half turn for a small change of direction.
"""
import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
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
        # Velocity mode only: scale each wheel's speed by
        # max(0, cos(steer error)) ** steer_gate_power, the error being the
        # knuckle's measured angle against its target. A stiff velocity drive
        # otherwise spins a wheel at full speed while its knuckle is still
        # sweeping, so the wheel pushes the base in the wrong direction: on
        # flat ground that scrub put +2.03 deg of heading error into every
        # transition into the crab. Power 1 is ordinary cosine compensation,
        # which still lets a wheel 45 deg off target drive at 70 %; measured,
        # it cut that crab error only to +0.79 deg. Power 3 cut it to
        # +0.11 deg, but with the common gate it still let the wheels drive
        # at 76 % while the rear knuckles, which swing further into a crab,
        # were 24 deg off: the base turned +0.21 deg per crab on flat
        # ground. Power 10 (40 % at 24 deg) cut that to +0.07, at the cost
        # of reaching speed about 0.1 s later in any steered transition.
        # 0 disables the gate.
        # dynamic_typing so steer_gate_power:=3 works as well as :=3.0;
        # without it rclpy rejects the integer and the node exits.
        self.declare_parameter('steer_gate_power', 10.0,
                               ParameterDescriptor(dynamic_typing=True))
        # common: gate all wheels on the worst knuckle. per_wheel: each on
        # its own, which is what produced the crab-transition slip.
        self.declare_parameter('steer_gate_mode', 'common')
        # Velocity mode only: the most the wheel speeds may change per
        # second, in m/s at the tyre. A velocity drive is stiff, so without
        # a limit it starts and stops the wheels instantly; the base's
        # inertia then skids the tyres at every segment boundary, worst on
        # low grip, and wheel odometry cannot see the skid. A base skids
        # once it brakes harder than mu*g -- 2.45 m/s^2 at mu 0.25, 1.47 on
        # the mu 0.15 patches -- so the default sits below both. The four
        # wheels are limited together, by one common factor, so a ramp
        # keeps the inner/outer speed ratio and stays consistent with one
        # body twist. 0 disables.
        self.declare_parameter('max_wheel_accel', 1.0,
                               ParameterDescriptor(dynamic_typing=True))
        # 'free' mode only. Below this speed at every wheel (m/s) the
        # command's direction is noise: small corrections near a goal point
        # anywhere, and chasing them swung the knuckles back and forth by up
        # to 90 deg while the base stood still. All knuckles then hold and
        # the wheels stop, together, keeping one steering geometry.
        self.declare_parameter('steer_deadband', 0.03,
                               ParameterDescriptor(dynamic_typing=True))
        # A target crossing the +/-90 deg fold (crab left <-> crab right,
        # say) would swing a knuckle 180 deg for a direction change of a few
        # degrees. Within this many degrees of the fold the knuckle keeps
        # its side and the wheel spins the other way instead.
        self.declare_parameter('fold_hysteresis_deg', 10.0,
                               ParameterDescriptor(dynamic_typing=True))
        # Parking brake (velocity mode): a stop that has lasted brake_delay
        # holds each wheel at the angle it had then, by commanding a speed
        # back towards it (brake_gain per second of angle error, at most
        # brake_max rad/s), as the real hub motors' controllers hold position
        # at zero speed. Parked on a 10 deg ramp the simulated wheels rolled
        # back at ~20 mm/s under a zero speed command whatever the drive
        # gains were; a hold on position does not depend on them.
        # Speed loop (velocity mode): the simulator's wheel drives yield under
        # load (a zero speed command rolled back at ~20 mm/s on a 10 deg ramp;
        # 0.04 m/s uphill did not move the base), so the error between the
        # commanded and measured wheel speed is integrated into the command
        # (speed_ki per second, at most speed_i_max rad/s), as a hub motor's
        # own PI speed loop would. Reset at a stop and on a change of sign.
        # A wheel turning against its command (slipping, or off the ground)
        # does not integrate, and no wheel's integral exceeds the median of
        # the four by more than speed_i_spread: one wheel wound up to +3
        # rad/s alone (0.75 m/s commanded against 0.5) while the base
        # climbed over a crest, and the base went over backwards.
        self.declare_parameter('speed_ki', 4.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('speed_i_max', 3.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('speed_i_spread', 0.5, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('parking_brake', True)
        self.declare_parameter('brake_delay', 0.3, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('brake_gain', 5.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('brake_max', 0.5, ParameterDescriptor(dynamic_typing=True))
        # 'agilex': execute a twist the way the real Ranger does. Its driver
        # (agilexrobotics/ranger_ros2, TwistCmdCallback) never runs a general
        # (vx, vy, wz): it picks ONE mode per command -- parallel if
        # linear.y != 0 (all wheels at one angle, angular.z ignored),
        # spinning if |vx|/|wz| < min_turn_radius (vx ignored), otherwise
        # dual Ackermann (front and rear mirrored about the lateral axis,
        # steer capped at max_steer_ackermann). The wheels therefore always
        # hold one of those geometries. 'free': any twist, per-wheel angles.
        self.declare_parameter('steer_modes', 'agilex')
        # AgileX's Ranger Mini V3 constants (ranger_params.hpp), which its
        # mode logic uses; the wheel angles themselves use the description's
        # geometry above.
        self.declare_parameter('agilex_wheelbase', 0.494,
                               ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('agilex_track', 0.364,
                               ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('min_turn_radius', 0.4764,
                               ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('max_steer_ackermann', 0.601,
                               ParameterDescriptor(dynamic_typing=True))
        # The driver sends atan((l/2)/R) as the steering angle, and its own
        # odometry reads the robot's steering angle back as the INNER wheel's
        # (ConvertInnerAngleToCentral). If the firmware realises it as the
        # inner angle, the base turns about R + track/2, not R. Assumed so;
        # the firmware is closed, so check it on the real robot.
        self.declare_parameter('ackermann_steer_is_inner', True)
        # 'effort' closes the speed loop here and sends torque. 'velocity'
        # sends the wheel speed straight through to a simulator whose joint
        # velocity drive is a torque-limited actuator, which then closes the
        # loop itself at the physics rate; the PI gains below are unused.
        #
        # Isaac has always used 'velocity'. gz used 'effort' because under
        # bullet-featherstone a joint velocity command is a rigid motor
        # constraint that over-determines the chassis and stops it yawing
        # (see ranger_wheels.xacro). The effort loop avoids that, but it
        # closes over DDS on a 10 ms timer and cannot be made stiff: on
        # rough ground the stock gains let wheel speeds sag up to 27 % and
        # the base turned 59 % of a commanded arc, and any gain high enough
        # to hold the speeds chatters between the effort clamps. sim.launch.py's
        # wheel_drive argument selects between the two on the gz side, and
        # defaults to velocity. So does this: the gz launch and Isaac both
        # pass the mode explicitly, and a node restarted by hand without it
        # should not send PI torques to what is now a velocity controller.
        self.declare_parameter('command_mode', 'velocity')

        self.r = self.get_parameter('wheel_radius').value
        lx = self.get_parameter('half_wheelbase').value
        ly = self.get_parameter('half_track').value
        self.max_w = self.get_parameter('max_wheel_speed').value
        self.timeout = self.get_parameter('cmd_timeout').value
        self.kp = self.get_parameter('wheel_kp').value
        self.ki = self.get_parameter('wheel_ki').value
        self.max_eff = self.get_parameter('max_wheel_effort').value
        self.gate_power = float(self.get_parameter('steer_gate_power').value)
        self.gate_mode = self.get_parameter('steer_gate_mode').value
        self.max_accel = float(self.get_parameter('max_wheel_accel').value)
        self.steer_deadband = float(self.get_parameter('steer_deadband').value)
        self.fold_hyst = math.radians(float(self.get_parameter('fold_hysteresis_deg').value))
        self.speed_ki = float(self.get_parameter('speed_ki').value)
        self.speed_i_max = float(self.get_parameter('speed_i_max').value)
        self.speed_i_spread = float(self.get_parameter('speed_i_spread').value)
        self.speed_i = {c: 0.0 for c in CORNERS}
        self.speed_sign = {c: 0.0 for c in CORNERS}
        self.brake = bool(self.get_parameter('parking_brake').value)
        self.brake_delay = float(self.get_parameter('brake_delay').value)
        self.brake_gain = float(self.get_parameter('brake_gain').value)
        self.brake_max = float(self.get_parameter('brake_max').value)
        self.stopped_for = 0.0
        self.brake_ref = None               # wheel angles held while parked
        self.steer_modes = self.get_parameter('steer_modes').value
        self.ag_l = float(self.get_parameter('agilex_wheelbase').value)
        self.ag_w = float(self.get_parameter('agilex_track').value)
        self.min_turn_radius = float(self.get_parameter('min_turn_radius').value)
        self.max_steer_ack = float(self.get_parameter('max_steer_ackermann').value)
        self.steer_is_inner = bool(self.get_parameter('ackermann_steer_is_inner').value)
        self.steer_mode = 'stop'
        self.last_cmd_speeds = [0.0] * len(CORNERS)
        if self.gate_mode not in ('common', 'per_wheel'):
            raise ValueError(f"steer_gate_mode must be common or per_wheel, got {self.gate_mode}")
        self.mode = self.get_parameter('command_mode').value
        if self.mode not in ('effort', 'velocity'):
            raise ValueError(f"command_mode must be effort or velocity, got {self.mode}")

        # Must match the corner order used for the controller's joint list.
        self.pos = {
            'front_left': (lx, ly),
            'front_right': (lx, -ly),
            'rear_left': (-lx, ly),
            'rear_right': (-lx, -ly),
        }
        self.last_steer = {c: 0.0 for c in CORNERS}
        self.wheel_vel = {c: 0.0 for c in CORNERS}
        self.wheel_pos = {c: None for c in CORNERS}
        # Measured knuckle angles, for the steering gate. None until the
        # first joint state arrives; the gate stays open until then rather
        # than holding the wheels still on a guess.
        self.steer_pos = {c: None for c in CORNERS}
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
            f'4WIS ready ({self.mode}'
            + (f', steer gate {self.gate_mode} power {self.gate_power:g}, '
               f'wheel accel {self.max_accel:g} m/s^2' if self.mode == 'velocity' else '')
            + f', steer deadband {self.steer_deadband:g} m/s, '
              f'fold hysteresis {math.degrees(self.fold_hyst):g} deg, modes {self.steer_modes}'
            + f'): r={self.r} half_wheelbase={lx} half_track={ly}')

    def _on_cmd(self, msg):
        self.twist = msg
        self.last_cmd_time = self.get_clock().now()

    def _on_joints(self, msg):
        for c in CORNERS:
            steer = f'{c}_steer_joint'
            if steer in msg.joint_names:
                iv = msg.interface_values[msg.joint_names.index(steer)]
                if 'position' in iv.interface_names:
                    self.steer_pos[c] = iv.values[iv.interface_names.index('position')]
            name = f'{c}_wheel_joint'
            if name not in msg.joint_names:
                continue
            iv = msg.interface_values[msg.joint_names.index(name)]
            if 'velocity' in iv.interface_names:
                self.wheel_vel[c] = iv.values[iv.interface_names.index('velocity')]
                self.got_feedback = True
            if 'position' in iv.interface_names:
                self.wheel_pos[c] = iv.values[iv.interface_names.index('position')]

    def _check_feedback(self):
        if not self.got_feedback:
            self.get_logger().warn(
                'no wheel velocity on /dynamic_joint_states; the speed loop '
                'is running open loop and the base will not brake')

    def _speed_loop(self, stopped, speeds):
        """Add the integral of the wheel speed error to the command."""
        if self.speed_ki <= 0.0 or not self.got_feedback:
            return speeds
        for c, w in zip(CORNERS, speeds):
            sign = math.copysign(1.0, w)
            if stopped or abs(w) < 1e-6 or sign != self.speed_sign[c]:
                self.speed_i[c] = 0.0                    # stopped, or the command reversed
                self.speed_sign[c] = sign if abs(w) >= 1e-6 else 0.0
            elif self.wheel_vel[c] * w >= 0.0:           # not turning against the command
                self.speed_i[c] = max(-self.speed_i_max, min(
                    self.speed_i_max, self.speed_i[c] + self.speed_ki * (w - self.wheel_vel[c]) * self.dt))
        if self.speed_i_spread > 0.0:
            mags = sorted(abs(self.speed_i[c]) for c in CORNERS)
            cap = 0.5 * (mags[1] + mags[2]) + self.speed_i_spread
            for c in CORNERS:
                self.speed_i[c] = max(-cap, min(cap, self.speed_i[c]))
        return [w + self.speed_i[c] for c, w in zip(CORNERS, speeds)]

    def _parking_brake(self, stopped, speeds):
        """Hold the wheels where they were once a stop has lasted brake_delay."""
        if not stopped or not self.brake or any(self.wheel_pos[c] is None for c in CORNERS):
            self.stopped_for, self.brake_ref = 0.0, None
            return speeds
        self.stopped_for += self.dt
        if self.stopped_for < self.brake_delay:
            return speeds
        if self.brake_ref is None:
            self.brake_ref = {c: self.wheel_pos[c] for c in CORNERS}
        return [max(-self.brake_max, min(self.brake_max,
                                          -self.brake_gain * (self.wheel_pos[c] - self.brake_ref[c])))
                for c in CORNERS]

    def _gate(self, steers, speeds):
        """Scale velocity-mode wheel speeds by knuckle convergence.

        'common' gates all four wheels on the WORST knuckle, so none drives
        until every knuckle has nearly converged and they then ramp
        together; the wheel set stays consistent with one body twist.
        'per_wheel' gates each wheel on its own knuckle, which lets the
        front wheels (shorter swing into a crab) push while the rears are
        still turning: the wheels' motion then implies 1.4-1.9 deg of yaw
        that the base only partly follows, and wheel odometry integrates
        the part it does not as error. Measured in both simulators; see
        crab_timeline.py and docs/TODO.md.
        """
        if self.gate_power <= 0.0 or any(self.steer_pos[c] is None for c in CORNERS):
            return speeds
        factors = [_gate_factor(a, self.steer_pos[c], self.gate_power)
                   for a, c in zip(steers, CORNERS)]
        if self.gate_mode == 'common':
            g = min(factors)
            return [w * g for w in speeds]
        return [w * f for w, f in zip(speeds, factors)]

    def _limit_accel(self, speeds):
        """Rate-limit the wheel set as a whole (see max_wheel_accel)."""
        if self.max_accel <= 0.0:
            self.last_cmd_speeds = list(speeds)
            return speeds
        step = self.max_accel * self.dt / self.r      # rad/s per tick
        deltas = [w - p for w, p in zip(speeds, self.last_cmd_speeds)]
        worst = max(abs(d) for d in deltas)
        k = 1.0 if worst <= step else step / worst
        out = [p + k * d for p, d in zip(self.last_cmd_speeds, deltas)]
        self.last_cmd_speeds = out
        return out

    def _agilex_twist(self, vx, vy, wz):
        """The body twist the real Ranger executes for (vx, vy, wz).

        Mirrors ranger_ros2's TwistCmdCallback mode choice and limits, then
        the firmware's dual-Ackermann turn about the lateral axis."""
        if vy != 0.0:                                   # parallel: angular ignored
            self.steer_mode = 'parallel'
            return vx, vy, 0.0
        if abs(wz) < 1e-6:
            self.steer_mode = 'ackermann' if vx != 0.0 else 'stop'
            return vx, 0.0, 0.0
        radius = abs(vx) / abs(wz)
        if radius < self.min_turn_radius:               # spinning: linear ignored
            self.steer_mode = 'spinning'
            return 0.0, 0.0, wz
        self.steer_mode = 'ackermann'
        half = self.ag_l / 2.0
        steer = min(math.atan(half / radius), self.max_steer_ack, math.radians(40.0))
        r_turn = half / math.tan(steer)
        if self.steer_is_inner:
            r_turn += self.ag_w / 2.0
        k = 1.0 if wz * vx >= 0.0 else -1.0
        return vx, 0.0, k * abs(vx) / r_turn

    def _tick(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        # A dropped publisher must coast to a stop, not keep driving.
        t = Twist() if age > self.timeout else self.twist
        vx, vy, wz = t.linear.x, t.linear.y, t.angular.z
        if self.steer_modes == 'agilex':
            vx, vy, wz = self._agilex_twist(vx, vy, wz)

        steers, speeds = [], []
        targets = []
        for c in CORNERS:
            xi, yi = self.pos[c]
            vxi = vx - wz * yi
            vyi = vy + wz * xi
            targets.append((c, vxi, vyi, math.hypot(vxi, vyi)))
        # With no command the real Ranger's wheels snap back to straight
        # ahead (reported on the robot; the driver sends a zero twist as dual
        # Ackermann with steer 0), so every stop costs the next move its
        # knuckle swing again. In 'free' mode, too slow for any wheel's
        # direction to mean anything: the knuckles hold together instead.
        stopped = max(tg[3] for tg in targets) < 1e-6
        rest_forward = self.steer_modes == 'agilex' and stopped
        hold = (self.steer_modes != 'agilex'
                and max(tg[3] for tg in targets) < max(self.steer_deadband, 1e-6))
        for c, vxi, vyi, speed in targets:
            last = self.last_steer[c]
            if rest_forward:
                angle = 0.0
                omega = 0.0
                self.last_steer[c] = 0.0
            elif hold:
                angle = last
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
                # Near the fold, stay on the knuckle's side rather than swing
                # 180 degrees across it.
                if (abs(angle - last) > math.pi / 2.0
                        and abs(angle) > math.pi / 2.0 - self.fold_hyst):
                    other = angle - math.copysign(math.pi, angle)
                    if abs(other) <= math.pi / 2.0 + self.fold_hyst:
                        angle = max(-math.pi / 2.0, min(math.pi / 2.0, other))
                        omega = -omega
                self.last_steer[c] = angle

            omega = max(-self.max_w, min(self.max_w, omega))
            steers.append(angle)

            if self.mode == 'velocity':
                speeds.append(omega)      # gated below, once every angle is known
                continue

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

        if self.mode == 'velocity':
            speeds = self._gate(steers, speeds)
            speeds = self._limit_accel(speeds)
            speeds = self._speed_loop(stopped, speeds)
            speeds = self._parking_brake(stopped, speeds)

        self.steer_pub.publish(Float64MultiArray(data=steers))
        self.wheel_pub.publish(Float64MultiArray(data=speeds))


def _gate_factor(target, measured, power):
    return max(0.0, math.cos(target - measured)) ** power


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
