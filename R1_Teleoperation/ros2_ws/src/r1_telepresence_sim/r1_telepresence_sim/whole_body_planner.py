"""
Simulation-only whole-body coordination for the R1 avatar.

This module deliberately stops at the Gazebo boundary.  It combines the
simulation leg visualizer, headset/IK arm targets, and a small torso
compensation into one deadman-gated output stream.  It is a visual proxy for
whole-body timing, not a balance controller and never opens a Unitree
transport.
"""

from __future__ import annotations

import math
import time
from typing import Dict, Mapping, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .leg_visualizer import LEG_JOINTS, JOINT_LIMITS as LEG_LIMITS, neutral_leg_targets


ARM_JOINTS: Tuple[str, ...] = (
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
)
WAIST_JOINTS: Tuple[str, ...] = ('waist_roll_joint', 'waist_yaw_joint')

# Conservative simulation limits.  These values are intentionally narrower
# than the physical actuator envelope: this node is a coordination visualizer,
# not a replacement for Unitree's balance controller.
ARM_LIMITS: Mapping[str, Tuple[float, float]] = {
    name: (-2.7, 2.7) for name in ARM_JOINTS
}
WAIST_LIMITS: Mapping[str, Tuple[float, float]] = {
    'waist_roll_joint': (-0.30, 0.30),
    'waist_yaw_joint': (-0.45, 0.45),
}


def _finite(value: float) -> float:
    """Return a finite scalar or zero."""
    value = float(value)
    return value if math.isfinite(value) else 0.0


def _clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a scalar to an inclusive interval."""
    return max(float(lower), min(float(upper), _finite(value)))


def _bounded(name: str, value: float, limits: Mapping[str, Tuple[float, float]]) -> float:
    """Apply a named joint limit and reject non-finite input."""
    lower, upper = limits[name]
    return _clamp(value, lower, upper)


def neutral_arm_targets() -> Dict[str, float]:
    """Return a complete neutral pose for both simulation arms."""
    return {name: 0.0 for name in ARM_JOINTS}


def neutral_waist_targets() -> Dict[str, float]:
    """Return a neutral waist pose."""
    return {name: 0.0 for name in WAIST_JOINTS}


def compute_waist_targets(
    linear_y: float,
    yaw: float,
    *,
    max_lateral: float = 0.25,
    max_yaw: float = 0.60,
    roll_gain: float = 0.08,
    yaw_gain: float = 0.12,
) -> Dict[str, float]:
    """
    Compute small counter-lean and turn compensation for the torso.

    The real Sport controller closes the balance loop with IMU/contact
    feedback.  The fixed-pelvis Gazebo model has no such loop, therefore this
    function only gives the avatar a bounded visual response to lateral and
    yaw intent.
    """
    lateral_ratio = _clamp(linear_y, -max_lateral, max_lateral) / max(
        float(max_lateral), 1.0e-6
    )
    yaw_ratio = _clamp(yaw, -max_yaw, max_yaw) / max(float(max_yaw), 1.0e-6)
    return {
        'waist_roll_joint': _bounded(
            'waist_roll_joint', -roll_gain * lateral_ratio, WAIST_LIMITS
        ),
        'waist_yaw_joint': _bounded(
            'waist_yaw_joint', -yaw_gain * yaw_ratio, WAIST_LIMITS
        ),
    }


def apply_counter_swing(
    arm_targets: Mapping[str, float],
    phase: float,
    speed_ratio: float,
    *,
    # Keep the visual gait cue below the arm workspace scale.  Larger values
    # made a valid VR hand target move forward enough to look like crossed arms.
    pitch_gain: float = 0.08,
    roll_gain: float = 0.020,
    yaw_gain: float = 0.010,
    lateral_ratio: float = 0.0,
    yaw_ratio: float = 0.0,
) -> Dict[str, float]:
    """
    Add a bounded Sport-like counter-swing to already solved arm joints.

    The left leg's visual swing is ``sin(phase)`` and the right leg is its
    opposite.  Arms are deliberately commanded in the opposite phase.  User
    hand/arm motion remains the base target; this function contributes only a
    small gait offset and clamps every result.
    """
    speed = _clamp(speed_ratio, 0.0, 1.0)
    left_leg_swing = math.sin(_finite(phase)) * speed
    left_offset = -pitch_gain * left_leg_swing
    right_offset = -left_offset
    lateral = _clamp(lateral_ratio, -1.0, 1.0)
    yaw = _clamp(yaw_ratio, -1.0, 1.0)
    offsets = {
        'left_shoulder_pitch_joint': left_offset,
        'right_shoulder_pitch_joint': right_offset,
        'left_shoulder_roll_joint': roll_gain * lateral,
        'right_shoulder_roll_joint': -roll_gain * lateral,
        'left_shoulder_yaw_joint': yaw_gain * yaw,
        'right_shoulder_yaw_joint': yaw_gain * yaw,
    }
    output = {
        name: _bounded(name, arm_targets.get(name, 0.0), ARM_LIMITS)
        for name in ARM_JOINTS
    }
    for name, offset in offsets.items():
        output[name] = _bounded(name, output[name] + offset, ARM_LIMITS)
    return output


def rate_limit_targets(
    targets: Mapping[str, float],
    previous: Mapping[str, float],
    previous_velocity: Mapping[str, float],
    dt: float,
    *,
    limits: Mapping[str, Tuple[float, float]],
    max_velocity: float,
    max_acceleration: float,
) -> Tuple[Dict[str, float], Dict[str, float]]:
    """
    Apply finite, joint-limit, velocity and acceleration guards.

    The returned velocity is the velocity actually used for this sample.  A
    malformed target is therefore reduced to a safe bounded value instead of
    propagating NaN into a trajectory message.
    """
    dt = _clamp(dt, 1.0e-4, 0.2)
    max_velocity = max(_finite(max_velocity), 1.0e-6)
    max_acceleration = max(_finite(max_acceleration), 1.0e-6)
    output: Dict[str, float] = {}
    velocity: Dict[str, float] = {}
    for name, (lower, upper) in limits.items():
        old = _bounded(name, previous.get(name, 0.0), limits)
        old_velocity = _clamp(
            previous_velocity.get(name, 0.0), -max_velocity, max_velocity
        )
        desired = _bounded(name, targets.get(name, 0.0), limits)
        desired_velocity = (desired - old) / dt
        desired_velocity = _clamp(
            desired_velocity, -max_velocity, max_velocity
        )
        velocity_step = max_acceleration * dt
        next_velocity = _clamp(
            old_velocity + _clamp(
                desired_velocity - old_velocity, -velocity_step, velocity_step
            ),
            -max_velocity,
            max_velocity,
        )
        next_position = _bounded(name, old + next_velocity * dt, limits)
        if abs(desired - next_position) <= max_velocity * dt:
            next_position = desired
            next_velocity = (next_position - old) / dt
        output[name] = next_position
        velocity[name] = next_velocity
    return output, velocity


def _trajectory_targets(message: JointTrajectory) -> Dict[str, float] | None:
    """Extract and validate the first point of a trajectory."""
    if not message.points:
        return None
    point = message.points[0]
    if len(message.joint_names) != len(point.positions):
        return None
    names = tuple(str(name) for name in message.joint_names)
    if not names or len(names) != len(set(names)):
        return None
    values = [float(value) for value in point.positions]
    if not all(math.isfinite(value) for value in values):
        return None
    return dict(zip(names, values))


class SimulationWholeBodyPlanner(Node):
    """Coordinate simulation legs, arms and waist behind one safety gate."""

    def __init__(self):
        super().__init__('r1_sim_whole_body_planner')
        self.declare_parameter('cmd_topic', '/r1/sim/cmd_vel')
        self.declare_parameter('active_topic', '/vr/teleop/active')
        self.declare_parameter('leg_input_topic', '/r1/sim/leg_trajectory')
        self.declare_parameter(
            'arm_input_topic', '/r1_kinematics_control/debug/arm_trajectory'
        )
        self.declare_parameter(
            'leg_output_topic', '/leg_trajectory_controller/joint_trajectory'
        )
        self.declare_parameter(
            'arm_output_topic', '/arm_trajectory_controller/joint_trajectory'
        )
        self.declare_parameter(
            'waist_output_topic', '/waist_hold_controller/commands'
        )
        self.declare_parameter('debug_topic', '/r1/telepresence/whole_body_targets')
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 0.25)
        self.declare_parameter('active_timeout_sec', 1.5)
        self.declare_parameter('step_frequency_hz', 1.6)
        self.declare_parameter('max_forward_mps', 0.35)
        self.declare_parameter('max_lateral_mps', 0.25)
        self.declare_parameter('max_yaw_rps', 0.60)
        self.declare_parameter('trajectory_duration_sec', 0.08)
        self.declare_parameter('max_joint_velocity_rad_s', 1.8)
        self.declare_parameter('max_joint_acceleration_rad_s2', 6.0)
        self.declare_parameter('arm_swing_enabled', True)

        value = self.get_parameter
        self._cmd_topic = str(value('cmd_topic').value)
        self._active_topic = str(value('active_topic').value)
        self._leg_input_topic = str(value('leg_input_topic').value)
        self._arm_input_topic = str(value('arm_input_topic').value)
        self._leg_output_topic = str(value('leg_output_topic').value)
        self._arm_output_topic = str(value('arm_output_topic').value)
        self._waist_output_topic = str(value('waist_output_topic').value)
        self._debug_topic = str(value('debug_topic').value)
        self._rate = float(value('update_rate_hz').value)
        self._cmd_timeout = float(value('cmd_timeout_sec').value)
        self._active_timeout = float(value('active_timeout_sec').value)
        self._step_frequency = float(value('step_frequency_hz').value)
        self._max_forward = float(value('max_forward_mps').value)
        self._max_lateral = float(value('max_lateral_mps').value)
        self._max_yaw = float(value('max_yaw_rps').value)
        self._duration = float(value('trajectory_duration_sec').value)
        self._max_velocity = float(value('max_joint_velocity_rad_s').value)
        self._max_acceleration = float(
            value('max_joint_acceleration_rad_s2').value
        )
        self._arm_swing_enabled = bool(value('arm_swing_enabled').value)
        if min(
            self._rate,
            self._cmd_timeout,
            self._active_timeout,
            self._step_frequency,
            self._max_forward,
            self._max_lateral,
            self._max_yaw,
            self._duration,
            self._max_velocity,
            self._max_acceleration,
        ) <= 0.0:
            raise ValueError('whole-body rates, limits and timeouts must be positive')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._leg_publisher = self.create_publisher(
            JointTrajectory, self._leg_output_topic, qos
        )
        self._arm_publisher = self.create_publisher(
            JointTrajectory, self._arm_output_topic, qos
        )
        self._waist_publisher = self.create_publisher(
            Float64MultiArray, self._waist_output_topic, qos
        )
        self._debug_publisher = self.create_publisher(
            JointState, self._debug_topic, qos
        )
        self.create_subscription(Twist, self._cmd_topic, self._cmd_callback, qos)
        self.create_subscription(Bool, self._active_topic, self._active_callback, qos)
        self.create_subscription(
            JointState, self._leg_input_topic, self._leg_callback, qos
        )
        self.create_subscription(
            JointTrajectory, self._arm_input_topic, self._arm_callback, qos
        )

        self._active = False
        self._active_arrival = 0.0
        self._last_cmd = Twist()
        self._cmd_arrival = 0.0
        self._last_leg = neutral_leg_targets()
        self._leg_arrival = 0.0
        self._last_arm = neutral_arm_targets()
        self._arm_arrival = 0.0
        self._phase = 0.0
        self._last_tick = time.monotonic()
        all_joints = tuple(LEG_JOINTS) + tuple(ARM_JOINTS) + tuple(WAIST_JOINTS)
        self._limits = {
            **LEG_LIMITS,
            **ARM_LIMITS,
            **WAIST_LIMITS,
        }
        self._previous = {name: 0.0 for name in all_joints}
        self._previous_velocity = {name: 0.0 for name in all_joints}
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            'simulation whole-body planner ready; hardware transport disabled '
            f'({self._leg_input_topic}, {self._arm_input_topic})'
        )

    def _active_callback(self, message: Bool) -> None:
        self._active = bool(message.data)
        self._active_arrival = time.monotonic()
        if not self._active:
            self._phase = 0.0

    def _cmd_callback(self, message: Twist) -> None:
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(float(item)) for item in values):
            self._last_cmd = Twist()
            self._cmd_arrival = 0.0
            return
        self._last_cmd = Twist()
        self._last_cmd.linear.x = _clamp(
            message.linear.x, -self._max_forward, self._max_forward
        )
        self._last_cmd.linear.y = _clamp(
            message.linear.y, -self._max_lateral, self._max_lateral
        )
        self._last_cmd.angular.z = _clamp(
            message.angular.z, -self._max_yaw, self._max_yaw
        )
        self._cmd_arrival = time.monotonic()

    def _leg_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, message.position))
        if not all(name in values and math.isfinite(float(values[name])) for name in LEG_JOINTS):
            return
        self._last_leg = {
            name: _bounded(name, values[name], LEG_LIMITS) for name in LEG_JOINTS
        }
        self._leg_arrival = time.monotonic()

    def _arm_callback(self, message: JointTrajectory) -> None:
        values = _trajectory_targets(message)
        if values is None:
            return
        if not any(name in values for name in ARM_JOINTS):
            return
        self._last_arm = {
            name: _bounded(name, values.get(name, 0.0), ARM_LIMITS)
            for name in ARM_JOINTS
        }
        self._arm_arrival = time.monotonic()

    def _is_fresh(self, now: float) -> bool:
        return (
            self._active
            and now - self._active_arrival <= self._active_timeout
            and self._cmd_arrival > 0.0
            and now - self._cmd_arrival <= self._cmd_timeout
        )

    def _tick(self) -> None:
        if not rclpy.ok():
            return
        now = time.monotonic()
        dt = min(0.1, max(1.0e-4, now - self._last_tick))
        self._last_tick = now
        fresh = self._is_fresh(now)
        if fresh:
            cmd = self._last_cmd
            speed = min(
                1.0,
                math.hypot(
                    cmd.linear.x / self._max_forward,
                    cmd.linear.y / self._max_lateral,
                ),
            )
            if speed > 1.0e-3 or abs(cmd.angular.z) > 1.0e-3:
                cadence = 0.35 + 0.65 * speed
                self._phase = (
                    self._phase
                    + dt * 2.0 * math.pi * self._step_frequency * cadence
                ) % (2.0 * math.pi)
            else:
                speed = 0.0
            leg_targets = self._last_leg
            arm_targets = self._last_arm
            waist_targets = compute_waist_targets(
                cmd.linear.y,
                cmd.angular.z,
                max_lateral=self._max_lateral,
                max_yaw=self._max_yaw,
            )
            if self._arm_swing_enabled:
                arm_targets = apply_counter_swing(
                    arm_targets,
                    self._phase,
                    speed,
                    lateral_ratio=cmd.linear.y / self._max_lateral,
                    yaw_ratio=cmd.angular.z / self._max_yaw,
                )
        else:
            speed = 0.0
            self._phase = 0.0
            leg_targets = neutral_leg_targets()
            arm_targets = neutral_arm_targets()
            waist_targets = neutral_waist_targets()

        targets = {**leg_targets, **arm_targets, **waist_targets}
        try:
            limited, velocity = rate_limit_targets(
                targets,
                self._previous,
                self._previous_velocity,
                dt,
                limits=self._limits,
                max_velocity=self._max_velocity,
                max_acceleration=self._max_acceleration,
            )
            self._previous = limited
            self._previous_velocity = velocity
            self._publish_leg(limited)
            self._publish_arm(limited)
            self._publish_waist(limited)
            self._publish_debug(limited, fresh, speed)
        except Exception:
            if rclpy.ok():
                raise

    def _publish_leg(self, targets: Mapping[str, float]) -> None:
        self._publish_trajectory(
            self._leg_publisher, LEG_JOINTS, targets
        )

    def _publish_arm(self, targets: Mapping[str, float]) -> None:
        self._publish_trajectory(
            self._arm_publisher, ARM_JOINTS, targets
        )

    def _publish_trajectory(self, publisher, names: Sequence[str], targets) -> None:
        message = JointTrajectory()
        message.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(targets.get(name, 0.0)) for name in names]
        seconds = int(self._duration)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int(
            round((self._duration - seconds) * 1.0e9)
        )
        if point.time_from_start.nanosec >= 1_000_000_000:
            point.time_from_start.sec += 1
            point.time_from_start.nanosec -= 1_000_000_000
        message.points = [point]
        publisher.publish(message)

    def _publish_waist(self, targets: Mapping[str, float]) -> None:
        message = Float64MultiArray()
        message.data = [float(targets.get(name, 0.0)) for name in WAIST_JOINTS]
        self._waist_publisher.publish(message)

    def _publish_debug(
        self, targets: Mapping[str, float], fresh: bool, speed: float
    ) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(LEG_JOINTS + ARM_JOINTS + WAIST_JOINTS)
        message.position = [float(targets[name]) for name in message.name]
        # Keep JointState arrays well formed.  The first two entries are a
        # compact diagnostic (speed ratio and fresh/deadman flag); the rest are
        # zero because this topic describes targets, not measured velocities.
        message.velocity = [0.0] * len(message.name)
        message.velocity[0] = float(speed)
        message.velocity[1] = 1.0 if fresh else 0.0
        self._debug_publisher.publish(message)

    def stop(self) -> None:
        """Return every simulated group to a bounded neutral pose."""
        self._active = False
        neutral = {
            **neutral_leg_targets(),
            **neutral_arm_targets(),
            **neutral_waist_targets(),
        }
        self._previous = neutral
        self._previous_velocity = {name: 0.0 for name in neutral}
        self._publish_leg(neutral)
        self._publish_arm(neutral)
        self._publish_waist(neutral)


def main(args=None):
    """Run the localhost-only whole-body simulation node."""
    rclpy.init(args=args)
    node = SimulationWholeBodyPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                node.stop()
        except BaseException:
            pass
        try:
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
