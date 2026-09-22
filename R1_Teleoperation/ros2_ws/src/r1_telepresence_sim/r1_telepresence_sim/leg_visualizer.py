"""
Simulation-only leg motion visualizer driven by ``/r1/sim/cmd_vel``.

The checked-in R1 Gazebo model has a pelvis fixed to ``world`` and exposes
position *state* (but no position command) interfaces for the leg joints.  A
real biped gait controller therefore cannot be inferred from ``/cmd_vel``.
This node is an explicitly simulation-only visual proxy: it applies a small,
bounded stepping pose through a Gazebo ``ros2_control`` trajectory controller
so a stick command is visible in a demonstration.  It never opens a Unitree
transport or publishes hardware commands.
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
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


LEG_JOINTS: Tuple[str, ...] = (
    'left_hip_pitch_joint',
    'left_hip_roll_joint',
    'left_hip_yaw_joint',
    'left_knee_joint',
    'left_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_hip_pitch_joint',
    'right_hip_roll_joint',
    'right_hip_yaw_joint',
    'right_knee_joint',
    'right_ankle_pitch_joint',
    'right_ankle_roll_joint',
)

# Conservative limits from the local R1 URDF.  The visualizer stays well
# inside these limits; the table is also a final guard against bad parameters.
JOINT_LIMITS: Mapping[str, Tuple[float, float]] = {
    'left_hip_pitch_joint': (-1.5, 1.5),
    'left_hip_roll_joint': (-1.047, 1.745),
    'left_hip_yaw_joint': (-2.74, 2.74),
    'left_knee_joint': (-0.174, 2.426),
    'left_ankle_pitch_joint': (-0.872, 0.575),
    'left_ankle_roll_joint': (-0.26, 0.26),
    'right_hip_pitch_joint': (-1.5, 1.5),
    'right_hip_roll_joint': (-1.745, 1.047),
    'right_hip_yaw_joint': (-2.74, 2.74),
    'right_knee_joint': (-0.174, 2.426),
    'right_ankle_pitch_joint': (-0.872, 0.575),
    'right_ankle_roll_joint': (-0.26, 0.26),
}


def _clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a finite scalar to an inclusive interval."""
    return max(lower, min(upper, float(value)))


def _finite_or_zero(value: float) -> float:
    """Return zero for NaN/Inf input so one bad packet cannot poison gait."""
    return float(value) if math.isfinite(float(value)) else 0.0


def _bounded_joint(name: str, value: float) -> float:
    """Apply the URDF limit and reject non-finite target values."""
    lower, upper = JOINT_LIMITS[name]
    return _clamp(_finite_or_zero(value), lower, upper)


def neutral_leg_targets() -> Dict[str, float]:
    """Return a stable zero pose for all leg joints."""
    return {name: 0.0 for name in LEG_JOINTS}


def compute_leg_targets(
    linear_x: float,
    linear_y: float,
    yaw: float,
    phase: float,
    *,
    max_forward: float = 0.35,
    max_lateral: float = 0.25,
    max_yaw: float = 0.60,
    # The pelvis is fixed in this Gazebo stand-in, so Sport-sized joint
    # amplitudes look like a dance rather than a supported step.  Keep the
    # visual stride deliberately small until a contact-aware controller exists.
    hip_pitch_amplitude: float = 0.14,
    knee_amplitude: float = 0.18,
    ankle_pitch_amplitude: float = 0.07,
    hip_roll_amplitude: float = 0.045,
    hip_yaw_amplitude: float = 0.05,
    stance_roll_amplitude: float = 0.015,
) -> Dict[str, float]:
    """
    Compute a bounded, mirrored stepping pose from a velocity intent.

    This is deliberately not a balance controller.  Forward speed controls
    the alternating pitch/knee swing, lateral speed adds a small symmetric
    roll, and yaw adds a small turn bias.  At zero speed the caller should use
    :func:`neutral_leg_targets` so the legs settle instead of oscillating.
    """
    max_forward = max(float(max_forward), 1.0e-6)
    max_lateral = max(float(max_lateral), 1.0e-6)
    max_yaw = max(float(max_yaw), 1.0e-6)
    vx = _clamp(_finite_or_zero(linear_x), -max_forward, max_forward)
    vy = _clamp(_finite_or_zero(linear_y), -max_lateral, max_lateral)
    wz = _clamp(_finite_or_zero(yaw), -max_yaw, max_yaw)

    forward_ratio = abs(vx) / max_forward
    lateral_ratio = vy / max_lateral
    yaw_ratio = wz / max_yaw
    speed_ratio = min(1.0, math.hypot(vx / max_forward, vy / max_lateral))
    # Do not make a full forward stride out of a tiny turn/side command.  The
    # old mapping used ``speed_ratio`` for every axis, which made a small
    # right-stick yaw look like an abrupt leg kick in Gazebo.
    step_ratio = min(
        1.0,
        max(
            forward_ratio,
            0.55 * abs(lateral_ratio),
            0.35 * abs(yaw_ratio),
        ),
    )
    direction = -1.0 if vx < 0.0 else 1.0
    targets: Dict[str, float] = {}

    # Opposite phases give an immediately readable left/right stepping motion.
    for side, side_sign, side_phase in (
        ('left', 1.0, float(phase)),
        ('right', -1.0, float(phase) + math.pi),
    ):
        # In this URDF a positive hip-pitch angle moves the foot backwards.
        # Negate the phase so a positive forward stick command produces a
        # forward swing.  The knee uses the same half-wave envelope and never
        # snaps through a negative bend at the phase boundary.
        swing = -math.sin(side_phase) * direction * speed_ratio
        # The lifted leg is selected by phase, not travel direction: when
        # walking backwards the same leg still lifts, it simply swings behind
        # the pelvis instead of in front of it.
        lift = max(0.0, math.sin(side_phase)) * speed_ratio
        # Keep the feet slightly outside the pelvis during a step.  This is a
        # visual-only substitute for the lateral support loop that Sport mode
        # normally provides and prevents the fixed-pelvis model from looking
        # as if its legs cross during the swing.
        roll = side_sign * (
            lateral_ratio * hip_roll_amplitude
            + step_ratio * stance_roll_amplitude
        )
        # Mirrored yaw offsets keep a turn from rotating both knees toward the
        # same side of the body.
        yaw_target = side_sign * yaw_ratio * hip_yaw_amplitude
        values = {
            f'{side}_hip_pitch_joint': hip_pitch_amplitude * swing,
            f'{side}_hip_roll_joint': roll,
            f'{side}_hip_yaw_joint': yaw_target,
            f'{side}_knee_joint': knee_amplitude * lift,
            f'{side}_ankle_pitch_joint': -ankle_pitch_amplitude * swing,
            f'{side}_ankle_roll_joint': -0.5 * roll,
        }
        targets.update({name: _bounded_joint(name, value) for name, value in values.items()})

    return targets


def _ordered_positions(targets: Mapping[str, float]) -> Sequence[float]:
    """Return targets in the stable service/debug joint order."""
    return [float(targets.get(name, 0.0)) for name in LEG_JOINTS]


class SimulationLegVisualizer(Node):
    """
    Publish a deadman-gated stepping pose to the simulation controller.

    The checked-in R1 simulation URDF exposes position command interfaces for
    the legs and starts ``leg_trajectory_controller``.  Publishing a short
    JointTrajectory point is supported by ros2_control and avoids relying on
    the Gazebo Classic ``SetModelConfiguration`` service, which is not
    implemented by the ROS 2 plugin shipped on this machine.
    """

    def __init__(self):
        super().__init__('r1_sim_leg_visualizer')
        self.declare_parameter('model_name', 'unitree_r1')
        self.declare_parameter('input_topic', '/r1/sim/cmd_vel')
        self.declare_parameter('active_topic', '/vr/teleop/active')
        self.declare_parameter(
            'output_topic', '/leg_trajectory_controller/joint_trajectory'
        )
        self.declare_parameter('debug_topic', '/r1/telepresence/leg_targets')
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.25)
        self.declare_parameter('command_duration_sec', 0.08)
        self.declare_parameter('max_forward_mps', 0.35)
        self.declare_parameter('max_lateral_mps', 0.25)
        self.declare_parameter('max_yaw_rps', 0.60)
        self.declare_parameter('step_frequency_hz', 1.6)

        self._model_name = str(self.get_parameter('model_name').value)
        self._input_topic = str(self.get_parameter('input_topic').value)
        self._active_topic = str(self.get_parameter('active_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._rate = float(self.get_parameter('update_rate_hz').value)
        self._timeout = float(self.get_parameter('cmd_timeout_sec').value)
        self._command_duration = float(
            self.get_parameter('command_duration_sec').value
        )
        self._max_forward = float(self.get_parameter('max_forward_mps').value)
        self._max_lateral = float(self.get_parameter('max_lateral_mps').value)
        self._max_yaw = float(self.get_parameter('max_yaw_rps').value)
        self._step_frequency = float(self.get_parameter('step_frequency_hz').value)
        if min(
            self._rate,
            self._timeout,
            self._max_forward,
            self._max_lateral,
            self._max_yaw,
            self._step_frequency,
            self._command_duration,
        ) <= 0.0:
            raise ValueError('visualizer rate, timeout, limits and frequency must be positive')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._active = False
        self._last_cmd = Twist()
        self._last_arrival = time.monotonic()
        self._phase = 0.0
        self._last_tick = time.monotonic()

        self._trajectory = self.create_publisher(
            JointTrajectory, self._output_topic, qos
        )
        self._debug = self.create_publisher(
            JointState,
            str(self.get_parameter('debug_topic').value),
            qos,
        )
        self.create_subscription(Twist, self._input_topic, self._cmd_callback, qos)
        self.create_subscription(Bool, self._active_topic, self._active_callback, qos)
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f'simulation leg visualizer: {self._input_topic} -> '
            f'{self._output_topic} ({self._model_name}); hardware disabled'
        )

    def _active_callback(self, message: Bool) -> None:
        self._active = bool(message.data)
        if not self._active:
            self._last_cmd = Twist()

    def _cmd_callback(self, message: Twist) -> None:
        if not self._active:
            return
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(float(value)) for value in values):
            self._last_cmd = Twist()
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
        self._last_arrival = time.monotonic()

    def _tick(self) -> None:
        # Launch can invalidate the ROS context while one timer callback is
        # still queued.  Do not attempt a publish after shutdown has started.
        if not rclpy.ok():
            return
        now = time.monotonic()
        dt = min(0.1, max(1.0e-4, now - self._last_tick))
        self._last_tick = now
        fresh = self._active and now - self._last_arrival <= self._timeout
        if fresh:
            cmd = self._last_cmd
            normalized_speed = min(
                1.0,
                math.hypot(
                    cmd.linear.x / self._max_forward,
                    cmd.linear.y / self._max_lateral,
                ),
            )
            if normalized_speed > 1.0e-3 or abs(cmd.angular.z) > 1.0e-3:
                cadence = 0.35 + 0.65 * normalized_speed
                self._phase = (
                    self._phase
                    + dt * 2.0 * math.pi * self._step_frequency * cadence
                ) % (2.0 * math.pi)
                targets = compute_leg_targets(
                    cmd.linear.x,
                    cmd.linear.y,
                    cmd.angular.z,
                    self._phase,
                    max_forward=self._max_forward,
                    max_lateral=self._max_lateral,
                    max_yaw=self._max_yaw,
                )
            else:
                targets = neutral_leg_targets()
        else:
            self._phase = 0.0
            targets = neutral_leg_targets()
        try:
            self._publish_debug(targets)
            self._publish_trajectory(targets)
        except Exception:
            # A second shutdown edge can race the publish calls.  Suppress
            # only that expected invalid-context error; keep real runtime
            # failures visible while the node is still active.
            if rclpy.ok():
                raise

    def _publish_debug(self, targets: Mapping[str, float]) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(LEG_JOINTS)
        message.position = list(_ordered_positions(targets))
        self._debug.publish(message)

    def _publish_trajectory(self, targets: Mapping[str, float]) -> None:
        """Send one short position segment to the simulation-only controller."""
        trajectory = JointTrajectory()
        # Leave the stamp at zero: the Gazebo controller uses this as
        # "start immediately".  The visualizer does not own a simulation-time
        # clock, and a wall-time stamp would otherwise be interpreted as a
        # trajectory scheduled billions of seconds in the future.
        trajectory.joint_names = list(LEG_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(_ordered_positions(targets))
        duration = self._command_duration
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int(
            round((duration - int(duration)) * 1.0e9)
        )
        if point.time_from_start.nanosec >= 1_000_000_000:
            point.time_from_start.sec += 1
            point.time_from_start.nanosec -= 1_000_000_000
        trajectory.points = [point]
        self._trajectory.publish(trajectory)

    def stop(self) -> None:
        """Return the simulated legs to neutral without touching hardware."""
        self._active = False
        self._last_cmd = Twist()
        self._publish_trajectory(neutral_leg_targets())


def main(args=None):
    rclpy.init(args=args)
    node = SimulationLegVisualizer()
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
