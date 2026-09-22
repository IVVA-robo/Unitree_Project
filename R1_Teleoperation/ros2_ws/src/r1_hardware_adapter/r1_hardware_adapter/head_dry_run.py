"""
ROS 2 dry-run node for headset-relative R1 head-orientation diagnostics.

The node deliberately has no Unitree SDK import and no actuator/controller
publisher.  Its JointTrajectory output is a debug topic only; it records what
a separately reviewed physical head writer could consume in the future.
"""

from dataclasses import dataclass
import math
import time
from typing import Optional, Tuple

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Vector3Stamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .head_controller import (
    Axes,
    HeadController,
    HeadDecision,
    HeadLimits,
    euler_from_quaternion,
)


_MIN_HEAD_PUBLISH_RATE_HZ = 10.0
_MAX_HEAD_PUBLISH_RATE_HZ = 100.0
_MAX_HEAD_STATUS_PERIOD_SEC = 2.0


@dataclass(frozen=True)
class HeadPoseSample:
    """A valid VR quaternion and its local, monotonic arrival time."""

    quaternion: Tuple[float, float, float, float]
    arrival: float


class HeadControllerNode(Node):
    """Produce dry-run, safety-gated head targets from VR ``PoseStamped`` data."""

    def __init__(self):
        super().__init__('r1_head_dry_run')
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        hardware_enabled = bool(self.get_parameter('hardware_enabled').value)
        dry_run = bool(self.get_parameter('dry_run').value)
        if hardware_enabled or not dry_run:
            raise RuntimeError(
                'r1_head_dry_run is diagnostics-only; it requires '
                'hardware_enabled=false and dry_run=true'
            )

        self._controller = HeadController(self._limits)
        self._latest_pose: Optional[HeadPoseSample] = None
        self._latest_pose_error = 'pose_missing'
        self._last_source_euler: Axes = (0.0, 0.0, 0.0)
        self._deadman_seen = False
        self._deadman_active = False
        self._deadman_arrival: Optional[float] = None
        # Fail closed: a kill must be explicitly cleared by a false message.
        self._kill_seen = False
        self._kill_active = True
        self._last_status_key = None
        self._last_status_time = -math.inf

        debug_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # A supervisor publishes this state with TRANSIENT_LOCAL durability,
        # allowing a node that starts late to remain safely killed until it
        # receives the retained explicit state.
        kill_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._raw_publisher = self.create_publisher(
            Vector3Stamped, f'{self._debug_prefix}/vr_euler', debug_qos
        )
        self._relative_publisher = self.create_publisher(
            Vector3Stamped, f'{self._debug_prefix}/relative_euler', debug_qos
        )
        self._command_publisher = self.create_publisher(
            Vector3Stamped, f'{self._debug_prefix}/command_euler', debug_qos
        )
        self._trajectory_publisher = self.create_publisher(
            JointTrajectory, f'{self._debug_prefix}/joint_trajectory', debug_qos
        )
        self._status_publisher = self.create_publisher(
            String, f'{self._debug_prefix}/status', debug_qos
        )

        self.create_subscription(
            PoseStamped,
            self._head_pose_topic,
            self._head_pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            self._active_topic,
            self._deadman_callback,
            debug_qos,
        )
        self.create_subscription(
            Bool,
            self._emergency_stop_topic,
            self._kill_callback,
            kill_qos,
        )
        self.create_service(
            Trigger,
            self._calibration_service,
            self._calibrate_callback,
        )
        self.create_service(
            Trigger,
            self._reset_service,
            self._reset_callback,
        )
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._tick)
        self.get_logger().warning(
            'head dry-run active: no Unitree SDK or actuator writer is '
            'configured; kill remains blocked until an explicit false Bool is '
            f'received on {self._emergency_stop_topic}',
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('hardware_enabled', False)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('head_pose_topic', '/vr/head/pose')
        self.declare_parameter('active_topic', '/vr/teleop/active')
        self.declare_parameter('emergency_stop_topic', '/r1/safety/kill')
        self.declare_parameter(
            'debug_topic_prefix', '/r1_hardware_adapter/debug/head'
        )
        self.declare_parameter(
            'calibration_service', '/r1/head/calibrate_neutral'
        )
        self.declare_parameter(
            'reset_service', '/r1/head/reset_calibration'
        )
        self.declare_parameter('pose_timeout_sec', 0.35)
        self.declare_parameter('active_timeout_sec', 1.50)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('status_publish_period_sec', 0.50)
        self.declare_parameter('trajectory_time_sec', 0.05)
        self.declare_parameter('yaw_scale', 1.0)
        self.declare_parameter('pitch_scale', 1.0)
        self.declare_parameter('roll_scale', 1.0)
        self.declare_parameter('yaw_limit_rad', 0.65)
        self.declare_parameter('pitch_limit_rad', 0.45)
        self.declare_parameter('roll_limit_rad', 0.20)
        self.declare_parameter('deadzone_rad', 0.025)
        self.declare_parameter('smoothing_alpha', 0.22)
        self.declare_parameter('max_yaw_rate_rad_s', 0.80)
        self.declare_parameter('max_pitch_rate_rad_s', 0.65)
        self.declare_parameter('max_roll_rate_rad_s', 0.45)
        self.declare_parameter('max_step_dt_sec', 0.10)
        self.declare_parameter('yaw_enabled', True)
        self.declare_parameter('pitch_enabled', True)
        self.declare_parameter('roll_enabled', False)
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('invert_pitch', False)
        self.declare_parameter('invert_roll', False)
        self.declare_parameter('yaw_joint_name', 'head_yaw_joint')
        self.declare_parameter('pitch_joint_name', 'head_pitch_joint')
        self.declare_parameter('roll_joint_name', 'head_roll_joint')

    def _read_parameters(self) -> None:
        def value(name):
            return self.get_parameter(name).value

        self._head_pose_topic = str(value('head_pose_topic'))
        self._active_topic = str(value('active_topic'))
        self._emergency_stop_topic = str(value('emergency_stop_topic'))
        self._debug_prefix = str(value('debug_topic_prefix')).rstrip('/')
        self._calibration_service = str(value('calibration_service'))
        self._reset_service = str(value('reset_service'))
        self._pose_timeout_sec = float(value('pose_timeout_sec'))
        self._active_timeout_sec = float(value('active_timeout_sec'))
        self._publish_rate_hz = float(value('publish_rate_hz'))
        self._status_period_sec = float(value('status_publish_period_sec'))
        self._trajectory_time_sec = float(value('trajectory_time_sec'))
        self._yaw_joint_name = str(value('yaw_joint_name'))
        self._pitch_joint_name = str(value('pitch_joint_name'))
        self._roll_joint_name = str(value('roll_joint_name'))
        self._limits = HeadLimits(
            yaw_scale=float(value('yaw_scale')),
            pitch_scale=float(value('pitch_scale')),
            roll_scale=float(value('roll_scale')),
            yaw_limit_rad=float(value('yaw_limit_rad')),
            pitch_limit_rad=float(value('pitch_limit_rad')),
            roll_limit_rad=float(value('roll_limit_rad')),
            deadzone_rad=float(value('deadzone_rad')),
            smoothing_alpha=float(value('smoothing_alpha')),
            max_yaw_rate_rad_s=float(value('max_yaw_rate_rad_s')),
            max_pitch_rate_rad_s=float(value('max_pitch_rate_rad_s')),
            max_roll_rate_rad_s=float(value('max_roll_rate_rad_s')),
            max_step_dt_sec=float(value('max_step_dt_sec')),
            yaw_enabled=bool(value('yaw_enabled')),
            pitch_enabled=bool(value('pitch_enabled')),
            roll_enabled=bool(value('roll_enabled')),
            invert_yaw=bool(value('invert_yaw')),
            invert_pitch=bool(value('invert_pitch')),
            invert_roll=bool(value('invert_roll')),
        )

    def _validate_parameters(self) -> None:
        self._limits.validate()
        positive = (
            self._pose_timeout_sec,
            self._active_timeout_sec,
            self._publish_rate_hz,
            self._status_period_sec,
            self._trajectory_time_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('head timing parameters must be finite and positive')
        if not _MIN_HEAD_PUBLISH_RATE_HZ <= self._publish_rate_hz <= _MAX_HEAD_PUBLISH_RATE_HZ:
            raise ValueError(
                'head publish_rate_hz must be between %.0f and %.0f for watchdog safety'
                % (_MIN_HEAD_PUBLISH_RATE_HZ, _MAX_HEAD_PUBLISH_RATE_HZ)
            )
        if self._status_period_sec > _MAX_HEAD_STATUS_PERIOD_SEC:
            raise ValueError('head status_publish_period_sec exceeds the diagnostics ceiling')
        for name, topic in (
            ('head_pose_topic', self._head_pose_topic),
            ('active_topic', self._active_topic),
            ('emergency_stop_topic', self._emergency_stop_topic),
            ('debug_topic_prefix', self._debug_prefix),
            ('calibration_service', self._calibration_service),
            ('reset_service', self._reset_service),
        ):
            if not topic.startswith('/'):
                raise ValueError('%s must be an absolute ROS name' % name)
        enabled_names = (
            (self._limits.yaw_enabled, self._yaw_joint_name),
            (self._limits.pitch_enabled, self._pitch_joint_name),
            (self._limits.roll_enabled, self._roll_joint_name),
        )
        if any(enabled and not name for enabled, name in enabled_names):
            raise ValueError('each enabled head axis requires a joint name')

    def _head_pose_callback(self, message: PoseStamped) -> None:
        now = time.monotonic()
        quaternion = (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        try:
            source = euler_from_quaternion(quaternion)
        except ValueError as exc:
            self._latest_pose = None
            self._latest_pose_error = 'pose_invalid:%s' % exc
            self._publish_status(now, force=True)
            return
        self._latest_pose = HeadPoseSample(
            quaternion=tuple(float(value) for value in quaternion),
            arrival=now,
        )
        self._latest_pose_error = 'pose_fresh'
        self._last_source_euler = source

    def _deadman_callback(self, message: Bool) -> None:
        previous = self._deadman_active if self._deadman_seen else None
        self._deadman_seen = True
        self._deadman_active = bool(message.data)
        self._deadman_arrival = time.monotonic()
        self._evaluate_and_publish(
            self._deadman_arrival,
            force_status=previous is None or previous != self._deadman_active,
        )

    def _kill_callback(self, message: Bool) -> None:
        previous = self._kill_active if self._kill_seen else None
        self._kill_seen = True
        self._kill_active = bool(message.data)
        self._evaluate_and_publish(
            time.monotonic(),
            force_status=previous is None or previous != self._kill_active,
        )

    def _calibrate_callback(self, _request, response):
        now = time.monotonic()
        pose = self._fresh_pose(now)
        if pose is None:
            response.success = False
            response.message = 'Fresh finite VR head pose is required: %s' % (
                self._latest_pose_error
            )
            self._evaluate_and_publish(now, force_status=True)
            return response
        self._controller.calibrate(pose.quaternion, now)
        response.success = True
        response.message = (
            'Neutral head orientation recorded; output remains dry-run and '
            'requires kill=false plus active deadman for diagnostic targets'
        )
        self._evaluate_and_publish(now, force_status=True)
        return response

    def _reset_callback(self, _request, response):
        now = time.monotonic()
        self._controller.reset_calibration(now)
        response.success = True
        response.message = 'Head neutral calibration cleared; debug command is zero'
        self._evaluate_and_publish(now, force_status=True)
        return response

    def _tick(self) -> None:
        self._evaluate_and_publish(time.monotonic())

    def _evaluate_and_publish(self, now: float, force_status: bool = False) -> None:
        reason, kill, deadman, watchdog = self._gate_state(now)
        pose = self._fresh_pose(now)
        if reason:
            decision = self._controller.safe_fallback(
                reason, now, self._last_source_euler
            )
        elif pose is None:
            decision = self._controller.safe_fallback(
                self._latest_pose_error, now, self._last_source_euler
            )
        else:
            decision = self._controller.update(pose.quaternion, now)
        self._publish_decision(decision)
        self._publish_status(
            now,
            decision=decision,
            kill=kill,
            deadman=deadman,
            watchdog=watchdog,
            force=force_status,
        )

    def _gate_state(self, now: float):
        if not self._kill_seen:
            return 'kill_unconfirmed', 'unconfirmed', self._deadman_state(now), 'blocked'
        if self._kill_active:
            return 'kill_active', 'active', self._deadman_state(now), 'blocked'
        deadman = self._deadman_state(now)
        if deadman != 'active':
            return 'deadman_%s' % deadman, 'clear', deadman, 'blocked'
        if self._latest_pose is None:
            return '', 'clear', deadman, 'missing'
        if now - self._latest_pose.arrival > self._pose_timeout_sec:
            self._latest_pose_error = 'pose_stale'
            return '', 'clear', deadman, 'stale'
        return '', 'clear', deadman, 'fresh'

    def _deadman_state(self, now: float) -> str:
        if not self._deadman_seen or self._deadman_arrival is None:
            return 'unconfirmed'
        if not self._deadman_active:
            return 'inactive'
        if now - self._deadman_arrival > self._active_timeout_sec:
            return 'stale'
        return 'active'

    def _fresh_pose(self, now: float) -> Optional[HeadPoseSample]:
        if self._latest_pose is None:
            return None
        if now - self._latest_pose.arrival > self._pose_timeout_sec:
            self._latest_pose_error = 'pose_stale'
            return None
        return self._latest_pose

    def _publish_decision(self, decision: HeadDecision) -> None:
        self._raw_publisher.publish(self._vector_message(decision.source_euler))
        self._relative_publisher.publish(
            self._vector_message(decision.relative_euler)
        )
        self._command_publisher.publish(
            self._vector_message(decision.command_euler)
        )
        names, positions = self._trajectory_values(decision.command_euler)
        if not names:
            return
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = _duration(self._trajectory_time_sec)
        trajectory.points = [point]
        self._trajectory_publisher.publish(trajectory)

    def _vector_message(self, values: Axes) -> Vector3Stamped:
        message = Vector3Stamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'yaw_pitch_roll_rad'
        message.vector.x = float(values[0])
        message.vector.y = float(values[1])
        message.vector.z = float(values[2])
        return message

    def _trajectory_values(self, values: Axes):
        axes = (
            (self._limits.yaw_enabled, self._yaw_joint_name, values[0]),
            (self._limits.pitch_enabled, self._pitch_joint_name, values[1]),
            (self._limits.roll_enabled, self._roll_joint_name, values[2]),
        )
        names = [name for enabled, name, _ in axes if enabled]
        positions = [position for enabled, _, position in axes if enabled]
        return names, positions

    def _publish_status(
        self,
        now: float,
        decision: Optional[HeadDecision] = None,
        kill: Optional[str] = None,
        deadman: Optional[str] = None,
        watchdog: Optional[str] = None,
        force: bool = False,
    ) -> None:
        if decision is None:
            decision = self._controller.safe_fallback(
                'status_initialization', now, self._last_source_euler
            )
        if kill is None or deadman is None or watchdog is None:
            _, kill, deadman, watchdog = self._gate_state(now)
        pose_age = 'n/a'
        if self._latest_pose is not None:
            pose_age = '%.3f' % max(0.0, now - self._latest_pose.arrival)
        command = decision.command_euler
        text = (
            'dry_run=true hardware_enabled=false kill=%s deadman=%s '
            'watchdog=%s calibrated=%s pose_age_sec=%s decision=%s '
            'allowed=%s command_ypr_rad=(%.4f,%.4f,%.4f) '
            'limits_ypr_rad=(%.4f,%.4f,%.4f) '
            'rates_ypr_rad_s=(%.4f,%.4f,%.4f) max_step_dt_sec=%.3f'
            % (
                kill,
                deadman,
                watchdog,
                self._controller.calibrated,
                pose_age,
                decision.reason,
                decision.allowed,
                command[0],
                command[1],
                command[2],
                self._limits.yaw_limit_rad,
                self._limits.pitch_limit_rad,
                self._limits.roll_limit_rad,
                self._limits.max_yaw_rate_rad_s,
                self._limits.max_pitch_rate_rad_s,
                self._limits.max_roll_rate_rad_s,
                self._limits.max_step_dt_sec,
            )
        )
        status_key = (
            kill,
            deadman,
            watchdog,
            self._controller.calibrated,
            decision.reason,
            decision.allowed,
            tuple(round(value, 4) for value in command),
        )
        if (
            not force
            and status_key == self._last_status_key
            and now - self._last_status_time < self._status_period_sec
        ):
            return
        self._last_status_key = status_key
        self._last_status_time = now
        message = String()
        message.data = text
        self._status_publisher.publish(message)
        self.get_logger().info(text)

    def stop(self) -> None:
        """Publish a final zero-valued diagnostic result before shutdown."""
        self._evaluate_and_publish(time.monotonic(), force_status=True)


def _duration(seconds: float) -> Duration:
    """Convert a positive floating-point duration to a ROS Duration message."""
    nanoseconds = int(round(float(seconds) * 1.0e9))
    duration = Duration()
    duration.sec = nanoseconds // 1_000_000_000
    duration.nanosec = nanoseconds % 1_000_000_000
    return duration


def main(args=None) -> None:
    """Run the standalone diagnostics-only head controller."""
    rclpy.init(args=args)
    node = HeadControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
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
