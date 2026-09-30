"""
ROS wrapper for the isolated dry-run locomotion safety controller.

The node subscribes to VR ``TwistStamped`` intent, a deadman heartbeat, and a
kill-switch topic.  It publishes only explicitly named debug topics.  There is
no Unitree command client, DDS writer, or physical ``/cmd_vel`` publisher in
this file.
"""

import math
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .locomotion import (
    DryRunLocomotionController,
    LocomotionProfile,
    LocomotionSafetyConfig,
    default_profiles,
    format_status,
    require_dry_run,
)


# The kill subscriber deliberately requests a transient-local sample.  A
# supervisor must offer the same durability to latch its current kill state
# for a controller that starts later.  If no compatible publisher exists, the
# controller remains fail-closed in ``awaiting_explicit_clear``.
KILL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# The watchdog needs a bounded scheduler period.  A lower rate would leave a
# stale velocity visible for too long; a higher one brings no safety benefit
# for this diagnostics-only path.
_MIN_PUBLISH_RATE_HZ = 10.0
_MAX_PUBLISH_RATE_HZ = 100.0
_MAX_STATUS_RATE_HZ = 20.0


class R1LocomotionDryRun(Node):
    """Calculate bounded high-level locomotion commands for diagnostics only."""

    def __init__(self):
        super().__init__('r1_locomotion_dry_run')
        self.declare_parameter('hardware_enabled', False)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('input_topic', '/vr/cmd_vel')
        self.declare_parameter('active_topic', '/vr/locomotion/active')
        self.declare_parameter('emergency_stop_topic', '/r1/safety/kill')
        self.declare_parameter('mode_topic', '/r1/locomotion/mode')
        self.declare_parameter(
            'reset_emergency_stop_service',
            '/r1/locomotion_dry_run/reset_emergency_stop',
        )
        self.declare_parameter(
            'debug_topic_prefix', '/r1/locomotion_dry_run/debug'
        )
        self.declare_parameter('mode', 'slow-safe')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('status_rate_hz', 5.0)
        self.declare_parameter('deadman_timeout_sec', 1.50)
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('max_step_dt_sec', 0.10)

        defaults = default_profiles()
        for key, profile in (
            ('slow_safe', defaults['slow-safe']),
            ('normal', defaults['normal']),
            ('exhibition', defaults['exhibition']),
        ):
            self._declare_profile_parameters(key, profile)

        require_dry_run(
            self.get_parameter('hardware_enabled').value,
            self.get_parameter('dry_run').value,
        )
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        status_rate = float(self.get_parameter('status_rate_hz').value)
        if not all(math.isfinite(value) and value > 0.0
                   for value in (publish_rate, status_rate)):
            raise ValueError(
                'publish_rate_hz and status_rate_hz must be finite and positive'
            )
        if not _MIN_PUBLISH_RATE_HZ <= publish_rate <= _MAX_PUBLISH_RATE_HZ:
            raise ValueError(
                'publish_rate_hz must be between %.0f and %.0f for watchdog safety'
                % (_MIN_PUBLISH_RATE_HZ, _MAX_PUBLISH_RATE_HZ)
            )
        if status_rate > _MAX_STATUS_RATE_HZ:
            raise ValueError(
                'status_rate_hz exceeds the immutable diagnostics ceiling'
            )

        prefix = str(self.get_parameter('debug_topic_prefix').value).rstrip('/')
        if not prefix.startswith('/') or not prefix:
            raise ValueError('debug_topic_prefix must be an absolute topic')
        mode = str(self.get_parameter('mode').value).strip().lower()
        profiles = {
            'slow-safe': self._profile_from_parameters('slow_safe', 'slow-safe'),
            'normal': self._profile_from_parameters('normal', 'normal'),
            'exhibition': self._profile_from_parameters('exhibition', 'exhibition'),
        }
        config = LocomotionSafetyConfig(
            deadman_timeout_sec=float(
                self.get_parameter('deadman_timeout_sec').value
            ),
            command_timeout_sec=float(
                self.get_parameter('command_timeout_sec').value
            ),
            max_step_dt_sec=float(self.get_parameter('max_step_dt_sec').value),
        )
        self._controller = DryRunLocomotionController(
            config=config,
            profiles=profiles,
            mode=mode,
        )
        self._status_period_sec = 1.0 / status_rate
        self._last_status_time = 0.0
        self._last_status_text = ''

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._debug_velocity_publisher = self.create_publisher(
            TwistStamped, f'{prefix}/cmd_vel', qos
        )
        self._status_publisher = self.create_publisher(String, f'{prefix}/status', qos)
        self._kill_publisher = self.create_publisher(
            Bool, f'{prefix}/kill', KILL_QOS
        )
        self._deadman_publisher = self.create_publisher(
            Bool, f'{prefix}/deadman_active', qos
        )
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter('input_topic').value),
            self._velocity_callback,
            qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('active_topic').value),
            self._deadman_callback,
            qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('emergency_stop_topic').value),
            self._emergency_stop_callback,
            KILL_QOS,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mode_topic').value),
            self._mode_callback,
            qos,
        )
        self.create_service(
            Trigger,
            str(self.get_parameter('reset_emergency_stop_service').value),
            self._reset_emergency_stop_callback,
        )
        self._timer = self.create_timer(1.0 / publish_rate, self._tick)

        # Publish the startup interlock immediately so dashboards do not infer
        # a missing message means an enabled controller.
        self._publish_snapshot(time.monotonic(), force_status=True)
        self.get_logger().warning(
            'locomotion dry-run active: debug-only output; waiting for an '
            'explicit false on %s before deadman input can be accepted'
            % self.get_parameter('emergency_stop_topic').value
        )

    def _declare_profile_parameters(self, prefix, profile):
        """Declare numeric ROS parameters for one named safety profile."""
        self.declare_parameter(f'{prefix}.max_forward_mps', profile.max_forward_mps)
        self.declare_parameter(f'{prefix}.max_lateral_mps', profile.max_lateral_mps)
        self.declare_parameter(f'{prefix}.max_yaw_rps', profile.max_yaw_rps)
        self.declare_parameter(
            f'{prefix}.acceleration_mps2', profile.acceleration_mps2
        )
        self.declare_parameter(
            f'{prefix}.deceleration_mps2', profile.deceleration_mps2
        )
        self.declare_parameter(
            f'{prefix}.yaw_acceleration_rps2', profile.yaw_acceleration_rps2
        )
        self.declare_parameter(
            f'{prefix}.yaw_deceleration_rps2', profile.yaw_deceleration_rps2
        )

    def _profile_from_parameters(self, prefix, name):
        """Read one validated profile from the ROS parameter namespace."""
        return LocomotionProfile(
            name=name,
            max_forward_mps=float(
                self.get_parameter(f'{prefix}.max_forward_mps').value
            ),
            max_lateral_mps=float(
                self.get_parameter(f'{prefix}.max_lateral_mps').value
            ),
            max_yaw_rps=float(
                self.get_parameter(f'{prefix}.max_yaw_rps').value
            ),
            acceleration_mps2=float(
                self.get_parameter(f'{prefix}.acceleration_mps2').value
            ),
            deceleration_mps2=float(
                self.get_parameter(f'{prefix}.deceleration_mps2').value
            ),
            yaw_acceleration_rps2=float(
                self.get_parameter(f'{prefix}.yaw_acceleration_rps2').value
            ),
            yaw_deceleration_rps2=float(
                self.get_parameter(f'{prefix}.yaw_deceleration_rps2').value
            ),
        )

    def _velocity_callback(self, message):
        now = time.monotonic()
        decision = self._controller.receive_velocity(
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.angular.z,
            now,
        )
        if not decision.accepted:
            self._publish_snapshot(now, force_status=True)

    def _deadman_callback(self, message):
        now = time.monotonic()
        self._controller.set_deadman(bool(message.data), now)
        self._publish_snapshot(now, force_status=True)

    def _emergency_stop_callback(self, message):
        now = time.monotonic()
        self._controller.set_emergency_stop(bool(message.data))
        # A kill edge publishes zero immediately instead of waiting for the
        # periodic timer.  A cleared signal does not clear a later latched kill.
        self._publish_snapshot(now, force_status=True)

    def _mode_callback(self, message):
        now = time.monotonic()
        mode = str(message.data).strip().lower()
        if self._controller.set_mode(mode):
            self._publish_snapshot(now, force_status=True)
            return
        self.get_logger().warning('ignored unsupported locomotion mode: %s' % mode)
        self._publish_snapshot(now, force_status=True)

    def _reset_emergency_stop_callback(self, request, response):
        """Reset a latched kill after the kill source explicitly reports clear."""
        del request
        success, reason = self._controller.reset_emergency_stop()
        response.success = success
        response.message = reason
        self._publish_snapshot(time.monotonic(), force_status=True)
        return response

    def _tick(self):
        self._publish_snapshot(time.monotonic())

    def _publish_snapshot(self, now, force_status=False):
        """Publish debug-only velocity and compact safety diagnostics."""
        output = self._controller.step(now)
        velocity = TwistStamped()
        velocity.header.stamp = self.get_clock().now().to_msg()
        velocity.header.frame_id = 'base_link'
        velocity.twist.linear.x = output.velocity[0]
        velocity.twist.linear.y = output.velocity[1]
        velocity.twist.angular.z = output.velocity[2]
        self._debug_velocity_publisher.publish(velocity)
        self._kill_publisher.publish(Bool(data=output.kill != 'clear'))
        self._deadman_publisher.publish(Bool(data=output.deadman == 'active'))

        profile = self._controller.profile
        text = (
            '%s limits_mps_rps=(%.3f,%.3f,%.3f) '
            'ramp_mps2_rps2=(%.3f,%.3f,%.3f,%.3f)'
            % (
                format_status(output),
                profile.max_forward_mps,
                profile.max_lateral_mps,
                profile.max_yaw_rps,
                profile.acceleration_mps2,
                profile.deceleration_mps2,
                profile.yaw_acceleration_rps2,
                profile.yaw_deceleration_rps2,
            )
        )
        changed = text != self._last_status_text
        if (
            force_status
            or changed
            or now - self._last_status_time >= self._status_period_sec
        ):
            self._status_publisher.publish(String(data=text))
            self._last_status_text = text
            self._last_status_time = now
            if changed:
                self.get_logger().info(text)

    def stop(self):
        """Publish one final debug zero during a normal ROS shutdown."""
        self._controller.set_emergency_stop(True)
        self._publish_snapshot(time.monotonic(), force_status=True)


def main(args=None):
    """Run the dry-run locomotion diagnostics node."""
    rclpy.init(args=args)
    node = R1LocomotionDryRun()
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
