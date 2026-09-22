"""Dry-run-only boundary for a future Unitree R1 hardware transport."""

import time

from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory

from .safety import SafetyGate, SafetyLimits


class R1HardwareAdapter(Node):
    """
    Validate ROS commands without opening a Unitree command channel.

    The node intentionally refuses ``hardware_enabled=true`` and
    ``dry_run=false``.  This prevents a partially reviewed prototype from
    writing ``rt/lowcmd`` or invoking the R1 locomotion service.
    """

    def __init__(self):
        super().__init__('r1_hardware_adapter')
        self.declare_parameter('hardware_enabled', False)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('active_topic', '/vr/teleop/active')
        self.declare_parameter(
            'arm_input_topic', '/arm_trajectory_controller/joint_trajectory'
        )
        self.declare_parameter('velocity_input_topic', '/vr/safe_cmd_vel')
        self.declare_parameter('debug_topic_prefix', '/r1_hardware_adapter/debug')
        self.declare_parameter('active_timeout_sec', 1.5)
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('max_forward_mps', 0.35)
        self.declare_parameter('max_lateral_mps', 0.25)
        self.declare_parameter('max_yaw_rps', 0.60)
        self.declare_parameter('max_joint_step_rad', 0.15)
        self.declare_parameter('max_arm_joints', 10)

        hardware_enabled = bool(self.get_parameter('hardware_enabled').value)
        dry_run = bool(self.get_parameter('dry_run').value)
        if hardware_enabled or not dry_run:
            raise RuntimeError(
                'r1_hardware_adapter is a dry-run scaffold; hardware output is '
                'not implemented and must remain hardware_enabled=false, dry_run=true'
            )

        prefix = str(self.get_parameter('debug_topic_prefix').value).rstrip('/')
        if not prefix.startswith('/'):
            raise ValueError('debug_topic_prefix must be an absolute topic')
        limits = SafetyLimits(
            active_timeout_sec=float(
                self.get_parameter('active_timeout_sec').value
            ),
            command_timeout_sec=float(
                self.get_parameter('command_timeout_sec').value
            ),
            max_forward_mps=float(self.get_parameter('max_forward_mps').value),
            max_lateral_mps=float(self.get_parameter('max_lateral_mps').value),
            max_yaw_rps=float(self.get_parameter('max_yaw_rps').value),
            max_joint_step_rad=float(
                self.get_parameter('max_joint_step_rad').value
            ),
            max_arm_joints=int(self.get_parameter('max_arm_joints').value),
        )
        self._gate = SafetyGate(limits)
        self._active_seen = False
        self._last_status = ''
        self._last_status_time = 0.0
        self._last_velocity = (0.0, 0.0, 0.0)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._arm_debug = self.create_publisher(
            JointTrajectory, f'{prefix}/arm_trajectory', qos
        )
        self._velocity_debug = self.create_publisher(
            Twist, f'{prefix}/cmd_vel', qos
        )
        self._status = self.create_publisher(String, f'{prefix}/status', qos)
        self.create_subscription(
            Bool,
            str(self.get_parameter('active_topic').value),
            self._active_callback,
            qos,
        )
        self.create_subscription(
            JointTrajectory,
            str(self.get_parameter('arm_input_topic').value),
            self._arm_callback,
            qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('velocity_input_topic').value),
            self._velocity_callback,
            qos,
        )
        self._timer = self.create_timer(0.05, self._watchdog_tick)
        self.get_logger().warning(
            'dry-run hardware adapter active: no Unitree DDS writer is configured'
        )

    def _active_callback(self, message):
        active = bool(message.data)
        transitioned = not self._active_seen or active != self._gate.active_flag
        self._active_seen = True
        self._gate.set_active(active)
        if not active and transitioned:
            self._publish_zero_velocity('deadman_released')
        if transitioned:
            self._publish_status('deadman=%s' % active)

    def _arm_callback(self, message):
        if not message.points:
            self._publish_status('arm_rejected:empty_trajectory')
            return
        positions = message.points[0].positions
        if len(message.joint_names) != len(positions):
            self._publish_status('arm_rejected:joint_name_count')
            return
        if len(set(message.joint_names)) != len(message.joint_names):
            self._publish_status('arm_rejected:duplicate_joint_name')
            return
        decision = self._gate.arm(positions)
        if not decision.accepted:
            self._publish_status('arm_rejected:%s' % decision.reason)
            return
        debug = JointTrajectory()
        debug.joint_names = list(message.joint_names)
        point = message.points[0]
        point.positions = list(decision.positions)
        debug.points = [point]
        self._arm_debug.publish(debug)
        self._publish_status('arm_debug:%s' % decision.reason)

    def _velocity_callback(self, message):
        decision = self._gate.velocity(
            message.linear.x,
            message.linear.y,
            message.angular.z,
        )
        if not decision.accepted:
            self._publish_zero_velocity(decision.reason)
            return
        self._last_velocity = (
            decision.linear_x,
            decision.linear_y,
            decision.angular_z,
        )
        self._publish_velocity(*self._last_velocity)
        self._publish_status('velocity_debug:%s' % decision.reason)

    def _watchdog_tick(self):
        if self._gate.watchdog_expired():
            if self._last_velocity != (0.0, 0.0, 0.0):
                self._publish_zero_velocity('watchdog')

    def _publish_velocity(self, linear_x, linear_y, angular_z):
        message = Twist()
        message.linear.x = float(linear_x)
        message.linear.y = float(linear_y)
        message.angular.z = float(angular_z)
        self._velocity_debug.publish(message)

    def _publish_zero_velocity(self, reason):
        self._last_velocity = (0.0, 0.0, 0.0)
        self._publish_velocity(0.0, 0.0, 0.0)
        self._publish_status('velocity_zero:%s' % reason)

    def _publish_status(self, message):
        now = time.monotonic()
        if message == self._last_status and now - self._last_status_time < 0.5:
            return
        self._last_status = message
        self._last_status_time = now
        status = String()
        status.data = 'dry_run=true hardware_enabled=false %s' % message
        self._status.publish(status)
        self.get_logger().info(status.data)

    def stop(self):
        """Publish a debug-only zero before shutdown."""
        self._publish_zero_velocity('shutdown')


def main(args=None):
    rclpy.init(args=args)
    node = R1HardwareAdapter()
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
