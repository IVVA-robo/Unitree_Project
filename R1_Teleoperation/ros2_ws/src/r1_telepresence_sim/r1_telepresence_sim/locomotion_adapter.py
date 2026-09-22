"""
Simulation-only locomotion safety adapter.

The local R1 URDF has no verified walking/gait controller.  This node therefore
keeps a standard Twist interface and publishes only to the simulator's
``/r1/sim/cmd_vel`` topic.  It never opens a hardware transport and refuses
``hardware_enabled=true`` unless an explicit future hardware adapter replaces it.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class SimulationLocomotionAdapter(Node):
    """Gate and rate-limit velocity intent for Gazebo only."""

    def __init__(self):
        super().__init__('r1_sim_locomotion_adapter')
        self.declare_parameter('hardware_enabled', False)
        self.declare_parameter('input_topic', '/r1/sim/safe_cmd_vel')
        self.declare_parameter('output_topic', '/r1/sim/cmd_vel')
        self.declare_parameter('active_topic', '/vr/teleop/active')
        self.declare_parameter('cmd_timeout_sec', 0.25)
        self.declare_parameter('max_forward_mps', 0.35)
        self.declare_parameter('max_lateral_mps', 0.25)
        self.declare_parameter('max_yaw_rps', 0.60)
        self.declare_parameter('accel_mps2', 0.8)
        self.declare_parameter('yaw_accel_rps2', 1.5)

        if bool(self.get_parameter('hardware_enabled').value):
            raise RuntimeError(
                'hardware_enabled=true is forbidden in the simulation adapter; '
                'use a separately reviewed R1 hardware adapter'
            )
        self._input_topic = str(self.get_parameter('input_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._active_topic = str(self.get_parameter('active_topic').value)
        self._timeout = float(self.get_parameter('cmd_timeout_sec').value)
        self._limits = (
            float(self.get_parameter('max_forward_mps').value),
            float(self.get_parameter('max_lateral_mps').value),
            float(self.get_parameter('max_yaw_rps').value),
        )
        self._accel = float(self.get_parameter('accel_mps2').value)
        self._yaw_accel = float(self.get_parameter('yaw_accel_rps2').value)
        if self._timeout <= 0.0 or min(*self._limits, self._accel, self._yaw_accel) <= 0.0:
            raise ValueError('adapter limits and timeout must be positive')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Twist, self._output_topic, qos)
        self.create_subscription(Twist, self._input_topic, self._cmd_callback, qos)
        self.create_subscription(Bool, self._active_topic, self._active_callback, qos)
        self._timer = self.create_timer(0.02, self._tick)
        self._active = False
        self._last_cmd = Twist()
        self._published = Twist()
        self._last_input = time.monotonic()
        self._last_tick = time.monotonic()
        # Keep the zero command watchdog active on every tick, but report a
        # stop reason only when it changes.  Without this guard a stale input
        # produces one INFO line every 20 ms and hides useful diagnostics.
        self._last_stop_reason = None
        self._publish_zero('startup')
        self.get_logger().info(
            f'simulation-only adapter: {self._input_topic} -> {self._output_topic}; '
            'hardware_enabled=false'
        )

    def _active_callback(self, msg):
        was_active = self._active
        self._active = bool(msg.data)
        if not self._active:
            self._publish_zero('deadman' if was_active else 'inactive')

    def _cmd_callback(self, msg):
        self._last_input = time.monotonic()
        if not self._active:
            return
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(float(v)) for v in values):
            self._publish_zero('nonfinite')
            return
        self._last_cmd = Twist()
        self._last_cmd.linear.x = _clamp(msg.linear.x, self._limits[0])
        self._last_cmd.linear.y = _clamp(msg.linear.y, self._limits[1])
        self._last_cmd.angular.z = _clamp(msg.angular.z, self._limits[2])

    def _tick(self):
        now = time.monotonic()
        dt = min(0.1, max(1.0e-4, now - self._last_tick))
        self._last_tick = now
        if not self._active or now - self._last_input > self._timeout:
            self._publish_zero('timeout' if self._active else 'inactive')
            return
        output = Twist()
        output.linear.x = _slew(
            self._last_cmd.linear.x, self._published.linear.x, self._accel * dt
        )
        output.linear.y = _slew(
            self._last_cmd.linear.y, self._published.linear.y, self._accel * dt
        )
        output.angular.z = _slew(
            self._last_cmd.angular.z,
            self._published.angular.z,
            self._yaw_accel * dt,
        )
        self._published = output
        # A valid command clears the edge-triggered stop diagnostic.  The next
        # timeout/deadman event will therefore be reported once again.
        self._last_stop_reason = None
        self._publisher.publish(output)

    def _publish_zero(self, reason):
        self._last_cmd = Twist()
        self._published = Twist()
        self._publisher.publish(Twist())
        if reason not in ('inactive',) and reason != self._last_stop_reason:
            self.get_logger().info(f'locomotion stopped: {reason}')
        self._last_stop_reason = reason

    def stop(self):
        self._publish_zero('shutdown')


def _clamp(value, limit):
    return max(-limit, min(limit, float(value)))


def _slew(target, current, step):
    delta = _clamp(target - current, step)
    return current + delta


def main(args=None):
    rclpy.init(args=args)
    node = SimulationLocomotionAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Humble may report an invalid wait-set after a launch-level shutdown.
        # Do not hide real runtime failures while the ROS context is active.
        if rclpy.ok():
            raise
    finally:
        # A launch supervisor may deliver SIGINT while cleanup is running.
        # Treat that second edge as an ordinary shutdown instead of emitting a
        # traceback after the safety zero has already been published.
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
