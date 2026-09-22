"""Read-only Unitree R1 low-state monitor for ROS diagnostics."""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
except ImportError as exc:  # pragma: no cover - depends on host SDK install
    ChannelFactoryInitialize = None
    ChannelSubscriber = None
    LowState_ = None
    SDK_IMPORT_ERROR = exc
else:
    SDK_IMPORT_ERROR = None


R1_JOINT_NAMES = (
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
    'waist_roll_joint',
    'waist_yaw_joint',
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
    'head_pitch_joint',
    'head_yaw_joint',
)

# The HG LowState array has reserved entries; these are the R1 motor slots
# documented by the SDK's r1_low_level_example.py.
R1_IDL_MOTOR_INDICES = (
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 13,
    15, 16, 17, 18, 19,
    22, 23, 24, 25, 26,
    29, 30,
)


def extract_joint_positions(state):
    """Extract the 26 R1 joint positions from one SDK LowState object."""
    return _extract_motor_values(state, 'q')


def extract_joint_feedback(state):
    """Extract finite position, velocity, and effort arrays for all R1 joints."""
    positions = _extract_motor_values(state, 'q')
    velocities = _extract_motor_values(state, 'dq')
    efforts = _extract_motor_values(state, 'tau_est')
    if positions is None or velocities is None or efforts is None:
        return None
    return positions, velocities, efforts


def _extract_motor_values(state, field):
    """Extract one finite scalar field using the configured R1 motor slots."""
    motors = list(getattr(state, 'motor_state', ()))
    if len(motors) <= max(R1_IDL_MOTOR_INDICES):
        return None
    try:
        values = tuple(
            float(getattr(motors[index], field))
            for index in R1_IDL_MOTOR_INDICES
        )
    except (AttributeError, TypeError, ValueError):
        return None
    if not all(_finite(value) for value in values):
        return None
    return values


def state_summary(state, sample_count, age_sec):
    """Return a compact status line without exposing raw SDK objects."""
    motors = list(getattr(state, 'motor_state', ()))
    enabled_modes = sum(
        1 for index in R1_IDL_MOTOR_INDICES
        if index < len(motors) and int(getattr(motors[index], 'mode', 0)) != 0
    )
    nonzero_motorstates = []
    for index in R1_IDL_MOTOR_INDICES:
        if index >= len(motors):
            continue
        value = int(getattr(motors[index], 'motorstate', 0))
        if value != 0:
            nonzero_motorstates.append((index, value))
    motorstate_text = 'none'
    if nonzero_motorstates:
        motorstate_text = ','.join(
            '%d:%d' % item for item in nonzero_motorstates
        )
    voltages = [
        float(getattr(motors[index], 'vol', 0.0))
        for index in R1_IDL_MOTOR_INDICES
        if index < len(motors) and _finite(float(getattr(motors[index], 'vol', 0.0)))
    ]
    voltage_text = 'n/a'
    if voltages:
        voltage_text = '%.1f..%.1fV' % (min(voltages), max(voltages))
    velocity_text = _maximum_absolute_text(
        _extract_motor_values(state, 'dq')
    )
    effort_text = _maximum_absolute_text(
        _extract_motor_values(state, 'tau_est')
    )
    temperatures = []
    for index in R1_IDL_MOTOR_INDICES:
        if index >= len(motors):
            continue
        try:
            readings = tuple(getattr(motors[index], 'temperature', ()))
        except TypeError:
            continue
        if len(readings) < 2:
            continue
        try:
            pair = tuple(int(value) for value in readings[:2])
        except (TypeError, ValueError):
            continue
        temperatures.extend(pair)
    temperature_text = 'n/a'
    if temperatures:
        temperature_text = '%d..%d' % (
            min(temperatures), max(temperatures)
        )
    remote = getattr(state, 'wireless_remote', ())
    remote_present = any(int(value) != 0 for value in remote)
    imu = getattr(state, 'imu_state', None)
    imu_temperature = getattr(imu, 'temperature', 'n/a')
    return (
        'lowstate=ok samples=%d age=%.3fs mode_machine=%s mode_pr=%s '
        'motors=%d enabled_modes=%d motorstate_nonzero=%d '
        'remote_nonzero=%s imu_temp=%s voltage=%s max_abs_dq=%s '
        'max_abs_tau_est=%s motor_temp=%s motorstate=%s writer=disabled'
        % (
            sample_count,
            age_sec,
            getattr(state, 'mode_machine', 'n/a'),
            getattr(state, 'mode_pr', 'n/a'),
            len(motors),
            enabled_modes,
            len(nonzero_motorstates),
            remote_present,
            imu_temperature,
            voltage_text,
            velocity_text,
            effort_text,
            temperature_text,
            motorstate_text,
        )
    )


class R1StateMonitor(Node):
    """Subscribe to R1 LowState and publish read-only ROS diagnostics."""

    def __init__(self):
        super().__init__('r1_state_monitor')
        self.declare_parameter('network_interface', 'enxb4b024be59fe')
        self.declare_parameter('lowstate_channel', 'rt/lf/lowstate')
        self.declare_parameter('joint_state_topic', '/r1/hardware/joint_states')
        self.declare_parameter('status_topic', '/r1/hardware/status')
        self.declare_parameter('state_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        if SDK_IMPORT_ERROR is not None:
            raise RuntimeError(
                'unitree_sdk2py is required for read-only state monitor: '
                f'{SDK_IMPORT_ERROR}'
            )

        interface = str(self.get_parameter('network_interface').value)
        channel = str(self.get_parameter('lowstate_channel').value)
        self._timeout = float(self.get_parameter('state_timeout_sec').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not interface or not channel or self._timeout <= 0.0 or rate <= 0.0:
            raise ValueError('state monitor interface, channel, timeout, and rate are required')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._joint_publisher = self.create_publisher(
            JointState, str(self.get_parameter('joint_state_topic').value), qos
        )
        self._status_publisher = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), qos
        )
        self._lock = threading.Lock()
        self._latest_state = None
        self._latest_arrival = None
        self._sample_count = 0

        # ChannelFactoryInitialize creates a DDS participant but does not write
        # any command channel.  This node only constructs a LowState subscriber.
        ChannelFactoryInitialize(0, interface)
        self._subscriber = ChannelSubscriber(channel, LowState_)
        self._subscriber.Init(self._state_callback, 10)
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'read-only R1 LowState monitor on {interface}:{channel}; '
            'no command publisher is configured'
        )

    def _state_callback(self, state):
        with self._lock:
            self._latest_state = state
            self._latest_arrival = time.monotonic()
            self._sample_count += 1

    def _publish(self):
        with self._lock:
            state = self._latest_state
            arrival = self._latest_arrival
            count = self._sample_count
        if state is None or arrival is None:
            self._publish_status('lowstate=missing')
            return
        age = time.monotonic() - arrival
        if age > self._timeout:
            self._publish_status('lowstate=stale age=%.3fs' % age)
            return
        feedback = extract_joint_feedback(state)
        if feedback is None:
            self._publish_status('lowstate=invalid_joint_array')
            return
        positions, velocities, efforts = feedback
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(R1_JOINT_NAMES)
        message.position = list(positions)
        message.velocity = list(velocities)
        message.effort = list(efforts)
        self._joint_publisher.publish(message)
        self._publish_status(state_summary(state, count, age))

    def _publish_status(self, text):
        message = String()
        message.data = text
        self._status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = R1StateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _finite(value):
    return value == value and abs(value) != float('inf')


def _maximum_absolute_text(values):
    if values is None:
        return 'n/a'
    return '%.3f' % max(abs(value) for value in values)
