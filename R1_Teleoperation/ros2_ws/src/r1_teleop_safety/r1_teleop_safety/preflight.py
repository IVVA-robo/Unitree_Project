"""Read-only network, VR, and environment checks for R1 teleoperation."""

import argparse
import math
import statistics
import subprocess
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory

from .environment import ActuationPolicy


def vr_pose_qos():
    """Return the QoS which is compatible with the VR pose publisher."""
    return qos_profile_sensor_data


class Observation(Node):
    """Collect bounded, passive samples from the local ROS graph."""

    def __init__(self, require_prepare_signals=False):
        super().__init__('r1_teleop_preflight')
        command_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.head_times = []
        self.command_times = []
        self.head_transport_latencies_ms = []
        self.command_transport_latencies_ms = []
        self.last_active = None
        self.last_kill = None
        self.last_active_arrival = None
        self.last_kill_arrival = None
        self.last_head_debug_status = None
        self.last_head_debug_status_arrival = None
        self.last_locomotion_debug_status = None
        self.last_locomotion_debug_status_arrival = None
        self.last_head_command = None
        self.last_arm_command = None
        self.last_locomotion_debug_command = None
        self.head_seed_samples = []
        self.head_seed_failure = ''
        # The bridge publishes tracking poses as best-effort sensor data.  A
        # reliable subscription is QoS-incompatible with that publisher and
        # silently receives no HMD samples, which would make preflight report
        # a false VR failure.
        self.create_subscription(
            PoseStamped,
            '/vr/head/pose',
            self._head,
            vr_pose_qos(),
        )
        self.create_subscription(
            TwistStamped,
            '/vr/cmd_vel',
            self._command,
            10,
        )
        self.create_subscription(
            Bool,
            '/vr/teleop/active',
            self._active,
            command_qos,
        )
        self.create_subscription(
            Bool,
            '/r1/safety/kill',
            self._kill,
            latch_qos,
        )
        self.create_subscription(
            String,
            '/r1_hardware_adapter/debug/head/status',
            self._head_debug_status,
            command_qos,
        )
        self.create_subscription(
            String,
            '/r1/locomotion_dry_run/debug/status',
            self._locomotion_debug_status,
            command_qos,
        )
        if require_prepare_signals:
            # These are the same passive observations which r1-robot-prepare
            # previously collected through six separate ROS CLI processes.
            # One participant removes duplicate DDS discovery without calling
            # a service, clearing KILL or constructing a Unitree SDK writer.
            self.create_subscription(
                JointTrajectory,
                '/r1_hardware_adapter/debug/head/joint_trajectory',
                self._head_command,
                command_qos,
            )
            self.create_subscription(
                JointTrajectory,
                '/r1_kinematics_control/debug/arm_trajectory',
                self._arm_command,
                command_qos,
            )
            self.create_subscription(
                TwistStamped,
                '/r1/locomotion_dry_run/debug/cmd_vel',
                self._locomotion_debug_command,
                command_qos,
            )
            self.create_subscription(
                JointState,
                '/r1/sdk/joint_states',
                self._joint_state,
                command_qos,
            )

    def _head(self, message):
        self.head_times.append(time.monotonic())
        self._append_local_transport_latency(
            message,
            self.head_transport_latencies_ms,
        )

    def _command(self, message):
        self.command_times.append(time.monotonic())
        self._append_local_transport_latency(
            message,
            self.command_transport_latencies_ms,
        )

    def _active(self, message):
        self.last_active = bool(message.data)
        self.last_active_arrival = time.monotonic()

    def _kill(self, message):
        self.last_kill = bool(message.data)
        self.last_kill_arrival = time.monotonic()

    def _head_debug_status(self, message):
        self.last_head_debug_status = str(message.data)
        self.last_head_debug_status_arrival = time.monotonic()

    def _locomotion_debug_status(self, message):
        self.last_locomotion_debug_status = str(message.data)
        self.last_locomotion_debug_status_arrival = time.monotonic()

    def _head_command(self, message):
        self.last_head_command = message

    def _arm_command(self, message):
        self.last_arm_command = message

    def _locomotion_debug_command(self, message):
        self.last_locomotion_debug_command = message

    def _joint_state(self, message):
        if self.head_seed_failure or len(self.head_seed_samples) >= 5:
            return
        if (
            len(message.name) != len(message.position)
            or len(message.name) != len(message.velocity)
            or len(set(message.name)) != len(message.name)
        ):
            self.head_seed_failure = (
                'malformed name/position/velocity arrays'
            )
            return
        position = dict(zip(message.name, message.position))
        velocity = dict(zip(message.name, message.velocity))
        required = ('head_yaw_joint', 'head_pitch_joint')
        if any(name not in position or name not in velocity for name in required):
            self.head_seed_failure = 'head joints are missing from JointState'
            return
        values = (
            position['head_yaw_joint'],
            position['head_pitch_joint'],
            velocity['head_yaw_joint'],
            velocity['head_pitch_joint'],
        )
        if not all(math.isfinite(value) for value in values):
            self.head_seed_failure = 'head q/dq contains NaN or Inf'
            return
        self.head_seed_samples.append(values)

    def _append_local_transport_latency(self, message, values):
        """Append bridge-to-observer age when both nodes share a ROS clock."""
        stamp = message.header.stamp
        stamp_nanoseconds = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_nanoseconds <= 0:
            return
        age_nanoseconds = (
            self.get_clock().now().nanoseconds - stamp_nanoseconds
        )
        # A future/very old stamp usually means simulated or unsynchronised
        # clocks. Do not report that as an implausible transport latency.
        if 0 <= age_nanoseconds <= 10_000_000_000:
            values.append(age_nanoseconds / 1_000_000.0)


def _probe_network(interface, robot_ip):
    """Return passive link and ICMP probe messages without opening SDK DDS."""
    result = []
    link = subprocess.run(
        ('ip', '-br', 'link', 'show', 'dev', interface),
        capture_output=True,
        text=True,
        check=False,
    )
    if link.returncode:
        result.append(('FAIL', f'network interface {interface} is absent'))
        return result
    link_text = link.stdout.strip()
    link_level = 'OK'
    if 'NO-CARRIER' in link_text or ' DOWN ' in f' {link_text} ':
        link_level = 'WARN'
    result.append((link_level, link_text))
    ping = subprocess.run(
        ('ping', '-I', interface, '-c', '1', '-W', '1', robot_ip),
        capture_output=True,
        text=True,
        check=False,
    )
    state = 'OK' if ping.returncode == 0 else 'WARN'
    result.append((state, f'robot reachability {robot_ip}: {state.lower()}'))
    return result


def _rate_text(values):
    """Summarize packet count and approximate frequency for an input stream."""
    if not values:
        return 'no samples'
    if len(values) == 1:
        return 'one sample'
    intervals = [right - left for left, right in zip(values, values[1:])]
    interval = statistics.median(intervals)
    if interval <= 0.0:
        return f'{len(values)} samples, invalid timestamps'
    return f'{len(values)} samples, {1.0 / interval:.1f} Hz median'


def _latency_text(values):
    """Summarize local bridge-to-observer latency, not end-to-end delay."""
    if not values:
        return 'unavailable (missing stamps or unsynchronised clocks)'
    return '%.1f ms median, %.1f ms maximum (%d samples)' % (
        statistics.median(values),
        max(values),
        len(values),
    )


def _age_text(arrival, now):
    """Return a bounded age string for a locally observed ROS message."""
    if arrival is None:
        return 'unavailable'
    return '%.3f s ago' % max(0.0, float(now) - float(arrival))


def _evaluate_head_seed(samples, mode):
    """Apply the existing five-sample physical head seed policy."""
    if len(samples) < 5:
        return False, '[FAIL] fewer than five fresh physical head samples arrived'
    yaw, pitch, yaw_velocity, pitch_velocity = samples[-1]
    feedback_margin = 0.01
    if mode == 'probe':
        yaw_limit = 2.0071
        pitch_limit = 0.6283
    elif mode == 'auto-center':
        yaw_limit = 2.0071 + feedback_margin
        pitch_limit = 0.6283 + feedback_margin
    else:
        yaw_limit = 0.35
        pitch_limit = 0.25
    if any(
        abs(sample[0]) > yaw_limit or abs(sample[1]) > pitch_limit
        for sample in samples
    ):
        return False, (
            '[FAIL] physical head is outside the accepted seed envelope: '
            f'yaw={yaw:.6f} pitch={pitch:.6f}; '
            f'limits yaw={yaw_limit:.4f} pitch={pitch_limit:.4f} rad'
        )
    if any(
        abs(sample[2]) > 0.05 or abs(sample[3]) > 0.05
        for sample in samples
    ):
        return False, (
            '[FAIL] physical head is not stationary enough to seed: '
            f'yaw_dq={yaw_velocity:.6f} pitch_dq={pitch_velocity:.6f}; '
            'limit=0.05 rad/s'
        )
    if (
        max(sample[0] for sample in samples)
        - min(sample[0] for sample in samples) > 0.01
        or max(sample[1] for sample in samples)
        - min(sample[1] for sample in samples) > 0.01
    ):
        return False, '[FAIL] physical head position changed across the seed samples'
    outside_tracking_window = abs(yaw) > 0.35 or abs(pitch) > 0.25
    if mode == 'auto-center' and outside_tracking_window:
        return True, (
            '[AUTO_CENTER] stable physical head will be centered automatically '
            'before arms and locomotion are enabled: '
            f'yaw={yaw:.6f} pitch={pitch:.6f} '
            f'yaw_dq={yaw_velocity:.6f} pitch_dq={pitch_velocity:.6f}'
        )
    label = 'ownership micro-probe' if mode == 'probe' else 'normal tracking'
    return True, (
        '[OK] physical head seed is stable and accepted for '
        f'{label}: yaw={yaw:.6f} pitch={pitch:.6f} '
        f'yaw_dq={yaw_velocity:.6f} pitch_dq={pitch_velocity:.6f}'
    )


def main(argv=None):
    """Run read-only preflight and return a warning-aware process status."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='enxb4b024be59fe')
    parser.add_argument('--robot-ip', default='192.168.123.161')
    parser.add_argument('--observe-seconds', type=float, default=2.0)
    parser.add_argument(
        '--max-local-transport-latency-ms',
        type=float,
        default=100.0,
    )
    parser.add_argument('--require-vr', action='store_true')
    parser.add_argument(
        '--require-kill-clear',
        action='store_true',
        help=(
            'fail unless the retained software kill state is explicitly '
            'false; used by the prepare commissioning gate'
        ),
    )
    parser.add_argument(
        '--require-debug-controllers',
        action='store_true',
        help=(
            'fail unless fresh head and locomotion dry-run status messages '
            'were observed; used by the prepare commissioning gate'
        ),
    )
    parser.add_argument(
        '--require-prepare-signals',
        action='store_true',
        help=(
            'require the fresh head, arm, locomotion, deadman and physical '
            'head-seed observations needed immediately before prepare'
        ),
    )
    parser.add_argument(
        '--head-seed-mode',
        choices=('normal', 'probe', 'auto-center'),
        default='normal',
    )
    parser.add_argument(
        '--locomotion-mode',
        choices=('slow-safe', 'normal', 'exhibition'),
        default='slow-safe',
    )
    arguments = parser.parse_args(argv)
    if not 0.1 <= arguments.observe_seconds <= 10.0:
        parser.error('--observe-seconds must be between 0.1 and 10 seconds')
    if arguments.max_local_transport_latency_ms <= 0.0:
        parser.error('--max-local-transport-latency-ms must be positive')

    warnings = False
    try:
        policy = ActuationPolicy.from_environment()
    except ValueError as exc:
        print(f'[FAIL] invalid safety environment: {exc}')
        return 1
    print('R1 teleoperation preflight (READ-ONLY)')
    print(f'[OK] {policy.summary()}')
    print(f'[OK] locomotion_mode={arguments.locomotion_mode}')
    network_results = _probe_network(arguments.interface, arguments.robot_ip)
    for level, message in network_results:
        print(f'[{level}] {message}')
        warnings = warnings or level == 'WARN'

    rclpy.init(args=None)
    observer = Observation(arguments.require_prepare_signals)
    deadline = time.monotonic() + arguments.observe_seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
    finally:
        observer.destroy_node()
        rclpy.shutdown()

    observation_finished = time.monotonic()
    for name, samples in (
        ('VR head pose', observer.head_times),
        ('VR locomotion command', observer.command_times),
    ):
        text = _rate_text(samples)
        if samples:
            print(f'[OK] {name}: {text}')
        else:
            print(f'[WARN] {name}: {text}')
            warnings = True
    for name, samples in (
        (
            'VR head pose local ROS transport latency',
            observer.head_transport_latencies_ms,
        ),
        ('VR locomotion local ROS transport latency',
         observer.command_transport_latencies_ms),
    ):
        text = _latency_text(samples)
        if not samples:
            print(f'[WARN] {name}: {text}')
            warnings = True
            continue
        median_latency = statistics.median(samples)
        if median_latency > arguments.max_local_transport_latency_ms:
            print(f'[WARN] {name}: {text}')
            warnings = True
        else:
            print(f'[OK] {name}: {text}')
    if observer.last_active is None:
        print('[WARN] deadman state was not observed')
        warnings = True
    else:
        print(
            '[OK] deadman_active=%s age=%s' % (
                str(observer.last_active).lower(),
                _age_text(observer.last_active_arrival, observation_finished),
            )
        )
    if observer.last_kill is None:
        print('[WARN] safety kill state was not observed')
        warnings = True
    else:
        print(
            '[OK] safety_kill=%s age=%s' % (
                str(observer.last_kill).lower(),
                _age_text(observer.last_kill_arrival, observation_finished),
            )
        )
    debug_statuses = (
        (
            'head_dry_run',
            observer.last_head_debug_status,
            observer.last_head_debug_status_arrival,
        ),
        (
            'locomotion_dry_run',
            observer.last_locomotion_debug_status,
            observer.last_locomotion_debug_status_arrival,
        ),
    )
    missing_debug_status = False
    for name, status, arrival in debug_statuses:
        if status is None:
            print(f'[WARN] {name} status was not observed')
            warnings = True
            missing_debug_status = True
            continue
        print('[OK] %s status age=%s: %s' % (
            name,
            _age_text(arrival, observation_finished),
            status,
        ))
    if arguments.require_vr and not observer.head_times:
        print('[FAIL] --require-vr was set, but no VR head pose arrived')
        return 1
    if arguments.require_kill_clear and observer.last_kill is not False:
        print(
            '[FAIL] --require-kill-clear was set, but the software kill is '
            'not explicitly false'
        )
        return 1
    if arguments.require_debug_controllers and missing_debug_status:
        print(
            '[FAIL] --require-debug-controllers was set, but one or more '
            'diagnostic controller status messages were missing'
        )
        return 1
    if arguments.require_prepare_signals:
        prepare_failures = []
        if observer.last_active is not True:
            prepare_failures.append(
                'control session is not active; no fresh true deadman sample'
            )
        head_status = (observer.last_head_debug_status or '').lower()
        if 'calibrated=true' not in head_status:
            prepare_failures.append('head neutral is not calibrated')
        if observer.last_head_command is None:
            prepare_failures.append('fresh bounded head command was not observed')
        if observer.last_locomotion_debug_command is None:
            prepare_failures.append(
                'fresh bounded locomotion command was not observed'
            )
        if observer.last_arm_command is None:
            prepare_failures.append('fresh arm IK command was not observed')
        elif 'left_shoulder_pitch_joint' not in observer.last_arm_command.joint_names:
            prepare_failures.append(
                'arm IK command does not contain the R1 arm joint set'
            )
        if observer.head_seed_failure:
            prepare_failures.append(
                f'physical head feedback rejected: {observer.head_seed_failure}'
            )
            seed_ok = False
            seed_message = ''
        else:
            seed_ok, seed_message = _evaluate_head_seed(
                observer.head_seed_samples,
                arguments.head_seed_mode,
            )
            if not seed_ok:
                prepare_failures.append(seed_message.removeprefix('[FAIL] '))
        if prepare_failures:
            for failure in prepare_failures:
                print(f'[FAIL] prepare observation: {failure}')
            return 1
        print(seed_message)
        print(
            '[OK] control prepare signals observed concurrently with one '
            'read-only DDS participant'
        )
    print(
        'No Unitree SDK writer, stand command, stiffness command, or '
        'locomotion request was created.'
    )
    return 2 if warnings else 0


if __name__ == '__main__':  # pragma: no cover - console entry point
    raise SystemExit(main())
