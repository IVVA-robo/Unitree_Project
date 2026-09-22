"""Read-only network, VR, and environment checks for R1 teleoperation."""

import argparse
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
from std_msgs.msg import Bool, String

from .environment import ActuationPolicy


def vr_pose_qos():
    """Return the QoS which is compatible with the VR pose publisher."""
    return qos_profile_sensor_data


class Observation(Node):
    """Collect bounded, passive samples from the local ROS graph."""

    def __init__(self):
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
    observer = Observation()
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
    print(
        'No Unitree SDK writer, stand command, stiffness command, or '
        'locomotion request was created.'
    )
    return 2 if warnings else 0


if __name__ == '__main__':  # pragma: no cover - console entry point
    raise SystemExit(main())
