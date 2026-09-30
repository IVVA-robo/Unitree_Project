"""ROS 2 node that turns validated UDP snapshots into teleoperation intents."""

import errno
import socket
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .protocol import (
    PacketError,
    TrackingAvailability,
    VRPacket,
    apply_deadzone,
    shape_translation_stick,
    is_newer_sequence,
    parse_packet,
    scale_stick_axis,
)
from .resilience import (
    DebouncedButton,
    EmergencyStopButton,
    ExhibitionSessionGate,
    PoseRecoveryFilter,
)


class VRBridgeNode(Node):
    """Receive Pico XR snapshots and expose safe, robot-independent ROS topics."""

    def __init__(self) -> None:
        super().__init__('vr_teleop_bridge')

        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('udp_port', 9090)
        self.declare_parameter('discovery_port', 9091)
        self.declare_parameter('allowed_source_ip', '')
        self.declare_parameter('tracking_frame', 'vr_tracking')
        self.declare_parameter('packet_timeout_sec', 0.25)
        self.declare_parameter('pause_on_packet_timeout', False)
        self.declare_parameter('deadzone', 0.15)
        # The commissioned Pico runtime reports full thumbstick travel as
        # approximately 0.25. Normalize that range before deadzone/mapping;
        # the final robot velocity remains bounded by the independent limits.
        self.declare_parameter('stick_input_scale', 4.0)
        self.declare_parameter('translation_axis_snap_ratio', 0.60)
        self.declare_parameter('max_forward_mps', 0.35)
        self.declare_parameter('max_lateral_mps', 0.25)
        self.declare_parameter('max_yaw_rps', 0.60)
        self.declare_parameter('forward_sign', 1.0)
        self.declare_parameter('lateral_sign', -1.0)
        self.declare_parameter('yaw_sign', -1.0)
        self.declare_parameter('max_position_m', 5.0)
        self.declare_parameter('max_datagram_bytes', 4096)
        self.declare_parameter('activation_mode', 'deadman')
        self.declare_parameter('tracking_grace_sec', 2.0)
        self.declare_parameter('recovery_blend_sec', 0.5)
        self.declare_parameter('button_debounce_sec', 0.05)
        self.declare_parameter('safety_profile', 'standard')

        self._bind_address = str(self.get_parameter('bind_address').value)
        self._udp_port = int(self.get_parameter('udp_port').value)
        self._discovery_port = int(self.get_parameter('discovery_port').value)
        self._allowed_source_ip = str(self.get_parameter('allowed_source_ip').value)
        self._tracking_frame = str(self.get_parameter('tracking_frame').value)
        self._packet_timeout_sec = float(
            self.get_parameter('packet_timeout_sec').value
        )
        self._pause_on_packet_timeout = bool(
            self.get_parameter('pause_on_packet_timeout').value
        )
        self._deadzone = float(self.get_parameter('deadzone').value)
        self._stick_input_scale = float(
            self.get_parameter('stick_input_scale').value
        )
        self._translation_axis_snap_ratio = float(
            self.get_parameter('translation_axis_snap_ratio').value
        )
        self._max_forward_mps = float(
            self.get_parameter('max_forward_mps').value
        )
        self._max_lateral_mps = float(
            self.get_parameter('max_lateral_mps').value
        )
        self._max_yaw_rps = float(self.get_parameter('max_yaw_rps').value)
        self._forward_sign = float(self.get_parameter('forward_sign').value)
        self._lateral_sign = float(self.get_parameter('lateral_sign').value)
        self._yaw_sign = float(self.get_parameter('yaw_sign').value)
        self._max_position_m = float(self.get_parameter('max_position_m').value)
        self._max_datagram_bytes = int(
            self.get_parameter('max_datagram_bytes').value
        )
        self._activation_mode = str(
            self.get_parameter('activation_mode').value
        ).strip().lower()
        self._tracking_grace_sec = float(
            self.get_parameter('tracking_grace_sec').value
        )
        self._recovery_blend_sec = float(
            self.get_parameter('recovery_blend_sec').value
        )
        self._button_debounce_sec = float(
            self.get_parameter('button_debounce_sec').value
        )
        self._safety_profile = str(
            self.get_parameter('safety_profile').value
        ).strip().lower()
        self._validate_parameters()

        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        session_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._left_pub = self.create_publisher(
            PoseStamped, '/vr/left_controller/pose', pose_qos
        )
        self._right_pub = self.create_publisher(
            PoseStamped, '/vr/right_controller/pose', pose_qos
        )
        self._head_pub = self.create_publisher(PoseStamped, '/vr/head/pose', pose_qos)
        self._joy_pub = self.create_publisher(Joy, '/vr/joy', pose_qos)
        self._velocity_pub = self.create_publisher(
            TwistStamped, '/vr/cmd_vel', command_qos
        )
        self._active_pub = self.create_publisher(
            Bool, '/vr/teleop/active', command_qos
        )
        self._locomotion_active_pub = self.create_publisher(
            Bool, '/vr/locomotion/active', command_qos
        )
        self._session_armed_pub = self.create_publisher(
            Bool, '/vr/teleop/session_armed', session_qos
        )
        self._status_pub = self.create_publisher(
            String, '/vr/teleop/status', command_qos
        )
        self._arms_neutral_pub = self.create_publisher(
            Bool, '/vr/actions/arms_neutral', command_qos
        )
        self._emergency_stop_pub = self.create_publisher(
            Bool, '/vr/actions/emergency_stop', session_qos
        )
        self._action_status_pub = self.create_publisher(
            String, '/vr/actions/status', session_qos
        )
        # The safety supervisor requests TRANSIENT_LOCAL durability.  Match it
        # here so a right-B emergency edge cannot be rejected by DDS QoS.
        self._kill_request_pub = self.create_publisher(
            Bool, '/r1/safety/kill_request', session_qos
        )

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self._socket.setblocking(False)
        self._socket.bind((self._bind_address, self._udp_port))
        self._discovery_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._discovery_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._discovery_socket.setblocking(False)
        self._discovery_socket.bind((self._bind_address, self._discovery_port))

        self._last_sequence: Optional[int] = None
        self._last_accepted_sequence: Optional[int] = None
        self._last_valid_monotonic: Optional[float] = None
        self._locked_source_ip: Optional[str] = None
        self._last_source_ip: Optional[str] = None
        self._active = False
        self._locomotion_active = False
        self._valid_packet_count = 0
        self._bad_packet_count = 0
        self._last_warning_monotonic = 0.0
        self._packet_outage_active = False
        self._last_packet: Optional[VRPacket] = None
        self._stick_motion_active = False
        self._session_gate = (
            ExhibitionSessionGate(self._tracking_grace_sec)
            if self._activation_mode == 'session_arm' else None
        )
        self._pose_recovery = (
            PoseRecoveryFilter(self._recovery_blend_sec)
            if self._activation_mode == 'session_arm' else None
        )
        self._held_poses = {}
        self._left_x = DebouncedButton(self._button_debounce_sec)
        self._right_b = EmergencyStopButton(self._button_debounce_sec)
        self._arms_neutral_active = False
        self._emergency_latched = False
        self._last_action = 'none'

        self.create_service(
            Trigger, '/vr/teleop/arm_session', self._arm_session
        )
        self.create_service(
            Trigger, '/vr/teleop/disarm_session', self._disarm_session
        )
        self.create_service(
            Trigger, '/vr/teleop/pause_session', self._pause_session
        )
        self.create_service(
            Trigger, '/vr/teleop/resume_session', self._resume_session
        )
        self.create_service(
            Trigger,
            '/vr/teleop/clear_emergency_stop',
            self._clear_emergency_stop,
        )
        self._publish_session_armed()
        self._publish_action_state()

        # Drain all queued datagrams often, but publish only the newest valid snapshot.
        self._receive_timer = self.create_timer(0.005, self._receive_pending)
        self._discovery_timer = self.create_timer(0.05, self._receive_discovery)
        self._watchdog_timer = self.create_timer(0.02, self._watchdog)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'Listening for Pico UDP v1 on {self._bind_address}:{self._udp_port}; '
            f'discovery on {self._bind_address}:{self._discovery_port}; '
            f'activation_mode={self._activation_mode}; '
            f'safety_profile={self._safety_profile}; '
            'robot output is intentionally not connected'
        )

    def _validate_parameters(self) -> None:
        if not 1 <= self._udp_port <= 65535:
            raise ValueError('udp_port must be in [1, 65535]')
        if not 1024 <= self._discovery_port <= 65535:
            raise ValueError('discovery_port must be in [1024, 65535]')
        if self._discovery_port == self._udp_port:
            raise ValueError('discovery_port must differ from udp_port')
        if not 0.05 <= self._packet_timeout_sec <= 5.0:
            raise ValueError('packet_timeout_sec must be in [0.05, 5.0]')
        if not 0.0 <= self._deadzone < 1.0:
            raise ValueError('deadzone must be in [0, 1)')
        if not 0.1 <= self._stick_input_scale <= 8.0:
            raise ValueError('stick_input_scale must be in [0.1, 8.0]')
        if not 0.0 <= self._translation_axis_snap_ratio <= 0.8:
            raise ValueError('translation_axis_snap_ratio must be in [0, 0.8]')
        if min(
            self._max_forward_mps,
            self._max_lateral_mps,
            self._max_yaw_rps,
            self._max_position_m,
        ) <= 0.0:
            raise ValueError('speed and position limits must be positive')
        if not 256 <= self._max_datagram_bytes <= 65507:
            raise ValueError('max_datagram_bytes must be in [256, 65507]')
        if self._activation_mode not in ('deadman', 'session_arm'):
            raise ValueError('activation_mode must be deadman or session_arm')
        if not 1.0 <= self._tracking_grace_sec <= 3.0:
            raise ValueError('tracking_grace_sec must be in [1, 3]')
        if not 0.1 <= self._recovery_blend_sec <= 2.0:
            raise ValueError('recovery_blend_sec must be in [0.1, 2.0]')
        if not 0.01 <= self._button_debounce_sec <= 0.25:
            raise ValueError('button_debounce_sec must be in [0.01, 0.25]')
        if self._safety_profile not in ('standard', 'exhibition'):
            raise ValueError('safety_profile must be standard or exhibition')

    def _receive_pending(self) -> None:
        newest_packet = None
        # A cap prevents an incoming flood from starving the ROS executor.
        for _ in range(64):
            try:
                payload, address = self._socket.recvfrom(self._max_datagram_bytes + 1)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                self.get_logger().error(f'UDP receive failed: {exc}')
                return

            if self._allowed_source_ip and address[0] != self._allowed_source_ip:
                self._warn_throttled(f'rejected UDP source {address[0]}')
                continue
            if self._locked_source_ip and address[0] != self._locked_source_ip:
                self._warn_throttled(
                    f'rejected competing UDP source {address[0]}'
                )
                continue
            if len(payload) > self._max_datagram_bytes:
                self._bad_packet_count += 1
                self._warn_throttled('rejected oversized UDP datagram')
                continue

            try:
                packet = parse_packet(payload, max_position_m=self._max_position_m)
                previous = (
                    newest_packet.sequence
                    if newest_packet is not None
                    else self._last_sequence
                )
                if not is_newer_sequence(packet.sequence, previous):
                    raise PacketError(
                        f'duplicate/out-of-order sequence {packet.sequence}'
                    )
            except PacketError as exc:
                self._bad_packet_count += 1
                self._warn_throttled(f'rejected packet from {address}: {exc}')
                continue

            if self._locked_source_ip is None:
                self._locked_source_ip = address[0]
            self._last_source_ip = address[0]
            # Inspect every accepted snapshot for face-button edges.  Pose and
            # stick streams may drop intermediate UDP frames, but an emergency
            # B press must not disappear merely because a newer frame arrived
            # in the same 5 ms drain cycle.
            self._handle_button_actions(packet, time.monotonic())
            newest_packet = packet

        if newest_packet is not None:
            now = time.monotonic()
            if self._packet_outage_active:
                self.get_logger().info('VR packets recovered; validating tracking')
                self._packet_outage_active = False
            self._last_valid_monotonic = now
            self._last_sequence = newest_packet.sequence
            self._last_accepted_sequence = newest_packet.sequence
            self._valid_packet_count += 1
            self._last_packet = newest_packet
            self._publish_packet(newest_packet, now, handle_actions=False)

    def _receive_discovery(self) -> None:
        """Answer LAN discovery probes without accepting any robot command."""
        for _ in range(16):
            try:
                payload, address = self._discovery_socket.recvfrom(256)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    return
                self.get_logger().error(f'UDP discovery receive failed: {exc}')
                return

            parts = payload.decode('ascii', errors='ignore').strip().split()
            if len(parts) != 3 or parts[:2] != ['R1_TELEOP_DISCOVER', 'v1']:
                continue
            nonce = parts[2]
            if not 8 <= len(nonce) <= 64 or not all(
                character.isalnum() or character in '-_' for character in nonce
            ):
                continue
            response_prefix = 'R1_TELEOP_ENDPOINT'
            response = f'{response_prefix} v1 {self._udp_port} {nonce}'.encode(
                'ascii'
            )
            try:
                self._discovery_socket.sendto(response, address)
            except OSError as exc:
                self.get_logger().warning(
                    f'UDP discovery response to {address[0]} failed: {exc}'
                )

    def _publish_packet(
        self, packet: VRPacket, now=None, handle_actions: bool = True
    ) -> None:
        action_now = time.monotonic() if now is None else now
        if handle_actions:
            self._handle_button_actions(packet, action_now)
        if self._emergency_latched:
            self._publish_emergency_hold()
            return
        if self._activation_mode == 'session_arm':
            self._publish_session_packet(
                packet, action_now
            )
            return

        stamp = self.get_clock().now().to_msg()
        self._publish_poses(stamp, {
            'left': packet.left, 'right': packet.right, 'head': packet.head,
        })
        self._joy_pub.publish(self._joy_message(stamp, packet, packet.deadman))
        self._set_active(packet.deadman)
        self._set_locomotion_active(packet.deadman)
        self._report_stick_state(packet, packet.deadman)
        self._velocity_pub.publish(
            self._velocity_message(stamp, packet, packet.deadman)
        )

    def _publish_session_packet(self, packet: VRPacket, now: float) -> None:
        """Publish session-authorized input or a per-device safe hold."""
        tracking = packet.tracking
        self._session_gate.observe(
            tracking, now, self._locomotion_controls_neutral(packet)
        )
        raw = {'left': packet.left, 'right': packet.right, 'head': packet.head}
        # A calibration pause keeps the writer's session latch armed.  Freeze
        # every pose here so new controller/HMD movement cannot reach that
        # writer while the calibration helper reads the captured neutral pose.
        self._held_poses = self._pose_recovery.update(
            raw, self._session_gate.pose_tracking, now
        )
        decision = self._session_gate.decision(
            now, snapshot_ready=self._pose_recovery.ready
        )
        stamp = self.get_clock().now().to_msg()
        if self._pose_recovery.ready:
            self._publish_poses(stamp, self._held_poses)
        self._joy_pub.publish(
            self._joy_message(
                stamp,
                packet,
                decision.control_active,
                zero_controls=not decision.locomotion_active,
            )
        )
        self._velocity_pub.publish(
            self._velocity_message(stamp, packet, decision.locomotion_active)
        )
        self._report_stick_state(packet, decision.locomotion_active)
        self._set_active(decision.control_active)
        self._set_locomotion_active(decision.locomotion_active)

    def _handle_button_actions(self, packet: VRPacket, now: float) -> None:
        """Apply the exhibition X/B mapping before any motion is published."""
        neutral, neutral_changed = self._left_x.update(
            packet.buttons.left_x, now
        )
        if neutral_changed:
            self._arms_neutral_active = neutral
            self._arms_neutral_pub.publish(Bool(data=neutral))
            if neutral:
                self._last_action = 'arms_reset_to_neutral'
                self._action_status_pub.publish(
                    String(data='arms_reset_to_neutral')
                )
                self.get_logger().info('arms_reset_to_neutral')
            else:
                self._last_action = 'arms_vr_follow_resumed'
                self._action_status_pub.publish(
                    String(data='arms_vr_follow_resumed')
                )
                self.get_logger().info('arms VR follow resumed after left X')

        if self._right_b.update(packet.buttons.right_b, now):
            self._latch_emergency_stop()

    def _latch_emergency_stop(self) -> None:
        """Latch explicit right-B emergency stop and request the central KILL."""
        if self._emergency_latched:
            return
        self._emergency_latched = True
        self._last_action = 'emergency_stop_right_b'
        if self._session_gate is not None:
            self._session_gate.disarm()
            self._publish_session_armed()
        self._publish_emergency_hold()
        self._arms_neutral_pub.publish(Bool(data=True))
        self._emergency_stop_pub.publish(Bool(data=True))
        self._action_status_pub.publish(String(data=self._last_action))
        self._kill_request_pub.publish(Bool(data=True))
        self.get_logger().error(
            'emergency_stop_right_b: locomotion zeroed, control disarmed, '
            'central KILL requested'
        )

    def _publish_emergency_hold(self) -> None:
        """Refresh zero/hold output while an explicit emergency is latched."""
        stamp = self.get_clock().now().to_msg()
        self._velocity_pub.publish(self._velocity_message(stamp, None))
        self._joy_pub.publish(self._joy_message(stamp, None, False))
        self._set_locomotion_active(False)
        self._set_active(False)

    def _publish_action_state(self) -> None:
        neutral = self._arms_neutral_active or self._emergency_latched
        self._arms_neutral_pub.publish(Bool(data=neutral))
        self._emergency_stop_pub.publish(Bool(data=self._emergency_latched))
        self._action_status_pub.publish(String(data=self._last_action))

    def _locomotion_controls_neutral(self, packet: VRPacket) -> bool:
        """Check only the three axes used by the high-level locomotion map."""
        return all(
            apply_deadzone(
                scale_stick_axis(value, self._stick_input_scale),
                self._deadzone,
            ) == 0.0
            for value in (
                packet.left_stick[0],
                packet.left_stick[1],
                packet.right_stick[0],
            )
        )

    def _publish_poses(self, stamp, poses) -> None:
        self._left_pub.publish(self._pose_message(stamp, poses['left']))
        self._right_pub.publish(self._pose_message(stamp, poses['right']))
        self._head_pub.publish(self._pose_message(stamp, poses['head']))

    def _joy_message(
        self, stamp, packet: Optional[VRPacket], active: bool,
        zero_controls: bool = False,
    ) -> Joy:
        message = Joy()
        message.header.stamp = stamp
        message.header.frame_id = self._tracking_frame
        if packet is None or zero_controls:
            message.axes = [0.0] * 6
            message.buttons = [1 if active else 0, 0, 0]
            return message
        message.axes = [
            float(packet.left_stick[0]),
            float(packet.left_stick[1]),
            float(packet.right_stick[0]),
            float(packet.right_stick[1]),
            float(packet.left_trigger),
            float(packet.right_trigger),
        ]
        message.buttons = [
            1 if active else 0,
            1 if packet.left_trigger >= 0.5 else 0,
            1 if packet.right_trigger >= 0.5 else 0,
        ]
        return message

    def _pose_message(self, stamp, source) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self._tracking_frame
        message.pose.position.x = source.position[0]
        message.pose.position.y = source.position[1]
        message.pose.position.z = source.position[2]
        message.pose.orientation.x = source.orientation[0]
        message.pose.orientation.y = source.orientation[1]
        message.pose.orientation.z = source.orientation[2]
        message.pose.orientation.w = source.orientation[3]
        return message

    def _velocity_message(
        self, stamp, packet: Optional[VRPacket], active=False
    ) -> TwistStamped:
        message = TwistStamped()
        message.header.stamp = stamp
        message.header.frame_id = 'base_link'
        if packet is None or not active:
            return message

        shaped_x, shaped_y = shape_translation_stick(
            packet.left_stick[0], packet.left_stick[1],
            self._translation_axis_snap_ratio,
        )
        left_x = apply_deadzone(
            scale_stick_axis(shaped_x, self._stick_input_scale),
            self._deadzone,
        )
        left_y = apply_deadzone(
            scale_stick_axis(shaped_y, self._stick_input_scale),
            self._deadzone,
        )
        right_x = apply_deadzone(
            scale_stick_axis(packet.right_stick[0], self._stick_input_scale),
            self._deadzone,
        )
        message.twist.linear.x = (
            self._forward_sign * self._max_forward_mps * left_y
        )
        message.twist.linear.y = (
            self._lateral_sign * self._max_lateral_mps * left_x
        )
        message.twist.angular.z = self._yaw_sign * self._max_yaw_rps * right_x
        return message

    def _report_stick_state(self, packet: VRPacket, active: bool) -> None:
        """Log joystick motion edges without flooding the 72 Hz control log."""
        raw = (
            packet.left_stick[0], packet.left_stick[1],
            packet.right_stick[0], packet.right_stick[1],
        )
        moving = any(
            apply_deadzone(
                scale_stick_axis(value, self._stick_input_scale),
                self._deadzone,
            ) != 0.0
            for value in raw[:3]
        )
        if moving == self._stick_motion_active:
            return
        self._stick_motion_active = moving
        if not moving:
            self.get_logger().info('VR locomotion sticks returned to neutral')
            return
        mapped = self._velocity_message(
            self.get_clock().now().to_msg(), packet, active
        ).twist
        self.get_logger().info(
            'VR locomotion stick motion: '
            f'left=({raw[0]:.3f},{raw[1]:.3f}) '
            f'right=({raw[2]:.3f},{raw[3]:.3f}) '
            f'input_scale={self._stick_input_scale:.2f} '
            f'authorized={str(active).lower()} '
            f'cmd=({mapped.linear.x:.3f},{mapped.linear.y:.3f},'
            f'{mapped.angular.z:.3f})'
        )

    def _watchdog(self) -> None:
        if self._last_valid_monotonic is None:
            return
        age = time.monotonic() - self._last_valid_monotonic
        if age <= self._packet_timeout_sec:
            return

        if self._emergency_latched:
            self._publish_emergency_hold()
            return

        if self._activation_mode == 'session_arm':
            self._publish_session_hold(time.monotonic(), age)
            # A restarted Unity app may restart its sequence counter. Keep the
            # locked source while armed so another LAN sender cannot take over.
            self._last_sequence = None
            if not self._session_gate.armed:
                self._locked_source_ip = None
            return

        # Publish the stop edge once. Every downstream hardware adapter must also
        # implement its own timeout; it must never rely only on this process.
        if self._active:
            self._velocity_pub.publish(
                self._velocity_message(self.get_clock().now().to_msg(), None)
            )
            self._set_active(False)
            self._set_locomotion_active(False)
            self.get_logger().warning(
                f'VR packet timeout ({age:.3f} s): velocity forced to zero'
            )
        # Permit a restarted Unity application to begin its sequence again.
        self._last_sequence = None
        self._locked_source_ip = None

    def _publish_session_hold(self, now: float, packet_age: float) -> None:
        """Keep fresh arm hold frames while forcing locomotion inactive."""
        if self._pause_on_packet_timeout and self._session_gate.armed:
            # Keep the writer's hold authority, but require explicit RUN to
            # resume. Reconnecting USB must not restart a moving robot.
            self._session_gate.pause()
        self._session_gate.observe_packet_timeout(now)
        unavailable = TrackingAvailability.unavailable(reported=True)
        self._held_poses = self._pose_recovery.update({}, unavailable, now)
        decision = self._session_gate.decision(
            now, snapshot_ready=self._pose_recovery.ready
        )
        stamp = self.get_clock().now().to_msg()
        if self._pose_recovery.ready:
            self._publish_poses(stamp, self._held_poses)
        self._joy_pub.publish(
            self._joy_message(stamp, None, decision.control_active)
        )
        self._velocity_pub.publish(self._velocity_message(stamp, None))
        self._set_active(decision.control_active)
        self._set_locomotion_active(False)
        # Heartbeat while unchanged: downstream arm hold must not be mistaken
        # for a released session. A bridge crash still makes it expire.
        self._active_pub.publish(Bool(data=decision.control_active))
        self._locomotion_active_pub.publish(Bool(data=False))
        if not self._packet_outage_active:
            self._packet_outage_active = True
            self.get_logger().warning(
                f'VR packet timeout ({packet_age:.3f} s): arms held, '
                'locomotion forced to zero, waiting for reconnect'
            )

    def _set_active(self, active: bool) -> None:
        if active == self._active:
            return
        self._active = active
        self._active_pub.publish(Bool(data=active))
        self.get_logger().info(f'teleop active: {active}')

    def _set_locomotion_active(self, active: bool) -> None:
        if active == self._locomotion_active:
            return
        self._locomotion_active = active
        self._locomotion_active_pub.publish(Bool(data=active))
        self.get_logger().info(f'locomotion input active: {active}')

    def _publish_session_armed(self) -> None:
        armed = self._session_gate is not None and self._session_gate.armed
        self._session_armed_pub.publish(Bool(data=armed))

    def _arm_session(self, request, response):
        del request
        if self._activation_mode != 'session_arm':
            response.success = False
            response.message = 'session arm unavailable: activation_mode=deadman'
            return response
        if self._emergency_latched:
            response.success = False
            response.message = (
                'session arm blocked: use explicit clear_emergency_stop/RUN'
            )
            return response
        now = time.monotonic()
        fresh = (
            self._last_valid_monotonic is not None
            and now - self._last_valid_monotonic <= self._packet_timeout_sec
        )
        if not fresh:
            response.success = False
            response.message = 'session arm blocked: fresh VR packet required'
            return response
        success, reason = self._session_gate.arm(self._pose_recovery.ready)
        response.success = success
        response.message = reason
        self._publish_session_armed()
        decision = self._session_gate.decision(now, self._pose_recovery.ready)
        self._set_active(decision.control_active)
        self._set_locomotion_active(decision.locomotion_active)
        return response

    def _disarm_session(self, request, response):
        del request
        if self._activation_mode != 'session_arm':
            response.success = False
            response.message = 'session disarm unavailable: activation_mode=deadman'
            return response
        self._session_gate.disarm()
        self._publish_session_armed()
        stamp = self.get_clock().now().to_msg()
        self._velocity_pub.publish(self._velocity_message(stamp, None))
        self._joy_pub.publish(self._joy_message(stamp, None, False))
        self._set_locomotion_active(False)
        self._set_active(False)
        response.success = True
        response.message = 'session_disarmed'
        return response

    def _pause_session(self, request, response):
        """Hold arms and zero locomotion without dropping writer authority."""
        del request
        if self._activation_mode != 'session_arm':
            response.success = False
            response.message = 'session pause unavailable: activation_mode=deadman'
            return response
        success, reason = self._session_gate.pause()
        response.success = success
        response.message = reason
        if not success:
            return response
        stamp = self.get_clock().now().to_msg()
        self._velocity_pub.publish(self._velocity_message(stamp, None))
        self._joy_pub.publish(self._joy_message(stamp, None, False))
        self._set_locomotion_active(False)
        self._set_active(False)
        # The writer's independent session latch deliberately stays true.
        self._publish_session_armed()
        return response

    def _resume_session(self, request, response):
        """Resume calibration pause from fresh tracking and neutral sticks."""
        del request
        if self._activation_mode != 'session_arm':
            response.success = False
            response.message = 'session resume unavailable: activation_mode=deadman'
            return response
        if self._emergency_latched:
            response.success = False
            response.message = (
                'session resume blocked: use explicit clear_emergency_stop/RUN'
            )
            return response
        now = time.monotonic()
        fresh = (
            self._last_valid_monotonic is not None
            and now - self._last_valid_monotonic <= self._packet_timeout_sec
        )
        if not fresh:
            response.success = False
            response.message = 'session resume blocked: fresh VR packet required'
            return response
        success, reason = self._session_gate.resume(self._pose_recovery.ready)
        response.success = success
        response.message = reason
        self._publish_session_armed()
        decision = self._session_gate.decision(now, self._pose_recovery.ready)
        self._set_active(decision.control_active)
        self._set_locomotion_active(decision.locomotion_active)
        return response

    def _clear_emergency_stop(self, request, response):
        """Clear the VR latch only from an explicit, ready RUN/re-arm action."""
        del request
        if not self._emergency_latched:
            response.success = True
            response.message = 'emergency_stop_not_latched'
            return response
        if self._activation_mode != 'session_arm':
            response.success = False
            response.message = 'clear requires exhibition session_arm mode'
            return response
        if self._right_b.active:
            response.success = False
            response.message = 'clear blocked: release right controller B first'
            return response
        now = time.monotonic()
        fresh = (
            self._last_valid_monotonic is not None
            and now - self._last_valid_monotonic <= self._packet_timeout_sec
        )
        if not fresh:
            response.success = False
            response.message = 'clear blocked: fresh VR packet required'
            return response
        success, reason = self._session_gate.arm(self._pose_recovery.ready)
        if not success:
            response.success = False
            response.message = f'clear blocked: {reason}'
            return response
        self._emergency_latched = False
        self._last_action = 'emergency_stop_cleared'
        self._publish_session_armed()
        self._publish_action_state()
        decision = self._session_gate.decision(now, self._pose_recovery.ready)
        self._set_active(decision.control_active)
        self._set_locomotion_active(decision.locomotion_active)
        response.success = True
        response.message = 'emergency_stop_cleared; run prepare still required'
        self.get_logger().warning(
            'VR emergency latch cleared by explicit service; downstream KILL '
            'remains latched until the reviewed RUN prepare path succeeds'
        )
        return response

    def _publish_status(self) -> None:
        self._active_pub.publish(Bool(data=self._active))
        self._locomotion_active_pub.publish(
            Bool(data=self._locomotion_active)
        )
        self._publish_session_armed()
        self._publish_action_state()
        now = time.monotonic()
        if self._last_valid_monotonic is None:
            age_text = 'n/a'
            fresh = False
        else:
            age = max(0.0, now - self._last_valid_monotonic)
            age_text = f'{age:.3f}'
            fresh = age <= self._packet_timeout_sec
        source = self._last_source_ip or 'none'
        sequence = (
            str(self._last_accepted_sequence)
            if self._last_accepted_sequence is not None
            else 'none'
        )
        status = (
            f'source={source} valid_packets={self._valid_packet_count} '
            f'bad_packets={self._bad_packet_count} last_sequence={sequence} '
            f'packet_age_sec={age_text} fresh={str(fresh).lower()} '
            f'active={str(self._active).lower()} '
            f'locomotion_active={str(self._locomotion_active).lower()} '
            f'activation_mode={self._activation_mode} '
            f'safety_profile={self._safety_profile} '
            f'arms_neutral={str(self._arms_neutral_active).lower()} '
            f'emergency_stop={str(self._emergency_latched).lower()} '
            f'last_action={self._last_action}'
        )
        if self._last_packet is not None:
            tracking = self._last_packet.tracking
            status += (
                f' packet_deadman={str(self._last_packet.deadman).lower()} '
                f'left_stick=({self._last_packet.left_stick[0]:.3f},'
                f'{self._last_packet.left_stick[1]:.3f}) '
                f'right_stick=({self._last_packet.right_stick[0]:.3f},'
                f'{self._last_packet.right_stick[1]:.3f}) '
                f'left_x={str(self._last_packet.buttons.left_x).lower()} '
                f'right_b={str(self._last_packet.buttons.right_b).lower()} '
                f'stick_input_scale={self._stick_input_scale:.2f} '
                f'left_tracked={str(tracking.left).lower()} '
                f'right_tracked={str(tracking.right).lower()} '
                f'head_tracked={str(tracking.head).lower()}'
            )
        if self._session_gate is not None:
            decision = self._session_gate.decision(
                now, self._pose_recovery.ready
            )
            missing = ','.join(decision.missing) or 'none'
            status += (
                f' session_armed={str(self._session_gate.armed).lower()} '
                f'tracking_phase={decision.phase} missing={missing}'
            )
        self._status_pub.publish(String(data=status))

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_monotonic >= 1.0:
            self.get_logger().warning(
                f'{message}; rejected total={self._bad_packet_count}'
            )
            self._last_warning_monotonic = now

    def destroy_node(self) -> bool:
        try:
            if self._session_gate is not None:
                self._session_gate.disarm()
            self._publish_session_armed()
            self._socket.close()
            self._discovery_socket.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VRBridgeNode()
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
                node._velocity_pub.publish(  # Last best-effort process-local stop edge.
                    node._velocity_message(node.get_clock().now().to_msg(), None)
                )
                node._set_locomotion_active(False)
                if node._session_gate is not None:
                    node._session_gate.disarm()
                node._publish_session_armed()
                node._set_active(False)
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
