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

from .protocol import (
    PacketError,
    VRPacket,
    apply_deadzone,
    is_newer_sequence,
    parse_packet,
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
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('max_forward_mps', 0.35)
        self.declare_parameter('max_lateral_mps', 0.25)
        self.declare_parameter('max_yaw_rps', 0.60)
        self.declare_parameter('forward_sign', 1.0)
        self.declare_parameter('lateral_sign', -1.0)
        self.declare_parameter('yaw_sign', -1.0)
        self.declare_parameter('max_position_m', 5.0)
        self.declare_parameter('max_datagram_bytes', 4096)

        self._bind_address = str(self.get_parameter('bind_address').value)
        self._udp_port = int(self.get_parameter('udp_port').value)
        self._discovery_port = int(self.get_parameter('discovery_port').value)
        self._allowed_source_ip = str(self.get_parameter('allowed_source_ip').value)
        self._tracking_frame = str(self.get_parameter('tracking_frame').value)
        self._packet_timeout_sec = float(
            self.get_parameter('packet_timeout_sec').value
        )
        self._deadzone = float(self.get_parameter('deadzone').value)
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
        self._validate_parameters()

        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
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
        self._status_pub = self.create_publisher(
            String, '/vr/teleop/status', command_qos
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
        self._valid_packet_count = 0
        self._bad_packet_count = 0
        self._last_warning_monotonic = 0.0

        # Drain all queued datagrams often, but publish only the newest valid snapshot.
        self._receive_timer = self.create_timer(0.005, self._receive_pending)
        self._discovery_timer = self.create_timer(0.05, self._receive_discovery)
        self._watchdog_timer = self.create_timer(0.02, self._watchdog)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'Listening for Pico UDP v1 on {self._bind_address}:{self._udp_port}; '
            f'discovery on {self._bind_address}:{self._discovery_port}; '
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
        if min(
            self._max_forward_mps,
            self._max_lateral_mps,
            self._max_yaw_rps,
            self._max_position_m,
        ) <= 0.0:
            raise ValueError('speed and position limits must be positive')
        if not 256 <= self._max_datagram_bytes <= 65507:
            raise ValueError('max_datagram_bytes must be in [256, 65507]')

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
            newest_packet = packet

        if newest_packet is not None:
            self._last_valid_monotonic = time.monotonic()
            self._last_sequence = newest_packet.sequence
            self._last_accepted_sequence = newest_packet.sequence
            self._valid_packet_count += 1
            self._publish_packet(newest_packet)

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

    def _publish_packet(self, packet: VRPacket) -> None:
        stamp = self.get_clock().now().to_msg()
        self._left_pub.publish(self._pose_message(stamp, packet.left))
        self._right_pub.publish(self._pose_message(stamp, packet.right))
        self._head_pub.publish(self._pose_message(stamp, packet.head))

        joy = Joy()
        joy.header.stamp = stamp
        joy.header.frame_id = self._tracking_frame
        joy.axes = [
            float(packet.left_stick[0]),
            float(packet.left_stick[1]),
            float(packet.right_stick[0]),
            float(packet.right_stick[1]),
            float(packet.left_trigger),
            float(packet.right_trigger),
        ]
        joy.buttons = [
            1 if packet.deadman else 0,
            1 if packet.left_trigger >= 0.5 else 0,
            1 if packet.right_trigger >= 0.5 else 0,
        ]
        self._joy_pub.publish(joy)

        self._set_active(packet.deadman)
        self._velocity_pub.publish(self._velocity_message(stamp, packet))

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

    def _velocity_message(self, stamp, packet: Optional[VRPacket]) -> TwistStamped:
        message = TwistStamped()
        message.header.stamp = stamp
        message.header.frame_id = 'base_link'
        if packet is None or not packet.deadman:
            return message

        left_x = apply_deadzone(packet.left_stick[0], self._deadzone)
        left_y = apply_deadzone(packet.left_stick[1], self._deadzone)
        right_x = apply_deadzone(packet.right_stick[0], self._deadzone)
        message.twist.linear.x = (
            self._forward_sign * self._max_forward_mps * left_y
        )
        message.twist.linear.y = (
            self._lateral_sign * self._max_lateral_mps * left_x
        )
        message.twist.angular.z = self._yaw_sign * self._max_yaw_rps * right_x
        return message

    def _watchdog(self) -> None:
        if self._last_valid_monotonic is None:
            return
        age = time.monotonic() - self._last_valid_monotonic
        if age <= self._packet_timeout_sec:
            return

        # Publish the stop edge once. Every downstream hardware adapter must also
        # implement its own timeout; it must never rely only on this process.
        if self._active:
            self._velocity_pub.publish(
                self._velocity_message(self.get_clock().now().to_msg(), None)
            )
            self._set_active(False)
            self.get_logger().warning(
                f'VR packet timeout ({age:.3f} s): velocity forced to zero'
            )
        # Permit a restarted Unity application to begin its sequence again.
        self._last_sequence = None
        self._locked_source_ip = None

    def _set_active(self, active: bool) -> None:
        if active == self._active:
            return
        self._active = active
        self._active_pub.publish(Bool(data=active))
        self.get_logger().info(f'teleop active: {active}')

    def _publish_status(self) -> None:
        self._active_pub.publish(Bool(data=self._active))
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
            f'active={str(self._active).lower()}'
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
