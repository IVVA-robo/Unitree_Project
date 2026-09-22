import errno
import socket
import time
from typing import Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from .protocol import VRPacket, apply_deadzone, is_newer_sequence, parse_packet, PacketError

class VRBridgeNode(Node):
    def __init__(self):
        super().__init__('vr_teleop_bridge')
        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('udp_port', 9090)
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

        self._bind_address = self.get_parameter('bind_address').value
        self._udp_port = self.get_parameter('udp_port').value
        self._allowed_source_ip = self.get_parameter('allowed_source_ip').value
        self._tracking_frame = self.get_parameter('tracking_frame').value
        self._packet_timeout_sec = self.get_parameter('packet_timeout_sec').value
        self._deadzone = self.get_parameter('deadzone').value
        self._max_forward_mps = self.get_parameter('max_forward_mps').value
        self._max_lateral_mps = self.get_parameter('max_lateral_mps').value
        self._max_yaw_rps = self.get_parameter('max_yaw_rps').value
        self._forward_sign = self.get_parameter('forward_sign').value
        self._lateral_sign = self.get_parameter('lateral_sign').value
        self._yaw_sign = self.get_parameter('yaw_sign').value
        self._max_position_m = self.get_parameter('max_position_m').value
        self._max_datagram_bytes = self.get_parameter('max_datagram_bytes').value

        self._left_pub = self.create_publisher(PoseStamped, '/vr/left_controller/pose', 1)
        self._right_pub = self.create_publisher(PoseStamped, '/vr/right_controller/pose', 1)
        self._head_pub = self.create_publisher(PoseStamped, '/vr/head/pose', 1)
        self._joy_pub = self.create_publisher(Joy, '/vr/joy', 1)
        self._velocity_pub = self.create_publisher(TwistStamped, '/vr/cmd_vel', 1)
        self._active_pub = self.create_publisher(Bool, '/vr/teleop/active', 1)

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((self._bind_address, self._udp_port))
        self._socket.setblocking(False)

        self._last_sequence = None
        self._last_valid_monotonic = None
        self._active = False
        self._bad_packet_count = 0
        self._last_warning_monotonic = 0.0

        self.create_timer(0.005, self._receive_all)
        self.create_timer(0.05, self._watchdog)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(f'Listening on UDP {self._bind_address}:{self._udp_port}')

    def _receive_all(self):
        newest = None
        newest_address = None
        for _ in range(64):
            try:
                payload, address = self._socket.recvfrom(self._max_datagram_bytes + 1)
                newest = payload
                newest_address = address
            except (BlockingIOError, OSError):
                break
        if newest is None: return
        try:
            packet = parse_packet(newest, max_position_m=self._max_position_m)
            if not is_newer_sequence(packet.sequence, self._last_sequence): return
        except PacketError: return
        self._last_valid_monotonic = time.monotonic()
        self._last_sequence = packet.sequence
        self._publish_packet(packet)

    def _publish_packet(self, packet: VRPacket):
        stamp = self.get_clock().now().to_msg()
        self._left_pub.publish(self._pose_message(stamp, packet.left))
        self._right_pub.publish(self._pose_message(stamp, packet.right))
        self._head_pub.publish(self._pose_message(stamp, packet.head))
        joy = Joy()
        joy.header.stamp = stamp
        joy.header.frame_id = self._tracking_frame
        joy.axes = [float(packet.left_stick[0]), float(packet.left_stick[1]), float(packet.right_stick[0]), float(packet.right_stick[1]), float(packet.left_trigger), float(packet.right_trigger)]
        joy.buttons = [1 if packet.deadman else 0, 1 if packet.left_trigger >= 0.5 else 0, 1 if packet.right_trigger >= 0.5 else 0]
        self._joy_pub.publish(joy)
        self._set_active(packet.deadman)
        self._velocity_pub.publish(self._velocity_message(stamp, packet))

    def _pose_message(self, stamp, source):
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

    def _velocity_message(self, stamp, packet):
        message = TwistStamped()
        message.header.stamp = stamp
        message.header.frame_id = 'base_link'
        if packet is None or not packet.deadman: return message
        left_x = apply_deadzone(packet.left_stick[0], self._deadzone)
        left_y = apply_deadzone(packet.left_stick[1], self._deadzone)
        right_x = apply_deadzone(packet.right_stick[0], self._deadzone)
        message.twist.linear.x = self._forward_sign * self._max_forward_mps * left_y
        message.twist.linear.y = self._lateral_sign * self._max_lateral_mps * left_x
        message.twist.angular.z = self._yaw_sign * self._max_yaw_rps * right_x
        return message

    def _watchdog(self):
        if self._last_valid_monotonic is None: return
        age = time.monotonic() - self._last_valid_monotonic
        if age <= self._packet_timeout_sec: return
        if self._active:
            self._velocity_pub.publish(self._velocity_message(self.get_clock().now().to_msg(), None))
            self._set_active(False)
            self.get_logger().warning('VR packet timeout: velocity forced to zero')
        self._last_sequence = None

    def _set_active(self, active: bool):
        if active == self._active: return
        self._active = active
        self._active_pub.publish(Bool(data=active))
        self.get_logger().info(f'teleop active: {active}')

    def _publish_status(self):
        self._active_pub.publish(Bool(data=self._active))

def main(args=None):
    rclpy.init(args=args)
    node = VRBridgeNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()
if __name__ == '__main__': main()
