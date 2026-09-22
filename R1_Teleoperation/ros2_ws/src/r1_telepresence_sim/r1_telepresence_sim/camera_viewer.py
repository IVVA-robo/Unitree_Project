"""
Desktop stereo camera fallback and camera diagnostics.

This is deliberately independent of OpenXR.  It provides a verified desktop
viewer now and leaves the same ROS Image/CameraInfo topics available for a
future native OpenXR or WebRTC client.
"""

import time
import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class StereoCameraViewer(Node):
    """Show side-by-side Gazebo images and publish measurable diagnostics."""

    def __init__(self):
        super().__init__('r1_stereo_camera_viewer')
        # gazebo_ros_camera publishes below the sensor name in this URDF.
        # Keep these explicit so a future camera plugin can override them.
        self.declare_parameter(
            'left_image_topic', '/r1/camera/left/left_eye/image_raw'
        )
        self.declare_parameter(
            'right_image_topic', '/r1/camera/right/right_eye/image_raw'
        )
        self.declare_parameter(
            'left_info_topic', '/r1/camera/left/left_eye/camera_info'
        )
        self.declare_parameter(
            'right_info_topic', '/r1/camera/right/right_eye/camera_info'
        )
        self.declare_parameter('display', True)
        self.declare_parameter('window_name', 'R1 telepresence stereo fallback')
        self.declare_parameter('diagnostics_topic', '/r1/telepresence/diagnostics')
        self.declare_parameter('display_scale', 0.75)
        self.declare_parameter('expected_fps', 30.0)
        self._left = None
        self._right = None
        self._left_stamp = None
        self._right_stamp = None
        self._left_info = False
        self._right_info = False
        self._frames = {'left': 0, 'right': 0}
        self._last_frames = {'left': 0, 'right': 0}
        self._last_report = time.monotonic()
        self._fps = {'left': 0.0, 'right': 0.0}
        self._dropped = {'left': 0, 'right': 0}
        self._display = bool(self.get_parameter('display').value)
        self._scale = float(self.get_parameter('display_scale').value)
        self._expected_fps = float(self.get_parameter('expected_fps').value)
        if self._scale <= 0.0:
            raise ValueError('display_scale must be positive')
        if self._expected_fps <= 0.0:
            raise ValueError('expected_fps must be positive')
        self.create_subscription(
            Image,
            str(self.get_parameter('left_image_topic').value),
            lambda msg: self._image_callback('left', msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('right_image_topic').value),
            lambda msg: self._image_callback('right', msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('left_info_topic').value),
            lambda msg: self._info_callback('left', msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('right_info_topic').value),
            lambda msg: self._info_callback('right', msg),
            qos_profile_sensor_data,
        )
        self._diagnostics = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value),
            10,
        )
        self.create_timer(0.1, self._render)
        self.create_timer(1.0, self._publish_diagnostics)

    def _image_callback(self, side, msg):
        try:
            image = self._image_to_bgr(msg)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f'{side} camera conversion failed: {exc}')
            return
        self._estimate_dropped_frames(side, msg)
        setattr(self, f'_{side}', image)
        setattr(self, f'_{side}_stamp', msg.header.stamp)
        self._frames[side] += 1

    def _estimate_dropped_frames(self, side, msg):
        """Estimate skipped Gazebo frames from monotonic image timestamps."""
        previous = self._latest_stamp_seconds(side)
        current = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if previous is not None and current > previous:
            expected_period = 1.0 / self._expected_fps
            missed = int(round((current - previous) / expected_period)) - 1
            if missed > 0:
                self._dropped[side] += missed

    def _latest_stamp_seconds(self, side):
        stamp = getattr(self, f'_{side}_stamp')
        if stamp is None:
            return None
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _image_to_bgr(msg):
        """
        Convert the common Gazebo ROS image encodings without cv_bridge.

        cv_bridge is intentionally not used here: the desktop environment can
        have a NumPy major-version mismatch with the binary ROS extension.  A
        copy-free view is made for each row, then converted only when needed.
        """
        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)
        if width <= 0 or height <= 0 or step <= 0:
            raise ValueError(f'invalid image dimensions {width}x{height}, step={step}')
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        required = height * step
        if raw.size < required:
            raise ValueError(f'image payload too short: {raw.size} < {required}')
        rows = raw[:required].reshape((height, step))
        encoding = str(msg.encoding).lower()
        if encoding in ('bgr8', '8uc3'):
            channels = 3
            row_bytes = width * channels
            if step < row_bytes:
                raise ValueError(f'step {step} is smaller than {row_bytes} bytes for {encoding}')
            return rows[:, :row_bytes].reshape((height, width, channels)).copy()
        if encoding == 'rgb8':
            channels = 3
            row_bytes = width * channels
            if step < row_bytes:
                raise ValueError(f'step {step} is smaller than {row_bytes} bytes for {encoding}')
            rgb = rows[:, :row_bytes].reshape((height, width, channels))
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if encoding in ('mono8', '8uc1'):
            if step < width:
                raise ValueError(f'step {step} is smaller than image width {width}')
            gray = rows[:, :width].copy()
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if encoding == 'rgba8':
            channels = 4
            row_bytes = width * channels
            if step < row_bytes:
                raise ValueError(f'step {step} is smaller than {row_bytes} bytes for {encoding}')
            rgba = rows[:, :row_bytes].reshape((height, width, channels))
            return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            channels = 4
            row_bytes = width * channels
            if step < row_bytes:
                raise ValueError(f'step {step} is smaller than {row_bytes} bytes for {encoding}')
            bgra = rows[:, :row_bytes].reshape((height, width, channels))
            return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        raise ValueError(f'unsupported image encoding: {msg.encoding!r}')

    def _latest(self, side):
        return getattr(self, f'_{side}')

    def _info_callback(self, side, _msg):
        setattr(self, f'_{side}_info', True)

    def _render(self):
        now = time.monotonic()
        elapsed = now - self._last_report
        if elapsed >= 0.5:
            for side in ('left', 'right'):
                self._fps[side] = (self._frames[side] - self._last_frames[side]) / elapsed
                self._last_frames[side] = self._frames[side]
            self._last_report = now
        if not self._display or self._left is None or self._right is None:
            return
        try:
            height = min(self._left.shape[0], self._right.shape[0])
            left_width = int(self._left.shape[1] * height / self._left.shape[0])
            right_width = int(self._right.shape[1] * height / self._right.shape[0])
            left = cv2.resize(self._left, (left_width, height))
            right = cv2.resize(self._right, (right_width, height))
            stereo = cv2.hconcat([left, right])
            if self._scale != 1.0:
                stereo = cv2.resize(stereo, None, fx=self._scale, fy=self._scale)
            cv2.imshow(str(self.get_parameter('window_name').value), stereo)
            cv2.waitKey(1)
        except cv2.error as exc:
            self.get_logger().warning(f'desktop display unavailable: {exc}')
            self._display = False

    def _publish_diagnostics(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(
            name='r1_stereo_camera',
            level=DiagnosticStatus.OK,
            message='camera stream',
        )
        for key, value in (
            ('left_fps', self._fps['left']),
            ('right_fps', self._fps['right']),
            ('left_dropped_frames', self._dropped['left']),
            ('right_dropped_frames', self._dropped['right']),
            ('left_camera_info', self._left_info),
            ('right_camera_info', self._right_info),
            ('display_enabled', self._display),
        ):
            status.values.append(KeyValue(key=str(key), value=str(value)))
        if not self._left_info or not self._right_info:
            status.level = DiagnosticStatus.WARN
            status.message = 'waiting for CameraInfo'
        msg.status.append(status)
        self._diagnostics.publish(msg)

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        # SIGINT can arrive again while DDS entities are being destroyed by a
        # launch supervisor; cleanup is best-effort after the process is safe.
        try:
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
