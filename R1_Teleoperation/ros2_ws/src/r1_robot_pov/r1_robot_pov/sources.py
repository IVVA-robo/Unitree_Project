"""
Video sources for the offline Robot POV frame pipeline.

All sources terminate at :class:`~r1_robot_pov.frame_pipeline.FrameHub` and
never import robot motion or actuator interfaces.  ROS imports are optional so
the mock and OpenCV exhibition modes remain usable in a plain Python process.
"""

from __future__ import annotations

from dataclasses import dataclass
import datetime as _datetime
import math
import threading
import time
from typing import Any, Callable, Optional, Union

import cv2
import numpy as np

from .frame_pipeline import FrameHub, FrameProcessor


try:  # Keep mock/USB/RTSP modes importable on laptops without ROS installed.
    import rclpy
    from rclpy.node import Node as _RosNode
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image as _RosImage

    ROS_AVAILABLE = True
    _ROS_IMPORT_ERROR: Optional[BaseException] = None
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
    rclpy = None
    qos_profile_sensor_data = None
    _RosImage = Any
    _RosNode = object
    ROS_AVAILABLE = False
    _ROS_IMPORT_ERROR = exc


try:  # Unitree SDK is optional for mock/USB/RTSP/ROS camera modes.
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize as _ChannelFactoryInitialize,
    )
    from unitree_sdk2py.go2.video.video_client import (
        VideoClient as _VideoClient,
    )

    UNITREE_SDK_AVAILABLE = True
    UNITREE_SDK_IMPORT_ERROR: Optional[BaseException] = None
except (ImportError, ModuleNotFoundError, OSError) as exc:  # pragma: no cover
    _ChannelFactoryInitialize = None
    _VideoClient = None
    UNITREE_SDK_AVAILABLE = False
    UNITREE_SDK_IMPORT_ERROR = exc


def ros_image_to_bgr(message: Any) -> np.ndarray:
    """
    Convert common ``sensor_msgs/Image`` encodings to contiguous BGR8.

    The converter intentionally avoids ``cv_bridge``.  Apart from making unit
    tests independent of ROS, this avoids binary NumPy ABI mismatches and
    correctly handles row padding through the message's ``step`` field.
    """
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    if width <= 0 or height <= 0 or step <= 0:
        raise ValueError(
            f'invalid image dimensions {width}x{height}, step={step}'
        )

    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    required = height * step
    if raw.size < required:
        raise ValueError(f'image payload too short: {raw.size} < {required}')
    rows = raw[:required].reshape((height, step))
    encoding = str(message.encoding).strip().lower()

    if encoding in ('bgr8', '8uc3'):
        row_bytes = width * 3
        if step < row_bytes:
            raise ValueError(
                f'step {step} is smaller than {row_bytes} for {encoding}'
            )
        return rows[:, :row_bytes].reshape((height, width, 3)).copy()
    if encoding == 'rgb8':
        row_bytes = width * 3
        if step < row_bytes:
            raise ValueError(
                f'step {step} is smaller than {row_bytes} for rgb8'
            )
        rgb = rows[:, :row_bytes].reshape((height, width, 3))
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if encoding in ('mono8', '8uc1'):
        if step < width:
            raise ValueError(
                f'step {step} is smaller than width {width} for {encoding}'
            )
        return cv2.cvtColor(rows[:, :width].copy(), cv2.COLOR_GRAY2BGR)
    if encoding == 'rgba8':
        row_bytes = width * 4
        if step < row_bytes:
            raise ValueError(
                f'step {step} is smaller than {row_bytes} for rgba8'
            )
        rgba = rows[:, :row_bytes].reshape((height, width, 4))
        return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    if encoding == 'bgra8':
        row_bytes = width * 4
        if step < row_bytes:
            raise ValueError(
                f'step {step} is smaller than {row_bytes} for bgra8'
            )
        bgra = rows[:, :row_bytes].reshape((height, width, 4))
        return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
    raise ValueError(f'unsupported ROS image encoding: {message.encoding!r}')


class MockSource:
    """Generate an animated mono or stereo test pattern at a bounded rate."""

    def __init__(
        self,
        hub: FrameHub,
        processor: Optional[FrameProcessor] = None,
        *,
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
        stereo: bool = True,
        name: str = 'mock',
    ):
        """Configure a generated source without starting its worker."""
        if int(width) <= 0 or int(height) <= 0:
            raise ValueError('mock width and height must be positive')
        if not math.isfinite(fps) or fps <= 0.0:
            raise ValueError('mock fps must be a positive finite number')
        self.hub = hub
        self.processor = processor or FrameProcessor()
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.stereo = bool(stereo)
        self.name = str(name)
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._started_monotonic = time.monotonic()

    @property
    def running(self) -> bool:
        """Return whether the source worker is alive."""
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        """Start frame generation; repeated calls are harmless."""
        if self.running:
            return
        self._stop_event.clear()
        self._started_monotonic = time.monotonic()
        self._thread = threading.Thread(
            target=self._run,
            name=f'{self.name}-video-source',
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        """Stop frame generation and mark the source disconnected."""
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(max(0.0, float(timeout)))
        self._thread = None
        self.hub.mark_source_disconnected(self.name)

    def _run(self) -> None:
        period = 1.0 / self.fps
        deadline = time.monotonic()
        self.hub.mark_source_connected(self.name)
        while not self._stop_event.is_set():
            captured = time.monotonic()
            try:
                frame = self.render_frame(captured - self._started_monotonic)
                self.hub.update(frame, self.name, captured_monotonic=captured)
            except Exception as exc:  # keep a demo alive and expose the error
                self.hub.mark_source_error(self.name, exc)
            deadline += period
            delay = deadline - time.monotonic()
            if delay <= -period:
                # Do not generate a burst to catch up after CPU starvation.
                deadline = time.monotonic()
                delay = 0.0
            self._stop_event.wait(max(0.0, delay))

    def render_frame(self, elapsed_s: Optional[float] = None) -> np.ndarray:
        """Render one deterministic frame without starting a thread."""
        elapsed = (
            time.monotonic() - self._started_monotonic
            if elapsed_s is None
            else float(elapsed_s)
        )
        left = self._render_eye('LEFT', elapsed, disparity=-8)
        right = (
            self._render_eye('RIGHT', elapsed, disparity=8)
            if self.stereo
            else None
        )
        return self.processor.process(left, right)

    def _render_eye(
        self,
        label: str,
        elapsed: float,
        disparity: int,
    ) -> np.ndarray:
        height, width = self.height, self.width
        image = np.zeros((height, width, 3), dtype=np.uint8)
        horizontal = np.linspace(20, 105, width, dtype=np.uint8)
        image[:, :, 1] = horizontal[None, :]
        image[:, :, 0] = np.uint8(55 if label == 'LEFT' else 25)
        image[:, :, 2] = np.uint8(25 if label == 'LEFT' else 55)
        spacing = max(24, min(width, height) // 8)
        image[::spacing, :, :] = (60, 150, 60)
        image[:, ::spacing, :] = (60, 150, 60)

        travel = max(1, width - 120)
        centre_x = 60 + int((0.5 + 0.5 * math.sin(elapsed * 1.4)) * travel)
        centre_x = max(25, min(width - 25, centre_x + disparity))
        centre_y = height // 2 + int(math.sin(elapsed * 0.9) * height * 0.18)
        cv2.circle(
            image,
            (centre_x, centre_y),
            max(10, height // 18),
            (0, 220, 255),
            -1,
        )
        cv2.line(
            image,
            (width // 2, 0),
            (width // 2, height - 1),
            (255, 255, 255),
            1,
        )
        cv2.line(
            image,
            (0, height // 2),
            (width - 1, height // 2),
            (255, 255, 255),
            1,
        )

        font_scale = max(0.45, min(width, height) / 700.0)
        cv2.putText(
            image,
            f'ROBOT POV - {label}',
            (18, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        timestamp = _datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        cv2.putText(
            image,
            timestamp,
            (18, height - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            max(0.4, font_scale * 0.8),
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
        return image


CaptureSpec = Union[int, str]


class OpenCVSource:
    """Read one or two USB/RTSP streams, reconnecting without a frame queue."""

    def __init__(
        self,
        hub: FrameHub,
        left_source: CaptureSpec,
        right_source: Optional[CaptureSpec] = None,
        processor: Optional[FrameProcessor] = None,
        *,
        name: str = 'opencv',
        reconnect_delay_s: float = 1.0,
        capture_width: Optional[int] = None,
        capture_height: Optional[int] = None,
        capture_fps: Optional[float] = None,
        api_preference: Optional[int] = None,
        read_timeout_ms: int = 1500,
    ):
        """Configure USB/RTSP capture without opening physical devices."""
        if not math.isfinite(reconnect_delay_s) or reconnect_delay_s < 0.0:
            raise ValueError(
                'reconnect_delay_s must be finite and non-negative'
            )
        self.hub = hub
        self.left_source = left_source
        self.right_source = right_source
        self.processor = processor or FrameProcessor()
        self.name = str(name)
        self.reconnect_delay_s = float(reconnect_delay_s)
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.capture_fps = capture_fps
        self.api_preference = api_preference
        self.read_timeout_ms = max(1, int(read_timeout_ms))
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._captures_lock = threading.Lock()
        self._captures: list[cv2.VideoCapture] = []

    @property
    def running(self) -> bool:
        """Return whether the capture worker is alive."""
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        """Start capture and automatic reconnect."""
        if self.running:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name=f'{self.name}-video-source',
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        """Stop capture within the configured OpenCV read timeout."""
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(max(0.0, float(timeout)))
        self._release_captures()
        self._thread = None
        self.hub.mark_source_disconnected(self.name)

    @staticmethod
    def _normalise_spec(spec: CaptureSpec) -> CaptureSpec:
        if isinstance(spec, str) and spec.strip().isdigit():
            return int(spec.strip())
        return spec

    def _open_capture(self, spec: CaptureSpec) -> cv2.VideoCapture:
        source = self._normalise_spec(spec)
        parameters = []
        if isinstance(source, str) and '://' in source:
            if hasattr(cv2, 'CAP_PROP_OPEN_TIMEOUT_MSEC'):
                parameters.extend(
                    [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, self.read_timeout_ms]
                )
            if hasattr(cv2, 'CAP_PROP_READ_TIMEOUT_MSEC'):
                parameters.extend(
                    [cv2.CAP_PROP_READ_TIMEOUT_MSEC, self.read_timeout_ms]
                )
        if self.api_preference is not None:
            api = int(self.api_preference)
        elif isinstance(source, int):
            api = cv2.CAP_V4L2
        elif '://' in str(source):
            api = cv2.CAP_FFMPEG
        else:
            api = cv2.CAP_ANY

        try:
            capture = cv2.VideoCapture(source, api, parameters)
        except (TypeError, cv2.error):  # OpenCV builds before params overload.
            capture = cv2.VideoCapture(source, api)

        if hasattr(cv2, 'CAP_PROP_OPEN_TIMEOUT_MSEC'):
            capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, self.read_timeout_ms)
        if hasattr(cv2, 'CAP_PROP_READ_TIMEOUT_MSEC'):
            capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, self.read_timeout_ms)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.capture_width is not None:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.capture_width))
        if self.capture_height is not None:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.capture_height))
        if self.capture_fps is not None:
            capture.set(cv2.CAP_PROP_FPS, float(self.capture_fps))
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f'cannot open camera stream {spec!r}')
        return capture

    def _open_all(self) -> list[cv2.VideoCapture]:
        opened: list[cv2.VideoCapture] = []
        try:
            opened.append(self._open_capture(self.left_source))
            if self.right_source is not None:
                opened.append(self._open_capture(self.right_source))
            return opened
        except Exception:
            for capture in opened:
                capture.release()
            raise

    def _release_captures(self) -> None:
        with self._captures_lock:
            captures, self._captures = self._captures, []
        for capture in captures:
            capture.release()

    def _read_pair(
        self,
        captures: list[cv2.VideoCapture],
    ) -> tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        if len(captures) == 1:
            ok, left = captures[0].read()
            return bool(ok), left if ok else None, None

        # Grab both first to reduce skew between independent stereo devices.
        left_grabbed = captures[0].grab()
        right_grabbed = captures[1].grab()
        if not left_grabbed or not right_grabbed:
            return False, None, None
        left_ok, left = captures[0].retrieve()
        right_ok, right = captures[1].retrieve()
        return bool(left_ok and right_ok), left, right

    def _run(self) -> None:
        reconnecting = False
        pending_dropped = 0
        while not self._stop_event.is_set():
            try:
                captures = self._open_all()
                with self._captures_lock:
                    self._captures = captures
                self.hub.mark_source_connected(
                    self.name,
                    reconnected=reconnecting,
                )
                reconnecting = False
                while not self._stop_event.is_set():
                    captured = time.monotonic()
                    ok, left, right = self._read_pair(captures)
                    if not ok or left is None:
                        raise RuntimeError('camera read failed or timed out')
                    frame = self.processor.process(left, right)
                    self.hub.update(
                        frame,
                        self.name,
                        captured_monotonic=captured,
                        dropped=pending_dropped,
                    )
                    pending_dropped = 0
            except Exception as exc:
                if self._stop_event.is_set():
                    break
                pending_dropped += 1
                self.hub.mark_source_error(self.name, exc, disconnected=True)
            finally:
                self._release_captures()

            if self._stop_event.is_set():
                break
            reconnecting = True
            self.hub.mark_source_reconnect(self.name)
            self._stop_event.wait(self.reconnect_delay_s)


_UNITREE_MAX_RECONNECT_DELAY_S = 5.0


def _next_unitree_reconnect_delay(
    initial_delay_s: float,
    previous_delay_s: Optional[float],
) -> float:
    """Return the next bounded delay in the Unitree reconnect backoff."""
    if previous_delay_s is None:
        return min(initial_delay_s, _UNITREE_MAX_RECONNECT_DELAY_S)
    return min(
        max(initial_delay_s, previous_delay_s * 2.0),
        _UNITREE_MAX_RECONNECT_DELAY_S,
    )


class UnitreeVideoSource:
    """Read the R1 front camera through Unitree's video-only RPC client."""

    def __init__(
        self,
        hub: FrameHub,
        processor: Optional[FrameProcessor] = None,
        *,
        network_interface: str,
        domain_id: int = 0,
        rpc_timeout_s: float = 1.0,
        reconnect_delay_s: float = 0.5,
        name: str = 'unitree-video',
        channel_initializer: Optional[Callable[..., Any]] = None,
        video_client_factory: Optional[Callable[[], Any]] = None,
    ):
        """Configure capture without creating DDS participants or clients."""
        interface = str(network_interface).strip()
        if not interface:
            raise ValueError('network_interface must not be empty')
        if int(domain_id) < 0:
            raise ValueError('domain_id must be non-negative')
        if not math.isfinite(rpc_timeout_s) or rpc_timeout_s <= 0.0:
            raise ValueError('rpc_timeout_s must be a positive finite number')
        if (
            not math.isfinite(reconnect_delay_s)
            or reconnect_delay_s < 0.0
        ):
            raise ValueError(
                'reconnect_delay_s must be finite and non-negative'
            )
        source_name = str(name).strip()
        if not source_name:
            raise ValueError('name must not be empty')

        self.hub = hub
        self.processor = processor or FrameProcessor()
        self.network_interface = interface
        self.domain_id = int(domain_id)
        self.rpc_timeout_s = float(rpc_timeout_s)
        self.reconnect_delay_s = float(reconnect_delay_s)
        self.name = source_name
        self._channel_initializer = (
            _ChannelFactoryInitialize
            if channel_initializer is None
            else channel_initializer
        )
        self._video_client_factory = (
            _VideoClient
            if video_client_factory is None
            else video_client_factory
        )
        self._stop_event = threading.Event()
        self._lifecycle_lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    @property
    def available(self) -> bool:
        """Return whether both read-only Unitree SDK entry points exist."""
        return callable(self._channel_initializer) and callable(
            self._video_client_factory
        )

    @property
    def availability_error(self) -> Optional[BaseException]:
        """Return the optional SDK import error used by command-line UIs."""
        if self.available:
            return None
        if UNITREE_SDK_IMPORT_ERROR is not None:
            return UNITREE_SDK_IMPORT_ERROR
        return RuntimeError('Unitree video SDK dependencies are unavailable')

    @property
    def running(self) -> bool:
        """Return whether the Unitree capture worker is alive."""
        thread = self._thread
        return thread is not None and thread.is_alive()

    def start(self) -> None:
        """Start video capture; all SDK and network setup occurs in worker."""
        if not self.available:
            message = (
                'UnitreeVideoSource requires unitree_sdk2py video support'
            )
            error = self.availability_error
            if error is not None:
                raise RuntimeError(f'{message}: {error}') from error
            raise RuntimeError(message)

        with self._lifecycle_lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run,
                name=f'{self.name}-video-source',
                daemon=True,
            )
            self._thread.start()

    def stop(self, timeout: Optional[float] = None) -> None:
        """Stop capture within the SDK RPC timeout plus scheduling margin."""
        wait_timeout = (
            self.rpc_timeout_s + 0.5
            if timeout is None
            else float(timeout)
        )
        if not math.isfinite(wait_timeout) or wait_timeout < 0.0:
            raise ValueError('stop timeout must be finite and non-negative')
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(wait_timeout)
        with self._lifecycle_lock:
            if self._thread is thread and (
                thread is None or not thread.is_alive()
            ):
                self._thread = None

    def _create_client(self) -> Any:
        initializer = self._channel_initializer
        factory = self._video_client_factory
        if not callable(initializer) or not callable(factory):
            raise RuntimeError(
                'Unitree video SDK dependencies are unavailable'
            )
        initializer(self.domain_id, self.network_interface)
        client = factory()
        required = ('SetTimeout', 'Init', 'GetImageSample')
        missing = [
            name
            for name in required
            if not callable(getattr(client, name, None))
        ]
        if missing:
            raise TypeError(
                'Unitree VideoClient is missing methods: '
                + ', '.join(missing)
            )
        client.SetTimeout(self.rpc_timeout_s)
        client.Init()
        return client

    @staticmethod
    def _decode_sample(result: Any) -> np.ndarray:
        try:
            code, payload = result
        except (TypeError, ValueError) as exc:
            raise ValueError(
                'GetImageSample must return a (code, JPEG payload) pair'
            ) from exc
        if code != 0:
            raise RuntimeError(f'GetImageSample failed with code {code}')
        if payload is None:
            raise ValueError('GetImageSample returned an empty JPEG payload')
        try:
            encoded = np.frombuffer(bytes(payload), dtype=np.uint8)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                'GetImageSample returned an invalid JPEG payload'
            ) from exc
        if encoded.size == 0:
            raise ValueError('GetImageSample returned an empty JPEG payload')
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None or image.size == 0:
            raise ValueError(
                'GetImageSample returned a malformed JPEG payload'
            )
        return image

    def _run(self) -> None:
        reconnecting = False
        pending_dropped = 0
        reconnect_delay_s: Optional[float] = None
        client = None
        connected = False
        try:
            while not self._stop_event.is_set():
                if client is None:
                    try:
                        client = self._create_client()
                    except Exception as exc:
                        if self._stop_event.is_set():
                            break
                        pending_dropped += 1
                        self.hub.mark_source_error(
                            self.name,
                            exc,
                            disconnected=True,
                        )
                        reconnecting = True
                        self.hub.mark_source_reconnect(self.name)
                        reconnect_delay_s = _next_unitree_reconnect_delay(
                            self.reconnect_delay_s,
                            reconnect_delay_s,
                        )
                        if self._stop_event.wait(reconnect_delay_s):
                            break
                        continue

                try:
                    captured = time.monotonic()
                    result = client.GetImageSample()
                    if self._stop_event.is_set():
                        break
                    image = self._decode_sample(result)
                    frame = self.processor.process(image)
                    if self._stop_event.is_set():
                        break
                    if not connected:
                        self.hub.mark_source_connected(
                            self.name,
                            reconnected=reconnecting,
                        )
                        connected = True
                        reconnecting = False
                    self.hub.update(
                        frame,
                        self.name,
                        captured_monotonic=captured,
                        dropped=pending_dropped,
                    )
                    pending_dropped = 0
                    reconnect_delay_s = None
                except Exception as exc:
                    if self._stop_event.is_set():
                        break
                    pending_dropped += 1
                    connected = False
                    self.hub.mark_source_error(
                        self.name,
                        exc,
                        disconnected=True,
                    )
                    # Keep the initialized VideoClient alive while the service
                    # is temporarily unavailable.  Recreating it for every
                    # non-zero RPC result leaks DDS request/response endpoints
                    # and can prevent a late-starting videohub from matching.
                    reconnecting = True
                    self.hub.mark_source_reconnect(self.name)
                    reconnect_delay_s = _next_unitree_reconnect_delay(
                        self.reconnect_delay_s,
                        reconnect_delay_s,
                    )
                    if self._stop_event.wait(reconnect_delay_s):
                        break
        finally:
            self.hub.mark_source_disconnected(self.name)
            with self._lifecycle_lock:
                if self._thread is threading.current_thread():
                    self._thread = None


@dataclass
class _PendingRosImage:
    stamp: float
    received_monotonic: float
    message: Any


def _ros_stamp_seconds(message: Any, fallback: float) -> float:
    try:
        stamp = message.header.stamp
        seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return fallback
    if seconds == 0.0 or not math.isfinite(seconds):
        return fallback
    return seconds


class RosStereoSource(_RosNode):
    """Approximately synchronize raw mono/stereo ROS Image topics."""

    def __init__(
        self,
        hub: FrameHub,
        processor: Optional[FrameProcessor] = None,
        *,
        left_topic: str = '/r1/camera/left/left_eye/image_raw',
        right_topic: Optional[str] = '/r1/camera/right/right_eye/image_raw',
        sync_slop_s: float = 0.045,
        sync_queue_size: int = 6,
        name: str = 'ros-camera',
        node_name: str = 'r1_robot_pov_source',
        qos: Any = None,
    ):
        """Create ROS subscriptions and the bounded timestamp synchronizer."""
        if not ROS_AVAILABLE:
            raise RuntimeError(
                'RosStereoSource requires rclpy and sensor_msgs; '
                f'import failed: {_ROS_IMPORT_ERROR}'
            )
        if not math.isfinite(sync_slop_s) or sync_slop_s < 0.0:
            raise ValueError('sync_slop_s must be finite and non-negative')
        if int(sync_queue_size) < 1:
            raise ValueError('sync_queue_size must be at least one')
        super().__init__(node_name)
        self.hub = hub
        self.processor = processor or FrameProcessor()
        self.source_name = str(name)
        self.right_topic = str(right_topic).strip() if right_topic else None
        self.sync_slop_s = float(sync_slop_s)
        self.sync_queue_size = int(sync_queue_size)
        self._pair_lock = threading.Lock()
        self._pending = {'left': [], 'right': []}
        self._pending_dropped = 0
        selected_qos = qos if qos is not None else qos_profile_sensor_data
        self._left_subscription = self.create_subscription(
            _RosImage,
            str(left_topic),
            lambda message: self._image_callback('left', message),
            selected_qos,
        )
        self._right_subscription = None
        if self.right_topic:
            self._right_subscription = self.create_subscription(
                _RosImage,
                self.right_topic,
                lambda message: self._image_callback('right', message),
                selected_qos,
            )

    def _image_callback(self, side: str, message: Any) -> None:
        received = time.monotonic()
        if not self.right_topic:
            if side == 'left':
                self._publish_messages(message, None, received, 0)
            return

        item = _PendingRosImage(
            stamp=_ros_stamp_seconds(message, received),
            received_monotonic=received,
            message=message,
        )
        with self._pair_lock:
            queue = self._pending[side]
            queue.append(item)
            while len(queue) > self.sync_queue_size:
                queue.pop(0)
                self._pending_dropped += 1
            pairs = self._take_pairs_locked()

        for left, right, dropped in pairs:
            captured = min(left.received_monotonic, right.received_monotonic)
            self._publish_messages(
                left.message,
                right.message,
                captured,
                dropped,
            )

    def _take_pairs_locked(
        self,
    ) -> list[tuple[_PendingRosImage, _PendingRosImage, int]]:
        pairs = []
        left_queue = self._pending['left']
        right_queue = self._pending['right']
        while left_queue and right_queue:
            left_index, right_index, difference = min(
                (
                    (left_i, right_i, abs(left.stamp - right.stamp))
                    for left_i, left in enumerate(left_queue)
                    for right_i, right in enumerate(right_queue)
                ),
                key=lambda candidate: candidate[2],
            )
            if difference > self.sync_slop_s:
                break
            dropped = self._pending_dropped + left_index + right_index
            self._pending_dropped = 0
            left = left_queue[left_index]
            right = right_queue[right_index]
            del left_queue[:left_index + 1]
            del right_queue[:right_index + 1]
            pairs.append((left, right, dropped))
        return pairs

    def _publish_messages(
        self,
        left_message: Any,
        right_message: Optional[Any],
        captured_monotonic: float,
        dropped: int,
    ) -> None:
        try:
            left = ros_image_to_bgr(left_message)
            right = (
                None
                if right_message is None
                else ros_image_to_bgr(right_message)
            )
            frame = self.processor.process(left, right)
            self.hub.update(
                frame,
                self.source_name,
                captured_monotonic=captured_monotonic,
                dropped=dropped,
            )
        except (AttributeError, TypeError, ValueError, cv2.error) as exc:
            self.hub.mark_source_error(self.source_name, exc)

    def destroy_node(self) -> Any:
        """Mark the video source offline before destroying the ROS node."""
        self.hub.mark_source_disconnected(self.source_name)
        return super().destroy_node()
