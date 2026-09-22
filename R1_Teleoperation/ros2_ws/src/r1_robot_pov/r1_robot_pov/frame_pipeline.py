"""
Thread-safe latest-frame storage and camera image processing.

The module is deliberately independent from ROS and from every robot-control
package.  Producers publish fully composed video frames into :class:`FrameHub`;
network transports can then wait for the newest frame without ever building a
latency-inducing queue.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import threading
import time
from typing import Any, Mapping, Optional, Sequence, Union

import cv2
import numpy as np

from .config import PROFILES as CONFIG_PROFILES


# ``width`` and ``height`` describe the final side-by-side frame.  Mono sizes
# avoid stretching a single eye to the stereo aspect ratio.  ``config.py`` is
# the canonical profile table; this mapping only adapts its typed values to the
# transform API retained for backwards compatibility and custom profiles.
PROFILES = {
    name: {
        'width': profile.width,
        'height': profile.height,
        'mono_width': max(1, profile.width // 2),
        'mono_height': profile.height,
        'fps': profile.fps,
        'jpeg_quality': profile.jpeg_quality,
        'video_bitrate': profile.video_bitrate_kbps * 1000,
    }
    for name, profile in CONFIG_PROFILES.items()
}


@dataclass(frozen=True)
class FrameSnapshot:
    """Immutable metadata and read-only pixels for one published frame."""

    sequence: int
    frame: np.ndarray
    source: str
    captured_monotonic: float
    received_monotonic: float
    reported_dropped: int = 0
    skipped: int = 0

    @property
    def width(self) -> int:
        """Return frame width in pixels."""
        return int(self.frame.shape[1])

    @property
    def height(self) -> int:
        """Return frame height in pixels."""
        return int(self.frame.shape[0])


@dataclass
class _SourceMetrics:
    connected: bool = False
    error: Optional[str] = None
    last_error: Optional[str] = None
    errors: int = 0
    reconnects: int = 0
    successful_reconnects: int = 0
    disconnects: int = 0
    frames: int = 0
    reported_dropped: int = 0
    fps: float = 0.0
    last_frame_monotonic: Optional[float] = None
    last_error_monotonic: Optional[float] = None


class FrameHub:
    """
    Keep exactly one immutable frame and coordinate multiple consumers.

    A frame is copied once when it enters the hub and is then marked read-only.
    Consumers therefore share the same pixels safely, while a slow HTTP client
    can only skip sequence numbers rather than make capture latency grow.
    """

    def __init__(self, stale_after: float = 1.0):
        """Initialize an empty hub with a configurable stale-frame limit."""
        if not math.isfinite(stale_after) or stale_after <= 0.0:
            raise ValueError('stale_after must be a positive finite number')
        self._stale_after = float(stale_after)
        self._condition = threading.Condition()
        self._latest: Optional[FrameSnapshot] = None
        self._sequence = 0
        self._total_updates = 0
        self._reported_dropped = 0
        self._sources: dict[str, _SourceMetrics] = {}

    @staticmethod
    def _validate_source(source: str) -> str:
        name = str(source).strip()
        if not name:
            raise ValueError('source must not be empty')
        return name

    def _metrics(self, source: str) -> _SourceMetrics:
        return self._sources.setdefault(source, _SourceMetrics())

    def update(
        self,
        frame: np.ndarray,
        source: str,
        captured_monotonic: Optional[float] = None,
        dropped: Optional[int] = None,
    ) -> int:
        """
        Publish a frame and return its monotonically increasing sequence.

        ``dropped`` is an incremental count reported by the source since its
        previous successful update.  It is distinct from per-consumer skipped
        sequence numbers returned by :meth:`wait_for_new`.
        """
        source_name = self._validate_source(source)
        pixels = np.asarray(frame)
        if pixels.ndim not in (2, 3) or pixels.size == 0:
            raise ValueError('frame must be a non-empty HxW or HxWxC array')
        if pixels.ndim == 3 and pixels.shape[2] not in (1, 3, 4):
            raise ValueError('frame must have 1, 3, or 4 channels')
        if pixels.dtype != np.uint8:
            raise TypeError('frame dtype must be uint8')
        immutable = np.array(pixels, dtype=np.uint8, order='C', copy=True)
        immutable.setflags(write=False)

        received = time.monotonic()
        captured = (
            received
            if captured_monotonic is None
            else float(captured_monotonic)
        )
        if not math.isfinite(captured):
            raise ValueError('captured_monotonic must be finite')
        dropped_count = 0 if dropped is None else int(dropped)
        if dropped_count < 0:
            raise ValueError('dropped must not be negative')

        with self._condition:
            self._sequence += 1
            self._total_updates += 1
            self._reported_dropped += dropped_count
            metrics = self._metrics(source_name)
            metrics.connected = True
            metrics.error = None
            metrics.frames += 1
            metrics.reported_dropped += dropped_count
            if metrics.last_frame_monotonic is not None:
                interval = received - metrics.last_frame_monotonic
                if interval > 0.0:
                    instant_fps = 1.0 / interval
                    metrics.fps = (
                        instant_fps
                        if metrics.fps == 0.0
                        else 0.85 * metrics.fps + 0.15 * instant_fps
                    )
            metrics.last_frame_monotonic = received
            self._latest = FrameSnapshot(
                sequence=self._sequence,
                frame=immutable,
                source=source_name,
                captured_monotonic=captured,
                received_monotonic=received,
                reported_dropped=dropped_count,
            )
            self._condition.notify_all()
            return self._sequence

    def snapshot(self) -> Optional[FrameSnapshot]:
        """
        Return the current immutable snapshot.

        Return ``None`` before the first frame arrives.
        """
        with self._condition:
            return self._latest

    def wait_for_new(
        self,
        after_sequence: int,
        timeout: Optional[float] = None,
    ) -> Optional[FrameSnapshot]:
        """
        Wait until a sequence newer than ``after_sequence`` is available.

        The returned ``skipped`` field is local to this wait call.  A timeout
        returns ``None``; it never returns an old frame or queues frames.
        """
        sequence = int(after_sequence)
        if timeout is not None:
            timeout = float(timeout)
            if not math.isfinite(timeout) or timeout < 0.0:
                raise ValueError(
                    'timeout must be a non-negative finite number'
                )
            deadline = time.monotonic() + timeout
        else:
            deadline = None

        with self._condition:
            while self._latest is None or self._latest.sequence <= sequence:
                if deadline is None:
                    self._condition.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
            latest = self._latest
            skipped = max(0, latest.sequence - sequence - 1)
            return replace(latest, skipped=skipped)

    def mark_source_connected(
        self,
        source: str,
        reconnected: bool = False,
    ) -> None:
        """Record that a source opened successfully, before its first frame."""
        source_name = self._validate_source(source)
        with self._condition:
            metrics = self._metrics(source_name)
            metrics.connected = True
            metrics.error = None
            if reconnected:
                metrics.successful_reconnects += 1
            self._condition.notify_all()

    def mark_source_disconnected(
        self,
        source: str,
        error: Optional[Union[str, BaseException]] = None,
    ) -> None:
        """Record a closed or failed source connection."""
        source_name = self._validate_source(source)
        with self._condition:
            metrics = self._metrics(source_name)
            if metrics.connected:
                metrics.disconnects += 1
            metrics.connected = False
            if error is not None:
                self._record_error(metrics, error)
            self._condition.notify_all()

    def mark_source_error(
        self,
        source: str,
        error: Union[str, BaseException],
        disconnected: bool = False,
    ) -> None:
        """Record a source error without necessarily declaring it offline."""
        source_name = self._validate_source(source)
        with self._condition:
            metrics = self._metrics(source_name)
            self._record_error(metrics, error)
            if disconnected:
                if metrics.connected:
                    metrics.disconnects += 1
                metrics.connected = False
            self._condition.notify_all()

    def mark_source_reconnect(self, source: str) -> None:
        """Increment the reconnect-attempt counter for ``source``."""
        source_name = self._validate_source(source)
        with self._condition:
            metrics = self._metrics(source_name)
            metrics.reconnects += 1
            metrics.connected = False
            self._condition.notify_all()

    @staticmethod
    def _record_error(
        metrics: _SourceMetrics,
        error: Union[str, BaseException],
    ) -> None:
        message = str(error).strip() or type(error).__name__
        metrics.error = message
        metrics.last_error = message
        metrics.errors += 1
        metrics.last_error_monotonic = time.monotonic()

    def status(self) -> dict[str, Any]:
        """Return a JSON-serializable diagnostics snapshot."""
        now = time.monotonic()
        with self._condition:
            latest = self._latest
            active_name = None if latest is None else latest.source
            active_metrics = (
                self._sources.get(active_name) if active_name else None
            )
            captured_age = (
                None
                if latest is None
                else max(0.0, now - latest.captured_monotonic)
            )
            received_age = (
                None
                if latest is None
                else max(0.0, now - latest.received_monotonic)
            )
            stale = latest is None or received_age > self._stale_after
            sources = {
                name: {
                    'connected': item.connected,
                    'error': item.error,
                    'last_error': item.last_error,
                    'errors': item.errors,
                    'reconnects': item.reconnects,
                    'successful_reconnects': item.successful_reconnects,
                    'disconnects': item.disconnects,
                    'frames': item.frames,
                    'fps': item.fps,
                    'reported_dropped_frames': item.reported_dropped,
                    'last_frame_age_s': (
                        None
                        if item.last_frame_monotonic is None
                        else max(0.0, now - item.last_frame_monotonic)
                    ),
                }
                for name, item in self._sources.items()
            }
            shape = None if latest is None else list(latest.frame.shape)
            return {
                'sequence': self._sequence,
                'has_frame': latest is not None,
                'source': active_name,
                'connected': bool(
                    active_metrics is not None
                    and active_metrics.connected
                    and not stale
                ),
                'stale': stale,
                'stale_after_s': self._stale_after,
                'captured_age_s': captured_age,
                'received_age_s': received_age,
                'frame_age_s': captured_age,
                'frame_age_ms': (
                    None if captured_age is None else captured_age * 1000.0
                ),
                'shape': shape,
                'fps': 0.0 if active_metrics is None else active_metrics.fps,
                'dropped_frames': self._reported_dropped,
                'reconnect_count': (
                    0 if active_metrics is None else active_metrics.reconnects
                ),
                'last_error': (
                    None
                    if active_metrics is None
                    else active_metrics.last_error
                ),
                'total_updates': self._total_updates,
                'reported_dropped_frames': self._reported_dropped,
                'sources': sources,
            }


CropSpec = Union[Sequence[int], Mapping[str, Sequence[int]]]
PerEyeValue = Union[Any, Mapping[str, Any]]


class FrameProcessor:
    """
    Apply deterministic transforms and compose mono or stereo frames.

    Crop order is ``(left, top, right, bottom)`` in pixels.  The configuration
    adapter converts its public CSS-style order before constructing this class.
    Positive rotation is clockwise.  Per-eye mappings may use ``left``,
    ``right``, and ``default`` keys; scalar values apply to both eyes.
    """

    def __init__(
        self,
        profile: Optional[Union[str, Mapping[str, Any]]] = None,
        *,
        swap_left_right: bool = False,
        crop_pixels: Optional[CropSpec] = None,
        rotation: PerEyeValue = 0,
        flip_horizontal: PerEyeValue = False,
        flip_vertical: PerEyeValue = False,
        duplicate_mono_for_stereo: bool = False,
    ):
        """Configure per-eye transforms and an optional output profile."""
        self.profile = self._resolve_profile(profile)
        self.swap_left_right = bool(swap_left_right)
        self.crop_pixels = crop_pixels
        self.rotation = rotation
        self.flip_horizontal = flip_horizontal
        self.flip_vertical = flip_vertical
        self.duplicate_mono_for_stereo = bool(duplicate_mono_for_stereo)
        for side in ('left', 'right'):
            crop = self._per_eye(
                self.crop_pixels,
                side,
                (0, 0, 0, 0),
            )
            self._normalise_crop(crop)
            self._normalise_rotation(self._per_eye(self.rotation, side, 0))

    @staticmethod
    def _resolve_profile(
        profile: Optional[Union[str, Mapping[str, Any]]]
    ) -> dict[str, Any]:
        if profile is None:
            return {}
        if isinstance(profile, str):
            key = profile.strip().lower().replace('_', '-')
            if key not in PROFILES:
                choices = sorted(PROFILES)
                raise ValueError(
                    f'unknown profile {profile!r}; expected one of {choices}'
                )
            result = dict(PROFILES[key])
        elif isinstance(profile, Mapping):
            result = dict(profile)
        elif all(hasattr(profile, key) for key in ('width', 'height')):
            result = {
                key: getattr(profile, key)
                for key in (
                    'width',
                    'height',
                    'fps',
                    'jpeg_quality',
                    'video_bitrate',
                    'video_bitrate_kbps',
                )
                if hasattr(profile, key)
            }
        else:
            raise TypeError('profile must be a name, mapping, or None')
        for key in ('width', 'height', 'mono_width', 'mono_height'):
            if key in result and int(result[key]) <= 0:
                raise ValueError(f'profile {key} must be positive')
        return result

    @staticmethod
    def _per_eye(value: PerEyeValue, side: str, default: Any) -> Any:
        if value is None:
            return default
        if isinstance(value, Mapping):
            return value.get(side, value.get('default', default))
        return value

    @staticmethod
    def _normalise_crop(crop: Sequence[int]) -> tuple[int, int, int, int]:
        if isinstance(crop, (str, bytes)) or len(crop) != 4:
            raise ValueError(
                'crop_pixels must contain left, top, right, bottom'
            )
        values = tuple(int(item) for item in crop)
        if any(item < 0 for item in values):
            raise ValueError('crop_pixels values must not be negative')
        return values

    @staticmethod
    def _normalise_rotation(rotation: Any) -> int:
        value = int(rotation) % 360
        if value not in (0, 90, 180, 270):
            raise ValueError('rotation must be 0, 90, 180, or 270 degrees')
        return value

    @staticmethod
    def _to_bgr(frame: np.ndarray) -> np.ndarray:
        image = np.asarray(frame)
        if image.dtype != np.uint8 or image.size == 0:
            raise ValueError('camera frame must be a non-empty uint8 array')
        if image.ndim == 2:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if image.ndim != 3:
            raise ValueError('camera frame must have 2 or 3 dimensions')
        if image.shape[2] == 1:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if image.shape[2] == 3:
            return np.ascontiguousarray(image)
        if image.shape[2] == 4:
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        raise ValueError('camera frame must have 1, 3, or 4 channels')

    def _transform_eye(self, frame: np.ndarray, side: str) -> np.ndarray:
        image = self._to_bgr(frame)
        crop = self._normalise_crop(
            self._per_eye(self.crop_pixels, side, (0, 0, 0, 0))
        )
        left, top, right, bottom = crop
        height, width = image.shape[:2]
        if left + right >= width or top + bottom >= height:
            raise ValueError(
                f'{side} crop {crop} removes entire {width}x{height} frame'
            )
        image = image[top:height - bottom, left:width - right]

        rotation = self._normalise_rotation(
            self._per_eye(self.rotation, side, 0)
        )
        if rotation == 90:
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        elif rotation == 180:
            image = cv2.rotate(image, cv2.ROTATE_180)
        elif rotation == 270:
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        horizontal = bool(self._per_eye(self.flip_horizontal, side, False))
        vertical = bool(self._per_eye(self.flip_vertical, side, False))
        if horizontal and vertical:
            image = cv2.flip(image, -1)
        elif horizontal:
            image = cv2.flip(image, 1)
        elif vertical:
            image = cv2.flip(image, 0)
        return np.ascontiguousarray(image)

    @staticmethod
    def compose_side_by_side(
        left: np.ndarray,
        right: np.ndarray,
    ) -> np.ndarray:
        """Compose two BGR eyes, preserving aspect ratio at a common height."""
        if left.shape[0] != right.shape[0]:
            target_height = min(left.shape[0], right.shape[0])

            def resize_to_height(image: np.ndarray) -> np.ndarray:
                width = max(
                    1,
                    round(image.shape[1] * target_height / image.shape[0]),
                )
                return cv2.resize(
                    image,
                    (width, target_height),
                    interpolation=cv2.INTER_AREA,
                )

            left = resize_to_height(left)
            right = resize_to_height(right)
        return np.ascontiguousarray(np.concatenate((left, right), axis=1))

    def process(
        self,
        left: np.ndarray,
        right: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Process mono or return one side-by-side stereo frame."""
        if right is None and self.duplicate_mono_for_stereo:
            right = left

        processed_left = self._transform_eye(left, 'left')
        is_stereo = right is not None
        if is_stereo:
            processed_right = self._transform_eye(right, 'right')
            if self.swap_left_right:
                processed_left, processed_right = (
                    processed_right,
                    processed_left,
                )
            output = self.compose_side_by_side(processed_left, processed_right)
        else:
            output = processed_left

        if self.profile:
            if is_stereo:
                width = self.profile.get('width')
                height = self.profile.get('height')
            else:
                width = self.profile.get(
                    'mono_width', self.profile.get('width')
                )
                height = self.profile.get(
                    'mono_height', self.profile.get('height')
                )
            if width is not None and height is not None:
                target = (int(width), int(height))
                if (output.shape[1], output.shape[0]) != target:
                    shrinking = (
                        target[0] < output.shape[1]
                        or target[1] < output.shape[0]
                    )
                    interpolation = (
                        cv2.INTER_AREA if shrinking else cv2.INTER_LINEAR
                    )
                    output = cv2.resize(
                        output,
                        target,
                        interpolation=interpolation,
                    )
        return np.ascontiguousarray(output)
