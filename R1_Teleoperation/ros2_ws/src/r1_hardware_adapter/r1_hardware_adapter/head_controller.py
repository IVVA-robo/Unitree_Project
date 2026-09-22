"""
Pure, dry-run head-orientation processing for the Unitree R1 stack.

This module has no ROS publishers, Unitree SDK imports, or actuator writer.
It turns a VR-head quaternion into bounded yaw/pitch/roll debug targets that a
future, separately reviewed transport may inspect.  Keeping the math here
allows all safety behaviour to be tested without a robot.
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


Axes = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
_ZERO_AXES: Axes = (0.0, 0.0, 0.0)
_EPSILON = 1.0e-9

# These ceilings deliberately live next to the pure shaping math rather than
# only in YAML.  A malformed or overly optimistic config must not be able to
# remove the diagnostic safety boundary.  They are *not* physical R1 joint
# limits: a future reviewed SDK writer must impose its own, verified limits as
# well.
_MAX_HEAD_SCALE = 2.0
_MAX_HEAD_ANGLE_LIMIT_RAD = 0.80
_MAX_HEAD_RATE_RAD_S = 2.0
_MAX_HEAD_DEADZONE_RAD = 0.20
_MAX_HEAD_STEP_DT_SEC = 0.10


@dataclass(frozen=True)
class HeadLimits:
    """
    Validated transform, filtering, and speed limits for VR head input.

    Axes use ROS/REP-103 conventions: yaw is about Z, pitch about Y, and
    roll about X.  Each configured limit is symmetric about its neutral pose.
    """

    yaw_scale: float = 1.0
    pitch_scale: float = 1.0
    roll_scale: float = 1.0
    yaw_limit_rad: float = 0.65
    pitch_limit_rad: float = 0.45
    roll_limit_rad: float = 0.20
    deadzone_rad: float = 0.025
    smoothing_alpha: float = 0.22
    max_yaw_rate_rad_s: float = 0.80
    max_pitch_rate_rad_s: float = 0.65
    max_roll_rate_rad_s: float = 0.45
    max_step_dt_sec: float = 0.10
    yaw_enabled: bool = True
    pitch_enabled: bool = True
    roll_enabled: bool = False
    invert_yaw: bool = False
    invert_pitch: bool = False
    invert_roll: bool = False

    def validate(self) -> None:
        """Reject invalid values that could remove a motion safety limit."""
        scales = (self.yaw_scale, self.pitch_scale, self.roll_scale)
        limits = (self.yaw_limit_rad, self.pitch_limit_rad, self.roll_limit_rad)
        rates = (
            self.max_yaw_rate_rad_s,
            self.max_pitch_rate_rad_s,
            self.max_roll_rate_rad_s,
        )
        finite = (
            *scales,
            *limits,
            *rates,
            self.deadzone_rad,
            self.smoothing_alpha,
            self.max_step_dt_sec,
        )
        if not all(math.isfinite(value) for value in finite):
            raise ValueError('head limits must be finite')
        if any(value < 0.0 for value in scales):
            raise ValueError('head scales must be non-negative')
        if any(value <= 0.0 for value in (*limits, *rates)):
            raise ValueError('head angle limits and rate limits must be positive')
        if self.deadzone_rad < 0.0:
            raise ValueError('head deadzone must be non-negative')
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError('head smoothing_alpha must be in (0, 1]')
        if any(value > _MAX_HEAD_SCALE for value in scales):
            raise ValueError('head scales exceed the immutable safety ceiling')
        if any(value > _MAX_HEAD_ANGLE_LIMIT_RAD for value in limits):
            raise ValueError('head angle limits exceed the immutable safety ceiling')
        if any(value > _MAX_HEAD_RATE_RAD_S for value in rates):
            raise ValueError('head rates exceed the immutable safety ceiling')
        if self.deadzone_rad > _MAX_HEAD_DEADZONE_RAD:
            raise ValueError('head deadzone exceeds the immutable safety ceiling')
        if not 0.0 < self.max_step_dt_sec <= _MAX_HEAD_STEP_DT_SEC:
            raise ValueError('head max_step_dt_sec must be in (0, 0.10]')


@dataclass(frozen=True)
class HeadDecision:
    """One safe, diagnostic-only evaluation of a VR head pose."""

    allowed: bool
    source_euler: Axes
    relative_euler: Axes
    requested_euler: Axes
    command_euler: Axes
    reason: str


class HeadController:
    """Calibrate and shape VR head orientation without commanding hardware."""

    def __init__(self, limits: HeadLimits = HeadLimits()):
        limits.validate()
        self.limits = limits
        self._neutral: Optional[Quaternion] = None
        self._filtered: Axes = _ZERO_AXES
        self._command: Axes = _ZERO_AXES
        self._last_update: Optional[float] = None

    @property
    def calibrated(self) -> bool:
        """Return whether a neutral headset orientation has been recorded."""
        return self._neutral is not None

    @property
    def command(self) -> Axes:
        """Return the most recent bounded diagnostic command."""
        return self._command

    def calibrate(self, quaternion: Sequence[float], now: float) -> None:
        """Record the supplied finite headset orientation as the neutral pose."""
        self._neutral = normalize_quaternion(quaternion)
        self._filtered = _ZERO_AXES
        self._command = _ZERO_AXES
        self._last_update = _finite_time(now)

    def reset_calibration(self, now: float) -> None:
        """Forget neutral orientation and restore a zero, safe debug command."""
        self._neutral = None
        self._filtered = _ZERO_AXES
        self._command = _ZERO_AXES
        self._last_update = _finite_time(now)

    def update(self, quaternion: Sequence[float], now: float) -> HeadDecision:
        """Convert a fresh headset quaternion to a bounded, smoothed command."""
        current = normalize_quaternion(quaternion)
        now = _finite_time(now)
        source = euler_from_quaternion(current)
        if self._neutral is None:
            return HeadDecision(
                allowed=False,
                source_euler=source,
                relative_euler=_ZERO_AXES,
                requested_euler=_ZERO_AXES,
                command_euler=_ZERO_AXES,
                reason='uncalibrated',
            )

        relative_quaternion = quaternion_multiply(
            quaternion_conjugate(self._neutral), current
        )
        relative = euler_from_quaternion(relative_quaternion)
        requested = self._shape(relative)
        self._filtered = tuple(
            previous + self.limits.smoothing_alpha * (target - previous)
            for previous, target in zip(self._filtered, requested)
        )
        elapsed = 0.0
        if self._last_update is not None:
            # A scheduler pause must not turn the next fresh pose into an
            # angle-sized jump.  Advance the rate limiter by at most one
            # bounded control interval, just as locomotion does.
            elapsed = min(
                self.limits.max_step_dt_sec,
                max(0.0, now - self._last_update),
            )
        self._last_update = now
        self._command = _rate_limit_axes(
            self._command,
            self._filtered,
            elapsed,
            (
                self.limits.max_yaw_rate_rad_s,
                self.limits.max_pitch_rate_rad_s,
                self.limits.max_roll_rate_rad_s,
            ),
        )
        reason = 'accepted'
        if self._command != requested:
            reason = 'smoothed_or_rate_limited'
        return HeadDecision(
            allowed=True,
            source_euler=source,
            relative_euler=relative,
            requested_euler=requested,
            command_euler=self._command,
            reason=reason,
        )

    def safe_fallback(
        self,
        reason: str,
        now: float,
        source_euler: Axes = _ZERO_AXES,
    ) -> HeadDecision:
        """
        Clear the debug target on a watchdog, kill, or deadman condition.

        The fallback deliberately becomes neutral immediately.  This class is
        dry-run only, and a future physical writer must independently decide
        how a robot transitions to neutral without bypassing its own limits.
        """
        self._filtered = _ZERO_AXES
        self._command = _ZERO_AXES
        self._last_update = _finite_time(now)
        return HeadDecision(
            allowed=False,
            source_euler=_axes(source_euler),
            relative_euler=_ZERO_AXES,
            requested_euler=_ZERO_AXES,
            command_euler=_ZERO_AXES,
            reason=str(reason),
        )

    def _shape(self, relative: Axes) -> Axes:
        return (
            _shape_axis(
                relative[0],
                self.limits.yaw_scale,
                self.limits.yaw_limit_rad,
                self.limits.deadzone_rad,
                self.limits.invert_yaw,
                self.limits.yaw_enabled,
            ),
            _shape_axis(
                relative[1],
                self.limits.pitch_scale,
                self.limits.pitch_limit_rad,
                self.limits.deadzone_rad,
                self.limits.invert_pitch,
                self.limits.pitch_enabled,
            ),
            _shape_axis(
                relative[2],
                self.limits.roll_scale,
                self.limits.roll_limit_rad,
                self.limits.deadzone_rad,
                self.limits.invert_roll,
                self.limits.roll_enabled,
            ),
        )


def normalize_quaternion(quaternion: Sequence[float]) -> Quaternion:
    """Return a normalized ``(x, y, z, w)`` quaternion or raise ValueError."""
    try:
        values = tuple(float(value) for value in quaternion)
    except (TypeError, ValueError) as exc:
        raise ValueError('head quaternion must contain four numeric values') from exc
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError('head quaternion must contain four finite values')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= _EPSILON:
        raise ValueError('head quaternion norm is too small')
    return tuple(value / norm for value in values)


def quaternion_conjugate(quaternion: Sequence[float]) -> Quaternion:
    """Return the conjugate of a normalized quaternion."""
    x, y, z, w = normalize_quaternion(quaternion)
    return (-x, -y, -z, w)


def quaternion_multiply(
    left: Sequence[float], right: Sequence[float]
) -> Quaternion:
    """Multiply two quaternions in ``(x, y, z, w)`` order."""
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def euler_from_quaternion(quaternion: Sequence[float]) -> Axes:
    """Return yaw, pitch, roll in radians using ROS axis conventions."""
    x, y, z, w = normalize_quaternion(quaternion)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw, pitch, roll


def _shape_axis(
    value: float,
    scale: float,
    limit: float,
    deadzone: float,
    invert: bool,
    enabled: bool,
) -> float:
    """Apply one axis's optional inversion, deadzone, scale, and clamp."""
    if not enabled:
        return 0.0
    signed = -float(value) if invert else float(value)
    magnitude = max(0.0, abs(signed) - deadzone)
    shaped = math.copysign(magnitude * scale, signed)
    return max(-limit, min(limit, shaped))


def _rate_limit_axes(
    current: Axes,
    target: Axes,
    elapsed: float,
    rates: Axes,
) -> Axes:
    """Return target motion constrained by each configured angular rate."""
    return tuple(
        previous + max(-rate * elapsed, min(rate * elapsed, desired - previous))
        for previous, desired, rate in zip(current, target, rates)
    )


def _axes(values: Sequence[float]) -> Axes:
    """Convert an arbitrary three-value sequence to finite yaw/pitch/roll."""
    try:
        converted = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError('head axes must contain three numeric values') from exc
    if len(converted) != 3 or not all(math.isfinite(value) for value in converted):
        raise ValueError('head axes must contain three finite values')
    return converted


def _finite_time(value: float) -> float:
    """Return a finite monotonic timestamp or raise ValueError."""
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError('head timestamp must be finite')
    return converted
