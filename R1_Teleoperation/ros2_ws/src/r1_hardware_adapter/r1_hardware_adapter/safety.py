"""
Pure-Python safety gates used before any future hardware transport.

This module deliberately knows nothing about the Unitree SDK.  Keeping the
gate independent makes it possible to test all stop conditions without a
robot, DDS writer, or motor controller.
"""

from dataclasses import dataclass
import math
import time
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class SafetyLimits:
    """Limits applied by the adapter boundary."""

    active_timeout_sec: float = 1.5
    command_timeout_sec: float = 0.25
    max_forward_mps: float = 0.35
    max_lateral_mps: float = 0.25
    max_yaw_rps: float = 0.60
    max_joint_step_rad: float = 0.15
    max_arm_joints: int = 10

    def validate(self) -> None:
        """Reject a limit set that could disable the watchdog."""
        positive = (
            self.active_timeout_sec,
            self.command_timeout_sec,
            self.max_forward_mps,
            self.max_lateral_mps,
            self.max_yaw_rps,
            self.max_joint_step_rad,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('all safety limits must be finite and positive')
        if self.max_arm_joints <= 0:
            raise ValueError('max_arm_joints must be positive')


@dataclass(frozen=True)
class ArmDecision:
    """Result of validating one arm trajectory point."""

    accepted: bool
    positions: Tuple[float, ...]
    reason: str


@dataclass(frozen=True)
class VelocityDecision:
    """Result of validating one velocity command."""

    accepted: bool
    linear_x: float
    linear_y: float
    angular_z: float
    reason: str


class SafetyGate:
    """
    Deadman, freshness, finite-value, and slew gates.

    The gate stores no ROS messages and emits no network traffic.  A caller
    must still keep the returned values in a dry-run/debug channel until a
    separately reviewed Unitree transport is implemented.
    """

    def __init__(self, limits: SafetyLimits = SafetyLimits()):
        limits.validate()
        self.limits = limits
        self._active = False
        self._active_time: Optional[float] = None
        self._last_arm_time: Optional[float] = None
        self._last_velocity_time: Optional[float] = None
        self._last_arm_positions: Optional[Tuple[float, ...]] = None

    @property
    def active_flag(self) -> bool:
        """Return the last received Deadman value."""
        return self._active

    def set_active(self, active: bool, now: Optional[float] = None) -> None:
        """Update Deadman state and clear command history on release."""
        now = time.monotonic() if now is None else float(now)
        self._active = bool(active)
        self._active_time = now
        if not self._active:
            self._last_arm_time = None
            self._last_velocity_time = None
            self._last_arm_positions = None

    def is_active(self, now: Optional[float] = None) -> bool:
        """Return whether Deadman is true and its heartbeat is fresh."""
        if not self._active or self._active_time is None:
            return False
        now = time.monotonic() if now is None else float(now)
        return now - self._active_time <= self.limits.active_timeout_sec

    def arm(self, positions: Sequence[float], now: Optional[float] = None) -> ArmDecision:
        """Validate and slew-limit a trajectory point."""
        now = time.monotonic() if now is None else float(now)
        values = tuple(float(value) for value in positions)
        if not self.is_active(now):
            return ArmDecision(False, tuple(), 'deadman_inactive_or_stale')
        if not values:
            return ArmDecision(False, tuple(), 'empty_trajectory')
        if len(values) > self.limits.max_arm_joints:
            return ArmDecision(False, tuple(), 'too_many_arm_joints')
        if not all(math.isfinite(value) for value in values):
            return ArmDecision(False, tuple(), 'nonfinite_joint_position')

        reason = 'accepted'
        if self._last_arm_positions is not None:
            if len(self._last_arm_positions) != len(values):
                return ArmDecision(False, tuple(), 'joint_count_changed')
            step = self.limits.max_joint_step_rad
            limited = tuple(
                previous + max(-step, min(step, value - previous))
                for previous, value in zip(self._last_arm_positions, values)
            )
            if limited != values:
                reason = 'joint_step_clamped'
            values = limited
        self._last_arm_positions = values
        self._last_arm_time = now
        return ArmDecision(True, values, reason)

    def velocity(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        now: Optional[float] = None,
    ) -> VelocityDecision:
        """Validate and symmetrically clamp a velocity command."""
        now = time.monotonic() if now is None else float(now)
        values = (float(linear_x), float(linear_y), float(angular_z))
        if not self.is_active(now):
            return VelocityDecision(False, 0.0, 0.0, 0.0, 'deadman_inactive_or_stale')
        if not all(math.isfinite(value) for value in values):
            return VelocityDecision(False, 0.0, 0.0, 0.0, 'nonfinite_velocity')
        limited = (
            _clamp(values[0], self.limits.max_forward_mps),
            _clamp(values[1], self.limits.max_lateral_mps),
            _clamp(values[2], self.limits.max_yaw_rps),
        )
        reason = 'accepted' if limited == values else 'velocity_clamped'
        self._last_velocity_time = now
        return VelocityDecision(True, *limited, reason)

    def watchdog_expired(self, now: Optional[float] = None) -> bool:
        """Return true when a fresh velocity command is not available."""
        now = time.monotonic() if now is None else float(now)
        if not self.is_active(now):
            return True
        if self._last_velocity_time is None:
            return True
        return now - self._last_velocity_time > self.limits.command_timeout_sec


def _clamp(value: float, magnitude: float) -> float:
    """Clamp a scalar symmetrically around zero."""
    return max(-magnitude, min(magnitude, value))
