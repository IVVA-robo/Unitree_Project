"""
Pure exhibition session and tracking-recovery policy.

The default bridge still uses the physical Deadman bit.  This module is used
only when the launch explicitly selects ``activation_mode=session_arm``.
It contains no ROS or robot SDK code, which keeps the reconnect behavior easy
to exercise with deterministic unit tests.
"""

from dataclasses import dataclass
import math
from typing import Dict, Mapping, Optional

from .protocol import Pose, TrackingAvailability


_DEVICES = ('head', 'left', 'right')


class DebouncedButton:
    """Turn a noisy level into a stable level and one-shot edge information."""

    def __init__(self, debounce_sec: float = 0.05):
        if not math.isfinite(debounce_sec) or not 0.01 <= debounce_sec <= 0.25:
            raise ValueError('debounce_sec must be in [0.01, 0.25]')
        self.debounce_sec = float(debounce_sec)
        self._stable = False
        self._candidate = False
        self._candidate_since: Optional[float] = None

    @property
    def active(self) -> bool:
        return self._stable

    def update(self, raw: bool, now: float) -> tuple:
        """Return ``(stable_level, changed)`` after the bounded debounce."""
        if type(raw) is not bool or not math.isfinite(now):
            raise ValueError('button level must be bool and now must be finite')
        if raw == self._stable:
            self._candidate = raw
            self._candidate_since = None
            return self._stable, False
        if raw != self._candidate or self._candidate_since is None:
            self._candidate = raw
            self._candidate_since = float(now)
            return self._stable, False
        if now - self._candidate_since < self.debounce_sec:
            return self._stable, False
        self._stable = raw
        self._candidate_since = None
        return self._stable, True


class EmergencyStopButton:
    """Trigger immediately on press and debounce release before re-arming."""

    def __init__(self, release_debounce_sec: float = 0.05):
        if (
            not math.isfinite(release_debounce_sec)
            or not 0.01 <= release_debounce_sec <= 0.25
        ):
            raise ValueError('release_debounce_sec must be in [0.01, 0.25]')
        self.release_debounce_sec = float(release_debounce_sec)
        self._pressed = False
        self._release_since: Optional[float] = None

    @property
    def active(self) -> bool:
        return self._pressed

    def update(self, raw: bool, now: float) -> bool:
        """Return true once per deliberate press; a held button never repeats."""
        if type(raw) is not bool or not math.isfinite(now):
            raise ValueError('button level must be bool and now must be finite')
        if raw:
            self._release_since = None
            if not self._pressed:
                self._pressed = True
                return True
            return False
        if not self._pressed:
            return False
        if self._release_since is None:
            self._release_since = float(now)
        elif now - self._release_since + 1.0e-12 >= self.release_debounce_sec:
            self._pressed = False
            self._release_since = None
        return False


@dataclass(frozen=True)
class SessionDecision:
    """Current output authorization and operator-facing tracking state."""

    control_active: bool
    locomotion_active: bool
    hold_left: bool
    hold_right: bool
    phase: str
    missing: tuple


class ExhibitionSessionGate:
    """
    Keep one explicitly armed exhibition session through VR dropouts.

    Session authorization is memory-only and starts disarmed on every process
    start.  Tracking loss never grants authorization: it only changes an
    already armed session from live tracking to a held arm pose and zero
    locomotion.  Robot feedback, SDK, KILL, and motor-health checks remain in
    the downstream live writer and are intentionally outside this policy.
    """

    def __init__(self, grace_period_sec: float = 2.0):
        if not math.isfinite(grace_period_sec) or not 1.0 <= grace_period_sec <= 3.0:
            raise ValueError('grace_period_sec must be in [1, 3]')
        self.grace_period_sec = float(grace_period_sec)
        self._armed = False
        self._paused = False
        self._locomotion_neutral = False
        self._locomotion_rearm_required = True
        self._tracking = TrackingAvailability.unavailable(reported=False)
        self._missing_since: Dict[str, Optional[float]] = {
            device: None for device in _DEVICES
        }

    @property
    def armed(self) -> bool:
        """Return whether the operator explicitly armed this process session."""
        return self._armed

    @property
    def tracking(self) -> TrackingAvailability:
        """Return the newest reported per-device tracking availability."""
        return self._tracking

    @property
    def paused(self) -> bool:
        """Return whether control is deliberately paused for calibration."""
        return self._paused

    @property
    def pose_tracking(self) -> TrackingAvailability:
        """
        Return tracking to use for the pose-recovery output.

        Calibration pause deliberately keeps the session latch armed so the
        physical writer retains its already reviewed authority.  The writer
        therefore cannot use ``/vr/teleop/active`` as a pause signal.  Freeze
        every pose at the bridge while paused, while ``tracking`` continues to
        record real XR availability for the guarded resume operation.
        """
        if self._paused:
            return TrackingAvailability.unavailable(reported=True)
        return self._tracking

    def observe(
        self,
        tracking: TrackingAvailability,
        now: float,
        locomotion_neutral: bool = False,
    ) -> None:
        """Record one current packet's tracking flags."""
        if not math.isfinite(now):
            raise ValueError('now must be finite')
        self._tracking = tracking
        self._locomotion_neutral = bool(locomotion_neutral)
        if self._armed and not tracking.all_available:
            # Any XR dropout revokes joystick motion until a complete fresh
            # tracking packet is observed with all used axes neutral.  Arms
            # may keep their session authorization and recover independently.
            self._locomotion_rearm_required = True
        elif (
            self._armed
            and tracking.all_available
            and self._locomotion_neutral
        ):
            self._locomotion_rearm_required = False
        for device in _DEVICES:
            available = tracking.reported and bool(getattr(tracking, device))
            if available:
                self._missing_since[device] = None
            elif self._missing_since[device] is None:
                self._missing_since[device] = float(now)

    def observe_packet_timeout(self, now: float) -> None:
        """Treat a missing UDP heartbeat as loss of all three XR devices."""
        self.observe(TrackingAvailability.unavailable(reported=True), now)

    def arm(self, snapshot_ready: bool) -> tuple:
        """Arm only from a complete, explicitly reported tracking snapshot."""
        if not self._tracking.reported:
            return False, 'tracking_flags_missing_update_vr_app'
        if not self._tracking.all_available:
            return False, 'tracking_incomplete:' + ','.join(self._tracking.missing)
        if not snapshot_ready:
            return False, 'safe_pose_snapshot_incomplete'
        if not self._locomotion_neutral:
            return False, 'locomotion_controls_not_neutral'
        self._armed = True
        self._paused = False
        self._locomotion_rearm_required = False
        return True, 'session_armed'

    def disarm(self) -> None:
        """Remove session authorization immediately."""
        self._armed = False
        self._paused = False
        self._locomotion_rearm_required = True

    def pause(self) -> tuple:
        """Pause commands without dropping the writer's session latch."""
        if not self._armed:
            return False, 'session_not_armed'
        self._paused = True
        self._locomotion_rearm_required = True
        return True, 'session_paused'

    def resume(self, snapshot_ready: bool) -> tuple:
        """Resume a paused session only from fresh, neutral tracking."""
        if not self._armed:
            return False, 'session_not_armed'
        if not self._paused:
            return True, 'session_already_running'
        if not self._tracking.all_available:
            return False, 'tracking_incomplete:' + ','.join(self._tracking.missing)
        if not snapshot_ready:
            return False, 'safe_pose_snapshot_incomplete'
        if not self._locomotion_neutral:
            return False, 'locomotion_controls_not_neutral'
        self._paused = False
        self._locomotion_rearm_required = False
        return True, 'session_resumed'

    def decision(self, now: float, snapshot_ready: bool) -> SessionDecision:
        """Return hold/zero behavior without changing session authorization."""
        if not math.isfinite(now):
            raise ValueError('now must be finite')
        missing = self._tracking.missing if self._tracking.reported else _DEVICES
        complete = not missing
        control_active = self._armed and snapshot_ready and not self._paused
        locomotion_active = (
            control_active and complete and not self._locomotion_rearm_required
        )

        if not self._armed:
            phase = 'disarmed'
        elif self._paused:
            phase = 'paused'
        elif not snapshot_ready:
            phase = 'awaiting_safe_snapshot'
        elif complete:
            phase = (
                'awaiting_neutral'
                if self._locomotion_rearm_required else 'tracking'
            )
        else:
            oldest_loss = max(
                0.0,
                max(
                    now - self._missing_since[device]
                    for device in missing
                    if self._missing_since[device] is not None
                ),
            )
            phase = (
                'reconnecting' if oldest_loss <= self.grace_period_sec
                else 'holding'
            )

        head_missing = 'head' in missing
        return SessionDecision(
            control_active=control_active,
            locomotion_active=locomotion_active,
            hold_left=self._paused or head_missing or 'left' in missing,
            hold_right=self._paused or head_missing or 'right' in missing,
            phase=phase,
            missing=tuple(missing),
        )


class PoseRecoveryFilter:
    """Hold missing poses and blend tracking back in without a target jump."""

    def __init__(self, blend_duration_sec: float = 0.5):
        if (
            not math.isfinite(blend_duration_sec)
            or not 0.1 <= blend_duration_sec <= 2.0
        ):
            raise ValueError('blend_duration_sec must be in [0.1, 2.0]')
        self.blend_duration_sec = float(blend_duration_sec)
        self._output: Dict[str, Pose] = {}
        self._was_available = {device: False for device in _DEVICES}
        self._recovery_start: Dict[str, float] = {}
        self._recovery_origin: Dict[str, Pose] = {}

    @property
    def ready(self) -> bool:
        """Return true after at least one safe pose exists for every device."""
        return all(device in self._output for device in _DEVICES)

    def update(
        self,
        raw: Mapping[str, Pose],
        tracking: TrackingAvailability,
        now: float,
    ) -> Dict[str, Pose]:
        """Update live devices, hold missing devices, and blend reconnects."""
        if not math.isfinite(now):
            raise ValueError('now must be finite')
        # Freeze the body reference whenever either hand disappears. Holding
        # a missing controller in world coordinates while the HMD keeps moving
        # would still move that robot arm after headset-relative conversion.
        head_available = tracking.all_available
        for device in _DEVICES:
            if device == 'head':
                available = head_available and device in raw
            else:
                available = (
                    tracking.reported
                    and tracking.head
                    and bool(getattr(tracking, device))
                    and device in raw
                )
            if not available:
                self._was_available[device] = False
                self._recovery_start.pop(device, None)
                self._recovery_origin.pop(device, None)
                continue

            target = raw[device]
            if device not in self._output:
                self._output[device] = target
                self._was_available[device] = True
                continue

            if not self._was_available[device]:
                self._recovery_start[device] = float(now)
                self._recovery_origin[device] = self._output[device]

            started = self._recovery_start.get(device)
            if started is None:
                self._output[device] = target
            else:
                fraction = max(
                    0.0,
                    min(1.0, (now - started) / self.blend_duration_sec),
                )
                self._output[device] = _blend_pose(
                    self._recovery_origin[device], target, fraction
                )
                if fraction >= 1.0:
                    self._recovery_start.pop(device, None)
                    self._recovery_origin.pop(device, None)
            self._was_available[device] = True
        return dict(self._output)


def _blend_pose(origin: Pose, target: Pose, fraction: float) -> Pose:
    """Linearly blend position and shortest-path normalize quaternion."""
    position = tuple(
        start + fraction * (end - start)
        for start, end in zip(origin.position, target.position)
    )
    dot = sum(
        start * end
        for start, end in zip(origin.orientation, target.orientation)
    )
    target_orientation = (
        tuple(-value for value in target.orientation)
        if dot < 0.0 else target.orientation
    )
    orientation = [
        start + fraction * (end - start)
        for start, end in zip(origin.orientation, target_orientation)
    ]
    norm = math.sqrt(sum(value * value for value in orientation))
    if norm < 1.0e-9:
        orientation = list(origin.orientation)
        norm = math.sqrt(sum(value * value for value in orientation))
    return Pose(
        position=position,
        orientation=tuple(value / norm for value in orientation),
    )
