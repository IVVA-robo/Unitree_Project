"""
Pure, dry-run locomotion gating and rate limiting.

This module deliberately contains no ROS, DDS, Unitree SDK, or hardware
transport code.  It turns a VR velocity intent into a bounded *debug* command
that a future, separately reviewed high-level locomotion writer may inspect.
Keeping the state machine here makes all fail-safe behaviour unit-testable
without a robot attached.
"""

from dataclasses import dataclass
import math
from typing import Dict, Optional, Tuple


Velocity = Tuple[float, float, float]

# Immutable dry-run ceilings.  YAML can choose a more conservative profile,
# but cannot turn this diagnostic controller into an unbounded command source.
# A future physical LocoClient transport must keep a separate, reviewed set of
# limits tied to the actual R1 firmware and commissioning procedure.
_MAX_FORWARD_MPS = 0.55
_MAX_LATERAL_MPS = 0.30
_MAX_YAW_RPS = 0.90
_MAX_LINEAR_ACCELERATION_MPS2 = 0.60
_MAX_LINEAR_DECELERATION_MPS2 = 1.00
_MAX_YAW_ACCELERATION_RPS2 = 1.10
_MAX_YAW_DECELERATION_RPS2 = 1.80
_MAX_DEADMAN_TIMEOUT_SEC = 2.0
_MAX_COMMAND_TIMEOUT_SEC = 0.50
_MAX_STEP_DT_SEC = 0.10


@dataclass(frozen=True)
class LocomotionProfile:
    """One bounded velocity and ramp profile."""

    name: str
    max_forward_mps: float
    max_lateral_mps: float
    max_yaw_rps: float
    acceleration_mps2: float
    deceleration_mps2: float
    yaw_acceleration_rps2: float
    yaw_deceleration_rps2: float

    def validate(self) -> None:
        """Reject invalid or non-conservative profile values."""
        values = (
            self.max_forward_mps,
            self.max_lateral_mps,
            self.max_yaw_rps,
            self.acceleration_mps2,
            self.deceleration_mps2,
            self.yaw_acceleration_rps2,
            self.yaw_deceleration_rps2,
        )
        if not self.name:
            raise ValueError('locomotion profile name is required')
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('locomotion profile values must be finite and positive')
        ceilings = (
            _MAX_FORWARD_MPS,
            _MAX_LATERAL_MPS,
            _MAX_YAW_RPS,
            _MAX_LINEAR_ACCELERATION_MPS2,
            _MAX_LINEAR_DECELERATION_MPS2,
            _MAX_YAW_ACCELERATION_RPS2,
            _MAX_YAW_DECELERATION_RPS2,
        )
        if any(value > ceiling for value, ceiling in zip(values, ceilings)):
            raise ValueError('locomotion profile exceeds immutable safety ceiling')


@dataclass(frozen=True)
class LocomotionSafetyConfig:
    """Time bounds shared by every locomotion profile."""

    # vr_teleop_bridge republishes an unchanged deadman state at 1 Hz.  Leave
    # margin over that heartbeat instead of treating a held button as stale.
    deadman_timeout_sec: float = 1.50
    command_timeout_sec: float = 0.25
    max_step_dt_sec: float = 0.10

    def validate(self) -> None:
        """Ensure a scheduler delay cannot disable a safety path."""
        values = (
            self.deadman_timeout_sec,
            self.command_timeout_sec,
            self.max_step_dt_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('locomotion safety values must be finite and positive')
        if self.deadman_timeout_sec > _MAX_DEADMAN_TIMEOUT_SEC:
            raise ValueError('deadman timeout exceeds immutable safety ceiling')
        if self.command_timeout_sec > _MAX_COMMAND_TIMEOUT_SEC:
            raise ValueError('command timeout exceeds immutable safety ceiling')
        if self.max_step_dt_sec > _MAX_STEP_DT_SEC:
            raise ValueError('locomotion max_step_dt_sec exceeds immutable safety ceiling')


@dataclass(frozen=True)
class LocomotionInputDecision:
    """Result of accepting or rejecting an incoming velocity intent."""

    accepted: bool
    target: Velocity
    reason: str


@dataclass(frozen=True)
class LocomotionOutput:
    """The current debug-only command and its safety state."""

    velocity: Velocity
    target: Velocity
    mode: str
    kill: str
    deadman: str
    watchdog: str
    reason: str


def default_profiles() -> Dict[str, LocomotionProfile]:
    """
    Return conservative built-in profiles for dry-run review.

    ``exhibition`` is still bounded and remains dry-run only.  Its values are
    intentionally not a recommendation for a physical R1 commissioning run.
    """
    return {
        'slow-safe': LocomotionProfile(
            name='slow-safe',
            max_forward_mps=0.20,
            max_lateral_mps=0.12,
            max_yaw_rps=0.35,
            acceleration_mps2=0.25,
            deceleration_mps2=0.45,
            yaw_acceleration_rps2=0.50,
            yaw_deceleration_rps2=0.90,
        ),
        'normal': LocomotionProfile(
            name='normal',
            max_forward_mps=0.40,
            max_lateral_mps=0.22,
            max_yaw_rps=0.65,
            acceleration_mps2=0.45,
            deceleration_mps2=0.75,
            yaw_acceleration_rps2=0.85,
            yaw_deceleration_rps2=1.40,
        ),
        'exhibition': LocomotionProfile(
            name='exhibition',
            max_forward_mps=0.55,
            max_lateral_mps=0.30,
            max_yaw_rps=0.90,
            acceleration_mps2=0.60,
            deceleration_mps2=1.00,
            yaw_acceleration_rps2=1.10,
            yaw_deceleration_rps2=1.80,
        ),
    }


class DryRunLocomotionController:
    """
    Fail-closed high-level locomotion state machine.

    It starts in a killed state.  A received ``False`` kill signal explicitly
    clears the startup interlock.  A later ``True`` signal latches the kill;
    it then needs a separate reset after the source reports ``False``.  This
    prevents a transient kill input from silently re-enabling a stale command.
    """

    def __init__(
        self,
        config: LocomotionSafetyConfig = LocomotionSafetyConfig(),
        profiles: Optional[Dict[str, LocomotionProfile]] = None,
        mode: str = 'slow-safe',
    ):
        config.validate()
        self._config = config
        self._profiles = dict(default_profiles() if profiles is None else profiles)
        if not self._profiles:
            raise ValueError('at least one locomotion profile is required')
        for profile_name, profile in self._profiles.items():
            profile.validate()
            if profile_name != profile.name:
                raise ValueError('profile dictionary keys must match profile names')
        if mode not in self._profiles:
            raise ValueError('unsupported locomotion mode: %s' % mode)

        self._mode = mode
        self._awaiting_kill_clear = True
        self._kill_signal_seen = False
        self._kill_signal_active = True
        self._kill_latched = False
        self._deadman_active = False
        self._deadman_time: Optional[float] = None
        self._last_command_time: Optional[float] = None
        self._target: Velocity = (0.0, 0.0, 0.0)
        self._output: Velocity = (0.0, 0.0, 0.0)
        self._last_step_time: Optional[float] = None
        self._last_reason = 'awaiting_kill_clear'

    @property
    def mode(self) -> str:
        """Return the currently selected profile name."""
        return self._mode

    @property
    def profile(self) -> LocomotionProfile:
        """Return the currently selected bounded profile."""
        return self._profiles[self._mode]

    @property
    def output(self) -> Velocity:
        """Return the most recently calculated debug velocity."""
        return self._output

    def set_mode(self, mode: str) -> bool:
        """Select a known profile without bypassing clamp or slew limits."""
        if mode not in self._profiles:
            return False
        self._mode = mode
        self._target = _clamp_velocity(self._target, self.profile)
        # Reducing a profile must never leave a prior, higher mode's output in
        # flight until the next timer tick.  Raising a profile does not create
        # a jump because the current output is already within its new bounds.
        self._output = _clamp_velocity(self._output, self.profile)
        self._last_reason = 'mode_changed'
        return True

    def set_emergency_stop(self, active: bool) -> str:
        """Record the physical/software kill signal and apply its interlock."""
        self._kill_signal_seen = True
        self._kill_signal_active = bool(active)
        if active:
            self._target = (0.0, 0.0, 0.0)
            self._output = (0.0, 0.0, 0.0)
            # The supervisor intentionally starts by publishing a retained
            # ``true``.  That sample establishes the fail-closed startup
            # state; it is not an emergency edge that should require a second
            # reset after the operator later requests the first clear.  Once
            # startup has been explicitly cleared, every later true sample is
            # a real stop event and remains latched until reset.
            if self._awaiting_kill_clear:
                self._last_reason = 'startup_kill_active'
            else:
                self._kill_latched = True
                self._last_reason = 'emergency_stop'
            return self.kill_state
        if self._awaiting_kill_clear:
            self._awaiting_kill_clear = False
            self._last_reason = 'kill_startup_interlock_cleared'
        elif self._kill_latched:
            self._last_reason = 'kill_latched_waiting_for_reset'
        else:
            self._last_reason = 'kill_signal_clear'
        return self.kill_state

    def reset_emergency_stop(self) -> Tuple[bool, str]:
        """Clear a latched kill only after an explicit clear signal was seen."""
        if not self._kill_signal_seen:
            return False, 'kill_signal_not_received'
        if self._kill_signal_active:
            return False, 'kill_signal_still_active'
        self._awaiting_kill_clear = False
        self._kill_latched = False
        self._target = (0.0, 0.0, 0.0)
        self._last_command_time = None
        self._last_reason = 'kill_reset'
        return True, 'kill_reset'

    @property
    def kill_state(self) -> str:
        """Return a diagnostic state for the kill gate."""
        if self._awaiting_kill_clear:
            return 'awaiting_explicit_clear'
        if self._kill_signal_active:
            return 'active'
        if self._kill_latched:
            return 'latched'
        return 'clear'

    def is_killed(self) -> bool:
        """Return whether the kill gate blocks all velocity intents."""
        return self.kill_state != 'clear'

    def set_deadman(self, active: bool, now: float) -> None:
        """Update deadman heartbeat; a false edge zeros output immediately."""
        self._deadman_active = bool(active)
        self._deadman_time = float(now)
        if not self._deadman_active:
            self._target = (0.0, 0.0, 0.0)
            self._output = (0.0, 0.0, 0.0)
            self._last_command_time = None
            self._last_reason = 'deadman_released'

    def deadman_state(self, now: float) -> str:
        """Return active, released, missing, or stale for status output."""
        if self._deadman_time is None:
            return 'missing'
        if not self._deadman_active:
            return 'released'
        if float(now) - self._deadman_time > self._config.deadman_timeout_sec:
            return 'stale'
        return 'active'

    def deadman_is_active(self, now: float) -> bool:
        """Return whether the deadman is asserted and freshly received."""
        return self.deadman_state(now) == 'active'

    def receive_velocity(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        now: float,
    ) -> LocomotionInputDecision:
        """Validate, clamp, and retain a fresh VR velocity intent."""
        values = (float(linear_x), float(linear_y), float(angular_z))
        if not all(math.isfinite(value) for value in values):
            self._target = (0.0, 0.0, 0.0)
            self._output = (0.0, 0.0, 0.0)
            self._last_command_time = None
            self._last_reason = 'nonfinite_velocity'
            return LocomotionInputDecision(False, self._target, self._last_reason)
        if self.is_killed():
            self._target = (0.0, 0.0, 0.0)
            self._output = (0.0, 0.0, 0.0)
            self._last_command_time = None
            self._last_reason = 'emergency_stop_%s' % self.kill_state
            return LocomotionInputDecision(False, self._target, self._last_reason)
        if not self.deadman_is_active(now):
            self._target = (0.0, 0.0, 0.0)
            self._output = (0.0, 0.0, 0.0)
            self._last_command_time = None
            self._last_reason = 'deadman_%s' % self.deadman_state(now)
            return LocomotionInputDecision(False, self._target, self._last_reason)

        clamped = _clamp_velocity(values, self.profile)
        self._target = clamped
        self._last_command_time = float(now)
        self._last_reason = (
            'velocity_clamped' if clamped != values else 'accepted'
        )
        return LocomotionInputDecision(True, clamped, self._last_reason)

    def watchdog_state(self, now: float) -> str:
        """Return the velocity-input watchdog state for diagnostics."""
        if self.is_killed():
            return 'blocked_by_kill'
        if not self.deadman_is_active(now):
            return 'blocked_by_deadman'
        if self._last_command_time is None:
            return 'missing'
        if float(now) - self._last_command_time > self._config.command_timeout_sec:
            return 'expired'
        return 'fresh'

    def step(self, now: float) -> LocomotionOutput:
        """Advance the bounded output state at the caller's timer rate."""
        now = float(now)
        if self._last_step_time is None:
            dt = 0.0
        else:
            dt = min(
                self._config.max_step_dt_sec,
                max(0.0, now - self._last_step_time),
            )
        self._last_step_time = now

        kill = self.kill_state
        deadman = self.deadman_state(now)
        watchdog = self.watchdog_state(now)
        if kill != 'clear':
            self._target = (0.0, 0.0, 0.0)
            self._output = (0.0, 0.0, 0.0)
            reason = 'emergency_stop_%s' % kill
        elif deadman != 'active':
            self._target = (0.0, 0.0, 0.0)
            # Deadman loss is a safety edge, not an ordinary stick release.
            # A future high-level writer must turn this into an immediate
            # zero/StopMove request rather than use a visual smoothing ramp.
            self._output = (0.0, 0.0, 0.0)
            reason = 'deadman_%s' % deadman
        elif watchdog != 'fresh':
            self._target = (0.0, 0.0, 0.0)
            # The same applies to missing/upstream-stale velocity intent.
            self._output = (0.0, 0.0, 0.0)
            reason = 'command_watchdog_%s' % watchdog
        else:
            profile = self.profile
            self._output = _slew_velocity(
                self._output,
                self._target,
                profile.acceleration_mps2,
                profile.deceleration_mps2,
                dt,
                profile.yaw_acceleration_rps2,
                profile.yaw_deceleration_rps2,
            )
            reason = self._last_reason
        self._last_reason = reason
        return LocomotionOutput(
            velocity=self._output,
            target=self._target,
            mode=self._mode,
            kill=kill,
            deadman=deadman,
            watchdog=watchdog,
            reason=reason,
        )


def format_status(output: LocomotionOutput) -> str:
    """Format a compact, machine-readable dry-run status line."""
    target = output.target
    velocity = output.velocity
    return (
        'dry_run=true hardware_enabled=false mode=%s kill=%s deadman=%s '
        'watchdog=%s reason=%s target=(%.3f,%.3f,%.3f) '
        'output=(%.3f,%.3f,%.3f)'
        % (
            output.mode,
            output.kill,
            output.deadman,
            output.watchdog,
            output.reason,
            target[0],
            target[1],
            target[2],
            velocity[0],
            velocity[1],
            velocity[2],
        )
    )


def require_dry_run(hardware_enabled: bool, dry_run: bool) -> None:
    """Fail closed because this package has no reviewed hardware writer."""
    if bool(hardware_enabled) or not bool(dry_run):
        raise RuntimeError(
            'r1_locomotion_dry_run has no hardware transport; '
            'hardware_enabled=false and dry_run=true are required'
        )


def _clamp_velocity(values: Velocity, profile: LocomotionProfile) -> Velocity:
    """Symmetrically clamp forward, lateral, and yaw intent."""
    return (
        _clamp(values[0], profile.max_forward_mps),
        _clamp(values[1], profile.max_lateral_mps),
        _clamp(values[2], profile.max_yaw_rps),
    )


def _slew_velocity(
    current: Velocity,
    target: Velocity,
    linear_acceleration: float,
    linear_deceleration: float,
    dt: float,
    yaw_acceleration: Optional[float] = None,
    yaw_deceleration: Optional[float] = None,
) -> Velocity:
    """Rate-limit all axes while avoiding an overshoot through zero."""
    yaw_acceleration = (
        linear_acceleration if yaw_acceleration is None else yaw_acceleration
    )
    yaw_deceleration = (
        linear_deceleration if yaw_deceleration is None else yaw_deceleration
    )
    return (
        _slew(current[0], target[0], linear_acceleration, linear_deceleration, dt),
        _slew(current[1], target[1], linear_acceleration, linear_deceleration, dt),
        _slew(current[2], target[2], yaw_acceleration, yaw_deceleration, dt),
    )


def _slew(
    current: float,
    target: float,
    acceleration: float,
    deceleration: float,
    dt: float,
) -> float:
    """Move one scalar towards its target with directional ramp selection."""
    if current == target or dt <= 0.0:
        return current
    increasing_same_direction = (
        current * target >= 0.0 and abs(target) > abs(current)
    )
    rate = acceleration if increasing_same_direction else deceleration
    maximum_delta = rate * dt
    delta = _clamp(target - current, maximum_delta)
    return current + delta


def _clamp(value: float, magnitude: float) -> float:
    """Clamp a scalar symmetrically about zero."""
    return max(-magnitude, min(magnitude, value))
