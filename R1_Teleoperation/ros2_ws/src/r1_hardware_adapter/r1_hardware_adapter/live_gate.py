"""
Transport-agnostic, fail-closed gate for a future R1 live command writer.

This module contains only deterministic state and validation logic.  It does
not import ROS, DDS, a Unitree SDK, or any transport client.  A future writer
must interpret :class:`GateDecision` and independently keep its own physical
emergency-stop and communication safeguards.
"""

from dataclasses import dataclass
from enum import Enum
import math
import os
import time
from typing import Mapping, Optional, Tuple


Velocity = Tuple[float, float, float]
_TRUE_VALUES = frozenset(('1', 'true', 'yes', 'on'))
_FALSE_VALUES = frozenset(('0', 'false', 'no', 'off', ''))


class GateAction(str, Enum):
    """A side-effect-free action a separately reviewed writer may interpret."""

    HOLD = 'hold'
    STOP = 'stop'
    VELOCITY = 'velocity'


@dataclass(frozen=True)
class LiveAuthorization:
    """Explicit human acknowledgements required before a live transport exists."""

    dry_run: bool = True
    enable_actuation: bool = False
    confirm_off_charger: bool = False
    confirm_clear_area: bool = False
    confirm_estop_ready: bool = False
    environment_errors: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        """Reject ambiguous manually constructed authorization values."""
        values = (
            self.dry_run,
            self.enable_actuation,
            self.confirm_off_charger,
            self.confirm_clear_area,
            self.confirm_estop_ready,
        )
        if not all(type(value) is bool for value in values):
            raise ValueError('live authorization values must be booleans')

    @classmethod
    def from_environment(
        cls, environment: Optional[Mapping[str, object]] = None
    ) -> 'LiveAuthorization':
        """Read the required environment interlocks with fail-closed defaults."""
        values = os.environ if environment is None else environment
        dry_run, dry_run_error = _environment_flag(
            'ROBOT_DRY_RUN', True, values
        )
        enable_actuation, enable_error = _environment_flag(
            'ROBOT_ENABLE_ACTUATION', False, values
        )
        off_charger, charger_error = _environment_flag(
            'ROBOT_CONFIRM_OFF_CHARGER', False, values
        )
        clear_area, area_error = _environment_flag(
            'ROBOT_CONFIRM_CLEAR_AREA', False, values
        )
        estop_ready, estop_error = _environment_flag(
            'ROBOT_CONFIRM_ESTOP_READY', False, values
        )
        errors = tuple(
            error
            for error in (
                dry_run_error,
                enable_error,
                charger_error,
                area_error,
                estop_error,
            )
            if error is not None
        )
        return cls(
            dry_run=dry_run,
            enable_actuation=enable_actuation,
            confirm_off_charger=off_charger,
            confirm_clear_area=clear_area,
            confirm_estop_ready=estop_ready,
            environment_errors=errors,
        )

    @property
    def live_authorized(self) -> bool:
        """Return true only when every explicit live interlock is present."""
        return (
            not self.environment_errors
            and not self.dry_run
            and self.enable_actuation
            and self.confirm_off_charger
            and self.confirm_clear_area
            and self.confirm_estop_ready
        )

    def missing_interlocks(self) -> Tuple[str, ...]:
        """Return stable diagnostics for every missing live authorization."""
        missing = list(self.environment_errors)
        if self.dry_run:
            missing.append('ROBOT_DRY_RUN=0')
        if not self.enable_actuation:
            missing.append('ROBOT_ENABLE_ACTUATION=1')
        if not self.confirm_off_charger:
            missing.append('ROBOT_CONFIRM_OFF_CHARGER=1')
        if not self.confirm_clear_area:
            missing.append('ROBOT_CONFIRM_CLEAR_AREA=1')
        if not self.confirm_estop_ready:
            missing.append('ROBOT_CONFIRM_ESTOP_READY=1')
        return tuple(missing)


@dataclass(frozen=True)
class LiveGateLimits:
    """Conservative freshness windows and immutable slow-safe velocity caps."""

    deadman_timeout_sec: float = 1.50
    command_timeout_sec: float = 0.25
    robot_state_timeout_sec: float = 0.50
    max_forward_mps: float = 0.20
    max_lateral_mps: float = 0.12
    max_yaw_rps: float = 0.35

    def validate(self) -> None:
        """Reject values that could relax the reviewed slow-safe boundary."""
        positive = (
            self.deadman_timeout_sec,
            self.command_timeout_sec,
            self.robot_state_timeout_sec,
            self.max_forward_mps,
            self.max_lateral_mps,
            self.max_yaw_rps,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('live gate limits must be finite and positive')
        if self.deadman_timeout_sec > 1.50:
            raise ValueError('deadman_timeout_sec may not exceed 1.50 seconds')
        if self.command_timeout_sec > 0.25:
            raise ValueError('command_timeout_sec may not exceed 0.25 seconds')
        if self.robot_state_timeout_sec > 0.50:
            raise ValueError('robot_state_timeout_sec may not exceed 0.50 seconds')
        if self.max_forward_mps > 0.20:
            raise ValueError('max_forward_mps may not exceed slow-safe 0.20')
        if self.max_lateral_mps > 0.12:
            raise ValueError('max_lateral_mps may not exceed slow-safe 0.12')
        if self.max_yaw_rps > 0.35:
            raise ValueError('max_yaw_rps may not exceed slow-safe 0.35')


@dataclass(frozen=True)
class RobotSafetyState:
    """One fresh, independently verified robot readiness observation."""

    charging: Optional[bool]
    ready: bool

    def validate(self) -> None:
        """Ensure unknown charging remains explicit rather than truthy/falsy."""
        if self.charging is not None and type(self.charging) is not bool:
            raise ValueError('charging must be True, False, or None')
        if type(self.ready) is not bool:
            raise ValueError('ready must be a boolean')


@dataclass(frozen=True)
class KillResetResult:
    """Result of an explicit request to clear a latched kill condition."""

    accepted: bool
    reason: str


@dataclass(frozen=True)
class GateDecision:
    """Pure output that tells a future writer whether it may call transport."""

    action: GateAction
    velocity: Velocity
    reason: str
    transport_permitted: bool
    motion_permitted: bool
    kill_state: str
    deadman_state: str
    robot_state: str
    command_state: str

    @property
    def should_send_velocity(self) -> bool:
        """Return true only for a reviewed writer's bounded velocity call."""
        return self.action is GateAction.VELOCITY and self.motion_permitted

    @property
    def should_send_stop(self) -> bool:
        """Return true only when an already-authorized writer should stop."""
        return self.action is GateAction.STOP and self.transport_permitted


class LiveCommandGate:
    """Fail closed until authorization, state, kill, Deadman, and input agree."""

    def __init__(
        self,
        authorization: Optional[LiveAuthorization] = None,
        send_commands: bool = False,
        limits: LiveGateLimits = LiveGateLimits(),
    ) -> None:
        """Create a gate whose transport path is disabled unless explicitly set."""
        limits.validate()
        self.authorization = (
            LiveAuthorization.from_environment()
            if authorization is None
            else authorization
        )
        if not isinstance(self.authorization, LiveAuthorization):
            raise ValueError('authorization must be a LiveAuthorization')
        self.limits = limits
        self._send_commands = _strict_bool(send_commands, 'send_commands')
        self._kill_cleared_once = False
        self._kill_active = True
        self._kill_latched = False
        self._deadman_active = False
        self._deadman_time: Optional[float] = None
        self._robot_state: Optional[RobotSafetyState] = None
        self._robot_state_time: Optional[float] = None
        self._velocity: Optional[Velocity] = None
        self._velocity_time: Optional[float] = None

    @classmethod
    def from_environment(
        cls,
        environment: Optional[Mapping[str, object]] = None,
        send_commands: bool = False,
        limits: LiveGateLimits = LiveGateLimits(),
    ) -> 'LiveCommandGate':
        """Build a disabled-by-default gate using the current environment map."""
        return cls(
            authorization=LiveAuthorization.from_environment(environment),
            send_commands=send_commands,
            limits=limits,
        )

    @property
    def send_commands(self) -> bool:
        """Return the explicit writer-enable setting, not a motion permission."""
        return self._send_commands

    def set_send_commands(self, enabled: bool) -> None:
        """Change the writer-enable setting and discard any prior velocity intent."""
        self._send_commands = _strict_bool(enabled, 'send_commands')
        self._clear_velocity()

    def observe_kill(self, active: bool) -> None:
        """Record a kill signal; a post-clear kill must be explicitly reset."""
        active = _strict_bool(active, 'kill active')
        if active:
            if self._kill_cleared_once:
                self._kill_latched = True
            self._kill_active = True
            self._clear_velocity()
            return
        self._kill_active = False
        self._kill_cleared_once = True

    def reset_kill(self) -> KillResetResult:
        """Clear a kill latch only after the source has explicitly reported false."""
        if not self._kill_cleared_once:
            return KillResetResult(False, 'kill_clear_not_observed')
        if self._kill_active:
            return KillResetResult(False, 'kill_still_active')
        if not self._kill_latched:
            return KillResetResult(False, 'kill_not_latched')
        self._kill_latched = False
        self._clear_velocity()
        return KillResetResult(True, 'kill_reset')

    def observe_deadman(self, active: bool, now: Optional[float] = None) -> None:
        """Record a Deadman heartbeat; a false edge immediately discards intent."""
        active = _strict_bool(active, 'deadman active')
        self._deadman_active = active
        self._deadman_time = _monotonic_time(now)
        if not active:
            self._clear_velocity()

    def observe_robot_state(
        self,
        charging: Optional[bool],
        ready: bool,
        now: Optional[float] = None,
    ) -> None:
        """Record a state sample; charging must be known false and ready true."""
        state = RobotSafetyState(charging=charging, ready=ready)
        state.validate()
        self._robot_state = state
        self._robot_state_time = _monotonic_time(now)
        if charging is not False or not ready:
            self._clear_velocity()

    def receive_velocity(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        now: Optional[float] = None,
    ) -> GateDecision:
        """Accept a fresh bounded velocity only after every safety gate passes."""
        now = _monotonic_time(now)
        try:
            requested = (float(linear_x), float(linear_y), float(angular_z))
        except (TypeError, ValueError):
            self._clear_velocity()
            return self._decision(now, 'invalid_velocity')
        if not all(math.isfinite(value) for value in requested):
            self._clear_velocity()
            return self._decision(now, 'nonfinite_velocity')
        block = self._motion_block_reason(now)
        if block is not None:
            self._clear_velocity()
            return self._decision(now, block)

        limited = (
            _clamp(requested[0], self.limits.max_forward_mps),
            _clamp(requested[1], self.limits.max_lateral_mps),
            _clamp(requested[2], self.limits.max_yaw_rps),
        )
        self._velocity = limited
        self._velocity_time = now
        reason = 'velocity_clamped' if limited != requested else None
        return self._decision(now, reason)

    def evaluate(self, now: Optional[float] = None) -> GateDecision:
        """Evaluate watchdogs and return a transport-free action for this tick."""
        now = _monotonic_time(now)
        block = self._motion_block_reason(now)
        if block is not None:
            self._clear_velocity()
            return self._decision(now, block)
        # Retain an expired timestamp solely for diagnostics. It cannot be
        # sent because _decision() reports command_stale, and only a new
        # receive_velocity() call can replace it with a fresh packet.
        return self._decision(now)

    def kill_state(self) -> str:
        """Return the explicit-clear, active, latched, or clear kill state."""
        if not self._kill_cleared_once:
            return 'awaiting_explicit_clear'
        if self._kill_active:
            return 'active'
        if self._kill_latched:
            return 'latched'
        return 'clear'

    def deadman_state(self, now: Optional[float] = None) -> str:
        """Return missing, inactive, stale, or active for the Deadman input."""
        now = _monotonic_time(now)
        if self._deadman_time is None:
            return 'missing'
        if not self._deadman_active:
            return 'inactive'
        if now - self._deadman_time > self.limits.deadman_timeout_sec:
            return 'stale'
        return 'active'

    def robot_state_status(self, now: Optional[float] = None) -> str:
        """Return missing/stale/unsafe state before allowing a future writer."""
        now = _monotonic_time(now)
        if self._robot_state is None or self._robot_state_time is None:
            return 'missing'
        if now - self._robot_state_time > self.limits.robot_state_timeout_sec:
            return 'stale'
        if self._robot_state.charging is None:
            return 'charging_unknown'
        if self._robot_state.charging:
            return 'charging'
        if not self._robot_state.ready:
            return 'not_ready'
        return 'ready'

    def command_state(self, now: Optional[float] = None) -> str:
        """Return missing, stale, or fresh for the last accepted velocity input."""
        return self._command_state(_monotonic_time(now))

    def _motion_block_reason(self, now: float) -> Optional[str]:
        """Return the first deterministic condition that forbids motion."""
        if not self.authorization.live_authorized:
            return 'live_authorization_missing'
        if not self._send_commands:
            return 'send_commands_disabled'
        kill = self.kill_state()
        if kill != 'clear':
            return 'kill_%s' % kill
        deadman = self.deadman_state(now)
        if deadman != 'active':
            return 'deadman_%s' % deadman
        robot = self.robot_state_status(now)
        if robot != 'ready':
            return 'robot_%s' % robot
        return None

    def _command_state(self, now: float) -> str:
        """Inspect the command watchdog without touching its stored timestamp."""
        if self._velocity is None or self._velocity_time is None:
            return 'missing'
        if now - self._velocity_time > self.limits.command_timeout_sec:
            return 'stale'
        return 'fresh'

    def _decision(self, now: float, requested_reason: Optional[str] = None) -> GateDecision:
        """Build one immutable action while preserving no transport side effects."""
        transport_permitted = (
            self.authorization.live_authorized and self._send_commands
        )
        kill = self.kill_state()
        deadman = self.deadman_state(now)
        robot = self.robot_state_status(now)
        command = self._command_state(now)

        if not self.authorization.live_authorized:
            missing = ','.join(self.authorization.missing_interlocks())
            reason = 'live_authorization_missing:%s' % missing
            return self._hold(reason, kill, deadman, robot, command)
        if not self._send_commands:
            return self._hold(
                'send_commands_disabled', kill, deadman, robot, command
            )
        block = self._motion_block_reason(now)
        if block is not None:
            return self._stop(
                requested_reason or block, kill, deadman, robot, command
            )
        if requested_reason in ('invalid_velocity', 'nonfinite_velocity'):
            return self._stop(
                requested_reason, kill, deadman, robot, command
            )
        if command != 'fresh':
            return self._stop(
                requested_reason or 'command_%s' % command,
                kill,
                deadman,
                robot,
                command,
            )

        velocity = self._velocity
        assert velocity is not None
        if velocity == (0.0, 0.0, 0.0):
            return self._stop('zero_velocity', kill, deadman, robot, command)
        return GateDecision(
            action=GateAction.VELOCITY,
            velocity=velocity,
            reason=requested_reason or 'accepted',
            transport_permitted=transport_permitted,
            motion_permitted=True,
            kill_state=kill,
            deadman_state=deadman,
            robot_state=robot,
            command_state=command,
        )

    def _hold(
        self,
        reason: str,
        kill: str,
        deadman: str,
        robot: str,
        command: str,
    ) -> GateDecision:
        """Return no transport action when configuration has not authorized it."""
        return GateDecision(
            action=GateAction.HOLD,
            velocity=(0.0, 0.0, 0.0),
            reason=reason,
            transport_permitted=False,
            motion_permitted=False,
            kill_state=kill,
            deadman_state=deadman,
            robot_state=robot,
            command_state=command,
        )

    def _stop(
        self,
        reason: str,
        kill: str,
        deadman: str,
        robot: str,
        command: str,
    ) -> GateDecision:
        """Return a zero-motion stop action for an already authorized writer."""
        return GateDecision(
            action=GateAction.STOP,
            velocity=(0.0, 0.0, 0.0),
            reason=reason,
            transport_permitted=True,
            motion_permitted=False,
            kill_state=kill,
            deadman_state=deadman,
            robot_state=robot,
            command_state=command,
        )

    def _clear_velocity(self) -> None:
        """Discard stale intent so recovery always needs a new command packet."""
        self._velocity = None
        self._velocity_time = None


def _environment_flag(
    name: str,
    default: bool,
    environment: Mapping[str, object],
) -> Tuple[bool, Optional[str]]:
    """Read one boolean environment flag without treating malformed text true."""
    raw = environment.get(name)
    if raw is None:
        return default, None
    normalized = str(raw).strip().lower()
    if normalized in _TRUE_VALUES:
        return True, None
    if normalized in _FALSE_VALUES:
        return False, None
    return default, '%s=invalid' % name


def _strict_bool(value: bool, name: str) -> bool:
    """Reject non-bool values rather than accepting truthy configuration text."""
    if type(value) is not bool:
        raise ValueError('%s must be a boolean' % name)
    return value


def _monotonic_time(value: Optional[float]) -> float:
    """Return one finite monotonic timestamp or reject a broken clock value."""
    now = time.monotonic() if value is None else float(value)
    if not math.isfinite(now):
        raise ValueError('time must be finite')
    return now


def _clamp(value: float, magnitude: float) -> float:
    """Clamp a scalar symmetrically around zero."""
    return max(-magnitude, min(magnitude, value))
