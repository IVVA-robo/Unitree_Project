"""Parse and validate the explicit environment interlocks for R1 control."""

from dataclasses import dataclass
import os
from typing import Mapping, Tuple


TRUE_VALUES = frozenset(('1', 'true', 'yes', 'on'))
FALSE_VALUES = frozenset(('0', 'false', 'no', 'off', ''))


def environment_flag(
    name: str,
    default: bool,
    environment: Mapping[str, str] = None,
) -> bool:
    """Read one explicit boolean flag and reject ambiguous values."""
    values = os.environ if environment is None else environment
    raw = values.get(name)
    if raw is None:
        return default
    normalized = str(raw).strip().lower()
    if normalized in TRUE_VALUES:
        return True
    if normalized in FALSE_VALUES:
        return False
    raise ValueError(
        f'{name} must be one of 0/1, false/true, no/yes, or off/on; got {raw!r}'
    )


@dataclass(frozen=True)
class ActuationPolicy:
    """The human-acknowledged conditions required before a live request."""

    dry_run: bool
    enable_actuation: bool
    confirm_off_charger: bool
    confirm_clear_area: bool
    confirm_estop_ready: bool

    @classmethod
    def from_environment(cls, environment: Mapping[str, str] = None):
        """Build the policy using fail-closed defaults."""
        return cls(
            dry_run=environment_flag('ROBOT_DRY_RUN', True, environment),
            enable_actuation=environment_flag(
                'ROBOT_ENABLE_ACTUATION', False, environment
            ),
            confirm_off_charger=environment_flag(
                'ROBOT_CONFIRM_OFF_CHARGER', False, environment
            ),
            confirm_clear_area=environment_flag(
                'ROBOT_CONFIRM_CLEAR_AREA', False, environment
            ),
            confirm_estop_ready=environment_flag(
                'ROBOT_CONFIRM_ESTOP_READY', False, environment
            ),
        )

    @property
    def live_authorized(self) -> bool:
        """Return whether all human interlocks explicitly permit a live request."""
        return (
            not self.dry_run
            and self.enable_actuation
            and self.confirm_off_charger
            and self.confirm_clear_area
            and self.confirm_estop_ready
        )

    @property
    def mode(self) -> str:
        """Return a concise mode suitable for diagnostics."""
        if self.dry_run:
            return 'dry-run'
        if self.live_authorized:
            return 'live-authorized'
        return 'live-blocked'

    def missing_live_interlocks(self) -> Tuple[str, ...]:
        """List every missing explicit authorization in a stable order."""
        missing = []
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

    def summary(self) -> str:
        """Return diagnostic text without implying that a robot is armed."""
        return (
            f'mode={self.mode} dry_run={str(self.dry_run).lower()} '
            f'enable_actuation={str(self.enable_actuation).lower()} '
            f'off_charger={str(self.confirm_off_charger).lower()} '
            f'clear_area={str(self.confirm_clear_area).lower()} '
            f'estop_ready={str(self.confirm_estop_ready).lower()}'
        )
