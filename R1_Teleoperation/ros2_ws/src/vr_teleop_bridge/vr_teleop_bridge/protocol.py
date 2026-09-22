"""Parsing and validation for the versioned Pico UDP wire protocol."""

from dataclasses import dataclass
import json
import math
from typing import Any, Mapping, Optional, Tuple


PROTOCOL_VERSION = 1
UINT32_MODULUS = 1 << 32
UINT32_HALF_RANGE = 1 << 31


class PacketError(ValueError):
    """A datagram is not a valid protocol packet."""


@dataclass(frozen=True)
class Pose:
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]


@dataclass(frozen=True)
class VRPacket:
    sequence: int
    client_time_ms: int
    left: Pose
    right: Pose
    head: Pose
    left_stick: Tuple[float, float]
    right_stick: Tuple[float, float]
    left_trigger: float
    right_trigger: float
    deadman: bool


def parse_packet(payload: bytes, max_position_m: float = 5.0) -> VRPacket:
    """Decode, structurally validate, and normalize one complete UDP snapshot."""
    try:
        root = json.loads(payload.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PacketError(f'invalid UTF-8/JSON: {exc}') from exc

    if not isinstance(root, dict):
        raise PacketError('packet root must be an object')
    if root.get('v') != PROTOCOL_VERSION:
        raise PacketError(f"unsupported protocol version: {root.get('v')!r}")

    sequence = _integer(root, 'seq', minimum=0, maximum=UINT32_MODULUS - 1)
    client_time_ms = _integer(root, 'client_time_ms', minimum=0)
    deadman = root.get('deadman')
    if type(deadman) is not bool:  # bool is intentionally stricter than truthiness.
        raise PacketError('deadman must be a boolean')

    sticks = _mapping(root, 'sticks')
    left_stick = _stick(sticks, 'left')
    right_stick = _stick(sticks, 'right')
    triggers = _triggers(root)

    return VRPacket(
        sequence=sequence,
        client_time_ms=client_time_ms,
        left=_pose(root, 'left', max_position_m),
        right=_pose(root, 'right', max_position_m),
        head=_pose(root, 'head', max_position_m),
        left_stick=left_stick,
        right_stick=right_stick,
        left_trigger=triggers[0],
        right_trigger=triggers[1],
        deadman=deadman,
    )


def is_newer_sequence(sequence: int, previous: Optional[int]) -> bool:
    """Return true when an unsigned 32-bit sequence is newer, including wraparound."""
    if previous is None:
        return True
    delta = (sequence - previous) % UINT32_MODULUS
    return 0 < delta < UINT32_HALF_RANGE


def apply_deadzone(value: float, deadzone: float) -> float:
    """Apply a continuous axial dead zone and rescale the remainder to [-1, 1]."""
    value = max(-1.0, min(1.0, value))
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


def _mapping(parent: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = parent.get(key)
    if not isinstance(value, dict):
        raise PacketError(f'{key} must be an object')
    return value


def _integer(
    parent: Mapping[str, Any], key: str, minimum: int, maximum: Optional[int] = None
) -> int:
    value = parent.get(key)
    if type(value) is not int:
        raise PacketError(f'{key} must be an integer')
    if value < minimum or (maximum is not None and value > maximum):
        raise PacketError(f'{key} is outside the allowed range')
    return value


def _numbers(
    parent: Mapping[str, Any], key: str, length: int
) -> Tuple[float, ...]:
    values = parent.get(key)
    if not isinstance(values, list) or len(values) != length:
        raise PacketError(f'{key} must be an array of {length} numbers')
    result = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise PacketError(f'{key} contains a non-number')
        converted = float(value)
        if not math.isfinite(converted):
            raise PacketError(f'{key} contains NaN or infinity')
        result.append(converted)
    return tuple(result)


def _pose(parent: Mapping[str, Any], key: str, max_position_m: float) -> Pose:
    value = _mapping(parent, key)
    position = _numbers(value, 'p', 3)
    if math.sqrt(sum(component * component for component in position)) > max_position_m:
        raise PacketError(f'{key}.p is outside the tracking workspace')

    orientation = _numbers(value, 'q', 4)
    norm = math.sqrt(sum(component * component for component in orientation))
    if norm < 0.5 or norm > 1.5:
        raise PacketError(f'{key}.q has an invalid norm')
    normalized = tuple(component / norm for component in orientation)
    return Pose(position=position, orientation=normalized)


def _stick(parent: Mapping[str, Any], key: str) -> Tuple[float, float]:
    values = _numbers(parent, key, 2)
    # Small overshoot is clamped, grossly invalid values indicate a bad producer.
    if any(abs(value) > 1.1 for value in values):
        raise PacketError(f'{key} stick is outside [-1, 1]')
    return tuple(max(-1.0, min(1.0, value)) for value in values)


def _triggers(parent: Mapping[str, Any]) -> Tuple[float, float]:
    if 'triggers' not in parent:
        return (0.0, 0.0)  # Backward compatibility with early v1 Unity clients.
    values = _numbers(parent, 'triggers', 2)
    if any(value < -0.05 or value > 1.05 for value in values):
        raise PacketError('triggers are outside [0, 1]')
    return tuple(max(0.0, min(1.0, value)) for value in values)
