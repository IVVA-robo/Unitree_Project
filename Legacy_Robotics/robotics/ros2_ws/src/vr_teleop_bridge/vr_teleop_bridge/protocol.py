from dataclasses import dataclass
import json
import math
from typing import Any, Mapping, Optional, Sequence, Tuple
PROTOCOL_VERSION = 1
UINT32_MODULUS = 1 << 32
UINT32_HALF_RANGE = 1 << 31
class PacketError(ValueError): pass
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
    try: root = json.loads(payload.decode('utf-8'))
    except Exception as exc: raise PacketError(f'invalid UTF-8/JSON: {exc}') from exc
    sequence = root.get('seq', 0)
    client_time_ms = root.get('client_time_ms', 0)
    deadman = root.get('deadman', False)
    sticks = root.get('sticks', {})
    triggers = root.get('triggers', [0.0, 0.0])
    return VRPacket(
        sequence=sequence, client_time_ms=client_time_ms,
        left=_pose(root, 'left', max_position_m),
        right=_pose(root, 'right', max_position_m),
        head=_pose(root, 'head', max_position_m),
        left_stick=(sticks.get('left', [0,0])[0], sticks.get('left', [0,0])[1]),
        right_stick=(sticks.get('right', [0,0])[0], sticks.get('right', [0,0])[1]),
        left_trigger=triggers[0], right_trigger=triggers[1], deadman=deadman,
    )
def is_newer_sequence(sequence: int, previous: Optional[int]) -> bool:
    if previous is None: return True
    return 0 < ((sequence - previous) % UINT32_MODULUS) < UINT32_HALF_RANGE
def apply_deadzone(value: float, deadzone: float) -> float:
    value = max(-1.0, min(1.0, value))
    if abs(value) <= deadzone: return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)
def _pose(parent: Mapping[str, Any], key: str, max_position_m: float) -> Pose:
    value = parent.get(key, {})
    position = value.get('p', [0,0,0])
    orientation = value.get('q', [0,0,0,1])
    norm = math.sqrt(sum(c * c for c in orientation))
    return Pose(position=tuple(position), orientation=tuple(c / (norm if norm > 0 else 1) for c in orientation))
