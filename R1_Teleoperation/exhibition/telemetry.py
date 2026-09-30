"""Read-only status cache; diagnostic events cannot refresh a state snapshot."""

from collections import deque
from dataclasses import dataclass
import math
import re
import time


REQUIRED_FIELDS = {
    'writer': frozenset('transport prepared kill_clear session_armed state_fresh '
                        'motor_health_fresh locomotion_active vr_recovery '
                        'control_permission head_fresh arm_fresh'.split()),
    'vr': frozenset('source valid_packets bad_packets packet_age_sec fresh active '
                    'locomotion_active session_armed tracking_phase emergency_stop '
                    'head_tracked left_tracked right_tracked left_stick right_stick'.split()),
}


def status_fields(message):
    return dict(re.findall(r'(\w+)=([^\s]+)', message))


@dataclass(frozen=True)
class StatusSample:
    received: float
    fields: dict


class StatusSnapshots:
    """Separate state from events without merging or extending state freshness.

    A safety event aborts the current observation even if an older complete
    snapshot looked healthy. Only a new observer can start a new test.
    """

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.latest = {}
        self.events = deque(maxlen=200)
        self.safety_event = None

    def observe(self, channel, message):
        if channel not in REQUIRED_FIELDS:
            raise ValueError('unknown status channel')
        parsed = status_fields(message)
        received = self.clock()
        if channel == 'writer' and (
            message.startswith('fail_closed ')
            or parsed.get('kill_latched') == 'true'
            or parsed.get('kill_clear') == 'false'
        ):
            self.safety_event = message
        if channel == 'vr' and parsed.get('emergency_stop') == 'true':
            self.safety_event = message
        marker = 'transport' if channel == 'writer' else 'source'
        if marker not in parsed:
            self.events.append({'channel': channel, 'received': received, 'message': message})
            return False
        # Keep malformed full snapshots too: read() must refuse them, rather
        # than silently falling back to an older healthy state.
        self.latest[channel] = StatusSample(received, parsed)
        return True

    def read(self, channel, max_age=2.0):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('max_age must be positive and finite')
        if self.safety_event is not None:
            raise RuntimeError('safety event: ' + self.safety_event)
        sample = self.latest.get(channel)
        if sample is None:
            raise RuntimeError(channel + ' full status missing')
        age = self.clock() - sample.received
        if not math.isfinite(age) or not 0 <= age < max_age:
            raise RuntimeError(channel + ' full status stale')
        missing = REQUIRED_FIELDS[channel] - sample.fields.keys()
        if missing:
            raise RuntimeError(channel + ' full status incomplete: ' + ','.join(sorted(missing)))
        return dict(sample.fields), age
