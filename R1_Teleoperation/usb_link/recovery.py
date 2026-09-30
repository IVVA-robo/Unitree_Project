"""Recover pose transport only after the existing owner confirms LOCK."""

import signal
import time

from exhibition.orchestrator import _read_state


class RelayRecovery:
    def __init__(self):
        self.attempts = []

    def admit(self, now):
        self.attempts = [stamp for stamp in self.attempts if now - stamp < 60.0]
        if len(self.attempts) >= 3:
            raise RuntimeError('USB relay repeatedly fails; reviewed STOP required')
        self.attempts.append(now)


def pause_for_recovery(manager, settings, stopping, timeout=10.0):
    state = _read_state(settings)
    if (stopping() or manager.poll() is not None or state.get('pid') != manager.pid
            or state.get('mode') != 'control' or state.get('session_mode') != 'session_arm'
            or state.get('status') not in {'ready', 'locked', 'degraded'}):
        raise RuntimeError('USB recovery has no ready matching control owner')
    session_id = state.get('session_id')
    if state.get('status') == 'locked':
        return
    manager.send_signal(signal.SIGUSR1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not stopping() and manager.poll() is None:
        state = _read_state(settings)
        if state.get('session_id') != session_id or state.get('pid') != manager.pid:
            break
        if state.get('status') == 'locked':
            return
        if state.get('status') in {'blocked', 'stopped'}:
            break
        time.sleep(0.05)
    raise RuntimeError('USB recovery could not confirm LOCK; reviewed STOP required')
