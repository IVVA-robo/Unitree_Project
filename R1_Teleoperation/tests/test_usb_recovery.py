import signal
from types import SimpleNamespace

import pytest

from usb_link.recovery import RelayRecovery, pause_for_recovery


def state(status='ready', **kwargs):
    return dict(pid=123, session_id='owned', mode='control',
                session_mode='session_arm', status=status, **kwargs)


def test_recovery_requires_matching_owner_and_confirmed_pause(monkeypatch):
    from usb_link import recovery
    samples = iter([state(), state(), state('locked')])
    monkeypatch.setattr(recovery, '_read_state', lambda _: next(samples))
    signals = []
    manager = SimpleNamespace(pid=123, poll=lambda: None, send_signal=signals.append)
    pause_for_recovery(manager, None, lambda: False)
    assert signals == [signal.SIGUSR1]  # never SIGUSR2 / auto-RUN


@pytest.mark.parametrize('bad', [state('stopped'), state('starting_control'),
                                dict(state(), pid=999), dict(state(), mode='static')])
def test_wrong_or_unready_owner_cannot_be_recovered(monkeypatch, bad):
    from usb_link import recovery
    monkeypatch.setattr(recovery, '_read_state', lambda _: bad)
    signals = []
    manager = SimpleNamespace(pid=123, poll=lambda: None, send_signal=signals.append)
    with pytest.raises(RuntimeError):
        pause_for_recovery(manager, None, lambda: False)
    assert signals == []


def test_unconfirmed_pause_prevents_restart(monkeypatch):
    from usb_link import recovery
    monkeypatch.setattr(recovery, '_read_state', lambda _: state())
    manager = SimpleNamespace(pid=123, poll=lambda: None, send_signal=lambda _: None)
    with pytest.raises(RuntimeError, match='could not confirm'):
        pause_for_recovery(manager, None, lambda: False, timeout=0.05)


def test_stop_during_recovery_prevents_restart(monkeypatch):
    from usb_link import recovery
    monkeypatch.setattr(recovery, '_read_state', lambda _: state())
    stopped = [False]
    manager = SimpleNamespace(pid=123, poll=lambda: None,
                              send_signal=lambda _: stopped.__setitem__(0, True))
    with pytest.raises(RuntimeError):
        pause_for_recovery(manager, None, lambda: stopped[0])


def test_crash_loop_is_bounded_and_budget_recovers_after_quiet_period():
    recovery = RelayRecovery()
    for now in (0, 10, 20):
        recovery.admit(now)
    with pytest.raises(RuntimeError):
        recovery.admit(21)
    recovery.admit(61)
