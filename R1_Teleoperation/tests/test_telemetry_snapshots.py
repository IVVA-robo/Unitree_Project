"""Recorded-event regression and fail-closed status parsing; no ROS/network."""
import pytest

from exhibition.telemetry import REQUIRED_FIELDS, StatusSnapshots


def payload(channel, **changes):
    values = {key: 'true' for key in REQUIRED_FIELDS[channel]}
    values.update(transport='sdk') if channel == 'writer' else values.update(source='127.0.0.1', emergency_stop='false')
    values.update(changes)
    return ' '.join(f'{key}={value}' for key, value in values.items())


def test_recovered_event_preserves_last_full_status_and_timestamp():
    now = [1.0]
    cache = StatusSnapshots(clock=lambda: now[0])
    cache.observe('writer', payload('writer'))
    now[0] = 1.5
    assert not cache.observe('writer', 'exhibition_vr=RECOVERED automatic_resume=true')
    state, age = cache.read('writer')
    assert state['prepared'] == 'true'
    assert age == 0.5
    assert len(cache.events) == 1
    now[0] = 3.0
    cache.observe('writer', 'exhibition_vr=RECOVERED automatic_resume=true')
    with pytest.raises(RuntimeError, match='stale'):
        cache.read('writer')


def test_event_without_full_snapshot_never_establishes_ready():
    cache = StatusSnapshots()
    cache.observe('writer', 'exhibition_vr=RECOVERED automatic_resume=true')
    with pytest.raises(RuntimeError, match='missing'):
        cache.read('writer')


@pytest.mark.parametrize('event', ['fail_closed reason=operator_stop kill_latched=true',
                                  'arm_feedback_follow_timeout kill_latched=true',
                                  'kill_clear=false'])
def test_safety_events_abort_even_with_previous_healthy_state(event):
    cache = StatusSnapshots()
    cache.observe('writer', payload('writer'))
    cache.observe('writer', event)
    cache.observe('writer', payload('writer'))
    with pytest.raises(RuntimeError, match='safety event'):
        cache.read('writer')


def test_emergency_vr_event_aborts_observer():
    cache = StatusSnapshots()
    cache.observe('vr', payload('vr'))
    cache.observe('vr', 'emergency_stop=true')
    with pytest.raises(RuntimeError, match='safety event'):
        cache.read('vr')


def test_incomplete_full_snapshot_does_not_reuse_older_health():
    cache = StatusSnapshots()
    cache.observe('writer', payload('writer'))
    cache.observe('writer', 'transport=sdk prepared=false')
    with pytest.raises(RuntimeError, match='incomplete'):
        cache.read('writer')


def test_full_negative_state_is_preserved_not_masked_as_diagnostic():
    cache = StatusSnapshots()
    cache.observe('writer', payload('writer', prepared='false'))
    assert cache.read('writer')[0]['prepared'] == 'false'


def test_real_kill_in_full_snapshot_aborts():
    cache = StatusSnapshots()
    cache.observe('writer', payload('writer', kill_clear='false'))
    with pytest.raises(RuntimeError, match='safety event'):
        cache.read('writer')


def test_read_returns_copy_and_events_are_bounded():
    cache = StatusSnapshots()
    cache.observe('writer', payload('writer'))
    cache.read('writer')[0]['prepared'] = 'false'
    assert cache.read('writer')[0]['prepared'] == 'true'
    for _ in range(300):
        cache.observe('writer', 'diagnostic=event')
    assert len(cache.events) == 200
