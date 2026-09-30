"""Repeated STOP shortcut checks with fake /proc and runtime; no robot/ROS."""
import fcntl
import json
from pathlib import Path

import pytest

from exhibition import orchestrator, process_owner


def fake_process(root, argv, *, state='S', pid='7654321'):
    process = root / pid
    process.mkdir()
    (process / 'cmdline').write_bytes(b'\0'.join(arg.encode() for arg in argv) + b'\0')
    (process / 'stat').write_text(f'{pid} (a process) {state} 0 0 0')
    return process


@pytest.mark.parametrize('argv', [
    ['/opt/ros/lib/r1_live_writer_node', '--ros-args'],
    ['bash', '/project/scripts/r1-teleop-live'],
    ['python3', '-m', 'exhibition.orchestrator', 'control'],
    ['python3', '-m', 'exhibition.orchestrator', 'static'],
    ['python3', '-m', 'usb_link.control', 'control'],
    ['bash', '/project/scripts/r1-exhibition', 'static'],
])
def test_control_process_blocks_noop(tmp_path, argv):
    fake_process(tmp_path, argv)
    assert process_owner.local_control_process_exists(tmp_path)


@pytest.mark.parametrize('argv', [
    ['python3', '-m', 'exhibition.orchestrator', 'stop'],
    ['python3', '-m', 'exhibition.orchestrator', 'status'],
    ['/opt/ros/lib/r1_sdk_transport'],  # read-only reader is not a control writer
    ['bash', '-c', 'echo r1_live_writer_node'],
])
def test_unrelated_readonly_process_does_not_block(tmp_path, argv):
    fake_process(tmp_path, argv)
    assert not process_owner.local_control_process_exists(tmp_path)


def test_zombie_writer_is_not_live(tmp_path):
    fake_process(tmp_path, ['r1_live_writer_node'], state='Z')
    assert not process_owner.local_control_process_exists(tmp_path)


def test_unreadable_process_or_proc_root_cannot_establish_absence(tmp_path, monkeypatch):
    fake_process(tmp_path, ['r1_live_writer_node'])
    original = Path.read_bytes

    def unreadable(path):
        if path.name == 'cmdline':
            raise PermissionError('unreadable')
        return original(path)

    monkeypatch.setattr(Path, 'read_bytes', unreadable)
    assert process_owner.local_control_process_exists(tmp_path)
    assert process_owner.local_control_process_exists(tmp_path / 'missing')


@pytest.fixture
def stopped(tmp_path, monkeypatch):
    monkeypatch.setenv('R1_EXHIBITION_RUNTIME_DIR', str(tmp_path))
    monkeypatch.setenv('R1_EXHIBITION_MOCK', '0')
    settings = orchestrator.ExhibitionSettings.from_environment()
    state = {'session_id': 'reviewed', 'pid': 99999999, 'status': 'stopped',
             'safe_stop_confirmed': True, 'children': {}, 'transport_owner': {},
             'mock': False, 'updated_at': '2026-09-29T14:14:13Z'}
    (tmp_path / 'state.json').write_text(json.dumps(state))
    (tmp_path / 'session.lock').touch()
    monkeypatch.setattr(orchestrator, 'local_control_process_exists', lambda: False)
    monkeypatch.setattr(orchestrator, 'owner_alive', lambda _: False)
    monkeypatch.setattr(orchestrator, '_active_pid', lambda _: None)
    return settings, state


def test_repeated_stop_avoids_absent_ros_services(stopped, monkeypatch):
    monkeypatch.setattr(orchestrator, '_run_action', lambda *_: pytest.fail('must not send commands'))
    assert orchestrator.main(['stop']) == 0


@pytest.mark.parametrize('change', [
    {'safe_stop_confirmed': False}, {'children': {'control': 42}}, {'status': 'ready'},
    {'mock': True}, {'session_id': ''}, {'updated_at': ''},
])
def test_incomplete_or_unconfirmed_record_never_noops(stopped, change):
    settings, state = stopped
    state.update(change)
    assert not orchestrator._confirmed_cleanup_is_idle(settings, state)


def test_another_manager_holding_lock_blocks_shortcut(stopped):
    settings, state = stopped
    with (settings.runtime_dir / 'session.lock').open('r+') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert not orchestrator._confirmed_cleanup_is_idle(settings, state)


def test_changed_state_or_missing_lock_blocks_shortcut(stopped):
    settings, state = stopped
    (settings.runtime_dir / 'state.json').write_text(json.dumps({**state, 'session_id': 'new'}))
    assert not orchestrator._confirmed_cleanup_is_idle(settings, state)
    (settings.runtime_dir / 'state.json').write_text(json.dumps(state))
    (settings.runtime_dir / 'session.lock').unlink()
    assert not orchestrator._confirmed_cleanup_is_idle(settings, state)


@pytest.mark.parametrize('guard', ['local_control_process_exists', 'owner_alive', '_active_pid'])
def test_live_process_prevents_false_already_stopped(stopped, monkeypatch, guard):
    settings, state = stopped
    monkeypatch.setattr(orchestrator, guard, lambda *args: True)
    assert not orchestrator._confirmed_cleanup_is_idle(settings, state)


def test_unknown_or_lost_state_still_uses_stop_fallback(stopped, monkeypatch):
    settings, _ = stopped
    (settings.runtime_dir / 'state.json').write_text('{}')
    calls = []
    monkeypatch.setattr(orchestrator, '_run_action', lambda settings, action, timeout: calls.append(action) or 0)
    assert orchestrator.main(['stop']) == 0
    assert calls == ['stop']


def test_live_writer_with_old_stopped_record_still_gets_stop(stopped, monkeypatch):
    monkeypatch.setattr(orchestrator, 'local_control_process_exists', lambda: True)
    calls = []
    monkeypatch.setattr(orchestrator, '_run_action', lambda settings, action, timeout: calls.append(action) or 0)
    assert orchestrator.main(['stop']) == 0
    assert calls == ['stop']
