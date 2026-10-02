import json
import os
from pathlib import Path
import signal
import shlex
import subprocess
import sys
import time

import pytest

from exhibition.orchestrator import (
    ExhibitionManager,
    ExhibitionSettings,
    _wait_for_manager_stop,
    build_plan,
)


PROJECT_DIR = Path(__file__).resolve().parents[1]


def _base_environment(tmp_path):
    environment = os.environ.copy()
    environment.update(
        {
            "PYTHONPATH": str(PROJECT_DIR),
            "R1_EXHIBITION_PROJECT_DIR": str(PROJECT_DIR),
            "R1_EXHIBITION_RUNTIME_DIR": str(tmp_path / "run"),
            "R1_EXHIBITION_LOG_DIR": str(tmp_path / "logs"),
            "R1_EXHIBITION_MOCK": "1",
            "R1_EXHIBITION_PREFLIGHT_TIMEOUT_SEC": "3",
            "R1_EXHIBITION_READY_TIMEOUT_SEC": "3",
            "R1_EXHIBITION_STOP_WAIT_SEC": "3",
            "R1_EXHIBITION_RESTART_INITIAL_SEC": "0.05",
            "R1_EXHIBITION_RESTART_MAX_SEC": "0.10",
            "R1_EXHIBITION_OFFLINE_POV_RECOVERY_GRACE_SEC": "0.01",
        }
    )
    return environment


def _write_worker(tmp_path):
    worker = tmp_path / "worker.py"
    worker.write_text(
        """
import os
from pathlib import Path
import signal
import sys
import time

mode = sys.argv[1]
events_path = os.environ.get('R1_TEST_EVENTS')
def record(event):
    if not events_path:
        return
    with Path(events_path).open('a', encoding='utf-8') as stream:
        stream.write(event + '\\n')

record(mode)
if os.environ.get('R1_TEST_CAPTURE_ENV'):
    record(
        'env:' + mode
        + ':static=' + os.environ.get('ROBOT_CONFIRM_STATIC_PREPARE', '')
        + ':session=' + os.environ.get('R1_EXHIBITION_SESSION_MODE', '')
        + ':exhibition=' + os.environ.get('ROBOT_EXHIBITION_SESSION', '')
    )
offline_state = os.environ.get('R1_TEST_OFFLINE_STATE')
if mode in {'offline-status', 'offline-pov-status'}:
    probe_count_path = os.environ.get('R1_TEST_OFFLINE_PROBE_COUNT')
    if probe_count_path:
        probe_count = Path(probe_count_path)
        previous_count = (
            probe_count.read_text() if probe_count.exists() else '0'
        )
        count = int(previous_count) + 1
        probe_count.write_text(str(count))
        if os.environ.get('R1_TEST_FAIL_SECOND_OFFLINE_PROBE') and count == 2:
            raise SystemExit(6)
    if offline_state and Path(offline_state).read_text().strip() == 'active':
        raise SystemExit(0)
    raise SystemExit(3)
if mode == 'offline-stop':
    Path(offline_state).write_text('inactive')
    raise SystemExit(0)
if mode == 'offline-start':
    Path(offline_state).write_text('active')
    raise SystemExit(0)
if mode == 'offline-restart':
    Path(offline_state).write_text('active')
    raise SystemExit(0)
if mode == 'hard-fail':
    raise SystemExit(6)
if mode == 'needs-rearm':
    raise SystemExit(3)
if mode in {
    'ok', 'preflight', 'prepare', 'arm', 'disarm', 'pause', 'resume', 'clear',
    'calibrate', 'stop', 'kill'
}:
    raise SystemExit(0)
if mode == 'ready-delay':
    time.sleep(0.25)
    record('ready-complete')
    raise SystemExit(0)
if mode == 'fail-soon':
    time.sleep(0.20)
    raise SystemExit(7)
if mode == 'pov-restart':
    counter = Path(os.environ['R1_TEST_POV_COUNTER'])
    count = int(counter.read_text() if counter.exists() else '0') + 1
    counter.write_text(str(count))
    if count == 1:
        raise SystemExit(9)
def stopped(_signum, _frame):
    record(mode + '-stopped')
    raise SystemExit(0)

signal.signal(signal.SIGTERM, stopped)
while True:
    time.sleep(0.10)
""".lstrip(),
        encoding="utf-8",
    )
    return worker


def _command(worker, mode):
    return shlex.join([sys.executable, str(worker), mode])


def _wait_for_state(environment, predicate, timeout=6.0):
    path = Path(environment["R1_EXHIBITION_RUNTIME_DIR"]) / "state.json"
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            time.sleep(0.05)
            continue
        if predicate(last):
            return last
        time.sleep(0.05)
    raise AssertionError(f"state predicate timed out; last={last!r}")


def _manager(environment, action):
    return subprocess.Popen(
        [sys.executable, "-m", "exhibition.orchestrator", action],
        cwd=PROJECT_DIR,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def _action(environment, action):
    return subprocess.run(
        [sys.executable, "-m", "exhibition.orchestrator", action],
        cwd=PROJECT_DIR,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=8,
        check=False,
    )


def _stop_manager(process, environment):
    if process.poll() is None:
        _action(environment, "stop")
    try:
        return process.communicate(timeout=6)
    except subprocess.TimeoutExpired:
        process.terminate()
        return process.communicate(timeout=3)


def test_usb_cleanup_handoff_then_static_and_control_remain_usable(tmp_path):
    """Real subprocess handoff with fake workers: no ADB, ROS or robot access."""
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    environment['R1_EXHIBITION_SESSION_MODE'] = 'session_arm'
    for action, mode in {
        'OFFLINE_STATUS': 'offline-status', 'PREFLIGHT': 'preflight',
        'POV': 'pov', 'CONTROL': 'control', 'STATIC_WRITER': 'static-writer',
        'READY_STATIC': 'ok', 'READY_CONTROL': 'ok', 'PREPARE': 'prepare',
        'ARM_SESSION': 'arm', 'DISARM_SESSION': 'disarm',
        'CALIBRATE': 'calibrate', 'STOP': 'stop', 'KILL': 'kill',
    }.items():
        environment[f'R1_EXHIBITION_{action}_COMMAND'] = _command(worker, mode)
    hold_cleanup = tmp_path / 'hold-cleanup'
    hold_cleanup.touch()
    # Exact argv marker lets the read-only identity code recognize this test
    # owner. It runs only the mock manager, never usb_link.control itself.
    owner = subprocess.Popen([
        sys.executable, '-c',
        "import subprocess,sys,time,signal; from pathlib import Path; "
        "p=subprocess.Popen([sys.executable,'-m','exhibition.orchestrator','control']); "
        "signal.signal(signal.SIGTERM,lambda *_: p.terminate() if p.poll() is None else None); "
        "p.wait(); "
        "exec('while Path(sys.argv[2]).exists(): time.sleep(0.02)')",
        'usb_link.control', str(hold_cleanup),
    ], cwd=PROJECT_DIR, env=environment, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL)
    stopper = next_manager = None
    try:
        state = _wait_for_state(environment, lambda item: item.get('status') == 'ready')
        assert state['transport_owner']['pid'] == owner.pid
        stopper = _manager(environment, 'stop')
        _wait_for_state(environment, lambda item: item.get('status') == 'stopped')
        time.sleep(0.15)
        assert stopper.poll() is None  # manager STOP alone is not a handoff
        early = _action(environment, 'static')
        assert early.returncode == 2
        assert 'still cleaning up' in early.stdout
        hold_cleanup.unlink()
        owner.wait(timeout=3)
        output, _ = stopper.communicate(timeout=4)
        assert stopper.returncode == 0, output
        for mode in ('static', 'control', 'static'):
            next_manager = _manager(environment, mode)
            ready = _wait_for_state(environment, lambda item:
                                    item.get('status') == 'ready' and item.get('mode') == mode)
            assert ready['session_id'] != state['session_id']
            # The finished old owner has no late global STOP to hit this mode.
            time.sleep(0.25)
            assert next_manager.poll() is None
            stopped = _action(environment, 'stop')
            assert stopped.returncode == 0, stopped.stdout
            output, _ = next_manager.communicate(timeout=4)
            assert next_manager.returncode == 0, output
    finally:
        hold_cleanup.unlink(missing_ok=True)
        if next_manager is not None:
            _stop_manager(next_manager, environment)
        if owner.poll() is None:
            owner.terminate()
            owner.wait(timeout=6)
        if stopper is not None:
            stopper.communicate(timeout=6)


def test_cleanup_timeout_never_sends_second_robot_stop(tmp_path, monkeypatch):
    from exhibition import orchestrator

    environment = _base_environment(tmp_path)
    environment['R1_EXHIBITION_STOP_WAIT_SEC'] = '0.05'
    for key, value in environment.items():
        monkeypatch.setenv(key, value)
    state = {'session_id': 'old', 'status': 'stopped', 'safe_stop_confirmed': True,
             'transport_owner': {'pid': 123, 'start_ticks': '456'}}
    monkeypatch.setattr(orchestrator, '_read_state', lambda _: state)
    monkeypatch.setattr(orchestrator, 'owner_alive', lambda _: True)
    monkeypatch.setattr(orchestrator, '_active_pid', lambda _: 123)
    monkeypatch.setattr(orchestrator.os, 'kill', lambda *_: None)
    monkeypatch.setattr(orchestrator, '_wait_for_manager_stop', lambda *_: True)
    calls = []
    monkeypatch.setattr(orchestrator, '_run_action', lambda *args: calls.append(args))
    assert orchestrator.main(['stop']) == 2
    assert calls == []
    monkeypatch.setattr(orchestrator, '_active_pid', lambda _: None)
    assert orchestrator.main(['stop']) == 2
    assert calls == []


@pytest.mark.parametrize('last_status', ['locked', 'degraded'])
def test_lock_retries_wait_for_new_ack_not_previous_error(tmp_path, monkeypatch, last_status):
    from exhibition import orchestrator
    for key, value in _base_environment(tmp_path).items():
        monkeypatch.setenv(key, value)
    previous = {'session_id': 'current', 'mode': 'control', 'session_mode': 'session_arm',
                'status': 'degraded', 'updated_at': 'old'}
    states = iter([previous, previous, {**previous, 'status': last_status, 'updated_at': 'new'}])
    monkeypatch.setattr(orchestrator, '_read_state', lambda _: next(states))
    monkeypatch.setattr(orchestrator, '_active_pid', lambda _: 123)
    signals = []
    monkeypatch.setattr(orchestrator.os, 'kill', lambda *args: signals.append(args))
    assert orchestrator.main(['lock']) == (0 if last_status == 'locked' else 2)
    assert len(signals) == 1


def test_usb_owner_rejects_pid_reuse_and_unrelated_process(monkeypatch):
    from exhibition import process_owner

    assert process_owner.usb_owner(os.getpid()) == {}
    assert process_owner.usb_owner(-1) == {}
    monkeypatch.setattr(process_owner, 'usb_owner', lambda _: {'pid': 123, 'start_ticks': 'new'})
    assert not process_owner.owner_alive({'pid': 123, 'start_ticks': 'old'})
    assert process_owner.owner_alive({'pid': 123, 'start_ticks': 'new'})


def test_plans_use_static_writer_and_control_slow_safe(tmp_path):
    environment = _base_environment(tmp_path)
    environment["R1_EXHIBITION_MOCK"] = "0"
    environment["R1_EXHIBITION_SESSION_MODE"] = "session_arm"
    environment.pop("R1_EXHIBITION_OFFLINE_POV_RECOVERY_GRACE_SEC")
    settings = ExhibitionSettings.from_environment(environment)

    static = build_plan("static", settings)
    control = build_plan("control", settings)

    assert [step["name"] for step in static["steps"]] == [
        "preflight",
        "pov",
        "static_writer",
        "ready_static",
        "prepare",
    ]
    assert static["physical_profile"] == "static-stand"
    assert static["reuse_offline_bridge"] is True
    assert static["reuse_offline_pov_when_healthy"] is True
    assert static["offline_pov_recovery_grace_sec"] == 8.0
    assert static["offline_service_policy"] == (
        "reuse-static-stop-restore-control"
    )
    assert control["physical_profile"] == "slow-safe"
    assert control["reuse_offline_bridge"] is False
    assert control["reuse_offline_pov_when_healthy"] is False
    assert control["session_mode"] == "session_arm"
    assert control["tracking_grace_sec"] == 2.0
    assert control["live_writer_restart"] is False
    control_step = next(
        step for step in control["steps"] if step["name"] == "control"
    )
    assert control_step["restart"] is False


def test_mock_pov_uses_a_dedicated_port_without_stopping_offline_service(tmp_path):
    environment = _base_environment(tmp_path)
    settings = ExhibitionSettings.from_environment(environment)

    assert settings.mock is True
    assert settings.commands["offline_status"] == []
    assert settings.commands["offline_stop"] == []
    assert settings.commands["offline_start"] == []
    assert settings.commands["pov"][-2:] == ["--port", "18080"]
    assert "8080" not in settings.commands["pov"]
    assert settings.commands["preflight"][-3:] == ["18080", "19090", "19091"]
    assert "robot-preflight" not in settings.commands["preflight"]
    assert "R1_DRY_RUN_UDP_PORT=19090" in settings.commands["control"]
    assert "R1_DRY_RUN_DISCOVERY_PORT=19091" in settings.commands["control"]
    assert "R1_DRY_RUN_UDP_BIND_ADDRESS=127.0.0.1" in settings.commands["control"]
    assert "9090" not in settings.commands["control"]
    assert "9091" not in settings.commands["control"]


def test_live_pov_leaves_discovery_port_to_vr_bridge(tmp_path):
    environment = _base_environment(tmp_path)
    environment["R1_EXHIBITION_MOCK"] = "0"
    settings = ExhibitionSettings.from_environment(environment)

    assert settings.commands["pov"][:2] == ["/usr/bin/env", "ROBOT_POV_DISCOVERY_ENABLED=false"]
    assert settings.commands["pov"][-1] == "pov-vr"


def test_offline_service_treats_expected_sigterm_exit_as_success():
    unit = (
        PROJECT_DIR / "systemd" / "user" / "r1-offline-session.service"
    ).read_text(encoding="utf-8")

    assert "SuccessExitStatus=143" in unit


def test_unitree_video_session_pins_the_matching_sdk_dds_runtime():
    script = (
        PROJECT_DIR / "scripts" / "r1-teleoperation"
    ).read_text(encoding="utf-8")

    assert 'if [[ "${VIDEO_SOURCE}" == unitree ]]' in script
    assert 'source "${SCRIPT_DIR}/r1-unitree-sdk-env"' in script


def test_connection_helper_recovers_video_without_starting_robot_control():
    script = (
        PROJECT_DIR / "scripts" / "r1-connection-ensure"
    ).read_text(encoding="utf-8")

    assert "/readyz" in script
    assert "source.frame_age_s" in script
    assert "R1_CONNECTION_FRAME_GRACE_SEC" in script
    assert "VIDEO_RECOVERING" in script
    assert "remote_service_unavailable" in script
    assert "client.ServiceList()" in script
    assert "ServiceSwitch(" not in script
    assert "R1_CONNECTION_PROBE_COOLDOWN_SEC" in script
    assert "R1_CONNECTION_RESTART_LIMIT" in script
    assert "ping -I" not in script
    assert "timeout --kill-after=1s 7s python3" in script
    assert 'systemctl --user restart "${UNIT}"' in script
    assert "r1_live_writer" not in script
    assert "SetVelocity" not in script


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("R1_EXHIBITION_SESSION_MODE", "always_on"),
        ("R1_EXHIBITION_TRACKING_GRACE_SEC", "3.1"),
        ("R1_EXHIBITION_TRACKING_GRACE_SEC", "0.9"),
        ("R1_EXHIBITION_RECOVERY_BLEND_SEC", "0.09"),
        ("R1_EXHIBITION_RECOVERY_BLEND_SEC", "2.1"),
        ("R1_EXHIBITION_POV_RESTART_LIMIT", "-1"),
        ("R1_EXHIBITION_OFFLINE_POV_RECOVERY_GRACE_SEC", "0"),
        ("R1_EXHIBITION_OFFLINE_POV_RECOVERY_GRACE_SEC", "15.1"),
    ],
)
def test_invalid_exhibition_settings_fail_closed(tmp_path, name, value):
    environment = _base_environment(tmp_path)
    environment[name] = value
    with pytest.raises(ValueError):
        ExhibitionSettings.from_environment(environment)


def test_static_session_reconnects_and_stop_finds_execed_python_pid(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    environment.update(
        {
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(worker, "ok"),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "hold"),
            # A value which must never be copied into state or logs.
            "ROBOT_COMMISSIONING_TOKEN": "do-not-persist-this-token",
        }
    )
    process = _manager(environment, "static")
    try:
        state = _wait_for_state(
            environment, lambda item: item.get("status") == "ready"
        )
        assert state["mode"] == "static"
        assert "pov" in state["children"]
        original_children = dict(state["children"])

        reconnect = _action(environment, "reconnect")
        assert reconnect.returncode == 0, reconnect.stdout
        reconnected = _wait_for_state(
            environment,
            lambda item: item.get("reconnect_count") == 1,
        )
        assert reconnected["detail"] == (
            "all managed components already healthy; nothing restarted"
        )
        assert reconnected["children"] == original_children

        stopped = _action(environment, "stop")
        assert stopped.returncode == 0, stopped.stdout
        output, _ = process.communicate(timeout=6)
        assert process.returncode == 0, output
        state = _wait_for_state(
            environment, lambda item: item.get("status") == "stopped"
        )
        persisted = json.dumps(state) + Path(state["log"]).read_text(
            encoding="utf-8"
        )
        assert "do-not-persist-this-token" not in persisted
    finally:
        _stop_manager(process, environment)


def test_active_offline_service_is_handed_off_until_exhibition_stop(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    events = tmp_path / "events"
    offline_state = tmp_path / "offline-state"
    offline_state.write_text("active", encoding="utf-8")
    environment.update(
        {
            "R1_TEST_EVENTS": str(events),
            "R1_TEST_OFFLINE_STATE": str(offline_state),
            "R1_EXHIBITION_OFFLINE_STATUS_COMMAND": _command(
                worker, "offline-status"
            ),
            "R1_EXHIBITION_OFFLINE_STOP_COMMAND": _command(
                worker, "offline-stop"
            ),
            "R1_EXHIBITION_OFFLINE_START_COMMAND": _command(
                worker, "offline-start"
            ),
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(
                worker, "preflight"
            ),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "pov"),
        }
    )
    process = _manager(environment, "static")
    try:
        state = _wait_for_state(
            environment, lambda item: item.get("status") == "ready"
        )
        assert offline_state.read_text(encoding="utf-8") == "inactive"
        assert state["offline_session_handoff"] == {
            "was_active": True,
            "stopped_for_exhibition": True,
            "restored": False,
            "reused_for_static": False,
            "pov_owner": "manager",
        }

        reconnect = _action(environment, "reconnect")
        assert reconnect.returncode == 0, reconnect.stdout
        reconnected = _wait_for_state(
            environment,
            lambda item: item.get("reconnect_count") == 1,
        )
        assert reconnected["detail"] == (
            "all managed components already healthy; nothing restarted"
        )
        assert offline_state.read_text(encoding="utf-8") == "inactive"
        assert "offline-start" not in events.read_text(
            encoding="utf-8"
        ).splitlines()
    finally:
        output, _ = _stop_manager(process, environment)
        assert process.returncode == 0, output

    state = _wait_for_state(
        environment, lambda item: item.get("status") == "stopped"
    )
    assert offline_state.read_text(encoding="utf-8") == "active"
    assert state["offline_session_handoff"]["restored"] is True
    sequence = events.read_text(encoding="utf-8").splitlines()
    assert sequence.index("offline-stop") < sequence.index("preflight")
    assert sequence.index("pov-stopped") < sequence.index("offline-start")


def test_static_reuses_healthy_offline_pov_and_reconnect_repairs_only_it(
    tmp_path,
):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    events = tmp_path / "events"
    offline_state = tmp_path / "offline-state"
    offline_state.write_text("active", encoding="utf-8")
    environment.update(
        {
            "R1_TEST_EVENTS": str(events),
            "R1_TEST_OFFLINE_STATE": str(offline_state),
            "R1_EXHIBITION_OFFLINE_STATUS_COMMAND": _command(
                worker, "offline-status"
            ),
            "R1_EXHIBITION_OFFLINE_POV_STATUS_COMMAND": _command(
                worker, "offline-pov-status"
            ),
            "R1_EXHIBITION_OFFLINE_STOP_COMMAND": _command(
                worker, "offline-stop"
            ),
            "R1_EXHIBITION_OFFLINE_START_COMMAND": _command(
                worker, "offline-start"
            ),
            "R1_EXHIBITION_OFFLINE_RESTART_COMMAND": _command(
                worker, "offline-restart"
            ),
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(
                worker, "preflight"
            ),
            # A second manager-owned POV would be observable in the event
            # stream and is forbidden while the healthy service is reused.
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "pov"),
        }
    )
    process = _manager(environment, "static")
    try:
        state = _wait_for_state(
            environment, lambda item: item.get("status") == "ready"
        )
        assert state["offline_session_handoff"] == {
            "was_active": True,
            "stopped_for_exhibition": False,
            "restored": False,
            "reused_for_static": True,
            "pov_owner": "offline_service",
        }
        assert "pov" not in state["children"]
        sequence = events.read_text(encoding="utf-8").splitlines()
        assert "offline-stop" not in sequence
        assert "pov" not in sequence

        offline_state.write_text("inactive", encoding="utf-8")
        reconnect = _action(environment, "reconnect")
        assert reconnect.returncode == 0, reconnect.stdout
        repaired = _wait_for_state(
            environment,
            lambda item: item.get("reconnect_count") == 1,
        )
        assert repaired["status"] == "ready"
        assert repaired["pov_restarts"] == 1
        assert repaired["detail"] == (
            "systemd-managed POV restored; static writer stayed warm"
        )
        assert offline_state.read_text(encoding="utf-8") == "active"
        sequence = events.read_text(encoding="utf-8").splitlines()
        assert sequence.count("offline-restart") == 1
        assert "pov" not in sequence
    finally:
        output, _ = _stop_manager(process, environment)
        assert process.returncode == 0, output

    assert offline_state.read_text(encoding="utf-8") == "active"
    sequence = events.read_text(encoding="utf-8").splitlines()
    assert "offline-start" not in sequence


def test_static_reconnect_waits_for_process_local_offline_pov_respawn(
    tmp_path, monkeypatch
):
    manager = ExhibitionManager(
        ExhibitionSettings.from_environment(_base_environment(tmp_path)),
        "static",
    )
    manager._status = "ready"
    manager.offline_service_reused = True
    manager.children["static_writer"] = type(
        "AliveProcess", (), {"poll": lambda self: None}
    )()
    states = []
    probes = []

    def probe(name="offline_status"):
        probes.append(name)
        return False

    monkeypatch.setattr(manager, "_probe_offline_service", probe)
    monkeypatch.setattr(manager, "_wait_for_offline_pov_recovery", lambda: True)
    monkeypatch.setattr(
        manager,
        "_run_checked",
        lambda name, _timeout: pytest.fail(
            f"healthy parent service must not restart: {name}"
        ),
    )
    monkeypatch.setattr(
        manager,
        "_write_state",
        lambda status, detail: states.append((status, detail)),
    )
    monkeypatch.setattr(manager, "_log", lambda _message: None)

    assert manager._reconnect() is True
    assert probes == ["offline_pov_status"]
    assert manager.pov_restarts == 1
    assert manager.reconnect_count == 1
    assert states == [
        ("reconnecting", "waiting for process-local offline POV recovery"),
        (
            "ready",
            "systemd-managed POV auto-recovered; writer and VR bridge stayed warm",
        ),
    ]


def test_offline_handoff_probe_failure_blocks_before_children(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    events = tmp_path / "events"
    environment.update(
        {
            "R1_TEST_EVENTS": str(events),
            "R1_EXHIBITION_OFFLINE_STATUS_COMMAND": _command(
                worker, "hard-fail"
            ),
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(
                worker, "preflight"
            ),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "pov"),
        }
    )
    result = subprocess.run(
        [sys.executable, "-m", "exhibition.orchestrator", "static"],
        cwd=PROJECT_DIR,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=8,
        check=False,
    )
    assert result.returncode == 2, result.stdout
    assert events.read_text(encoding="utf-8").splitlines() == ["hard-fail"]


def test_offline_service_is_restored_after_post_stop_probe_failure(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    offline_state = tmp_path / "offline-state"
    probe_count = tmp_path / "probe-count"
    offline_state.write_text("active", encoding="utf-8")
    environment.update(
        {
            "R1_TEST_OFFLINE_STATE": str(offline_state),
            "R1_TEST_OFFLINE_PROBE_COUNT": str(probe_count),
            "R1_TEST_FAIL_SECOND_OFFLINE_PROBE": "1",
            "R1_EXHIBITION_OFFLINE_STATUS_COMMAND": _command(
                worker, "offline-status"
            ),
            "R1_EXHIBITION_OFFLINE_STOP_COMMAND": _command(
                worker, "offline-stop"
            ),
            "R1_EXHIBITION_OFFLINE_START_COMMAND": _command(
                worker, "offline-start"
            ),
        }
    )
    result = subprocess.run(
        [sys.executable, "-m", "exhibition.orchestrator", "static"],
        cwd=PROJECT_DIR,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=8,
        check=False,
    )
    assert result.returncode == 2, result.stdout
    assert offline_state.read_text(encoding="utf-8") == "active"
    state = _wait_for_state(
        environment, lambda item: item.get("status") == "stopped"
    )
    assert state["offline_session_handoff"]["restored"] is True


def test_pov_is_restarted_without_restarting_any_writer(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    counter = tmp_path / "pov-count"
    environment.update(
        {
            "R1_TEST_POV_COUNTER": str(counter),
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(worker, "ok"),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "pov-restart"),
            "R1_EXHIBITION_POV_RESTART_LIMIT": "2",
        }
    )
    process = _manager(environment, "static")
    try:
        state = _wait_for_state(
            environment,
            lambda item: item.get("pov_restarts", 0) >= 1
            and item.get("status") == "ready"
            and counter.exists()
            and counter.read_text(encoding="utf-8") == "2",
        )
        assert state["pov_restarts"] == 1
        assert int(counter.read_text(encoding="utf-8")) == 2
    finally:
        output, _ = _stop_manager(process, environment)
        assert process.returncode == 0, output


@pytest.mark.parametrize('previous_status', ['locked', 'degraded', 'blocked', 'ready'])
def test_video_restart_preserves_locked_control(tmp_path, monkeypatch, previous_status):
    manager = ExhibitionManager(
        ExhibitionSettings.from_environment(_base_environment(tmp_path)), 'control'
    )
    manager._status = previous_status
    manager.pov_restarts = 5000  # unlimited camera retries must not overflow
    monkeypatch.setattr(manager, '_terminate_child', lambda name: None)
    monkeypatch.setattr(manager, '_start_child', lambda name: name == 'pov')
    monkeypatch.setattr(manager, '_log', lambda message: None)
    states = []

    def write_state(status, detail):
        manager._status = status
        states.append(status)

    monkeypatch.setattr(manager, '_write_state', write_state)
    assert manager._restart_pov()
    assert states == ['reconnecting', previous_status]


def test_selective_reconnect_restarts_only_a_dead_pov(tmp_path, monkeypatch):
    manager = ExhibitionManager(
        ExhibitionSettings.from_environment(_base_environment(tmp_path)), "static"
    )
    manager._status = "ready"
    manager.children["pov"] = type(
        "ExitedProcess", (), {"poll": lambda self: 9}
    )()
    restarts = []
    states = []
    monkeypatch.setattr(
        manager,
        "_restart_pov",
        lambda mark_ready=True: restarts.append(mark_ready) or True,
    )
    monkeypatch.setattr(
        manager,
        "_write_state",
        lambda status, detail: states.append((status, detail)),
    )
    monkeypatch.setattr(manager, "_log", lambda _message: None)

    assert manager._reconnect() is True
    assert restarts == [True]
    assert manager.reconnect_count == 1
    assert states == [
        (
            "ready",
            "POV restored; existing writer and VR/ROS graph stayed warm",
        )
    ]


def test_selective_reconnect_reports_latched_writer_without_resuming_or_rebuild(
    tmp_path, monkeypatch
):
    environment = _base_environment(tmp_path)
    environment.update(
        R1_EXHIBITION_MOCK="0",
        R1_EXHIBITION_SESSION_MODE="session_arm",
    )
    manager = ExhibitionManager(
        ExhibitionSettings.from_environment(environment), "control"
    )
    manager._status = "locked"
    alive = type("AliveProcess", (), {"poll": lambda self: None})
    manager.children["control"] = alive()
    manager.children["pov"] = alive()
    checked = []
    states = []

    def health(name, timeout):
        checked.append((name, timeout))
        manager.last_action_exit_code = 3
        return False

    monkeypatch.setattr(manager, "_run_checked", health)
    monkeypatch.setattr(manager, "_log", lambda _message: None)
    monkeypatch.setattr(
        manager,
        "_write_state",
        lambda status, detail: states.append((status, detail)),
    )
    monkeypatch.setattr(
        manager,
        "_restart_pov",
        lambda **_kwargs: pytest.fail("healthy POV must not restart"),
    )
    monkeypatch.setattr(
        manager,
        "_rebuild_writer",
        lambda **_kwargs: pytest.fail("reconnect must not rebuild writer"),
    )

    assert manager._reconnect() is True
    assert checked == [("health_session", 8.0)]
    assert manager.reconnect_count == 1
    assert states == [
        (
            "degraded",
            "writer safety latched after link/feedback loss; reconnect kept "
            "services warm but explicit RUN is required",
        )
    ]


def test_stop_interrupts_long_startup_check(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    events = tmp_path / 'events'
    environment.update(R1_TEST_EVENTS=str(events),
                       R1_EXHIBITION_PREFLIGHT_TIMEOUT_SEC='60',
                       R1_EXHIBITION_PREFLIGHT_COMMAND=_command(worker, 'long-check'))
    process = _manager(environment, 'static')
    try:
        deadline = time.monotonic() + 3
        while not events.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert events.exists()
        started = time.monotonic()
        result = _action(environment, 'stop')
        assert result.returncode == 0, result.stdout
        process.communicate(timeout=3)
        assert time.monotonic() - started < 2
        assert 'long-check-stopped' in events.read_text()
    finally:
        _stop_manager(process, environment)


@pytest.mark.parametrize('action', ['prepare', 'arm_session', 'warm_resume', 'clear_emergency'])
def test_stop_during_startup_cancels_later_activation(tmp_path, monkeypatch, action):
    manager = ExhibitionManager(
        ExhibitionSettings.from_environment(_base_environment(tmp_path)), 'control'
    )
    manager.stop_requested = True
    monkeypatch.setattr(manager, '_log', lambda message: None)

    def unexpected_run(*args, **kwargs):
        pytest.fail('activation must not run after a stop request')

    monkeypatch.setattr(subprocess, 'run', unexpected_run)
    assert not manager._run_checked(action, 1)
    assert manager.last_action_exit_code == 2


def test_control_child_exit_fails_closed_instead_of_auto_restart(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    environment.update(
        {
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(worker, "ok"),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "hold"),
            "R1_EXHIBITION_CONTROL_COMMAND": _command(worker, "fail-soon"),
            "R1_EXHIBITION_PREPARE_COMMAND": _command(worker, "ok"),
        }
    )
    process = _manager(environment, "control")
    output, _ = process.communicate(timeout=8)

    assert process.returncode == 1, output
    state = _wait_for_state(
        environment, lambda item: item.get("status") == "stopped"
    )
    assert state["mode"] == "control"
    assert state["children"] == {}
    assert output.count("control: starting") == 1


def test_selective_reconnect_never_stops_or_rebuilds_healthy_writer(tmp_path):
    """Reconnect preserves a healthy writer even when STOP would be available."""

    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    environment.update(
        {
            # Exercise the physical-manager branch using local test workers;
            # every external command is overridden, so no ROS/SDK process is
            # constructed by this test.
            "R1_EXHIBITION_MOCK": "0",
            "R1_EXHIBITION_SESSION_MODE": "deadman",
            "R1_TEST_EVENTS": str(tmp_path / "events"),
            "R1_EXHIBITION_OFFLINE_STATUS_COMMAND": _command(
                worker, "offline-status"
            ),
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(worker, "ok"),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "hold"),
            "R1_EXHIBITION_STATIC_WRITER_COMMAND": _command(
                worker, "static-writer"
            ),
            "R1_EXHIBITION_READY_STATIC_COMMAND": _command(worker, "ok"),
            "R1_EXHIBITION_PREPARE_COMMAND": _command(worker, "ok"),
            "R1_EXHIBITION_STOP_COMMAND": _command(worker, "stop"),
            "R1_EXHIBITION_KILL_COMMAND": _command(worker, "hard-fail"),
        }
    )
    process = _manager(environment, "static")
    try:
        _wait_for_state(
            environment, lambda item: item.get("status") == "ready"
        )
        reconnect = _action(environment, "reconnect")
        assert reconnect.returncode == 0, reconnect.stdout
        state = _wait_for_state(
            environment, lambda item: item.get("reconnect_count") == 1
        )
        assert state["status"] == "ready"
        assert process.poll() is None
        events_before_stop = Path(environment["R1_TEST_EVENTS"]).read_text(
            encoding="utf-8"
        ).splitlines()
        assert events_before_stop.count("static-writer") == 1
        assert "stop" not in events_before_stop
    finally:
        output, _ = _stop_manager(process, environment)
        assert process.returncode == 0, output


def test_manager_disappearance_is_not_stop_confirmation(tmp_path, monkeypatch):
    """The independent fallback remains required after an unclean exit."""

    environment = _base_environment(tmp_path)
    environment["R1_EXHIBITION_MOCK"] = "0"
    settings = ExhibitionSettings.from_environment(environment)
    state = {"session_id": "lost", "status": "ready"}

    def process_is_gone(_pid, _signal):
        raise ProcessLookupError

    monkeypatch.setattr(os, "kill", process_is_gone)
    assert _wait_for_manager_stop(settings, state, 999999) is False


def test_safe_stop_retries_after_unconfirmed_stop_and_kill(tmp_path):
    environment = _base_environment(tmp_path)
    environment["R1_EXHIBITION_MOCK"] = "0"
    manager = ExhibitionManager(
        ExhibitionSettings.from_environment(environment), "control"
    )
    calls = []

    def first_attempt(name, _timeout):
        calls.append(name)
        return False

    manager._run_checked = first_attempt
    assert manager._safe_stop() is False
    assert calls == ["safe_stop", "stop", "kill"]
    assert manager._safe_stop_completed is False

    manager._run_checked = lambda name, _timeout: name == "stop"
    assert manager._safe_stop() is True
    assert manager._safe_stop_confirmed is True


def test_combined_safe_stop_avoids_duplicate_stop_helpers(tmp_path):
    environment = _base_environment(tmp_path)
    environment["R1_EXHIBITION_MOCK"] = "0"
    manager = ExhibitionManager(
        ExhibitionSettings.from_environment(environment), "control"
    )
    calls = []
    manager._run_checked = lambda name, timeout: (
        calls.append((name, timeout)) or name == "safe_stop"
    )

    assert manager._safe_stop() is True
    assert calls == [("safe_stop", 26.0)]
    assert manager._safe_stop_confirmed is True


def test_confirmed_stop_retires_independent_children_in_parallel(
    tmp_path, monkeypatch
):
    manager = ExhibitionManager(
        ExhibitionSettings.from_environment(_base_environment(tmp_path)),
        "control",
    )
    events = []

    class Child:
        def __init__(self, pid):
            self.pid = pid
            self.signalled = False

        def poll(self):
            events.append(("poll", self.pid, self.signalled))
            return 0 if self.signalled else None

    pov = Child(101)
    control = Child(202)
    manager.children = {"pov": pov, "control": control}

    def kill_group(pid, sent_signal):
        events.append(("signal", pid, sent_signal))
        {101: pov, 202: control}[pid].signalled = True

    monkeypatch.setattr(os, "killpg", kill_group)
    monkeypatch.setattr(manager, "_log", lambda _message: None)

    assert manager._terminate_children(("pov", "control")) is True
    term_events = [
        event for event in events
        if event[0] == "signal" and event[2] == signal.SIGTERM
    ]
    assert term_events == [
        ("signal", 101, signal.SIGTERM),
        ("signal", 202, signal.SIGTERM),
    ]
    first_completion_poll = next(
        index for index, event in enumerate(events)
        if event[0] == "poll" and event[2]
    )
    assert events.index(term_events[-1]) < first_completion_poll
    assert manager.children == {}


def test_saved_calibrated_arm_profile_preserves_body_and_sets_head_start_reference(tmp_path):
    environment = _base_environment(tmp_path)
    environment.update(R1_EXHIBITION_MOCK="0", R1_RESPONSE_PROFILE="exhibition",
                       R1_EXHIBITION_PROJECT_DIR=str(tmp_path))
    (tmp_path / "config").mkdir()
    (tmp_path / "config/operator_arm_range.exhibition.json").write_text("{}")
    settings = ExhibitionSettings.from_environment(environment)
    assert settings.calibrate_on_start
    assert settings.commands['startup_calibrate'] == [
        str(tmp_path / 'scripts/r1-head-calibrate')
    ]
    assert settings.commands['calibrate'] == [
        str(tmp_path / 'scripts/r1-exhibition-calibrate')
    ]
    environment["R1_EXHIBITION_CALIBRATE_ON_START"] = "0"
    assert not ExhibitionSettings.from_environment(environment).calibrate_on_start


def test_session_arm_activates_once_before_writer_internal_rearm(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    events = tmp_path / "events"
    environment.update(
        {
            "R1_TEST_EVENTS": str(events),
            "R1_EXHIBITION_SESSION_MODE": "session_arm",
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(worker, "preflight"),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "hold"),
            "R1_EXHIBITION_CONTROL_COMMAND": _command(worker, "hold"),
            "R1_EXHIBITION_PREPARE_COMMAND": _command(worker, "prepare"),
            "R1_EXHIBITION_ARM_SESSION_COMMAND": _command(worker, "arm"),
            "R1_EXHIBITION_DISARM_SESSION_COMMAND": _command(
                worker, "disarm"
            ),
        }
    )
    process = _manager(environment, "control")
    try:
        _wait_for_state(
            environment, lambda item: item.get("status") == "ready"
        )
    finally:
        output, _ = _stop_manager(process, environment)
        assert process.returncode == 0, output

    sequence = events.read_text(encoding="utf-8").splitlines()
    # POV/control are concurrent, so assert only the safety-critical service
    # subsequence. Writer prepared_session() performs its internal re-arm; an
    # external disarm after prepare would latch KILL. Normal STOP disarms once.
    critical = [
        event
        for event in sequence
        if event in {"arm", "prepare", "disarm"}
    ]
    assert critical == ["arm", "prepare", "disarm"]


def test_static_live_path_waits_for_services_then_prepares_and_stops(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    events = tmp_path / "events"
    environment.update(
        {
            "R1_EXHIBITION_MOCK": "0",
            "R1_EXHIBITION_SESSION_MODE": "deadman",
            "R1_TEST_EVENTS": str(events),
            "R1_TEST_CAPTURE_ENV": "1",
            "R1_EXHIBITION_OFFLINE_STATUS_COMMAND": _command(
                worker, "offline-status"
            ),
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(
                worker, "preflight"
            ),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "pov"),
            "R1_EXHIBITION_STATIC_WRITER_COMMAND": _command(
                worker, "static-writer"
            ),
            "R1_EXHIBITION_READY_STATIC_COMMAND": _command(
                worker, "ready-delay"
            ),
            "R1_EXHIBITION_PREPARE_COMMAND": _command(worker, "prepare"),
            "R1_EXHIBITION_STOP_COMMAND": _command(worker, "stop"),
            "R1_EXHIBITION_KILL_COMMAND": _command(worker, "kill"),
        }
    )
    process = _manager(environment, "static")
    try:
        state = _wait_for_state(
            environment, lambda item: item.get("status") == "ready"
        )
        assert "static_writer" in state["children"]
        sequence = events.read_text(encoding="utf-8").splitlines()
        assert sequence.index("ready-complete") < sequence.index("prepare")
        assert any(
            entry
            == "env:static-writer:static=1:session=deadman:exhibition="
            for entry in sequence
        )
        assert any(
            entry == "env:prepare:static=1:session=deadman:exhibition="
            for entry in sequence
        )
    finally:
        output, _ = _stop_manager(process, environment)
        assert process.returncode == 0, output

    sequence = events.read_text(encoding="utf-8").splitlines()
    assert sequence.count("stop") == 1
    assert sequence.index("stop") < sequence.index("static-writer-stopped")


def test_control_waits_for_typed_readiness_before_arm_and_prepare(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    events = tmp_path / "events"
    environment.update(
        {
            "R1_TEST_EVENTS": str(events),
            "R1_TEST_CAPTURE_ENV": "1",
            "R1_EXHIBITION_SESSION_MODE": "session_arm",
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(
                worker, "preflight"
            ),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "pov"),
            "R1_EXHIBITION_CONTROL_COMMAND": _command(worker, "control"),
            "R1_EXHIBITION_READY_CONTROL_COMMAND": _command(
                worker, "ready-delay"
            ),
            "R1_EXHIBITION_ARM_SESSION_COMMAND": _command(worker, "arm"),
            "R1_EXHIBITION_DISARM_SESSION_COMMAND": _command(
                worker, "disarm"
            ),
            "R1_EXHIBITION_PREPARE_COMMAND": _command(worker, "prepare"),
        }
    )
    process = _manager(environment, "control")
    try:
        _wait_for_state(
            environment, lambda item: item.get("status") == "ready"
        )
        sequence = events.read_text(encoding="utf-8").splitlines()
        assert sequence.index("ready-complete") < sequence.index("arm")
        assert sequence.index("arm") < sequence.index("prepare")
        assert any(
            entry
            == "env:control:static=:session=session_arm:exhibition=1"
            for entry in sequence
        )
    finally:
        output, _ = _stop_manager(process, environment)
        assert process.returncode == 0, output


def test_session_arm_calibration_pauses_and_resumes_without_disarm(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    events = tmp_path / "events"
    environment.update(
        {
            "R1_TEST_EVENTS": str(events),
            "R1_EXHIBITION_SESSION_MODE": "session_arm",
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(
                worker, "preflight"
            ),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "pov"),
            "R1_EXHIBITION_CONTROL_COMMAND": _command(worker, "control"),
            "R1_EXHIBITION_PREPARE_COMMAND": _command(worker, "prepare"),
            "R1_EXHIBITION_ARM_SESSION_COMMAND": _command(worker, "arm"),
            "R1_EXHIBITION_DISARM_SESSION_COMMAND": _command(
                worker, "disarm"
            ),
            "R1_EXHIBITION_PAUSE_SESSION_COMMAND": _command(
                worker, "pause"
            ),
            "R1_EXHIBITION_RESUME_SESSION_COMMAND": _command(
                worker, "resume"
            ),
            "R1_EXHIBITION_CALIBRATE_COMMAND": _command(
                worker, "calibrate"
            ),
        }
    )
    process = _manager(environment, "control")
    try:
        _wait_for_state(
            environment, lambda item: item.get("status") == "ready"
        )
        before = len(events.read_text(encoding="utf-8").splitlines())
        result = _action(environment, "calibrate")
        assert result.returncode == 0, result.stdout
        calibration = events.read_text(encoding="utf-8").splitlines()[before:]
        assert calibration == ["pause", "calibrate", "resume"]
        assert "disarm" not in calibration
    finally:
        output, _ = _stop_manager(process, environment)
        assert process.returncode == 0, output


def test_latched_stop_run_retires_old_writer_but_preserves_pov_and_manager(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    events = tmp_path / "events"
    environment.update(
        {
            "R1_TEST_EVENTS": str(events),
            "R1_EXHIBITION_CALIBRATE_ON_START": "1",
            "R1_EXHIBITION_WARM_RESUME_COMMAND": _command(worker, "needs-rearm"),
            "R1_EXHIBITION_SESSION_MODE": "session_arm",
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(worker, "preflight"),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "pov"),
            "R1_EXHIBITION_CONTROL_COMMAND": _command(worker, "control"),
            "R1_EXHIBITION_READY_CONTROL_COMMAND": _command(worker, "ready-delay"),
            "R1_EXHIBITION_PREPARE_COMMAND": _command(worker, "prepare"),
            "R1_EXHIBITION_ARM_SESSION_COMMAND": _command(worker, "arm"),
            "R1_EXHIBITION_DISARM_SESSION_COMMAND": _command(worker, "disarm"),
            "R1_EXHIBITION_PAUSE_SESSION_COMMAND": _command(worker, "pause"),
            "R1_EXHIBITION_RESUME_SESSION_COMMAND": _command(worker, "resume"),
            "R1_EXHIBITION_CLEAR_EMERGENCY_COMMAND": _command(worker, "clear"),
            "R1_EXHIBITION_CALIBRATE_COMMAND": _command(worker, "calibrate"),
        }
    )
    process = _manager(environment, "control")
    try:
        initial = _wait_for_state(environment, lambda item: item.get("status") == "ready")
        for _ in range(2):
            before = len(events.read_text(encoding="utf-8").splitlines())
            result = _action(environment, "rearm")
            assert result.returncode == 0, result.stdout
            rearm = events.read_text(encoding="utf-8").splitlines()[before:]
            assert rearm.count('control') == 1
            assert rearm.index('control-stopped') < rearm.index('control')
            assert rearm.index('control') < rearm.index('ready-complete')
            # Concurrent child/readiness process startup order is irrelevant;
            # calibration must wait for the readiness acknowledgement.
            assert [event for event in rearm if event not in {'control', 'ready-delay'}] == [
                "needs-rearm", "disarm", "control-stopped", "preflight",
                "ready-complete", "calibrate", "arm", "prepare",
            ]
            current = _wait_for_state(environment, lambda item: item.get("status") == "ready")
            assert current['pid'] == initial['pid']
            assert current['children']['pov'] == initial['children']['pov']
            assert current['children']['control'] != initial['children']['control']
            initial = current
        # No pause/resume of the old, possibly disarmed session; no count=2
        # exception is needed for its stale DDS command publisher.
        assert process.poll() is None
    finally:
        output, _ = _stop_manager(process, environment)
        assert process.returncode == 0, output


def test_latched_stop_run_cannot_rebuild_without_confirmed_stop(tmp_path, monkeypatch):
    manager = ExhibitionManager(
        ExhibitionSettings.from_environment(_base_environment(tmp_path)), 'control'
    )
    states = []

    def needs_rearm(name, timeout):
        assert name == 'warm_resume'
        manager.last_action_exit_code = 3
        return False

    monkeypatch.setattr(manager, '_run_checked', needs_rearm)
    monkeypatch.setattr(manager, '_safe_stop', lambda: False)
    monkeypatch.setattr(
        manager, '_write_state',
        lambda status, detail: states.append((status, detail))
    )
    monkeypatch.setattr(manager, '_log', lambda message: None)
    monkeypatch.setattr(manager, '_terminate_child', lambda name: pytest.fail('unconfirmed STOP'))
    monkeypatch.setattr(manager, '_start_child', lambda name: pytest.fail('unconfirmed STOP'))
    manager._resume_control()
    assert states[-1] == ('blocked', 'STOP/KILL unconfirmed; writer reconnect aborted')


def test_fast_lock_pauses_control_without_restarting_graph(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    events = tmp_path / "events"
    environment.update(
        {
            "R1_TEST_EVENTS": str(events),
            "R1_EXHIBITION_WARM_RESUME_COMMAND": _command(worker, "resume"),
            "R1_EXHIBITION_SESSION_MODE": "session_arm",
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(worker, "preflight"),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "pov"),
            "R1_EXHIBITION_CONTROL_COMMAND": _command(worker, "control"),
            "R1_EXHIBITION_PREPARE_COMMAND": _command(worker, "prepare"),
            "R1_EXHIBITION_ARM_SESSION_COMMAND": _command(worker, "arm"),
            "R1_EXHIBITION_DISARM_SESSION_COMMAND": _command(worker, "disarm"),
            "R1_EXHIBITION_PAUSE_SESSION_COMMAND": _command(worker, "pause"),
            "R1_EXHIBITION_RESUME_SESSION_COMMAND": _command(worker, "resume"),
            "R1_EXHIBITION_CALIBRATE_COMMAND": _command(worker, "calibrate"),
        }
    )
    process = _manager(environment, "control")
    try:
        _wait_for_state(environment, lambda item: item.get("status") == "ready")
        before = len(events.read_text(encoding="utf-8").splitlines())

        result = _action(environment, "lock")

        assert result.returncode == 0, result.stdout
        state = _wait_for_state(
            environment, lambda item: item.get("status") == "locked"
        )
        assert state["mode"] == "control"
        assert events.read_text(encoding="utf-8").splitlines()[before:] == [
            "pause"
        ]
        assert process.poll() is None

        before_run = len(events.read_text(encoding="utf-8").splitlines())
        run_result = _action(environment, "rearm")
        assert run_result.returncode == 0, run_result.stdout
        _wait_for_state(
            environment, lambda item: item.get("status") == "ready"
        )
        assert events.read_text(encoding="utf-8").splitlines()[before_run:] == [
            "resume"
        ]
        assert process.poll() is None
    finally:
        output, _ = _stop_manager(process, environment)
        assert process.returncode == 0, output


def test_rejected_warm_resume_does_not_change_fsm_outside_writer(tmp_path):
    worker = _write_worker(tmp_path)
    environment = _base_environment(tmp_path)
    events = tmp_path / "events"
    environment.update(
        {
            "R1_TEST_EVENTS": str(events),
            "R1_EXHIBITION_WARM_RESUME_COMMAND": _command(worker, "hard-fail"),
            "R1_EXHIBITION_SESSION_MODE": "session_arm",
            "R1_EXHIBITION_PREFLIGHT_COMMAND": _command(worker, "preflight"),
            "R1_EXHIBITION_POV_COMMAND": _command(worker, "pov"),
            "R1_EXHIBITION_CONTROL_COMMAND": _command(worker, "control"),
            "R1_EXHIBITION_PREPARE_COMMAND": _command(worker, "prepare"),
            "R1_EXHIBITION_ARM_SESSION_COMMAND": _command(worker, "arm"),
            "R1_EXHIBITION_PAUSE_SESSION_COMMAND": _command(worker, "pause"),
            "R1_EXHIBITION_RESUME_SESSION_COMMAND": _command(worker, "hard-fail"),
            "R1_EXHIBITION_CLEAR_EMERGENCY_COMMAND": _command(worker, "hard-fail"),
            # Old process environments must not resurrect the removed external
            # FSM helper. These tripwires would record a prepare on either edge.
            "R1_EXHIBITION_FSM_LOCK_COMMAND": _command(worker, "prepare"),
            "R1_EXHIBITION_FSM_RUN_COMMAND": _command(worker, "prepare"),
        }
    )
    process = _manager(environment, "control")
    try:
        ready = _wait_for_state(
            environment, lambda item: item.get("status") == "ready"
        )
        before = len(events.read_text(encoding="utf-8").splitlines())
        locked = _action(environment, "lock")
        assert locked.returncode == 0, locked.stdout
        assert events.read_text(encoding="utf-8").splitlines()[before:] == ["pause"]

        before_run = len(events.read_text(encoding="utf-8").splitlines())
        resumed = _action(environment, "rearm")

        assert resumed.returncode == 2, resumed.stdout
        # Ordinary tracking/service failures do not clear KILL or cycle FSM.
        assert events.read_text(encoding="utf-8").splitlines()[before_run:] == [
            "hard-fail"
        ]
        degraded = _wait_for_state(
            environment, lambda item: item.get("status") == "degraded"
        )
        assert degraded["children"] == ready["children"]
        assert process.poll() is None
    finally:
        output, _ = _stop_manager(process, environment)
        assert process.returncode == 0, output


def test_direct_control_plan_defaults_to_session_arm(tmp_path):
    environment = _base_environment(tmp_path)
    environment.pop("R1_EXHIBITION_SESSION_MODE", None)
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "exhibition.orchestrator",
            "plan",
            "control",
        ],
        cwd=PROJECT_DIR,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=8,
        check=False,
    )
    assert result.returncode == 0, result.stdout
    assert json.loads(result.stdout)["session_mode"] == "session_arm"


def test_makefile_exposes_all_exhibition_entry_points():
    makefile = (PROJECT_DIR / "Makefile").read_text(encoding="utf-8")
    for target in (
        "exhibition-panel",
        "exhibition-static",
        "exhibition-control",
        "exhibition-reconnect",
        "exhibition-lock",
        "exhibition-rearm",
        "exhibition-calibrate",
        "exhibition-stop",
        "exhibition-status",
    ):
        assert f"{target}:" in makefile
    assert "R1_EXHIBITION_SESSION_MODE:-session_arm" in makefile
    assert "ROBOT_EXHIBITION_SESSION=1" in makefile
    assert "SAFETY_PROFILE=exhibition" in makefile
    static_recipe = makefile.split("exhibition-static:", 1)[1].split(
        "exhibition-control:", 1
    )[0]
    assert "R1_EXHIBITION_SESSION_MODE=deadman" in static_recipe
    assert "ROBOT_CONFIRM_STATIC_PREPARE=1" in static_recipe


def test_session_gate_never_clears_robot_kill_or_constructs_sdk():
    wrapper = (
        PROJECT_DIR / "scripts" / "r1-exhibition-session-gate"
    ).read_text(encoding="utf-8")
    client = (
        PROJECT_DIR / "scripts" / "r1-exhibition-ros-client"
    ).read_text(encoding="utf-8")
    script = wrapper + client
    assert "/vr/teleop/arm_session" in script
    assert "/vr/teleop/disarm_session" in script
    assert "/vr/teleop/pause_session" in script
    assert "/vr/teleop/resume_session" in script
    assert "/vr/teleop/clear_emergency_stop" in script
    assert "health" in wrapper
    health_body = client.split("def run_health", 1)[1].split(
        "def run_safe_stop", 1
    )[0]
    assert ".trigger(" not in health_body
    assert "/r1/live_writer/reset_kill" not in script
    assert "/r1/safety/set_kill" not in script
    assert "unitree_sdk" not in script.lower()


def test_control_prepare_enables_only_token_bound_fast_path(tmp_path):
    environment = _base_environment(tmp_path)
    environment["R1_EXHIBITION_MOCK"] = "0"
    environment["R1_EXHIBITION_SESSION_MODE"] = "session_arm"
    manager = ExhibitionManager(
        ExhibitionSettings.from_environment(environment), "control"
    )

    child_environment = manager._command_environment("prepare")

    assert child_environment["ROBOT_EXHIBITION_SESSION"] == "1"
    assert child_environment["R1_EXHIBITION_FAST_PREPARE"] == "1"
    assert child_environment["SAFETY_PROFILE"] == "exhibition"


def test_static_prepare_reuses_token_bound_preflight_without_enabling_control(tmp_path):
    manager = ExhibitionManager(
        ExhibitionSettings.from_environment(_base_environment(tmp_path)), 'static')
    environment = manager._command_environment('prepare')
    assert environment['R1_EXHIBITION_FAST_PREPARE'] == '1'
    assert environment['R1_EXHIBITION_STATIC_READINESS'] == '1'
    assert environment['ROBOT_CONFIRM_STATIC_PREPARE'] == '1'
    assert environment['R1_EXHIBITION_SESSION_MODE'] == 'deadman'
    assert environment['R1_EXHIBITION_SESSION_ID'] == manager.session_id
    assert environment['R1_EXHIBITION_READINESS_ATTESTATION'] == str(
        manager.readiness_attestation_path)
    ready_environment = manager._command_environment('ready_static')
    assert ready_environment['R1_EXHIBITION_READINESS_ATTESTATION'] == str(
        manager.readiness_attestation_path)
    assert 'ROBOT_EXHIBITION_SESSION' not in environment


def test_offline_handoff_only_manages_the_exact_compatible_user_unit():
    script = (
        PROJECT_DIR / "scripts" / "r1-exhibition-offline-handoff"
    ).read_text(encoding="utf-8")
    assert "systemctl --user" in script
    assert "r1-offline-session.service" in script
    assert "--property=ExecStart" in script
    assert "r1-offline-session r1" in script
    assert "pov-status" in script
    assert "systemctl --user restart" in script
    assert "wait_for_video_port" in script
    assert "pkill" not in script
    assert "killall" not in script
    assert "pgrep" not in script


def test_static_writer_does_not_reject_the_reused_offline_bridge_port():
    script = (
        PROJECT_DIR / "scripts" / "r1-live-session"
    ).read_text(encoding="utf-8")
    port_gate = script.index("UDP port ${UDP_PORT} is already in use")
    start_bridge_gate = script.rfind(
        "[[ ${START_BRIDGE} == true ]]", 0, port_gate
    )
    assert start_bridge_gate != -1
    assert port_gate - start_bridge_gate < 400


def test_readiness_helper_checks_typed_services_topics_and_static_params():
    script = (
        PROJECT_DIR / "scripts" / "r1-exhibition-wait-ready"
    ).read_text(encoding="utf-8")
    for token in (
        "r1-exhibition-graph-probe",
        "/r1/live_writer/prepare|std_srvs/srv/Trigger",
        "/r1/safety/set_kill|std_srvs/srv/SetBool",
        "/r1/sdk/joint_states|sensor_msgs/msg/JointState",
        "/r1/sdk_transport/motors_healthy|std_msgs/msg/Bool",
        "/vr/teleop/session_armed|std_msgs/msg/Bool",
        "/vr/actions/arms_neutral|std_msgs/msg/Bool",
        "/vr/actions/emergency_stop|std_msgs/msg/Bool",
        "/vr/teleop/clear_emergency_stop|std_srvs/srv/Trigger",
        (
            "/r1_kinematics_control/debug/arm_trajectory|"
            "trajectory_msgs/msg/JointTrajectory"
        ),
        "/r1/locomotion_dry_run/debug/cmd_vel|geometry_msgs/msg/TwistStamped",
        '--parameter "${expectation}"',
        "static_prepare_mode|true",
        "enable_head|false",
        "enable_arms|false",
        "enable_locomotion|false",
        "exhibition_session_mode|false",
        "head_recenter_enabled|false",
        "--string-parameter 'transport|sdk'",
        "--string-parameter 'profile|slow-safe'",
        "--token-parameter commissioning_token",
        "--attestation-path",
        "--session-id",
        "--domain-id",
    ):
        assert token in script
    assert "R1_EXHIBITION_READY_TIMEOUT_SEC" in script
    assert "R1_EXHIBITION_READY_TIMEOUT_SEC:-90" in script
    client = (PROJECT_DIR / 'exhibition/readiness.py').read_text()
    assert 'get_service_names_and_types' in client
    assert 'get_topic_names_and_types' in client
    assert "create_client(GetParameters, '/r1_live_writer/get_parameters')" in client
    assert 'create_client(Trigger' not in client
    assert 'create_client(SetBool' not in client


def test_manager_leaves_grace_for_readiness_helper_diagnostics(tmp_path, monkeypatch):
    environment = _base_environment(tmp_path)
    environment["R1_EXHIBITION_READY_TIMEOUT_SEC"] = "17"
    settings = ExhibitionSettings.from_environment(environment)
    manager = ExhibitionManager(settings, "static")
    manager.children["static_writer"] = type(
        "RunningProcess", (), {"poll": lambda self: None}
    )()
    seen = {}

    def fake_run_checked(name, timeout):
        seen.update(name=name, timeout=timeout)
        return False

    monkeypatch.setattr(manager, "_run_checked", fake_run_checked)

    assert manager._wait_for_graph("static_writer", "ready_static") is False
    assert seen == {"name": "ready_static", "timeout": 37.0}


def test_manager_propagates_its_validated_readiness_timeout(tmp_path):
    environment = _base_environment(tmp_path)
    environment["R1_EXHIBITION_READY_TIMEOUT_SEC"] = "17"
    settings = ExhibitionSettings.from_environment(environment)
    manager = ExhibitionManager(settings, "static")

    child_environment = manager._command_environment("ready_static")

    assert settings.ready_timeout_sec == 17
    assert child_environment["R1_EXHIBITION_READY_TIMEOUT_SEC"] == "17"
    assert child_environment["R1_EXHIBITION_SESSION_ID"] == manager.session_id


def test_physical_control_defaults_to_calibration_before_session_arm(tmp_path):
    environment = _base_environment(tmp_path)
    environment["R1_EXHIBITION_MOCK"] = "0"
    settings = ExhibitionSettings.from_environment(environment)

    plan = build_plan("control", settings)
    steps = [step["name"] for step in plan["steps"]]

    assert settings.calibrate_on_start is True
    assert settings.preflight_timeout_sec == 3.0
    assert steps.index("ready_control") < steps.index("startup_calibrate")
    assert steps.index("startup_calibrate") < steps.index("arm_session")
    assert steps.index("arm_session") < steps.index("prepare")
    calibrate = next(
        step for step in plan["steps"] if step["name"] == "startup_calibrate"
    )
    assert calibrate["argv"][-1].endswith("scripts/r1-exhibition-calibrate")


def test_physical_default_prepare_timeout_allows_full_sdk_preflight(tmp_path):
    environment = _base_environment(tmp_path)
    environment["R1_EXHIBITION_MOCK"] = "0"
    environment.pop("R1_EXHIBITION_PREFLIGHT_TIMEOUT_SEC")

    settings = ExhibitionSettings.from_environment(environment)

    assert settings.preflight_timeout_sec == 180.0
