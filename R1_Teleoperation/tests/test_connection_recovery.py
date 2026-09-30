"""Exercise recovery with mock commands; no robot or live units are touched."""

import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest


PROJECT_DIR = Path(__file__).resolve().parents[1]


@pytest.fixture
def recovery_environment(tmp_path):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    shutil.copy(PROJECT_DIR / "scripts/r1-connection-ensure", scripts)
    shutil.copy(PROJECT_DIR / "scripts/r1-network-ensure", scripts)
    (scripts / "r1-network-autodetect").write_text(
        'r1_resolve_control_ip() { printf "%s\\n" "$2"; }\n'
    )
    commands = tmp_path / "bin"
    commands.mkdir()
    dispatcher = commands / "mock-command"
    dispatcher.write_text(
        """#!/usr/bin/python3
import json, os, pathlib, sys, time
name = pathlib.Path(sys.argv[0]).name
with open(os.environ['MOCK_CALLS'], 'a') as stream:
    stream.write(json.dumps([name, *sys.argv[1:]]) + '\\n')
if name == 'curl':
    print(os.environ['MOCK_VIDEO'])
elif name == 'ping':
    sys.exit(int(os.environ['MOCK_PING_CODE']))
elif name == 'r1-video-hub-ensure':
    print(os.environ['MOCK_HUB_OUTPUT'])
    sys.exit(int(os.environ['MOCK_HUB_CODE']))
elif name == 'sleep':
    time.sleep(float(os.environ.get('MOCK_SLEEP_DURATION', '0')))
    path = pathlib.Path(os.environ['MOCK_SLEEPS'])
    count = int(path.read_text()) + 1 if path.exists() else 1
    path.write_text(str(count))
    if count >= int(os.environ['MOCK_SLEEP_LIMIT']):
        sys.exit(37)
"""
    )
    dispatcher.chmod(0o755)
    for name in ("systemctl", "curl", "ping", "sleep"):
        (commands / name).symlink_to(dispatcher)
    (scripts / "r1-video-hub-ensure").symlink_to(dispatcher)
    (scripts / "r1-unitree-sdk-env").write_text(":\n")
    sdk = tmp_path / "sdk"
    for suffix in ("", "core", "go2", "go2/robot_state"):
        package = sdk / "unitree_sdk2py" / suffix
        package.mkdir(parents=True, exist_ok=True)
        (package / "__init__.py").touch()
    (sdk / "unitree_sdk2py/core/channel.py").write_text(
        "def ChannelFactoryInitialize(domain, interface):\n"
        "    assert domain == 0\n"
        "    assert interface\n"
    )
    (sdk / "unitree_sdk2py/go2/robot_state/robot_state_client.py").write_text(
        """import json, os
from types import SimpleNamespace

class RobotStateClient:
    def SetTimeout(self, seconds):
        assert seconds == 2.0

    def Init(self):
        pass

    def ServiceList(self):
        with open(os.environ['MOCK_CALLS'], 'a') as stream:
            stream.write(json.dumps(['ServiceList']) + '\\n')
        status = os.environ.get('MOCK_HUB_STATUS', '0')
        services = [] if status == 'missing' else [SimpleNamespace(name='video_hub', status=int(status))]
        return int(os.environ['MOCK_HUB_CODE']), services

    def ServiceSwitch(self, *_args):
        with open(os.environ['MOCK_CALLS'], 'a') as stream:
            stream.write(json.dumps(['ServiceSwitch']) + '\\n')
        raise AssertionError('Recovery probes must never mutate remote services')
"""
    )
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    exhibition = runtime / "r1-exhibition"
    exhibition.mkdir()
    environment = {
        **os.environ,
        "PATH": f"{commands}:/usr/bin:/bin",
        "PYTHONPATH": str(sdk),
        "XDG_RUNTIME_DIR": str(runtime),
        "R1_EXHIBITION_RUNTIME_DIR": str(exhibition),
        "R1_CONNECTION_LOCK_FILE": str(runtime / "connection.lock"),
        "R1_CONNECTION_FAILURE_LIMIT": "1",
        "MOCK_CALLS": str(tmp_path / "calls.jsonl"),
        "MOCK_SLEEPS": str(tmp_path / "sleeps"),
        "MOCK_SLEEP_LIMIT": "3",
        "MOCK_PING_CODE": "0",
        "MOCK_HUB_CODE": "0",
        "MOCK_HUB_OUTPUT": "video_hub is available",
        "MOCK_VIDEO": json.dumps(
            {"ready": False, "source": {"has_frame": False, "stale": True}}
        ),
    }
    return scripts / "r1-connection-ensure", environment


def run_recovery(fixture, **overrides):
    script, environment = fixture
    environment = {**environment, **overrides}
    result = subprocess.run(
        [str(script)], env=environment, capture_output=True, text=True, timeout=8
    )
    path = Path(environment["MOCK_CALLS"])
    calls = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
    return result, calls


def test_duplicate_watchdog_does_not_touch_services(recovery_environment):
    _, environment = recovery_environment
    with open(environment["R1_CONNECTION_LOCK_FILE"], "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result, calls = run_recovery(recovery_environment)
    assert result.returncode == 0
    assert "watchdog_already_running" in result.stdout
    assert calls == []


def test_active_exhibition_keeps_watchdog_away_from_camera(recovery_environment):
    _, environment = recovery_environment
    lock_path = Path(environment["R1_EXHIBITION_RUNTIME_DIR"]) / "session.lock"
    with lock_path.open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result, calls = run_recovery(recovery_environment)
    assert result.returncode == 37  # Test clock stops the infinite watchdog.
    assert "exhibition_owns_camera" in result.stdout
    assert not any(call[0] == "r1-video-hub-ensure" for call in calls)
    assert not any(call[0] == "ServiceList" for call in calls)
    assert not any(call[0] == "systemctl" and call[2] in {"start", "restart"} for call in calls)


@pytest.mark.parametrize(
    "overrides",
    [
        {"MOCK_HUB_CODE": "3102"},
        {"MOCK_HUB_CODE": "3104"},
        {"MOCK_HUB_STATUS": "1"},
        {"MOCK_HUB_STATUS": "missing"},
    ],
)
def test_unavailable_robot_or_camera_does_not_restart_bridge(recovery_environment, overrides):
    result, calls = run_recovery(recovery_environment, **overrides)
    assert result.returncode == 37
    assert [call for call in calls if call[:3] == ["systemctl", "--user", "start"]]
    assert not [call for call in calls if call[:3] == ["systemctl", "--user", "restart"]]
    # Repeated UI/watchdog checks reuse the probe cooldown, not repeated RPCs.
    assert len([call for call in calls if call[0] == "ServiceList"]) == 1
    assert not any(call[0] in {"ServiceSwitch", "r1-video-hub-ensure"} for call in calls)


def test_confirmed_camera_with_stuck_client_can_restart(recovery_environment):
    result, calls = run_recovery(recovery_environment, MOCK_SLEEP_LIMIT="1")
    assert result.returncode == 37
    assert "VIDEO_RECOVERING" in result.stdout
    assert len([call for call in calls if call[0] == "ServiceList"]) == 1
    assert not any(call[0] in {"ServiceSwitch", "r1-video-hub-ensure"} for call in calls)
    assert len([call for call in calls if call[:3] == ["systemctl", "--user", "restart"]]) == 1


def test_recent_stale_frame_waits_without_claiming_ready(recovery_environment):
    result, calls = run_recovery(
        recovery_environment,
        MOCK_VIDEO=json.dumps({"ready": False, "source": {
            "has_frame": True, "stale": True, "frame_age_s": 2,
        }}),
    )
    assert result.returncode == 37
    assert "VIDEO_READY" not in result.stdout
    assert not any(call[0] == "r1-video-hub-ensure" for call in calls)
    assert not any(call[0] == "ServiceList" for call in calls)


def test_leftover_exhibition_lock_does_not_disable_recovery(recovery_environment):
    _, environment = recovery_environment
    (Path(environment["R1_EXHIBITION_RUNTIME_DIR"]) / "session.lock").touch()
    result, calls = run_recovery(recovery_environment, MOCK_SLEEP_LIMIT="1")
    assert result.returncode == 37
    assert any(call[:3] == ["systemctl", "--user", "restart"] for call in calls)


def test_dds_discovery_does_not_require_diagnostic_ip_to_answer_ping(recovery_environment):
    result, calls = run_recovery(
        recovery_environment, MOCK_PING_CODE="1", MOCK_SLEEP_LIMIT="1"
    )
    assert result.returncode == 37
    assert "VIDEO_HUB state=RUNNING read_only=true" in result.stdout
    assert any(call[:3] == ["systemctl", "--user", "restart"] for call in calls)
    assert not any(call[0] in {"ping", "ServiceSwitch", "r1-video-hub-ensure"} for call in calls)


def test_local_restarts_are_bounded_when_running_service_produces_no_frames(recovery_environment):
    result, calls = run_recovery(
        recovery_environment,
        R1_CONNECTION_PROBE_COOLDOWN_SEC="1",
        R1_CONNECTION_RESTART_LIMIT="2",
        MOCK_SLEEP_DURATION="0.4",
        MOCK_SLEEP_LIMIT="9",
    )
    assert result.returncode == 37
    assert len([call for call in calls if call[:3] == ["systemctl", "--user", "restart"]]) == 2
    assert "local_recovery_limit" in result.stdout
    assert not any(call[0] in {"ServiceSwitch", "r1-video-hub-ensure"} for call in calls)
