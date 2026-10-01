"""Pure contracts for the one-participant final prepare helper."""

import pytest

from exhibition.prepare_client import (
    _environment_authorized,
    head_center_confirmed,
    head_center_failed,
    prepare_confirmed,
    prepare_failed,
)


@pytest.mark.parametrize(
    "status,expected",
    (
        (
            "prepare_result=R1 StandUp confirmed prepare_rearm=ready",
            "ready",
        ),
        ("prepared=true prepare_rearm=ready", "ready"),
        (
            "prepare_result=R1 StandUp confirmed "
            "prepare_rearm=await_deadman_release",
            "await_deadman_release",
        ),
    ),
)
def test_structured_prepare_success_requires_expected_rearm(status, expected):
    assert prepare_confirmed(status, expected)
    other = (
        "ready" if expected == "await_deadman_release"
        else "await_deadman_release"
    )
    assert not prepare_confirmed(status, other)


@pytest.mark.parametrize(
    "status",
    (
        "fail_closed reason=prepare_fsm_timeout",
        "prepare_cancelled by kill",
        "prepare_in_progress=false prepared=false kill_clear=false",
    ),
)
def test_structured_prepare_failure_is_fail_closed(status):
    assert prepare_failed(status)


def test_head_auto_center_uses_only_explicit_success_or_failure_markers():
    assert head_center_confirmed("head_auto_center_complete=true")
    assert head_center_confirmed("head_tracking_seed_verified=true")
    assert head_center_failed("fail_closed reason=head_auto_center_timeout")
    assert not head_center_confirmed("head_auto_center_active=true")


def test_prepare_client_requires_all_live_acknowledgements(monkeypatch):
    required = {
        "R1_PREPARE_CLIENT_AUTH": "post-traffic-gate",
        "ROBOT_DRY_RUN": "0",
        "ROBOT_ENABLE_ACTUATION": "1",
        "ROBOT_CONFIRM_OFF_CHARGER": "1",
        "ROBOT_CONFIRM_CLEAR_AREA": "1",
        "ROBOT_CONFIRM_ESTOP_READY": "1",
        "ROBOT_CONFIRM_COMMISSIONING": "1",
        "ROBOT_COMMISSIONING_TOKEN": "test-token-at-least-16",
    }
    for name, value in required.items():
        monkeypatch.setenv(name, value)
    assert _environment_authorized()
    monkeypatch.setenv("ROBOT_CONFIRM_ESTOP_READY", "0")
    assert not _environment_authorized()


def test_shell_invokes_single_client_only_after_mandatory_traffic_gate():
    from pathlib import Path

    script = (
        Path(__file__).resolve().parents[1] / "scripts/r1-robot-prepare"
    ).read_text(encoding="utf-8")
    traffic = script.index('"${SCRIPT_DIR}/r1-arm-sdk-traffic-check"')
    gate_success = script.index(
        "final ArmSdk traffic gate is idle", traffic
    )
    client = script.index('python3 "${SCRIPT_DIR}/r1-robot-prepare-client"')
    assert traffic < gate_success < client
    assert "--expected-writers 1" in script
    assert "R1_PREPARE_CLIENT_AUTH=post-traffic-gate" in script
    assert "trap emergency_relock ERR" in script
    assert "/r1/safety/emergency_stop" in script
    assert "/r1/live_writer/kill" in script
