"""Tests for the pure, transport-free live command gate."""

import pytest

from r1_hardware_adapter.live_gate import (
    GateAction,
    LiveAuthorization,
    LiveCommandGate,
    LiveGateLimits,
)


LIVE_ENVIRONMENT = {
    'ROBOT_DRY_RUN': '0',
    'ROBOT_ENABLE_ACTUATION': '1',
    'ROBOT_CONFIRM_OFF_CHARGER': '1',
    'ROBOT_CONFIRM_CLEAR_AREA': '1',
    'ROBOT_CONFIRM_ESTOP_READY': '1',
}


def _authorized_gate():
    """Return a gate with all prerequisites except a fresh velocity packet."""
    gate = LiveCommandGate.from_environment(
        LIVE_ENVIRONMENT,
        send_commands=True,
    )
    gate.observe_kill(False)
    gate.observe_deadman(True, now=0.0)
    gate.observe_robot_state(charging=False, ready=True, now=0.0)
    return gate


def test_missing_environment_flags_never_permit_transport_or_motion():
    """Absent live acknowledgements remain dry-run and produce no send action."""
    gate = LiveCommandGate.from_environment({}, send_commands=True)
    decision = gate.evaluate(now=0.0)

    assert not gate.authorization.live_authorized
    assert decision.action is GateAction.HOLD
    assert not decision.transport_permitted
    assert not decision.motion_permitted
    assert decision.velocity == (0.0, 0.0, 0.0)
    assert 'ROBOT_DRY_RUN=0' in decision.reason


def test_send_commands_defaults_false_even_with_all_live_environment_flags():
    """A reviewed writer must opt in separately after environment validation."""
    gate = LiveCommandGate(
        authorization=LiveAuthorization.from_environment(LIVE_ENVIRONMENT)
    )
    gate.observe_kill(False)
    gate.observe_deadman(True, now=0.0)
    gate.observe_robot_state(charging=False, ready=True, now=0.0)
    decision = gate.receive_velocity(0.1, 0.0, 0.0, now=0.01)

    assert not gate.send_commands
    assert decision.action is GateAction.HOLD
    assert decision.reason == 'send_commands_disabled'
    assert not decision.should_send_velocity
    assert not decision.should_send_stop


def test_kill_requires_explicit_clear_then_latches_until_reset():
    """A true kill stops immediately and a later false never silently rearms."""
    gate = _authorized_gate()
    accepted = gate.receive_velocity(0.1, 0.0, 0.0, now=0.01)
    assert accepted.should_send_velocity

    gate.observe_kill(True)
    killed = gate.evaluate(now=0.02)
    assert killed.action is GateAction.STOP
    assert killed.should_send_stop
    assert killed.reason == 'kill_active'
    assert killed.velocity == (0.0, 0.0, 0.0)

    gate.observe_kill(False)
    latched = gate.evaluate(now=0.03)
    assert latched.action is GateAction.STOP
    assert latched.reason == 'kill_latched'
    assert gate.reset_kill().accepted

    reset = gate.evaluate(now=0.04)
    assert reset.action is GateAction.STOP
    assert reset.reason == 'command_missing'
    assert not reset.should_send_velocity


def test_deadman_false_or_stale_never_allows_motion():
    """Deadman release and heartbeat loss both return an explicit stop action."""
    gate = _authorized_gate()
    assert gate.receive_velocity(0.1, 0.0, 0.0, now=0.01).should_send_velocity

    gate.observe_deadman(False, now=0.02)
    released = gate.evaluate(now=0.03)
    assert released.action is GateAction.STOP
    assert released.reason == 'deadman_inactive'

    gate.observe_deadman(True, now=0.10)
    gate.observe_robot_state(charging=False, ready=True, now=0.10)
    assert gate.receive_velocity(0.1, 0.0, 0.0, now=0.11).should_send_velocity
    stale = gate.evaluate(now=1.61)
    assert stale.action is GateAction.STOP
    assert stale.reason == 'deadman_stale'
    assert stale.velocity == (0.0, 0.0, 0.0)


def test_command_watchdog_discards_old_velocity_and_requires_new_input():
    """A command older than its watchdog cannot resume after a later tick."""
    gate = _authorized_gate()
    assert gate.receive_velocity(0.1, 0.0, 0.0, now=0.01).should_send_velocity

    stopped = gate.evaluate(now=0.27)
    assert stopped.action is GateAction.STOP
    assert stopped.reason == 'command_stale'
    assert stopped.command_state == 'stale'
    assert stopped.velocity == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    ('charging', 'ready', 'reason'),
    (
        (True, True, 'robot_charging'),
        (None, True, 'robot_charging_unknown'),
        (False, False, 'robot_not_ready'),
    ),
)
def test_charging_unknown_charging_or_not_ready_state_never_allows_motion(
    charging, ready, reason
):
    """Motion needs a fresh state that explicitly says off-charger and ready."""
    gate = _authorized_gate()
    gate.observe_robot_state(charging=charging, ready=ready, now=0.01)
    decision = gate.receive_velocity(0.1, 0.0, 0.0, now=0.02)

    assert decision.action is GateAction.STOP
    assert decision.reason == reason
    assert not decision.motion_permitted
    assert decision.velocity == (0.0, 0.0, 0.0)


def test_missing_or_stale_robot_state_never_allows_motion():
    """No observation and an expired observation are both hard stop states."""
    gate = LiveCommandGate.from_environment(LIVE_ENVIRONMENT, send_commands=True)
    gate.observe_kill(False)
    gate.observe_deadman(True, now=0.0)
    missing = gate.receive_velocity(0.1, 0.0, 0.0, now=0.01)
    assert missing.action is GateAction.STOP
    assert missing.reason == 'robot_missing'

    gate.observe_robot_state(charging=False, ready=True, now=0.02)
    assert gate.receive_velocity(0.1, 0.0, 0.0, now=0.03).should_send_velocity
    stale = gate.evaluate(now=0.53)
    assert stale.action is GateAction.STOP
    assert stale.reason == 'robot_stale'


def test_velocity_is_clamped_to_immutable_slow_safe_limits():
    """A live-approved request cannot exceed the reviewed first-run envelope."""
    gate = _authorized_gate()
    decision = gate.receive_velocity(2.0, -2.0, 2.0, now=0.01)

    assert decision.action is GateAction.VELOCITY
    assert decision.should_send_velocity
    assert decision.reason == 'velocity_clamped'
    assert decision.velocity == pytest.approx((0.20, -0.12, 0.35))


def test_unsafe_limit_relaxation_and_invalid_environment_remain_blocked():
    """Configuration mistakes cannot expand limits or accidentally arm a writer."""
    with pytest.raises(ValueError):
        LiveCommandGate(
            authorization=LiveAuthorization.from_environment(LIVE_ENVIRONMENT),
            send_commands=True,
            limits=LiveGateLimits(max_forward_mps=0.21),
        )

    authorization = LiveAuthorization.from_environment({
        **LIVE_ENVIRONMENT,
        'ROBOT_ENABLE_ACTUATION': 'sometimes',
    })
    gate = LiveCommandGate(authorization=authorization, send_commands=True)
    decision = gate.evaluate(now=0.0)
    assert not authorization.live_authorized
    assert decision.action is GateAction.HOLD
    assert 'ROBOT_ENABLE_ACTUATION=invalid' in decision.reason
