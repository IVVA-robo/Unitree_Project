"""Tests for fail-closed environment interlocks."""

from pathlib import Path

import pytest

from r1_teleop_safety.environment import ActuationPolicy, environment_flag
from r1_teleop_safety.prepare import prepare_result


def test_defaults_are_dry_run_and_not_live_authorized():
    """An empty environment must never be able to authorize hardware."""
    policy = ActuationPolicy.from_environment({})
    assert policy.dry_run
    assert not policy.live_authorized
    assert 'ROBOT_DRY_RUN=0' in policy.missing_live_interlocks()


def test_only_all_explicit_live_flags_authorize_live():
    """Every safety acknowledgement is required for a live request."""
    environment = {
        'ROBOT_DRY_RUN': '0',
        'ROBOT_ENABLE_ACTUATION': '1',
        'ROBOT_CONFIRM_OFF_CHARGER': '1',
        'ROBOT_CONFIRM_CLEAR_AREA': '1',
        'ROBOT_CONFIRM_ESTOP_READY': '1',
    }
    assert ActuationPolicy.from_environment(environment).live_authorized


def test_invalid_environment_value_is_rejected():
    """Ambiguous truthy strings must not accidentally arm anything."""
    with pytest.raises(ValueError):
        environment_flag(
            'ROBOT_ENABLE_ACTUATION',
            False,
            {'ROBOT_ENABLE_ACTUATION': 'maybe'},
        )


def test_prepare_is_dry_run_or_blocked_without_a_transport():
    """Prepare never calls a robot API in either supported safety state."""
    dry_run = ActuationPolicy.from_environment({})
    code, text = prepare_result(dry_run, 'ready')
    assert code == 0
    assert 'no SDK client' in text

    live = ActuationPolicy.from_environment({
        'ROBOT_DRY_RUN': '0',
        'ROBOT_ENABLE_ACTUATION': '1',
        'ROBOT_CONFIRM_OFF_CHARGER': '1',
        'ROBOT_CONFIRM_CLEAR_AREA': '1',
        'ROBOT_CONFIRM_ESTOP_READY': '1',
    })
    code, text = prepare_result(live, 'stand')
    assert code == 3
    assert 'no command was sent' in text


def test_safety_supervisor_launch_keeps_ros_traffic_local():
    """The standalone software interlock must not join a robot ROS graph."""
    package = Path(__file__).parents[1]
    launch_path = package / 'launch' / 'r1_safety_supervisor.launch.py'
    launch_source = launch_path.read_text()
    assert "SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')" in launch_source


def test_downstream_writer_can_only_assert_the_supervisor_kill():
    """The writer fault topic must latch true and never provide a clear path."""
    package = Path(__file__).parents[1]
    source = (package / 'r1_teleop_safety' / 'supervisor.py').read_text()
    config = (package / 'config' / 'r1_teleop_safety.yaml').read_text()
    assert '/r1/safety/kill_request' in config
    assert "self._reason = 'downstream_kill_request'" in source
    assert 'if message is None or not bool(message.data):' in source


def test_safety_kill_heartbeat_has_margin_over_writer_timeout():
    """The retained kill state must also refresh well inside the 0.5 s gate."""
    package = Path(__file__).parents[1]
    source = (package / 'r1_teleop_safety' / 'supervisor.py').read_text()
    config = (package / 'config' / 'r1_teleop_safety.yaml').read_text()
    assert "declare_parameter('publish_rate_hz', 20.0)" in source
    assert 'publish_rate_hz: 20.0' in config
