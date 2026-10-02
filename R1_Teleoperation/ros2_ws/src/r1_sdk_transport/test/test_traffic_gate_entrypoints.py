"""Static fail-closed contracts for every physical ArmSdk entry point."""

from pathlib import Path


PROJECT = Path(__file__).resolve().parents[4]
SCRIPTS = PROJECT / 'scripts'


def text(name: str) -> str:
    return (SCRIPTS / name).read_text()


def assert_exact_zero_gate(
        script: str, *, before: str, expected_writers: int,
        duration: str = '6.0') -> None:
    """Require an observer result check before the named physical boundary."""
    gate = script.rindex('"${SCRIPT_DIR}/r1-arm-sdk-traffic-check"', 0,
                         script.index(before))
    capture = script.index('arm_sdk_final_gate_status=$?', gate)
    exact_zero = script.index('((arm_sdk_final_gate_status == 0))', capture)
    failure = script.index(
        'final ArmSdk traffic gate failed closed', exact_zero)
    boundary = script.index(before)
    assert gate < capture < exact_zero < failure < boundary
    invocation = script[gate:capture]
    assert f'--duration-sec {duration}' in invocation
    assert '--fresh-sec 0.5' in invocation
    assert '--action-fresh-sec 3.5' in invocation
    assert '--min-samples 3' in invocation
    assert f'--expected-writers {expected_writers}' in invocation


def test_live_session_runs_mandatory_preflight_before_writer_launch():
    script = text('r1-live-session')
    preflight = script.index('"${SCRIPT_DIR}/r1-sdk-preflight"')
    launch = script.index('exec ros2 launch r1_teleoperation_app')
    assert preflight < launch


def test_prepare_rechecks_traffic_immediately_before_kill_release():
    script = text('r1-robot-prepare')
    initial_preflight = script.index('"${SCRIPT_DIR}/r1-sdk-preflight"')
    final_gate = script.index('"${SCRIPT_DIR}/r1-arm-sdk-traffic-check"')
    deadman = script.rindex("grep -Eq 'data:[[:space:]]*true'", 0, final_gate)
    release = script.index('release="$(timeout 8s ros2 service call')
    default_duration = script.index('arm_sdk_gate_duration=6.0', deadman)
    fast_duration = script.index(
        '[[ ${FAST_PREPARE} == true ]] && arm_sdk_gate_duration=2.25',
        default_duration,
    )
    assert (
        initial_preflight < deadman < default_duration < fast_duration
        < final_gate < release
    )
    assert_exact_zero_gate(
        script,
        before='release="$(timeout 8s ros2 service call',
        expected_writers=1,
        duration='"${arm_sdk_gate_duration}"',
    )


def test_probe_rechecks_traffic_and_relocks_on_every_gate_failure():
    script = text('r1-robot-head-ownership-probe')
    trap = script.index('trap abort_probe_if_needed EXIT')
    unsafe_boundary = script.index('PROBE_REQUESTED=true', trap)
    gate = script.index('"${SCRIPT_DIR}/r1-arm-sdk-traffic-check"', unsafe_boundary)
    service_request = script.index('future = node.client.call_async')
    assert trap < unsafe_boundary < gate < service_request
    assert_exact_zero_gate(
        script,
        before='future = node.client.call_async',
        expected_writers=2,
    )

    cleanup = script[script.index('abort_probe_if_needed()'):trap]
    assert '/r1/live_writer/kill' in cleanup
    assert '/r1/safety/emergency_stop' in cleanup


def test_preflight_accepts_only_observer_exit_zero():
    script = text('r1-sdk-preflight')
    gate = script.index('"$SCRIPT_DIR/r1-arm-sdk-traffic-check"')
    result_case = script.index('case "$arm_sdk_observer_status" in', gate)
    accepted = script.index('0)', result_case)
    active = script.index('10|11)', accepted)
    action = script.index('12|13|14)', active)
    fallback = script.index('*)', action)
    assert result_case < accepted < active < action < fallback
    assert 'mark_ok' in script[accepted:active]
    assert 'mark_fail' in script[active:fallback]
    assert 'mark_fail' in script[fallback:script.index('esac', fallback)]
    invocation = script[gate:result_case]
    assert '--action-fresh-sec 3.5' in invocation
    assert '--expected-writers 1' in invocation
