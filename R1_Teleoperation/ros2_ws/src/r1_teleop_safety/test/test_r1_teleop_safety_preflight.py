"""Unit tests for passive teleoperation preflight helpers."""

from pathlib import Path

from rclpy.qos import ReliabilityPolicy

from r1_teleop_safety.preflight import (
    _age_text,
    _evaluate_head_seed,
    _latency_text,
    _rate_text,
    vr_pose_qos,
)


def test_vr_pose_subscription_matches_best_effort_sensor_publisher():
    """HMD preflight must receive the bridge's best-effort pose stream."""
    assert vr_pose_qos().reliability == ReliabilityPolicy.BEST_EFFORT


def test_rate_summary_handles_empty_single_and_regular_samples():
    """Preflight diagnostics report useful stream summaries safely."""
    assert _rate_text([]) == 'no samples'
    assert _rate_text([1.0]) == 'one sample'
    assert _rate_text([1.0, 1.1, 1.2]) == '3 samples, 10.0 Hz median'


def test_latency_summary_distinguishes_unavailable_and_local_samples():
    """Preflight labels local ROS latency, not end-to-end timing."""
    assert 'unavailable' in _latency_text([])
    assert _latency_text([2.0, 4.0, 6.0]) == (
        '4.0 ms median, 6.0 ms maximum (3 samples)'
    )


def test_age_summary_distinguishes_missing_and_fresh_samples():
    assert _age_text(None, 10.0) == 'unavailable'
    assert _age_text(9.75, 10.0) == '0.250 s ago'


def test_preflight_supports_an_explicit_clear_kill_requirement():
    """The prepare wrapper can reject an asserted software interlock."""
    source = Path(__file__).parents[1] / 'r1_teleop_safety' / 'preflight.py'
    text = source.read_text()
    assert "'--require-kill-clear'" in text
    assert 'observer.last_kill is not False' in text
    assert "'--require-debug-controllers'" in text
    assert 'last_head_debug_status' in text
    assert "'--require-prepare-signals'" in text
    assert '/r1_kinematics_control/debug/arm_trajectory' in text
    assert '/r1/locomotion_dry_run/debug/cmd_vel' in text
    assert '/r1/sdk/joint_states' in text


def test_head_seed_policy_accepts_stable_normal_tracking_samples():
    samples = [(0.10, -0.05, 0.0, 0.0)] * 5

    okay, message = _evaluate_head_seed(samples, 'normal')

    assert okay
    assert '[OK]' in message
    assert 'normal tracking' in message


def test_head_seed_policy_preserves_auto_center_and_probe_envelopes():
    samples = [(0.60, 0.30, 0.0, 0.0)] * 5

    auto_okay, auto_message = _evaluate_head_seed(samples, 'auto-center')
    normal_okay, normal_message = _evaluate_head_seed(samples, 'normal')
    probe_okay, probe_message = _evaluate_head_seed(samples, 'probe')

    assert auto_okay and '[AUTO_CENTER]' in auto_message
    assert not normal_okay and 'outside the accepted seed envelope' in normal_message
    assert probe_okay and 'ownership micro-probe' in probe_message


def test_head_seed_policy_rejects_missing_moving_or_unstable_feedback():
    missing_okay, _ = _evaluate_head_seed([(0.0, 0.0, 0.0, 0.0)] * 4, 'normal')
    moving_okay, moving_message = _evaluate_head_seed(
        [(0.0, 0.0, 0.06, 0.0)] * 5,
        'normal',
    )
    unstable_okay, unstable_message = _evaluate_head_seed(
        [
            (0.00, 0.0, 0.0, 0.0),
            (0.02, 0.0, 0.0, 0.0),
            (0.01, 0.0, 0.0, 0.0),
            (0.02, 0.0, 0.0, 0.0),
            (0.00, 0.0, 0.0, 0.0),
        ],
        'normal',
    )

    assert not missing_okay
    assert not moving_okay and 'not stationary enough' in moving_message
    assert not unstable_okay and 'position changed' in unstable_message
