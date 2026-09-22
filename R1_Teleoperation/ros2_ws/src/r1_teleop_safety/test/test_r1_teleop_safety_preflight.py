"""Unit tests for passive teleoperation preflight helpers."""

from pathlib import Path

from rclpy.qos import ReliabilityPolicy

from r1_teleop_safety.preflight import (
    _age_text,
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
