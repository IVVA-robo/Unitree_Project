import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from std_msgs.msg import Bool

from r1_hardware_adapter import head_dry_run
from r1_hardware_adapter.head_controller import HeadController, HeadLimits
from r1_hardware_adapter.head_dry_run import HeadControllerNode, HeadPoseSample


def _quaternion(yaw=0.0, pitch=0.0, roll=0.0):
    """Return an x/y/z/w quaternion for yaw, pitch, and roll radians."""
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _unfiltered_limits(**overrides):
    values = {
        'deadzone_rad': 0.0,
        'smoothing_alpha': 1.0,
        # This is the largest immutable dry-run rate ceiling.  It is high
        # enough for the unfiltered math tests without disabling validation.
        'max_yaw_rate_rad_s': 2.0,
        'max_pitch_rate_rad_s': 2.0,
        'max_roll_rate_rad_s': 2.0,
    }
    values.update(overrides)
    return HeadLimits(**values)


def test_neutral_calibration_uses_relative_orientation_and_disables_roll():
    controller = HeadController(_unfiltered_limits())
    controller.calibrate(_quaternion(yaw=0.40), now=1.0)

    decision = controller.update(
        _quaternion(yaw=0.60, pitch=0.20, roll=0.10), now=1.1
    )

    assert decision.allowed
    assert decision.relative_euler == pytest.approx((0.20, 0.20, 0.10))
    assert decision.requested_euler == pytest.approx((0.20, 0.20, 0.0))
    assert decision.command_euler == pytest.approx((0.20, 0.20, 0.0))


def test_deadzone_inversion_scale_and_clamp_are_applied_per_axis():
    controller = HeadController(_unfiltered_limits(
        yaw_scale=2.0,
        yaw_limit_rad=0.40,
        deadzone_rad=0.10,
        invert_yaw=True,
        roll_enabled=True,
        roll_limit_rad=0.05,
    ))
    controller.calibrate(_quaternion(), now=0.0)

    decision = controller.update(_quaternion(yaw=0.50, roll=0.30), now=0.1)

    assert decision.requested_euler == pytest.approx((-0.40, 0.0, 0.05))
    # The immutable 2 rad/s ceiling still applies even to this otherwise
    # unfiltered shaping test, so yaw advances by at most 0.2 rad in 0.1 s.
    assert decision.command_euler == pytest.approx((-0.20, 0.0, 0.05))


def test_rate_limit_and_smoothing_prevent_a_head_target_jump():
    controller = HeadController(HeadLimits(
        deadzone_rad=0.0,
        smoothing_alpha=1.0,
        max_yaw_rate_rad_s=1.0,
        max_pitch_rate_rad_s=1.0,
        max_roll_rate_rad_s=1.0,
    ))
    controller.calibrate(_quaternion(), now=0.0)

    first = controller.update(_quaternion(yaw=0.60), now=0.10)
    second = controller.update(_quaternion(yaw=0.60), now=0.20)

    assert first.command_euler[0] == pytest.approx(0.10)
    assert second.command_euler[0] == pytest.approx(0.20)


def test_long_pause_cannot_bypass_the_head_rate_limit():
    controller = HeadController(HeadLimits(
        deadzone_rad=0.0,
        smoothing_alpha=1.0,
        max_yaw_rate_rad_s=1.0,
        max_pitch_rate_rad_s=1.0,
        max_roll_rate_rad_s=1.0,
        max_step_dt_sec=0.10,
    ))
    controller.calibrate(_quaternion(), now=0.0)

    # A new pose after a long executor pause remains limited to one bounded
    # control interval instead of jumping directly toward the angle clamp.
    decision = controller.update(_quaternion(yaw=0.60), now=10.0)
    assert decision.command_euler[0] == pytest.approx(0.10)


@pytest.mark.parametrize('limits', [
    {'yaw_limit_rad': 0.81},
    {'max_yaw_rate_rad_s': 2.01},
    {'max_step_dt_sec': 0.11},
])
def test_head_immutable_safety_ceilings_reject_unsafe_configuration(limits):
    with pytest.raises(ValueError):
        HeadLimits(**limits).validate()


def test_uncalibrated_and_fallback_paths_are_zero_and_not_allowed():
    controller = HeadController(_unfiltered_limits())
    uncalibrated = controller.update(_quaternion(yaw=0.20), now=0.1)
    assert not uncalibrated.allowed
    assert uncalibrated.reason == 'uncalibrated'
    assert uncalibrated.command_euler == pytest.approx((0.0, 0.0, 0.0))

    controller.calibrate(_quaternion(), now=0.2)
    controller.update(_quaternion(yaw=0.30), now=0.3)
    fallback = controller.safe_fallback('pose_stale', now=1.0)
    assert not fallback.allowed
    assert fallback.reason == 'pose_stale'
    assert fallback.command_euler == pytest.approx((0.0, 0.0, 0.0))
    assert controller.command == pytest.approx((0.0, 0.0, 0.0))


@pytest.mark.parametrize('quaternion', [
    (0.0, 0.0, 0.0, 0.0),
    (float('nan'), 0.0, 0.0, 1.0),
    (0.0, 0.0, 0.0),
])
def test_invalid_quaternions_are_rejected(quaternion):
    controller = HeadController()
    with pytest.raises(ValueError):
        controller.calibrate(quaternion, now=0.0)


def test_node_starts_killed_until_kill_topic_explicitly_clears():
    node = object.__new__(HeadControllerNode)
    node._kill_seen = False
    node._kill_active = True
    node._deadman_seen = False
    node._deadman_active = False
    node._deadman_arrival = None
    node._latest_pose = None
    node._latest_pose_error = 'pose_missing'
    node._pose_timeout_sec = 0.35
    node._active_timeout_sec = 1.50

    reason, kill, deadman, watchdog = node._gate_state(10.0)
    assert (reason, kill, deadman, watchdog) == (
        'kill_unconfirmed', 'unconfirmed', 'unconfirmed', 'blocked'
    )

    node._kill_seen = True
    node._kill_active = False
    node._deadman_seen = True
    node._deadman_active = True
    node._deadman_arrival = 10.0
    node._latest_pose = HeadPoseSample(_quaternion(), 10.0)
    reason, kill, deadman, watchdog = node._gate_state(10.1)
    assert (reason, kill, deadman, watchdog) == ('', 'clear', 'active', 'fresh')


def test_pose_watchdog_marks_a_stale_head_pose_safe():
    node = object.__new__(HeadControllerNode)
    node._kill_seen = True
    node._kill_active = False
    node._deadman_seen = True
    node._deadman_active = True
    node._deadman_arrival = 10.0
    node._latest_pose = HeadPoseSample(_quaternion(), 10.0)
    node._latest_pose_error = 'pose_fresh'
    node._pose_timeout_sec = 0.35
    node._active_timeout_sec = 1.50

    reason, kill, deadman, watchdog = node._gate_state(10.36)
    assert reason == ''
    assert (kill, deadman, watchdog) == ('clear', 'active', 'stale')
    assert node._fresh_pose(10.36) is None
    assert node._latest_pose_error == 'pose_stale'


def test_debug_trajectory_maps_only_enabled_head_axes():
    node = object.__new__(HeadControllerNode)
    node._limits = SimpleNamespace(
        yaw_enabled=True, pitch_enabled=True, roll_enabled=False
    )
    node._yaw_joint_name = 'head_yaw_joint'
    node._pitch_joint_name = 'head_pitch_joint'
    node._roll_joint_name = 'head_roll_joint'

    names, positions = node._trajectory_values((0.1, -0.2, 0.3))
    assert names == ['head_yaw_joint', 'head_pitch_joint']
    assert positions == pytest.approx([0.1, -0.2])


def test_kill_and_deadman_heartbeats_only_force_status_on_transitions(monkeypatch):
    node = object.__new__(HeadControllerNode)
    node._kill_seen = False
    node._kill_active = True
    node._deadman_seen = False
    node._deadman_active = False
    calls = []
    monkeypatch.setattr(head_dry_run.time, 'monotonic', lambda: 12.0)
    node._evaluate_and_publish = lambda now, force_status=False: calls.append(
        (now, force_status)
    )

    node._kill_callback(Bool(data=False))
    node._kill_callback(Bool(data=False))
    node._kill_callback(Bool(data=True))
    node._deadman_callback(Bool(data=True))
    node._deadman_callback(Bool(data=True))
    node._deadman_callback(Bool(data=False))

    assert [force for _, force in calls] == [True, False, True, True, False, True]


def test_head_dry_run_config_stays_locked_and_has_no_sdk_writer():
    package = Path(__file__).parents[1]
    config = (package / 'config' / 'r1_head_dry_run.yaml').read_text()
    node_source = (package / 'r1_hardware_adapter' / 'head_dry_run.py').read_text()
    assert 'hardware_enabled: false' in config
    assert 'dry_run: true' in config
    assert 'emergency_stop_topic: "/r1/safety/kill"' in config
    assert 'unitree_sdk2py' not in node_source
    assert 'ChannelPublisher' not in node_source
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in node_source


def test_head_only_launch_keeps_ros_traffic_local():
    """A standalone debug launch must not discover a robot ROS graph."""
    package = Path(__file__).parents[1]
    launch_source = (package / 'launch' / 'r1_head_dry_run.launch.py').read_text()
    assert "SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')" in launch_source
