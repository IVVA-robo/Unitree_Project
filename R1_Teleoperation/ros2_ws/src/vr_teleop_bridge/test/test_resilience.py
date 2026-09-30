"""Deterministic tests for the opt-in exhibition reconnect policy."""

import pytest

from vr_teleop_bridge.protocol import Pose, TrackingAvailability
from vr_teleop_bridge.resilience import (
    DebouncedButton,
    EmergencyStopButton,
    ExhibitionSessionGate,
    PoseRecoveryFilter,
)


ALL = TrackingAvailability(left=True, right=True, head=True)


def pose(x, y=0.0, z=1.0):
    return Pose((x, y, z), (0.0, 0.0, 0.0, 1.0))


def test_session_starts_disarmed_and_requires_explicit_tracking_flags():
    gate = ExhibitionSessionGate(2.0)
    gate.observe(
        TrackingAvailability(left=True, right=True, head=True, reported=False),
        now=0.0,
        locomotion_neutral=True,
    )
    assert gate.arm(snapshot_ready=True) == (
        False, 'tracking_flags_missing_update_vr_app'
    )
    assert gate.decision(0.0, snapshot_ready=True).phase == 'disarmed'


def test_explicit_arm_survives_controller_loss_with_zero_locomotion():
    gate = ExhibitionSessionGate(2.0)
    gate.observe(ALL, now=0.0, locomotion_neutral=True)
    assert gate.arm(snapshot_ready=True) == (True, 'session_armed')

    gate.observe(
        TrackingAvailability(left=False, right=True, head=True), now=1.0,
        locomotion_neutral=False,
    )
    decision = gate.decision(1.1, snapshot_ready=True)
    assert decision.control_active is True
    assert decision.locomotion_active is False
    assert decision.hold_left is True
    assert decision.hold_right is False
    assert decision.phase == 'reconnecting'

    decision = gate.decision(3.1, snapshot_ready=True)
    assert decision.control_active is True
    assert decision.phase == 'holding'


def test_hmd_or_complete_packet_loss_holds_both_arms_without_disarming():
    gate = ExhibitionSessionGate(1.0)
    gate.observe(ALL, now=0.0, locomotion_neutral=True)
    assert gate.arm(snapshot_ready=True)[0]

    gate.observe(
        TrackingAvailability(left=True, right=True, head=False), now=0.2,
        locomotion_neutral=False,
    )
    hmd_loss = gate.decision(0.3, snapshot_ready=True)
    assert hmd_loss.hold_left and hmd_loss.hold_right
    assert not hmd_loss.locomotion_active

    gate.observe_packet_timeout(now=2.0)
    timeout = gate.decision(3.1, snapshot_ready=True)
    assert timeout.phase == 'holding'
    assert timeout.control_active
    assert gate.armed


def test_tracking_recovery_auto_resumes_without_a_second_arm_action():
    gate = ExhibitionSessionGate(2.0)
    gate.observe(ALL, now=0.0, locomotion_neutral=True)
    gate.arm(snapshot_ready=True)
    gate.observe_packet_timeout(now=1.0)
    assert not gate.decision(1.1, True).locomotion_active

    gate.observe(ALL, now=1.2, locomotion_neutral=True)
    resumed = gate.decision(1.2, snapshot_ready=True)
    assert resumed.phase == 'tracking'
    assert resumed.control_active
    assert resumed.locomotion_active


def test_disarm_is_immediate_and_does_not_auto_rearm():
    gate = ExhibitionSessionGate(2.0)
    gate.observe(ALL, now=0.0, locomotion_neutral=True)
    gate.arm(snapshot_ready=True)
    gate.disarm()
    gate.observe(ALL, now=0.1, locomotion_neutral=True)
    decision = gate.decision(0.1, snapshot_ready=True)
    assert not decision.control_active
    assert not decision.locomotion_active


def test_initial_session_arm_rejects_non_neutral_locomotion_controls():
    gate = ExhibitionSessionGate(2.0)
    gate.observe(ALL, now=0.0, locomotion_neutral=False)

    assert gate.arm(snapshot_ready=True) == (
        False, 'locomotion_controls_not_neutral'
    )
    assert not gate.armed


def test_tracking_recovery_waits_for_neutral_before_locomotion_resumes():
    gate = ExhibitionSessionGate(2.0)
    gate.observe(ALL, now=0.0, locomotion_neutral=True)
    assert gate.arm(snapshot_ready=True)[0]

    gate.observe_packet_timeout(now=0.5)
    gate.observe(ALL, now=0.6, locomotion_neutral=False)
    unsafe_recovery = gate.decision(0.6, snapshot_ready=True)
    assert unsafe_recovery.control_active
    assert not unsafe_recovery.locomotion_active
    assert unsafe_recovery.phase == 'awaiting_neutral'

    gate.observe(ALL, now=0.7, locomotion_neutral=True)
    neutral_recovery = gate.decision(0.7, snapshot_ready=True)
    assert neutral_recovery.locomotion_active
    assert neutral_recovery.phase == 'tracking'


def test_calibration_pause_keeps_session_armed_and_requires_neutral_resume():
    gate = ExhibitionSessionGate(2.0)
    gate.observe(ALL, now=0.0, locomotion_neutral=True)
    assert gate.arm(snapshot_ready=True)[0]

    assert gate.pause() == (True, 'session_paused')
    paused = gate.decision(0.1, snapshot_ready=True)
    assert gate.armed
    assert gate.paused
    assert not gate.pose_tracking.all_available
    assert gate.tracking.all_available
    assert not paused.control_active
    assert not paused.locomotion_active
    assert paused.hold_left and paused.hold_right

    gate.observe(ALL, now=0.2, locomotion_neutral=False)
    assert gate.resume(snapshot_ready=True) == (
        False, 'locomotion_controls_not_neutral'
    )
    gate.observe(ALL, now=0.3, locomotion_neutral=True)
    assert gate.resume(snapshot_ready=True) == (True, 'session_resumed')
    assert gate.decision(0.3, snapshot_ready=True).locomotion_active


@pytest.mark.parametrize('value', [0.99, 3.01, float('nan')])
def test_grace_period_has_a_bounded_exhibition_range(value):
    with pytest.raises(ValueError):
        ExhibitionSessionGate(value)


def test_pose_filter_holds_missing_hand_and_blends_reconnect():
    recovery = PoseRecoveryFilter(blend_duration_sec=0.5)
    initial = {'head': pose(0.0), 'left': pose(0.2), 'right': pose(-0.2)}
    assert recovery.update(initial, ALL, now=0.0) == initial

    missing_left = TrackingAvailability(left=False, right=True, head=True)
    moved = {'head': pose(0.8), 'left': pose(2.0), 'right': pose(-0.4)}
    held = recovery.update(moved, missing_left, now=0.1)
    assert held['left'].position[0] == pytest.approx(0.2)
    assert held['right'].position[0] == pytest.approx(-0.4)
    assert held['head'].position[0] == pytest.approx(0.0)

    reconnect_start = recovery.update(moved, ALL, now=0.2)
    assert reconnect_start['left'].position[0] == pytest.approx(0.2)
    reconnect_mid = recovery.update(moved, ALL, now=0.45)
    assert reconnect_mid['left'].position[0] == pytest.approx(1.1)
    reconnect_done = recovery.update(moved, ALL, now=0.7)
    assert reconnect_done['left'].position[0] == pytest.approx(2.0)


def test_hmd_loss_freezes_both_hands_even_if_controller_flags_stay_true():
    recovery = PoseRecoveryFilter(0.5)
    initial = {'head': pose(0.0), 'left': pose(0.2), 'right': pose(-0.2)}
    recovery.update(initial, ALL, now=0.0)
    hmd_missing = TrackingAvailability(left=True, right=True, head=False)
    moved = {'head': pose(3.0), 'left': pose(1.2), 'right': pose(-1.2)}
    held = recovery.update(moved, hmd_missing, now=0.1)
    assert held == initial


def test_left_x_debounces_hold_and_release_without_repeated_edges():
    button = DebouncedButton(0.05)
    assert button.update(True, 0.00) == (False, False)
    assert button.update(False, 0.02) == (False, False)
    assert button.update(True, 0.03) == (False, False)
    assert button.update(True, 0.08) == (True, True)
    assert button.update(True, 1.00) == (True, False)
    assert button.update(False, 1.01) == (True, False)
    assert button.update(False, 1.06) == (False, True)


def test_right_b_is_immediate_one_shot_and_requires_debounced_release():
    button = EmergencyStopButton(0.05)
    assert button.update(True, 0.00) is True
    assert button.update(True, 0.01) is False
    assert button.update(False, 0.02) is False
    assert button.update(True, 0.03) is False
    assert button.update(False, 0.10) is False
    assert button.update(False, 0.15) is False
    assert button.update(True, 0.16) is True
