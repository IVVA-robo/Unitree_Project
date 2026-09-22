import pytest
from std_msgs.msg import Bool

from r1_hardware_adapter.node import R1HardwareAdapter
from r1_hardware_adapter.safety import SafetyGate, SafetyLimits


def test_deadman_and_watchdog_are_fail_safe():
    gate = SafetyGate(SafetyLimits(active_timeout_sec=1.0, command_timeout_sec=0.2))
    assert not gate.velocity(1.0, 0.0, 0.0, now=0.0).accepted

    gate.set_active(True, now=0.0)
    accepted = gate.velocity(1.0, 0.0, 0.0, now=0.01)
    assert accepted.accepted
    assert accepted.linear_x == pytest.approx(0.35)
    assert not gate.watchdog_expired(now=0.1)
    assert gate.watchdog_expired(now=0.3)

    gate.set_active(False, now=0.4)
    stopped = gate.velocity(0.0, 0.0, 0.0, now=0.41)
    assert not stopped.accepted
    assert stopped.linear_x == 0.0


def test_nonfinite_velocity_is_rejected():
    gate = SafetyGate()
    gate.set_active(True, now=0.0)
    decision = gate.velocity(float('nan'), 0.0, 0.0, now=0.01)
    assert not decision.accepted
    assert decision.reason == 'nonfinite_velocity'


def test_joint_step_is_clamped_and_shape_changes_are_rejected():
    gate = SafetyGate(SafetyLimits(max_joint_step_rad=0.1))
    gate.set_active(True, now=0.0)
    first = gate.arm([0.0, 0.0], now=0.01)
    assert first.positions == pytest.approx((0.0, 0.0))

    second = gate.arm([1.0, -1.0], now=0.02)
    assert second.accepted
    assert second.reason == 'joint_step_clamped'
    assert second.positions == pytest.approx((0.1, -0.1))

    changed = gate.arm([0.0], now=0.03)
    assert not changed.accepted
    assert changed.reason == 'joint_count_changed'


def test_arm_input_validation():
    gate = SafetyGate(SafetyLimits(max_arm_joints=2))
    gate.set_active(True, now=0.0)
    assert gate.arm([], now=0.01).reason == 'empty_trajectory'
    assert gate.arm([0.0, 0.0, 0.0], now=0.02).reason == 'too_many_arm_joints'
    assert gate.arm([0.0, float('inf')], now=0.03).reason == (
        'nonfinite_joint_position'
    )


def test_repeated_deadman_heartbeat_does_not_repeat_stop(monkeypatch):
    adapter = object.__new__(R1HardwareAdapter)
    adapter._gate = SafetyGate()
    adapter._active_seen = False
    zero_reasons = []
    statuses = []
    monkeypatch.setattr(adapter, '_publish_zero_velocity', zero_reasons.append)
    monkeypatch.setattr(adapter, '_publish_status', statuses.append)

    adapter._active_callback(Bool(data=False))
    adapter._active_callback(Bool(data=False))
    assert zero_reasons == ['deadman_released']
    assert statuses == ['deadman=False']

    adapter._active_callback(Bool(data=True))
    adapter._active_callback(Bool(data=True))
    adapter._active_callback(Bool(data=False))
    assert zero_reasons == ['deadman_released', 'deadman_released']
    assert statuses == ['deadman=False', 'deadman=True', 'deadman=False']
