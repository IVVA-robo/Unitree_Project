"""Unit tests for the transport-free locomotion safety state machine."""

import pytest
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from r1_hardware_adapter.locomotion import (
    DryRunLocomotionController,
    LocomotionProfile,
    LocomotionSafetyConfig,
    format_status,
    require_dry_run,
)
from r1_hardware_adapter.locomotion_dry_run import KILL_QOS


def _armed_controller():
    controller = DryRunLocomotionController()
    assert controller.is_killed()
    controller.set_emergency_stop(False)
    controller.set_deadman(True, now=0.0)
    assert not controller.is_killed()
    assert controller.deadman_is_active(now=0.01)
    return controller


def test_kill_qos_is_reliable_and_transient_local():
    assert KILL_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert KILL_QOS.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_default_deadman_timeout_allows_the_bridge_one_hz_heartbeat():
    assert LocomotionSafetyConfig().deadman_timeout_sec >= 1.50


def test_starts_killed_until_an_explicit_clear_signal_arrives():
    controller = DryRunLocomotionController()
    rejected = controller.receive_velocity(0.1, 0.0, 0.0, now=0.0)
    assert not rejected.accepted
    assert rejected.reason == 'emergency_stop_awaiting_explicit_clear'

    controller.set_emergency_stop(False)
    controller.set_deadman(True, now=0.01)
    accepted = controller.receive_velocity(0.1, 0.0, 0.0, now=0.02)
    assert accepted.accepted
    assert accepted.target == pytest.approx((0.1, 0.0, 0.0))


def test_retained_startup_kill_does_not_latch_but_later_kill_does():
    """A supervisor's startup true is distinct from a later E-stop edge."""
    controller = DryRunLocomotionController()

    # The transient-local startup sample must keep the controller fail-closed,
    # but the first explicit false should be sufficient to start dry-run input.
    controller.set_emergency_stop(True)
    assert controller.kill_state == 'awaiting_explicit_clear'
    controller.set_emergency_stop(False)
    assert controller.kill_state == 'clear'
    controller.set_deadman(True, now=0.0)
    assert controller.receive_velocity(0.1, 0.0, 0.0, now=0.01).accepted

    # Once running, a true is an E-stop edge and requires the separate reset.
    controller.set_emergency_stop(True)
    assert controller.kill_state == 'active'
    controller.set_emergency_stop(False)
    assert controller.kill_state == 'latched'
    assert controller.reset_emergency_stop() == (True, 'kill_reset')
    assert controller.kill_state == 'clear'


def test_profiles_clamp_each_velocity_axis_and_can_switch_modes():
    controller = _armed_controller()
    slow = controller.receive_velocity(2.0, -2.0, 2.0, now=0.01)
    assert slow.accepted
    assert slow.reason == 'velocity_clamped'
    assert slow.target == pytest.approx((0.20, -0.12, 0.35))

    assert controller.set_mode('normal')
    normal = controller.receive_velocity(2.0, -2.0, 2.0, now=0.02)
    assert normal.target == pytest.approx((0.40, -0.22, 0.65))
    assert not controller.set_mode('unsafe')
    assert controller.mode == 'normal'


def test_lower_profile_clamps_an_already_ramped_output_immediately():
    config = LocomotionSafetyConfig(
        deadman_timeout_sec=2.0,
        command_timeout_sec=0.50,
    )
    controller = DryRunLocomotionController(config=config, mode='normal')
    controller.set_emergency_stop(False)
    controller.set_deadman(True, now=0.0)
    controller.receive_velocity(0.40, 0.0, 0.0, now=0.0)
    controller.step(now=0.0)
    for sample in range(1, 11):
        now = sample * 0.10
        # Keep the input watchdog fresh while the output ramps.  The tested
        # safety ceiling intentionally forbids an arbitrarily long timeout.
        controller.receive_velocity(0.40, 0.0, 0.0, now=now)
        controller.step(now=now)
    assert controller.output[0] > 0.20

    assert controller.set_mode('slow-safe')
    assert controller.output[0] == pytest.approx(0.20)


def test_acceleration_and_normal_stop_are_rate_limited():
    controller = _armed_controller()
    controller.receive_velocity(0.20, 0.0, 0.35, now=0.0)
    controller.step(now=0.0)
    output = controller.step(now=0.10)
    assert output.velocity == pytest.approx((0.025, 0.0, 0.05))

    controller.receive_velocity(0.0, 0.0, 0.0, now=0.10)
    stopped = controller.step(now=0.15)
    assert stopped.velocity[0] == pytest.approx(0.0025)
    assert stopped.velocity[2] == pytest.approx(0.005)


def test_deadman_and_command_watchdogs_zero_output_immediately():
    config = LocomotionSafetyConfig(
        deadman_timeout_sec=0.20,
        command_timeout_sec=0.10,
        max_step_dt_sec=0.10,
    )
    controller = DryRunLocomotionController(config=config)
    controller.set_emergency_stop(False)
    controller.set_deadman(True, now=0.0)
    controller.receive_velocity(0.2, 0.0, 0.0, now=0.0)
    controller.step(now=0.0)
    moving = controller.step(now=0.10)
    assert moving.velocity[0] == pytest.approx(0.025)

    # No fresh velocity by 0.11: the command watchdog is a hard zero edge.
    stale = controller.step(now=0.11)
    assert stale.watchdog == 'expired'
    assert stale.reason == 'command_watchdog_expired'
    assert stale.velocity == (0.0, 0.0, 0.0)

    controller.set_deadman(False, now=0.12)
    released = controller.step(now=0.13)
    assert released.deadman == 'released'
    assert released.reason == 'deadman_released'
    assert released.velocity == (0.0, 0.0, 0.0)


def test_deadman_false_zeros_output_before_the_next_timer_tick():
    """The ROS callback itself must provide a zero safety command edge."""
    controller = _armed_controller()
    controller.receive_velocity(0.20, 0.0, 0.0, now=0.0)
    controller.step(now=0.0)
    assert controller.step(now=0.10).velocity[0] > 0.0

    controller.set_deadman(False, now=0.11)
    assert controller.output == (0.0, 0.0, 0.0)


def test_stale_deadman_blocks_motion_even_if_the_last_twist_was_fresh():
    config = LocomotionSafetyConfig(
        deadman_timeout_sec=0.10,
        command_timeout_sec=0.50,
    )
    controller = DryRunLocomotionController(config=config)
    controller.set_emergency_stop(False)
    controller.set_deadman(True, now=0.0)
    controller.receive_velocity(0.20, 0.0, 0.0, now=0.0)
    controller.step(now=0.0)
    controller.step(now=0.05)

    stopped = controller.step(now=0.11)
    assert stopped.deadman == 'stale'
    assert stopped.watchdog == 'blocked_by_deadman'
    assert stopped.reason == 'deadman_stale'
    assert stopped.velocity == (0.0, 0.0, 0.0)


def test_estop_is_immediate_and_requires_reset_after_a_true_edge():
    controller = _armed_controller()
    controller.receive_velocity(0.20, 0.0, 0.0, now=0.0)
    controller.step(now=0.0)
    assert controller.step(now=0.10).velocity[0] > 0.0

    controller.set_emergency_stop(True)
    killed = controller.step(now=0.11)
    assert killed.kill == 'active'
    assert killed.velocity == (0.0, 0.0, 0.0)

    controller.set_emergency_stop(False)
    assert controller.kill_state == 'latched'
    assert not controller.receive_velocity(0.1, 0.0, 0.0, now=0.12).accepted
    assert controller.reset_emergency_stop() == (True, 'kill_reset')
    assert controller.kill_state == 'clear'


def test_invalid_input_and_non_dry_run_configuration_fail_closed():
    controller = _armed_controller()
    controller.receive_velocity(0.20, 0.0, 0.0, now=0.0)
    controller.step(now=0.0)
    assert controller.step(now=0.10).velocity[0] > 0.0
    rejected = controller.receive_velocity(float('nan'), 0.0, 0.0, now=0.01)
    assert not rejected.accepted
    assert rejected.reason == 'nonfinite_velocity'
    assert controller.output == (0.0, 0.0, 0.0)

    with pytest.raises(RuntimeError):
        require_dry_run(hardware_enabled=True, dry_run=True)
    with pytest.raises(RuntimeError):
        require_dry_run(hardware_enabled=False, dry_run=False)
    require_dry_run(hardware_enabled=False, dry_run=True)


def test_status_exposes_kill_deadman_and_watchdog_fields():
    controller = DryRunLocomotionController()
    status = format_status(controller.step(now=0.0))
    assert 'dry_run=true hardware_enabled=false' in status
    assert 'kill=awaiting_explicit_clear' in status
    assert 'deadman=missing' in status
    assert 'watchdog=blocked_by_kill' in status


def test_rejects_profile_with_non_positive_safety_values():
    profile = LocomotionProfile(
        name='bad',
        max_forward_mps=0.2,
        max_lateral_mps=0.1,
        max_yaw_rps=0.3,
        acceleration_mps2=0.0,
        deceleration_mps2=0.5,
        yaw_acceleration_rps2=0.5,
        yaw_deceleration_rps2=0.5,
    )
    with pytest.raises(ValueError):
        DryRunLocomotionController(profiles={'bad': profile}, mode='bad')


def test_immutable_ceilings_reject_unsafe_profile_or_watchdog_values():
    profile = LocomotionProfile(
        name='too_fast',
        max_forward_mps=0.56,
        max_lateral_mps=0.10,
        max_yaw_rps=0.30,
        acceleration_mps2=0.25,
        deceleration_mps2=0.45,
        yaw_acceleration_rps2=0.50,
        yaw_deceleration_rps2=0.90,
    )
    with pytest.raises(ValueError):
        DryRunLocomotionController(profiles={'too_fast': profile}, mode='too_fast')
    with pytest.raises(ValueError):
        DryRunLocomotionController(
            config=LocomotionSafetyConfig(command_timeout_sec=0.51)
        )
