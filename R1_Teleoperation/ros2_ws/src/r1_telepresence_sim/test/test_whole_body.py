"""Pure-function safety and coordination tests for the simulation planner."""

import math

import pytest

from r1_telepresence_sim.whole_body_planner import (
    ARM_JOINTS,
    ARM_LIMITS,
    LEG_JOINTS,
    WAIST_LIMITS,
    apply_counter_swing,
    compute_waist_targets,
    neutral_arm_targets,
    rate_limit_targets,
)


def test_neutral_arm_pose_is_complete():
    targets = neutral_arm_targets()
    assert tuple(targets) == ARM_JOINTS
    assert all(value == 0.0 for value in targets.values())


def test_counter_swing_is_bilateral_and_bounded():
    neutral = neutral_arm_targets()
    targets = apply_counter_swing(neutral, math.pi / 2.0, 1.0)
    assert targets['left_shoulder_pitch_joint'] == pytest.approx(
        -targets['right_shoulder_pitch_joint']
    )
    assert abs(targets['left_shoulder_pitch_joint']) <= 0.08
    assert all(
        lower <= targets[name] <= upper
        for name, (lower, upper) in ARM_LIMITS.items()
    )


def test_waist_compensation_is_small_and_limited():
    targets = compute_waist_targets(100.0, -100.0)
    assert targets['waist_roll_joint'] == pytest.approx(-0.08)
    assert targets['waist_yaw_joint'] == pytest.approx(0.12)
    assert all(
        lower <= targets[name] <= upper
        for name, (lower, upper) in WAIST_LIMITS.items()
    )


def test_rate_limit_rejects_nonfinite_and_limits_velocity():
    previous = {name: 0.0 for name in (*LEG_JOINTS, *ARM_JOINTS, *WAIST_LIMITS)}
    velocity = dict(previous)
    targets, next_velocity = rate_limit_targets(
        {'left_knee_joint': float('nan'), 'waist_yaw_joint': 2.0},
        previous,
        velocity,
        0.02,
        limits={
            'left_knee_joint': (-0.174, 2.426),
            **ARM_LIMITS,
            **WAIST_LIMITS,
        },
        max_velocity=1.0,
        max_acceleration=2.0,
    )
    assert math.isfinite(targets['left_knee_joint'])
    assert targets['waist_yaw_joint'] <= 0.04 + 1.0e-9
    assert abs(next_velocity['waist_yaw_joint']) <= 1.0 + 1.0e-9
