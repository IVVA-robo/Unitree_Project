import pytest

from r1_telepresence_sim.leg_visualizer import (
    LEG_JOINTS,
    compute_leg_targets,
    neutral_leg_targets,
)
from r1_telepresence_sim.locomotion_adapter import _clamp, _slew


def test_clamp_is_symmetric():
    assert _clamp(2.0, 0.5) == 0.5
    assert _clamp(-2.0, 0.5) == -0.5
    assert _clamp(0.2, 0.5) == 0.2


def test_slew_limits_acceleration_step():
    assert _slew(1.0, 0.0, 0.1) == 0.1
    assert _slew(-1.0, 0.0, 0.1) == -0.1
    assert _slew(0.05, 0.0, 0.1) == 0.05


def test_leg_visualizer_neutral_pose_is_zero_and_complete():
    targets = neutral_leg_targets()
    assert tuple(targets) == LEG_JOINTS
    assert all(value == 0.0 for value in targets.values())


def test_leg_visualizer_mirrors_forward_step():
    targets = compute_leg_targets(0.35, 0.0, 0.0, 0.25)
    assert targets['left_hip_pitch_joint'] == pytest.approx(
        -targets['right_hip_pitch_joint']
    )
    assert targets['left_knee_joint'] != targets['right_knee_joint']
    assert abs(targets['left_hip_pitch_joint']) <= 0.22
    assert abs(targets['right_hip_pitch_joint']) <= 0.22


def test_forward_command_swings_the_urdf_feet_forward():
    # In the checked-in URDF positive hip pitch moves the ankle backwards, so
    # a positive forward command must use the negative pitch phase.
    targets = compute_leg_targets(0.35, 0.0, 0.0, 0.5 * 3.141592653589793)
    assert targets['left_hip_pitch_joint'] < 0.0
    assert targets['right_hip_pitch_joint'] > 0.0
    assert targets['left_knee_joint'] > 0.0
    assert targets['right_knee_joint'] == 0.0


def test_turn_uses_mirrored_hip_yaw_and_keeps_outward_roll():
    targets = compute_leg_targets(0.0, 0.0, 0.60, 0.0)
    assert targets['left_hip_yaw_joint'] == pytest.approx(
        -targets['right_hip_yaw_joint']
    )
    assert targets['left_hip_roll_joint'] > 0.0
    assert targets['right_hip_roll_joint'] < 0.0


def test_leg_visualizer_lateral_and_yaw_are_bounded():
    targets = compute_leg_targets(10.0, -10.0, 10.0, 1.2)
    assert all(
        lower - 1.0e-9 <= targets[name] <= upper + 1.0e-9
        for name, (lower, upper) in {
            'left_hip_roll_joint': (-1.047, 1.745),
            'right_hip_roll_joint': (-1.745, 1.047),
            'left_ankle_roll_joint': (-0.26, 0.26),
            'right_ankle_roll_joint': (-0.26, 0.26),
        }.items()
    )


def test_leg_visualizer_nonfinite_input_is_safe():
    targets = compute_leg_targets(float('nan'), float('inf'), float('-inf'), 0.0)
    assert all(value == 0.0 for value in targets.values())
