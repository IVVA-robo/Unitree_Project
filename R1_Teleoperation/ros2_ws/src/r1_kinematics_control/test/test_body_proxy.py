import math

import numpy as np
import pytest

from r1_kinematics_control.body_proxy import (
    BodyCalibration,
    BodyProxyConfig,
    BodyProxyTransformer,
)
from r1_kinematics_control.math3d import pose_matrix


def _pose(position, yaw=0.0):
    half = 0.5 * yaw
    return pose_matrix(
        position,
        [0.0, 0.0, math.sin(half), math.cos(half)],
    )


def _proxy(follow_head_position=True, follow_head_yaw=True):
    config = BodyProxyConfig(
        body_position_tau_sec=0.0,
        body_yaw_tau_sec=0.0,
        follow_head_position=follow_head_position,
        follow_head_yaw=follow_head_yaw,
        max_input_jump_m=10.0,
        max_target_speed_mps=100.0,
    )
    proxy = BodyProxyTransformer(config)
    proxy.set_robot_geometry(
        [0.0, 0.10, 0.20],
        [0.0, -0.10, 0.20],
        _pose([0.15, 0.15, 0.0]),
        _pose([0.15, -0.15, 0.0]),
        0.40,
        0.40,
    )
    return proxy


def _neutral_poses():
    return (
        _pose([0.0, 0.0, 1.65]),
        _pose([0.45, 0.30, 1.20]),
        _pose([0.45, -0.30, 1.20]),
    )


def test_world_translation_does_not_move_avatar_targets():
    proxy = _proxy()
    head, left, right = _neutral_poses()
    proxy.calibrate(head, left, right)
    initial = proxy.update(head, left, right, 0.02)

    translation = np.eye(4)
    translation[:3, 3] = [2.0, -1.0, 0.25]
    moved = proxy.update(
        translation @ head,
        translation @ left,
        translation @ right,
        0.02,
    )
    assert moved.targets['left'] == pytest.approx(initial.targets['left'])
    assert moved.targets['right'] == pytest.approx(initial.targets['right'])


def test_neutral_position_errors_detect_stale_saved_calibration():
    proxy = _proxy()
    head, left, right = _neutral_poses()
    proxy.calibrate(head, left, right)
    result = proxy.update(head, left, right, 0.02)

    errors = proxy.neutral_position_errors(result.hands_body_local)
    assert errors['left'] == pytest.approx(0.0)
    assert errors['right'] == pytest.approx(0.0)

    moved_right = right.copy()
    moved_right[:3, 3] += [0.0, 0.0, 0.30]
    moved = proxy.update(head, left, moved_right, 0.02)
    errors = proxy.neutral_position_errors(moved.hands_body_local)
    assert errors['right'] > 0.15


def test_asymmetric_start_pose_becomes_independent_zero_for_each_hand():
    proxy = _proxy()
    head, left, right = _neutral_poses()
    left = left.copy()
    right = right.copy()
    left[:3, 3] += [-0.05, 0.03, 0.04]
    right[:3, 3] += [0.03, 0.01, -0.04]

    calibration = proxy.calibrate(head, left, right)
    result = proxy.update(head, left, right, 0.02)
    errors = proxy.neutral_position_errors(result.hands_body_local)

    assert errors['left'] == pytest.approx(0.0)
    assert errors['right'] == pytest.approx(0.0)
    assert result.targets['left'][:3, 3] == pytest.approx(
        [0.15, 0.15, 0.0]
    )
    assert result.targets['right'][:3, 3] == pytest.approx(
        [0.15, -0.15, 0.0]
    )
    mirrored_right = calibration.right_neutral_body[:3, 3].copy()
    mirrored_right[1] *= -1.0
    assert calibration.left_neutral_body[:3, 3] != pytest.approx(
        mirrored_right
    )


def test_world_yaw_rotation_does_not_move_body_local_hands():
    config = BodyProxyConfig(
        body_position_tau_sec=0.0,
        body_yaw_tau_sec=0.0,
        follow_head_position=True,
        follow_head_yaw=True,
        max_input_jump_m=10.0,
        max_target_speed_mps=100.0,
    )
    proxy = BodyProxyTransformer(config)
    proxy.set_robot_geometry(
        [0.0, 0.10, 0.20], [0.0, -0.10, 0.20],
        _pose([0.15, 0.15, 0.0]), _pose([0.15, -0.15, 0.0]),
        0.40, 0.40,
    )
    head, left, right = _neutral_poses()
    proxy.calibrate(head, left, right)
    initial = proxy.update(head, left, right, 0.02)

    world_rotation = _pose([0.0, 0.0, 0.0], yaw=1.1)
    rotated = proxy.update(
        world_rotation @ head,
        world_rotation @ left,
        world_rotation @ right,
        0.02,
    )
    assert rotated.hands_body_local['left'] == pytest.approx(
        initial.hands_body_local['left']
    )
    assert rotated.hands_body_local['right'] == pytest.approx(
        initial.hands_body_local['right']
    )
    assert rotated.targets['left'] == pytest.approx(initial.targets['left'])
    assert rotated.targets['right'] == pytest.approx(initial.targets['right'])


def test_head_only_yaw_does_not_command_either_arm_by_default():
    """Looking around must be independent from stationary hand targets."""
    proxy = _proxy(follow_head_position=False, follow_head_yaw=False)
    head, left, right = _neutral_poses()
    proxy.calibrate(head, left, right)
    initial = proxy.update(head, left, right, 0.02)

    turned_head = _pose(head[:3, 3], yaw=0.55)
    turned = proxy.update(turned_head, left, right, 0.02)

    assert turned.targets['left'] == pytest.approx(initial.targets['left'])
    assert turned.targets['right'] == pytest.approx(initial.targets['right'])
    assert turned.hands_body_local['left'] == pytest.approx(
        initial.hands_body_local['left']
    )
    assert turned.hands_body_local['right'] == pytest.approx(
        initial.hands_body_local['right']
    )


def test_mirrored_hands_produce_symmetric_robot_positions():
    proxy = _proxy()
    head, left, right = _neutral_poses()
    proxy.calibrate(head, left, right)
    left = left.copy()
    right = right.copy()
    left[:3, 3] += [0.10, 0.04, 0.06]
    right[:3, 3] += [0.10, -0.04, 0.06]
    result = proxy.update(head, left, right, 0.02)

    left_target = result.targets['left'][:3, 3]
    right_target = result.targets['right'][:3, 3]
    assert left_target[0] == pytest.approx(right_target[0])
    assert left_target[1] == pytest.approx(-right_target[1])
    assert left_target[2] == pytest.approx(right_target[2])
    assert result.target_symmetry_error == pytest.approx([0.0, 0.0, 0.0])


def test_reach_and_behind_safety_limits_are_applied():
    proxy = _proxy()
    head, left, right = _neutral_poses()
    proxy.calibrate(head, left, right)
    proxy.update(head, left, right, 0.02)
    left = left.copy()
    right = right.copy()
    left[0, 3] = -5.0
    right[0, 3] = -5.0
    result = proxy.update(head, left, right, 1.0)

    assert result.targets['left'][0, 3] >= -0.04
    assert result.targets['right'][0, 3] >= -0.04
    assert result.clamped['left']
    assert result.clamped['right']


def test_crossed_controllers_stay_on_their_own_side_of_body():
    proxy = _proxy()
    head, left, right = _neutral_poses()
    proxy.calibrate(head, left, right)

    # Deliberately swap the controller lateral positions.  The body proxy must
    # not pass a left target through the centre plane to the right arm (or vice
    # versa), even when both controllers are held forward.
    left = left.copy()
    right = right.copy()
    left[1, 3] = -0.35
    right[1, 3] = 0.35
    result = proxy.update(head, left, right, 0.02)

    minimum = proxy.config.min_hand_lateral_m
    assert result.hands_body_local['left'][1, 3] >= minimum
    assert result.hands_body_local['right'][1, 3] <= -minimum
    assert result.targets['left'][1, 3] >= minimum
    assert result.targets['right'][1, 3] <= -minimum
    assert 'side_separation' in result.clamped['left']
    assert 'side_separation' in result.clamped['right']


def test_calibration_persistence_roundtrip(tmp_path):
    proxy = _proxy()
    calibration = proxy.calibrate(*_neutral_poses())
    path = tmp_path / 'body.json'
    calibration.save(path)
    loaded = BodyCalibration.load(path)

    assert loaded.head_height_m == pytest.approx(calibration.head_height_m)
    assert loaded.waist_offset == pytest.approx(calibration.waist_offset)
    assert loaded.left_neutral_body == pytest.approx(
        calibration.left_neutral_body
    )


def test_exhibition_shoulder_tuning_is_captured_on_calibration():
    config = BodyProxyConfig(
        shoulder_height_offset_m=0.08,
        shoulder_forward_offset_m=0.03,
        shoulder_width_m=0.46,
    )
    proxy = BodyProxyTransformer(config)
    calibration = proxy.calibrate(*_neutral_poses())
    expected_height = (
        config.shoulder_height_ratio * calibration.user_height_m
        - calibration.head_height_m
        + config.shoulder_height_offset_m
    )

    assert calibration.left_shoulder == pytest.approx(
        [0.03, 0.23, expected_height]
    )
    assert calibration.right_shoulder == pytest.approx(
        [0.03, -0.23, expected_height]
    )


def test_arm_motion_scale_applies_without_recalibrating_saved_neutral():
    head, left, right = _neutral_poses()
    calibration_source = _proxy()
    calibration = calibration_source.calibrate(head, left, right)

    def mapped_forward_delta(motion_scale):
        config = BodyProxyConfig(
            motion_scale=motion_scale,
            body_position_tau_sec=0.0,
            body_yaw_tau_sec=0.0,
            max_input_jump_m=10.0,
            max_target_speed_mps=100.0,
        )
        proxy = BodyProxyTransformer(config)
        proxy.set_robot_geometry(
            [0.0, 0.10, 0.20],
            [0.0, -0.10, 0.20],
            _pose([0.15, 0.15, 0.0]),
            _pose([0.15, -0.15, 0.0]),
            0.40,
            0.40,
        )
        proxy.use_calibration(calibration)
        neutral = proxy.update(head, left, right, 0.02)
        moved_left = left.copy()
        moved_right = right.copy()
        moved_left[0, 3] += 0.03
        moved_right[0, 3] += 0.03
        moved = proxy.update(head, moved_left, moved_right, 0.02)
        return (
            moved.targets['left'][0, 3]
            - neutral.targets['left'][0, 3]
        )

    assert mapped_forward_delta(1.0) == pytest.approx(
        2.0 * mapped_forward_delta(0.5)
    )


@pytest.mark.parametrize(
    'override',
    [
        {'shoulder_height_offset_m': float('nan')},
        {'shoulder_forward_offset_m': 0.081},
        {'shoulder_width_m': 0.56},
        {'motion_scale': 1.21},
    ],
)
def test_exhibition_body_tuning_rejects_nonfinite_or_out_of_range(override):
    with pytest.raises(ValueError):
        BodyProxyTransformer(BodyProxyConfig(**override))


def test_calibration_save_preserves_compatibility_symlink(tmp_path):
    proxy = _proxy()
    calibration = proxy.calibrate(*_neutral_poses())
    canonical_path = tmp_path / 'canonical.json'
    compatibility_path = tmp_path / 'legacy.json'
    canonical_path.write_text('{}', encoding='utf-8')
    compatibility_path.symlink_to(canonical_path)

    calibration.save(compatibility_path)

    assert compatibility_path.is_symlink()
    loaded = BodyCalibration.load(canonical_path)
    assert loaded.head_height_m == pytest.approx(calibration.head_height_m)


def test_robot_neutral_targets_are_available_and_cannot_mutate_geometry():
    proxy = _proxy()
    expected = proxy.neutral_targets()
    expected['left'][0, 3] = 99.0

    fresh = proxy.neutral_targets()
    assert fresh['left'][0, 3] != 99.0
    assert fresh['left'][1, 3] == pytest.approx(-fresh['right'][1, 3])
