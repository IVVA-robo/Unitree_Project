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


def _proxy():
    config = BodyProxyConfig(
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


def test_world_yaw_rotation_does_not_move_body_local_hands():
    proxy = _proxy()
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
