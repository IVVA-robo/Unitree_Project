import math

import numpy as np
import pytest

from r1_kinematics_control.filters import PoseEMA, ScalarEMA, exponential_alpha
from r1_kinematics_control.math3d import (
    axis_angle_matrix,
    matrix_quaternion,
    pose_matrix,
    quaternion_matrix,
    quaternion_slerp,
)


def test_quaternion_roundtrip_and_shortest_slerp():
    transform = axis_angle_matrix([0, 0, 1], math.pi * 0.75)
    quaternion = matrix_quaternion(transform)
    assert quaternion_matrix(quaternion) == pytest.approx(transform)
    halfway = quaternion_slerp([0, 0, 0, 1], -quaternion, 0.5)
    expected = axis_angle_matrix([0, 0, 1], math.pi * 0.375)
    assert quaternion_matrix(halfway) == pytest.approx(expected)


def test_pose_ema_is_time_based():
    pose_filter = PoseEMA()
    start = pose_matrix([0, 0, 0], [0, 0, 0, 1])
    target = pose_matrix([1, 0, 0], [0, 0, 0, 1])
    pose_filter.update(start, 0.01, 0.1, 0.1)
    output = pose_filter.update(target, 0.1, 0.1, 0.1)
    assert output[0, 3] == pytest.approx(1.0 - math.exp(-1.0))
    assert exponential_alpha(0.1, 0.1) == pytest.approx(1.0 - math.exp(-1.0))
    assert np.isfinite(output).all()


def test_trigger_step_is_smoothed():
    trigger_filter = ScalarEMA()
    first = trigger_filter.update(1.0, 0.02, 0.12)
    second = trigger_filter.update(1.0, 0.02, 0.12)
    assert 0.0 < first < second < 1.0
