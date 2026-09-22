"""Time-based filters for VR poses and finger targets."""

import math

import numpy as np

from .math3d import matrix_quaternion, pose_matrix, quaternion_slerp


def exponential_alpha(dt, time_constant):
    """Return a sample-rate-independent EMA coefficient."""
    if time_constant <= 0.0:
        return 1.0
    return 1.0 - math.exp(-max(0.0, dt) / time_constant)


class PoseEMA:
    """EMA for translation plus shortest-path quaternion SLERP for rotation."""

    def __init__(self):
        self._position = None
        self._orientation = None

    def reset(self):
        """Forget filter history."""
        self._position = None
        self._orientation = None

    def update(self, target, dt, position_tau, orientation_tau):
        """Filter a homogeneous target transform."""
        position = np.asarray(target, dtype=float)[:3, 3]
        orientation = matrix_quaternion(target)
        if self._position is None:
            self._position = position.copy()
            self._orientation = orientation.copy()
        else:
            position_alpha = exponential_alpha(dt, position_tau)
            rotation_alpha = exponential_alpha(dt, orientation_tau)
            self._position += position_alpha * (position - self._position)
            self._orientation = quaternion_slerp(
                self._orientation, orientation, rotation_alpha
            )
        return pose_matrix(self._position, self._orientation)


class ScalarEMA:
    """EMA for a scalar such as normalized trigger pressure."""

    def __init__(self, initial=0.0):
        self.value = float(initial)

    def update(self, target, dt, time_constant):
        """Advance the scalar filter."""
        alpha = exponential_alpha(dt, time_constant)
        self.value += alpha * (float(target) - self.value)
        return self.value
