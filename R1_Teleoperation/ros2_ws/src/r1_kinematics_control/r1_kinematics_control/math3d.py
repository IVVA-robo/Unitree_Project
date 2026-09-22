"""Small, dependency-light rigid-body math helpers."""

import math

import numpy as np


def normalize(vector, fallback=None):
    """Return a normalized vector or a provided fallback for a tiny norm."""
    value = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(value)
    if norm < 1.0e-12:
        if fallback is None:
            raise ValueError('cannot normalize a zero vector')
        return np.asarray(fallback, dtype=float)
    return value / norm


def rpy_matrix(roll, pitch, yaw):
    """Build a homogeneous fixed-axis roll/pitch/yaw transform."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    result = np.eye(4)
    result[:3, :3] = rotation
    return result


def translation_matrix(xyz):
    """Build a homogeneous translation transform."""
    result = np.eye(4)
    result[:3, 3] = np.asarray(xyz, dtype=float)
    return result


def origin_matrix(xyz, rpy):
    """Build a URDF origin transform."""
    result = rpy_matrix(*rpy)
    result[:3, 3] = np.asarray(xyz, dtype=float)
    return result


def axis_angle_matrix(axis, angle):
    """Build a homogeneous right-handed axis-angle rotation."""
    x, y, z = normalize(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c
    rotation = np.array([
        [c + x * x * one_minus_c,
         x * y * one_minus_c - z * s,
         x * z * one_minus_c + y * s],
        [y * x * one_minus_c + z * s,
         c + y * y * one_minus_c,
         y * z * one_minus_c - x * s],
        [z * x * one_minus_c - y * s,
         z * y * one_minus_c + x * s,
         c + z * z * one_minus_c],
    ])
    result = np.eye(4)
    result[:3, :3] = rotation
    return result


def normalize_quaternion(quaternion):
    """Normalize an xyzw quaternion and choose a deterministic hemisphere."""
    value = normalize(quaternion)
    if value[3] < 0.0:
        value = -value
    return value


def quaternion_matrix(quaternion):
    """Convert an xyzw quaternion to a homogeneous rotation transform."""
    x, y, z, w = normalize_quaternion(quaternion)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ])
    return result


def matrix_quaternion(transform):
    """Convert a homogeneous rotation transform to an xyzw quaternion."""
    rotation = np.asarray(transform, dtype=float)[:3, :3]
    trace = np.trace(rotation)
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(
                max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            ) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(
                max(0.0, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            ) * 2.0
            quaternion = np.array([
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(
                max(0.0, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            ) * 2.0
            quaternion = np.array([
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ])
    return normalize_quaternion(quaternion)


def pose_matrix(position, quaternion):
    """Build a homogeneous pose from xyz and xyzw values."""
    result = quaternion_matrix(quaternion)
    result[:3, 3] = np.asarray(position, dtype=float)
    return result


def orientation_error(current_rotation, target_rotation):
    """Return the world-frame small-angle error used by differential IK."""
    current = np.asarray(current_rotation, dtype=float)
    target = np.asarray(target_rotation, dtype=float)
    return 0.5 * sum(
        np.cross(current[:, index], target[:, index]) for index in range(3)
    )


def quaternion_slerp(start, target, fraction):
    """Spherically interpolate xyzw quaternions along the shortest arc."""
    first = normalize_quaternion(start)
    second = normalize_quaternion(target)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    fraction = min(1.0, max(0.0, float(fraction)))
    if dot > 0.9995:
        return normalize_quaternion(first + fraction * (second - first))
    angle = math.acos(dot)
    sin_angle = math.sin(angle)
    return normalize_quaternion(
        math.sin((1.0 - fraction) * angle) / sin_angle * first
        + math.sin(fraction * angle) / sin_angle * second
    )
