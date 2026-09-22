"""Headset-relative virtual-body mapping for symmetric avatar arm targets."""

from dataclasses import dataclass
import json
import math
import os
from typing import Dict, Optional

import numpy as np

from .filters import exponential_alpha


CALIBRATION_VERSION = 1


def _as_pose(value, name):
    pose = np.asarray(value, dtype=float)
    if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
        raise ValueError(f'{name} must be a finite 4x4 transform')
    return pose.copy()


def _inverse_pose(transform):
    """Invert a homogeneous rigid transform without a general matrix inverse."""
    transform = _as_pose(transform, 'transform')
    result = np.eye(4)
    rotation = transform[:3, :3]
    result[:3, :3] = rotation.T
    result[:3, 3] = -(rotation.T @ transform[:3, 3])
    return result


def _yaw_from_pose(transform, fallback=0.0):
    """Extract REP-103 yaw from the projected head-forward axis."""
    forward = np.asarray(transform, dtype=float)[:3, 0]
    if np.linalg.norm(forward[:2]) < 1.0e-6:
        return float(fallback)
    return math.atan2(float(forward[1]), float(forward[0]))


def _yaw_pose(position, yaw):
    result = np.eye(4)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    result[:3, :3] = np.array([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])
    result[:3, 3] = np.asarray(position, dtype=float)
    return result


def _wrap_angle(value):
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _symmetric_positions(left, right):
    """Return an exactly mirrored pair using both measurements equally."""
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    forward = 0.5 * (left[0] + right[0])
    lateral = 0.5 * (abs(left[1]) + abs(right[1]))
    height = 0.5 * (left[2] + right[2])
    return (
        np.array([forward, lateral, height]),
        np.array([forward, -lateral, height]),
    )


def symmetry_error(left, right):
    """Return xyz mismatch after mirroring the right position across body Y."""
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    return np.array([left[0] - right[0], left[1] + right[1], left[2] - right[2]])


@dataclass(frozen=True)
class BodyProxyConfig:
    """Tunable anthropometry, smoothing, and safety limits."""

    user_height_m: float = 0.0
    fallback_user_height_m: float = 1.75
    shoulder_width_m: float = 0.40
    user_arm_reach_m: float = 0.65
    head_height_ratio: float = 0.93
    neck_height_ratio: float = 0.86
    shoulder_height_ratio: float = 0.82
    chest_height_ratio: float = 0.72
    waist_height_ratio: float = 0.53
    shoulder_forward_offset_m: float = -0.02
    body_position_tau_sec: float = 0.12
    body_yaw_tau_sec: float = 0.20
    motion_scale: float = 1.0
    robot_reach_scale: float = 0.95
    max_behind_shoulder_m: float = 0.04
    # Keep the two wrist targets on their own side of the torso.  The VR
    # controllers can physically cross while the operator reaches forward;
    # allowing that through to IK makes the solver choose a crossed-arm pose.
    # This is a body-frame distance from the centre plane, not a total gap.
    min_hand_lateral_m: float = 0.08
    max_input_jump_m: float = 0.25
    max_target_speed_mps: float = 0.80

    def validate(self):
        """Reject unsafe or nonsensical body-proxy parameters."""
        positive = (
            self.fallback_user_height_m,
            self.shoulder_width_m,
            self.user_arm_reach_m,
            self.head_height_ratio,
            self.motion_scale,
            self.robot_reach_scale,
            self.max_input_jump_m,
            self.max_target_speed_mps,
        )
        if min(positive) <= 0.0:
            raise ValueError('body proxy dimensions, scales, and limits must be positive')
        if self.user_height_m < 0.0:
            raise ValueError('body_proxy.user_height_m must be zero or positive')
        if not 0.0 < self.robot_reach_scale <= 1.0:
            raise ValueError('body_proxy.robot_reach_scale must be in (0, 1]')
        if self.max_behind_shoulder_m < 0.0:
            raise ValueError('body_proxy.max_behind_shoulder_m must be non-negative')
        if self.min_hand_lateral_m < 0.0:
            raise ValueError('body_proxy.min_hand_lateral_m must be non-negative')
        if self.body_position_tau_sec < 0.0 or self.body_yaw_tau_sec < 0.0:
            raise ValueError('body proxy filter time constants must be non-negative')
        ratios = (
            self.neck_height_ratio,
            self.shoulder_height_ratio,
            self.chest_height_ratio,
            self.waist_height_ratio,
        )
        if any(not 0.0 < ratio < 1.0 for ratio in ratios):
            raise ValueError('body proxy height ratios must be in (0, 1)')


@dataclass
class BodyCalibration:
    """Captured neutral pose and approximate user anthropometry."""

    head_height_m: float
    user_height_m: float
    shoulder_width_m: float
    user_arm_reach_m: float
    neck_offset: np.ndarray
    chest_offset: np.ndarray
    waist_offset: np.ndarray
    left_shoulder: np.ndarray
    right_shoulder: np.ndarray
    left_neutral_body: np.ndarray
    right_neutral_body: np.ndarray

    def to_dict(self):
        """Return a JSON/YAML-compatible representation."""
        return {
            'version': CALIBRATION_VERSION,
            'head_height_m': float(self.head_height_m),
            'user_height_m': float(self.user_height_m),
            'shoulder_width_m': float(self.shoulder_width_m),
            'user_arm_reach_m': float(self.user_arm_reach_m),
            'neck_offset': self.neck_offset.tolist(),
            'chest_offset': self.chest_offset.tolist(),
            'waist_offset': self.waist_offset.tolist(),
            'left_shoulder': self.left_shoulder.tolist(),
            'right_shoulder': self.right_shoulder.tolist(),
            'left_neutral_body': self.left_neutral_body.tolist(),
            'right_neutral_body': self.right_neutral_body.tolist(),
        }

    @classmethod
    def from_dict(cls, value):
        """Validate and construct a calibration from decoded JSON."""
        if int(value.get('version', -1)) != CALIBRATION_VERSION:
            raise ValueError('unsupported body calibration version')

        def vector(name):
            result = np.asarray(value[name], dtype=float)
            if result.shape != (3,) or not np.all(np.isfinite(result)):
                raise ValueError(f'calibration {name} must contain three numbers')
            return result

        def scalar(name):
            result = float(value[name])
            if not math.isfinite(result) or result <= 0.0:
                raise ValueError(f'calibration {name} must be positive')
            return result

        return cls(
            head_height_m=scalar('head_height_m'),
            user_height_m=scalar('user_height_m'),
            shoulder_width_m=scalar('shoulder_width_m'),
            user_arm_reach_m=scalar('user_arm_reach_m'),
            neck_offset=vector('neck_offset'),
            chest_offset=vector('chest_offset'),
            waist_offset=vector('waist_offset'),
            left_shoulder=vector('left_shoulder'),
            right_shoulder=vector('right_shoulder'),
            left_neutral_body=_as_pose(
                value['left_neutral_body'], 'left_neutral_body'
            ),
            right_neutral_body=_as_pose(
                value['right_neutral_body'], 'right_neutral_body'
            ),
        )

    def save(self, path):
        """Atomically persist the calibration as JSON (also valid YAML)."""
        path = os.path.abspath(os.path.expanduser(path))
        # Preserve migration/compatibility links instead of replacing the
        # symlink itself during the final atomic os.replace().
        if os.path.islink(path):
            path = os.path.realpath(path)
        directory = os.path.dirname(path)
        os.makedirs(directory, exist_ok=True)
        temporary = f'{path}.tmp'
        with open(temporary, 'w', encoding='utf-8') as stream:
            json.dump(self.to_dict(), stream, indent=2, sort_keys=True)
            stream.write('\n')
        os.replace(temporary, path)

    @classmethod
    def load(cls, path):
        """Load a previously persisted calibration."""
        path = os.path.abspath(os.path.expanduser(path))
        with open(path, 'r', encoding='utf-8') as stream:
            return cls.from_dict(json.load(stream))


@dataclass(frozen=True)
class BodyProxyResult:
    """Mapped targets plus observable intermediate coordinate frames."""

    targets: Dict[str, np.ndarray]
    head_world: np.ndarray
    hands_world: Dict[str, np.ndarray]
    hands_head_local: Dict[str, np.ndarray]
    hands_body_local: Dict[str, np.ndarray]
    body_world: np.ndarray
    input_symmetry_error: np.ndarray
    target_symmetry_error: np.ndarray
    clamped: Dict[str, tuple]


class BodyProxyTransformer:
    """Map HMD-relative controller poses into a symmetric robot torso frame."""

    def __init__(self, config: BodyProxyConfig):
        config.validate()
        self.config = config
        self.calibration: Optional[BodyCalibration] = None
        self._robot_shoulders: Dict[str, np.ndarray] = {}
        self._robot_neutral: Dict[str, np.ndarray] = {}
        self._robot_arm_reach = 0.0
        self._filtered_head_position = None
        self._filtered_yaw = None
        self._last_body_hands: Dict[str, np.ndarray] = {}
        self._last_targets: Dict[str, np.ndarray] = {}

    @property
    def ready(self):
        """Return true when both calibration and robot geometry are available."""
        return self.calibration is not None and bool(self._robot_neutral)

    def set_robot_geometry(
        self,
        left_shoulder,
        right_shoulder,
        left_neutral,
        right_neutral,
        left_arm_reach,
        right_arm_reach,
    ):
        """Install symmetrized geometry extracted from the active robot URDF."""
        left_shoulder, right_shoulder = _symmetric_positions(
            left_shoulder, right_shoulder
        )
        self._robot_shoulders = {
            'left': left_shoulder,
            'right': right_shoulder,
        }
        left_pose = _as_pose(left_neutral, 'left robot neutral')
        right_pose = _as_pose(right_neutral, 'right robot neutral')
        left_position, right_position = _symmetric_positions(
            left_pose[:3, 3], right_pose[:3, 3]
        )
        left_pose[:3, 3] = left_position
        right_pose[:3, 3] = right_position
        self._robot_neutral = {'left': left_pose, 'right': right_pose}
        distal_reach = min(float(left_arm_reach), float(right_arm_reach))
        if not math.isfinite(distal_reach) or distal_reach <= 0.05:
            raise ValueError('robot distal arm reach is invalid')
        self._robot_arm_reach = distal_reach * self.config.robot_reach_scale
        self._last_targets.clear()

    def reset_filter(self, head_world=None):
        """Reset body tracking and discontinuity history."""
        self._filtered_head_position = None
        self._filtered_yaw = None
        self._last_body_hands.clear()
        self._last_targets.clear()
        if head_world is not None:
            head_world = _as_pose(head_world, 'head_world')
            self._filtered_head_position = head_world[:3, 3].copy()
            self._filtered_yaw = _yaw_from_pose(head_world)

    def calibrate(self, head_world, left_world, right_world):
        """Capture a neutral HMD/body pose and enforce bilateral symmetry."""
        head_world = _as_pose(head_world, 'head_world')
        hands = {
            'left': _as_pose(left_world, 'left_world'),
            'right': _as_pose(right_world, 'right_world'),
        }
        self.reset_filter(head_world)
        body_world = _yaw_pose(head_world[:3, 3], self._filtered_yaw)
        body_from_world = _inverse_pose(body_world)
        body_hands = {
            side: body_from_world @ pose for side, pose in hands.items()
        }
        left_position, right_position = _symmetric_positions(
            body_hands['left'][:3, 3], body_hands['right'][:3, 3]
        )
        body_hands['left'][:3, 3] = left_position
        body_hands['right'][:3, 3] = right_position

        head_height = float(head_world[2, 3])
        measured_height_valid = math.isfinite(head_height) and head_height > 0.5
        if self.config.user_height_m > 0.0:
            user_height = self.config.user_height_m
        elif measured_height_valid:
            user_height = head_height / self.config.head_height_ratio
        else:
            user_height = self.config.fallback_user_height_m
            head_height = user_height * self.config.head_height_ratio

        def height_offset(ratio):
            return np.array([0.0, 0.0, ratio * user_height - head_height])

        shoulder_height = (
            self.config.shoulder_height_ratio * user_height - head_height
        )
        shoulder_half_width = 0.5 * self.config.shoulder_width_m
        left_shoulder = np.array([
            self.config.shoulder_forward_offset_m,
            shoulder_half_width,
            shoulder_height,
        ])
        right_shoulder = left_shoulder.copy()
        right_shoulder[1] = -right_shoulder[1]
        self.calibration = BodyCalibration(
            head_height_m=head_height,
            user_height_m=user_height,
            shoulder_width_m=self.config.shoulder_width_m,
            user_arm_reach_m=self.config.user_arm_reach_m,
            neck_offset=height_offset(self.config.neck_height_ratio),
            chest_offset=height_offset(self.config.chest_height_ratio),
            waist_offset=height_offset(self.config.waist_height_ratio),
            left_shoulder=left_shoulder,
            right_shoulder=right_shoulder,
            left_neutral_body=body_hands['left'],
            right_neutral_body=body_hands['right'],
        )
        self._last_body_hands.clear()
        self._last_targets.clear()
        return self.calibration

    def use_calibration(self, calibration: BodyCalibration):
        """Install a loaded calibration and clear transient filter state."""
        self.calibration = calibration
        self.reset_filter()

    def update(self, head_world, left_world, right_world, dt):
        """Return symmetric, bounded robot wrist targets for one VR snapshot."""
        if not self.ready:
            raise RuntimeError('body proxy requires calibration and robot geometry')
        head_world = _as_pose(head_world, 'head_world')
        hands_world = {
            'left': _as_pose(left_world, 'left_world'),
            'right': _as_pose(right_world, 'right_world'),
        }
        head_from_world = _inverse_pose(head_world)
        hands_head = {
            side: head_from_world @ pose for side, pose in hands_world.items()
        }
        body_world = self._filtered_body_pose(head_world, dt)
        body_from_world = _inverse_pose(body_world)
        hands_body = {
            side: body_from_world @ pose for side, pose in hands_world.items()
        }

        targets = {}
        clamped = {}
        for side in ('left', 'right'):
            body_hand, reasons = self._limit_user_hand(side, hands_body[side])
            hands_body[side] = body_hand
            target, target_reasons = self._map_target(side, body_hand, dt)
            targets[side] = target
            clamped[side] = tuple(reasons + target_reasons)

        return BodyProxyResult(
            targets=targets,
            head_world=head_world,
            hands_world=hands_world,
            hands_head_local=hands_head,
            hands_body_local=hands_body,
            body_world=body_world,
            input_symmetry_error=symmetry_error(
                hands_body['left'][:3, 3], hands_body['right'][:3, 3]
            ),
            target_symmetry_error=symmetry_error(
                targets['left'][:3, 3], targets['right'][:3, 3]
            ),
            clamped=clamped,
        )

    def _filtered_body_pose(self, head_world, dt):
        position = head_world[:3, 3]
        yaw = _yaw_from_pose(
            head_world,
            0.0 if self._filtered_yaw is None else self._filtered_yaw,
        )
        if self._filtered_head_position is None:
            self._filtered_head_position = position.copy()
            self._filtered_yaw = yaw
        else:
            position_alpha = exponential_alpha(
                dt, self.config.body_position_tau_sec
            )
            yaw_alpha = exponential_alpha(dt, self.config.body_yaw_tau_sec)
            self._filtered_head_position += position_alpha * (
                position - self._filtered_head_position
            )
            self._filtered_yaw += yaw_alpha * _wrap_angle(
                yaw - self._filtered_yaw
            )
            self._filtered_yaw = _wrap_angle(self._filtered_yaw)
        return _yaw_pose(self._filtered_head_position, self._filtered_yaw)

    def _limit_user_hand(self, side, body_hand):
        result = body_hand.copy()
        position = result[:3, 3]
        reasons = []
        previous = self._last_body_hands.get(side)
        if previous is not None:
            jump = position - previous
            jump_distance = float(np.linalg.norm(jump))
            if jump_distance > self.config.max_input_jump_m:
                position = previous + jump * (
                    self.config.max_input_jump_m / jump_distance
                )
                reasons.append('input_jump')
        shoulder = (
            self.calibration.left_shoulder
            if side == 'left'
            else self.calibration.right_shoulder
        )
        shoulder_to_hand = position - shoulder
        distance = float(np.linalg.norm(shoulder_to_hand))
        if distance > self.calibration.user_arm_reach_m:
            position = shoulder + shoulder_to_hand * (
                self.calibration.user_arm_reach_m / distance
            )
            reasons.append('user_reach')
        # A controller can cross the headset/body centre line even though the
        # avatar's hands should remain side-separated.  Clamp in body space
        # before mapping so the condition is independent of world yaw.
        minimum_lateral = self.config.min_hand_lateral_m
        if side == 'left' and position[1] < minimum_lateral:
            position[1] = minimum_lateral
            reasons.append('side_separation')
        elif side == 'right' and position[1] > -minimum_lateral:
            position[1] = -minimum_lateral
            reasons.append('side_separation')
        result[:3, 3] = position
        self._last_body_hands[side] = position.copy()
        return result, reasons

    def _map_target(self, side, body_hand, dt):
        neutral_user = (
            self.calibration.left_neutral_body
            if side == 'left'
            else self.calibration.right_neutral_body
        )
        neutral_robot = self._robot_neutral[side]
        motion_scale = (
            self.config.motion_scale
            * self._robot_arm_reach
            / self.calibration.user_arm_reach_m
        )
        target = neutral_robot.copy()
        target[:3, 3] += motion_scale * (
            body_hand[:3, 3] - neutral_user[:3, 3]
        )
        relative_orientation = (
            neutral_user[:3, :3].T @ body_hand[:3, :3]
        )
        target[:3, :3] = neutral_robot[:3, :3] @ relative_orientation

        reasons = []
        shoulder = self._robot_shoulders[side]
        minimum_forward = shoulder[0] - self.config.max_behind_shoulder_m
        if target[0, 3] < minimum_forward:
            target[0, 3] = minimum_forward
            reasons.append('behind')
        # Repeat the separation after the user-to-robot scale.  A robot may
        # have narrower shoulders than the operator, so never demand more
        # than the measured shoulder offset can provide.
        minimum_lateral = min(
            self.config.min_hand_lateral_m, abs(float(shoulder[1]))
        )
        if side == 'left' and target[1, 3] < minimum_lateral:
            target[1, 3] = minimum_lateral
            reasons.append('side_separation')
        elif side == 'right' and target[1, 3] > -minimum_lateral:
            target[1, 3] = -minimum_lateral
            reasons.append('side_separation')
        shoulder_to_target = target[:3, 3] - shoulder
        distance = float(np.linalg.norm(shoulder_to_target))
        if distance > self._robot_arm_reach:
            target[:3, 3] = shoulder + shoulder_to_target * (
                self._robot_arm_reach / distance
            )
            reasons.append('robot_reach')

        # Reach limiting scales toward the shoulder and can only reduce the
        # lateral offset.  Re-apply the invariant as the final Cartesian
        # guard before the target is handed to IK.
        if side == 'left' and target[1, 3] < minimum_lateral:
            target[1, 3] = minimum_lateral
            reasons.append('side_separation')
        elif side == 'right' and target[1, 3] > -minimum_lateral:
            target[1, 3] = -minimum_lateral
            reasons.append('side_separation')

        previous = self._last_targets.get(side)
        if previous is not None:
            maximum_step = self.config.max_target_speed_mps * max(0.0, float(dt))
            step = target[:3, 3] - previous
            distance = float(np.linalg.norm(step))
            if distance > maximum_step > 0.0:
                target[:3, 3] = previous + step * (maximum_step / distance)
                reasons.append('target_speed')
        self._last_targets[side] = target[:3, 3].copy()
        return target, reasons
