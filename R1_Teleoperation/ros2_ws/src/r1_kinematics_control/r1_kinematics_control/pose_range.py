"""Recorded operator poses mapped to bounded joint-space preview endpoints.

Pure calculation only. The live node can opt into a supervised probe.
A profile describes a configured range, not mechanical maxima.
"""

import json
from pathlib import Path

import numpy as np


class ArmPoseRange:
    """Continuous interpolation within the convex hull of measured anchors."""

    def __init__(self, data, chains):
        if data.get('version') != 1 or data.get('scope') not in ('offline_preview', 'supervised_bounded'):
            raise ValueError('unsupported pose range profile')
        if set(data.get('arms', {})) != {'left', 'right'}:
            raise ValueError('both arms are required')
        self._arms = {}
        self._bounds = {}
        self._reset_targets = {}
        self._lowered_hands = bool(data.get('lowered_hands_to_rest', False))
        self.max_probe_fraction = .10
        self.max_probe_rate = .20
        bounded = data['scope'] == 'supervised_bounded'
        space = data.get('command_space', 'feedback_relative')
        if space not in ('feedback_relative', 'absolute') or (space == 'absolute' and not bounded):
            raise ValueError('absolute targets require a bounded profile')
        self.absolute_targets = space == 'absolute'
        self.rest_start_enabled = data.get('startup_pose') == 'hands_at_hips'
        if self.rest_start_enabled and not (self.absolute_targets and self._lowered_hands):
            raise ValueError('hands-at-hips start requires absolute poses and lowered-hand rest mapping')
        for side, chain in chains.items():
            arm = data['arms'][side]
            if tuple(arm['joint_names']) != chain.joint_names:
                raise ValueError('profile joint names differ from robot chain')
            basis = np.asarray(arm['input_basis_m'], dtype=float)
            neutral = np.asarray(arm['robot_neutral_rad'], dtype=float)
            endpoints = np.asarray(arm['robot_endpoints_rad'], dtype=float)
            reset = np.asarray(arm.get('robot_reset_rad', neutral), dtype=float)
            n = len(chain.joint_names)
            if basis.shape != (3, 3) or neutral.shape != (n,) or reset.shape != (n,) or endpoints.shape != (3, n):
                raise ValueError('invalid pose range dimensions')
            if not all(np.isfinite(v).all() for v in (basis, neutral, reset, endpoints)):
                raise ValueError('pose range must be finite')
            if np.linalg.cond(basis) > 30.0:
                raise ValueError('recorded poses cannot distinguish independent movements')
            for positions in (neutral, reset, *endpoints):
                if np.any(positions < chain.lower) or np.any(positions > chain.upper):
                    raise ValueError('pose range exceeds URDF joint limits')
            self._arms[side] = (np.linalg.inv(basis), neutral.copy(), endpoints.copy())
            self._reset_targets[side] = reset.copy()
            if bounded:
                offset = np.asarray(arm['writer_offset_rad'], dtype=float)
                if offset.shape != (n,) or not np.isfinite(offset).all():
                    raise ValueError('bounded probe requires finite writer offset')
                if self.absolute_targets and np.any(offset != 0):
                    raise ValueError('absolute profile must not add a writer offset')
                self._bounds[side] = (chain, offset)
                if not self.contains_target(side, reset):
                    raise ValueError('reset pose exceeds physical wrist/joint envelope')
                # Sample the entire convex range, including the neutral path.
                # This is a wrist/joint envelope check, not full collision detection.
                for i in range(11):
                    for j in range(11 - i):
                        for k in range(11 - i - j):
                            q = neutral + np.array([i, j, k]) / 10 @ (endpoints - neutral)
                            if not self.contains_target(side, q):
                                raise ValueError('bounded probe exceeds physical wrist/joint envelope')
        if bounded:
            self.max_probe_fraction = 1.0
            self.max_probe_rate = 1.0

    def contains_target(self, side, target):
        if side not in self._bounds:
            return True
        chain, offset = self._bounds[side]
        q = offset + target
        if not np.isfinite(q).all() or np.any(q < chain.lower + .08) or np.any(q > chain.upper - .08):
            return False
        wrist = chain.forward(q)[:3, 3]
        return bool(wrist[0] >= -.02 and wrist[1] * (1 if side == 'left' else -1) >= .10)

    def neutral_target(self, side):
        return self._arms[side][1].copy()

    def reset_target(self, side):
        return self._reset_targets[side].copy()

    def allows_rest_start(self, displacements):
        """Accept both fresh hands at hips only when their output is fully at rest."""
        if not self.rest_start_enabled or set(displacements) != {'left', 'right'}:
            return False
        for side, value in displacements.items():
            d = np.asarray(value, dtype=float)
            if d.shape != (3,) or not np.isfinite(d).all():
                return False
            if not (-.8 <= d[0] <= -.1 and abs(d[1]) <= .3 and -.9 <= d[2] <= -.35):
                return False
            target, _ = self.evaluate(side, d)
            if not np.allclose(target, self.reset_target(side), atol=1e-8):
                return False
        return True

    @classmethod
    def load(cls, path, chains):
        return cls(json.loads(Path(path).read_text()), chains)

    def evaluate(self, side, displacement_m):
        """Return desired joints and normalized weights; no velocity shaping.

        Displacement is relative to the operator neutral in its fixed torso
        frame. Moving beyond an anchor saturates; combinations share the range.
        Unrecorded directions outside the measured hull project onto its faces.
        Real output would additionally need feedback anchoring and rate limits.
        """
        delta = np.asarray(displacement_m, dtype=float)
        if delta.shape != (3,) or not np.isfinite(delta).all():
            raise ValueError('operator displacement must be a finite 3-vector')
        inverse, neutral, endpoints = self._arms[side]
        weights = np.maximum(inverse @ delta, 0.0)
        if self._lowered_hands:
            # Lowering controllers must not extrapolate the near-eyes anchor.
            # The recorded eye-level anchor remains fully enabled above 8 cm.
            weights[2] *= float(np.clip((delta[2] + .02) / .10, 0., 1.))
        weights /= max(1.0, float(weights.sum()))
        target = neutral + weights @ (endpoints - neutral)
        if self._lowered_hands:
            lowered = float(np.clip((-delta[2] - .04) / .31, 0., 1.))
            target = target * (1. - lowered) + self._reset_targets[side] * lowered
        return target, weights

    def probe_target(self, side, displacement_m, fraction):
        """Unbounded preview stays at 10%; a checked session profile permits 100%."""
        if not np.isfinite(fraction) or not 0.0 < fraction <= self.max_probe_fraction:
            raise ValueError('commissioning range fraction exceeds profile limit')
        target, _ = self.evaluate(side, displacement_m)
        neutral = self._arms[side][1]
        return neutral + fraction * (target - neutral)
