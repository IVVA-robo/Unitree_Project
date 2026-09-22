"""URDF serial-chain extraction and damped-least-squares inverse kinematics."""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence
import xml.etree.ElementTree as ET

import numpy as np

from .math3d import (
    axis_angle_matrix,
    normalize,
    orientation_error,
    origin_matrix,
    translation_matrix,
)


ACTIVE_TYPES = ('revolute', 'continuous', 'prismatic')


def urdf_joint_names(urdf_xml):
    """Return all named joints in a URDF, rejecting malformed XML."""
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError as exc:
        raise ValueError(f'malformed URDF XML: {exc}') from exc
    return {
        element.attrib['name']
        for element in root.findall('joint')
        if 'name' in element.attrib
    }


@dataclass(frozen=True)
class Joint:
    """The kinematic subset of one URDF joint."""

    name: str
    parent: str
    child: str
    joint_type: str
    origin: np.ndarray
    origin_translation: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


@dataclass(frozen=True)
class IKResult:
    """Result and residual diagnostics from one IK solve."""

    positions: np.ndarray
    converged: bool
    position_error: float
    orientation_error: float
    iterations: int
    damping: float


class SerialChain:
    """A base-to-tip URDF chain with numerical differential IK."""

    def __init__(self, joints: Sequence[Joint], base_link: str, tip_link: str):
        self.joints = tuple(joints)
        self.base_link = base_link
        self.tip_link = tip_link
        self.active_joints = tuple(
            joint for joint in self.joints if joint.joint_type in ACTIVE_TYPES
        )
        if not self.active_joints:
            raise ValueError(f'chain {base_link} -> {tip_link} has no movable joints')
        self.joint_names = tuple(joint.name for joint in self.active_joints)
        self.lower = np.array([joint.lower for joint in self.active_joints])
        self.upper = np.array([joint.upper for joint in self.active_joints])
        self.nominal_reach = sum(
            float(np.linalg.norm(joint.origin_translation)) for joint in self.joints
        )
        if self.nominal_reach < 1.0e-3:
            self.nominal_reach = 1.0
        first_active_index = next(
            index
            for index, joint in enumerate(self.joints)
            if joint.joint_type in ACTIVE_TYPES
        )
        self.distal_reach = sum(
            float(np.linalg.norm(joint.origin_translation))
            for joint in self.joints[first_active_index + 1:]
        )
        if self.distal_reach < 1.0e-3:
            self.distal_reach = self.nominal_reach

    @classmethod
    def from_urdf(
        cls,
        urdf_xml: str,
        base_link: str,
        tip_link: str,
        expected_joint_names: Optional[Sequence[str]] = None,
    ):
        """Extract the unique parent path from a URDF string."""
        try:
            root = ET.fromstring(urdf_xml)
        except ET.ParseError as exc:
            raise ValueError(f'malformed URDF XML: {exc}') from exc
        links = {element.attrib['name'] for element in root.findall('link')}
        if base_link not in links:
            raise ValueError(f'base link {base_link!r} is absent from URDF')
        if tip_link not in links:
            raise ValueError(f'tip link {tip_link!r} is absent from URDF')

        by_child = {}
        for element in root.findall('joint'):
            child = element.find('child')
            if child is None or 'link' not in child.attrib:
                raise ValueError('URDF joint is missing its child link')
            child_name = child.attrib['link']
            if child_name in by_child:
                raise ValueError(f'link {child_name!r} has multiple parent joints')
            by_child[child_name] = element

        path_elements: List[ET.Element] = []
        current = tip_link
        visited = set()
        while current != base_link:
            if current in visited:
                raise ValueError('cycle detected while extracting URDF chain')
            visited.add(current)
            element = by_child.get(current)
            if element is None:
                raise ValueError(f'no URDF path from {base_link!r} to {tip_link!r}')
            path_elements.append(element)
            parent = element.find('parent')
            if parent is None or 'link' not in parent.attrib:
                raise ValueError('URDF joint is missing its parent link')
            current = parent.attrib['link']

        joints = [cls._parse_joint(element) for element in reversed(path_elements)]
        result = cls(joints, base_link, tip_link)
        if expected_joint_names:
            expected = tuple(expected_joint_names)
            if expected != result.joint_names:
                raise ValueError(
                    f'configured joints {expected} do not match URDF chain '
                    f'{result.joint_names}'
                )
        return result

    @staticmethod
    def _parse_joint(element):
        name = element.attrib.get('name', '')
        joint_type = element.attrib.get('type', '')
        if not name or not joint_type:
            raise ValueError('URDF joint requires name and type')
        if joint_type not in ACTIVE_TYPES + ('fixed',):
            raise ValueError(f'joint {name!r} has unsupported type {joint_type!r}')
        if element.find('mimic') is not None and joint_type in ACTIVE_TYPES:
            raise ValueError(f'mimic joint {name!r} cannot be an independent IK axis')

        parent = element.find('parent').attrib['link']
        child = element.find('child').attrib['link']
        origin = element.find('origin')
        xyz = _vector_attribute(origin, 'xyz', [0.0, 0.0, 0.0])
        rpy = _vector_attribute(origin, 'rpy', [0.0, 0.0, 0.0])
        axis_element = element.find('axis')
        axis = _vector_attribute(axis_element, 'xyz', [1.0, 0.0, 0.0])
        if joint_type in ACTIVE_TYPES:
            axis = normalize(axis)

        lower, upper = 0.0, 0.0
        if joint_type == 'continuous':
            lower, upper = -math.pi, math.pi
        elif joint_type in ('revolute', 'prismatic'):
            limit = element.find('limit')
            if limit is None or 'lower' not in limit.attrib or 'upper' not in limit.attrib:
                raise ValueError(f'joint {name!r} requires lower and upper limits')
            lower = float(limit.attrib['lower'])
            upper = float(limit.attrib['upper'])
            if not lower < upper:
                raise ValueError(f'joint {name!r} has invalid limits')

        return Joint(
            name=name,
            parent=parent,
            child=child,
            joint_type=joint_type,
            origin=origin_matrix(xyz, rpy),
            origin_translation=np.asarray(xyz, dtype=float),
            axis=np.asarray(axis, dtype=float),
            lower=lower,
            upper=upper,
        )

    def neutral_positions(self):
        """Return zero clipped into every joint's valid interval."""
        return np.clip(np.zeros(len(self.active_joints)), self.lower, self.upper)

    def proximal_joint_position(self):
        """Return the first movable joint origin in the chain base frame."""
        _, axes = self._forward_with_axes(self.neutral_positions())
        return axes[0][1].copy()

    def forward(self, positions):
        """Compute the base-to-tip homogeneous transform."""
        transform, _ = self._forward_with_axes(positions)
        return transform

    def jacobian(self, positions):
        """Compute a 6xN geometric Jacobian in the base frame."""
        end_transform, axes = self._forward_with_axes(positions)
        end_position = end_transform[:3, 3]
        result = np.zeros((6, len(self.active_joints)))
        for index, (joint, joint_position, world_axis) in enumerate(axes):
            if joint.joint_type in ('revolute', 'continuous'):
                result[:3, index] = np.cross(
                    world_axis, end_position - joint_position
                )
                result[3:, index] = world_axis
            else:
                result[:3, index] = world_axis
        return result

    def _forward_with_axes(self, positions):
        positions = np.asarray(positions, dtype=float)
        if positions.shape != (len(self.active_joints),):
            raise ValueError('joint position vector has the wrong size')
        transform = np.eye(4)
        axes = []
        active_index = 0
        for joint in self.joints:
            transform = transform @ joint.origin
            if joint.joint_type not in ACTIVE_TYPES:
                continue
            world_axis = transform[:3, :3] @ joint.axis
            axes.append((joint, transform[:3, 3].copy(), world_axis))
            value = positions[active_index]
            active_index += 1
            if joint.joint_type in ('revolute', 'continuous'):
                transform = transform @ axis_angle_matrix(joint.axis, value)
            else:
                transform = transform @ translation_matrix(joint.axis * value)
        return transform, axes

    def solve(
        self,
        target,
        seed,
        max_iterations=30,
        damping=0.03,
        singularity_threshold=0.05,
        max_joint_step=0.15,
        position_tolerance=0.005,
        orientation_tolerance=0.04,
        orientation_weight=0.35,
    ):
        """Solve IK using adaptively damped least squares and hard joint limits."""
        positions = np.clip(np.asarray(seed, dtype=float), self.lower, self.upper)
        target = np.asarray(target, dtype=float)
        best_positions = positions.copy()
        best_score = math.inf
        best_position_error = math.inf
        best_orientation_error = math.inf
        used_damping = damping

        for iteration in range(1, max_iterations + 1):
            current = self.forward(positions)
            linear_error = target[:3, 3] - current[:3, 3]
            angular_error = orientation_error(current[:3, :3], target[:3, :3])
            position_norm = float(np.linalg.norm(linear_error))
            orientation_norm = float(np.linalg.norm(angular_error))
            score = position_norm + orientation_weight * orientation_norm
            if score < best_score:
                best_score = score
                best_positions = positions.copy()
                best_position_error = position_norm
                best_orientation_error = orientation_norm
            orientation_ok = (
                orientation_weight <= 0.0
                or orientation_norm <= orientation_tolerance
            )
            if position_norm <= position_tolerance and orientation_ok:
                return IKResult(
                    positions=positions,
                    converged=True,
                    position_error=position_norm,
                    orientation_error=orientation_norm,
                    iterations=iteration,
                    damping=used_damping,
                )

            error = np.concatenate(
                (linear_error, orientation_weight * angular_error)
            )
            jacobian = self.jacobian(positions)
            jacobian[3:, :] *= orientation_weight
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
            smallest = float(singular_values[-1]) if singular_values.size else 0.0
            singular_fraction = max(
                0.0,
                (singularity_threshold - smallest)
                / max(singularity_threshold, 1.0e-9),
            )
            used_damping = damping * (1.0 + 9.0 * singular_fraction)
            regularized = (
                jacobian @ jacobian.T + used_damping * used_damping * np.eye(6)
            )
            try:
                delta = jacobian.T @ np.linalg.solve(regularized, error)
            except np.linalg.LinAlgError:
                delta = jacobian.T @ np.linalg.pinv(regularized) @ error
            delta = np.clip(delta, -max_joint_step, max_joint_step)
            positions = np.clip(positions + delta, self.lower, self.upper)

        return IKResult(
            positions=best_positions,
            converged=False,
            position_error=best_position_error,
            orientation_error=best_orientation_error,
            iterations=max_iterations,
            damping=used_damping,
        )


def _vector_attribute(element, attribute, default):
    if element is None or attribute not in element.attrib:
        return np.asarray(default, dtype=float)
    values = [float(value) for value in element.attrib[attribute].split()]
    if len(values) != 3 or not np.all(np.isfinite(values)):
        raise ValueError(f'URDF {attribute} must contain three finite numbers')
    return np.asarray(values, dtype=float)
