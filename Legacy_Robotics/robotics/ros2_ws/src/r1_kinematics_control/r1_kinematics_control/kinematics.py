"""URDF parsing and Inverse Kinematics solver."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET
from typing import List, Optional, Sequence

import numpy as np

from .math3d import axis_angle_matrix, origin_matrix, translation_matrix, orientation_error, normalize

ACTIVE_TYPES = ('revolute', 'prismatic', 'continuous')

@dataclass
class IKResult:
    positions: np.ndarray
    converged: bool
    position_error: float
    orientation_error: float
    iterations: int
    damping: float

@dataclass
class Joint:
    name: str
    parent: str
    child: str
    joint_type: str
    origin: np.ndarray
    origin_translation: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float

class SerialChain:
    def __init__(self, joints: List[Joint], base_link: str, tip_link: str):
        self.joints = joints
        self.base_link = base_link
        self.tip_link = tip_link
        self.active_joints = [j for j in joints if j.joint_type in ACTIVE_TYPES]
        self.joint_names = tuple(j.name for j in self.active_joints)
        self.lower = np.array([j.lower for j in self.active_joints])
        self.upper = np.array([j.upper for j in self.active_joints])
        self.nominal_reach = sum(np.linalg.norm(j.origin_translation) for j in joints)

    @classmethod
    def from_urdf(cls, description: str, base_link: str, tip_link: str, explicit_joints: Optional[Sequence[str]] = None):
        try:
            root = ET.fromstring(description)
        except ET.ParseError as exc:
            raise ValueError(f'malformed URDF: {exc}')
        
        links = {link.attrib['name'] for link in root.findall('link')}
        if base_link not in links or tip_link not in links:
            raise ValueError('base_link or tip_link absent from URDF')
        
        joint_map = {}
        for joint_el in root.findall('joint'):
            parent_el = joint_el.find('parent')
            child_el = joint_el.find('child')
            if parent_el is not None and child_el is not None:
                joint_map[child_el.attrib['link']] = joint_el
        
        chain_elements = []
        current = tip_link
        while current != base_link:
            if current not in joint_map:
                raise ValueError(f'chain disconnected at {current}')
            joint_el = joint_map[current]
            chain_elements.append(joint_el)
            current = joint_el.find('parent').attrib['link']
        chain_elements.reverse()
        
        joints = [cls._parse_joint(el) for el in chain_elements]
        chain = cls(joints, base_link, tip_link)
        
        if explicit_joints is not None and explicit_joints != ('AUTO',):
            if chain.joint_names != tuple(explicit_joints):
                raise ValueError('explicit joints do not match URDF path')
        return chain

    @staticmethod
    def _parse_joint(element) -> Joint:
        name = element.attrib.get('name', 'unnamed')
        joint_type = element.attrib.get('type', 'fixed')
        
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
                result[:3, index] = np.cross(world_axis, end_position - joint_position)
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
        self, target, seed, max_iterations=30, damping=0.03,
        singularity_threshold=0.05, max_joint_step=0.15,
        position_tolerance=0.005, orientation_tolerance=0.04,
        orientation_weight=0.35
    ):
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
                
            orientation_ok = (orientation_weight <= 0.0 or orientation_norm <= orientation_tolerance)
            if position_norm <= position_tolerance and orientation_ok:
                return IKResult(
                    positions=positions, converged=True,
                    position_error=position_norm, orientation_error=orientation_norm,
                    iterations=iteration, damping=used_damping,
                )

            error = np.concatenate((linear_error, orientation_weight * angular_error))
            jacobian = self.jacobian(positions)
            jacobian[3:, :] *= orientation_weight
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
            smallest = float(singular_values[-1]) if singular_values.size else 0.0
            singular_fraction = max(
                0.0, (singularity_threshold - smallest) / max(singularity_threshold, 1.0e-9)
            )
            used_damping = damping * (1.0 + 9.0 * singular_fraction)
            regularized = jacobian @ jacobian.T + used_damping * used_damping * np.eye(6)
            
            try:
                delta = jacobian.T @ np.linalg.solve(regularized, error)
            except np.linalg.LinAlgError:
                delta = jacobian.T @ np.linalg.pinv(regularized) @ error
                
            delta = np.clip(delta, -max_joint_step, max_joint_step)
            positions = np.clip(positions + delta, self.lower, self.upper)

        return IKResult(
            positions=best_positions, converged=False,
            position_error=best_position_error, orientation_error=best_orientation_error,
            iterations=max_iterations, damping=used_damping,
        )

def _vector_attribute(element, attribute, default):
    if element is None or attribute not in element.attrib:
        return np.asarray(default, dtype=float)
    values = [float(value) for value in element.attrib[attribute].split()]
    if len(values) != 3 or not np.all(np.isfinite(values)):
        raise ValueError(f'URDF {attribute} must contain three finite numbers')
    return np.asarray(values, dtype=float)
