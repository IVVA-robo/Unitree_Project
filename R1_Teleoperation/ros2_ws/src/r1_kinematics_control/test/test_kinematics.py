import math

import numpy as np
import pytest

from r1_kinematics_control.kinematics import SerialChain, urdf_joint_names
from r1_kinematics_control.math3d import pose_matrix


PLANAR_URDF = '''
<robot name="planar">
  <link name="base"/>
  <link name="upper"/>
  <link name="forearm"/>
  <link name="tip"/>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="upper"/>
    <axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="elbow" type="revolute">
    <parent link="upper"/><child link="forearm"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="tool" type="fixed">
    <parent link="forearm"/><child link="tip"/>
    <origin xyz="1 0 0"/>
  </joint>
</robot>
'''


def test_extract_chain_forward_and_limits():
    chain = SerialChain.from_urdf(
        PLANAR_URDF, 'base', 'tip', ['shoulder', 'elbow']
    )
    assert chain.joint_names == ('shoulder', 'elbow')
    end = chain.forward([0.0, math.pi / 2.0])
    assert end[:3, 3] == pytest.approx([1.0, 1.0, 0.0], abs=1.0e-6)
    assert chain.nominal_reach == pytest.approx(2.0)
    assert urdf_joint_names(PLANAR_URDF) == {'shoulder', 'elbow', 'tool'}


def test_damped_ik_reaches_planar_target():
    chain = SerialChain.from_urdf(PLANAR_URDF, 'base', 'tip')
    target = pose_matrix([1.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    result = chain.solve(
        target,
        np.array([0.05, 0.05]),
        max_iterations=80,
        orientation_weight=0.0,
    )
    assert result.converged
    assert result.position_error < 0.005
    assert chain.forward(result.positions)[:3, 3] == pytest.approx(
        [1.0, 1.0, 0.0], abs=0.005
    )


def test_bad_explicit_joint_mapping_is_rejected():
    with pytest.raises(ValueError, match='do not match'):
        SerialChain.from_urdf(PLANAR_URDF, 'base', 'tip', ['wrong_joint'])


def test_malformed_or_disconnected_urdf_is_rejected():
    with pytest.raises(ValueError, match='malformed'):
        SerialChain.from_urdf('<robot>', 'base', 'tip')
    with pytest.raises(ValueError, match='absent'):
        SerialChain.from_urdf(PLANAR_URDF, 'missing', 'tip')
