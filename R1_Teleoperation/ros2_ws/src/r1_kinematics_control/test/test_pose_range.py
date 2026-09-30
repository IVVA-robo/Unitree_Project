"""Offline range saturation and joint envelope checks, without ROS."""

from types import SimpleNamespace

import numpy as np
import pytest

from r1_kinematics_control.pose_range import ArmPoseRange


def example():
    chain = SimpleNamespace(joint_names=('pitch', 'roll'),
                            lower=np.array([-2., -1.]), upper=np.array([2., 1.]))
    arm = dict(joint_names=['pitch', 'roll'], input_basis_m=np.eye(3).tolist(),
               robot_neutral_rad=[0., 0.],
               robot_endpoints_rad=[[-1.8, .2], [0., .9], [.6, .1]])
    data = dict(version=1, scope='offline_preview', arms={'left': arm, 'right': arm})
    return data, {'left': chain, 'right': chain}


def test_neutral_and_measured_maximum_and_beyond():
    data, chains = example()
    profile = ArmPoseRange(data, chains)
    assert profile.evaluate('left', [0, 0, 0])[0] == pytest.approx([0, 0])
    for amount in (1., 1.5, 4.):
        target, weights = profile.evaluate('left', [amount, 0, 0])
        assert target == pytest.approx([-1.8, .2])
        assert weights == pytest.approx([1, 0, 0])


def test_mixed_and_outside_poses_stay_inside_joint_envelope():
    data, chains = example()
    profile = ArmPoseRange(data, chains)
    for delta in np.random.default_rng(42).uniform(-3, 3, (1000, 3)):
        target, weights = profile.evaluate('right', delta)
        assert weights.sum() <= 1.00000000001
        assert (weights >= 0).all()
        assert (target >= chains['right'].lower).all()
        assert (target <= chains['right'].upper).all()


def test_continuous_at_saturation_boundary():
    data, chains = example()
    profile = ArmPoseRange(data, chains)
    before, _ = profile.evaluate('left', [.5 - 1e-6, .5, 0])
    after, _ = profile.evaluate('left', [.5 + 1e-6, .5, 0])
    assert np.linalg.norm(after - before) < 1e-5


def test_forward_elbow_anchor_and_lowered_hands_have_distinct_outputs():
    data, chains = example()
    data['lowered_hands_to_rest'] = True
    for arm in data['arms'].values():
        arm['robot_reset_rad'] = [.1, -.1]
    profile = ArmPoseRange(data, chains)
    assert profile.evaluate('left', [0, 0, 0])[0] == pytest.approx([0., 0.])
    assert profile.evaluate('left', [0, 0, 1])[0] == pytest.approx([.6, .1])
    assert profile.evaluate('left', [-.4, 0, -.4])[0] == pytest.approx([.1, -.1])
    assert profile.reset_target('left') == pytest.approx([.1, -.1])
    for z in (-.35, -.04, -.02, .08):
        before = profile.evaluate('left', [.1, .1, z - 1e-7])[0]
        after = profile.evaluate('left', [.1, .1, z + 1e-7])[0]
        assert np.linalg.norm(after - before) < 1e-5


def test_probe_cannot_enable_full_range():
    data, chains = example()
    profile = ArmPoseRange(data, chains)
    assert profile.probe_target('left', [5, 0, 0], .1) == pytest.approx([-.18, .02])
    for invalid in (0, -.1, .101, 1, np.nan, np.inf):
        with pytest.raises(ValueError):
            profile.probe_target('left', [1, 0, 0], invalid)


def test_hips_start_needs_both_lowered_finite_hands_and_actual_rest_output():
    profile = SimpleNamespace(rest_start_enabled=True,
        evaluate=lambda *_: (np.zeros(2), None), reset_target=lambda _: np.zeros(2))
    valid = {'left': [-.4, .05, -.5], 'right': [-.4, -.05, -.5]}
    assert ArmPoseRange.allows_rest_start(profile, valid)
    for bad in ([0., 0., 0.], [-.4, .5, -.5], [-.4, 0., -.2], [np.nan, 0., -.5]):
        assert not ArmPoseRange.allows_rest_start(profile, dict(valid, right=bad))
    profile.evaluate = lambda *_: (np.ones(2), None)
    assert not ArmPoseRange.allows_rest_start(profile, valid)


def test_relative_preview_cannot_opt_into_hips_start():
    data, chains = example()
    data.update(startup_pose='hands_at_hips', lowered_hands_to_rest=True)
    with pytest.raises(ValueError):
        ArmPoseRange(data, chains)


def test_bounded_profile_requires_offset_and_checks_physical_envelope():
    import copy
    data, chains = example()
    data['arms']['right'] = copy.deepcopy(data['arms']['right'])
    chains['right'] = copy.deepcopy(chains['right'])
    data['scope'] = 'supervised_bounded'
    with pytest.raises(KeyError):
        ArmPoseRange(data, chains)
    for side, chain in chains.items():
        data['arms'][side]['writer_offset_rad'] = [0., 0.]
        def forward(q, side=side):
            pose = np.eye(4)
            pose[:3, 3] = [.1, .2 if side == 'left' else -.2, 0.]
            return pose
        chain.forward = forward
    profile = ArmPoseRange(data, chains)
    assert profile.probe_target('left', [1, 0, 0], 1.) == pytest.approx([-1.8, .2])
    with pytest.raises(ValueError):
        profile.probe_target('left', [1, 0, 0], 1.001)
    data['arms']['left']['writer_offset_rad'] = [1.5, 0.]
    with pytest.raises(ValueError):
        ArmPoseRange(data, chains)


def test_rejects_degenerate_calibration_and_invalid_joints():
    data, chains = example()
    data['arms']['left']['input_basis_m'] = np.zeros((3, 3)).tolist()
    with pytest.raises(ValueError):
        ArmPoseRange(data, chains)
    data, chains = example()
    data['arms']['left']['robot_endpoints_rad'][0][0] = -3
    with pytest.raises(ValueError):
        ArmPoseRange(data, chains)


def test_rejects_nonfinite_input_and_wont_claim_live_profile():
    data, chains = example()
    profile = ArmPoseRange(data, chains)
    with pytest.raises(ValueError):
        profile.evaluate('left', [0, np.nan, 1])
    data['scope'] = 'live'
    with pytest.raises(ValueError):
        ArmPoseRange(data, chains)
