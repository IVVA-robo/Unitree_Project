from types import SimpleNamespace

import numpy as np
import pytest
from std_msgs.msg import Bool

from r1_kinematics_control.node import R1KinematicsControl


class _Recorder:
    def __init__(self):
        self.calls = 0

    def __call__(self):
        self.calls += 1


@pytest.mark.parametrize('exhibition', [False, True])
def test_false_heartbeat_stops_outputs_only_on_transition(monkeypatch, exhibition):
    node = object.__new__(R1KinematicsControl)
    node._active_arrival = None
    node._teleop_active = False
    node._active_seen = False
    node._enabled_last_tick = False
    node._body_neutral_checked = True
    node._exhibition_session_mode = exhibition
    node._arms = {
        'left': SimpleNamespace(was_stale=False),
        'right': SimpleNamespace(was_stale=False),
    }
    node._body_proxy = SimpleNamespace(reset_filter=_Recorder())
    zero = _Recorder()
    hold = _Recorder()
    monkeypatch.setattr(node, '_publish_zero_velocity', zero)
    monkeypatch.setattr(node, '_publish_hold_trajectories', hold)

    node._active_callback(Bool(data=False))
    node._active_callback(Bool(data=False))
    assert zero.calls == 1
    assert hold.calls == 1
    assert node._body_proxy.reset_filter.calls == 1

    node._active_callback(Bool(data=True))
    node._active_callback(Bool(data=True))
    assert zero.calls == 1

    node._active_callback(Bool(data=False))
    assert zero.calls == 2
    assert hold.calls == 2
    assert node._body_proxy.reset_filter.calls == 2
    assert all(runtime.was_stale for runtime in node._arms.values())
    assert node._body_neutral_checked is exhibition


def _neutral_guard_node(errors):
    node = object.__new__(R1KinematicsControl)
    node._arm_range_probe = None
    result = SimpleNamespace(
        hands_body_local={'left': np.eye(4), 'right': np.eye(4)},
        targets={'left': np.eye(4), 'right': np.eye(4)},
        clamped={'left': (), 'right': ()},
    )
    node._fresh_tracking_poses = lambda _now, warn: {
        'head': np.eye(4), 'left': np.eye(4), 'right': np.eye(4)
    }
    node._body_proxy = SimpleNamespace(
        calibration=object(),
        update=lambda *_args: result,
        neutral_position_errors=lambda _hands: errors,
        neutral_targets=lambda: {
            'left': np.eye(4) * 2.0,
            'right': np.eye(4) * 3.0,
        },
    )
    node._body_require_calibration = True
    node._body_neutral_guard_enabled = True
    node._body_neutral_checked = False
    node._body_neutral_max_error = 0.15
    node._arms_neutral_requested = False
    node._body_calibration_service = '/vr/calibrate_body'
    node._arms = {
        'left': SimpleNamespace(was_stale=False),
        'right': SimpleNamespace(was_stale=False),
    }
    node._publish_body_debug = lambda *_args: None
    node._warned = []
    node._warn = lambda key, message: node._warned.append((key, message))
    return node


def test_probe_first_frame_is_neutral_then_rate_limited():
    node = object.__new__(R1KinematicsControl)
    node._arm_range_probe = SimpleNamespace(
        contains_target=lambda *_: True,
        max_probe_rate=.20,
        absolute_targets=False,
    )
    node._arm_range_probe_targets = {'left': np.array([-.2, .06])}
    node._max_joint_velocity = .75
    runtime = SimpleNamespace(side='left', command=None, chain=SimpleNamespace(
        neutral_positions=lambda: np.zeros(2), lower=np.array([-3., -3.]),
        upper=np.array([3., 3.])))
    assert node._solve_arm(runtime, np.eye(4), .02) == pytest.approx([0, 0])
    assert node._solve_arm(runtime, np.eye(4), .02) == pytest.approx([-.004, .0012])
    # A scheduler stall cannot bypass the capped probe rate.
    assert node._solve_arm(runtime, np.eye(4), 5.) == pytest.approx([-.014, .0042])
    node._arm_range_probe_targets = {}
    assert node._solve_arm(runtime, np.eye(4), .02) is None


def test_bounded_probe_holds_when_physical_envelope_rejects_step():
    node = object.__new__(R1KinematicsControl)
    node._arm_range_probe = SimpleNamespace(contains_target=lambda *_: False, max_probe_rate=.20)
    node._arm_range_probe_targets = {'left': np.array([-1., 1.])}
    node._max_joint_velocity = .2
    node._warn = lambda *_: None
    runtime = SimpleNamespace(side='left', command=np.zeros(2), chain=SimpleNamespace(
        lower=np.array([-3., -3.]), upper=np.array([3., 3.])))
    assert node._solve_arm(runtime, np.eye(4), .02) == pytest.approx([0., 0.])


def test_absolute_profile_starts_at_calibrated_neutral():
    node = object.__new__(R1KinematicsControl)
    node._arm_range_probe = SimpleNamespace(
        absolute_targets=True,
        reset_target=lambda _: np.array([.2, .3]),
    )
    node._arm_range_probe_targets = {'left': np.array([-1., 1.])}
    runtime = SimpleNamespace(side='left', command=None)
    assert node._solve_arm(runtime, np.eye(4), .02) == pytest.approx([.2, .3])


def test_probe_hold_does_not_add_physical_feedback_twice():
    node = object.__new__(R1KinematicsControl)
    node._arm_range_probe = object()
    node._arms = {
        side: SimpleNamespace(
            command=np.array([.1]),
            chain=SimpleNamespace(joint_names=(side,)),
        )
        for side in ('left', 'right')
    }
    node._joint_positions = {'left': .6, 'right': .7}
    node._trajectory_time_sec = .1
    node._hands_commanded = False
    sent = []
    node._publish_arm_command = sent.append
    node._publish_hold_trajectories()
    assert sent[0].points[0].positions == pytest.approx([.1, .1])


@pytest.mark.parametrize('rate', [.75, 1.0])
def test_checked_profile_faster_rate_keeps_ceiling_and_stall_bound(rate):
    node = object.__new__(R1KinematicsControl)
    node._arm_range_probe = SimpleNamespace(contains_target=lambda *_: True, max_probe_rate=rate)
    node._arm_range_probe_targets = {'left': np.array([-2., .6])}
    node._max_joint_velocity = 10.
    runtime = SimpleNamespace(side='left', command=np.zeros(2), chain=SimpleNamespace(
        lower=np.array([-3., -3.]), upper=np.array([3., 3.])))
    assert node._solve_arm(runtime, np.eye(4), .02) == pytest.approx([-rate * .02, rate * .006])
    before = runtime.command.copy()
    after = node._solve_arm(runtime, np.eye(4), 5.)
    assert np.max(np.abs(after - before)) == pytest.approx(rate * .05)


def test_probe_x_uses_neutral_for_both_arms():
    node = _neutral_guard_node({'left': 0., 'right': 0.})
    node._arm_range_probe = SimpleNamespace(reset_target=lambda _: np.zeros(5))
    node._arms_neutral_requested = True
    for runtime in node._arms.values():
        runtime.chain = SimpleNamespace(neutral_positions=lambda: np.zeros(5))
    node._headset_relative_targets(1., .02)
    assert set(node._arm_range_probe_targets) == {'left', 'right'}
    assert all(np.all(q == 0) for q in node._arm_range_probe_targets.values())


def test_stale_calibration_blocks_targets_before_ik_publish():
    node = _neutral_guard_node({'left': 0.02, 'right': 0.30})

    assert node._headset_relative_targets(1.0, 0.02) is None
    assert node._body_neutral_checked is False
    assert all(runtime.was_stale for runtime in node._arms.values())
    assert node._warned[0][0] == 'body_calibration_neutral_mismatch'


def test_matching_neutral_unlocks_targets_for_current_deadman_press():
    node = _neutral_guard_node({'left': 0.02, 'right': 0.03})

    targets = node._headset_relative_targets(1.0, 0.02)

    assert set(targets) == {'left', 'right'}
    assert node._body_neutral_checked is True
    assert node._warned == []


def test_left_x_selects_both_robot_neutral_targets_without_disarming():
    node = _neutral_guard_node({'left': 0.02, 'right': 0.03})
    node._arms_neutral_requested = True

    targets = node._headset_relative_targets(1.0, 0.02)

    assert targets['left'][0, 0] == 2.0
    assert targets['right'][0, 0] == 3.0
    assert node._body_neutral_checked is True


def _tracking_sync_node(arrivals, tolerance=0.08):
    node = object.__new__(R1KinematicsControl)
    node._head_target = SimpleNamespace(arrival=arrivals[0])
    node._left_target = SimpleNamespace(arrival=arrivals[1])
    node._right_target = SimpleNamespace(arrival=arrivals[2])
    node._pose_timeout_sec = 0.25
    node._body_sync_tolerance = tolerance
    return node


def test_pico_one_frame_tracking_skew_is_accepted():
    node = _tracking_sync_node((1.000, 1.079, 1.040))

    assert node._tracking_pose_issue(1.080) is None


def test_tracking_skew_above_bounded_tolerance_is_rejected():
    node = _tracking_sync_node((1.000, 1.081, 1.040))

    issue = node._tracking_pose_issue(1.082)

    assert issue[0] == 'sync'
    assert '0.081s exceeds 0.080s' in issue[1]
