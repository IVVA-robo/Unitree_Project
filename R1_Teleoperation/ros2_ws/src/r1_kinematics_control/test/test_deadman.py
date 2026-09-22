from types import SimpleNamespace

from std_msgs.msg import Bool

from r1_kinematics_control.node import R1KinematicsControl


class _Recorder:
    def __init__(self):
        self.calls = 0

    def __call__(self):
        self.calls += 1


def test_false_heartbeat_stops_outputs_only_on_transition(monkeypatch):
    node = object.__new__(R1KinematicsControl)
    node._active_arrival = None
    node._teleop_active = False
    node._active_seen = False
    node._enabled_last_tick = False
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
