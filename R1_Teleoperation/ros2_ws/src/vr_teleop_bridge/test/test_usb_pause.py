"""Exercise the bridge's real timeout method with a real session gate."""

from types import SimpleNamespace

import pytest

from vr_teleop_bridge.node import VRBridgeNode
from vr_teleop_bridge.protocol import TrackingAvailability
from vr_teleop_bridge.resilience import ExhibitionSessionGate


@pytest.mark.parametrize('usb', [True, False])
def test_usb_timeout_holds_session_but_needs_explicit_run(usb):
    tracking = TrackingAvailability(left=True, right=True, head=True)
    gate = ExhibitionSessionGate()
    gate.observe(tracking, 0.0, locomotion_neutral=True)
    assert gate.arm(True)[0]
    node = object.__new__(VRBridgeNode)
    node._session_gate = gate
    node._pause_on_packet_timeout = usb
    node._pose_recovery = SimpleNamespace(ready=True, update=lambda *args: {})
    node._packet_outage_active = True
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))
    node._publish_poses = lambda *args: None
    node._joy_message = lambda *args: None
    node._velocity_message = lambda *args: None
    publisher = SimpleNamespace(publish=lambda msg: None)
    node._joy_pub = node._velocity_pub = publisher
    node._active_pub = node._locomotion_active_pub = publisher
    node._set_active = node._set_locomotion_active = lambda *args: None

    node._publish_session_hold(0.5, 0.5)
    assert gate.armed
    gate.observe(tracking, 0.6, locomotion_neutral=True)
    assert gate.decision(0.6, True).control_active is (not usb)
    if usb:
        gate.observe(tracking, 0.7, locomotion_neutral=False)
        assert not gate.resume(True)[0]
        gate.observe(tracking, 0.8, locomotion_neutral=True)
        assert gate.resume(True)[0]
        assert gate.decision(0.8, True).control_active
