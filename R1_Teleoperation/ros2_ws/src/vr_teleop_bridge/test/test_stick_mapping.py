"""Exercise outgoing velocity mapping without opening sockets or ROS nodes."""
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from vr_teleop_bridge.node import VRBridgeNode


def bridge():
    node = object.__new__(VRBridgeNode)
    node._translation_axis_snap_ratio = .6
    node._stick_input_scale = 4.0
    node._deadzone = .15
    node._max_forward_mps = .20
    node._max_lateral_mps = .12
    node._max_yaw_rps = .35
    node._forward_sign = 1.0
    node._lateral_sign = node._yaw_sign = -1.0
    return node


@pytest.mark.parametrize('left,expected', [
    ((-.172, .328), (.20, 0)),
    ((.172, -.328), (-.20, 0)),
    ((-.328, .172), (0, .12)),
    ((.328, -.172), (0, -.12)),
    ((-.25, .25), (.20, .12)),
    ((0, 0), (0, 0)),
])
def test_velocity_assist_precedes_gain_and_keeps_direction_signs(left, expected):
    node = bridge()
    packet = SimpleNamespace(left_stick=left, right_stick=(.25, 0))
    twist = node._velocity_message(Time(), packet, True).twist
    assert (twist.linear.x, twist.linear.y) == pytest.approx(expected)
    assert twist.angular.z == -.35  # right-stick turning is unchanged


def test_paused_control_stays_zero_even_with_displaced_sticks():
    node = bridge()
    packet = SimpleNamespace(left_stick=(.2, .3), right_stick=(.3, 0))
    twist = node._velocity_message(Time(), packet, False).twist
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0, 0, 0)
    # Assistance must never disguise non-neutral input during RUN/reconnect.
    assert not node._locomotion_controls_neutral(packet)
