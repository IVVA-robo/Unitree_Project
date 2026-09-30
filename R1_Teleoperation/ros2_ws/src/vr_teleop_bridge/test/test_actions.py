from types import SimpleNamespace

from vr_teleop_bridge.node import VRBridgeNode
from vr_teleop_bridge.resilience import DebouncedButton, EmergencyStopButton


class Recorder:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(message.data)


class Gate:
    def __init__(self):
        self.disarmed = False

    def disarm(self):
        self.disarmed = True


class Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


def _button_node():
    node = object.__new__(VRBridgeNode)
    node._left_x = DebouncedButton(0.05)
    node._right_b = EmergencyStopButton(0.05)
    node._arms_neutral_active = False
    node._emergency_latched = False
    node._last_action = 'none'
    node._arms_neutral_pub = Recorder()
    node._emergency_stop_pub = Recorder()
    node._action_status_pub = Recorder()
    node._kill_request_pub = Recorder()
    node._session_gate = Gate()
    node._session_publish_count = 0
    node._emergency_hold_count = 0
    node._publish_session_armed = lambda: setattr(
        node, '_session_publish_count', node._session_publish_count + 1
    )
    node._publish_emergency_hold = lambda: setattr(
        node, '_emergency_hold_count', node._emergency_hold_count + 1
    )
    node.get_logger = lambda: Logger()
    return node


def _packet(left_x=False, right_b=False):
    return SimpleNamespace(
        buttons=SimpleNamespace(left_x=left_x, right_b=right_b)
    )


def test_left_x_holds_neutral_then_returns_to_vr_follow():
    node = _button_node()

    node._handle_button_actions(_packet(left_x=True), 0.00)
    node._handle_button_actions(_packet(left_x=True), 0.05)
    node._handle_button_actions(_packet(left_x=True), 0.50)
    node._handle_button_actions(_packet(left_x=False), 0.51)
    node._handle_button_actions(_packet(left_x=False), 0.56)

    assert node._arms_neutral_pub.values == [True, False]
    assert node._action_status_pub.values == [
        'arms_reset_to_neutral',
        'arms_vr_follow_resumed',
    ]
    assert node._emergency_latched is False


def test_right_b_zeros_disarms_and_requests_kill_only_once():
    node = _button_node()

    node._handle_button_actions(_packet(right_b=True), 0.00)
    node._handle_button_actions(_packet(right_b=True), 0.01)
    node._handle_button_actions(_packet(right_b=False), 0.10)

    assert node._emergency_latched is True
    assert node._session_gate.disarmed is True
    assert node._session_publish_count == 1
    assert node._emergency_hold_count == 1
    assert node._arms_neutral_pub.values == [True]
    assert node._emergency_stop_pub.values == [True]
    assert node._kill_request_pub.values == [True]
    assert node._action_status_pub.values == ['emergency_stop_right_b']
