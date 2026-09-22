import json
import math

import pytest

from vr_teleop_bridge.protocol import (
    PacketError,
    apply_deadzone,
    is_newer_sequence,
    parse_packet,
)


def packet_dict():
    identity = {'p': [0.1, 0.2, 1.0], 'q': [0.0, 0.0, 0.0, 1.2]}
    return {
        'v': 1,
        'seq': 7,
        'client_time_ms': 123456,
        'left': identity,
        'right': identity,
        'head': identity,
        'sticks': {'left': [0.2, -0.3], 'right': [0.4, 0.0]},
        'triggers': [0.25, 0.75],
        'deadman': True,
    }


def encode(data):
    return json.dumps(data, separators=(',', ':')).encode()


def test_parse_and_normalize_quaternion():
    packet = parse_packet(encode(packet_dict()))
    assert packet.sequence == 7
    assert packet.deadman is True
    assert packet.left_trigger == 0.25
    assert packet.right_trigger == 0.75
    assert packet.left.orientation == (0.0, 0.0, 0.0, 1.0)


@pytest.mark.parametrize('field', ['left', 'right', 'head'])
def test_reject_bad_pose(field):
    data = packet_dict()
    data[field] = {'p': [0, 0, 99], 'q': [0, 0, 0, 1]}
    with pytest.raises(PacketError):
        parse_packet(encode(data))


def test_reject_nan_even_when_python_json_accepts_it():
    data = packet_dict()
    data['sticks']['left'][0] = math.nan
    with pytest.raises(PacketError):
        parse_packet(encode(data))


def test_deadzone_is_continuous_and_rescaled():
    assert apply_deadzone(0.15, 0.15) == 0.0
    assert apply_deadzone(-0.15, 0.15) == 0.0
    assert apply_deadzone(1.0, 0.15) == 1.0
    assert apply_deadzone(-1.0, 0.15) == -1.0
    assert apply_deadzone(0.575, 0.15) == pytest.approx(0.5)


def test_sequence_order_and_wraparound():
    assert is_newer_sequence(1, None)
    assert is_newer_sequence(11, 10)
    assert not is_newer_sequence(10, 10)
    assert not is_newer_sequence(9, 10)
    assert is_newer_sequence(0, (1 << 32) - 1)


def test_legacy_v1_packet_defaults_triggers_to_released():
    data = packet_dict()
    del data['triggers']
    packet = parse_packet(encode(data))
    assert (packet.left_trigger, packet.right_trigger) == (0.0, 0.0)
