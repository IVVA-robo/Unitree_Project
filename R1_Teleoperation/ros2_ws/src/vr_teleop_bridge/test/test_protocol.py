import json
import math

import pytest

from vr_teleop_bridge.protocol import (
    PacketError,
    apply_deadzone,
    is_newer_sequence,
    parse_packet,
    scale_stick_axis,
    shape_translation_stick,
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
        'tracking': {'left': True, 'right': True, 'head': True},
        'buttons': {'left_x': False, 'right_b': False},
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
    assert packet.tracking.all_available
    assert packet.tracking.reported is True
    assert packet.buttons.reported is True
    assert packet.buttons.left_x is False
    assert packet.buttons.right_b is False
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


def test_pico_quarter_range_is_normalized_and_clamped():
    assert scale_stick_axis(0.25, 4.0) == 1.0
    assert scale_stick_axis(-0.25, 4.0) == -1.0
    assert scale_stick_axis(0.125, 4.0) == 0.5
    assert scale_stick_axis(0.8, 4.0) == 1.0


def test_sequence_order_and_wraparound():
    assert is_newer_sequence(1, None)
    assert is_newer_sequence(11, 10)
    assert not is_newer_sequence(10, 10)
    assert not is_newer_sequence(9, 10)
    assert is_newer_sequence(0, (1 << 32) - 1)


@pytest.mark.parametrize('sx', [-1, 1])
@pytest.mark.parametrize('sy', [-1, 1])
def test_translation_assist_keeps_forward_and_sideways_straight(sx, sy):
    # Captured real forward input previously produced a large lateral request.
    assert shape_translation_stick(sx * .172, sy * .328, .6) == (0, sy * .328)
    assert shape_translation_stick(sx * .328, sy * .172, .6) == (sx * .328, 0)
    assert shape_translation_stick(sx * .25, sy * .25, .6) == (sx * .25, sy * .25)


def test_translation_assist_has_no_step_or_amplification_at_direction_boundary():
    last = 0
    for i in range(101):
        minor = i / 100
        x, y = shape_translation_stick(minor, 1, .6)
        assert 0 <= last <= x <= minor
        assert y == 1
        last = x
    assert shape_translation_stick(.6, 1, .6) == (0, 1)
    assert shape_translation_stick(.600001, 1, .6)[0] < 1e-9
    assert shape_translation_stick(0, 0, .6) == (0, 0)
    assert shape_translation_stick(.2, .9, 0) == (.2, .9)


@pytest.mark.parametrize('args', [(float('nan'), 0, .6), (0, float('inf'), .6), (0, 0, .9)])
def test_translation_assist_rejects_invalid_inputs(args):
    with pytest.raises(ValueError):
        shape_translation_stick(*args)


def test_legacy_v1_packet_defaults_triggers_to_released():
    data = packet_dict()
    del data['triggers']
    packet = parse_packet(encode(data))
    assert (packet.left_trigger, packet.right_trigger) == (0.0, 0.0)


def test_legacy_v1_packet_is_valid_but_tracking_is_not_explicitly_reported():
    data = packet_dict()
    del data['tracking']
    packet = parse_packet(encode(data))
    assert packet.tracking.reported is False
    assert packet.tracking.all_available is False


def test_legacy_v1_packet_defaults_action_buttons_to_released():
    data = packet_dict()
    del data['buttons']
    packet = parse_packet(encode(data))
    assert packet.buttons.reported is False
    assert packet.buttons.left_x is False
    assert packet.buttons.right_b is False


@pytest.mark.parametrize('field', ['left', 'right', 'head'])
def test_tracking_flags_must_be_strict_booleans(field):
    data = packet_dict()
    data['tracking'][field] = 1
    with pytest.raises(PacketError, match=f'tracking.{field}'):
        parse_packet(encode(data))


@pytest.mark.parametrize('field', ['left_x', 'right_b'])
def test_action_buttons_must_be_strict_booleans(field):
    data = packet_dict()
    data['buttons'][field] = 1
    with pytest.raises(PacketError, match=f'buttons.{field}'):
        parse_packet(encode(data))
