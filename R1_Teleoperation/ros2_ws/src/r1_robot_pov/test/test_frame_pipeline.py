"""Tests for Robot POV frame processing and latest-frame semantics."""

from types import SimpleNamespace
import threading
import time

import numpy as np
import pytest

from r1_robot_pov.frame_pipeline import FrameHub, FrameProcessor, PROFILES
from r1_robot_pov.sources import MockSource, ros_image_to_bgr


def _image_message(width, height, step, encoding, payload):
    return SimpleNamespace(
        width=width,
        height=height,
        step=step,
        encoding=encoding,
        data=bytes(payload),
    )


def test_ros_rgb8_with_row_padding_converts_to_bgr():
    message = _image_message(
        width=2,
        height=2,
        step=8,
        encoding='rgb8',
        payload=[
            255, 0, 0, 0, 255, 0, 99, 99,
            0, 0, 255, 10, 20, 30, 88, 88,
        ],
    )

    output = ros_image_to_bgr(message)

    assert output.flags.c_contiguous
    assert output.tolist() == [
        [[0, 0, 255], [0, 255, 0]],
        [[255, 0, 0], [30, 20, 10]],
    ]


def test_ros_mono_conversion_and_invalid_payload():
    message = _image_message(2, 1, 2, 'mono8', [4, 9])
    output = ros_image_to_bgr(message)
    assert output.tolist() == [[[4, 4, 4], [9, 9, 9]]]

    short = _image_message(2, 2, 6, 'bgr8', [0] * 11)
    with pytest.raises(ValueError, match='payload too short'):
        ros_image_to_bgr(short)


def test_stereo_side_by_side_is_symmetric_and_can_swap_eyes():
    left = np.full((2, 3, 3), (10, 20, 30), dtype=np.uint8)
    right = np.full((2, 3, 3), (40, 50, 60), dtype=np.uint8)

    normal = FrameProcessor().process(left, right)
    swapped = FrameProcessor(swap_left_right=True).process(left, right)

    assert normal.shape == (2, 6, 3)
    np.testing.assert_array_equal(normal[:, :3], left)
    np.testing.assert_array_equal(normal[:, 3:], right)
    np.testing.assert_array_equal(swapped[:, :3], right)
    np.testing.assert_array_equal(swapped[:, 3:], left)


def test_compose_preserves_eye_aspect_ratio_at_common_height():
    left = np.zeros((4, 8, 3), dtype=np.uint8)
    right = np.ones((2, 3, 3), dtype=np.uint8)

    output = FrameProcessor.compose_side_by_side(left, right)

    # Left becomes 4x2 and right remains 3x2.
    assert output.shape == (2, 7, 3)


def test_crop_then_clockwise_rotate_then_flip_horizontal():
    image = np.zeros((3, 4, 3), dtype=np.uint8)
    image[:, :, 0] = np.arange(12, dtype=np.uint8).reshape(3, 4)
    processor = FrameProcessor(
        crop_pixels=(1, 0, 0, 1),
        rotation=90,
        flip_horizontal=True,
    )

    output = processor.process(image)

    assert output.shape == (3, 2, 3)
    np.testing.assert_array_equal(
        output[:, :, 0],
        np.array([[1, 5], [2, 6], [3, 7]], dtype=np.uint8),
    )


def test_per_eye_transform_and_profile_resize():
    left = np.zeros((3, 4, 3), dtype=np.uint8)
    right = np.zeros((3, 4, 3), dtype=np.uint8)
    left[0, 0] = (1, 2, 3)
    right[-1, -1] = (4, 5, 6)
    processor = FrameProcessor(
        profile={'width': 10, 'height': 4, 'mono_width': 5, 'mono_height': 4},
        flip_horizontal={'left': True, 'right': False},
        flip_vertical={'left': False, 'right': True},
    )

    stereo = processor.process(left, right)
    mono = processor.process(left)

    assert stereo.shape == (4, 10, 3)
    assert mono.shape == (4, 5, 3)


def test_named_profiles_accept_hyphen_and_underscore_alias():
    assert FrameProcessor('bad-wifi').profile == PROFILES['bad-wifi']
    assert FrameProcessor('low_latency').profile == PROFILES['low-latency']
    with pytest.raises(ValueError, match='unknown profile'):
        FrameProcessor('satellite')


def test_frame_hub_keeps_latest_and_reports_consumer_skips_and_drops():
    hub = FrameHub(stale_after=1.0)
    first = np.zeros((2, 3, 3), dtype=np.uint8)
    second = np.ones((2, 3, 3), dtype=np.uint8)

    assert hub.update(first, 'mock') == 1
    assert hub.update(second, 'mock', dropped=3) == 2
    snapshot = hub.snapshot()
    waited = hub.wait_for_new(after_sequence=0, timeout=0.0)

    assert snapshot.sequence == 2
    assert snapshot.frame.flags.writeable is False
    assert snapshot.frame[0, 0, 0] == 1
    assert waited.sequence == 2
    assert waited.skipped == 1
    status = hub.status()
    assert status['total_updates'] == 2
    assert status['reported_dropped_frames'] == 3
    assert status['sources']['mock']['frames'] == 2
    assert hub.wait_for_new(after_sequence=2, timeout=0.005) is None


def test_frame_hub_wait_wakes_for_new_frame():
    hub = FrameHub()

    def publish():
        hub.update(np.zeros((1, 1, 3), dtype=np.uint8), 'delayed')

    timer = threading.Timer(0.02, publish)
    timer.start()
    try:
        snapshot = hub.wait_for_new(0, timeout=0.5)
    finally:
        timer.join()
    assert snapshot is not None
    assert snapshot.sequence == 1


def test_frame_hub_stale_and_source_reconnect_metrics():
    hub = FrameHub(stale_after=0.01)
    hub.mark_source_connected('camera')
    hub.update(np.zeros((1, 1, 3), dtype=np.uint8), 'camera')
    hub.mark_source_error('camera', 'temporary decode error')
    hub.mark_source_disconnected('camera', 'stream closed')
    hub.mark_source_reconnect('camera')
    hub.mark_source_connected('camera', reconnected=True)

    time.sleep(0.02)
    status = hub.status()

    source = status['sources']['camera']
    assert status['stale'] is True
    assert status['connected'] is False
    assert source['connected'] is True
    assert source['errors'] == 2
    assert source['reconnects'] == 1
    assert source['successful_reconnects'] == 1
    assert source['last_error'] == 'stream closed'


def test_mock_source_renders_mono_and_stereo_without_thread():
    hub = FrameHub()
    mono = MockSource(hub, width=80, height=60, stereo=False)
    stereo = MockSource(hub, width=80, height=60, stereo=True)

    mono_frame = mono.render_frame(elapsed_s=0.25)
    stereo_frame = stereo.render_frame(elapsed_s=0.25)

    assert mono_frame.shape == (60, 80, 3)
    assert stereo_frame.shape == (60, 160, 3)
    assert np.any(stereo_frame[:, :80] != stereo_frame[:, 80:])
