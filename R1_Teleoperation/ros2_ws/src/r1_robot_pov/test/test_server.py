import asyncio
from dataclasses import replace
from types import SimpleNamespace

from aiohttp import web
import numpy as np
import pytest

from r1_robot_pov.config import PROFILES, load_config
from r1_robot_pov.frame_pipeline import FrameHub
import r1_robot_pov.server as server_module
from r1_robot_pov.server import (
    RobotPovWebServer,
    SERVER_SHUTDOWN_TIMEOUT_SEC,
    _fit_frame,
    _adapt_frame_layout,
    _no_signal_frame,
    _output_dimensions,
    _normalize_client_tag,
    _normalize_preferred_codec,
    _offer_preferences,
    _ordered_video_codecs,
    _set_video_bandwidth,
)


def test_server_uses_bounded_shutdown_timeout(monkeypatch, tmp_path):
    captured = {}

    class FakeRunner:
        def __init__(self, app, **kwargs):
            captured['app'] = app
            captured['runner_kwargs'] = kwargs

        async def setup(self):
            captured['runner_setup'] = True

        async def cleanup(self):
            captured['runner_cleanup'] = True

    class FakeSite:
        def __init__(self, runner, host, port, **kwargs):
            captured['site'] = (runner, host, port, kwargs)

        async def start(self):
            captured['site_started'] = True

    monkeypatch.setattr(server_module.web, 'AppRunner', FakeRunner)
    monkeypatch.setattr(server_module.web, 'TCPSite', FakeSite)
    config = replace(load_config(environ={}), log_dir=str(tmp_path))
    server = RobotPovWebServer(config, FrameHub())

    async def run_server():
        await server.start()
        await server.close()

    asyncio.run(run_server())

    assert captured['runner_kwargs'] == {
        'access_log': None,
        'shutdown_timeout': SERVER_SHUTDOWN_TIMEOUT_SEC,
    }
    assert captured['runner_setup'] is True
    assert captured['site_started'] is True
    assert captured['runner_cleanup'] is True


def test_fit_frame_letterboxes_without_distortion():
    source = np.full((100, 100, 3), 255, dtype=np.uint8)
    output = _fit_frame(source, PROFILES['bad-wifi'])
    assert output.shape == (240, 640, 3)
    assert np.all(output[:, :190] == 0)
    assert np.all(output[:, 200:440] == 255)


def test_no_signal_frame_has_requested_profile_size():
    frame = _no_signal_frame(PROFILES['balanced'])
    assert frame.shape == (360, 960, 3)
    assert np.any(frame != 0)


def test_mono_frame_keeps_unitree_camera_aspect_without_letterboxing():
    source = np.full((360, 640, 3), 255, dtype=np.uint8)

    output = _fit_frame(source, PROFILES['balanced'], layout='mono')

    assert output.shape == (360, 640, 3)
    assert np.all(output == 255)


def test_mono_no_signal_frame_matches_live_transport_dimensions():
    profile = PROFILES['low-latency']
    live = _fit_frame(
        np.zeros((720, 1280, 3), dtype=np.uint8),
        profile,
        layout='mono',
    )
    no_signal = _no_signal_frame(profile, layout='mono')

    assert live.shape == (360, 640, 3)
    assert no_signal.shape == live.shape


def test_adapt_frame_layout_crops_stereo_for_mono_clients():
    left = np.full((4, 3, 3), 11, dtype=np.uint8)
    right = np.full((4, 3, 3), 22, dtype=np.uint8)
    stereo = np.concatenate((left, right), axis=1)

    mono = _adapt_frame_layout(stereo, 'stereo', 'mono')

    assert mono.shape == left.shape
    assert np.all(mono == 11)


def test_adapt_frame_layout_duplicates_mono_for_stereo_clients():
    mono = np.full((4, 3, 3), 17, dtype=np.uint8)

    stereo = _adapt_frame_layout(mono, 'mono', 'stereo')

    assert stereo.shape == (4, 6, 3)
    assert np.all(stereo[:, :3] == 17)
    assert np.all(stereo[:, 3:] == 17)


def test_adapt_frame_layout_duplicates_mono_for_top_bottom_clients():
    mono = np.full((3, 4, 3), 17, dtype=np.uint8)

    top_bottom = _adapt_frame_layout(mono, 'mono', 'top-bottom')

    assert top_bottom.shape == (6, 4, 3)
    assert np.all(top_bottom[:3] == 17)
    assert np.all(top_bottom[3:] == 17)


def test_adapt_frame_layout_converts_stereo_to_top_bottom():
    left = np.full((3, 4, 3), 11, dtype=np.uint8)
    right = np.full((3, 4, 3), 22, dtype=np.uint8)
    stereo = np.concatenate((left, right), axis=1)

    top_bottom = _adapt_frame_layout(stereo, 'stereo', 'top-bottom')

    assert top_bottom.shape == (6, 4, 3)
    assert np.all(top_bottom[:3] == 11)
    assert np.all(top_bottom[3:] == 22)


def test_adapt_frame_layout_rejects_unknown_layout():
    frame = np.zeros((2, 2, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match='unsupported output video layout'):
        _adapt_frame_layout(frame, 'stereo', 'diagonal')


def test_output_dimensions_reject_unknown_layout():
    width, height = _output_dimensions(PROFILES['balanced'], 'top-bottom')
    assert (width, height) == (640, 720)
    with pytest.raises(ValueError, match='unsupported video layout'):
        _output_dimensions(PROFILES['balanced'], 'diagonal')


def test_top_bottom_transport_keeps_stereo_hub_source_layout():
    config = load_config(environ={
        'ROBOT_POV_SOURCE': 'mock',
        'ROBOT_POV_LAYOUT': 'top-bottom',
    })
    server = RobotPovWebServer(config, FrameHub())
    assert server.source_layout == 'stereo'


def test_video_bandwidth_is_inserted_and_replaces_old_limits():
    source = (
        'v=0\r\nm=audio 9 UDP/TLS/RTP/SAVPF 111\r\n'
        'c=IN IP4 0.0.0.0\r\nm=video 9 UDP/TLS/RTP/SAVPF 96\r\n'
        'c=IN IP4 0.0.0.0\r\nb=AS:9999\r\na=sendonly\r\n'
    )
    output = _set_video_bandwidth(source, 550)
    assert 'm=audio 9' in output
    assert 'b=AS:550\r\nb=TIAS:550000\r\na=sendonly' in output
    assert 'b=AS:9999' not in output


def test_native_codec_names_are_normalized_through_an_allowlist():
    assert _normalize_preferred_codec(None) is None
    assert _normalize_preferred_codec('  VP8 ') == 'vp8'
    assert _normalize_preferred_codec('video/H.264') == 'h264'
    with pytest.raises(ValueError, match='preferredCodec'):
        _normalize_preferred_codec('av1')
    with pytest.raises(ValueError, match='string'):
        _normalize_preferred_codec({'codec': 'h264'})


def test_h264_preference_orders_h264_before_vp8_and_removes_rtx():
    codecs = [
        SimpleNamespace(mimeType='video/VP8'),
        SimpleNamespace(mimeType='video/rtx'),
        SimpleNamespace(mimeType='video/H264'),
        SimpleNamespace(mimeType='video/VP9'),
        SimpleNamespace(mimeType='video/H264'),
    ]
    ordered = _ordered_video_codecs(codecs, 'h264')
    assert [codec.mimeType for codec in ordered] == [
        'video/H264', 'video/H264', 'video/VP8', 'video/VP9',
    ]


def test_offer_without_native_fields_preserves_browser_vp8_default():
    body = {'type': 'offer', 'sdp': 'v=0', 'profile': 'balanced'}
    assert _offer_preferences(body) == (None, None)
    codecs = [
        SimpleNamespace(mimeType='video/H264'),
        SimpleNamespace(mimeType='video/VP8'),
        SimpleNamespace(mimeType='video/VP9'),
    ]
    ordered = _ordered_video_codecs(codecs)
    assert [codec.mimeType for codec in ordered] == [
        'video/VP8', 'video/H264', 'video/VP9',
    ]


def test_client_tag_is_diagnostic_only_and_cannot_select_a_codec():
    preferred, client = _offer_preferences({'clientTag': 'unity-r1:h264'})
    assert preferred is None
    assert client == 'unity-r1:h264'
    codecs = [
        SimpleNamespace(mimeType='video/H264'),
        SimpleNamespace(mimeType='video/VP8'),
    ]
    assert _ordered_video_codecs(codecs, preferred)[0].mimeType == 'video/VP8'
    with pytest.raises(ValueError, match='unsupported characters'):
        _normalize_client_tag('../../admin\n')


def test_offer_rejects_invalid_codec_before_allocating_a_peer(monkeypatch):
    config = load_config(environ={})
    server = RobotPovWebServer(config, FrameHub())
    monkeypatch.setattr(server_module, 'WEBRTC_AVAILABLE', True)

    async def payload():
        return {
            'type': 'offer',
            'sdp': 'v=0',
            'preferredCodec': 'mpeg2',
            'client': 'unity-native',
        }

    request = SimpleNamespace(json=payload, remote='127.0.0.1')
    with pytest.raises(web.HTTPBadRequest) as error:
        asyncio.run(server._offer(request))
    assert 'preferredCodec' in error.value.text
    assert server.peer_connections == set()
    assert server.metrics['webrtc_connections'] == 0


def test_server_exposes_no_control_routes():
    config = load_config(environ={})
    server = RobotPovWebServer(config, FrameHub())
    paths = {resource.canonical for resource in server.app.router.resources()}
    assert '/healthz' in paths
    assert '/stream.mjpg' in paths
    assert '/offer' in paths
    assert not any('cmd_vel' in path or 'trajectory' in path for path in paths)


def test_server_refuses_disabled_video_only_config():
    config = load_config(environ={})
    unsafe = replace(config, video_only=False)
    try:
        RobotPovWebServer(unsafe, FrameHub())
    except ValueError as exc:
        assert 'video-only' in str(exc)
    else:
        raise AssertionError('server accepted a non-video-only config')


def test_client_metrics_are_sanitized_and_bounded():
    config = load_config(environ={})
    server = RobotPovWebServer(config, FrameHub())
    server.client_metrics.update({
        f'old-{index}': {'updated_unix': 0.0} for index in range(128)
    })

    async def payload():
        return {
            'clientId': 'new-client',
            'latencyMs': '17.5',
            'reconnects': 'not-a-number',
        }

    request = SimpleNamespace(json=payload, remote='127.0.0.1')
    response = asyncio.run(server._client_metrics(request))

    assert response.status == 200
    assert len(server.client_metrics) == 128
    assert 'old-0' not in server.client_metrics
    assert server.client_metrics['new-client']['latency_ms'] == 17.5
    assert server.client_metrics['new-client']['reconnects'] == 0
