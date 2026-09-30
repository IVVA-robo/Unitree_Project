import pytest

from r1_robot_pov.config import load_config


def test_dotenv_and_process_environment_precedence(tmp_path):
    path = tmp_path / 'viewer.env'
    path.write_text(
        'ROBOT_POV_SOURCE=mock\nROBOT_POV_PROFILE=bad-wifi\n',
        encoding='utf-8',
    )
    config = load_config(
        str(path),
        environ={'ROBOT_POV_PROFILE': 'low-latency'},
    )
    assert config.source == 'mock'
    assert config.profile == 'low-latency'


def test_video_only_cannot_be_disabled():
    with pytest.raises(ValueError, match='must remain true'):
        load_config(environ={'ROBOT_POV_VIDEO_ONLY': 'false'})


def test_discovery_responder_can_be_disabled_for_live_bridge():
    offline = load_config(environ={})
    live = load_config(environ={'ROBOT_POV_DISCOVERY_ENABLED': 'false'})
    assert offline.discovery_enabled is True
    assert live.discovery_enabled is False


def test_rtsp_credentials_are_redacted():
    config = load_config(environ={
        'ROBOT_POV_SOURCE': 'rtsp',
        'ROBOT_POV_LAYOUT': 'mono',
        'ROBOT_POV_LEFT_SOURCE': (
            'rtsp://operator:secret@192.168.1.20/live?token=private&camera=2'
        ),
    })
    redacted = config.redacted_dict()
    assert 'secret' not in str(redacted)
    assert 'private' not in str(redacted)
    assert '***:***@192.168.1.20' in redacted['left_source']
    assert 'token=%2A%2A%2A' in redacted['left_source']
    assert 'camera=2' in redacted['left_source']


def test_public_config_does_not_expose_camera_source():
    config = load_config(environ={
        'ROBOT_POV_SOURCE': 'rtsp',
        'ROBOT_POV_LAYOUT': 'mono',
        'ROBOT_POV_LEFT_SOURCE': 'rtsp://operator:secret@camera/live',
    })
    assert 'left_source' not in config.public_dict()
    assert 'secret' not in str(config.public_dict())


def test_public_config_describes_stereo_fallback_for_unitree_source():
    config = load_config(environ={
        'ROBOT_POV_SOURCE': 'unitree',
        'ROBOT_POV_LAYOUT': 'mono',
    })
    public = config.public_dict()
    assert public['stereoFallback'] is True
    assert public['stereoAvailable'] is False
    assert public['stereoStatus'] == 'mono-to-both-eyes fallback'


def test_rtsp_source_rejects_non_rtsp_and_malformed_urls():
    base = {
        'ROBOT_POV_SOURCE': 'rtsp',
        'ROBOT_POV_LAYOUT': 'mono',
    }
    with pytest.raises(ValueError, match='rtsp://'):
        load_config(environ={
            **base,
            'ROBOT_POV_LEFT_SOURCE': 'http://camera/live',
        })
    with pytest.raises(ValueError, match='valid RTSP'):
        load_config(environ={
            **base,
            'ROBOT_POV_LEFT_SOURCE': 'rtsp://camera:not-a-port/live',
        })


def test_unitree_source_requires_mono_and_valid_interface():
    config = load_config(environ={
        'ROBOT_POV_SOURCE': 'unitree',
        'ROBOT_POV_LAYOUT': 'mono',
        'ROBOT_POV_UNITREE_INTERFACE': 'enxrobot0',
        'ROBOT_POV_UNITREE_TIMEOUT_SEC': '2.5',
        'ROBOT_POV_UNITREE_FPS': '14.0',
    })
    assert config.unitree_interface == 'enxrobot0'
    assert config.unitree_timeout_sec == 2.5
    assert config.unitree_fps == 14.0

    with pytest.raises(ValueError, match='mono layout'):
        load_config(environ={
            'ROBOT_POV_SOURCE': 'unitree',
            'ROBOT_POV_LAYOUT': 'stereo',
        })
    with pytest.raises(ValueError, match='valid interface'):
        load_config(environ={
            'ROBOT_POV_SOURCE': 'unitree',
            'ROBOT_POV_LAYOUT': 'mono',
            'ROBOT_POV_UNITREE_INTERFACE': '../robot',
        })
