"""Deterministic tests for Robot POV exhibition preflight policy."""

from datetime import datetime, timezone
import json
from types import SimpleNamespace

import pytest

from r1_robot_pov.preflight import (
    CheckResult,
    HttpResponse,
    NetworkInterfaceStatus,
    PathStatus,
    PreflightReport,
    Status,
    StorageStatus,
    SystemAdapter,
    classify_fps,
    classify_upper_bound,
    extract_live_metrics,
    parse_env_text,
    redact_url,
    resolve_settings,
    run_preflight,
    usable_ipv4,
)


class FakeAdapter(SystemAdapter):
    """Side-effect-free adapter with configurable probe results."""

    def __init__(self):
        self.addresses = ['192.168.50.10']
        self.port_free = True
        self.http_available = True
        self.ready = True
        self.status_payload = {
            'video_only': True,
            'source': {'connected': True, 'fps': 24.0, 'frame_age_ms': 30.0},
            'latest_client': {'latency_ms': 70.0, 'reconnects': 0},
        }
        self.resolved = ['192.168.50.10']
        self.ping_ok = True
        self.modules = {
            'aiohttp', 'aiortc', 'av', 'cv2', 'numpy', 'qrcode',
            'rclpy', 'sensor_msgs', 'unitree_sdk2py',
        }
        self.interface = NetworkInterfaceStatus(
            True, True, ('192.168.123.162',)
        )
        self.paths = {}
        self.storage = StorageStatus(True, 2 * 1024**3, '/tmp')
        self.topic_ok = True
        self.tcp_ok = True
        self.sleeps = []

    def local_ipv4_addresses(self):
        return list(self.addresses)

    def port_available(self, _host, _port):
        return self.port_free

    def network_interface_status(self, _name):
        return self.interface

    def http_get(self, url, _timeout_s):
        if not self.http_available:
            raise ConnectionError('connection refused')
        if url.endswith('/readyz'):
            return HttpResponse(
                200 if self.ready else 503,
                {'ready': self.ready},
            )
        if url.endswith('/api/status'):
            return HttpResponse(200, self.status_payload)
        if url.endswith('/healthz'):
            return HttpResponse(200, {
                'ok': True,
                'service': 'r1_robot_pov',
                'video_only': True,
            })
        return HttpResponse(404, {'ok': False})

    def resolve_ipv4(self, _host):
        return list(self.resolved)

    def ping(self, _host, _timeout_s):
        return self.ping_ok

    def module_available(self, module):
        return module in self.modules

    def inspect_path(self, path):
        return self.paths.get(path, PathStatus(False, False, False))

    def storage_status(self, _path):
        return self.storage

    def topic_has_publisher(self, topic, _timeout_s):
        return self.topic_ok, f'topic={topic}'

    def tcp_reachable(self, _host, _port, _timeout_s):
        return self.tcp_ok

    def sleep(self, seconds):
        self.sleeps.append(seconds)

    def utc_now(self):
        return datetime(2026, 9, 11, 12, 0, tzinfo=timezone.utc)


def _config(**overrides):
    values = {
        'safety': {'video_only': True},
        'source': {'type': 'mock', 'layout': 'stereo'},
        'video': {'profile': 'balanced', 'transport': 'mjpeg', 'fps': 24.0},
        'network': {
            'bind_host': '0.0.0.0',
            'http_port': 8080,
            'advertise_host': 'auto',
            'mdns_name': 'robot-pov.local',
        },
        'logging': {'directory': 'logs/robot_pov'},
        'robot': {'ip': '192.168.50.20'},
    }
    for key, value in overrides.items():
        values[key] = value
    return values


def _checks(report):
    return {item.check_id: item for item in report.checks}


def test_redact_url_hides_user_info_and_secret_query_values():
    redacted = redact_url(
        'rtsp://alice:secret@192.168.50.20:8554/live?token=abc&channel=2'
    )
    assert redacted == (
        'rtsp://***:***@192.168.50.20:8554/live?'
        'token=%2A%2A%2A&channel=2'
    )
    assert 'alice' not in redacted
    assert 'secret' not in redacted
    assert 'abc' not in redacted


def test_parse_env_text_is_data_only_and_rejects_invalid_lines():
    values = parse_env_text(
        "# local profile\nexport ROBOT_POV_PORT='8080'\n"
        'ROBOT_POV_LEFT_SOURCE=$(touch never-run)\n'
    )
    assert values['ROBOT_POV_PORT'] == '8080'
    assert values['ROBOT_POV_LEFT_SOURCE'] == '$(touch never-run)'
    with pytest.raises(ValueError, match='missing ='):
        parse_env_text('not-an-assignment')


def test_resolve_settings_accepts_pov_config_shaped_object():
    config = SimpleNamespace(
        video_only=True,
        source='ros',
        layout='stereo',
        transport='auto',
        profile='balanced',
        bind_host='0.0.0.0',
        port=8080,
        advertise_host='auto',
        mdns_name='robot-pov.local',
        scheme='http',
        log_dir='logs/robot_pov',
        robot_ip='',
        left_source='/dev/video0',
        right_source='',
        left_topic='/left/image_raw',
        right_topic='/right/image_raw',
        stale_after_sec=0.6,
        video_profile=SimpleNamespace(fps=24.0),
    )
    settings = resolve_settings(config)
    assert settings.ros_topics == ('/left/image_raw', '/right/image_raw')
    assert settings.target_fps == 24.0
    assert settings.fail_frame_age_ms == 600.0


def test_usable_ipv4_filters_loopback_link_local_and_invalid_values():
    assert usable_ipv4([
        '127.0.0.1', '169.254.1.2', '192.168.50.20/24', 'bad',
        '10.0.0.2', '192.168.50.20',
    ]) == ['10.0.0.2', '192.168.50.20']


def test_metric_threshold_helpers_have_stable_boundaries():
    assert classify_fps(24.0, 30.0) == Status.OK
    assert classify_fps(18.0, 30.0) == Status.WARN
    assert classify_fps(17.9, 30.0) == Status.FAIL
    assert classify_fps(None, 30.0) == Status.WARN
    assert classify_upper_bound(250.0, 250.0, 750.0) == Status.OK
    assert classify_upper_bound(251.0, 250.0, 750.0) == Status.WARN
    assert classify_upper_bound(751.0, 250.0, 750.0) == Status.FAIL


def test_extract_live_metrics_accepts_current_server_status_shape():
    metrics = extract_live_metrics({
        'source': {'left_fps': 27.0, 'right_fps': 25.0,
                   'frame_age_ms': 42.0},
        'latest_client': {'latency_ms': 83.0, 'reconnects': 2},
    })
    assert metrics == {
        'fps': 25.0,
        'frame_age_ms': 42.0,
        'latency_ms': 83.0,
        'reconnect_count': 2.0,
    }


def test_safe_mock_configuration_passes_without_writing_files():
    report = run_preflight(_config(), adapter=FakeAdapter())
    assert report.overall == Status.OK
    assert _checks(report)['safety.video_only'].status == Status.OK
    assert report.viewer_urls == (
        'http://robot-pov.local:8080/',
        'http://192.168.50.10:8080/',
    )


def test_video_only_false_is_always_a_failure():
    config = _config(safety={'video_only': False})
    report = run_preflight(config, adapter=FakeAdapter())
    assert report.overall == Status.FAIL
    assert _checks(report)['safety.video_only'].status == Status.FAIL


def test_missing_robot_address_is_warning_in_video_only_mode():
    report = run_preflight(_config(robot={'ip': ''}), adapter=FakeAdapter())
    assert _checks(report)['robot.network'].status == Status.WARN
    assert report.overall == Status.WARN


def test_usb_stereo_requires_distinct_readable_character_devices():
    adapter = FakeAdapter()
    adapter.paths = {
        '/dev/video2': PathStatus(True, True, True),
        '/dev/video3': PathStatus(True, True, True),
    }
    config = _config(source={
        'type': 'usb', 'layout': 'stereo',
        'left_source': '/dev/video2', 'right_source': '/dev/video3',
    })
    report = run_preflight(config, adapter=adapter)
    assert _checks(report)['source'].status == Status.OK

    config['source']['right_source'] = '/dev/video2'
    report = run_preflight(config, adapter=adapter)
    assert _checks(report)['source'].status == Status.FAIL


def test_unitree_source_checks_read_only_interface_and_sdk():
    adapter = FakeAdapter()
    config = _config(source={
        'type': 'unitree',
        'layout': 'mono',
        'interface': 'enxrobot0',
        'fps': 15.0,
    })
    report = run_preflight(config, adapter=adapter)
    assert _checks(report)['source'].status == Status.OK
    assert resolve_settings(config).target_fps == 15.0

    adapter.interface = NetworkInterfaceStatus(True, False)
    report = run_preflight(config, adapter=adapter)
    assert _checks(report)['source'].status == Status.FAIL

    adapter.interface = NetworkInterfaceStatus(
        True, True, ('192.168.123.162',)
    )
    adapter.modules.remove('unitree_sdk2py')
    report = run_preflight(config, adapter=adapter)
    assert _checks(report)['dependencies.python'].status == Status.FAIL


def test_rtsp_credentials_are_redacted_from_report_json():
    config = _config(source={
        'type': 'rtsp', 'layout': 'mono',
        'left_source': (
            'rtsp://operator:secret@192.168.50.30/live?token=hidden'
        ),
    })
    report = run_preflight(config, adapter=FakeAdapter())
    serialized = json.dumps(report.to_dict())
    assert 'operator' not in serialized
    assert 'secret' not in serialized
    assert 'hidden' not in serialized
    assert _checks(report)['source'].status == Status.OK


def test_live_viewer_checks_health_lan_fps_frame_age_and_latency():
    adapter = FakeAdapter()
    adapter.port_free = False
    report = run_preflight(
        _config(), live=True, duration_s=1.0, adapter=adapter
    )
    checks = _checks(report)
    assert checks['network.port'].status == Status.OK
    assert checks['viewer.healthz'].status == Status.OK
    assert checks['viewer.readyz'].status == Status.OK
    assert checks['viewer.lan_url'].status == Status.OK
    assert checks['viewer.fps'].status == Status.OK
    assert checks['viewer.frame_age'].status == Status.OK
    assert checks['viewer.latency'].status == Status.OK
    assert adapter.sleeps == [0.5]
    assert report.overall == Status.OK


def test_live_viewer_fails_on_slow_stale_stream():
    adapter = FakeAdapter()
    adapter.port_free = False
    adapter.status_payload = {
        'video_only': True,
        'source': {'connected': True, 'fps': 5.0, 'frame_age_ms': 900.0}
    }
    report = run_preflight(
        _config(), live=True, duration_s=0.0, adapter=adapter
    )
    checks = _checks(report)
    assert checks['viewer.fps'].status == Status.FAIL
    assert checks['viewer.frame_age'].status == Status.FAIL
    assert report.overall == Status.FAIL


def test_occupied_unknown_port_fails_even_without_live_mode():
    adapter = FakeAdapter()
    adapter.port_free = False
    adapter.http_available = False
    report = run_preflight(_config(), adapter=adapter)
    assert _checks(report)['network.port'].status == Status.FAIL


def test_missing_required_dependency_and_low_storage_fail():
    adapter = FakeAdapter()
    adapter.modules.remove('aiohttp')
    adapter.storage = StorageStatus(True, 10 * 1024**2, '/tmp')
    report = run_preflight(_config(), adapter=adapter)
    checks = _checks(report)
    assert checks['dependencies.python'].status == Status.FAIL
    assert checks['logging.storage'].status == Status.FAIL


def test_report_serialization_redacts_nested_secrets():
    report = PreflightReport(
        '2026-09-11T12:00:00Z',
        (CheckResult(
            'source', Status.OK, 'configured',
            {'url': 'rtsp://u:p@camera/live', 'password': 'do-not-log'},
        ),),
    )
    serialized = json.dumps(report.to_dict())
    assert 'do-not-log' not in serialized
    assert 'rtsp://u:p@' not in serialized
