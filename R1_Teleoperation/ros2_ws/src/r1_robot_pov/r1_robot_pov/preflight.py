"""
Offline-first exhibition preflight checks for the Robot POV service.

The module deliberately uses only the Python standard library.  Runtime and
source probes are behind :class:`SystemAdapter`, which keeps the policy easy to
unit-test and lets camera transports provide a richer probe later.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
import importlib.util
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import shutil
import socket
import stat
import statistics
import subprocess
import sys
import time
from typing import Any, Iterable, Mapping, Sequence
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit
from urllib.request import Request, urlopen


SUPPORTED_SOURCES = frozenset({'mock', 'ros', 'usb', 'rtsp', 'unitree'})
SUPPORTED_PROFILES = frozenset(
    {'high', 'balanced', 'low-latency', 'bad-wifi'}
)
DEFAULT_TARGET_FPS = {
    'high': 30.0,
    'balanced': 24.0,
    'low-latency': 30.0,
    'bad-wifi': 15.0,
}
_SENSITIVE_QUERY_KEYS = frozenset(
    {
        'access_token', 'api_key', 'auth', 'key', 'pass', 'password',
        'secret', 'token',
    }
)
_MISSING = object()


class Status(str, Enum):
    """Severity of one preflight check."""

    OK = 'OK'
    WARN = 'WARN'
    FAIL = 'FAIL'


_STATUS_RANK = {Status.OK: 0, Status.WARN: 1, Status.FAIL: 2}


@dataclass(frozen=True)
class CheckResult:
    """One stable, machine-readable preflight result."""

    check_id: str
    status: Status
    message: str
    details: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Return a redacted JSON-compatible representation."""
        return {
            'id': self.check_id,
            'status': self.status.value,
            'message': self.message,
            'details': sanitize_for_log(dict(self.details)),
        }


@dataclass(frozen=True)
class PreflightReport:
    """Complete preflight result returned by :func:`run_preflight`."""

    generated_at: str
    checks: tuple[CheckResult, ...]
    viewer_urls: tuple[str, ...] = ()
    sample_duration_s: float = 0.0

    @property
    def overall(self) -> Status:
        """Return the worst status in the report."""
        return max(
            (item.status for item in self.checks),
            key=lambda value: _STATUS_RANK[value],
            default=Status.FAIL,
        )

    def to_dict(self) -> dict[str, Any]:
        """Return a redacted JSON-compatible representation."""
        return {
            'schema_version': 1,
            'generated_at': self.generated_at,
            'overall': self.overall.value,
            'sample_duration_s': self.sample_duration_s,
            'viewer_urls': [redact_url(url) for url in self.viewer_urls],
            'checks': [item.to_dict() for item in self.checks],
        }

    def render_human(self) -> str:
        """Render a compact terminal report."""
        lines = [
            f'Robot POV preflight: {self.overall.value}',
            *(f'[{item.status.value}] {item.check_id}: {item.message}'
              for item in self.checks),
        ]
        if self.viewer_urls:
            lines.append('Viewer URLs:')
            lines.extend(f'  {redact_url(url)}' for url in self.viewer_urls)
        return '\n'.join(lines)


@dataclass(frozen=True)
class HttpResponse:
    """Small transport-neutral HTTP response used by live checks."""

    status: int
    payload: Any = None
    text: str = ''


@dataclass(frozen=True)
class PathStatus:
    """Basic source-path metadata."""

    exists: bool
    readable: bool
    character_device: bool


@dataclass(frozen=True)
class NetworkInterfaceStatus:
    """Read-only state of one local network interface."""

    exists: bool
    up: bool
    ipv4: tuple[str, ...] = ()


@dataclass(frozen=True)
class StorageStatus:
    """Writability and free space for the configured log directory."""

    writable: bool
    free_bytes: int
    checked_path: str


@dataclass(frozen=True)
class Settings:
    """Normalized subset of configuration needed by preflight."""

    video_only: bool
    source_type: str
    source_layout: str
    transport: str
    profile: str
    bind_host: str
    port: int
    scheme: str
    advertise_host: str
    mdns_name: str
    health_base_url: str
    log_dir: Path
    min_free_disk_mb: float
    required_modules: tuple[str, ...]
    optional_modules: tuple[str, ...]
    robot_ip: str
    unitree_interface: str
    usb_path: str
    usb_right_path: str
    rtsp_url: str
    rtsp_right_url: str
    ros_topics: tuple[str, ...]
    target_fps: float
    ok_fps_ratio: float
    warn_fps_ratio: float
    warn_frame_age_ms: float
    fail_frame_age_ms: float
    warn_latency_ms: float
    fail_latency_ms: float
    http_timeout_s: float


class SystemAdapter:
    """Default read-only OS/network probes used by the preflight runner."""

    def local_ipv4_addresses(self) -> list[str]:
        """List IPv4 addresses assigned to interfaces that are up."""
        try:
            completed = subprocess.run(
                ['ip', '-j', '-4', 'address', 'show', 'up'],
                check=False,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
            if completed.returncode == 0:
                data = json.loads(completed.stdout)
                addresses = []
                for interface in data:
                    for item in interface.get('addr_info', []):
                        if item.get('family') == 'inet' and item.get('local'):
                            addresses.append(str(item['local']))
                return addresses
        except (
            FileNotFoundError, json.JSONDecodeError, subprocess.TimeoutExpired
        ):
            pass
        try:
            return list(socket.gethostbyname_ex(socket.gethostname())[2])
        except socket.gaierror:
            return []

    def port_available(self, host: str, port: int) -> bool:
        """Return true when an IPv4 TCP socket can bind to host and port."""
        probe_host = '0.0.0.0' if host in ('', '*') else host
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind((probe_host, port))
            return True
        except OSError:
            return False

    def network_interface_status(self, name: str) -> NetworkInterfaceStatus:
        """Inspect one interface without changing links or addresses."""
        try:
            completed = subprocess.run(
                ['ip', '-j', '-4', 'address', 'show', 'dev', str(name)],
                check=False,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
            if completed.returncode != 0:
                return NetworkInterfaceStatus(False, False)
            data = json.loads(completed.stdout)
            if not data:
                return NetworkInterfaceStatus(False, False)
            item = data[0]
            addresses = tuple(
                str(address['local'])
                for address in item.get('addr_info', [])
                if address.get('family') == 'inet' and address.get('local')
            )
            return NetworkInterfaceStatus(
                True,
                'UP' in item.get('flags', []),
                addresses,
            )
        except (
            FileNotFoundError,
            json.JSONDecodeError,
            subprocess.TimeoutExpired,
        ):
            return NetworkInterfaceStatus(False, False)

    def http_get(self, url: str, timeout_s: float) -> HttpResponse:
        """GET an endpoint and decode a JSON body when possible."""
        request = Request(
            url,
            headers={
                'Accept': 'application/json',
                'Cache-Control': 'no-cache',
            },
        )
        try:
            with urlopen(request, timeout=timeout_s) as response:
                body = response.read(1024 * 1024).decode(
                    'utf-8', errors='replace'
                )
                return HttpResponse(
                    int(response.status), _decode_json(body), body
                )
        except HTTPError as exc:
            body = exc.read(1024 * 1024).decode('utf-8', errors='replace')
            return HttpResponse(int(exc.code), _decode_json(body), body)
        except (URLError, OSError, TimeoutError) as exc:
            raise ConnectionError(str(exc)) from exc

    def resolve_ipv4(self, host: str) -> list[str]:
        """Resolve a hostname without requiring an external DNS service."""
        try:
            answers = socket.getaddrinfo(host, None, socket.AF_INET)
        except socket.gaierror:
            return []
        return sorted({str(item[4][0]) for item in answers})

    def ping(self, host: str, timeout_s: float) -> bool:
        """Send one bounded ICMP reachability probe."""
        try:
            completed = subprocess.run(
                [
                    'ping', '-c', '1', '-W',
                    str(max(1, math.ceil(timeout_s))), host,
                ],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=timeout_s + 1.0,
            )
            return completed.returncode == 0
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    def module_available(self, module: str) -> bool:
        """Return true when a Python import can be resolved."""
        try:
            return importlib.util.find_spec(module) is not None
        except (ImportError, ModuleNotFoundError, ValueError):
            return False

    def inspect_path(self, path: str) -> PathStatus:
        """Inspect a path without opening the camera device."""
        candidate = Path(path)
        try:
            mode = candidate.stat().st_mode
        except OSError:
            return PathStatus(False, False, False)
        return PathStatus(
            True,
            os.access(candidate, os.R_OK),
            stat.S_ISCHR(mode),
        )

    def storage_status(self, path: Path) -> StorageStatus:
        """Inspect the closest existing parent without creating files."""
        candidate = path.expanduser()
        while not candidate.exists() and candidate != candidate.parent:
            candidate = candidate.parent
        writable = candidate.is_dir() and os.access(
            candidate, os.W_OK | os.X_OK
        )
        try:
            free_bytes = int(shutil.disk_usage(candidate).free)
        except OSError:
            free_bytes = 0
            writable = False
        return StorageStatus(writable, free_bytes, str(candidate))

    def topic_has_publisher(
        self, topic: str, timeout_s: float
    ) -> tuple[bool, str]:
        """Probe a ROS topic with the CLI, without importing ROS."""
        try:
            completed = subprocess.run(
                ['ros2', 'topic', 'info', topic],
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout_s,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            return False, str(exc)
        output = (completed.stdout + completed.stderr).strip()
        match = re.search(r'Publisher count:\s*(\d+)', output)
        available = completed.returncode == 0 and match is not None
        available = available and int(match.group(1)) > 0
        return available, output[-500:]

    def tcp_reachable(self, host: str, port: int, timeout_s: float) -> bool:
        """Check that an RTSP TCP endpoint accepts a connection."""
        try:
            with socket.create_connection((host, port), timeout=timeout_s):
                return True
        except OSError:
            return False

    def sleep(self, seconds: float) -> None:
        """Sleep between live samples."""
        time.sleep(seconds)

    def utc_now(self) -> datetime:
        """Return current UTC time."""
        return datetime.now(timezone.utc)


def _decode_json(text: str) -> Any:
    try:
        return json.loads(text)
    except (json.JSONDecodeError, TypeError):
        return None


def redact_url(value: str) -> str:
    """Remove URL user-info and common secret query values from logs."""
    try:
        parts = urlsplit(value)
    except ValueError:
        return re.sub(r'(?<=//)[^/@\s]+@', '***@', value)
    if not parts.scheme or not parts.netloc:
        return value
    try:
        host = parts.hostname or ''
        if ':' in host and not host.startswith('['):
            host = f'[{host}]'
        port = f':{parts.port}' if parts.port is not None else ''
    except ValueError:
        return re.sub(r'(?<=//)[^/@\s]+@', '***@', value)
    user_info = ''
    if parts.username is not None:
        user_info = '***:***@' if parts.password is not None else '***@'
    query = []
    for key, item in parse_qsl(parts.query, keep_blank_values=True):
        hidden = '***' if key.lower() in _SENSITIVE_QUERY_KEYS else item
        query.append((key, hidden))
    return urlunsplit(
        (parts.scheme, f'{user_info}{host}{port}', parts.path,
         urlencode(query), parts.fragment)
    )


def sanitize_for_log(value: Any, key: str = '') -> Any:
    """Recursively redact credentials and convert values for JSON output."""
    lowered = key.lower()
    if lowered in _SENSITIVE_QUERY_KEYS:
        return '***'
    if isinstance(value, Mapping):
        return {
            str(item_key): sanitize_for_log(item_value, str(item_key))
            for item_key, item_value in value.items()
        }
    if isinstance(value, (list, tuple, set)):
        return [sanitize_for_log(item) for item in value]
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, str) and '://' in value:
        return redact_url(value)
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


def parse_env_text(text: str) -> dict[str, str]:
    """Parse a conservative dotenv subset without shell evaluation."""
    values = {}
    for line_number, raw_line in enumerate(text.splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith('#'):
            continue
        if line.startswith('export '):
            line = line[7:].lstrip()
        if '=' not in line:
            raise ValueError(f'invalid env line {line_number}: missing =')
        key, value = line.split('=', 1)
        key = key.strip()
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', key):
            raise ValueError(f'invalid env key on line {line_number}: {key!r}')
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "'\"":
            value = value[1:-1]
        values[key] = value
    return values


def load_env_file(path: Path) -> dict[str, str]:
    """Read a dotenv file as data, never as a shell program."""
    return parse_env_text(path.read_text(encoding='utf-8'))


def _lookup(config: Any, path: str) -> Any:
    current = config
    for part in path.split('.'):
        if isinstance(current, Mapping):
            if part in current:
                current = current[part]
                continue
            matches = [
                key for key in current
                if str(key).lower() == part.lower()
            ]
            if not matches:
                return _MISSING
            current = current[matches[0]]
            continue
        if hasattr(current, part):
            current = getattr(current, part)
            continue
        return _MISSING
    return current


def config_value(
    config: Any,
    path: str,
    *aliases: str,
    default: Any = None,
) -> Any:
    """Read nested dictionaries, objects, or flat environment mappings."""
    candidates = (path, *aliases)
    flat_candidates = []
    for candidate in candidates:
        flat = candidate.replace('.', '_').replace('-', '_').upper()
        flat_candidates.extend((flat, f'ROBOT_POV_{flat}'))
    for candidate in (*candidates, *flat_candidates):
        value = _lookup(config, candidate)
        if value is not _MISSING:
            return value
    return default


def parse_bool(value: Any) -> bool:
    """Parse a strict human-friendly boolean."""
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    normalized = str(value).strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'invalid boolean: {value!r}')


def _parse_list(value: Any) -> tuple[str, ...]:
    if value is None or value == '':
        return ()
    if isinstance(value, str):
        return tuple(item.strip() for item in value.split(',') if item.strip())
    if isinstance(value, Sequence):
        return tuple(str(item).strip() for item in value if str(item).strip())
    return (str(value).strip(),)


def normalize_source(value: Any) -> str:
    """Normalize common source aliases."""
    normalized = str(value).strip().lower().replace('_', '-')
    return {
        'test': 'mock',
        'gazebo': 'ros',
        'ros-topic': 'ros',
        'v4l2': 'usb',
    }.get(normalized, normalized)


def normalize_profile(value: Any) -> str:
    """Normalize profile spelling."""
    return str(value).strip().lower().replace('_', '-')


def resolve_settings(config: Any) -> Settings:
    """Normalize supported dict/object/env configuration fields."""
    video_only = parse_bool(config_value(
        config, 'safety.video_only', 'video_only', default=True
    ))
    source_type = normalize_source(config_value(
        config, 'source.type', 'source.kind', 'source_type', 'source',
        default='mock',
    ))
    profile = normalize_profile(config_value(
        config, 'video.profile', 'profile', default='balanced'
    ))
    source_layout = str(config_value(
        config, 'source.layout', 'source_layout', 'layout', default='mono'
    )).strip().lower()
    transport = str(config_value(
        config, 'video.transport', 'transport', default='auto'
    )).strip().lower()
    bind_host = str(config_value(
        config, 'network.bind_host', 'bind_host', 'host', default='0.0.0.0'
    )).strip()
    port = int(config_value(
        config, 'network.http_port', 'network.port', 'http_port', 'port',
        default=8080,
    ))
    if not 1 <= port <= 65535:
        raise ValueError(f'HTTP port out of range: {port}')
    tls_cert = str(config_value(
        config, 'network.tls_cert', 'tls_cert', default=''
    )).strip()
    tls_key = str(config_value(
        config, 'network.tls_key', 'tls_key', default=''
    )).strip()
    default_scheme = 'https' if tls_cert and tls_key else 'http'
    scheme = str(config_value(
        config, 'network.scheme', 'scheme', default=default_scheme
    )).strip().lower()
    if scheme not in ('http', 'https'):
        raise ValueError(f'unsupported viewer scheme: {scheme!r}')
    advertise_host = str(config_value(
        config, 'network.advertise_host', 'advertise_host', default='auto'
    )).strip()
    mdns_name = str(config_value(
        config, 'network.mdns_name', 'mdns_name', default='robot-pov.local'
    )).strip()
    health_base_url = str(config_value(
        config, 'network.health_base_url', 'health_base_url',
        default=f'{scheme}://127.0.0.1:{port}',
    )).rstrip('/')
    log_dir = Path(os.path.expandvars(os.path.expanduser(str(config_value(
        config, 'logging.directory', 'log_dir', default='logs/robot_pov'
    )))))
    min_free_disk_mb = float(config_value(
        config, 'preflight.min_free_disk_mb', 'min_free_disk_mb', default=512.0
    ))
    modules = _parse_list(config_value(
        config, 'dependencies.python', 'required_python_modules', default=()
    ))
    optional_modules = _parse_list(config_value(
        config, 'dependencies.optional_python', 'optional_python_modules',
        default=(),
    ))
    if not modules:
        modules = ('aiohttp', 'cv2', 'numpy')
        if source_type == 'ros':
            modules += ('rclpy', 'sensor_msgs')
        if source_type == 'unitree':
            modules += ('unitree_sdk2py',)
        if transport == 'webrtc':
            modules += ('aiortc', 'av')
    if not optional_modules:
        optional_modules = ('qrcode',)
        if transport == 'auto':
            optional_modules += ('aiortc', 'av')
    robot_ip = str(config_value(
        config, 'robot.ip', 'robot_ip', default=''
    )).strip()
    unitree_interface = str(config_value(
        config, 'source.interface', 'unitree_interface',
        default='enxb4b024be59fe',
    )).strip()
    left_source = config_value(
        config, 'source.left_source', 'source.device', 'source.path',
        'left_source', 'camera_device', 'usb_device',
        default='',
    )
    right_source = config_value(
        config, 'source.right_source', 'right_source', default=''
    )

    def camera_path(value: Any) -> str:
        if isinstance(value, int) or str(value).strip().isdigit():
            return f'/dev/video{int(value)}'
        return str(value).strip()

    usb_path = camera_path(left_source)
    usb_right_path = camera_path(right_source) if str(right_source) else ''
    rtsp_url = str(config_value(
        config, 'source.url', 'rtsp_url', 'stream_url', 'left_source',
        default=left_source,
    )).strip()
    rtsp_right_url = str(config_value(
        config, 'source.right_url', 'right_rtsp_url', 'right_source',
        default=right_source,
    )).strip()
    single_topic = str(config_value(
        config, 'source.topic', 'ros_topic', 'camera_topic', default=''
    )).strip()
    left_topic = str(config_value(
        config, 'source.left', 'source.left_topic', 'left_topic', default=''
    )).strip()
    right_topic = str(config_value(
        config, 'source.right', 'source.right_topic', 'right_topic', default=''
    )).strip()
    if single_topic:
        topics = (single_topic,)
    else:
        topics = tuple(item for item in (left_topic,) if item)
        if source_layout == 'stereo' and right_topic:
            topics += (right_topic,)
    if source_type == 'ros' and not topics:
        topics = ('/r1/camera/left/left_eye/image_raw',)
        if source_layout in ('stereo', 'stereo-topics', 'stereo_topics'):
            topics += ('/r1/camera/right/right_eye/image_raw',)
    if source_type == 'unitree':
        target_fps = float(config_value(
            config, 'source.fps', 'unitree_fps', default=15.0
        ))
    else:
        target_fps = float(config_value(
            config, 'video.fps', 'video_profile.fps', 'target_fps', 'fps',
            default=DEFAULT_TARGET_FPS.get(profile, 30.0),
        ))
    stale_after_sec = config_value(
        config, 'source.stale_after_sec', 'stale_after_sec',
        default=None,
    )
    fail_frame_age_default = (
        float(stale_after_sec) * 1000.0
        if stale_after_sec is not None else 750.0
    )
    return Settings(
        video_only=video_only,
        source_type=source_type,
        source_layout=source_layout,
        transport=transport,
        profile=profile,
        bind_host=bind_host,
        port=port,
        scheme=scheme,
        advertise_host=advertise_host,
        mdns_name=mdns_name,
        health_base_url=health_base_url,
        log_dir=log_dir,
        min_free_disk_mb=min_free_disk_mb,
        required_modules=modules,
        optional_modules=optional_modules,
        robot_ip=robot_ip,
        unitree_interface=unitree_interface,
        usb_path=usb_path,
        usb_right_path=usb_right_path,
        rtsp_url=rtsp_url,
        rtsp_right_url=rtsp_right_url,
        ros_topics=topics,
        target_fps=target_fps,
        ok_fps_ratio=float(config_value(
            config, 'preflight.ok_fps_ratio', 'ok_fps_ratio', default=0.8
        )),
        warn_fps_ratio=float(config_value(
            config, 'preflight.warn_fps_ratio', 'warn_fps_ratio', default=0.6
        )),
        warn_frame_age_ms=float(config_value(
            config, 'preflight.warn_frame_age_ms', 'warn_frame_age_ms',
            default=250.0,
        )),
        fail_frame_age_ms=float(config_value(
            config, 'preflight.fail_frame_age_ms', 'fail_frame_age_ms',
            'source.stale_after_ms', default=fail_frame_age_default,
        )),
        warn_latency_ms=float(config_value(
            config, 'preflight.warn_latency_ms', 'warn_latency_ms',
            default=120.0,
        )),
        fail_latency_ms=float(config_value(
            config, 'preflight.fail_latency_ms', 'fail_latency_ms',
            default=250.0,
        )),
        http_timeout_s=float(config_value(
            config, 'preflight.http_timeout_s', 'http_timeout_s', default=1.5
        )),
    )


def usable_ipv4(addresses: Iterable[str]) -> list[str]:
    """Filter, validate, and stably sort LAN-usable IPv4 addresses."""
    usable = set()
    for address in addresses:
        try:
            parsed = ipaddress.ip_address(str(address).split('/', 1)[0])
        except ValueError:
            continue
        if (
            parsed.version == 4
            and not parsed.is_loopback
            and not parsed.is_link_local
            and not parsed.is_multicast
            and not parsed.is_unspecified
        ):
            usable.add(str(parsed))
    return sorted(
        usable,
        key=lambda item: tuple(int(part) for part in item.split('.')),
    )


def classify_fps(
    actual: float | None,
    target: float,
    ok_ratio: float = 0.8,
    warn_ratio: float = 0.6,
) -> Status:
    """Classify an FPS sample against profile-relative thresholds."""
    if actual is None or not math.isfinite(actual):
        return Status.WARN
    if target <= 0.0 or not 0.0 <= warn_ratio <= ok_ratio <= 1.0:
        return Status.FAIL
    if actual >= target * ok_ratio:
        return Status.OK
    if actual >= target * warn_ratio:
        return Status.WARN
    return Status.FAIL


def classify_upper_bound(
    actual: float | None,
    warn_at: float,
    fail_at: float,
) -> Status:
    """Classify a metric where lower values are better."""
    if actual is None or not math.isfinite(actual):
        return Status.WARN
    if warn_at < 0.0 or fail_at < warn_at:
        return Status.FAIL
    if actual <= warn_at:
        return Status.OK
    if actual <= fail_at:
        return Status.WARN
    return Status.FAIL


def _result(
    check_id: str,
    status: Status,
    message: str,
    **details: Any,
) -> CheckResult:
    return CheckResult(check_id, status, message, details)


def _join_url(base: str, suffix: str) -> str:
    return f"{base.rstrip('/')}/{suffix.lstrip('/')}"


def _http_ok(
    adapter: SystemAdapter,
    url: str,
    timeout_s: float,
) -> tuple[bool, HttpResponse | None, str]:
    try:
        response = adapter.http_get(url, timeout_s)
    except ConnectionError as exc:
        return False, None, str(exc)
    return response.status == 200, response, f'HTTP {response.status}'


def _robot_pov_health(response: HttpResponse | None) -> bool:
    """Confirm that a response belongs to the safe Robot POV server."""
    if response is None or response.status != 200:
        return False
    payload = response.payload
    return (
        isinstance(payload, Mapping)
        and payload.get('service') == 'r1_robot_pov'
        and payload.get('video_only') is True
    )


def _valid_topic(topic: str) -> bool:
    return bool(re.fullmatch(r'/[A-Za-z0-9_][A-Za-z0-9_/]*', topic))


def _valid_host(host: str) -> bool:
    invalid_length = not host or len(host) > 253
    if invalid_length or any(character.isspace() for character in host):
        return False
    try:
        ipaddress.ip_address(host)
        return True
    except ValueError:
        labels = host.rstrip('.').split('.')
        return all(
            re.fullmatch(
                r'[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?',
                label,
            )
            for label in labels
        )


def _source_checks(
    settings: Settings,
    adapter: SystemAdapter,
    live: bool,
) -> list[CheckResult]:
    checks = []
    source = settings.source_type
    if source not in SUPPORTED_SOURCES:
        return [_result(
            'source', Status.FAIL, f'unsupported source {source!r}',
            supported=sorted(SUPPORTED_SOURCES),
        )]
    if source == 'mock':
        return [_result('source', Status.OK, 'local mock source selected')]
    if source == 'unitree':
        interface = settings.unitree_interface
        if (
            not interface
            or len(interface) > 15
            or not re.fullmatch(r'[A-Za-z0-9_.:-]+', interface)
        ):
            return [_result(
                'source', Status.FAIL,
                'invalid Unitree network interface name',
                interface=interface,
            )]
        state = adapter.network_interface_status(interface)
        if not state.exists:
            return [_result(
                'source', Status.FAIL,
                'Unitree network interface does not exist',
                interface=interface,
            )]
        if not state.up:
            return [_result(
                'source', Status.FAIL,
                'Unitree network interface is down',
                interface=interface,
            )]
        status = Status.OK if state.ipv4 else Status.WARN
        message = (
            'Unitree videohub interface is up'
            if state.ipv4
            else 'Unitree interface is up but has no IPv4 address'
        )
        return [_result(
            'source', status, message,
            interface=interface, ipv4=state.ipv4,
        )]
    if source == 'usb':
        paths = [settings.usb_path]
        if settings.source_layout == 'stereo':
            paths.append(settings.usb_right_path)
        if any(not path for path in paths):
            return [_result(
                'source', Status.FAIL, 'USB camera path is missing'
            )]
        if len(paths) == 2 and paths[0] == paths[1]:
            return [_result(
                'source', Status.FAIL,
                'stereo USB sources must use distinct devices', paths=paths,
            )]
        path_statuses = [adapter.inspect_path(path) for path in paths]
        missing = [
            path for path, item in zip(paths, path_statuses)
            if not item.exists or not item.readable
        ]
        if missing:
            return [_result(
                'source', Status.FAIL, 'USB camera is missing or unreadable',
                paths=missing,
            )]
        all_devices = all(item.character_device for item in path_statuses)
        status = Status.OK if all_devices else Status.WARN
        message = (
            'USB video devices are readable' if status == Status.OK
            else 'a source path exists but is not a character device'
        )
        return [_result('source', status, message, paths=paths)]
    if source == 'rtsp':
        urls = [settings.rtsp_url]
        if settings.source_layout == 'stereo':
            urls.append(settings.rtsp_right_url)
        if any(not url for url in urls):
            return [_result('source', Status.FAIL, 'RTSP URL is missing')]
        for url in urls:
            redacted = redact_url(url)
            try:
                parts = urlsplit(url)
                port = parts.port or (322 if parts.scheme == 'rtsps' else 554)
            except ValueError:
                parts = urlsplit('')
                port = 554
            if parts.scheme not in ('rtsp', 'rtsps') or not parts.hostname:
                return [_result(
                    'source', Status.FAIL, 'invalid RTSP URL', url=redacted
                )]
            if live and not adapter.tcp_reachable(
                parts.hostname, port, settings.http_timeout_s
            ):
                return [_result(
                    'source', Status.FAIL, 'RTSP endpoint is unreachable',
                    url=redacted, host=parts.hostname, port=port,
                )]
        return [_result(
            'source', Status.OK,
            'RTSP URLs are valid'
            + (' and TCP endpoints are reachable' if live else ''),
            urls=[redact_url(url) for url in urls],
        )]
    if not settings.ros_topics:
        return [_result('source', Status.FAIL, 'ROS image topic is missing')]
    invalid = [
        topic for topic in settings.ros_topics if not _valid_topic(topic)
    ]
    if invalid:
        return [_result(
            'source', Status.FAIL, 'invalid ROS topic name', topics=invalid
        )]
    missing = []
    probe_details = {}
    if live:
        for topic in settings.ros_topics:
            available, detail = adapter.topic_has_publisher(
                topic, settings.http_timeout_s
            )
            probe_details[topic] = detail
            if not available:
                missing.append(topic)
    if missing:
        return [_result(
            'source', Status.FAIL, 'ROS topic has no publisher',
            topics=missing, probe=probe_details,
        )]
    checks.append(_result(
        'source', Status.OK,
        'ROS topic names are valid' + (' and published' if live else ''),
        topics=settings.ros_topics,
    ))
    return checks


def _flatten_numbers(value: Any, prefix: str = '') -> dict[str, float]:
    flattened = {}
    if isinstance(value, Mapping):
        for key, item in value.items():
            item_prefix = f'{prefix}.{key}' if prefix else str(key)
            flattened.update(_flatten_numbers(item, item_prefix))
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        flattened[prefix.lower()] = float(value)
    return flattened


def _metric(
    flattened: Mapping[str, float], names: Sequence[str]
) -> float | None:
    lowered_names = tuple(name.lower() for name in names)
    for name in lowered_names:
        if name in flattened:
            return flattened[name]
    for key, value in flattened.items():
        leaf = key.rsplit('.', 1)[-1]
        if leaf in lowered_names:
            return value
    return None


def extract_live_metrics(payload: Any) -> dict[str, float | None]:
    """Extract common status fields without coupling to one server schema."""
    flattened = _flatten_numbers(payload)
    left_fps = _metric(flattened, ('left_fps', 'fps_left'))
    right_fps = _metric(flattened, ('right_fps', 'fps_right'))
    fps_values = [item for item in (left_fps, right_fps) if item is not None]
    fps = min(fps_values) if fps_values else _metric(
        flattened,
        ('fps', 'output_fps', 'stream_fps', 'rendered_fps', 'capture_fps'),
    )
    age_ms = _metric(
        flattened,
        ('frame_age_ms', 'source_age_ms', 'last_frame_age_ms', 'age_ms'),
    )
    if age_ms is None:
        age_s = _metric(flattened, ('frame_age_s', 'source_age_s'))
        age_ms = None if age_s is None else age_s * 1000.0
    latency_ms = _metric(
        flattened,
        ('latency_ms', 'estimated_latency_ms', 'pipeline_latency_ms'),
    )
    reconnects = _metric(flattened, ('reconnect_count', 'reconnects'))
    return {
        'fps': fps,
        'frame_age_ms': age_ms,
        'latency_ms': latency_ms,
        'reconnect_count': reconnects,
    }


def _live_checks(
    settings: Settings,
    adapter: SystemAdapter,
    duration_s: float,
    numeric_base_url: str | None,
) -> list[CheckResult]:
    checks = []
    endpoints = ('healthz', 'readyz')
    for endpoint in endpoints:
        url = _join_url(settings.health_base_url, endpoint)
        ok, response, detail = _http_ok(adapter, url, settings.http_timeout_s)
        if endpoint == 'healthz' and not _robot_pov_health(response):
            ok = False
            detail = 'identity/video-only invariant mismatch'
        message = (
            f'/{endpoint} is ready'
            if ok else f'/{endpoint} failed: {detail}'
        )
        checks.append(_result(
            f'viewer.{endpoint}', Status.OK if ok else Status.FAIL, message,
            url=url, response=getattr(response, 'payload', None),
        ))
    if numeric_base_url and numeric_base_url != settings.health_base_url:
        lan_health = _join_url(numeric_base_url, 'healthz')
        ok, _, detail = _http_ok(adapter, lan_health, settings.http_timeout_s)
        checks.append(_result(
            'viewer.lan_url', Status.OK if ok else Status.FAIL,
            'LAN viewer address is reachable' if ok
            else f'LAN viewer address failed: {detail}',
            url=lan_health,
        ))

    sample_count = max(1, int(math.ceil(max(0.0, duration_s) / 0.5)))
    payloads = []
    errors = []
    status_url = _join_url(settings.health_base_url, 'api/status')
    for index in range(sample_count):
        ok, response, detail = _http_ok(
            adapter, status_url, settings.http_timeout_s
        )
        valid_payload = (
            ok and response is not None
            and isinstance(response.payload, Mapping)
        )
        if valid_payload:
            payloads.append(response.payload)
        else:
            errors.append(detail)
        if index + 1 < sample_count:
            adapter.sleep(0.5)
    if errors or not payloads:
        checks.append(_result(
            'viewer.status', Status.FAIL,
            '/api/status is unavailable or invalid',
            url=status_url, errors=errors,
        ))
        return checks
    if any(payload.get('video_only') is not True for payload in payloads):
        checks.append(_result(
            'viewer.status', Status.FAIL,
            '/api/status does not confirm video_only=true',
        ))
        return checks
    checks.append(_result(
        'viewer.status', Status.OK, '/api/status returned live metrics',
        samples=len(payloads),
    ))
    metrics = [extract_live_metrics(payload) for payload in payloads]
    fps_samples = [item['fps'] for item in metrics if item['fps'] is not None]
    age_samples = [
        item['frame_age_ms'] for item in metrics
        if item['frame_age_ms'] is not None
    ]
    latency_samples = [
        item['latency_ms'] for item in metrics
        if item['latency_ms'] is not None
    ]
    fps = statistics.median(fps_samples) if fps_samples else None
    frame_age = max(age_samples) if age_samples else None
    latency = statistics.median(latency_samples) if latency_samples else None
    fps_status = classify_fps(
        fps, settings.target_fps,
        settings.ok_fps_ratio, settings.warn_fps_ratio,
    )
    checks.append(_result(
        'viewer.fps', fps_status,
        'FPS metric is missing' if fps is None
        else f'FPS {fps:.1f} / target {settings.target_fps:.1f}',
        actual_fps=fps, target_fps=settings.target_fps,
    ))
    age_status = classify_upper_bound(
        frame_age, settings.warn_frame_age_ms, settings.fail_frame_age_ms
    )
    checks.append(_result(
        'viewer.frame_age', age_status,
        'frame age metric is missing' if frame_age is None
        else f'maximum frame age {frame_age:.1f} ms',
        frame_age_ms=frame_age,
        warn_at_ms=settings.warn_frame_age_ms,
        fail_at_ms=settings.fail_frame_age_ms,
    ))
    if latency is not None:
        latency_status = classify_upper_bound(
            latency, settings.warn_latency_ms, settings.fail_latency_ms
        )
        checks.append(_result(
            'viewer.latency', latency_status,
            f'estimated latency {latency:.1f} ms', latency_ms=latency,
        ))
    return checks


def run_preflight(
    config: Any,
    *,
    live: bool = False,
    duration_s: float = 0.0,
    adapter: SystemAdapter | None = None,
) -> PreflightReport:
    """Run checks without writing files or activating any robot interface."""
    system = adapter or SystemAdapter()
    generated_at = system.utc_now().isoformat().replace('+00:00', 'Z')
    try:
        settings = resolve_settings(config)
    except (TypeError, ValueError, OverflowError) as exc:
        return PreflightReport(
            generated_at,
            (_result('config', Status.FAIL, str(exc)),),
            sample_duration_s=max(0.0, duration_s),
        )
    checks = []
    checks.append(_result(
        'safety.video_only',
        Status.OK if settings.video_only else Status.FAIL,
        'video-only invariant is enabled' if settings.video_only
        else 'video_only must be true; motor-control launch is forbidden',
    ))
    checks.append(_result(
        'config.profile',
        Status.OK if settings.profile in SUPPORTED_PROFILES else Status.FAIL,
        f'profile {settings.profile!r}'
        if settings.profile in SUPPORTED_PROFILES
        else f'unsupported profile {settings.profile!r}',
        supported=sorted(SUPPORTED_PROFILES),
    ))
    layout_ok = settings.source_layout in ('mono', 'stereo')
    checks.append(_result(
        'config.layout', Status.OK if layout_ok else Status.FAIL,
        f'layout {settings.source_layout!r}' if layout_ok
        else f'unsupported layout {settings.source_layout!r}',
    ))
    transport_ok = settings.transport in ('auto', 'webrtc', 'mjpeg')
    checks.append(_result(
        'config.transport', Status.OK if transport_ok else Status.FAIL,
        f'transport {settings.transport!r}' if transport_ok
        else f'unsupported transport {settings.transport!r}',
    ))

    local_addresses = usable_ipv4(system.local_ipv4_addresses())
    bind_valid = settings.bind_host in ('0.0.0.0', '*', '')
    if not bind_valid:
        try:
            bind_address = str(ipaddress.ip_address(settings.bind_host))
            bind_valid = bind_address in local_addresses
        except ValueError:
            resolved_bind = system.resolve_ipv4(settings.bind_host)
            bind_valid = bool(set(resolved_bind) & set(local_addresses))
    if not local_addresses:
        checks.append(_result(
            'network.ipv4', Status.FAIL, 'no non-loopback local IPv4 address'
        ))
    elif not bind_valid:
        checks.append(_result(
            'network.ipv4', Status.FAIL, 'bind_host is not assigned locally',
            bind_host=settings.bind_host, addresses=local_addresses,
        ))
    else:
        checks.append(_result(
            'network.ipv4', Status.OK,
            f'local IPv4 available: {local_addresses[0]}',
            addresses=local_addresses,
        ))

    numeric_host = local_addresses[0] if local_addresses else None
    try:
        advertised_ip = ipaddress.ip_address(settings.advertise_host)
        if advertised_ip.version == 4 and not advertised_ip.is_loopback:
            numeric_host = str(advertised_ip)
    except ValueError:
        pass
    numeric_base = (
        f'{settings.scheme}://{numeric_host}:{settings.port}'
        if numeric_host else None
    )
    urls = []
    if settings.mdns_name:
        urls.append(
            f'{settings.scheme}://{settings.mdns_name}:{settings.port}/'
        )
    if numeric_base:
        urls.append(f'{numeric_base}/')

    port_free = system.port_available(settings.bind_host, settings.port)
    health_url = _join_url(settings.health_base_url, 'healthz')
    if port_free and not live:
        checks.append(_result(
            'network.port', Status.OK,
            f'TCP port {settings.port} is free for the viewer',
        ))
    elif port_free and live:
        checks.append(_result(
            'network.port', Status.FAIL,
            f'TCP port {settings.port} is free; live viewer is not listening',
        ))
    else:
        healthy, health_response, detail = _http_ok(
            system, health_url, settings.http_timeout_s
        )
        healthy = healthy and _robot_pov_health(health_response)
        if health_response is not None and not healthy:
            detail = 'identity/video-only invariant mismatch'
        checks.append(_result(
            'network.port', Status.OK if healthy else Status.FAIL,
            'port is owned by a healthy Robot POV viewer' if healthy
            else (
                f'port {settings.port} is occupied by an unknown service: '
                f'{detail}'
            ),
            health_url=health_url,
        ))

    storage = system.storage_status(settings.log_dir)
    free_mb = storage.free_bytes / (1024.0 * 1024.0)
    storage_ok = storage.writable and free_mb >= settings.min_free_disk_mb
    checks.append(_result(
        'logging.storage', Status.OK if storage_ok else Status.FAIL,
        f'log storage writable, {free_mb:.0f} MiB free' if storage_ok
        else 'log storage is not writable or has insufficient free space',
        directory=settings.log_dir,
        checked_path=storage.checked_path,
        free_mb=round(free_mb, 1),
        required_mb=settings.min_free_disk_mb,
    ))

    missing_modules = [
        module for module in settings.required_modules
        if not system.module_available(module)
    ]
    checks.append(_result(
        'dependencies.python',
        Status.FAIL if missing_modules else Status.OK,
        f'missing Python modules: {", ".join(missing_modules)}'
        if missing_modules else 'required Python modules are available',
        required=settings.required_modules,
    ))
    missing_optional = [
        module for module in settings.optional_modules
        if not system.module_available(module)
    ]
    optional_status = Status.WARN if missing_optional else Status.OK
    optional_message = (
        f'optional modules unavailable: {", ".join(missing_optional)}'
        if missing_optional else 'optional Python modules are available'
    )
    checks.append(_result(
        'dependencies.optional_python', optional_status, optional_message,
        optional=settings.optional_modules,
    ))
    checks.extend(_source_checks(settings, system, live))

    if not settings.robot_ip:
        checks.append(_result(
            'robot.network', Status.WARN,
            'robot IP is not configured; acceptable for video-only mock/USB',
        ))
    elif not _valid_host(settings.robot_ip):
        checks.append(_result(
            'robot.network', Status.FAIL, 'robot IP/hostname is invalid',
            host=settings.robot_ip,
        ))
    elif system.ping(settings.robot_ip, settings.http_timeout_s):
        checks.append(_result(
            'robot.network', Status.OK, 'robot responds to ping',
            host=settings.robot_ip,
        ))
    else:
        checks.append(_result(
            'robot.network', Status.WARN,
            'robot is unreachable; video-only mode remains safe',
            host=settings.robot_ip,
        ))

    if settings.mdns_name:
        resolved = system.resolve_ipv4(settings.mdns_name)
        matching = bool(set(resolved) & set(local_addresses))
        checks.append(_result(
            'network.mdns', Status.OK if matching else Status.WARN,
            'mDNS name resolves to this host' if matching
            else 'mDNS name is unavailable; use the numeric viewer URL',
            hostname=settings.mdns_name, resolved=resolved,
        ))
    else:
        checks.append(_result(
            'network.mdns', Status.WARN,
            'mDNS name is disabled; use the numeric viewer URL',
        ))

    if live:
        checks.extend(_live_checks(
            settings, system, max(0.0, duration_s), numeric_base
        ))
    return PreflightReport(
        generated_at,
        tuple(checks),
        tuple(dict.fromkeys(urls)),
        max(0.0, duration_s) if live else 0.0,
    )


def write_json_report(report: PreflightReport, path: Path) -> Path:
    """Write the explicitly selected report and no other files."""
    destination = path.expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open('w', encoding='utf-8') as stream:
        json.dump(report.to_dict(), stream, indent=2, sort_keys=True)
        stream.write('\n')
    return destination


def _environment_config() -> dict[str, str]:
    known = {
        'VIDEO_ONLY', 'SOURCE', 'SOURCE_TYPE', 'PROFILE', 'BIND_HOST',
        'HOST', 'PORT', 'HTTP_PORT', 'ADVERTISE_HOST', 'MDNS_NAME',
        'LOG_DIR', 'ROBOT_IP', 'CAMERA_DEVICE', 'USB_DEVICE', 'RTSP_URL',
        'STREAM_URL', 'ROS_TOPIC', 'CAMERA_TOPIC', 'TARGET_FPS',
    }
    return {
        key: value for key, value in os.environ.items()
        if key.startswith('ROBOT_POV_') or key in known
    }


def _default_report_path(log_dir: Path, generated_at: str) -> Path:
    safe_stamp = (
        generated_at.replace('-', '').replace(':', '').split('.', 1)[0]
    )
    return log_dir / f'preflight-{safe_stamp}.json'


def main(argv: Sequence[str] | None = None) -> int:
    """Run preflight from a dotenv file/environment and save a JSON report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--env-file', type=Path)
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--log-dir', type=Path)
    parser.add_argument(
        '--report', help='JSON report path, or - to print JSON to stdout'
    )
    arguments = parser.parse_args(argv)
    config = {}
    if arguments.env_file is not None:
        try:
            file_config = load_env_file(arguments.env_file)
        except (OSError, ValueError) as exc:
            print(f'Robot POV preflight: FAIL\n[FAIL] config: {exc}')
            return 1
        config.update(file_config)
    # Match the runtime loader: process variables override venue defaults.
    config.update(_environment_config())
    if arguments.log_dir is not None:
        config['LOG_DIR'] = str(arguments.log_dir)
    report = run_preflight(
        config, live=arguments.live, duration_s=arguments.duration
    )
    print(report.render_human())
    if arguments.report == '-':
        print(json.dumps(report.to_dict(), indent=2, sort_keys=True))
    else:
        try:
            settings = resolve_settings(config)
            report_path = (
                Path(arguments.report) if arguments.report
                else _default_report_path(
                    settings.log_dir, report.generated_at
                )
            )
            written = write_json_report(report, report_path)
            print(f'JSON report: {written}')
        except (OSError, TypeError, ValueError) as exc:
            print(f'Could not write JSON report: {exc}', file=sys.stderr)
            return 1
    if report.overall == Status.FAIL:
        return 1
    if report.overall == Status.WARN:
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
