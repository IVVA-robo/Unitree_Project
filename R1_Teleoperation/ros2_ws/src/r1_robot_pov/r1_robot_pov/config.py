"""Environment-only configuration for the offline Robot POV service."""

from dataclasses import asdict, dataclass, replace
import os
from pathlib import Path
import re
import socket
from typing import Dict, Mapping, Optional, Tuple
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit


@dataclass(frozen=True)
class VideoProfile:
    """One bounded output-quality profile."""

    name: str
    width: int
    height: int
    fps: float
    jpeg_quality: int
    video_bitrate_kbps: int


PROFILES: Dict[str, VideoProfile] = {
    'high': VideoProfile('high', 1280, 480, 30.0, 85, 3500),
    'balanced': VideoProfile('balanced', 960, 360, 24.0, 72, 1800),
    'low-latency': VideoProfile('low-latency', 960, 360, 30.0, 62, 1200),
    'bad-wifi': VideoProfile('bad-wifi', 640, 240, 15.0, 48, 550),
}

SUPPORTED_SOURCES = ('mock', 'ros', 'usb', 'rtsp', 'unitree')
SUPPORTED_LAYOUTS = ('mono', 'stereo', 'top-bottom')
SUPPORTED_TRANSPORTS = ('auto', 'webrtc', 'mjpeg')
SENSITIVE_QUERY_KEYS = frozenset({
    'access_token', 'api_key', 'auth', 'key', 'pass', 'password',
    'secret', 'token',
})


def read_env_file(path: Optional[str]) -> Dict[str, str]:
    """Read a small dotenv file without adding a runtime dependency."""
    if not path:
        return {}
    env_path = Path(path).expanduser()
    if not env_path.is_file():
        raise FileNotFoundError(f'env file does not exist: {env_path}')
    values: Dict[str, str] = {}
    for number, raw_line in enumerate(
        env_path.read_text(encoding='utf-8').splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith('#'):
            continue
        if line.startswith('export '):
            line = line[7:].strip()
        if '=' not in line:
            raise ValueError(f'{env_path}:{number}: expected KEY=VALUE')
        key, value = line.split('=', 1)
        key = key.strip()
        value = value.strip()
        if not key or not key.replace('_', '').isalnum():
            raise ValueError(f'{env_path}:{number}: invalid variable name')
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "'\"":
            value = value[1:-1]
        values[key] = value
    return values


def _bool(value: str, name: str) -> bool:
    normalized = str(value).strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'{name} must be true or false')


def _integer(value: str, name: str, minimum: int, maximum: int) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be an integer') from exc
    if not minimum <= result <= maximum:
        raise ValueError(f'{name} must be in [{minimum}, {maximum}]')
    return result


def _number(value: str, name: str, minimum: float, maximum: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a number') from exc
    if not minimum <= result <= maximum:
        raise ValueError(f'{name} must be in [{minimum}, {maximum}]')
    return result


def _crop(value: str) -> Tuple[int, int, int, int]:
    try:
        values = tuple(int(item.strip()) for item in value.split(','))
    except ValueError as exc:
        raise ValueError('ROBOT_POV_CROP must be top,right,bottom,left pixels') from exc
    if len(values) != 4 or min(values) < 0:
        raise ValueError('ROBOT_POV_CROP must contain four non-negative integers')
    return values


def _camera_value(value: str):
    stripped = str(value).strip()
    if stripped.isdigit():
        return int(stripped)
    return stripped


def _validate_rtsp_url(value: object, name: str) -> None:
    try:
        parts = urlsplit(str(value))
        hostname = parts.hostname
        parts.port  # Parse now so a malformed port fails during preflight.
    except ValueError as exc:
        raise ValueError(f'{name} is not a valid RTSP URL') from exc
    if parts.scheme not in ('rtsp', 'rtsps') or not hostname:
        raise ValueError(f'{name} must be an rtsp:// or rtsps:// URL')


def discover_lan_ipv4() -> str:
    """Best-effort LAN IPv4 discovery that does not contact the internet."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(('192.0.2.1', 9))
        address = sock.getsockname()[0]
    except OSError:
        try:
            address = socket.gethostbyname(socket.gethostname())
        except OSError:
            address = '127.0.0.1'
    finally:
        sock.close()
    return address


@dataclass(frozen=True)
class PovConfig:
    """Validated process configuration for a strictly video-only service."""

    mode: str = 'mock'
    video_only: bool = True
    bind_host: str = '0.0.0.0'
    port: int = 8080
    advertise_host: str = 'auto'
    mdns_name: str = 'robot-pov.local'
    source: str = 'mock'
    layout: str = 'stereo'
    transport: str = 'auto'
    profile: str = 'balanced'
    adaptive: bool = True
    left_topic: str = '/r1/camera/left/left_eye/image_raw'
    right_topic: str = '/r1/camera/right/right_eye/image_raw'
    left_source: object = '/dev/video0'
    right_source: object = ''
    unitree_interface: str = 'enxb4b024be59fe'
    unitree_timeout_sec: float = 3.0
    unitree_fps: float = 15.0
    swap_eyes: bool = False
    crop: Tuple[int, int, int, int] = (0, 0, 0, 0)
    rotation: int = 0
    flip_horizontal: bool = False
    flip_vertical: bool = False
    stale_after_sec: float = 0.75
    stereo_max_skew_sec: float = 0.06
    reconnect_delay_sec: float = 0.5
    fov_deg: float = 80.0
    ipd_mm: float = 64.0
    distortion_k1: float = 0.0
    tls_cert: str = ''
    tls_key: str = ''
    log_dir: str = 'logs/robot_pov'
    robot_ip: str = ''
    preflight_duration_sec: float = 5.0

    @property
    def video_profile(self) -> VideoProfile:
        """Return the selected bounded profile."""
        return PROFILES[self.profile]

    @property
    def scheme(self) -> str:
        """Return HTTP scheme implied by configured TLS files."""
        return 'https' if self.tls_cert and self.tls_key else 'http'

    @property
    def lan_ip(self) -> str:
        """Return configured or automatically discovered advertised address."""
        if self.advertise_host and self.advertise_host != 'auto':
            return self.advertise_host
        return discover_lan_ipv4()

    @property
    def numeric_url(self) -> str:
        return f'{self.scheme}://{self.lan_ip}:{self.port}/'

    @property
    def mdns_url(self) -> str:
        host = self.mdns_name.rstrip('.')
        return f'{self.scheme}://{host}:{self.port}/' if host else ''

    def public_dict(self) -> dict:
        """Return browser-safe configuration; credentials never leave server."""
        profile_values = {name: asdict(value) for name, value in PROFILES.items()}
        return {
            'mode': self.mode,
            'videoOnly': self.video_only,
            'source': self.source,
            'layout': self.layout,
            'transport': self.transport,
            'profile': self.profile,
            'profiles': profile_values,
            'adaptive': self.adaptive,
            'swapEyes': self.swap_eyes,
            'fovDeg': self.fov_deg,
            'ipdMm': self.ipd_mm,
            'distortionK1': self.distortion_k1,
            'staleAfterMs': round(self.stale_after_sec * 1000.0),
            'numericUrl': self.numeric_url,
            'mdnsUrl': self.mdns_url,
            'webxrSecureContext': self.scheme == 'https',
        }

    def redacted_dict(self) -> dict:
        """Return configuration suitable for local diagnostics logs."""
        result = asdict(self)
        for name in ('left_source', 'right_source'):
            value = str(result[name])
            parts = urlsplit(value)
            if not parts.scheme or not parts.netloc:
                continue
            netloc = parts.netloc
            if '@' in netloc:
                netloc = f'***:***@{netloc.rsplit("@", 1)[1]}'
            query = urlencode([
                (key, '***' if key.lower() in SENSITIVE_QUERY_KEYS else item)
                for key, item in parse_qsl(parts.query, keep_blank_values=True)
            ])
            result[name] = urlunsplit(
                (parts.scheme, netloc, parts.path, query, parts.fragment)
            )
        return result

    def with_overrides(self, **values):
        """Return a revalidated copy with CLI overrides applied."""
        return validate_config(replace(self, **values))


def validate_config(config: PovConfig) -> PovConfig:
    """Reject unsafe and internally inconsistent configuration."""
    if not config.video_only:
        raise ValueError('ROBOT_POV_VIDEO_ONLY must remain true')
    if config.source not in SUPPORTED_SOURCES:
        raise ValueError(f'unsupported source: {config.source}')
    if config.layout not in SUPPORTED_LAYOUTS:
        raise ValueError(f'unsupported layout: {config.layout}')
    if config.transport not in SUPPORTED_TRANSPORTS:
        raise ValueError(f'unsupported transport: {config.transport}')
    if config.profile not in PROFILES:
        raise ValueError(f'unsupported profile: {config.profile}')
    if config.rotation not in (0, 90, 180, 270):
        raise ValueError('ROBOT_POV_ROTATION must be 0, 90, 180, or 270')
    if not 1 <= config.port <= 65535:
        raise ValueError('ROBOT_POV_PORT must be in [1, 65535]')
    if config.source == 'ros' and not config.left_topic:
        raise ValueError('ROS source requires ROBOT_POV_LEFT_TOPIC')
    if config.source == 'unitree':
        interface = str(config.unitree_interface).strip()
        if (
            not interface
            or len(interface) > 15
            or not re.fullmatch(r'[A-Za-z0-9_.:-]+', interface)
        ):
            raise ValueError(
                'ROBOT_POV_UNITREE_INTERFACE must be a valid interface name'
            )
        if config.layout != 'mono':
            raise ValueError('Unitree videohub source requires mono layout')
    if config.source in ('usb', 'rtsp') and not str(config.left_source):
        raise ValueError(f'{config.source} source requires ROBOT_POV_LEFT_SOURCE')
    if config.layout == 'stereo' and config.source in ('usb', 'rtsp'):
        if not str(config.right_source):
            raise ValueError('stereo USB/RTSP source requires ROBOT_POV_RIGHT_SOURCE')
    if config.source == 'rtsp':
        _validate_rtsp_url(config.left_source, 'ROBOT_POV_LEFT_SOURCE')
        if config.layout == 'stereo':
            _validate_rtsp_url(config.right_source, 'ROBOT_POV_RIGHT_SOURCE')
    if bool(config.tls_cert) != bool(config.tls_key):
        raise ValueError('both ROBOT_POV_TLS_CERT and ROBOT_POV_TLS_KEY are required')
    return config


def load_config(
    env_file: Optional[str] = None,
    environ: Optional[Mapping[str, str]] = None,
) -> PovConfig:
    """Load defaults, dotenv values, then process environment overrides."""
    values = read_env_file(env_file)
    values.update(dict(os.environ if environ is None else environ))

    def value(name: str, default):
        return values.get(name, default)

    config = PovConfig(
        mode=str(value('ROBOT_POV_MODE', 'mock')).strip().lower(),
        video_only=_bool(value('ROBOT_POV_VIDEO_ONLY', 'true'), 'ROBOT_POV_VIDEO_ONLY'),
        bind_host=str(value('ROBOT_POV_BIND_HOST', '0.0.0.0')).strip(),
        port=_integer(value('ROBOT_POV_PORT', '8080'), 'ROBOT_POV_PORT', 1, 65535),
        advertise_host=str(value('ROBOT_POV_ADVERTISE_HOST', 'auto')).strip(),
        mdns_name=str(value('ROBOT_POV_MDNS_NAME', 'robot-pov.local')).strip(),
        source=str(value('ROBOT_POV_SOURCE', 'mock')).strip().lower(),
        layout=str(value('ROBOT_POV_LAYOUT', 'stereo')).strip().lower(),
        transport=str(value('ROBOT_POV_TRANSPORT', 'auto')).strip().lower(),
        profile=str(value('ROBOT_POV_PROFILE', 'balanced')).strip().lower(),
        adaptive=_bool(value('ROBOT_POV_ADAPTIVE', 'true'), 'ROBOT_POV_ADAPTIVE'),
        left_topic=str(value('ROBOT_POV_LEFT_TOPIC', PovConfig.left_topic)).strip(),
        right_topic=str(value('ROBOT_POV_RIGHT_TOPIC', PovConfig.right_topic)).strip(),
        left_source=_camera_value(value('ROBOT_POV_LEFT_SOURCE', '/dev/video0')),
        right_source=_camera_value(value('ROBOT_POV_RIGHT_SOURCE', '')),
        unitree_interface=str(value(
            'ROBOT_POV_UNITREE_INTERFACE', PovConfig.unitree_interface
        )).strip(),
        unitree_timeout_sec=_number(
            value('ROBOT_POV_UNITREE_TIMEOUT_SEC', '3.0'),
            'ROBOT_POV_UNITREE_TIMEOUT_SEC', 0.2, 30.0,
        ),
        unitree_fps=_number(
            value('ROBOT_POV_UNITREE_FPS', '15.0'),
            'ROBOT_POV_UNITREE_FPS', 1.0, 30.0,
        ),
        swap_eyes=_bool(value('ROBOT_POV_SWAP_EYES', 'false'), 'ROBOT_POV_SWAP_EYES'),
        crop=_crop(value('ROBOT_POV_CROP', '0,0,0,0')),
        rotation=_integer(
            value('ROBOT_POV_ROTATION', '0'), 'ROBOT_POV_ROTATION', 0, 270
        ),
        flip_horizontal=_bool(
            value('ROBOT_POV_FLIP_HORIZONTAL', 'false'),
            'ROBOT_POV_FLIP_HORIZONTAL',
        ),
        flip_vertical=_bool(
            value('ROBOT_POV_FLIP_VERTICAL', 'false'),
            'ROBOT_POV_FLIP_VERTICAL',
        ),
        stale_after_sec=_number(
            value('ROBOT_POV_STALE_AFTER_SEC', '0.75'),
            'ROBOT_POV_STALE_AFTER_SEC', 0.1, 30.0,
        ),
        stereo_max_skew_sec=_number(
            value('ROBOT_POV_STEREO_MAX_SKEW_SEC', '0.06'),
            'ROBOT_POV_STEREO_MAX_SKEW_SEC', 0.0, 1.0,
        ),
        reconnect_delay_sec=_number(
            value('ROBOT_POV_RECONNECT_DELAY_SEC', '0.5'),
            'ROBOT_POV_RECONNECT_DELAY_SEC', 0.05, 30.0,
        ),
        fov_deg=_number(
            value('ROBOT_POV_FOV_DEG', '80.0'), 'ROBOT_POV_FOV_DEG', 20.0, 180.0
        ),
        ipd_mm=_number(
            value('ROBOT_POV_IPD_MM', '64.0'), 'ROBOT_POV_IPD_MM', 40.0, 85.0
        ),
        distortion_k1=_number(
            value('ROBOT_POV_DISTORTION_K1', '0.0'),
            'ROBOT_POV_DISTORTION_K1', -1.0, 1.0,
        ),
        tls_cert=str(value('ROBOT_POV_TLS_CERT', '')).strip(),
        tls_key=str(value('ROBOT_POV_TLS_KEY', '')).strip(),
        log_dir=str(value('ROBOT_POV_LOG_DIR', 'logs/robot_pov')).strip(),
        robot_ip=str(value('ROBOT_POV_ROBOT_IP', '')).strip(),
        preflight_duration_sec=_number(
            value('ROBOT_POV_PREFLIGHT_DURATION_SEC', '5.0'),
            'ROBOT_POV_PREFLIGHT_DURATION_SEC', 1.0, 120.0,
        ),
    )
    return validate_config(config)
