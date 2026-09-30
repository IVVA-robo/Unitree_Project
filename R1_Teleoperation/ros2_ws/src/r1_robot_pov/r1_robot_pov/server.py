"""Local-only WebRTC server with a resilient MJPEG fallback."""

from __future__ import annotations

import asyncio
from collections import deque
from fractions import Fraction
import io
import json
import logging
from pathlib import Path
import ssl
import time
from typing import Dict, Mapping, Optional, Set

from aiohttp import web
import cv2
import numpy as np

try:
    from aiortc import (
        RTCConfiguration,
        RTCPeerConnection,
        RTCRtpSender,
        RTCSessionDescription,
        VideoStreamTrack,
    )
    from av import VideoFrame

    WEBRTC_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised by dependency preflight.
    WEBRTC_AVAILABLE = False
    VideoStreamTrack = object

try:
    import qrcode

    QR_AVAILABLE = True
except ImportError:  # pragma: no cover - QR text page remains available.
    QR_AVAILABLE = False

from .config import PROFILES, PovConfig, VideoProfile


LOGGER = logging.getLogger('r1_robot_pov.server')
VIDEO_TIME_BASE = Fraction(1, 90000)
PREFERRED_CODECS = ('vp8', 'h264')
CLIENT_TAG_MAX_LENGTH = 64
SERVER_SHUTDOWN_TIMEOUT_SEC = 1.0
DISCOVERY_PORT = 9091
CONTROL_PORT = 9090
DISCOVERY_PROBE_PREFIX = b'R1_TELEOP_DISCOVER v1 '
DISCOVERY_RESPONSE_PREFIX = 'R1_TELEOP_ENDPOINT v1'


def _normalize_preferred_codec(value):
    """Normalize an optional native-client codec request through an allowlist."""
    if value is None:
        return None
    if not isinstance(value, str):
        raise ValueError('preferredCodec must be a string')
    normalized = value.strip().lower().replace('video/', '')
    normalized = normalized.replace('.', '').replace('-', '')
    if not normalized:
        return None
    if normalized not in PREFERRED_CODECS:
        allowed = ', '.join(PREFERRED_CODECS)
        raise ValueError(f'preferredCodec must be one of: {allowed}')
    return normalized


def _normalize_client_tag(value):
    """Validate an optional diagnostic-only native client identifier."""
    if value is None:
        return None
    if not isinstance(value, str):
        raise ValueError('client tag must be a string')
    normalized = value.strip()
    if not normalized:
        return None
    if len(normalized) > CLIENT_TAG_MAX_LENGTH:
        raise ValueError(
            f'client tag must not exceed {CLIENT_TAG_MAX_LENGTH} characters'
        )
    allowed = normalized[0].isalnum() and all(
        character.isalnum() or character in '._:-'
        for character in normalized
    )
    if not allowed or not normalized.isascii():
        raise ValueError('client tag contains unsupported characters')
    return normalized


def _offer_preferences(body):
    """Extract safe optional negotiation metadata from one JSON offer."""
    if not isinstance(body, Mapping):
        raise ValueError('offer body must be a JSON object')
    preferred_codec = _normalize_preferred_codec(body.get('preferredCodec'))
    client_value = body.get('clientTag', body.get('client'))
    if 'clientTag' in body and 'client' in body:
        if body['clientTag'] != body['client']:
            raise ValueError('client and clientTag must match when both are set')
    return preferred_codec, _normalize_client_tag(client_value)


def _ordered_video_codecs(codecs, preferred_codec=None):
    """Order video codecs stably, preserving VP8 as the browser default."""
    preferred = _normalize_preferred_codec(preferred_codec) or 'vp8'
    preferred_mime = f'video/{preferred}'.lower()

    def mime_type(codec):
        return str(getattr(codec, 'mimeType', '')).lower()

    primary = [codec for codec in codecs if mime_type(codec) == preferred_mime]
    fallback = [
        codec for codec in codecs
        if mime_type(codec) != preferred_mime
        and mime_type(codec) != 'video/rtx'
    ]
    return primary + fallback


def _set_video_bandwidth(sdp: str, bitrate_kbps: int) -> str:
    """Advertise a bounded video receive bandwidth in an SDP answer."""
    bitrate = max(64, min(100000, int(bitrate_kbps)))
    newline = '\r\n' if '\r\n' in sdp else '\n'
    lines = sdp.replace('\r\n', '\n').split('\n')
    output = []
    in_video = False
    inserted = False
    for line in lines:
        if line.startswith('m='):
            in_video = line.startswith('m=video ')
            inserted = False
        if in_video and (
            line.startswith('b=AS:') or line.startswith('b=TIAS:')
        ):
            continue
        output.append(line)
        if in_video and not inserted and line.startswith('c='):
            output.append(f'b=AS:{bitrate}')
            output.append(f'b=TIAS:{bitrate * 1000}')
            inserted = True
    return newline.join(output)


def _output_dimensions(
    profile: VideoProfile,
    layout: str = 'stereo',
) -> tuple[int, int]:
    """Return the transport dimensions for one profile and source layout."""
    if layout == 'stereo':
        return profile.width, profile.height
    if layout == 'mono':
        return max(1, round(profile.height * 16.0 / 9.0)), profile.height
    if layout == 'top-bottom':
        return max(1, round(profile.height * 16.0 / 9.0)), profile.height * 2
    raise ValueError(f'unsupported video layout: {layout}')


def _fit_frame(
    frame: np.ndarray,
    profile: VideoProfile,
    layout: str = 'stereo',
) -> np.ndarray:
    """Letterbox one BGR frame to the layout-aware profile resolution."""
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError('stream frame must be a BGR HxWx3 array')
    source_height, source_width = frame.shape[:2]
    if source_width <= 0 or source_height <= 0:
        raise ValueError('stream frame must have positive dimensions')
    target_width, target_height = _output_dimensions(profile, layout)
    scale = min(target_width / source_width, target_height / source_height)
    width = max(1, int(round(source_width * scale)))
    height = max(1, int(round(source_height * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(frame, (width, height), interpolation=interpolation)
    output = np.zeros((target_height, target_width, 3), dtype=np.uint8)
    x = (target_width - width) // 2
    y = (target_height - height) // 2
    output[y:y + height, x:x + width] = resized
    return output


def _adapt_frame_layout(
    frame: np.ndarray,
    source_layout: str,
    output_layout: str,
) -> np.ndarray:
    """
    Convert a hub frame between mono and side-by-side layouts.

    The hub stores the source-native layout.  Native Unity clients may request
    a different presentation layout independently of browser clients, so crop
    the left eye for mono output or duplicate a mono eye for stereo output
    before profile letterboxing.  This avoids stretching an SBS frame into
    every eye of a head-mounted display.
    """
    source = str(source_layout).strip().lower()
    output = str(output_layout).strip().lower()
    if source not in ('mono', 'stereo', 'top-bottom'):
        raise ValueError(f'unsupported source video layout: {source_layout!r}')
    if output not in ('mono', 'stereo', 'top-bottom'):
        raise ValueError(f'unsupported output video layout: {output_layout!r}')
    if source == output:
        return frame
    if frame.ndim != 3 or frame.shape[1] <= 0:
        raise ValueError('stream frame must have positive dimensions')
    if source == 'stereo' and output == 'mono':
        midpoint = max(1, frame.shape[1] // 2)
        return np.ascontiguousarray(frame[:, :midpoint])
    if source == 'stereo' and output == 'top-bottom':
        midpoint = max(1, frame.shape[1] // 2)
        return np.ascontiguousarray(np.concatenate(
            (frame[:, :midpoint], frame[:, midpoint:]), axis=0))
    if source == 'top-bottom' and output == 'mono':
        midpoint = max(1, frame.shape[0] // 2)
        return np.ascontiguousarray(frame[:midpoint, :])
    if source == 'top-bottom' and output == 'stereo':
        midpoint = max(1, frame.shape[0] // 2)
        return np.ascontiguousarray(np.concatenate(
            (frame[:midpoint, :], frame[midpoint:, :]), axis=1))
    if source == 'mono' and output == 'stereo':
        return np.ascontiguousarray(np.concatenate((frame, frame), axis=1))
    if source == 'mono' and output == 'top-bottom':
        return np.ascontiguousarray(np.concatenate((frame, frame), axis=0))
    raise ValueError(
        f'cannot convert source layout {source_layout!r} to {output_layout!r}')


def _no_signal_frame(
    profile: VideoProfile,
    message='NO SIGNAL',
    layout: str = 'stereo',
) -> np.ndarray:
    """Generate a visible, changing frame instead of freezing old imagery."""
    width, height = _output_dimensions(profile, layout)
    output = np.zeros((height, width, 3), dtype=np.uint8)
    output[:] = (13, 16, 23)
    center = (width // 2, height // 2)
    scale = max(0.55, width / 1000.0)
    size, _ = cv2.getTextSize(message, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
    origin = (center[0] - size[0] // 2, center[1] - 12)
    cv2.putText(
        output, message, origin, cv2.FONT_HERSHEY_SIMPLEX,
        scale, (74, 109, 255), 2, cv2.LINE_AA,
    )
    timestamp = time.strftime('%H:%M:%S')
    cv2.putText(
        output, timestamp, (center[0] - 48, center[1] + 34),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (170, 177, 192), 1, cv2.LINE_AA,
    )
    return output


def _snapshot_age(snapshot) -> float:
    captured = getattr(snapshot, 'captured_monotonic', None)
    if captured is None:
        captured = getattr(snapshot, 'timestamp', None)
    if captured is None:
        return 0.0
    return max(0.0, time.monotonic() - float(captured))


class HubVideoTrack(VideoStreamTrack):
    """aiortc track that always consumes only the newest available frame."""

    kind = 'video'

    def __init__(
        self,
        hub,
        profile: VideoProfile,
        stale_after_sec: float,
        metrics,
        layout: str = 'stereo',
        source_layout: str = 'stereo',
    ):
        super().__init__()
        self._hub = hub
        self._profile = profile
        _output_dimensions(profile, layout)
        self._layout = layout
        self._source_layout = source_layout
        self._stale_after_sec = stale_after_sec
        self._metrics = metrics
        self._sequence = -1
        self._next_frame_time = time.monotonic()
        self._pts = 0

    async def recv(self):
        period = 1.0 / self._profile.fps
        delay = self._next_frame_time - time.monotonic()
        if delay > 0.0:
            await asyncio.sleep(delay)
        self._next_frame_time = max(
            self._next_frame_time + period, time.monotonic()
        )
        snapshot = await asyncio.to_thread(
            self._hub.wait_for_new, self._sequence, min(period, 0.10)
        )
        if snapshot is None:
            snapshot = self._hub.snapshot()
        fresh = snapshot is not None and _snapshot_age(snapshot) <= self._stale_after_sec
        if fresh:
            self._sequence = int(getattr(snapshot, 'sequence', self._sequence + 1))
            source_frame = _adapt_frame_layout(
                snapshot.frame,
                self._source_layout,
                self._layout,
            )
            image = _fit_frame(source_frame, self._profile, self._layout)
        else:
            image = _no_signal_frame(self._profile, layout=self._layout)
            self._metrics['no_signal_frames'] += 1
        frame = VideoFrame.from_ndarray(np.ascontiguousarray(image), format='bgr24')
        self._pts += max(1, int(round(90000.0 / self._profile.fps)))
        frame.pts = self._pts
        frame.time_base = VIDEO_TIME_BASE
        self._metrics['webrtc_frames'] += 1
        return frame


class EventLog:
    """Small append-only local JSONL runtime log."""

    def __init__(self, directory: str):
        self._path: Optional[Path] = None
        try:
            target = Path(directory).expanduser().resolve()
            target.mkdir(parents=True, exist_ok=True)
            suffix = time.strftime('%Y%m%d-%H%M%S')
            self._path = target / f'viewer-{suffix}.jsonl'
        except OSError as exc:
            LOGGER.warning('runtime log disabled: %s', exc)

    @property
    def path(self):
        return self._path

    def write(self, event: str, **values):
        if self._path is None:
            return
        record = {
            'time_unix': time.time(),
            'event': event,
            **values,
        }
        try:
            with self._path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(record, sort_keys=True) + '\n')
        except OSError as exc:
            LOGGER.warning('cannot append runtime log: %s', exc)
            self._path = None


class RobotPovWebServer:
    """Serve a video-only browser UI, signaling, status, and fallbacks."""

    def __init__(self, config: PovConfig, hub, web_root: Optional[Path] = None):
        if not config.video_only:
            raise ValueError('Robot POV server refuses non-video-only operation')
        self.config = config
        self.hub = hub
        # FrameProcessor composes two physical sources as side-by-side. The
        # requested top-bottom layout is a transport/presentation conversion,
        # so keep the hub's native layout explicit instead of treating an SBS
        # frame as already top-bottom.
        self.source_layout = (
            'mono'
            if config.layout == 'mono' or config.source == 'unitree'
            else 'stereo'
        )
        self.web_root = web_root or self._find_web_root()
        self.active_profile = config.profile
        self.started_monotonic = time.monotonic()
        self.runner = None
        self.site = None
        self.discovery_transport = None
        self.peer_connections: Set[RTCPeerConnection] = set()
        self.client_metrics: Dict[str, dict] = {}
        self.metrics = {
            'webrtc_connections': 0,
            'webrtc_reconnects': 0,
            'webrtc_frames': 0,
            'mjpeg_connections': 0,
            'mjpeg_frames': 0,
            'mjpeg_bytes': 0,
            'no_signal_frames': 0,
        }
        self._rate_samples = deque(maxlen=30)
        self._last_rate_time = time.monotonic()
        self._last_rate_bytes = 0
        self.event_log = EventLog(config.log_dir)
        self.app = self._create_app()

    @staticmethod
    def _find_web_root() -> Path:
        source_tree = Path(__file__).resolve().parents[1] / 'web'
        if source_tree.is_dir():
            return source_tree
        try:
            from ament_index_python.packages import get_package_share_directory

            installed = Path(get_package_share_directory('r1_robot_pov')) / 'web'
            if installed.is_dir():
                return installed
        except (ImportError, LookupError):
            pass
        raise FileNotFoundError('Robot POV web assets were not found')

    def _create_app(self):
        app = web.Application(client_max_size=64 * 1024)
        app.router.add_get('/', self._index)
        app.router.add_get('/app.js', self._static)
        app.router.add_get('/style.css', self._static)
        app.router.add_get('/healthz', self._health)
        app.router.add_get('/readyz', self._ready)
        app.router.add_get('/api/config', self._api_config)
        app.router.add_get('/api/status', self._api_status)
        app.router.add_post('/api/profile', self._api_profile)
        app.router.add_post('/api/client-metrics', self._client_metrics)
        app.router.add_post('/offer', self._offer)
        app.router.add_get('/stream.mjpg', self._mjpeg)
        app.router.add_get('/qr.png', self._qr)
        app.router.add_get('/favicon.ico', self._favicon)
        return app

    def _resolve_layout(self, requested) -> str:
        """Validate one per-client layout override against the source layout."""
        candidate = self.config.layout if requested in (None, '') else requested
        candidate = str(candidate).strip().lower()
        if candidate not in ('mono', 'stereo', 'top-bottom'):
            raise web.HTTPBadRequest(text='unknown video layout')
        return candidate

    async def _index(self, _request):
        return web.FileResponse(
            self.web_root / 'index.html', headers={'Cache-Control': 'no-store'}
        )

    async def _static(self, request):
        name = request.path.lstrip('/')
        if name not in ('app.js', 'style.css'):
            raise web.HTTPNotFound()
        return web.FileResponse(
            self.web_root / name, headers={'Cache-Control': 'no-cache'}
        )

    async def _favicon(self, _request):
        raise web.HTTPNoContent()

    async def _health(self, _request):
        return web.json_response({
            'ok': True,
            'service': 'r1_robot_pov',
            'video_only': True,
            'uptime_sec': round(time.monotonic() - self.started_monotonic, 3),
        })

    def _hub_status(self):
        try:
            result = dict(self.hub.status())
        except Exception as exc:  # pragma: no cover - defensive adapter boundary.
            result = {'connected': False, 'error': str(exc)}
        return result

    async def _ready(self, _request):
        status = self._hub_status()
        age_ms = status.get('frame_age_ms')
        fresh = bool(status.get('connected')) and age_ms is not None
        fresh = fresh and float(age_ms) <= self.config.stale_after_sec * 1000.0
        payload = {'ready': fresh, 'source': status}
        return web.json_response(payload, status=200 if fresh else 503)

    async def _api_config(self, _request):
        payload = self.config.public_dict()
        payload.update({
            'profile': self.active_profile,
            'webrtcAvailable': WEBRTC_AVAILABLE,
            'mjpegAvailable': True,
            'supportedLayouts': ['mono', 'stereo', 'top-bottom'],
            'nativeOffer': {
                'preferredCodecs': list(PREFERRED_CODECS),
                'defaultPreferredCodec': 'vp8',
                'clientTagMaxLength': CLIENT_TAG_MAX_LENGTH,
            },
        })
        return web.json_response(payload)

    def _rate_status(self):
        now = time.monotonic()
        elapsed = now - self._last_rate_time
        if elapsed >= 1.0:
            current = int(self.metrics['mjpeg_bytes'])
            kbps = (current - self._last_rate_bytes) * 8.0 / elapsed / 1000.0
            self._rate_samples.append(kbps)
            self._last_rate_time = now
            self._last_rate_bytes = current
        if not self._rate_samples:
            return 0.0
        return round(sum(self._rate_samples) / len(self._rate_samples), 1)

    async def _api_status(self, _request):
        clients = list(self.client_metrics.values())
        latest_client = clients[-1] if clients else {}
        payload = {
            'ok': True,
            'video_only': True,
            'mode': self.config.mode,
            'source': self._hub_status(),
            'profile': self.active_profile,
            'profile_config': PROFILES[self.active_profile].__dict__,
            'transport': self.config.transport,
            'webrtc_available': WEBRTC_AVAILABLE,
            'peers': len(self.peer_connections),
            'mjpeg_bitrate_kbps': self._rate_status(),
            'latest_client': latest_client,
            'metrics': dict(self.metrics),
            'uptime_sec': round(time.monotonic() - self.started_monotonic, 2),
        }
        return web.json_response(payload, headers={'Cache-Control': 'no-store'})

    async def _api_profile(self, request):
        body = await request.json()
        profile = str(body.get('profile', ''))
        if profile not in PROFILES:
            raise web.HTTPBadRequest(text='unknown profile')
        previous = self.active_profile
        self.active_profile = profile
        if previous != profile:
            self.event_log.write('profile_changed', previous=previous, profile=profile)
        return web.json_response({'ok': True, 'profile': profile})

    async def _client_metrics(self, request):
        body = await request.json()
        client_id = str(body.get('clientId', request.remote or 'unknown'))[:80]
        reconnects = _safe_number(body.get('reconnects')) or 0.0
        sanitized = {
            'client_id': client_id,
            'transport': str(body.get('transport', 'unknown'))[:20],
            'rendered_fps': _safe_number(body.get('renderedFps')),
            'latency_ms': _safe_number(body.get('latencyMs')),
            'rtt_ms': _safe_number(body.get('rttMs')),
            'jitter_ms': _safe_number(body.get('jitterMs')),
            'packets_lost': _safe_number(body.get('packetsLost')),
            'frames_dropped': _safe_number(body.get('framesDropped')),
            'frame_age_ms': _safe_number(body.get('frameAgeMs')),
            'reconnects': int(max(0, min(100000, reconnects))),
            'updated_unix': time.time(),
        }
        self.client_metrics[client_id] = sanitized
        while len(self.client_metrics) > 128:
            self.client_metrics.pop(next(iter(self.client_metrics)))
        return web.json_response({'ok': True})

    async def _offer(self, request):
        if not WEBRTC_AVAILABLE or self.config.transport == 'mjpeg':
            raise web.HTTPServiceUnavailable(text='WebRTC unavailable; use MJPEG')
        try:
            body = await request.json()
            preferred_codec, client_tag = _offer_preferences(body)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc
        if body.get('type') != 'offer' or not isinstance(body.get('sdp'), str):
            raise web.HTTPBadRequest(text='invalid SDP offer')
        profile_name = str(body.get('profile', self.active_profile))
        if profile_name not in PROFILES:
            raise web.HTTPBadRequest(text='unknown profile')
        layout = self._resolve_layout(body.get('layout'))
        configuration = RTCConfiguration(iceServers=[])
        peer = RTCPeerConnection(configuration=configuration)
        self.peer_connections.add(peer)
        self.metrics['webrtc_connections'] += 1
        if self.metrics['webrtc_connections'] > 1:
            self.metrics['webrtc_reconnects'] += 1
        track = HubVideoTrack(
            self.hub,
            PROFILES[profile_name],
            self.config.stale_after_sec,
            self.metrics,
            layout=layout,
            source_layout=self.source_layout,
        )
        sender = peer.addTrack(track)
        try:
            transceiver = next(
                item for item in peer.getTransceivers() if item.sender is sender
            )
            codecs = RTCRtpSender.getCapabilities('video').codecs
            preferred = _ordered_video_codecs(codecs, preferred_codec)
            if preferred:
                transceiver.setCodecPreferences(preferred)
        except (StopIteration, AttributeError):
            LOGGER.debug('codec preference API is unavailable')

        @peer.on('connectionstatechange')
        async def connection_state_changed():
            state = peer.connectionState
            self.event_log.write('webrtc_state', state=state, peer=request.remote)
            if state in ('failed', 'closed'):
                await peer.close()
                self.peer_connections.discard(peer)

        try:
            await peer.setRemoteDescription(
                RTCSessionDescription(sdp=body['sdp'], type=body['type'])
            )
            answer = await peer.createAnswer()
            await peer.setLocalDescription(answer)
        except Exception:
            self.peer_connections.discard(peer)
            await peer.close()
            raise
        self.event_log.write(
            'webrtc_offer',
            profile=profile_name,
            peer=request.remote,
            client=client_tag or 'browser',
            preferred_codec=preferred_codec or 'vp8',
        )
        answer_sdp = _set_video_bandwidth(
            peer.localDescription.sdp,
            PROFILES[profile_name].video_bitrate_kbps,
        )
        payload = {
            'sdp': answer_sdp,
            'type': peer.localDescription.type,
            'profile': profile_name,
            'preferredCodec': preferred_codec or 'vp8',
        }
        if client_tag is not None:
            payload['clientTag'] = client_tag
        return web.json_response(payload)

    async def _mjpeg(self, request):
        profile_name = str(request.query.get('profile', self.active_profile))
        if profile_name not in PROFILES:
            raise web.HTTPBadRequest(text='unknown profile')
        layout = self._resolve_layout(request.query.get('layout'))
        profile = PROFILES[profile_name]
        response = web.StreamResponse(
            status=200,
            headers={
                'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                'Cache-Control': 'no-store, no-cache, must-revalidate',
                'Pragma': 'no-cache',
                'X-Content-Type-Options': 'nosniff',
            },
        )
        await response.prepare(request)
        self.metrics['mjpeg_connections'] += 1
        self.event_log.write('mjpeg_connected', profile=profile_name, peer=request.remote)
        sequence = -1
        next_output = time.monotonic()
        try:
            while True:
                period = 1.0 / profile.fps
                delay = next_output - time.monotonic()
                if delay > 0.0:
                    await asyncio.sleep(delay)
                next_output = max(next_output + period, time.monotonic())
                snapshot = await asyncio.to_thread(
                    self.hub.wait_for_new, sequence, min(period, 0.10)
                )
                if snapshot is None:
                    snapshot = self.hub.snapshot()
                fresh = (
                    snapshot is not None
                    and _snapshot_age(snapshot) <= self.config.stale_after_sec
                )
                if fresh:
                    sequence = int(getattr(snapshot, 'sequence', sequence + 1))
                    source_frame = _adapt_frame_layout(
                        snapshot.frame,
                        self.source_layout,
                        layout,
                    )
                    frame = _fit_frame(
                        source_frame,
                        profile,
                        layout,
                    )
                else:
                    frame = _no_signal_frame(
                        profile,
                        layout=layout,
                    )
                    self.metrics['no_signal_frames'] += 1
                success, encoded = cv2.imencode(
                    '.jpg', frame,
                    [cv2.IMWRITE_JPEG_QUALITY, profile.jpeg_quality],
                )
                if not success:
                    continue
                payload = encoded.tobytes()
                part = (
                    b'--frame\r\nContent-Type: image/jpeg\r\n'
                    + f'Content-Length: {len(payload)}\r\n'.encode()
                    + f'X-Frame-Sequence: {sequence}\r\n\r\n'.encode()
                    + payload + b'\r\n'
                )
                await response.write(part)
                self.metrics['mjpeg_frames'] += 1
                self.metrics['mjpeg_bytes'] += len(payload)
        except (ConnectionError, asyncio.CancelledError, RuntimeError):
            pass
        finally:
            self.event_log.write('mjpeg_disconnected', peer=request.remote)
        return response

    async def _qr(self, request):
        if not QR_AVAILABLE:
            raise web.HTTPServiceUnavailable(text=self.config.numeric_url)
        requested = request.query.get('url', self.config.numeric_url)
        allowed = {self.config.numeric_url, self.config.mdns_url}
        if requested not in allowed:
            requested = self.config.numeric_url
        image = qrcode.make(requested)
        buffer = io.BytesIO()
        image.save(buffer, format='PNG')
        return web.Response(
            body=buffer.getvalue(),
            content_type='image/png',
            headers={'Cache-Control': 'no-store'},
        )

    def ssl_context(self):
        if not self.config.tls_cert:
            return None
        context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        context.load_cert_chain(
            str(Path(self.config.tls_cert).expanduser()),
            str(Path(self.config.tls_key).expanduser()),
        )
        return context

    async def start(self):
        """Bind the local server without starting any control subsystem."""
        self.runner = web.AppRunner(
            self.app,
            access_log=None,
            shutdown_timeout=SERVER_SHUTDOWN_TIMEOUT_SEC,
        )
        await self.runner.setup()
        self.site = web.TCPSite(
            self.runner,
            self.config.bind_host,
            self.config.port,
            ssl_context=self.ssl_context(),
        )
        await self.site.start()
        # The headset can be opened before the live bridge or robot is ready.
        # Keep discovery on the video-only process so it can always learn the
        # current laptop address and reconnect when Wi-Fi/DHCP changes.
        if self.config.discovery_enabled:
            try:
                loop = asyncio.get_running_loop()
                self.discovery_transport, _ = await loop.create_datagram_endpoint(
                    lambda: _DiscoveryProtocol(CONTROL_PORT),
                    local_addr=('0.0.0.0', DISCOVERY_PORT),
                    allow_broadcast=True,
                )
                self.event_log.write('discovery_started', port=DISCOVERY_PORT)
            except OSError as exc:
                # The live bridge owns the same discovery port in RUN mode.
                # Its responder is equivalent, so a collision must not take
                # the camera down when an older process races the handoff.
                LOGGER.info('video discovery responder unavailable: %s', exc)
                self.discovery_transport = None
        else:
            self.event_log.write('discovery_disabled', reason='live_bridge_owns_port')
        self.event_log.write(
            'server_started',
            config=self.config.redacted_dict(),
            numeric_url=self.config.numeric_url,
            mdns_url=self.config.mdns_url,
        )

    async def close(self):
        """Close media peers and HTTP sockets deterministically."""
        peers = tuple(self.peer_connections)
        self.peer_connections.clear()
        await asyncio.gather(*(peer.close() for peer in peers), return_exceptions=True)
        if self.discovery_transport is not None:
            self.discovery_transport.close()
            self.discovery_transport = None
        if self.runner is not None:
            await self.runner.cleanup()
            self.runner = None
        self.event_log.write('server_stopped')


class _DiscoveryProtocol(asyncio.DatagramProtocol):
    """Answer headset endpoint probes without exposing control state."""

    def __init__(self, video_port: int):
        self.video_port = int(video_port)
        self.transport = None

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, address):
        if self.transport is None or not isinstance(data, bytes):
            return
        if not data.startswith(DISCOVERY_PROBE_PREFIX):
            return
        nonce = data[len(DISCOVERY_PROBE_PREFIX):].strip()
        if not nonce or len(nonce) > 96 or any(byte > 0x7F for byte in nonce):
            return
        response = (
            f'{DISCOVERY_RESPONSE_PREFIX} {self.video_port} '
            f'{nonce.decode("ascii", errors="ignore")}\n'
        ).encode('ascii')
        try:
            self.transport.sendto(response, address)
        except OSError:
            pass


def _safe_number(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(number):
        return None
    return round(number, 3)
