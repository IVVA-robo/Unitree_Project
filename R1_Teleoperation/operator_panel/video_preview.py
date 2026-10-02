"""Low-latency MJPEG preview embedded in the Qt operator panel.

Robot POV already exposes a browser-compatible MJPEG fallback.  Keeping the
desktop preview on that read-only endpoint avoids adding a second camera
client or any robot-control dependency to the panel.
"""

from __future__ import annotations

import threading
import os
import json
import time
from urllib.request import urlopen
from urllib.error import HTTPError
from urllib.parse import parse_qsl, urlencode, urljoin, urlsplit, urlunsplit

import cv2

# The desktop has the full OpenCV wheel, whose import points Qt at OpenCV's
# private plugin directory.  That directory contains an incompatible xcb
# plugin and makes a normal systemd/desktop launch abort before QApplication
# is created.  OpenCV is used only for decoding here, so restore Qt's normal
# plugin discovery when the wheel injected its private path.
for _qt_variable in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR"):
    if "cv2" in os.environ.get(_qt_variable, ""):
        os.environ.pop(_qt_variable, None)

from PyQt5.QtCore import QThread, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QLabel, QSizePolicy, QVBoxLayout, QWidget


def mjpeg_url(base_url: str, profile: str = "low-latency") -> str:
    """Return the Robot POV MJPEG endpoint for a configured viewer URL."""

    base = str(base_url).strip() or "http://127.0.0.1:8080/"
    endpoint = urljoin(base if base.endswith("/") else base + "/", "stream.mjpg")
    parts = urlsplit(endpoint)
    query = dict(parse_qsl(parts.query, keep_blank_values=True))
    query.update({"profile": profile or "low-latency", "layout": "mono"})
    return urlunsplit((parts.scheme, parts.netloc, parts.path, urlencode(query), ""))


def readyz_url(base_url: str) -> str:
    """Return the Robot POV readiness endpoint for a viewer or stream URL."""

    base = str(base_url).strip() or "http://127.0.0.1:8080/"
    parts = urlsplit(base)
    path = parts.path or "/"
    if path.endswith("/stream.mjpg"):
        path = path[: -len("stream.mjpg")]
    elif not path.endswith("/"):
        path += "/"
    endpoint = urljoin(urlunsplit((parts.scheme, parts.netloc, path, "", "")), "readyz")
    return endpoint


def video_unavailable_text(payload: object) -> str:
    """Turn the local readiness response into a useful operator instruction."""

    try:
        source = payload["source"]  # type: ignore[index]
        last_error = str(source.get("last_error") or "")
        if not last_error:
            sources = source.get("sources") or {}
            last_error = " ".join(
                str(details.get("last_error") or details.get("error") or "")
                for details in sources.values()
                if isinstance(details, dict)
            )
    except (AttributeError, KeyError, TypeError):
        last_error = ""
    if "3102" in last_error:
        return (
            "Проводной видеосервис R1 не отвечает — повторяю подключение "
            "по Ethernet (3102)…"
        )
    return "Камера не найдена — переподключаюсь…"


def video_payload_ready(payload: object) -> bool:
    """Return true only for a real, fresh camera frame."""

    if not isinstance(payload, dict):
        return False
    source = payload.get("source")
    if isinstance(source, dict) and (
        source.get("has_frame") is False or source.get("stale") is True
    ):
        return False
    if payload.get("ready") is True:
        return True
    return bool(
        isinstance(source, dict)
        and source.get("has_frame") is True
        and source.get("stale") is False
    )


class _MjpegWorker(QThread):
    frame_ready = pyqtSignal(QImage)
    state_changed = pyqtSignal(str, str)

    def __init__(self, url: str, readiness_url: str, parent=None):
        super().__init__(parent)
        self.url = url
        self.readiness_url = readiness_url
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def _readiness(self) -> tuple[bool, str]:
        try:
            try:
                response = urlopen(self.readiness_url, timeout=0.5)
            except HTTPError as error:
                # /readyz deliberately returns 503 with camera diagnostics.
                # urllib raises before entering the context, but its response
                # body is still readable and must not be discarded.
                if error.code != 503:
                    error.close()
                    raise
                response = error
            with response:
                payload = json.loads(response.read().decode("utf-8"))
        except (OSError, UnicodeError, ValueError):
            payload = None
        return video_payload_ready(payload), video_unavailable_text(payload)

    def run(self) -> None:  # noqa: D401 - Qt worker entry point
        """Reconnect until stopped and emit GUI-owned image copies."""

        self.state_changed.emit("Поиск видеопотока…", "searching")
        while not self._stop.is_set():
            capture = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                capture.release()
                _ready, unavailable = self._readiness()
                self.state_changed.emit(unavailable, "offline")
                self._stop.wait(0.75)
                continue

            source_ready, unavailable = self._readiness()
            current_state = "online" if source_ready else "offline"
            self.state_changed.emit(
                "Видео с камеры работает" if source_ready else unavailable,
                current_state,
            )
            next_readiness_check = 0.0
            consecutive_failures = 0
            while not self._stop.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    consecutive_failures += 1
                    if consecutive_failures >= 3:
                        break
                    continue
                consecutive_failures = 0
                now = time.monotonic()
                if now >= next_readiness_check:
                    source_ready, unavailable = self._readiness()
                    new_state = "online" if source_ready else "offline"
                    if new_state != current_state:
                        self.state_changed.emit(
                            "Видео с камеры работает"
                            if source_ready
                            else unavailable,
                            new_state,
                        )
                        current_state = new_state
                    next_readiness_check = now + 0.5
                if not source_ready:
                    continue
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                height, width, channels = rgb.shape
                image = QImage(
                    rgb.data,
                    width,
                    height,
                    channels * width,
                    QImage.Format_RGB888,
                ).copy()
                self.frame_ready.emit(image)
            capture.release()
            if not self._stop.is_set():
                _ready, unavailable = self._readiness()
                self.state_changed.emit(unavailable, "offline")
                self._stop.wait(0.35)
        self.state_changed.emit("Предпросмотр остановлен", "stopped")


class VideoPreview(QWidget):
    """Aspect-preserving preview with an explicit reconnect status."""

    state_changed = pyqtSignal(str, str)

    def __init__(self, base_url: str, profile: str, parent=None):
        super().__init__(parent)
        self._stream_url = mjpeg_url(base_url, profile)
        self._readiness_url = readyz_url(base_url)
        self._worker = None
        self._last_image = QImage()
        self.setObjectName("videoPreview")
        # Keep the exhibition dashboard compact enough for a normal desktop
        # window while allowing the preview to expand when space is available.
        self.setMinimumSize(480, 270)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.image_label = QLabel("Видео подключится автоматически")
        self.image_label.setObjectName("videoCanvas")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(480, 270)
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.image_label, 1)
        self.status_label = QLabel("Ожидание Robot POV")
        self.status_label.setObjectName("videoStatus")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

    @property
    def stream_url(self) -> str:
        return self._stream_url

    def start(self) -> None:
        if self._worker is not None and self._worker.isRunning():
            return
        self._worker = _MjpegWorker(
            self._stream_url,
            self._readiness_url,
            self,
        )
        self._worker.frame_ready.connect(self._show_frame)
        self._worker.state_changed.connect(self._show_state)
        self._worker.start()

    def stop(self, wait_ms: int = 1500) -> None:
        worker = self._worker
        if worker is None:
            return
        worker.stop()
        worker.wait(max(0, int(wait_ms)))

    def _show_frame(self, image: QImage) -> None:
        self._last_image = image
        self._render_image()

    def _show_state(self, text: str, state: str) -> None:
        if state != "online":
            # Do not leave an old camera image looking like a live picture
            # while the stream contains NO SIGNAL or is reconnecting.
            self._last_image = QImage()
            self.image_label.clear()
            self.image_label.setText(text)
        self.status_label.setText(text)
        self.status_label.setProperty("videoState", state)
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)
        self.state_changed.emit(text, state)

    def _render_image(self) -> None:
        if self._last_image.isNull():
            return
        pixmap = QPixmap.fromImage(self._last_image)
        self.image_label.setPixmap(
            pixmap.scaled(
                self.image_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt API name
        super().resizeEvent(event)
        self._render_image()
