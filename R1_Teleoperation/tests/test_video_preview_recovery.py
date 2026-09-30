import io
import json
import os
from urllib.error import HTTPError, URLError

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtGui import QImage
from PyQt5.QtWidgets import QApplication

import operator_panel.video_preview as preview


def test_readyz_503_retains_camera_diagnostic(monkeypatch):
    payload = {"ready": False, "source": {
        "has_frame": False,
        "last_error": "GetImageSample failed with code 3102",
    }}
    body = io.BytesIO(json.dumps(payload).encode())

    def unavailable(url, **_kwargs):
        raise HTTPError(url, 503, "not ready", {}, body)

    monkeypatch.setattr(preview, "urlopen", unavailable)
    worker = preview._MjpegWorker("http://test/stream.mjpg", "http://test/readyz")
    ready, message = worker._readiness()
    assert ready is False
    assert "3102" in message
    assert "Ethernet" in message
    assert body.closed


@pytest.mark.parametrize("status", [401, 404, 500])
def test_other_http_errors_do_not_report_ready(monkeypatch, status):
    body = io.BytesIO(b'{"ready":true}')

    def unavailable(url, **_kwargs):
        raise HTTPError(url, status, "error", {}, body)

    monkeypatch.setattr(preview, "urlopen", unavailable)
    worker = preview._MjpegWorker("http://test/stream.mjpg", "http://test/readyz")
    assert worker._readiness()[0] is False
    assert body.closed


def test_unreachable_readyz_does_not_report_ready(monkeypatch):
    def unavailable(*_args, **_kwargs):
        raise URLError("connection refused")

    monkeypatch.setattr(preview, "urlopen", unavailable)
    worker = preview._MjpegWorker("http://test/stream.mjpg", "http://test/readyz")
    assert worker._readiness()[0] is False


@pytest.mark.parametrize("source", [
    {"has_frame": False},
    {"has_frame": True, "stale": True},
])
def test_stale_or_placeholder_source_overrides_inconsistent_ready(source):
    assert preview.video_payload_ready({"ready": True, "source": source}) is False


def test_disconnect_clears_last_live_image():
    application = QApplication.instance() or QApplication([])
    widget = preview.VideoPreview("http://test/", "low-latency")
    widget._show_frame(QImage(8, 8, QImage.Format_RGB888))
    assert not widget._last_image.isNull()
    widget._show_state("Камера переподключается", "offline")
    assert widget._last_image.isNull()
    assert widget.image_label.text() == "Камера переподключается"
    widget.close()
    assert application is not None
