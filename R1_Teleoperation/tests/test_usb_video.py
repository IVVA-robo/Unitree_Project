"""USB video stays read-only and independent from the control tunnel."""

import fcntl
import json

import pytest

from usb_link import video


def test_video_mapping_reuses_exact_persistent_tunnel():
    calls = []

    def runner(*args, **_kwargs):
        calls.append(args)
        return "UsbFfs tcp:8080 tcp:8080"

    state = video.ensure_video_mapping(["adb", "-s", "pico"], runner=runner)
    assert state == "reused"
    assert len(calls) == 1


def test_video_mapping_rejects_foreign_target_before_write():
    calls = []

    def runner(*args, **_kwargs):
        calls.append(args)
        return "UsbFfs tcp:8080 tcp:18080"

    with pytest.raises(RuntimeError, match="another session"):
        video.ensure_video_mapping(["adb"], runner=runner)
    assert len(calls) == 1


def test_usb_app_marker_reuses_exact_android_process(tmp_path):
    marker = tmp_path / video.APP_MARKER
    marker.write_text(
        json.dumps(
            {"schema": 1, "serial": "pico", "pid": "42", "usb_mode": True}
        )
    )
    calls = []

    def runner(*args, **_kwargs):
        calls.append(args)
        return "42"

    state = video.ensure_usb_app(["adb"], tmp_path, "pico", runner=runner)
    assert state == "reused"
    assert len(calls) == 1
    assert "force-stop" not in calls[0]


def test_manual_app_launch_is_restarted_once_in_usb_mode(tmp_path):
    calls = []
    pid_reads = iter(["77", "88"])

    def runner(*args, **_kwargs):
        calls.append(args)
        if "pidof" in args:
            return next(pid_reads)
        return ""

    assert video.ensure_usb_app(
        ["adb"], tmp_path, "pico", runner=runner, sleeper=lambda _seconds: None
    ) == "started"
    assert any("force-stop" in call for call in calls)
    start = next(call for call in calls if "start" in call)
    assert start[-3:] == ("--ez", "r1_usb", "true")
    marker = json.loads((tmp_path / video.APP_MARKER).read_text())
    assert marker == {
        "schema": 1,
        "serial": "pico",
        "pid": "88",
        "usb_mode": True,
    }


def test_background_helper_never_competes_with_control_owner(tmp_path):
    environment = {
        "R1_VR_TRANSPORT": "usb",
        "R1_EXHIBITION_RUNTIME_DIR": str(tmp_path),
    }
    lock_path = tmp_path / "usb-transport.lock"
    lock_path.touch()
    with lock_path.open("a+") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert video.ensure_background_video(
            environment,
            runner=lambda *_args, **_kwargs: pytest.fail(
                "ADB must not run"
            ),
            device_resolver=lambda _serial: pytest.fail(
                "Pico must not be probed"
            ),
        ) == 0


def test_background_helper_refuses_orphan_control_tunnel(tmp_path):
    environment = {
        "R1_VR_TRANSPORT": "usb",
        "R1_EXHIBITION_RUNTIME_DIR": str(tmp_path),
    }

    def runner(*args, **_kwargs):
        if args[-2:] == ("reverse", "--list"):
            return "UsbFfs tcp:19092 tcp:19092"
        return ""

    with pytest.raises(RuntimeError, match="without the transport owner lock"):
        video.ensure_background_video(
            environment,
            runner=runner,
            device_resolver=lambda _serial: "pico",
        )
