"""Keep the read-only Robot POV path available over Pico USB.

The video tunnel is deliberately independent from the motion tunnel.  This
module may start the already installed headset application in USB mode and
may own only ``tcp:8080``.  It never starts the pose relay, a ROS graph, or a
robot writer.  ``usb_link.control`` continues to be the sole owner of
``tcp:19092`` and of every physical-control permission.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Callable, Mapping, Optional, Sequence

from exhibition.orchestrator import ExhibitionSettings

from .session import ACTIVITY, PACKAGE, run, usb_device


VIDEO_DEVICE_ENDPOINT = "tcp:8080"
VIDEO_HOST_ENDPOINT = "tcp:8080"
CONTROL_DEVICE_ENDPOINT = "tcp:19092"
CONTROL_HOST_ENDPOINT = "tcp:19092"
APP_MARKER = "usb-video-app.json"


def reverse_mappings(adb: Sequence[str], *, runner=None) -> dict[str, str]:
    """Return device-to-host ADB reverse mappings for one selected Pico."""

    runner = run if runner is None else runner
    result: dict[str, str] = {}
    for row in runner(*adb, "reverse", "--list").splitlines():
        fields = row.split()
        if len(fields) == 3:
            result[fields[1]] = fields[2]
    return result


def ensure_video_mapping(adb: Sequence[str], *, runner=None) -> str:
    """Create or reuse only the persistent read-only video tunnel."""

    runner = run if runner is None else runner
    existing = reverse_mappings(adb, runner=runner)
    target = existing.get(VIDEO_DEVICE_ENDPOINT)
    if target is not None and target != VIDEO_HOST_ENDPOINT:
        raise RuntimeError(
            f"USB port {VIDEO_DEVICE_ENDPOINT} belongs to another session"
        )
    if target == VIDEO_HOST_ENDPOINT:
        return "reused"
    runner(
        *adb,
        "reverse",
        "--no-rebind",
        VIDEO_DEVICE_ENDPOINT,
        VIDEO_HOST_ENDPOINT,
    )
    return "created"


def _read_marker(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _write_marker(path: Path, *, serial: str, pid: str) -> None:
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    payload = json.dumps(
        {"schema": 1, "serial": serial, "pid": pid, "usb_mode": True},
        sort_keys=True,
    )
    temporary.write_text(
        payload + "\n",
        encoding="utf-8",
    )
    os.chmod(temporary, 0o600)
    os.replace(temporary, path)


def _app_pid(adb: Sequence[str], *, runner=None) -> str:
    runner = run if runner is None else runner
    output = runner(*adb, "shell", "pidof", PACKAGE, check=False)
    candidates = output.split()
    return candidates[0] if candidates and candidates[0].isdigit() else ""


def ensure_usb_app(
    adb: Sequence[str],
    runtime_dir: Path,
    serial: str,
    *,
    runner=None,
    sleeper: Callable[[float], None] = time.sleep,
) -> str:
    """Ensure that the current APK process was launched with ``r1_usb``.

    Android does not apply the launch extra when an operator reopens an
    already running Unity activity.  A private marker ties the USB launch to
    the exact Android PID, so healthy launches are reused while a manual
    LAN-mode relaunch is corrected once.
    """

    runner = run if runner is None else runner
    marker_path = runtime_dir / APP_MARKER
    current_pid = _app_pid(adb, runner=runner)
    marker = _read_marker(marker_path)
    marker_matches = all(
        (
            current_pid,
            marker.get("usb_mode") is True,
            marker.get("serial") == serial,
            marker.get("pid") == current_pid,
        )
    )
    if marker_matches:
        return "reused"

    runner(*adb, "shell", "am", "force-stop", PACKAGE)
    runner(*adb, "shell", "input", "keyevent", "KEYCODE_WAKEUP")
    runner(
        *adb,
        "shell",
        "am",
        "start",
        "-n",
        ACTIVITY,
        "--ez",
        "r1_usb",
        "true",
    )
    current_pid = ""
    for _ in range(20):
        current_pid = _app_pid(adb, runner=runner)
        if current_pid:
            break
        sleeper(0.1)
    if not current_pid:
        raise RuntimeError("Pico USB application did not start")
    _write_marker(marker_path, serial=serial, pid=current_pid)
    return "started"


def ensure_background_video(
    environment: Optional[Mapping[str, str]] = None,
    *,
    runner=None,
    device_resolver=None,
    sleeper: Callable[[float], None] = time.sleep,
) -> int:
    """One non-blocking repair pass for idle/static USB video."""

    environment = dict(os.environ if environment is None else environment)
    if environment.get("R1_VR_TRANSPORT", "lan").strip().lower() != "usb":
        print("USB_VIDEO state=SKIPPED detail=transport_lan", flush=True)
        return 0

    settings = ExhibitionSettings.from_environment(environment)
    settings.runtime_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    lock_path = settings.runtime_dir / "usb-transport.lock"
    with lock_path.open("a+") as transport_lock:
        try:
            fcntl.flock(transport_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("USB_VIDEO state=OWNED detail=control_session", flush=True)
            return 0

        resolver = usb_device if device_resolver is None else device_resolver
        try:
            serial = resolver(environment.get("R1_VR_ADB_SERIAL"))
        except (RuntimeError, OSError, subprocess.SubprocessError) as error:
            print(f"USB_VIDEO state=WAITING detail={error}", flush=True)
            return 0

        runner = run if runner is None else runner
        adb = ["adb", "-s", serial]
        existing = reverse_mappings(adb, runner=runner)
        if CONTROL_DEVICE_ENDPOINT in existing:
            raise RuntimeError(
                "USB control tunnel exists without the transport owner lock"
            )
        mapping = ensure_video_mapping(adb, runner=runner)
        application = ensure_usb_app(
            adb,
            settings.runtime_dir,
            serial,
            runner=runner,
            sleeper=sleeper,
        )
        print(
            f"USB_VIDEO state=READY mapping={mapping} app={application}",
            flush=True,
        )
        return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["ensure"])
    parser.parse_args(sys.argv[1:] if argv is None else argv)
    try:
        return ensure_background_video()
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(f"[BLOCKED] USB video: {error}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    sys.exit(main())
