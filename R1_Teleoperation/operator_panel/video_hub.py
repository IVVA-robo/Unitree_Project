"""Start the R1 camera service through Unitree's official RobotState API."""

from __future__ import annotations

import argparse
import time
from typing import Callable, Iterable


VIDEO_SERVICE = "video_hub"
CONFLICTING_SERVICES = ("stereo_patch_pc1",)
RUNNING_STATUS = 0
STOPPED_STATUS = 1


def _service_map(services: Iterable[object]) -> dict[str, object]:
    return {
        str(service.name): service
        for service in services
        if str(getattr(service, "name", "")).strip()
    }


def _read_services(client) -> dict[str, object]:
    code, services = client.ServiceList()
    if code != 0 or services is None:
        raise RuntimeError(f"ServiceList failed with code {code}")
    return _service_map(services)


def ensure_video_hub(
    client,
    *,
    timeout_s: float = 8.0,
    poll_interval_s: float = 0.25,
    sleep: Callable[[float], None] = time.sleep,
) -> tuple[bool, tuple[str, ...]]:
    """Ensure only the R1 video service is running.

    Returns ``(changed, stopped_conflicts)``.  The function only uses
    ``robot_state`` service management; it does not construct motion clients.
    """

    services = _read_services(client)
    video = services.get(VIDEO_SERVICE)
    if video is None:
        raise RuntimeError(f"{VIDEO_SERVICE} is not present in ServiceList")

    changed = False
    stopped_conflicts = []
    for name in CONFLICTING_SERVICES:
        service = services.get(name)
        if service is None or int(service.status) != RUNNING_STATUS:
            continue
        code = client.ServiceSwitch(name, False)
        if code != 0:
            raise RuntimeError(f"failed to stop {name}: code {code}")
        stopped_conflicts.append(name)
        changed = True

    if int(video.status) != RUNNING_STATUS:
        code = client.ServiceSwitch(VIDEO_SERVICE, True)
        if code != 0:
            raise RuntimeError(
                f"failed to start {VIDEO_SERVICE}: code {code}"
            )
        changed = True

    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while True:
        services = _read_services(client)
        video_running = (
            VIDEO_SERVICE in services
            and int(services[VIDEO_SERVICE].status) == RUNNING_STATUS
        )
        conflicts_stopped = all(
            name not in services
            or int(services[name].status) == STOPPED_STATUS
            for name in stopped_conflicts
        )
        if video_running and conflicts_stopped:
            return changed, tuple(stopped_conflicts)
        if time.monotonic() >= deadline:
            raise RuntimeError("video_hub did not reach running status")
        sleep(max(0.0, float(poll_interval_s)))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="enxb4b024be59fe")
    parser.add_argument("--rpc-timeout-sec", type=float, default=5.0)
    args = parser.parse_args(argv)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.robot_state.robot_state_client import (
        RobotStateClient,
    )

    ChannelFactoryInitialize(0, args.interface)
    client = RobotStateClient()
    client.SetTimeout(args.rpc_timeout_sec)
    client.Init()
    try:
        changed, stopped = ensure_video_hub(client)
    except Exception as exc:
        print(f"VIDEO_HUB state=ERROR detail={exc}")
        return 20
    print(
        "VIDEO_HUB state=RUNNING "
        f"changed={str(changed).lower()} "
        f"stopped_conflicts={','.join(stopped) or 'none'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
