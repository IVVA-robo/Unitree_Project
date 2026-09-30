from types import SimpleNamespace

import pytest

from operator_panel.video_hub import ensure_video_hub


class FakeRobotStateClient:
    def __init__(self, states):
        self.states = dict(states)
        self.switches = []

    def ServiceList(self):
        services = [
            SimpleNamespace(name=name, status=status, protect=0)
            for name, status in self.states.items()
        ]
        return 0, services

    def ServiceSwitch(self, name, switch):
        self.switches.append((name, switch))
        self.states[name] = 0 if switch else 1
        return 0


def test_ensure_video_hub_starts_video_and_stops_known_conflict():
    client = FakeRobotStateClient(
        {"video_hub": 1, "stereo_patch_pc1": 0, "ai_sport": 0}
    )

    changed, stopped = ensure_video_hub(client, sleep=lambda _delay: None)

    assert changed is True
    assert stopped == ("stereo_patch_pc1",)
    assert client.switches == [
        ("stereo_patch_pc1", False),
        ("video_hub", True),
    ]
    assert client.states["ai_sport"] == 0


def test_ensure_video_hub_is_idempotent_when_camera_is_running():
    client = FakeRobotStateClient(
        {"video_hub": 0, "stereo_patch_pc1": 1}
    )

    changed, stopped = ensure_video_hub(client, sleep=lambda _delay: None)

    assert changed is False
    assert stopped == ()
    assert client.switches == []


def test_ensure_video_hub_rejects_missing_service():
    client = FakeRobotStateClient({"robot_state": 0})

    with pytest.raises(RuntimeError, match="not present"):
        ensure_video_hub(client, sleep=lambda _delay: None)
