from pathlib import Path


SOURCE_DIR = Path(__file__).resolve().parents[1] / "src"


def test_battery_observer_is_read_only():
    source = (SOURCE_DIR / "r1_battery_observer.cpp").read_text(encoding="utf-8")
    assert "CreateRecvChannel" in source
    assert "BmsState_" in source
    assert "CreateSendChannel" not in source
    assert "ChannelPublisher" not in source
    assert "Client" not in source


def test_app_command_observer_is_read_only_jsonl_capture():
    source = (SOURCE_DIR / "r1_app_command_observer.cpp").read_text(
        encoding="utf-8"
    )
    assert "CreateRecvChannel<Request>" in source
    assert "rt/api/sport/request" in source
    assert "api_id" in source
    assert "binary_size" in source
    assert "CreateSendChannel" not in source
    assert "ChannelPublisher" not in source
    assert "Client" not in source
