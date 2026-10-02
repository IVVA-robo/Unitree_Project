import importlib.machinery
import importlib.util
import json
from pathlib import Path
import stat
import time


PROJECT_DIR = Path(__file__).resolve().parents[1]


def _load_module():
    path = PROJECT_DIR / "scripts" / "r1-sdk-preflight-attestation"
    loader = importlib.machinery.SourceFileLoader(
        "r1_sdk_preflight_attestation_test", str(path)
    )
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def _prepare(monkeypatch, tmp_path):
    module = _load_module()
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    monkeypatch.setenv("R1_SDK_PREFLIGHT_ATTEST_MAX_AGE_SEC", "3600")
    link = {
        "carrier": "1",
        "address": "b4:b0:24:be:59:fe",
        "rx_errors": 0,
        "rx_length_errors": 0,
    }
    monkeypatch.setattr(module, "boot_id", lambda: "test-boot")
    monkeypatch.setattr(module, "current_link", lambda: dict(link))
    monkeypatch.setattr(module, "known_writer_running", lambda: False)
    monkeypatch.setattr(module, "run_quiet", lambda _argv: True)
    return module, link


def test_fresh_private_attestation_is_reusable(monkeypatch, tmp_path):
    module, _link = _prepare(monkeypatch, tmp_path)

    assert module.record() == 0
    path = module.runtime_path()
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert module.check() == 0


def test_link_error_change_invalidates_attestation(monkeypatch, tmp_path):
    module, link = _prepare(monkeypatch, tmp_path)
    assert module.record() == 0

    link["rx_errors"] = 1

    assert module.check() == 3


def test_stale_or_competing_writer_invalidates_attestation(monkeypatch, tmp_path):
    module, _link = _prepare(monkeypatch, tmp_path)
    assert module.record() == 0
    path = module.runtime_path()
    data = json.loads(path.read_text(encoding="utf-8"))
    data["recorded_at"] = time.time() - 3601
    path.write_text(json.dumps(data), encoding="utf-8")
    path.chmod(0o600)
    assert module.check() == 3

    data["recorded_at"] = time.time()
    path.write_text(json.dumps(data), encoding="utf-8")
    path.chmod(0o600)
    monkeypatch.setattr(module, "known_writer_running", lambda: True)
    assert module.check() == 3


def test_warmup_and_cached_live_path_remain_read_only_contracts():
    warmup = (PROJECT_DIR / "scripts" / "r1-sdk-warmup").read_text()
    live = (PROJECT_DIR / "scripts" / "r1-live-session").read_text()

    assert "r1-sdk-preflight" in warmup
    assert "r1-sdk-preflight-attestation" in warmup
    assert warmup.index('"${SCRIPT_DIR}/r1-sdk-preflight-attestation" invalidate') \
        < warmup.index("echo 'SDK_WARMUP state=WAITING_FOR_ROBOT'")
    assert "cache_invalidated_after_link_loss" in warmup
    assert "SDK_WARMUP state=REFRESHING detail=read_only_overlap" in warmup
    refresh_block = warmup.split(
        "SDK_WARMUP state=REFRESHING detail=read_only_overlap", 1
    )[0].rsplit("if [[ ${cache_link_ok} == true ]]", 1)[1]
    assert "invalidate_attestation" not in refresh_block
    assert "REFRESH_SEC=${R1_SDK_WARMUP_REFRESH_SEC:-120}" in warmup
    assert "r1_resolve_control_ip" in warmup
    assert "[r]1_live_writer_node" in warmup
    assert "r1-sdk-preflight-attestation\" check" in live
    for forbidden in (
        "/r1/safety/set_kill",
        "/r1/live_writer/reset_kill",
        "SetVelocity(",
        "StandUp(",
    ):
        assert forbidden not in warmup


def test_cold_preflight_uses_coherent_ros_snapshots_and_fast_gate_has_margin():
    preflight = (PROJECT_DIR / "scripts" / "r1-sdk-preflight").read_text()
    prepare = (PROJECT_DIR / "scripts" / "r1-robot-prepare").read_text()

    assert "ros2 param dump /r1_sdk_transport" in preflight
    assert "ros2 param get /r1_sdk_transport" not in preflight
    assert "ros2 topic list --no-daemon" in preflight
    assert "arm_sdk_gate_duration=2.25" in prepare


def test_cold_preflight_checks_ros2_only_after_desktop_environment_bootstrap():
    preflight = (PROJECT_DIR / "scripts" / "r1-sdk-preflight").read_text()

    ros_source = preflight.index("source /opt/ros/humble/setup.bash")
    ros_check = preflight.index("command -v ros2")
    assert ros_source < ros_check
    initial_requirements = preflight.split("source /opt/ros/humble/setup.bash", 1)[0]
    assert "for required in ip ros2" not in initial_requirements
