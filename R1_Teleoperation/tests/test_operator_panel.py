import os
import json
import subprocess

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication  # noqa: E402

import operator_panel.app as app_module  # noqa: E402
from operator_panel.app import (  # noqa: E402
    EXHIBITION_GRACEFUL_STOP_MS,
    OperatorPanel,
    SettingsDialog,
)
from operator_panel.commands import (  # noqa: E402
    command_catalog,
    required_make_targets,
)
from operator_panel.config import (  # noqa: E402
    OperatorConfig,
    load_config,
    save_config,
)
from operator_panel.video_preview import (  # noqa: E402
    mjpeg_url,
    readyz_url,
    video_payload_ready,
    video_unavailable_text,
)


@pytest.fixture(autouse=True)
def isolated_exhibition_discovery(monkeypatch):
    # Unit tests must never discover/control the real desktop's live manager.
    monkeypatch.setattr(app_module, "active_session_snapshot", lambda: {})


def test_mjpeg_url_preserves_host_and_selects_low_latency_mono():
    assert mjpeg_url("http://127.0.0.1:8080/") == (
        "http://127.0.0.1:8080/stream.mjpg?profile=low-latency&layout=mono"
    )
    assert mjpeg_url("http://robot.local:9000/view", "exhibition") == (
        "http://robot.local:9000/view/stream.mjpg?profile=exhibition&layout=mono"
    )


def test_video_readiness_hint_explains_unitree_client_send_error():
    assert readyz_url("http://127.0.0.1:8080/") == (
        "http://127.0.0.1:8080/readyz"
    )
    assert readyz_url(mjpeg_url("http://robot.local:9000/view")) == (
        "http://robot.local:9000/view/readyz"
    )
    text = video_unavailable_text(
        {"source": {"last_error": "GetImageSample failed with code 3102"}}
    )
    assert "Проводной видеосервис" in text
    assert "3102" in text


def test_video_preview_requires_a_real_fresh_source_frame():
    assert video_payload_ready({"ready": True}) is True
    assert video_payload_ready(
        {"ready": False, "source": {"has_frame": True, "stale": False}}
    ) is True
    assert video_payload_ready(
        {"ready": False, "source": {"has_frame": False, "stale": True}}
    ) is False


def test_catalog_contains_operator_actions_and_existing_targets():
    specs = {spec.key: spec for spec in command_catalog()}
    for key in (
        "check_all",
        "network",
        "robot_check",
        "stop",
        "kill",
        "zero_torque",
        "arms_live",
        "arms_running_live",
        "legs_live",
        "teleop_live",
        "video_robot",
        "video_stereo",
        "exhibition_static",
        "exhibition_control",
        "exhibition_lock",
        "exhibition_reconnect",
        "exhibition_calibrate",
        "exhibition_stop",
    ):
        assert key in specs
    assert "robot-stop" in required_make_targets()
    assert "robot-kill" in required_make_targets()
    assert "robot-zero-torque" in required_make_targets()
    assert specs["exhibition_static"].target == "exhibition-static"
    assert specs["exhibition_static"].long_running is True
    assert specs["exhibition_control"].target == "exhibition-control"
    assert specs["exhibition_control"].long_running is True
    assert specs["exhibition_lock"].target == "exhibition-lock"
    assert specs["exhibition_reconnect"].target == "exhibition-reconnect"
    assert specs["exhibition_calibrate"].target == "exhibition-calibrate-arms"
    assert specs["exhibition_stop"].target == "exhibition-stop"


def test_default_environment_is_fail_closed():
    env = OperatorConfig().as_environment()
    assert env["ROBOT_DRY_RUN"] == "1"
    assert env["ROBOT_ENABLE_ACTUATION"] == "0"
    assert env["ROBOT_CONFIRM_COMMISSIONING"] == "0"
    assert env["R1_ROBOT_IP"] == "192.168.123.164"
    assert env["R1_CONTROL_IP"] == "192.168.123.161"
    assert env["R1_LIVE_CONTROL_IP"] == "192.168.123.161"
    assert env["R1_CONTROL_IP_CANDIDATES"] == "192.168.123.161"
    assert env["ROBOT_POV_ROBOT_IP"] == "192.168.123.161"
    assert env["ROBOT_POV_ROBOT_IP_CANDIDATES"] == "192.168.123.161"
    assert env["R1_LIVE_ROS_DOMAIN_ID"] == "88"
    assert env["R1_TELEOP_HARDWARE_DOMAIN_ID"] == "88"
    assert env["R1_SDK_ALLOW_HALF_DUPLEX"] == "1"
    assert OperatorConfig().robot_name == "R1_03079"
    assert OperatorConfig().auto_start_bridge is True
    assert OperatorConfig().video_url == "http://127.0.0.1:8080/"
    assert env["R1_OPERATOR_REQUIRE_PREFLIGHT"] == "1"
    assert env["R1_SHOULDER_HEIGHT_OFFSET_M"] == "0"
    assert env["R1_SHOULDER_FORWARD_OFFSET_M"] == "-0.02"
    assert env["R1_SHOULDER_WIDTH_M"] == "0.4"
    assert env["R1_ARM_MOTION_SCALE"] == "0.5"
    assert env["R1_TURN_SENSITIVITY"] == "1"
    assert env["R1_LEG_SPEED_SCALE"] == "1"


def test_exhibition_tuning_roundtrips_and_rejects_unsafe_values(tmp_path):
    path = tmp_path / "operator.json"
    expected = OperatorConfig(
        response_profile='standard',
        shoulder_height_offset_m=0.04,
        shoulder_forward_offset_m=-0.05,
        shoulder_width_m=0.46,
        arm_motion_scale=0.85,
        turn_sensitivity=0.55,
        leg_speed_scale=0.60,
        allow_half_duplex_adapter=True,
    )
    save_config(expected, path)
    loaded = load_config(path)
    assert loaded.response_profile == 'standard'
    assert loaded.as_environment()['R1_RESPONSE_PROFILE'] == 'standard'

    assert loaded.shoulder_height_offset_m == pytest.approx(0.04)
    assert loaded.shoulder_forward_offset_m == pytest.approx(-0.05)
    assert loaded.shoulder_width_m == pytest.approx(0.46)
    assert loaded.arm_motion_scale == pytest.approx(0.85)
    assert loaded.turn_sensitivity == pytest.approx(0.55)
    assert loaded.leg_speed_scale == pytest.approx(0.60)
    assert loaded.allow_half_duplex_adapter is True
    assert loaded.as_environment()["R1_SDK_ALLOW_HALF_DUPLEX"] == "1"
    assert loaded.as_environment()["R1_TURN_SENSITIVITY"] == "0.55"

    with pytest.raises(ValueError, match="turn_sensitivity"):
        OperatorConfig(turn_sensitivity=1.01).as_environment()
    with pytest.raises(ValueError, match="leg_speed_scale"):
        OperatorConfig(leg_speed_scale=float("nan")).as_environment()
    with pytest.raises(ValueError, match='response_profile'):
        OperatorConfig(response_profile='unbounded').as_environment()


def test_settings_dialog_exposes_bounded_exhibition_tuning():
    app = QApplication.instance() or QApplication([])
    dialog = SettingsDialog(OperatorConfig())

    assert dialog.shoulder_height_offset.minimum() == pytest.approx(-0.15)
    assert dialog.shoulder_height_offset.maximum() == pytest.approx(0.15)
    assert dialog.shoulder_forward_offset.minimum() == pytest.approx(-0.12)
    assert dialog.shoulder_forward_offset.maximum() == pytest.approx(0.08)
    assert dialog.shoulder_width.minimum() == pytest.approx(0.25)
    assert dialog.shoulder_width.maximum() == pytest.approx(0.55)
    assert dialog.arm_motion_scale.maximum() == pytest.approx(1.20)
    assert dialog.turn_sensitivity.maximum() == pytest.approx(1.0)
    assert dialog.leg_speed_scale.maximum() == pytest.approx(1.0)
    assert dialog.allow_half_duplex_adapter.isChecked() is True
    assert dialog.response_profile.currentData() == 'exhibition'
    dialog.response_profile.setCurrentIndex(dialog.response_profile.findData('standard'))
    assert dialog.values().response_profile == 'standard'

    dialog.close()
    app.processEvents()


def test_live_actions_are_marked_and_reset_kill_uses_reviewed_rearm():
    specs = {spec.key: spec for spec in command_catalog()}
    assert specs["arms_live"].live
    assert specs["arms_running_live"].live
    assert specs["arms_running_live"].target == "arms-running-live"
    assert specs["legs_live"].live
    assert specs["teleop_live"].live
    assert specs["reset_kill"].implemented is True
    assert specs["reset_kill"].target == "exhibition-rearm"
    assert specs["reset_kill"].live is True
    assert specs["hands_calibrate"].implemented is True
    assert specs["hands_calibrate"].target == "arms-calibrate"
    assert specs["hands_calibrate"].live is False
    assert specs["head_calibrate"].implemented is True
    assert specs["head_calibrate"].target == "head-calibrate"
    assert specs["head_calibrate"].live is False


def test_panel_live_environment_is_ephemeral_and_complete():
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.config = OperatorConfig(
        dry_run=False,
        allow_live=True,
        vr_headset_ip="192.168.8.129",
    )
    panel.live_session_token = "panel-session-token-123456"

    environment = panel._live_environment()

    assert environment == {
        "R1_VR_TRANSPORT": "lan",
        "ROBOT_DRY_RUN": "0",
        "ROBOT_ENABLE_ACTUATION": "1",
        "ROBOT_CONFIRM_OFF_CHARGER": "1",
        "ROBOT_CONFIRM_CLEAR_AREA": "1",
        "ROBOT_CONFIRM_ESTOP_READY": "1",
        "ROBOT_CONFIRM_COMMISSIONING": "1",
        "ROBOT_COMMISSIONING_TOKEN": "panel-session-token-123456",
        "ROBOT_VR_SOURCE_IP": "192.168.8.129",
    }

    legs_environment = panel._live_environment(run_mode_confirmed=True)
    assert legs_environment["ROBOT_CONFIRM_RUN_MODE"] == "1"
    assert legs_environment["ROBOT_CONFIRM_NO_PHONE_CONTROL"] == "1"


def test_latched_kill_remains_visible_without_closing_fresh_preflight_gate():
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.status_values = {}
    panel.preflight_ok = True

    panel._parse_status_output("STATUS kill=LATCHED\n")

    assert panel.status_values["kill"] == "LATCHED"
    assert panel.preflight_ok is True


def test_panel_status_keeps_hmd_hands_and_sticks_indicators():
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.status_values = {}
    panel._refresh_exhibition_status = lambda: None

    panel._parse_status_output(
        "STATUS hmd=OK\nSTATUS hands=RECOVERING\nSTATUS sticks=OFFLINE\n"
    )

    assert panel.status_values == {
        "hmd": "OK",
        "hands": "RECOVERING",
        "sticks": "OFFLINE",
    }


def test_stop_completion_closes_motion_graph_and_requires_fresh_preflight():
    stopped = []

    class Controller:
        def stop(self, key, **kwargs):
            stopped.append((key, kwargs))

        def is_running(self, _key):
            return False

        def active_keys(self):
            return []

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.preflight_ok = True
    panel.close_after_stop = False
    panel._append_log = lambda *_args: None
    panel._refresh_summary = lambda: None

    panel._on_finished("stop", 0, 0)

    assert stopped == [
        ("motion", {"wait": False}),
        ("robot:prepare", {"wait": False}),
        (
            "exhibition",
            {
                "graceful_timeout_ms": EXHIBITION_GRACEFUL_STOP_MS,
                "wait": False,
            },
        ),
    ]
    assert panel.preflight_ok is False


def test_exhibition_home_keeps_lock_run_and_adds_separate_stand_button():
    app = QApplication.instance() or QApplication([])
    panel = OperatorPanel(OperatorConfig(status_poll_sec=60))
    panel.status_timer.stop()

    assert panel.tabs.tabText(0) == "Главный экран"
    assert panel.tabs.tabText(1) == "Расширенные настройки / Сервис"
    assert panel.static_mode_button.text().startswith("LOCK / СТАТИЧНЫЙ РЕЖИМ")
    assert panel.control_mode_button.text().startswith("RUN / ПОЛНОЕ УПРАВЛЕНИЕ")
    assert panel.stand_mode_button.text().startswith("СТОЙКА")
    assert panel.zero_torque_button.text().startswith("ZERO TORQUE / РАССЛАБИТЬ")
    assert panel.zero_torque_button.objectName() == "zeroTorqueButton"
    assert panel.service_zero_torque_button.objectName() == "zeroTorqueServiceButton"
    assert panel.specs['exhibition_stand'].target == 'exhibition-static'
    assert panel.specs['exhibition_stand'].requires_run_mode is False
    assert panel.tabs.tabBar().isHidden()
    assert panel.connect_button.text() == "↻  НАЙТИ И ПОДКЛЮЧИТЬ"
    assert panel.fullscreen_button.text() == "□"
    assert panel.fullscreen_button.objectName() == "fullscreenButton"
    assert panel.fullscreen_shortcut.key().toString() == "F11"
    assert all(
        "калибровать руки" not in button.text().lower()
        and "калибровать hmd и руки" not in button.text().lower()
        for button in panel.findChildren(app_module.QPushButton)
    )
    assert panel.robot_name_label.text() == "○  R1_03079  — не в сети"
    assert panel.video_preview.stream_url.endswith(
        "/stream.mjpg?profile=low-latency&layout=mono"
    )
    assert "stop_button" not in panel.__dict__
    assert "kill_button" not in panel.__dict__
    assert "B на правом контроллере" in panel.controller_action_hint.text()
    assert "X на левом контроллере" in panel.controller_action_hint.text()
    assert panel.zero_torque_button.parentWidget() is not panel.centralWidget()
    assert "Аварийное" in [
        panel.service_tabs.tabText(index)
        for index in range(panel.service_tabs.count())
    ]
    assert panel.service_tabs.count() >= 7

    panel.close()
    app.processEvents()


def test_fullscreen_toggle_does_not_change_operator_mode():
    app = QApplication.instance() or QApplication([])
    panel = OperatorPanel(OperatorConfig(status_poll_sec=60))
    panel.status_timer.stop()
    initial_mode = panel.exhibition_mode
    panel.show()
    app.processEvents()

    panel.toggle_fullscreen()
    app.processEvents()
    assert panel.isFullScreen()
    assert panel.fullscreen_button.text() == "❐"

    panel.exit_fullscreen()
    app.processEvents()
    assert not panel.isFullScreen()
    assert panel.fullscreen_button.text() == "□"
    assert panel.exhibition_mode == initial_mode

    panel.fullscreen_button.click()
    app.processEvents()
    assert panel.isMaximized()
    assert panel.fullscreen_button.text() == "❐"

    panel.fullscreen_button.click()
    app.processEvents()
    assert not panel.isMaximized()
    assert panel.fullscreen_button.text() == "□"
    panel.close()
    app.processEvents()


def test_main_reconnect_without_owner_never_changes_robot_mode(monkeypatch):
    calls = []

    class Controller:
        def is_running(self, _key):
            return False

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.exhibition_requested_mode = ""
    panel._begin_action = lambda key: calls.append(("begin", key)) or True
    panel._ensure_main_services = lambda: calls.append(("services",))
    panel._append_log = lambda *args: calls.append(("log", args[0]))
    panel._request_status_refresh = lambda **kwargs: calls.append(("status", kwargs))
    panel._run_exhibition_mode = lambda key: calls.append(key)

    panel.run_key("exhibition_reconnect")

    assert calls == [
        ("begin", "exhibition_reconnect"),
        ("services",),
        ("log", "exhibition_reconnect"),
        ("status", {"fast": True}),
    ]


@pytest.mark.parametrize("connected", [False, True])
def test_auto_connect_reuses_status_and_never_runs_full_check(connected):
    calls = []
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.config = OperatorConfig(vr_transport="usb")
    panel.connection_state = "connected" if connected else "offline"
    panel.status_values = {"robot": "OK" if connected else "OFFLINE"}
    panel._begin_action = lambda key: calls.append(("begin", key)) or True
    panel.start_background_services = lambda: calls.append(("services",))
    panel._request_status_refresh = lambda **kwargs: calls.append(("status", kwargs))
    panel._refresh_summary = lambda: None
    panel._finish_action = lambda *args: calls.append(("finish", *args))
    panel._run_check_all = lambda: pytest.fail("full preflight must stay manual")

    panel.auto_connect()

    assert calls[:3] == [
        ("begin", "connect"),
        ("services",),
        ("status", {"fast": True}),
    ]
    if connected:
        assert calls[-1][0:2] == ("finish", "connect")


def test_open_viewer_reuses_local_stream_without_auto_connect(monkeypatch):
    calls = []
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.config = OperatorConfig(video_url="http://127.0.0.1:8080/")
    panel.video_state = "online"
    panel._begin_action = lambda key: calls.append(("begin", key)) or True
    panel._ensure_main_services = lambda: calls.append(("services",))
    panel._finish_action = lambda *args: calls.append(("finish", *args))
    panel.auto_connect = lambda *args, **kwargs: pytest.fail("viewer must not run discovery")
    monkeypatch.setattr(app_module.QDesktopServices, "openUrl", lambda _url: True)

    panel.open_viewer()

    assert calls == [
        ("begin", "viewer"),
        ("services",),
        (
            "finish",
            "viewer",
            "готово",
            "использован уже работающий локальный поток",
        ),
    ]


def test_structured_manager_state_updates_mode_without_poll_delay():
    finished = []
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.output_line_buffers = {}
    panel.status_values = {}
    panel.action_started_at = {"exhibition_control": 1.0}
    panel._finish_action = lambda *args: finished.append(args)
    panel._refresh_exhibition_status = lambda: None

    panel._consume_exhibition_events(
        "exhibition",
        'EXHIBITION_STATE {"mode":"control","status":"ready",'
        '"detail":"warm graph"}\n',
    )

    assert panel.exhibition_mode == "Управление"
    assert panel.status_values["exhibition"] == "ready"
    assert finished == [("exhibition_control", "готово", "warm graph")]


def test_fragmented_sdk_warmup_ready_triggers_clean_handoff():
    stops = []

    class Controller:
        def stop(self, key, **kwargs):
            stops.append((key, kwargs))

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.output_line_buffers = {}
    panel.status_values = {"sdk_warmup": "CHECKING"}
    panel.sdk_warmup_cache_available = False
    panel.pending_warmup_key = "exhibition_control"
    panel.warmup_handoff_waiting = True
    panel._append_log = lambda *_args: None

    panel._consume_sdk_warmup_events("SDK_WARMUP state=REA")
    assert panel.status_values["sdk_warmup"] == "CHECKING"
    assert stops == []

    panel._consume_sdk_warmup_events("DY\n")
    assert panel.status_values["sdk_warmup"] == "READY"
    assert panel.sdk_warmup_cache_available is True
    assert panel.warmup_handoff_waiting is False
    assert stops == [
        ("sdk_warmup", {"graceful_timeout_ms": 12000, "wait": False})
    ]


def test_fragmented_early_manager_error_is_kept_for_button_result():
    finished = []
    retry_reasons = []
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.output_line_buffers = {}
    panel.last_process_errors = {}
    panel.status_values = {}
    panel.last_status = "Зелёный"
    panel.exhibition_requested_mode = "exhibition_control"
    panel.exhibition_switch_in_progress = False
    panel.pending_exhibition_key = None
    panel.exhibition_mode = "Подготовка"
    panel.close_after_stop = False
    panel._append_log = lambda *_args: None
    panel._refresh_summary = lambda: None
    panel._show_retry_reason = (
        lambda action, reason: retry_reasons.append((action, reason))
    )
    panel._finish_action = lambda *args: finished.append(args)

    panel._consume_process_diagnostics("exhibition", "[BLOC")
    panel._consume_process_diagnostics(
        "exhibition", "KED] USB RUN: Pico command timed out\n"
    )
    panel._on_finished("exhibition", 2, 0)

    reason = "USB RUN: Pico command timed out"
    assert panel.last_process_errors["exhibition"] == reason
    assert retry_reasons == [("exhibition_control", reason)]
    assert finished == [("exhibition_control", "ошибка", reason)]


def test_button_timing_records_monotonic_elapsed(monkeypatch):
    ticks = iter((10.0, 10.125))
    monkeypatch.setattr(app_module.time, "monotonic", lambda: next(ticks))
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.action_started_at = {}
    panel.action_generations = {}
    panel.last_action_timings = {}

    assert panel._begin_action("viewer") is True
    assert panel._finish_action("viewer", "готово", "test") == pytest.approx(0.125)
    assert panel.last_action_timings["viewer"] == {
        "elapsed_sec": 0.125,
        "result": "готово",
        "detail": "test",
    }


def test_hidden_main_calibration_never_starts_control_owner():
    calls = []

    class Controller:
        def is_running(self, _key):
            return False

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.exhibition_requested_mode = ""
    panel._append_log = lambda *args: calls.append(("blocked", args[0]))
    panel._run_exhibition_mode = lambda key: calls.append(key)

    panel.run_key("exhibition_calibrate")

    assert calls == [("blocked", "exhibition_calibrate")]


def test_main_calibration_reuses_existing_control_owner(monkeypatch):
    calls = []

    class Controller:
        def is_running(self, key):
            return key == "exhibition"

        def start(self, key, spec, **kwargs):
            calls.append((key, spec.key, kwargs))

    monkeypatch.setattr(app_module, "active_session_snapshot", lambda: {})
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.exhibition_requested_mode = "exhibition_control"

    panel.run_key("exhibition_calibrate")

    assert calls == [
        ("exhibition_service:exhibition_calibrate", "exhibition_calibrate", {}),
    ]


def test_exhibition_status_is_short_and_russian():
    app = QApplication.instance() or QApplication([])
    panel = OperatorPanel(OperatorConfig(status_poll_sec=60))
    panel.status_timer.stop()
    panel.status_values.update({
        "robot": "OK",
        "vr": "OK",
        "controllers": "RECOVERING",
        "video": "OFFLINE",
    })
    panel.exhibition_mode = "Управление"
    panel._refresh_exhibition_status()

    assert panel.exhibition_robot_status.text() == "● Робот найден"
    assert panel.exhibition_vr_status.text() == "● Очки найдены"
    assert "восстанавливаются" in panel.exhibition_controllers_status.text()
    assert panel.exhibition_video_status.text() == "○ Нет видео"
    assert panel.exhibition_mode_status.text() == "Режим: Управление"
    assert panel.robot_name_label.text() == "●  R1_03079    ETHERNET"
    assert panel.connect_button.text() == "✓  ПОДКЛЮЧЕНО"

    panel.close()
    app.processEvents()


def test_pc2_alone_is_not_reported_as_a_connected_robot():
    app = QApplication.instance() or QApplication([])
    panel = OperatorPanel(OperatorConfig(status_poll_sec=60))
    panel.status_timer.stop()
    panel.status_values.update({"robot": "OFFLINE", "pc2": "OK"})
    panel._refresh_exhibition_status()

    assert "PC2 по кабелю" in panel.exhibition_robot_status.text()
    assert "PC2 по кабелю" in panel.robot_name_label.text()
    assert panel.connect_button.text() == "…  УПРАВЛЕНИЕ НЕ В СЕТИ"
    assert panel.connection_state == "searching"

    panel.close()
    app.processEvents()


def test_exhibition_status_shows_latched_kill_over_stale_ready_mode():
    app = QApplication.instance() or QApplication([])
    panel = OperatorPanel(OperatorConfig(status_poll_sec=60))
    panel.status_timer.stop()
    panel.status_values.update({
        "robot": "OK",
        "mode": "control",
        "kill": "LATCHED",
    })
    panel._refresh_exhibition_status()

    assert (
        panel.exhibition_mode_status.text()
        == "Режим: Управление остановлено (KILL)"
    )

    panel.close()
    app.processEvents()


def test_exhibition_status_shows_latched_kill_after_lock_aliases_mode_static():
    app = QApplication.instance() or QApplication([])
    panel = OperatorPanel(OperatorConfig(status_poll_sec=60))
    panel.status_timer.stop()
    panel.status_values.update({
        "robot": "OK",
        "mode": "static",
        "exhibition": "degraded",
        "kill": "LATCHED",
    })
    panel._refresh_exhibition_status()

    assert (
        panel.exhibition_mode_status.text()
        == "Режим: Управление остановлено (KILL)"
    )

    panel.close()
    app.processEvents()


def test_exhibition_status_identifies_right_b_emergency_stop():
    app = QApplication.instance() or QApplication([])
    panel = OperatorPanel(OperatorConfig(status_poll_sec=60))
    panel.status_timer.stop()
    panel.status_values.update({
        "robot": "OK",
        "mode": "control",
        "kill": "LATCHED",
        "emergency": "RIGHT_B",
        "sequential_phase": "walking",
    })
    panel._refresh_exhibition_status()

    assert panel.exhibition_mode_status.text() == (
        "Режим: Аварийная остановка нажата на правом контроллере B"
    )
    assert panel.operator_instruction.text() == (
        "Аварийная остановка нажата на правом контроллере B"
    )

    panel.close()
    app.processEvents()


def test_sequential_phase_is_visible_without_overriding_emergency():
    app = QApplication.instance() or QApplication([])
    panel = OperatorPanel(OperatorConfig(status_poll_sec=60))
    panel.status_timer.stop()
    panel.status_values.update({"mode": "control", "kill": "CLEAR"})
    phases = {
        "arms": "Управление руками и головой",
        "releasing_arms": "Передача управления для ходьбы…",
        "waiting_811": "Ожидание режима ходьбы…",
        "walking": "Ходьба — руки и голова под управлением робота",
        "stopping": "Остановка перед возвратом VR…",
    }
    for phase, label in phases.items():
        panel.status_values["sequential_phase"] = phase
        panel._refresh_exhibition_status()
        assert panel.exhibition_mode_status.text() == f"Режим: {label}"
    assert "отпустите стик" in panel.controller_action_hint.text()
    panel.close()
    app.processEvents()


def test_exhibition_mode_uses_one_slot_and_stops_before_switch():
    calls = []

    class Controller:
        running = {"exhibition"}

        def is_running(self, key):
            return key in self.running

        def start(self, key, spec, **kwargs):
            calls.append((key, spec.key, kwargs))
            self.running.add(key)
            return True

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.exhibition_requested_mode = "exhibition_static"
    panel.exhibition_session_confirmed = True
    panel.exhibition_switch_in_progress = False
    panel.pending_exhibition_key = None
    panel.pending_exhibition_environment = None
    panel.exhibition_mode = "Статичный"
    panel._confirm_live = lambda *_args, **_kwargs: True
    panel._live_environment = lambda **_kwargs: {"ROBOT_ENABLE_ACTUATION": "1"}
    panel._append_log = lambda *_args: None
    panel._refresh_summary = lambda: None

    panel._run_exhibition_mode("exhibition_control")

    assert calls == [("exhibition_stop", "exhibition_stop", {})]
    assert panel.pending_exhibition_key == "exhibition_control"
    assert panel.exhibition_switch_in_progress is True


def test_repeated_run_rearms_the_existing_control_graph():
    calls = []

    class Controller:
        def is_running(self, key):
            return key == "exhibition"

        def start(self, key, spec, **kwargs):
            calls.append((key, spec.key, kwargs))
            return True

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.exhibition_requested_mode = "exhibition_control"
    panel.exhibition_mode = "Управление"
    panel._live_environment = lambda **_kwargs: {"TOKEN": "same-session"}
    panel._refresh_summary = lambda: None

    panel._run_exhibition_mode("exhibition_control")

    assert calls == [
        (
            "exhibition_rearm",
            "exhibition_rearm",
            {"env_overrides": {"TOKEN": "same-session"}},
        )
    ]
    assert panel.exhibition_mode == "Повторный запуск…"


@pytest.mark.parametrize('key,action', [
    ('exhibition_static', 'exhibition_lock'),
    ('exhibition_control', 'exhibition_rearm'),
])
def test_reopened_panel_reuses_external_manager(monkeypatch, key, action):
    calls = []
    monkeypatch.setattr(app_module, 'active_session_snapshot', lambda: {
        'mode': 'control', 'session_mode': 'session_arm', 'status': 'locked',
    })

    class Controller:
        def is_running(self, key):
            return False

        def start(self, key, spec, **kwargs):
            calls.append((key, spec.key, kwargs))

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel._refresh_summary = lambda: None
    panel._run_exhibition_mode(key)
    assert calls == [(action, action, {})]
    assert panel.exhibition_requested_mode == 'exhibition_control'


def test_fresh_exhibition_run_skips_legacy_checkbox_confirmation():
    confirmations = []
    starts = []

    class Controller:
        def is_running(self, _key):
            return False

        def stop(self, *_args, **_kwargs):
            raise AssertionError("fresh RUN must not stop a process")

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.exhibition_session_confirmed = False
    panel.exhibition_requested_mode = ""
    panel.exhibition_mode = "Остановлен"
    panel._confirm_live = lambda *_args, **kwargs: (
        confirmations.append(kwargs) or True
    )
    panel._live_environment = lambda **_kwargs: {"TOKEN": "one-click"}
    panel._start_exhibition_mode = (
        lambda key, environment: starts.append((key, environment))
    )

    panel._run_exhibition_mode("exhibition_control")

    assert confirmations[0]["skip_checklist"] is True
    assert panel.exhibition_session_confirmed is True
    assert starts == [
        ("exhibition_control", {"TOKEN": "one-click"})
    ]


@pytest.mark.parametrize(
    'completion', ['clean', 'failed', 'stop', 'new_destination', 'owner_changed']
)
def test_fresh_run_handoff_is_async_and_waits_for_sdk_warmup_cleanup(
    monkeypatch, completion
):
    starts = []
    stops = []
    timers = []

    class Controller:
        running = {'sdk_warmup'}

        def is_running(self, key):
            return key in self.running

        def stop(self, key, **kwargs):
            stops.append((key, kwargs))

        def start(self, key, spec, **kwargs):
            self.running.add(key)
            return True

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.exhibition_session_confirmed = False
    panel.exhibition_requested_mode = ""
    panel.exhibition_mode = "Остановлен"
    panel.status_values = {"sdk_warmup": "CHECKING"}
    panel.pending_warmup_key = None
    panel.pending_warmup_environment = None
    panel.warmup_handoff_waiting = False
    panel.warmup_handoff_generation = 0
    panel.sdk_warmup_cache_available = False
    panel._confirm_live = lambda *_args, **_kwargs: True
    panel._live_environment = lambda **_kwargs: {"TOKEN": "warm-cache"}
    panel._start_exhibition_mode = (
        lambda key, environment: starts.append((key, environment))
    )
    panel._refresh_summary = lambda: None
    panel._append_log = lambda *args: None
    from types import SimpleNamespace
    monkeypatch.setattr(
        app_module,
        'QTimer',
        SimpleNamespace(singleShot=lambda _ms, fn: timers.append(fn)),
    )

    panel._run_exhibition_mode("exhibition_control")

    assert stops == []
    assert len(timers) == 1
    assert starts == []
    assert panel.pending_warmup_key == 'exhibition_control'
    panel._start_pending_warmup_mode()
    assert starts == []
    # READY preserves the completed cache but still requests confirmed worker
    # cleanup before the physical manager can start.
    panel._on_output('sdk_warmup', 'SDK_WARMUP state=READY\n')
    assert stops == [
        ("sdk_warmup", {"graceful_timeout_ms": 12000, "wait": False})
    ]
    assert starts == []
    destination = 'exhibition_control'
    if completion == 'new_destination':
        destination = 'exhibition_stand'
        panel._run_exhibition_mode(destination)
        assert len(stops) == 1  # no second TERM while cleanup is in progress
    elif completion == 'stop':
        panel.run_key('exhibition_stop')
    panel.controller.running.clear()
    if completion == 'owner_changed':
        monkeypatch.setattr(app_module, 'active_session_snapshot', lambda: {'mode': 'static'})
    monkeypatch.setattr(app_module, 'QTimer', SimpleNamespace(singleShot=lambda _ms, fn: fn()))
    panel._append_log = lambda *args: None
    panel.close_after_stop = False
    panel._on_finished('sdk_warmup', 1 if completion == 'failed' else 0, 0)
    assert starts == ([(destination, {"TOKEN": "warm-cache"})]
                      if completion in {'clean', 'new_destination'} else [])
    assert panel.pending_warmup_key is None
    assert panel.pending_warmup_environment is None


def test_inflight_warmup_handoff_timeout_falls_back_asynchronously(monkeypatch):
    stops = []
    timers = []

    class Controller:
        def is_running(self, key):
            return key == 'sdk_warmup'

        def stop(self, key, **kwargs):
            stops.append((key, kwargs))

    from types import SimpleNamespace
    monkeypatch.setattr(
        app_module,
        'QTimer',
        SimpleNamespace(singleShot=lambda _ms, fn: timers.append(fn)),
    )
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.exhibition_session_confirmed = False
    panel.exhibition_requested_mode = ''
    panel.exhibition_mode = 'Остановлен'
    panel.status_values = {'sdk_warmup': 'CHECKING'}
    panel.pending_warmup_key = None
    panel.pending_warmup_environment = None
    panel.warmup_handoff_waiting = False
    panel.warmup_handoff_generation = 0
    panel.sdk_warmup_cache_available = False
    panel._confirm_live = lambda *_args, **_kwargs: True
    panel._live_environment = lambda **_kwargs: {'TOKEN': 'bounded'}
    panel._refresh_summary = lambda: None
    panel._append_log = lambda *_args: None

    panel._run_exhibition_mode('exhibition_stand')
    assert stops == []
    assert len(timers) == 1
    timers[0]()
    assert stops == [
        ('sdk_warmup', {'graceful_timeout_ms': 12000, 'wait': False})
    ]


def test_refreshing_warmup_with_valid_cache_stops_without_waiting(monkeypatch):
    stops = []
    timers = []

    class Controller:
        def is_running(self, key):
            return key == 'sdk_warmup'

        def stop(self, key, **kwargs):
            stops.append((key, kwargs))

    from types import SimpleNamespace
    monkeypatch.setattr(
        app_module,
        'QTimer',
        SimpleNamespace(singleShot=lambda _ms, fn: timers.append(fn)),
    )
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.exhibition_session_confirmed = False
    panel.exhibition_requested_mode = ''
    panel.exhibition_mode = 'Остановлен'
    panel.status_values = {'sdk_warmup': 'CHECKING'}
    panel.pending_warmup_key = None
    panel.pending_warmup_environment = None
    panel.warmup_handoff_waiting = False
    panel.warmup_handoff_generation = 0
    panel.sdk_warmup_cache_available = True
    panel._confirm_live = lambda *_args, **_kwargs: True
    panel._live_environment = lambda **_kwargs: {'TOKEN': 'cached'}
    panel._refresh_summary = lambda: None
    panel._append_log = lambda *_args: None

    panel._run_exhibition_mode('exhibition_control')
    assert timers == []
    assert stops == [
        ('sdk_warmup', {'graceful_timeout_ms': 12000, 'wait': False})
    ]


def test_usb_button_never_waits_for_adb_on_the_ui_thread(monkeypatch):
    import usb_link.session
    monkeypatch.setattr(
        usb_link.session,
        'usb_device',
        lambda *args: pytest.fail('blocking ADB in Qt thread'),
    )
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.config = OperatorConfig(
        vr_transport='usb', dry_run=False, allow_live=True
    )
    panel.preflight_ok = False
    spec = {spec.key: spec for spec in command_catalog()}['exhibition_control']
    assert panel._confirm_live(
        spec, automatic_preflight=True, require_vr=True, skip_checklist=True
    )


def test_run_to_lock_pauses_existing_control_graph_without_stopping_it():
    calls = []

    class Controller:
        def is_running(self, key):
            return key == "exhibition"

        def start(self, key, spec, **kwargs):
            calls.append((key, spec.key, kwargs))
            return True

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.exhibition_requested_mode = "exhibition_control"
    panel.exhibition_mode = "Управление"
    panel.status_values = {"mode": "control"}
    panel._refresh_summary = lambda: None

    panel._run_exhibition_mode("exhibition_static")

    assert calls == [
        ("exhibition_lock", "exhibition_lock", {})
    ]
    assert panel.exhibition_mode == "Переход в LOCK…"


@pytest.mark.parametrize('owned', [False, True])
@pytest.mark.parametrize('source_mode,key', [
    ('control', 'exhibition_stand'), ('static', 'exhibition_control'),
])
def test_stand_switch_waits_for_owner_cleanup(monkeypatch, owned, source_mode, key):
    calls = []
    confirmations = []
    monkeypatch.setattr(app_module, 'active_session_snapshot', lambda: {
        'mode': source_mode,
        'session_mode': (
            'session_arm' if source_mode == 'control' else 'deadman'
        ),
        'status': 'ready',
    })

    class Controller:
        def is_running(self, key):
            return owned and key == 'exhibition'

        def start(self, key, spec, **kwargs):
            calls.append((key, spec.key))
            return True

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.exhibition_requested_mode = (
        'exhibition_control'
        if source_mode == 'control'
        else 'exhibition_stand'
    )
    panel.status_values = {'mode': source_mode}
    panel._refresh_summary = lambda: None
    panel._append_log = lambda *args: None
    panel._confirm_live = lambda spec, **kwargs: confirmations.append(kwargs) or True
    panel._live_environment = lambda **kwargs: {'TEST': 'authorized'}
    panel._run_exhibition_mode(key)
    assert calls == [('exhibition_stop', 'exhibition_stop')]
    assert panel.pending_exhibition_key == key
    assert panel.exhibition_switch_in_progress is True
    assert confirmations[0]['require_vr'] is (key == 'exhibition_control')
    assert confirmations[0]['skip_checklist'] is True


@pytest.mark.parametrize(
    ("key", "expected_mode"),
    [
        ("exhibition_stand", "Стойка"),
        ("exhibition_static", "Статичный"),
    ],
)
def test_repeated_static_action_does_not_rearm_run(
    monkeypatch, key, expected_mode
):
    monkeypatch.setattr(app_module, 'active_session_snapshot', lambda: {
        'mode': 'static', 'session_mode': 'deadman', 'status': 'ready',
    })
    panel = OperatorPanel.__new__(OperatorPanel)
    starts = []

    class Controller:
        def is_running(self, _key):
            return False

        def start(self, *args, **kwargs):
            starts.append((args, kwargs))
            return True

    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel._refresh_summary = lambda: None
    finished = []
    panel._begin_action = lambda action: True
    panel._finish_action = lambda *args: finished.append(args)

    panel._run_exhibition_mode(key)

    assert panel.exhibition_mode == expected_mode
    assert starts == []
    assert finished[0][0:2] == (key, "уже готово")


def test_switch_destination_can_change_but_explicit_stop_cannot_queue_run():
    from types import SimpleNamespace
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = SimpleNamespace(
        is_running=lambda key: key in {'exhibition', 'exhibition_stop'}
    )
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel._append_log = lambda *args: None
    confirmations = []
    panel._confirm_live = lambda *args, **kwargs: confirmations.append(kwargs) or True
    panel._live_environment = lambda **kwargs: {'TEST': 'fresh-ack'}
    panel.pending_exhibition_key = 'exhibition_stand'
    panel.exhibition_switch_in_progress = True
    panel._run_exhibition_mode('exhibition_control')
    assert panel.pending_exhibition_key == 'exhibition_control'
    assert panel.pending_exhibition_environment == {'TEST': 'fresh-ack'}
    assert len(confirmations) == 1
    panel.pending_exhibition_key = None
    panel._run_exhibition_mode('exhibition_control')
    assert panel.pending_exhibition_key is None
    assert len(confirmations) == 1


def test_mode_button_during_zero_torque_never_queues_activation():
    from types import SimpleNamespace
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = SimpleNamespace(is_running=lambda key: key == 'zero_torque')
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel._append_log = lambda *args: None
    panel._run_exhibition_mode('exhibition_control')
    assert not panel.__dict__.get('pending_exhibition_key')


def test_process_start_is_not_robot_mode_confirmation():
    panel = OperatorPanel.__new__(OperatorPanel)
    panel._append_log = lambda *args: None
    panel._refresh_summary = lambda: None
    panel._on_started('exhibition')
    assert 'жду подтверждения' in panel.exhibition_mode


def test_only_confirmed_state_changes_preparation_to_ready():
    panel = OperatorPanel.__new__(OperatorPanel)
    panel._refresh_exhibition_status = lambda: None
    panel.status_values = {}
    panel.exhibition_mode = 'Подготовка'
    panel._parse_status_output('STATUS mode=control\nSTATUS exhibition=starting_control\n')
    assert panel.exhibition_mode == 'Подготовка'
    panel._parse_status_output('STATUS mode=control\nSTATUS exhibition=ready\n')
    assert panel.exhibition_mode == 'Управление'
    panel._parse_status_output('STATUS mode=static\nSTATUS exhibition=locked\n')
    assert panel.exhibition_mode.startswith('LOCK')


def test_operator_stop_cancels_a_pending_exhibition_switch():
    calls = []

    class Controller:
        def start(self, key, spec, **kwargs):
            calls.append((key, spec.key, kwargs))
            return True

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.pending_exhibition_key = "exhibition_control"
    panel.pending_exhibition_environment = {"ROBOT_ENABLE_ACTUATION": "1"}
    panel.exhibition_switch_in_progress = True
    panel.pending_warmup_key = 'exhibition_control'
    panel.pending_warmup_environment = {'TOKEN': 'pending'}

    panel.run_key("exhibition_stop")

    assert panel.pending_exhibition_key is None
    assert panel.pending_exhibition_environment is None
    assert panel.exhibition_switch_in_progress is False
    assert panel.pending_warmup_key is None
    assert panel.pending_warmup_environment is None
    assert calls == [("exhibition_stop", "exhibition_stop", {"env_overrides": None})]


def test_switch_waits_for_stop_helper_and_manager_cleanup(monkeypatch):
    starts = []
    stops = []

    class Controller:
        running = {"exhibition"}

        def is_running(self, key):
            return key in self.running

        def stop(self, key, **kwargs):
            stops.append((key, kwargs))

        def start(self, key, spec, **kwargs):
            starts.append((key, spec.key, kwargs))
            self.running.add(key)
            return True

        def active_keys(self):
            return list(self.running)

    class ImmediateTimer:
        @staticmethod
        def singleShot(_delay, callback):
            callback()

    monkeypatch.setattr(app_module, "QTimer", ImmediateTimer)
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.pending_exhibition_key = "exhibition_control"
    panel.pending_exhibition_environment = {"ROBOT_ENABLE_ACTUATION": "1"}
    panel.exhibition_switch_in_progress = True
    panel.exhibition_stop_complete = False
    panel.exhibition_manager_stopped = False
    panel.exhibition_requested_mode = "exhibition_static"
    panel.exhibition_mode = "Статичный"
    panel.exhibition_session_confirmed = True
    panel.preflight_ok = True
    panel.close_after_stop = False
    panel._append_log = lambda *_args: None
    panel._refresh_summary = lambda: None

    panel._on_finished("exhibition_stop", 0, 0)

    assert starts == []
    assert panel.pending_exhibition_key == "exhibition_control"
    assert stops[-1] == (
        "exhibition",
        {"graceful_timeout_ms": EXHIBITION_GRACEFUL_STOP_MS, "wait": False},
    )

    panel.controller.running.remove("exhibition")
    panel._on_finished("exhibition", 0, 0)

    assert starts == [
        (
            "exhibition",
            "exhibition_control",
            {"env_overrides": {"ROBOT_ENABLE_ACTUATION": "1"}},
        )
    ]


@pytest.mark.parametrize('owner', ['exhibition', 'sdk_warmup'])
def test_close_routes_physical_session_through_exhibition_stop(monkeypatch, owner):
    starts = []
    stops = []

    class Controller:
        running = {owner}

        def active_keys(self):
            return list(self.running)

        def is_running(self, key):
            return key in self.running

        def start(self, key, spec, **kwargs):
            starts.append((key, spec.key, kwargs))
            self.running.add(key)
            return True

        def stop(self, key, **kwargs):
            stops.append((key, kwargs))

    class StatusTimer:
        stopped = False

        def stop(self):
            self.stopped = True

    class Event:
        accepted = False
        ignored = False

        def accept(self):
            self.accepted = True

        def ignore(self):
            self.ignored = True

    monkeypatch.setattr(
        app_module.QMessageBox,
        "question",
        lambda *_args, **_kwargs: app_module.QMessageBox.Yes,
    )
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.status_timer = StatusTimer()
    panel.close_after_stop = False
    panel.pending_exhibition_key = "exhibition_control"
    panel.pending_exhibition_environment = {"ROBOT_ENABLE_ACTUATION": "1"}
    panel.pending_warmup_key = "exhibition_control"
    panel.pending_warmup_environment = {"TOKEN": "must-not-start-after-close"}
    panel.exhibition_switch_in_progress = True
    panel.exhibition_stop_complete = False
    panel.exhibition_manager_stopped = False
    event = Event()

    panel.closeEvent(event)

    assert event.ignored is True
    assert event.accepted is False
    assert panel.close_after_stop is True
    assert panel.status_timer.stopped is True
    assert panel.pending_warmup_key is None
    assert panel.pending_warmup_environment is None
    assert starts == (
        [("exhibition_stop", "exhibition_stop", {})]
        if owner == 'exhibition'
        else []
    )
    assert stops == ([] if owner == 'exhibition' else [
        ('sdk_warmup', {'graceful_timeout_ms': 12000, 'wait': False})])


def test_exhibition_checklists_match_static_and_session_arm_modes():
    static_items = OperatorPanel._live_checklist_items(False, False)
    control_items = OperatorPanel._live_checklist_items(True, True)

    assert not any("VR" in item or "deadman" in item.lower() for item in static_items)
    assert any("session-arm" in item for item in control_items)
    assert not any("deadman" in item.lower() for item in control_items)


def test_legs_checklist_confirms_panel_owned_run_and_no_phone_control():
    legs_items = OperatorPanel._live_checklist_items(True, False, True)

    assert any("Unitree Explore" in item for item in legs_items)
    assert any("FSM 811" in item for item in legs_items)
    assert any("панель автоматически" in item for item in legs_items)
    assert any("виртуальный стик" in item for item in legs_items)


def test_locomotion_and_combined_live_request_run_mode():
    specs = {spec.key: spec for spec in command_catalog()}

    assert specs["legs_live"].requires_run_mode is True
    assert specs["arms_live"].requires_run_mode is False
    assert specs["teleop_live"].implemented is True
    assert specs["teleop_live"].requires_run_mode is True
    assert specs["exhibition_control"].implemented is True
    assert specs["exhibition_control"].requires_run_mode is True


def test_full_run_environment_confirms_run_and_no_phone_control():
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.config = OperatorConfig(vr_headset_ip="192.168.8.129")
    panel.live_session_token = "panel-session-token-123456"

    environment = panel._live_environment(run_mode_confirmed=True)

    assert environment["ROBOT_CONFIRM_RUN_MODE"] == "1"
    assert environment["ROBOT_CONFIRM_NO_PHONE_CONTROL"] == "1"


def test_readonly_diagnostics_and_voice_actions_are_catalogued():
    specs = {spec.key: spec for spec in command_catalog()}
    assert specs["writer_check"].live is False
    assert specs["panel_status"].live is False
    assert specs["voice_check"].target == "r1-voice-preflight"


def test_panel_status_parser_accepts_line_protocol():
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.status_values = {}
    panel._parse_status_output(
        "noise\nSTATUS ethernet=OK\nSTATUS deadman=RELEASED\n"
    )
    assert panel.status_values == {"ethernet": "OK", "deadman": "RELEASED"}


def test_first_healthy_robot_status_does_not_restart_fresh_sdk_warmup():
    stops = []

    class Controller:
        def is_running(self, key):
            return key == "sdk_warmup"

        def stop(self, key, **kwargs):
            stops.append((key, kwargs))

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.status_values = {}
    panel.warmup_restart_pending = False
    panel.warmup_robot_offline_observed = False
    panel._refresh_exhibition_status = lambda: None

    panel._parse_status_output("STATUS robot=OK\n")

    assert stops == []
    assert panel.warmup_restart_pending is False


@pytest.mark.parametrize("exit_code", [0, 1])
def test_robot_offline_to_ok_restarts_sdk_warmup_only_after_clean_exit(
    monkeypatch, exit_code
):
    starts = []
    stops = []

    class Controller:
        running = {"sdk_warmup"}

        def is_running(self, key):
            return key in self.running

        def stop(self, key, **kwargs):
            stops.append((key, kwargs))

        def start(self, key, spec, **kwargs):
            starts.append((key, spec.key, kwargs))
            self.running.add(key)
            return True

    class ImmediateTimer:
        @staticmethod
        def singleShot(_delay, callback):
            callback()

    monkeypatch.setattr(app_module, "QTimer", ImmediateTimer)
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.status_values = {}
    panel.preflight_ok = True
    panel.pending_warmup_key = None
    panel.pending_warmup_environment = None
    panel.pending_exhibition_key = None
    panel.warmup_restart_pending = False
    panel.warmup_robot_offline_observed = False
    panel.close_after_stop = False
    panel._clear_live_confirmation = lambda: None
    panel._refresh_exhibition_status = lambda: None
    panel._refresh_summary = lambda: None
    panel._append_log = lambda *_args: None

    panel._parse_status_output("STATUS robot=OFFLINE\n")
    assert stops == []
    panel._parse_status_output("STATUS robot=OK\n")

    assert stops == [
        ("sdk_warmup", {"graceful_timeout_ms": 12000, "wait": False})
    ]
    assert panel.warmup_restart_pending is True
    assert starts == []

    panel.controller.running.remove("sdk_warmup")
    panel._on_finished("sdk_warmup", exit_code, 0)

    if exit_code == 0:
        assert starts == [("sdk_warmup", "sdk_warmup", {})]
        assert panel.warmup_restart_pending is False
    else:
        assert starts == []
        assert panel.warmup_restart_pending is True
        assert panel.status_values["sdk_warmup"] == "RESTART_FAILED"


def test_active_physical_session_blocks_recovered_link_warmup_restart():
    calls = []

    class Controller:
        def is_running(self, key):
            return key == "exhibition"

        def stop(self, *args, **kwargs):
            calls.append(("stop", args, kwargs))

        def start(self, *args, **kwargs):
            calls.append(("start", args, kwargs))
            return True

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.status_values = {}
    panel.preflight_ok = True
    panel.pending_warmup_key = None
    panel.pending_warmup_environment = None
    panel.pending_exhibition_key = None
    panel.warmup_restart_pending = False
    panel.warmup_robot_offline_observed = False
    panel._clear_live_confirmation = lambda: None
    panel._refresh_exhibition_status = lambda: None

    panel._parse_status_output("STATUS robot=OFFLINE\n")
    panel._parse_status_output("STATUS robot=READY\n")

    assert calls == []
    assert panel.warmup_restart_pending is False


@pytest.mark.parametrize("key", ["exhibition_control", "exhibition_stand"])
def test_physical_request_cancels_background_warmup_restart(key):
    stops = []

    class Controller:
        def is_running(self, process_key):
            return process_key == "sdk_warmup"

        def stop(self, process_key, **kwargs):
            stops.append((process_key, kwargs))

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.status_values = {}
    panel.exhibition_session_confirmed = False
    panel.exhibition_requested_mode = ""
    panel.exhibition_mode = "Остановлен"
    panel.pending_warmup_key = None
    panel.pending_warmup_environment = None
    panel.warmup_restart_pending = True
    panel._confirm_live = lambda *_args, **_kwargs: True
    panel._live_environment = lambda **_kwargs: {"TOKEN": "physical-priority"}
    panel._refresh_summary = lambda: None

    panel._run_exhibition_mode(key)

    assert panel.warmup_restart_pending is False
    assert panel.pending_warmup_key == key
    assert stops == [
        ("sdk_warmup", {"graceful_timeout_ms": 12000, "wait": False})
    ]


def test_live_ack_survives_transient_robot_warning_but_not_offline():
    panel = OperatorPanel.__new__(OperatorPanel)
    panel.status_values = {}
    panel.preflight_ok = True
    cleared = []
    panel._clear_live_confirmation = lambda: cleared.append(True)
    panel._refresh_exhibition_status = lambda: None

    panel._parse_status_output("STATUS robot=WARN\n")
    assert cleared == []

    panel._parse_status_output("STATUS robot=OFFLINE\n")
    assert cleared == [True]
    assert panel.preflight_ok is False


def test_panel_config_normalizes_safe_boolean_and_poll_values(tmp_path):
    path = tmp_path / "operator.json"
    path.write_text(json.dumps({
        "dry_run": "false",
        "allow_live": "true",
        "allow_half_duplex_adapter": "true",
        "require_preflight": "true",
        "status_poll_sec": 0.1,
    }), encoding="utf-8")
    config = load_config(path)
    assert config.dry_run is False
    assert config.allow_live is True
    assert config.allow_half_duplex_adapter is True
    assert config.require_preflight is True
    assert config.status_poll_sec == 1.0


def test_check_all_propagates_the_worst_check_status(tmp_path):
    captured = {}

    class Controller:
        def start(self, key, spec, *, shell_command=None):
            captured["key"] = key
            captured["shell_command"] = shell_command

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel._run_check_all()

    fake_make = tmp_path / "make"
    fake_make.write_text(
        "#!/bin/sh\n"
        "case \"$1\" in\n"
        "  r1-lan-preflight) exit 1;;\n"
        "  robot-preflight) exit 2;;\n"
        "  *) exit 0;;\n"
        "esac\n",
        encoding="utf-8",
    )
    fake_make.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{tmp_path}:{environment['PATH']}"
    result = subprocess.run(
        ["bash", "-lc", captured["shell_command"]],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert captured["key"] == "check_all"
    assert result.returncode == 2


def test_check_all_camera_warning_is_advisory_for_motion_gate(tmp_path):
    captured = {}

    class Controller:
        def start(self, key, spec, *, shell_command=None):
            captured["shell_command"] = shell_command

    panel = OperatorPanel.__new__(OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel._run_check_all()

    fake_make = tmp_path / "make"
    fake_make.write_text(
        "#!/bin/sh\n"
        "case \"$1\" in\n"
        "  r1-camera-preflight) exit 2;;\n"
        "  *) exit 0;;\n"
        "esac\n",
        encoding="utf-8",
    )
    fake_make.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{tmp_path}:{environment['PATH']}"
    result = subprocess.run(
        ["bash", "-lc", captured["shell_command"]],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0
    assert "advisory check returned 2" in result.stdout
