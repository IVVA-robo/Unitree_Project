"""Contracts for the explicit Damping -> Zero Torque operator action."""

from pathlib import Path
import subprocess

import operator_panel.app as app_module
from operator_panel.commands import command_catalog

PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / "scripts" / "r1-zero-torque"
TEXT = SCRIPT.read_text(encoding="utf-8")
MAKEFILE = (PROJECT / "Makefile").read_text(encoding="utf-8")


def test_zero_torque_requires_separate_support_and_action_acknowledgements():
    for token in (
        "ROBOT_CONFIRM_ZERO_TORQUE",
        "ROBOT_CONFIRM_ROBOT_SUPPORTED",
        "ROBOT_ZERO_TORQUE_TOKEN",
        "live writer still exists; Zero Torque was not sent",
    ):
        assert token in TEXT


def test_zero_torque_confirms_damping_before_fsm_zero():
    damping = TEXT.index("client.SetFsmId(1)")
    damping_confirmed = TEXT.index("wait_fsm(1, 5.0)", damping)
    zero = TEXT.index("client.SetFsmId(0)", damping_confirmed)
    zero_confirmed = TEXT.index("wait_fsm(0, 5.0)", zero)
    assert damping < damping_confirmed < zero < zero_confirmed
    assert "Damping/FSM 1 was not confirmed; Zero Torque was not sent" in TEXT


def test_zero_torque_has_make_target_and_valid_shell_syntax():
    assert "robot-zero-torque:" in MAKEFILE
    assert "./scripts/r1-zero-torque" in MAKEFILE
    subprocess.run(("bash", "-n", str(SCRIPT)), check=True)


def test_zero_torque_fails_closed_without_panel_acknowledgement():
    result = subprocess.run(
        (str(SCRIPT),),
        cwd=PROJECT,
        text=True,
        capture_output=True,
        timeout=3,
    )
    assert result.returncode == 2
    assert "ROBOT_CONFIRM_ZERO_TORQUE=1 is required" in result.stderr


def test_zero_torque_waits_for_offline_handoff_before_starting_fsm(monkeypatch):
    """Panel order is helper cleanup -> exact offline handoff -> FSM 1/0."""

    calls = []

    class Controller:
        def __init__(self):
            self.running = {"connection_ensure"}

        def active_keys(self):
            return list(self.running)

        def is_running(self, key):
            return key in self.running

        def stop(self, key, **kwargs):
            calls.append(("stop", key, kwargs))
            self.running.discard(key)

        def start(self, key, spec, **kwargs):
            calls.append(("start", key, spec.key, kwargs))
            self.running.add(key)
            return True

    class Timer:
        @staticmethod
        def singleShot(_delay, callback):
            callback()

    monkeypatch.setattr(app_module, "QTimer", Timer)
    panel = app_module.OperatorPanel.__new__(app_module.OperatorPanel)
    panel.controller = Controller()
    panel.specs = {spec.key: spec for spec in command_catalog()}
    panel.pending_zero_torque = True
    panel.zero_torque_deadline = 9999999999.0
    panel.zero_torque_handoff_started = False
    panel.zero_torque_handoff_complete = False
    panel.background_services_started = True
    panel.status_timer = type("StatusTimer", (), {"stop": lambda self: None})()
    panel.video_preview = type(
        "Preview",
        (),
        {"stop": lambda self, **kwargs: calls.append(("preview_stop", kwargs))},
    )()
    panel._append_log = lambda *_args: None
    panel._refresh_summary = lambda: None

    panel._continue_zero_torque_when_idle()
    assert calls[:3] == [
        ("preview_stop", {"wait_ms": 0}),
        ("stop", "connection_ensure", {"graceful_timeout_ms": 1200, "wait": False}),
        (
            "start",
            app_module.ZERO_TORQUE_HANDOFF_KEY,
            app_module.ZERO_TORQUE_HANDOFF_KEY,
            {"shell_command": app_module.ZERO_TORQUE_HANDOFF_COMMAND},
        ),
    ]
    assert not any(call[0] == "start" and call[1] == "zero_torque" for call in calls)

    panel.controller.running.remove(app_module.ZERO_TORQUE_HANDOFF_KEY)
    panel.zero_torque_handoff_complete = True
    panel._continue_zero_torque_when_idle()
    assert calls[-1][0:3] == ("start", "zero_torque", "zero_torque")
    assert panel.pending_zero_torque is False


def test_zero_torque_script_waits_bounded_for_writer_cleanup():
    assert "R1_ZERO_TORQUE_WRITER_WAIT_SEC" in TEXT
    assert "writer_deadline" in TEXT
    assert "waiting for live writer cleanup" in TEXT
