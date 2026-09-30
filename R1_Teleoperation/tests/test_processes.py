import os
from pathlib import Path
import shutil
import subprocess
import time

import pytest
from PyQt5.QtCore import QCoreApplication, QProcess

from operator_panel.commands import CommandSpec
from operator_panel.config import OperatorConfig
from operator_panel.processes import ProcessController


def test_failed_exec_finishes_action_and_allows_retry(monkeypatch):
    app = QCoreApplication.instance() or QCoreApplication([])
    controller = ProcessController(OperatorConfig())
    monkeypatch.setattr(shutil, 'which', lambda _: '/nonexistent-r1-test-executable')
    finished = []
    controller.finished.connect(lambda *args: finished.append(args))
    for attempt in range(2):
        assert controller.start('bad-exec', CommandSpec('bad-exec', 'test'))
        for _ in range(100):
            app.processEvents()
            if len(finished) == attempt + 1:
                break
            time.sleep(0.01)
        assert finished[-1][1] == 127
        assert not controller.active_keys()
    assert len(finished) == 2


def test_exhibition_owner_is_never_force_killed_after_grace():
    app = QCoreApplication.instance() or QCoreApplication([])
    controller = ProcessController(OperatorConfig())
    assert controller.start('exhibition', CommandSpec('test', 'test'), shell_command='exec sleep 30')
    process = controller.processes['exhibition']
    assert process.waitForStarted(1000)
    try:
        controller._force_kill('exhibition', process)
        assert controller.is_running('exhibition')
    finally:
        controller.stop('exhibition')
        app.processEvents()


@pytest.mark.skipif(shutil.which("setsid") is None, reason="setsid is required")
def test_stop_terminates_the_full_child_process_group():
    """Stopping a panel action must not leave its long-running child behind."""

    app = QCoreApplication.instance() or QCoreApplication([])
    controller = ProcessController(OperatorConfig())
    spec = CommandSpec("process_test", "process test")
    assert controller.start(
        "process_test",
        spec,
        shell_command="exec sleep 30",
    )
    process = controller.processes["process_test"]
    for _ in range(50):
        app.processEvents()
        if process.state() == QProcess.Running:
            break
        time.sleep(0.01)
    assert process.state() == QProcess.Running
    process_group = os.getpgid(int(process.processId()))
    assert process_group != os.getpgrp()

    controller.stop("process_test")
    app.processEvents()
    assert not controller.active_keys()
    # The numeric PGID can be reused immediately by the test runner.  Inspect
    # the command line as well, so the assertion is about the child we started,
    # rather than the continued existence of a recycled process-group number.
    for _ in range(20):
        rows = subprocess.run(
            ["ps", "-eo", "pgid=,args="],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.splitlines()
        if not any(
            row.strip().startswith(f"{process_group} ") and "sleep 30" in row
            for row in rows
        ):
            break
        app.processEvents()
        time.sleep(0.01)
    assert not any(
        row.strip().startswith(f"{process_group} ") and "sleep 30" in row
        for row in rows
    )


def test_start_applies_per_action_environment_overrides():
    app = QCoreApplication.instance() or QCoreApplication([])
    controller = ProcessController(OperatorConfig())
    spec = CommandSpec("environment_test", "environment test")

    assert controller.start(
        "environment_test",
        spec,
        shell_command="exec sleep 30",
        env_overrides={
            "ROBOT_DRY_RUN": "0",
            "R1_TEST_PANEL_TOKEN": "ephemeral",
        },
    )
    process = controller.processes["environment_test"]
    assert process.processEnvironment().value("ROBOT_DRY_RUN") == "0"
    assert process.processEnvironment().value("R1_TEST_PANEL_TOKEN") == "ephemeral"

    controller.stop("environment_test")
    app.processEvents()


def test_tp_link_half_duplex_setting_reaches_exhibition_child_environment():
    """The persisted adapter exception must reach the manager process."""

    app = QCoreApplication.instance() or QCoreApplication([])
    controller = ProcessController(
        OperatorConfig(allow_half_duplex_adapter=True)
    )
    spec = CommandSpec(
        "exhibition_control", "exhibition control environment"
    )

    assert controller.start(
        "exhibition", spec, shell_command="exec sleep 30"
    )
    process = controller.processes["exhibition"]
    adapter_override = process.processEnvironment().value(
        "R1_SDK_ALLOW_HALF_DUPLEX"
    )
    assert adapter_override == "1"

    # The accepting preflight branch remains restricted to cdc_ether and does
    # not replace the following receive-error/telemetry checks.
    preflight = (
        Path(__file__).resolve().parents[1] / "scripts" / "r1-sdk-preflight"
    ).read_text(encoding="utf-8")
    assert "ALLOW_HALF_DUPLEX == 1 && $driver_name == cdc_ether" in preflight
    assert "rx_errors_before" in preflight
    assert "motor_health" in preflight

    controller.stop("exhibition")
    app.processEvents()
    assert not controller.active_keys()


@pytest.mark.skipif(shutil.which("setsid") is None, reason="setsid is required")
def test_asynchronous_stop_honors_graceful_cleanup(tmp_path):
    app = QCoreApplication.instance() or QCoreApplication([])
    marker = tmp_path / "cleaned"
    ready = tmp_path / "ready"
    helper = tmp_path / "graceful_child.py"
    helper.write_text(
        "import pathlib, signal, time\n"
        f"marker = pathlib.Path({str(marker)!r})\n"
        f"ready = pathlib.Path({str(ready)!r})\n"
        "def stop(_signum, _frame):\n"
        "    time.sleep(0.30)\n"
        "    marker.write_text('clean', encoding='utf-8')\n"
        "    raise SystemExit(0)\n"
        "signal.signal(signal.SIGTERM, stop)\n"
        "ready.write_text('ready', encoding='utf-8')\n"
        "while True:\n"
        "    time.sleep(0.05)\n",
        encoding="utf-8",
    )
    output = []
    controller = ProcessController(OperatorConfig())
    controller.output.connect(lambda _key, text: output.append(text))
    spec = CommandSpec("graceful_test", "graceful test")
    assert controller.start(
        "graceful_test",
        spec,
        shell_command=f"exec python3 {helper}",
    )
    process = controller.processes["graceful_test"]
    for _ in range(50):
        app.processEvents()
        if process.state() == QProcess.Running and ready.is_file():
            break
        time.sleep(0.01)
    assert ready.is_file()

    started = time.monotonic()
    controller.stop(
        "graceful_test", graceful_timeout_ms=1500, wait=False
    )
    elapsed = time.monotonic() - started

    assert elapsed < 0.20
    assert controller.is_running("graceful_test")
    for _ in range(200):
        app.processEvents()
        if not controller.is_running("graceful_test"):
            break
        time.sleep(0.01)
    assert marker.read_text(encoding="utf-8") == "clean"
    assert not any("SIGKILL" in text for text in output)
