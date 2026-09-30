"""Non-blocking QProcess management for operator actions."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import signal
from typing import Dict, Mapping, Optional

from PyQt5.QtCore import QObject, QProcess, QProcessEnvironment, QTimer, pyqtSignal

from .commands import CommandSpec
from .config import OperatorConfig


class ProcessController(QObject):
    output = pyqtSignal(str, str)  # key, text
    started = pyqtSignal(str)
    finished = pyqtSignal(str, int, int)  # key, exit code, exit status
    failed = pyqtSignal(str, str)

    def __init__(self, config: OperatorConfig, parent: Optional[QObject] = None):
        super().__init__(parent)
        self.config = config
        self.processes: Dict[str, QProcess] = {}

    def is_running(self, key: str) -> bool:
        process = self.processes.get(key)
        return bool(process and process.state() != QProcess.NotRunning)

    def start(
        self,
        key: str,
        spec: CommandSpec,
        *,
        shell_command: Optional[str] = None,
        env_overrides: Optional[Mapping[str, str]] = None,
    ) -> bool:
        if self.is_running(key):
            self.output.emit(
                key,
                "[INFO] Этот режим уже запущен. Второй экземпляр не создаётся.\n",
            )
            return False

        process = QProcess(self)
        process.setWorkingDirectory(self.config.project_dir)
        environment = QProcessEnvironment.systemEnvironment()
        for name, value in self.config.as_environment().items():
            environment.insert(name, value)
        for name, value in (env_overrides or {}).items():
            environment.insert(name, value)
        process.setProcessEnvironment(environment)
        # Run every action in its own session/process group.  A Make target
        # commonly execs a ROS launch tree; terminating only the top-level
        # `make` process would otherwise leave the writer, viewer, or DDS
        # children behind after STOP or window close.
        setsid = shutil.which("setsid")
        if shell_command is not None:
            child_program = "/bin/bash"
            child_arguments = ["-lc", shell_command]
        elif key == "sdk_warmup" and spec.key == "sdk_warmup":
            # Observe the worker itself, not make: SIGTERM can terminate make
            # with code 15 before the worker has reaped its read-only reader.
            # QProcess.finished must mean the actual handoff owner is done.
            child_program = str(Path(self.config.project_dir) / "scripts" / "r1-sdk-warmup")
            child_arguments = []
        else:
            child_program = "/usr/bin/make"
            child_arguments = [spec.target] if spec.target else []
        if setsid:
            program = setsid
            arguments = [child_program, *child_arguments]
        else:  # pragma: no cover - coreutils provides setsid on supported hosts
            program = child_program
            arguments = child_arguments
        process.setProgram(program)
        process.setArguments(arguments)
        process.readyReadStandardOutput.connect(
            lambda p=process, k=key: self._read_output(p, k, False)
        )
        process.readyReadStandardError.connect(
            lambda p=process, k=key: self._read_output(p, k, True)
        )
        process.started.connect(lambda k=key: self.started.emit(k))
        process.errorOccurred.connect(
            lambda error, k=key, p=process: self._process_error(k, error, p)
        )
        process.finished.connect(
            lambda code, status, k=key, p=process: self._process_finished(
                k, code, status, p
            )
        )
        self.processes[key] = process
        process.start()
        return True

    def stop(
        self,
        key: str,
        *,
        graceful_timeout_ms: int = 1200,
        wait: bool = True,
    ) -> None:
        """Request SIGTERM, optionally leaving the grace period asynchronous.

        Most panel helpers are short wrappers and retain the bounded blocking
        path.  The exhibition manager is different: after SIGTERM it disarms
        the session, calls the reviewed robot STOP path, and reaps child
        process groups.  Its caller therefore uses a long asynchronous grace
        period so the Qt UI remains responsive and does not orphan children.
        """

        process = self.processes.get(key)
        if not process or process.state() == QProcess.NotRunning:
            return
        self.output.emit(key, "[INFO] Запрашивается остановка процесса.\n")
        self._signal_process_group(process, signal.SIGTERM, process.terminate)
        timeout_ms = max(0, int(graceful_timeout_ms))
        if not wait:
            QTimer.singleShot(
                timeout_ms,
                lambda k=key, p=process: self._kill_after_grace(k, p),
            )
            return
        if not process.waitForFinished(timeout_ms):
            self._force_kill(key, process)

    def kill(self, key: str) -> None:
        process = self.processes.get(key)
        if process and process.state() != QProcess.NotRunning:
            self.output.emit(key, "[INFO] Процесс принудительно остановлен.\n")
            self._force_kill(key, process)

    def stop_all(self) -> None:
        for key in list(self.processes):
            self.stop(key)

    def _kill_after_grace(self, key: str, process: QProcess) -> None:
        """Kill only the exact process that exhausted its requested grace."""

        if self.processes.get(key) is not process:
            return
        if process.state() == QProcess.NotRunning:
            return
        if key in {"exhibition", "sdk_warmup"}:
            self._force_kill(key, process)  # emits the protected-owner warning only
            return
        self.output.emit(
            key,
            "[WARN] Процесс не завершился за отведённое время; применяется SIGKILL.\n",
        )
        self._force_kill(key, process)

    def _force_kill(self, key: str, process: QProcess) -> None:
        if self.processes.get(key) is not process:
            return
        if process.state() == QProcess.NotRunning:
            return
        if key in {"exhibition", "sdk_warmup"}:
            self.output.emit(key, "[BLOCKED] Владелец дочерних процессов ещё выполняет очистку; SIGKILL запрещён. Новый запуск ждёт завершения.\n")
            return
        self._signal_process_group(process, signal.SIGKILL, process.kill)
        process.waitForFinished(1200)

    def active_keys(self):
        return [key for key in self.processes if self.is_running(key)]

    @staticmethod
    def _signal_process_group(process: QProcess, sig: int, fallback) -> None:
        """Signal a child session without ever targeting the panel's group."""

        try:
            pid = int(process.processId())
            pgid = os.getpgid(pid)
            own_group = os.getpgrp()
            if pid > 0 and pgid > 0 and pgid != own_group:
                os.killpg(pgid, sig)
                return
        except (OSError, ValueError):
            pass
        # If the process exited, or setsid is unavailable, retain QProcess's
        # normal signal path rather than risking a signal to our own group.
        fallback()

    def _read_output(self, process: QProcess, key: str, error: bool) -> None:
        data = (
            process.readAllStandardError() if error else process.readAllStandardOutput()
        )
        text = bytes(data).decode("utf-8", errors="replace")
        if text:
            self.output.emit(key, text)

    def _process_error(self, key: str, error: QProcess.ProcessError, process: QProcess) -> None:
        if self.processes.get(key) is not process:
            return
        self.failed.emit(key, f"ошибка QProcess: {error}")
        if error == QProcess.FailedToStart:
            # Qt emits no finished signal for a failed exec. Complete the
            # logical action so pending switches cannot stay stuck forever.
            self._process_finished(key, 127, QProcess.CrashExit, process)

    def _process_finished(
        self, key: str, code: int, status: QProcess.ExitStatus, process: QProcess
    ) -> None:
        self._read_output(process, key, False)
        self._read_output(process, key, True)
        self.finished.emit(key, code, int(status))
        # A mode switch may replace a process under the same logical key from
        # its finished callback.  Never let the old callback remove the new
        # QProcess from the registry.
        if self.processes.get(key) is process:
            self.processes.pop(key, None)
        process.deleteLater()
