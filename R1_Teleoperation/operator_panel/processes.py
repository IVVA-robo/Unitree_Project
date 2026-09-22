"""Non-blocking QProcess management for operator actions."""

from __future__ import annotations

import os
from typing import Dict, Optional

from PyQt5.QtCore import QObject, QProcess, QProcessEnvironment, pyqtSignal

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

    def start(self, key: str, spec: CommandSpec, *, shell_command: Optional[str] = None) -> bool:
        if self.is_running(key):
            self.output.emit(key, "[INFO] Этот режим уже запущен. Второй экземпляр не создаётся.\n")
            return False

        process = QProcess(self)
        process.setWorkingDirectory(self.config.project_dir)
        environment = QProcessEnvironment.systemEnvironment()
        for name, value in self.config.as_environment().items():
            environment.insert(name, value)
        process.setProcessEnvironment(environment)
        if shell_command is not None:
            program = "/bin/bash"
            arguments = ["-lc", shell_command]
        else:
            program = "/usr/bin/make"
            arguments = [spec.target] if spec.target else []
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
            lambda error, k=key: self._process_error(k, error)
        )
        process.finished.connect(
            lambda code, status, k=key, p=process: self._process_finished(
                k, code, status, p
            )
        )
        self.processes[key] = process
        process.start()
        return True

    def stop(self, key: str) -> None:
        process = self.processes.get(key)
        if not process or process.state() == QProcess.NotRunning:
            return
        self.output.emit(key, "[INFO] Запрашивается остановка процесса.\n")
        process.terminate()
        if not process.waitForFinished(1200):
            process.kill()

    def kill(self, key: str) -> None:
        process = self.processes.get(key)
        if process and process.state() != QProcess.NotRunning:
            self.output.emit(key, "[INFO] Процесс принудительно остановлен.\n")
            process.kill()

    def stop_all(self) -> None:
        for key in list(self.processes):
            self.stop(key)

    def active_keys(self):
        return [key for key in self.processes if self.is_running(key)]

    def _read_output(self, process: QProcess, key: str, error: bool) -> None:
        data = (
            process.readAllStandardError() if error else process.readAllStandardOutput()
        )
        text = bytes(data).decode("utf-8", errors="replace")
        if text:
            self.output.emit(key, text)

    def _process_error(self, key: str, error: QProcess.ProcessError) -> None:
        self.failed.emit(key, f"ошибка QProcess: {error}")

    def _process_finished(
        self, key: str, code: int, status: QProcess.ExitStatus, process: QProcess
    ) -> None:
        self._read_output(process, key, False)
        self._read_output(process, key, True)
        self.finished.emit(key, code, int(status))
        self.processes.pop(key, None)
        process.deleteLater()
