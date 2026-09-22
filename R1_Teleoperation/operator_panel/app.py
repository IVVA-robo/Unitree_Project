"""Russian Qt operator panel for Unitree R1 Teleoperation."""

from __future__ import annotations

import datetime as _datetime
import os
import sys
from pathlib import Path
from typing import Dict, Iterable, Optional

from PyQt5.QtCore import QTimer, QUrl, Qt
from PyQt5.QtGui import QColor, QDesktopServices, QFont, QIcon
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .commands import CommandSpec, command_catalog, spec_by_key
from .config import CONFIG_PATH, OperatorConfig, load_config, save_config
from .processes import ProcessController


class SettingsDialog(QDialog):
    def __init__(self, config: OperatorConfig, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setWindowTitle("Настройки панели")
        self.setModal(True)
        self.config = config
        form = QFormLayout(self)
        self.project_dir = QLineEdit(config.project_dir)
        self.robot_ip = QLineEdit(config.robot_ip)
        self.pc2_ip = QLineEdit(config.pc2_ip)
        self.robot_interface = QLineEdit(config.robot_interface)
        self.laptop_robot_ip = QLineEdit(config.laptop_robot_ip)
        self.ros_domain = QSpinBox()
        self.ros_domain.setRange(0, 232)
        self.ros_domain.setValue(config.ros_domain_id)
        self.vr_ip = QLineEdit(config.vr_headset_ip)
        self.video_url = QLineEdit(config.video_url)
        self.logs_dir = QLineEdit(config.logs_dir)
        self.video_profile = QLineEdit(config.video_profile)
        self.locomotion_profile = QLineEdit(config.locomotion_profile)
        self.dry_run = QCheckBox("Безопасный dry-run по умолчанию")
        self.dry_run.setChecked(config.dry_run)
        self.allow_live = QCheckBox("Разрешить кнопкам запрашивать live-сеанс")
        self.allow_live.setChecked(config.allow_live)
        form.addRow("Путь проекта", self.project_dir)
        form.addRow("IP робота / DDS", self.robot_ip)
        form.addRow("IP PC2 / видеосервис", self.pc2_ip)
        form.addRow("Ethernet-интерфейс", self.robot_interface)
        form.addRow("IP ноутбука на Ethernet", self.laptop_robot_ip)
        form.addRow("ROS_DOMAIN_ID", self.ros_domain)
        form.addRow("IP VR-шлема", self.vr_ip)
        form.addRow("URL Robot POV", self.video_url)
        form.addRow("Папка логов", self.logs_dir)
        form.addRow("Профиль видео", self.video_profile)
        form.addRow("Профиль locomotion", self.locomotion_profile)
        form.addRow(self.dry_run)
        form.addRow(self.allow_live)
        note = QLabel(
            "Live-кнопки всё равно проходят существующие safety gates проекта. "
            "Токен commissioning и физические подтверждения панель не подставляет."
        )
        note.setWordWrap(True)
        form.addRow(note)
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def values(self) -> OperatorConfig:
        return OperatorConfig(
            project_dir=self.project_dir.text().strip(),
            robot_ip=self.robot_ip.text().strip(),
            pc2_ip=self.pc2_ip.text().strip(),
            robot_interface=self.robot_interface.text().strip(),
            laptop_robot_ip=self.laptop_robot_ip.text().strip(),
            ros_domain_id=self.ros_domain.value(),
            vr_headset_ip=self.vr_ip.text().strip(),
            video_url=self.video_url.text().strip(),
            logs_dir=self.logs_dir.text().strip(),
            video_profile=self.video_profile.text().strip() or "low-latency",
            locomotion_profile=self.locomotion_profile.text().strip() or "slow-safe",
            dry_run=self.dry_run.isChecked(),
            allow_live=self.allow_live.isChecked(),
        )


class OperatorPanel(QMainWindow):
    """Main window.  All long-running actions are child processes, never UI threads."""

    def __init__(self, config: Optional[OperatorConfig] = None):
        super().__init__()
        self.config = config or load_config()
        self.controller = ProcessController(self.config, self)
        self.specs: Dict[str, CommandSpec] = {
            spec.key: spec for spec in command_catalog()
        }
        self.close_after_stop = False
        self.last_status = "Серый • диагностика ещё не запускалась"
        self.setWindowTitle("Unitree R1 Панель оператора")
        icon_path = (
            Path(__file__).resolve().parents[1]
            / "assets"
            / "icons"
            / "unitree-r1-robot.png"
        )
        if icon_path.is_file():
            self.setWindowIcon(QIcon(str(icon_path)))
        self.resize(1220, 820)
        self._build_ui()
        self._connect_controller()
        self._refresh_summary()

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(12, 12, 12, 8)

        header = QHBoxLayout()
        title = QLabel("UNITREE R1  •  ПАНЕЛЬ ОПЕРАТОРА")
        title.setObjectName("appTitle")
        title.setFont(QFont("Sans Serif", 16, QFont.Bold))
        header.addWidget(title)
        header.addStretch(1)
        self.mode_label = QLabel()
        self.mode_label.setObjectName("modeLabel")
        header.addWidget(self.mode_label)
        settings = QPushButton("Настройки")
        settings.clicked.connect(self.open_settings)
        header.addWidget(settings)
        layout.addLayout(header)

        safety = QHBoxLayout()
        self.status_label = QLabel()
        self.status_label.setObjectName("statusLabel")
        safety.addWidget(self.status_label, 1)
        self.active_label = QLabel("Активных процессов: 0")
        safety.addWidget(self.active_label)
        stop = QPushButton("■  STOP")
        stop.setObjectName("stopButton")
        stop.setMinimumHeight(46)
        stop.clicked.connect(lambda: self.run_key("stop"))
        safety.addWidget(stop)
        kill = QPushButton("⚠  KILL")
        kill.setObjectName("killButton")
        kill.setMinimumHeight(46)
        kill.clicked.connect(lambda: self.run_key("kill"))
        safety.addWidget(kill)
        layout.addLayout(safety)

        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, 1)
        self._add_general_tab()
        self._add_diagnostics_tab()
        self._add_robot_tab()
        self._add_vr_tab()
        self._add_video_tab()
        self._add_motion_tab()
        self._add_logs_tab()

    def _connect_controller(self) -> None:
        self.controller.output.connect(self._on_output)
        self.controller.started.connect(self._on_started)
        self.controller.finished.connect(self._on_finished)
        self.controller.failed.connect(
            lambda key, message: self._append_log(key, f"[FAIL] {message}\n")
        )

    def _add_general_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        cards = QGridLayout()
        self.summary_project = self._card("Проект", self.config.project_dir)
        self.summary_network = self._card(
            "Сеть робота",
            f"{self.config.robot_interface}\nDDS {self.config.robot_ip}\nPC2 {self.config.pc2_ip}",
        )
        self.summary_video = self._card("Видео", self.config.video_url)
        self.summary_mode = self._card("Режим", self._mode_text())
        self.summary_processes = self._card("Процессы", "Нет активных")
        cards.addWidget(self.summary_project, 0, 0)
        cards.addWidget(self.summary_network, 0, 1)
        cards.addWidget(self.summary_video, 1, 0)
        cards.addWidget(self.summary_mode, 1, 1)
        cards.addWidget(self.summary_processes, 2, 0, 1, 2)
        layout.addLayout(cards)
        explanation = QLabel(
            "Начните с «Проверить всё». Команды движения требуют отдельного "
            "чек-листа и остаются под защитой safety gates проекта."
        )
        explanation.setWordWrap(True)
        explanation.setObjectName("hint")
        layout.addWidget(explanation)
        layout.addStretch(1)
        self.tabs.addTab(tab, "Общий статус")

    def _add_diagnostics_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox("Диагностика без управления приводами")
        grid = QGridLayout(group)
        buttons = [
            ("check_all", "Проверить всё"),
            ("network", "Проверить сеть"),
            ("ethernet", "Проверить Ethernet к роботу"),
            ("robot_check", "Проверить робота"),
            ("vr_check", "Проверить VR-шлем"),
            ("controllers_check", "Проверить контроллеры"),
            ("camera_check", "Проверить камеры"),
            ("video_check", "Проверить видео"),
            ("ros_check", "Проверить ROS2"),
            ("sdk_check", "Проверить Unitree SDK"),
        ]
        for index, (key, text) in enumerate(buttons):
            grid.addWidget(self._action_button(key, text), index // 2, index % 2)
        layout.addWidget(group)
        hint = QLabel(
            "Проверка использует существующие read-only preflight-скрипты. "
            "Отсутствующий робот или выключенный VR показываются как предупреждение."
        )
        hint.setWordWrap(True)
        hint.setObjectName("hint")
        layout.addWidget(hint)
        layout.addStretch(1)
        self.tabs.addTab(tab, "Сеть и подключения")

    def _add_robot_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox("Робот")
        grid = QGridLayout(group)
        for index, (key, text) in enumerate(
            [
                ("prepare", "Подготовить робота"),
                ("stand", "Включить stand/balance"),
                ("stop", "Остановить робота"),
                ("kill", "Аварийная остановка"),
                ("reset_kill", "Снять аварийную остановку"),
            ]
        ):
            button = self._action_button(key, text)
            if key == "kill":
                button.setObjectName("killButton")
            if key == "stop":
                button.setObjectName("stopButton")
            grid.addWidget(button, index // 2, index % 2)
        layout.addWidget(group)
        note = QLabel(
            "Подготовка и stand/balance доступны только после подтверждения "
            "безопасного положения. Отдельной безопасной команды снятия kill latch "
            "в проекте сейчас нет."
        )
        note.setWordWrap(True)
        note.setObjectName("hint")
        layout.addWidget(note)
        layout.addStretch(1)
        self.tabs.addTab(tab, "Робот")

    def _add_vr_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox("VR и очки")
        grid = QGridLayout(group)
        for index, (key, text) in enumerate(
            [
                ("vr_calibrate", "Калибровать VR"),
                ("hands_calibrate", "Калибровать руки"),
                ("dry_run", "Запустить VR dry-run"),
                ("vr_check", "Проверить VR-шлем"),
            ]
        ):
            grid.addWidget(self._action_button(key, text), index // 2, index % 2)
        layout.addWidget(group)
        hint = QLabel(
            "Калибровочные сервисы вызываются активным ROS-сеансом; панель честно "
            "показывает их как недоступные, если такой сеанс не запущен."
        )
        hint.setWordWrap(True)
        hint.setObjectName("hint")
        layout.addWidget(hint)
        layout.addStretch(1)
        self.tabs.addTab(tab, "VR и очки")

    def _add_video_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox("Видео «глазами робота»")
        grid = QGridLayout(group)
        buttons = [
            ("video_robot", "Запустить видео глазами робота"),
            ("video_vr", "VR-вид от лица робота"),
            ("video_mono", "Mono video"),
            ("video_stereo", "Stereo video (демо)"),
            ("video_low", "Профиль: низкая задержка"),
            ("video_high", "Профиль: высокое качество"),
            ("video_exhibition", "Профиль: выставка"),
            ("video_preflight", "Проверить видеопуть"),
        ]
        for index, (key, text) in enumerate(buttons):
            grid.addWidget(self._action_button(key, text), index // 2, index % 2)
        layout.addWidget(group)
        open_viewer = QPushButton("Открыть viewer в браузере")
        open_viewer.clicked.connect(self.open_viewer)
        layout.addWidget(open_viewer)
        hint = QLabel(
            "Физическая камера R1 сейчас mono; Stereo video (демо) использует "
            "подготовленный mock stereo-источник."
        )
        hint.setWordWrap(True)
        hint.setObjectName("hint")
        layout.addWidget(hint)
        layout.addStretch(1)
        self.tabs.addTab(tab, "Видео глазами робота")

    def _add_motion_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        arms = QGroupBox("Руки")
        arms_grid = QGridLayout(arms)
        arms_grid.addWidget(
            self._action_button("arms_live", "Повторять движения рук из VR"), 0, 0
        )
        layout.addWidget(arms)
        legs = QGroupBox("Ноги")
        legs_grid = QGridLayout(legs)
        legs_grid.addWidget(self._action_button("legs_live", "Включить движение ног"), 0, 0)
        layout.addWidget(legs)
        full = QGroupBox("Полный режим")
        full_grid = QGridLayout(full)
        for index, (key, text) in enumerate(
            [
                ("teleop_dry_run", "Полный VR dry-run"),
                ("teleop_live", "Полный VR teleop"),
                ("teleop_stop", "Остановить VR teleop"),
            ]
        ):
            full_grid.addWidget(self._action_button(key, text), index, 0)
        layout.addWidget(full)
        warning = QLabel(
            "LIVE-кнопки требуют настройки allow_live, чек-листа оператора и "
            "всех существующих подтверждений проекта. Первый запуск должен быть "
            "slow/suspended; KILL остаётся доступен сверху."
        )
        warning.setWordWrap(True)
        warning.setObjectName("warning")
        layout.addWidget(warning)
        layout.addStretch(1)
        self.tabs.addTab(tab, "Руки / ноги / teleop")

    def _add_logs_tab(self) -> None:
        tab = QWidget()
        self.logs_tab = tab
        layout = QVBoxLayout(tab)
        toolbar = QHBoxLayout()
        show_latest = QPushButton("Показать последние логи")
        show_latest.clicked.connect(lambda: self.tabs.setCurrentWidget(self.logs_tab))
        toolbar.addWidget(show_latest)
        clear = QPushButton("Очистить экран логов")
        clear.clicked.connect(lambda: self.log_view.clear())
        toolbar.addWidget(clear)
        open_logs = QPushButton("Открыть папку логов")
        open_logs.clicked.connect(self.open_logs)
        toolbar.addWidget(open_logs)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.log_view.setFont(QFont("Monospace", 9))
        layout.addWidget(self.log_view)
        self.tabs.addTab(tab, "Логи")

    def _card(self, title: str, text: str) -> QGroupBox:
        card = QGroupBox(title)
        label = QLabel(text)
        label.setWordWrap(True)
        label.setObjectName("cardText")
        box = QVBoxLayout(card)
        box.addWidget(label)
        card._value_label = label  # type: ignore[attr-defined]
        return card

    def _action_button(self, key: str, text: str) -> QPushButton:
        button = QPushButton(text)
        button.setMinimumHeight(42)
        button.clicked.connect(lambda _checked=False, k=key: self.run_key(k))
        spec = self.specs.get(key)
        if spec and spec.live:
            button.setProperty("liveAction", True)
            button.setToolTip("Требует safety-чеклист и существующие gates проекта")
        if spec and not spec.implemented:
            button.setProperty("unavailable", True)
            button.setToolTip(spec.note)
        return button

    def _mode_text(self) -> str:
        if self.config.dry_run:
            return "DRY-RUN\nlive выключен"
        return "LIVE-подготовка\nчерез safety gates"

    def _refresh_summary(self) -> None:
        self.mode_label.setText("РЕЖИМ: " + ("DRY-RUN" if self.config.dry_run else "LIVE-подготовка"))
        self.status_label.setText(self.last_status)
        if self.last_status.startswith("Зелёный"):
            color = "#2d8a57"
        elif self.last_status.startswith("Жёлтый"):
            color = "#8c6b24"
        elif self.last_status.startswith("Красный"):
            color = "#8f2f3b"
        else:
            color = "#5b6269"
        self.status_label.setStyleSheet(
            f"padding: 8px; border-radius: 4px; background: {color};"
        )
        self.active_label.setText(
            f"Активных процессов: {len(self.controller.active_keys())}"
        )
        active = self.controller.active_keys()
        self.summary_processes._value_label.setText(  # type: ignore[attr-defined]
            ", ".join(active) if active else "Нет активных процессов"
        )
        self.summary_mode._value_label.setText(self._mode_text())  # type: ignore[attr-defined]

    def run_key(self, key: str) -> None:
        spec = self.specs[key]
        if not spec.implemented:
            QMessageBox.information(self, spec.title, spec.note)
            return
        if key == "check_all":
            self._run_check_all()
            return
        if spec.live and not self._confirm_live(spec):
            return
        if spec.category == "video":
            process_key = "video"
        elif spec.category in ("arms", "legs", "full", "vr"):
            process_key = "motion"
        elif key in ("prepare", "stand"):
            process_key = "robot:prepare"
        else:
            process_key = spec.category + ":" + key
        if spec.category == "robot" and key in ("stop", "kill"):
            process_key = key
        self.controller.start(process_key, spec)

    def _run_check_all(self) -> None:
        command = (
            "set +e; "
            "echo '=== Сеть и Ethernet ==='; make r1-lan-preflight; "
            "echo '=== Робот / VR / ROS2 ==='; make robot-preflight; "
            "echo '=== Камера и DDS ==='; make r1-camera-preflight; "
            "echo '=== Сводный статус ==='; make r1-teleoperation-status; "
            "exit 0"
        )
        spec = self.specs["check_all"]
        self.controller.start("check_all", spec, shell_command=command)

    def _confirm_live(self, spec: CommandSpec) -> bool:
        if not self.config.allow_live:
            QMessageBox.warning(
                self,
                "Live-режим отключён",
                "В настройках включите «Разрешить кнопкам запрашивать live-сеанс». "
                "По умолчанию панель работает в dry-run.",
            )
            return False
        dialog = QDialog(self)
        dialog.setWindowTitle("Проверка безопасности перед live")
        layout = QVBoxLayout(dialog)
        layout.addWidget(QLabel(f"Перед запуском: {spec.title}"))
        checks = []
        for text in (
            "Робот стоит устойчиво на полу или надёжно подвешен для slow-теста",
            "Вокруг робота свободная зона",
            "Человек рядом готов нажать физический E-stop",
            "VR-шлем и контроллеры готовы, deadman проверен",
        ):
            check = QCheckBox(text)
            layout.addWidget(check)
            checks.append(check)
        note = QLabel(
            "Панель не подставляет commissioning token и не обходит проверки "
            "скриптов. При любой ошибке процесс завершится fail-closed."
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        if dialog.exec_() != QDialog.Accepted:
            return False
        if not all(check.isChecked() for check in checks):
            QMessageBox.warning(self, "Запуск отменён", "Нужно отметить весь safety-чеклист.")
            return False
        return True

    def _on_output(self, key: str, text: str) -> None:
        self._append_log(key, text)
        if "[FAIL]" in text or "[BLOCKED]" in text:
            self.last_status = "Красный • обнаружена ошибка или блокировка"
        elif "[WARN]" in text:
            self.last_status = "Жёлтый • есть предупреждения"
        elif "[OK]" in text or "[PASS]" in text:
            self.last_status = "Зелёный • последняя проверка прошла"
        self._refresh_summary()

    def _append_log(self, key: str, text: str) -> None:
        stamp = _datetime.datetime.now().strftime("%H:%M:%S")
        for line in text.splitlines(True):
            self.log_view.appendPlainText(f"[{stamp}] [{key}] {line.rstrip()}" )

    def _on_started(self, key: str) -> None:
        self._append_log(key, "[INFO] процесс запущен\n")
        self._refresh_summary()

    def _on_finished(self, key: str, code: int, _status: int) -> None:
        level = "OK" if code == 0 else "WARN"
        self._append_log(key, f"[{level}] процесс завершён с кодом {code}\n")
        self._refresh_summary()
        if self.close_after_stop and not self.controller.active_keys():
            QApplication.instance().quit()

    def open_viewer(self) -> None:
        url = self.config.video_url.strip() or "http://127.0.0.1:8080/"
        QDesktopServices.openUrl(QUrl(url))
        self._append_log("viewer", f"[INFO] открываю {url}\n")

    def open_logs(self) -> None:
        path = Path(self.config.project_dir) / self.config.logs_dir
        path.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def open_settings(self) -> None:
        dialog = SettingsDialog(self.config, self)
        if dialog.exec_() != QDialog.Accepted:
            return
        new_config = dialog.values()
        self.config = new_config
        self.controller.config = new_config
        try:
            destination = save_config(new_config)
            self._append_log("settings", f"[OK] настройки сохранены: {destination}\n")
        except OSError as exc:
            QMessageBox.warning(self, "Ошибка сохранения", str(exc))
        self.summary_network._value_label.setText(  # type: ignore[attr-defined]
            f"{new_config.robot_interface}\nDDS {new_config.robot_ip}\nPC2 {new_config.pc2_ip}"
        )
        self.summary_video._value_label.setText(new_config.video_url)  # type: ignore[attr-defined]
        self._refresh_summary()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API name
        active = self.controller.active_keys()
        if not active:
            event.accept()
            return
        answer = QMessageBox.question(
            self,
            "Активные процессы",
            "Есть активные процессы. Остановить их перед закрытием?",
            QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel,
            QMessageBox.Yes,
        )
        if answer == QMessageBox.Cancel:
            event.ignore()
            return
        if answer == QMessageBox.Yes:
            self.controller.stop_all()
        event.accept()


def build_app(config: Optional[OperatorConfig] = None) -> QApplication:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(
        """
        QWidget { font-size: 11pt; }
        QMainWindow, QWidget { background: #20252b; color: #eef2f5; }
        QGroupBox { border: 1px solid #46515c; border-radius: 6px; margin-top: 10px; padding: 12px; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #a9d6ff; }
        QPushButton { background: #35414c; border: 1px solid #5b6b79; border-radius: 5px; padding: 8px 12px; }
        QPushButton:hover { background: #435463; }
        QPushButton[liveAction="true"] { border-color: #d39b3b; }
        QPushButton[unavailable="true"] { color: #89939d; border-color: #555e66; }
        #stopButton { background: #a86a22; border-color: #e8a849; font-weight: bold; }
        #killButton { background: #9d2632; border-color: #ff6875; font-weight: bold; }
        #appTitle { color: #9ed5ff; }
        #modeLabel { color: #ffd166; font-weight: bold; padding: 8px; }
        #statusLabel { padding: 8px; background: #2c343c; border-radius: 4px; }
        #hint { color: #b8c2ca; padding: 8px; }
        #warning { color: #ffd166; padding: 8px; }
        #cardText { color: #d7e4ee; padding: 10px; }
        QPlainTextEdit { background: #15191d; color: #dbe7ef; }
        QTabBar::tab { padding: 10px 14px; }
        """
    )
    return app


def main(argv: Optional[Iterable[str]] = None) -> int:
    argv = list(argv or sys.argv[1:])
    app = build_app()
    panel = OperatorPanel()
    panel.show()
    if "--smoke" in argv:
        print("operator-panel smoke: window constructed")
        QTimer.singleShot(0, app.quit)
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
