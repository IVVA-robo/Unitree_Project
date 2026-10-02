"""Russian Qt operator panel for Unitree R1 Teleoperation."""

from __future__ import annotations

import datetime as _datetime
import ipaddress
import json
import secrets
import shlex
import subprocess
import re
import shutil
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, Optional

from PyQt5.QtCore import QSize, QTimer, QUrl, Qt
from PyQt5.QtGui import QColor, QDesktopServices, QFont, QIcon, QKeySequence, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
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
    QScrollArea,
    QShortcut,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .commands import CommandSpec, command_catalog
from exhibition.orchestrator import active_session_snapshot
from .config import OperatorConfig, load_config, save_config
from .live_ack import (
    acknowledgement_valid,
    clear_acknowledgement,
    record_acknowledgement,
)
from .processes import ProcessController
from .video_preview import VideoPreview


# The exhibition manager may spend up to 8 s disarming, 20 s in reviewed
# robot-stop, 12 s in its kill fallback, and then reap separate POV/control
# process groups.  Keep this asynchronous so the UI and KILL button remain
# responsive while giving the manager enough time to avoid orphaned children.
EXHIBITION_GRACEFUL_STOP_MS = 60_000
# A cold read-only SDK preflight normally completes in 30-40 seconds. If an
# operator requests STAND/RUN while that exact check is already in flight,
# preserve its progress briefly instead of cancelling it and starting the same
# preflight again inside the exhibition manager. This timer never authorizes
# motion: after it expires the normal manager still performs every gate.
SDK_WARMUP_HANDOFF_WAIT_MS = 45_000
ZERO_TORQUE_HANDOFF_KEY = "zero_torque_offline_handoff"
ZERO_TORQUE_HANDOFF_COMMAND = (
    "exec ./scripts/r1-exhibition-offline-handoff stop"
)

# These are operator-facing deadlines, not shortcuts around safety checks. A
# deadline only changes the UI to a concrete retry message; it never kills a
# physical owner or treats an unfinished transition as successful.
MAIN_ACTION_TIMEOUTS_SEC = {
    "connect": 12.0,
    # Warm RUN -> LOCK normally confirms in a few seconds, but the same
    # button is also allowed to create the first static/StandUp session.  That
    # cold path keeps its complete SDK/FSM gates and can legitimately take
    # well over 20 seconds, so it shares the bounded physical-mode deadline.
    "exhibition_static": 300.0,
    "exhibition_control": 300.0,
    "exhibition_stand": 300.0,
    "zero_torque": 90.0,
    "exhibition_reconnect": 20.0,
    "viewer": 20.0,
}
MAIN_ACTION_TITLES = {
    "connect": "НАЙТИ И ПОДКЛЮЧИТЬ",
    "exhibition_static": "LOCK / СТАТИЧНЫЙ РЕЖИМ",
    "exhibition_control": "RUN / ПОЛНОЕ УПРАВЛЕНИЕ",
    "exhibition_stand": "СТОЙКА",
    "zero_torque": "ZERO TORQUE / РАССЛАБИТЬ",
    "exhibition_reconnect": "Переподключить всё",
    "viewer": "Открыть Robot POV",
}


def _is_headset_lan_address(value: str) -> bool:
    """Accept the exhibition Wi-Fi range as well as ordinary private LANs."""
    try:
        address = ipaddress.ip_address(value)
    except ValueError:
        return False
    if address.version != 4 or address.is_loopback or address.is_multicast:
        return False
    return address.is_private or ipaddress.ip_network("100.64.0.0/10").supernet_of(
        ipaddress.ip_network(f"{address}/32")
    )


def _discover_headset_ip() -> Optional[str]:
    """Read the one authorized Android headset address without touching control."""
    adb = shutil.which("adb")
    if not adb:
        return None
    try:
        devices = subprocess.run(
            [adb, "devices"], capture_output=True, text=True,
            timeout=2.0, check=False,
        ).stdout.splitlines()
        serials = [
            line.split()[0] for line in devices[1:]
            if len(line.split()) >= 2 and line.split()[1] == "device"
        ]
        if len(serials) != 1:
            return None
        output = subprocess.run(
            [adb, "-s", serials[0], "shell", "ip", "-o", "-4", "addr", "show", "dev", "wlan0"],
            capture_output=True, text=True, timeout=2.0, check=False,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for candidate in re.findall(r"\binet\s+(\d+\.\d+\.\d+\.\d+)/", output):
        if _is_headset_lan_address(candidate):
            return candidate
    return None


def _tuning_spin(value, minimum, maximum, step, suffix):
    """Build a compact bounded editor for one exhibition tuning value."""
    spin = QDoubleSpinBox()
    spin.setDecimals(2)
    spin.setRange(minimum, maximum)
    spin.setSingleStep(step)
    spin.setValue(value)
    spin.setSuffix(suffix)
    return spin


class BatteryRing(QWidget):
    """Compact circular battery gauge compatible with the old QLabel API."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._text = "—%"
        self._percent: Optional[int] = None
        self.setObjectName("batteryRing")
        self.setMinimumSize(86, 86)

    def sizeHint(self) -> QSize:  # noqa: N802 - Qt virtual method
        return QSize(96, 96)

    def minimumSizeHint(self) -> QSize:  # noqa: N802 - Qt virtual method
        return QSize(86, 86)

    def text(self) -> str:
        return self._text

    def setText(self, text: str) -> None:
        self._text = str(text)
        match = re.search(r"(\d{1,3})", self._text)
        self._percent = max(0, min(100, int(match.group(1)))) if match else None
        self.update()

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt virtual method
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        side = max(24, min(self.width(), self.height()) - 14)
        left = (self.width() - side) / 2
        top = (self.height() - side) / 2
        rect = self.rect().adjusted(
            int(left), int(top), -int(left), -int(top)
        )

        track = QPen(QColor("#3a3a3c"), 9, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(track)
        painter.drawArc(rect, 90 * 16, -360 * 16)

        if self._percent is None:
            accent = QColor("#636366")
            span = 0
        elif self._percent <= 20:
            accent = QColor("#ff453a")
            span = int(-360 * 16 * self._percent / 100)
        elif self._percent <= 40:
            accent = QColor("#ff9f0a")
            span = int(-360 * 16 * self._percent / 100)
        else:
            accent = QColor("#30d158")
            span = int(-360 * 16 * self._percent / 100)
        if span:
            painter.setPen(QPen(accent, 9, Qt.SolidLine, Qt.RoundCap))
            painter.drawArc(rect, 90 * 16, span)

        painter.setPen(QColor("#f5f5f7"))
        painter.setFont(QFont("Inter", 15, QFont.Bold))
        painter.drawText(self.rect(), Qt.AlignCenter, self._text)


class SettingsDialog(QDialog):
    def __init__(self, config: OperatorConfig, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setObjectName("settingsDialog")
        self.setWindowTitle("Настройки панели")
        self.setModal(True)
        self.setMinimumSize(540, 480)
        self.resize(580, 720)
        self.config = config

        dialog_layout = QVBoxLayout(self)
        dialog_layout.setContentsMargins(12, 12, 12, 12)
        dialog_layout.setSpacing(10)

        self.settings_scroll = QScrollArea(self)
        self.settings_scroll.setObjectName("settingsScroll")
        self.settings_scroll.setWidgetResizable(True)
        self.settings_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.settings_scroll.viewport().setObjectName("settingsViewport")
        settings_bar = self.settings_scroll.verticalScrollBar()
        settings_bar.setObjectName("settingsVerticalScroll")
        settings_bar.setStyleSheet(
            """
            QScrollBar:vertical {
                background: #1c1c1e;
                border: none;
                width: 10px;
                margin: 0;
            }
            QScrollBar::handle:vertical {
                background: #636366;
                border-radius: 5px;
                min-height: 32px;
            }
            QScrollBar::handle:vertical:hover { background: #8e8e93; }
            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical { height: 0; width: 0; }
            QScrollBar::add-page:vertical,
            QScrollBar::sub-page:vertical { background: transparent; }
            """
        )
        self.settings_content = QWidget()
        self.settings_content.setObjectName("settingsContent")
        form = QFormLayout(self.settings_content)
        form.setContentsMargins(8, 4, 10, 8)
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(8)
        self.project_dir = QLineEdit(config.project_dir)
        self.robot_name = QLineEdit(config.robot_name)
        self.robot_ip = QLineEdit(config.robot_ip)
        self.pc2_ip = QLineEdit(config.pc2_ip)
        self.robot_interface = QLineEdit(config.robot_interface)
        self.allow_half_duplex_adapter = QCheckBox(
            "Разрешить half-duplex только для TP-Link cdc_ether"
        )
        self.allow_half_duplex_adapter.setChecked(
            config.allow_half_duplex_adapter
        )
        self.allow_half_duplex_adapter.setToolTip(
            "Не отключает проверки сети и робота: принимается только драйвер "
            "cdc_ether при чистых RX-счётчиках, доступном роботе и свежей "
            "телеметрии. Для другого адаптера preflight останется закрыт."
        )
        self.laptop_robot_ip = QLineEdit(config.laptop_robot_ip)
        self.ros_domain = QSpinBox()
        self.ros_domain.setRange(0, 232)
        self.ros_domain.setValue(config.ros_domain_id)
        self.vr_ip = QLineEdit(config.vr_headset_ip)
        self.vr_transport = QComboBox()
        self.vr_transport.addItem("USB-C — без Wi-Fi", "usb")
        self.vr_transport.addItem("Wi-Fi / локальная сеть", "lan")
        self.vr_transport.setCurrentIndex(
            max(0, self.vr_transport.findData(config.vr_transport))
        )
        self.video_url = QLineEdit(config.video_url)
        self.logs_dir = QLineEdit(config.logs_dir)
        self.video_profile = QLineEdit(config.video_profile)
        self.locomotion_profile = QLineEdit(config.locomotion_profile)
        self.dry_run = QCheckBox("Безопасный dry-run по умолчанию")
        self.dry_run.setChecked(config.dry_run)
        self.allow_live = QCheckBox("Разрешить кнопкам запрашивать live-сеанс")
        self.allow_live.setChecked(config.allow_live)
        self.require_preflight = QCheckBox("Требовать успешный preflight перед live")
        self.require_preflight.setChecked(config.require_preflight)
        self.auto_start_video = QCheckBox("Автозапуск только video-only POV")
        self.auto_start_video.setChecked(config.auto_start_video)
        self.auto_start_bridge = QCheckBox("Автозапуск offline bridge + video-only POV")
        self.auto_start_bridge.setChecked(config.auto_start_bridge)
        self.shoulder_height_offset = _tuning_spin(
            config.shoulder_height_offset_m, -0.15, 0.15, 0.01, " м"
        )
        self.shoulder_forward_offset = _tuning_spin(
            config.shoulder_forward_offset_m, -0.12, 0.08, 0.01, " м"
        )
        self.shoulder_width = _tuning_spin(
            config.shoulder_width_m, 0.25, 0.55, 0.01, " м"
        )
        self.arm_motion_scale = _tuning_spin(
            config.arm_motion_scale, 0.50, 1.20, 0.05, "×"
        )
        self.turn_sensitivity = _tuning_spin(
            config.turn_sensitivity, 0.10, 1.00, 0.05, "×"
        )
        self.leg_speed_scale = _tuning_spin(
            config.leg_speed_scale, 0.10, 1.00, 0.05, "×"
        )
        calibration_hint = (
            "Применится при следующей калибровке рук; сохранённая "
            "калибровка не переписывается автоматически."
        )
        for widget in (
            self.shoulder_height_offset,
            self.shoulder_forward_offset,
            self.shoulder_width,
        ):
            widget.setToolTip(calibration_hint)
        self.arm_motion_scale.setToolTip(
            "Масштаб движения применяется без повторной калибровки."
        )
        self.response_profile = QComboBox()
        self.response_profile.addItem("Выставочный — быстрый отклик", "exhibition")
        self.response_profile.addItem("Прежний — плавный отклик", "standard")
        self.response_profile.setCurrentIndex(
            max(0, self.response_profile.findData(config.response_profile))
        )
        self.response_profile.setToolTip(
            "Меняет скорость рук и головы при следующем полном запуске. "
            "Пределы положения суставов и скорость ходьбы сохраняются."
        )
        self.turn_sensitivity.setToolTip(
            "Только уменьшает slow-safe предел поворота 0,35 рад/с."
        )
        self.leg_speed_scale.setToolTip(
            "Только уменьшает slow-safe пределы 0,20/0,12 м/с."
        )
        form.addRow("Путь проекта", self.project_dir)
        form.addRow("Имя робота", self.robot_name)
        form.addRow("IP робота / DDS", self.robot_ip)
        form.addRow("IP PC2 / диагностика кабеля", self.pc2_ip)
        form.addRow("Ethernet-интерфейс", self.robot_interface)
        form.addRow(self.allow_half_duplex_adapter)
        form.addRow("IP ноутбука на Ethernet", self.laptop_robot_ip)
        form.addRow("ROS_DOMAIN_ID", self.ros_domain)
        form.addRow("IP VR-шлема", self.vr_ip)
        form.addRow("Подключение очков", self.vr_transport)
        form.addRow("URL Robot POV", self.video_url)
        form.addRow("Папка логов", self.logs_dir)
        form.addRow("Профиль видео", self.video_profile)
        form.addRow("Профиль locomotion", self.locomotion_profile)
        tuning_title = QLabel("<b>Настройка движений для выставки</b>")
        form.addRow(tuning_title)
        form.addRow("Высота рук", self.shoulder_height_offset)
        form.addRow("Руки вперёд / назад", self.shoulder_forward_offset)
        form.addRow("Ширина плеч", self.shoulder_width)
        form.addRow("Чувствительность рук", self.arm_motion_scale)
        form.addRow("Отклик рук и головы", self.response_profile)
        form.addRow("Чувствительность поворота", self.turn_sensitivity)
        form.addRow("Скорость ног", self.leg_speed_scale)
        form.addRow(self.dry_run)
        form.addRow(self.allow_live)
        form.addRow(self.require_preflight)
        form.addRow(self.auto_start_video)
        form.addRow(self.auto_start_bridge)
        note = QLabel(
            "Live-кнопки всё равно проходят существующие safety gates проекта. "
            "После ручного safety-чеклиста панель создаёт одноразовый токен; "
            "он не сохраняется в конфиге или отчётах."
        )
        note.setWordWrap(True)
        form.addRow(note)
        self.settings_scroll.setWidget(self.settings_content)
        dialog_layout.addWidget(self.settings_scroll, 1)

        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Save).setText("Сохранить")
        buttons.button(QDialogButtonBox.Cancel).setText("Отмена")
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        self.settings_buttons = buttons
        dialog_layout.addWidget(buttons)

    def values(self) -> OperatorConfig:
        return OperatorConfig(
            project_dir=self.project_dir.text().strip(),
            robot_name=self.robot_name.text().strip() or "R1",
            robot_ip=self.robot_ip.text().strip(),
            pc2_ip=self.pc2_ip.text().strip(),
            robot_interface=self.robot_interface.text().strip(),
            allow_half_duplex_adapter=(
                self.allow_half_duplex_adapter.isChecked()
            ),
            laptop_robot_ip=self.laptop_robot_ip.text().strip(),
            ros_domain_id=self.ros_domain.value(),
            vr_headset_ip=self.vr_ip.text().strip(),
            vr_transport=self.vr_transport.currentData(),
            video_url=self.video_url.text().strip(),
            logs_dir=self.logs_dir.text().strip(),
            video_profile=self.video_profile.text().strip() or "low-latency",
            locomotion_profile=self.locomotion_profile.text().strip() or "slow-safe",
            dry_run=self.dry_run.isChecked(),
            allow_live=self.allow_live.isChecked(),
            require_preflight=self.require_preflight.isChecked(),
            auto_start_video=self.auto_start_video.isChecked(),
            auto_start_bridge=self.auto_start_bridge.isChecked(),
            status_poll_sec=self.config.status_poll_sec,
            shoulder_height_offset_m=self.shoulder_height_offset.value(),
            shoulder_forward_offset_m=self.shoulder_forward_offset.value(),
            shoulder_width_m=self.shoulder_width.value(),
            arm_motion_scale=self.arm_motion_scale.value(),
            response_profile=self.response_profile.currentData(),
            turn_sensitivity=self.turn_sensitivity.value(),
            leg_speed_scale=self.leg_speed_scale.value(),
        )


class OperatorPanel(QMainWindow):
    """Main window.  All long-running actions are child processes, never UI threads."""

    def __init__(self, config: Optional[OperatorConfig] = None):
        super().__init__()
        # Keep native desktop decorations enabled and explicitly request the
        # maximize control.  Some Linux themes hide the square button unless
        # the hint is present even for a normal QMainWindow.
        self.setWindowFlags(
            Qt.Window
            | Qt.WindowTitleHint
            | Qt.WindowSystemMenuHint
            | Qt.WindowMinimizeButtonHint
            | Qt.WindowMaximizeButtonHint
            | Qt.WindowCloseButtonHint
        )
        self.config = config or load_config()
        self.controller = ProcessController(self.config, self)
        self.specs: Dict[str, CommandSpec] = {
            spec.key: spec for spec in command_catalog()
        }
        self.close_after_stop = False
        self.preflight_ok = False
        self.exhibition_mode = "Остановлен"
        self.exhibition_requested_mode = ""
        self.pending_exhibition_key: Optional[str] = None
        self.pending_exhibition_environment: Optional[Dict[str, str]] = None
        self.pending_warmup_key: Optional[str] = None
        self.pending_warmup_environment: Optional[Dict[str, str]] = None
        self.warmup_handoff_waiting = False
        self.warmup_handoff_generation = 0
        self.sdk_warmup_cache_available = False
        # A recovered Ethernet link needs a fresh worker so interface/IP
        # autodetection and the private SDK attestation cannot stay tied to
        # the pre-disconnect process.  This is deliberately separate from
        # pending_warmup_key, which belongs to an operator-requested physical
        # mode and always has priority over background work.
        self.warmup_restart_pending = False
        self.warmup_robot_offline_observed = False
        self.exhibition_session_confirmed = acknowledgement_valid(self.config)
        self.exhibition_switch_in_progress = False
        self.exhibition_stop_complete = False
        self.exhibition_manager_stopped = False
        self.pending_zero_torque = False
        self.zero_torque_deadline = 0.0
        self.zero_torque_idle = False
        # Zero Torque also owns the video-only systemd session.  Keeping this
        # state separate from the panel's QProcesses matters when the offline
        # service was started before the panel: its ROS children can otherwise
        # retain UDP/TCP ports after the exhibition manager has exited.
        self.zero_torque_handoff_started = False
        self.zero_torque_handoff_complete = False
        self.video_state = "offline"
        self.connection_state = "offline"
        self.background_services_started = False
        self.last_capture_path = ""
        self.action_started_at: Dict[str, float] = {}
        self.action_generations: Dict[str, int] = {}
        self.last_action_timings: Dict[str, Dict[str, object]] = {}
        self.output_line_buffers: Dict[str, str] = {}
        self.last_process_errors: Dict[str, str] = {}
        # One ephemeral token is shared by every live child started during
        # this panel session. It is never written to config or exported logs.
        self.live_session_token = secrets.token_urlsafe(24)
        self.status_values = {}
        self.last_status = "Серый • диагностика ещё не запускалась"
        self._normal_geometry = None
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
        self.fullscreen_shortcut = QShortcut(QKeySequence("F11"), self)
        self.fullscreen_shortcut.setContext(Qt.WindowShortcut)
        self.fullscreen_shortcut.activated.connect(self.toggle_fullscreen)
        self.escape_fullscreen_shortcut = QShortcut(QKeySequence("Esc"), self)
        self.escape_fullscreen_shortcut.setContext(Qt.WindowShortcut)
        self.escape_fullscreen_shortcut.activated.connect(self.exit_fullscreen)
        self._connect_controller()
        self.status_timer = QTimer(self)
        self.status_timer.setInterval(max(1000, int(self.config.status_poll_sec * 1000)))
        self.status_timer.timeout.connect(self._poll_panel_status)
        self.status_timer.start()
        self._refresh_summary()

    def _build_ui(self) -> None:
        root = QWidget()
        root.setObjectName("rootPanel")
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        # Keep the operator dashboard usable on a second monitor with a
        # shorter work area.  The native window controls remain available;
        # this only removes decorative empty space around the content.
        layout.setContentsMargins(8, 8, 8, 6)
        layout.setSpacing(6)

        header_card = QWidget()
        self.header_card = header_card
        header_card.setObjectName("headerCard")
        header = QGridLayout(header_card)
        header.setContentsMargins(14, 10, 12, 10)
        header.setHorizontalSpacing(10)
        header.setVerticalSpacing(6)

        title = QLabel("UNITREE R1  •  ПАНЕЛЬ ОПЕРАТОРА")
        title.setObjectName("appTitle")
        title.setFont(QFont("Inter", 15, QFont.Bold))
        brand = QWidget()
        brand_layout = QVBoxLayout(brand)
        brand_layout.setContentsMargins(0, 0, 0, 0)
        brand_layout.setSpacing(4)
        title_row = QHBoxLayout()
        title_row.setContentsMargins(0, 0, 0, 0)
        title_row.setSpacing(8)
        robot_icon = QLabel()
        robot_icon.setObjectName("brandIcon")
        icon_path = (
            Path(__file__).resolve().parents[1]
            / "assets"
            / "icons"
            / "unitree-r1-robot-64.png"
        )
        if icon_path.is_file():
            robot_icon.setPixmap(QIcon(str(icon_path)).pixmap(28, 28))
        title_row.addWidget(robot_icon)
        title_row.addWidget(title)
        title_row.addStretch(1)
        brand_layout.addLayout(title_row)
        brand_subtitle = QLabel("↔  ETHERNET + USB-C   •   ПОЛНОСТЬЮ ОФЛАЙН")
        brand_subtitle.setObjectName("brandSubtitle")
        brand_layout.addWidget(brand_subtitle)

        self.mode_label = QLabel()
        self.mode_label.setObjectName("modeLabel")
        self.mode_label.setAlignment(Qt.AlignCenter)

        actions_widget = QWidget()
        self.header_actions = actions_widget
        actions = QHBoxLayout(actions_widget)
        actions.setContentsMargins(0, 0, 0, 0)
        actions.setSpacing(6)
        self.service_toggle = QPushButton("⚙  Расширенные настройки")
        self.service_toggle.setObjectName("headerAction")
        self.service_toggle.setMinimumHeight(44)
        self.service_toggle.clicked.connect(self.toggle_service_view)
        actions.addWidget(self.service_toggle)
        self.help_button = QPushButton("?  Как запустить")
        self.help_button.setObjectName("headerAction")
        self.help_button.setMinimumHeight(44)
        self.help_button.clicked.connect(self.show_quick_start)
        actions.addWidget(self.help_button)
        self.settings_button = QPushButton("⚙  Настройки соединения")
        self.settings_button.setObjectName("headerAction")
        self.settings_button.setMinimumHeight(44)
        self.settings_button.clicked.connect(self.open_settings)
        actions.addWidget(self.settings_button)

        summary = QWidget()
        summary_layout = QHBoxLayout(summary)
        summary_layout.setContentsMargins(0, 0, 0, 0)
        summary_layout.setSpacing(8)
        self.status_label = QLabel()
        self.status_label.setObjectName("statusLabel")
        summary_layout.addWidget(self.status_label, 1)
        self.active_label = QLabel("Активных процессов: 0")
        self.active_label.setObjectName("activeLabel")
        summary_layout.addWidget(self.active_label)

        header.addWidget(brand, 0, 0)
        header.addWidget(self.mode_label, 0, 1)
        header.addWidget(actions_widget, 0, 2, 2, 1)
        header.addWidget(summary, 1, 0, 1, 2)
        header.setColumnStretch(0, 3)
        header.setColumnStretch(1, 2)
        layout.addWidget(header_card)

        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, 1)
        self._add_exhibition_tab()
        self.service_tabs = QTabWidget()
        self.tabs.addTab(self.service_tabs, "Расширенные настройки / Сервис")
        self._add_general_tab()
        self._add_diagnostics_tab()
        self._add_robot_tab()
        self._add_vr_tab()
        self._add_voice_tab()
        self._add_command_capture_tab()
        self._add_video_tab()
        self._add_motion_tab()
        self._add_logs_tab()
        self.tabs.tabBar().hide()
        self._refresh_header_controls()

    def _add_exhibition_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(4)

        # Exhibition layout: controls on the left, a flexible video canvas in
        # the center, and safety/health on the right.  Each column is allowed
        # to grow with the window; the video receives the largest share.
        dashboard = QGridLayout()
        dashboard.setContentsMargins(0, 0, 0, 0)
        dashboard.setHorizontalSpacing(8)
        dashboard.setVerticalSpacing(4)

        video_group = QGroupBox("Видео глазами робота")
        video_group.setObjectName("videoGroup")
        video_layout = QVBoxLayout(video_group)
        video_layout.setContentsMargins(6, 6, 6, 6)
        self.video_preview = VideoPreview(
            self.config.video_url, self.config.video_profile, video_group
        )
        self.video_preview.state_changed.connect(self._on_video_state)
        video_layout.addWidget(self.video_preview)
        self.controller_action_hint = QLabel(
            "B справа — аварийная остановка  •  X слева — нейтраль рук  •  "
            "стики — ходьба; отпустите стик — возврат рук/головы  •  "
            "STOP/KILL — в «Сервисе»."
        )
        self.controller_action_hint.setObjectName("controllerHint")
        self.controller_action_hint.setAlignment(Qt.AlignCenter)
        self.controller_action_hint.setWordWrap(True)
        self.controller_action_hint.setMaximumHeight(40)
        video_layout.addWidget(self.controller_action_hint)

        left_column = QVBoxLayout()
        left_column.setSpacing(4)
        device = QGroupBox("Подключение и заряд")
        device.setObjectName("deviceGroup")
        device_layout = QGridLayout(device)
        device_layout.setContentsMargins(8, 6, 8, 6)
        device_layout.setHorizontalSpacing(8)
        device_layout.setVerticalSpacing(4)
        self.robot_name_label = QLabel(f"○  {self.config.robot_name}  — не в сети")
        self.robot_name_label.setObjectName("robotName")
        self.robot_name_label.setFont(QFont("Inter", 11, QFont.Bold))
        self.robot_name_label.setWordWrap(True)
        device_layout.addWidget(self.robot_name_label, 0, 1)
        self.battery_label = BatteryRing(device)
        device_layout.addWidget(self.battery_label, 0, 0, 2, 1)
        self.connect_button = QPushButton("↻  НАЙТИ И ПОДКЛЮЧИТЬ")
        self.connect_button.setObjectName("connectButton")
        self.connect_button.clicked.connect(self.auto_connect)
        self.connect_button.setMinimumHeight(34)
        device_layout.addWidget(self.connect_button, 1, 1)
        device_layout.setColumnStretch(1, 1)
        left_column.addWidget(device)

        mode_group = QGroupBox("Режим управления")
        mode_group.setObjectName("modeGroup")
        mode_layout = QVBoxLayout(mode_group)
        mode_layout.setContentsMargins(6, 4, 6, 4)
        mode_layout.setSpacing(3)
        self.static_mode_button = self._action_button(
            "exhibition_static", "LOCK / СТАТИЧНЫЙ РЕЖИМ\nВидео и устойчивое положение"
        )
        self.static_mode_button.setObjectName("staticModeButton")
        self.static_mode_button.setToolTip("")
        self.static_mode_button.setMinimumHeight(50)
        self.static_mode_button.setFont(QFont("Inter", 10, QFont.Bold))
        mode_layout.addWidget(self.static_mode_button)
        self.control_mode_button = self._action_button(
            "exhibition_control", "RUN / ПОЛНОЕ УПРАВЛЕНИЕ\nVR: голова, руки и ноги"
        )
        self.control_mode_button.setObjectName("controlModeButton")
        self.control_mode_button.setToolTip("")
        self.control_mode_button.setMinimumHeight(52)
        self.control_mode_button.setFont(QFont("Inter", 10, QFont.Bold))
        mode_layout.addWidget(self.control_mode_button)
        self.stand_mode_button = self._action_button(
            "exhibition_stand", "СТОЙКА\nШтатная поза • моторы удерживают"
        )
        self.stand_mode_button.setObjectName("standModeButton")
        self.stand_mode_button.setToolTip("")
        self.stand_mode_button.setMinimumHeight(50)
        self.stand_mode_button.setFont(QFont("Inter", 10, QFont.Bold))
        mode_layout.addWidget(self.stand_mode_button)
        left_column.addWidget(mode_group)

        # Keep the physical relaxation action visible on the operator's
        # normal screen.  It still uses the reviewed confirmation and cleanup
        # path in request_zero_torque(); this button only makes that path easy
        # to find when the operator has no time to open the service tab.
        self.zero_torque_button = QPushButton(
            "ZERO TORQUE  •  РАССЛАБИТЬ"
        )
        self.zero_torque_button.setObjectName("zeroTorqueButton")
        self.zero_torque_button.setMinimumHeight(42)
        self.zero_torque_button.setFont(QFont("Inter", 10, QFont.Bold))
        self.zero_torque_button.clicked.connect(self.request_zero_torque)
        safety_group = QGroupBox("Безопасное завершение")
        safety_group.setObjectName("safetyGroup")
        safety_layout = QVBoxLayout(safety_group)
        safety_layout.setContentsMargins(6, 4, 6, 4)
        safety_layout.setSpacing(2)
        safety_layout.addWidget(self.zero_torque_button)
        safety_hint = QLabel("Только на опоре или после СТОЙКИ.")
        safety_hint.setObjectName("safetyHint")
        safety_hint.setWordWrap(True)
        safety_hint.setMaximumHeight(22)
        safety_layout.addWidget(safety_hint)
        left_column.addWidget(safety_group)
        left_column.addStretch(1)

        status_group = QGroupBox("Состояние системы")
        status_group.setObjectName("statusGroup")
        status_layout = QVBoxLayout(status_group)
        status_layout.setContentsMargins(6, 4, 6, 4)
        status_layout.setSpacing(3)
        self.exhibition_robot_status = QLabel()
        self.exhibition_vr_status = QLabel()
        self.exhibition_controllers_status = QLabel()
        self.exhibition_video_status = QLabel()
        self.exhibition_mode_status = QLabel()
        status_labels = (
            self.exhibition_robot_status,
            self.exhibition_vr_status,
            self.exhibition_controllers_status,
            self.exhibition_video_status,
            self.exhibition_mode_status,
        )
        for label in status_labels:
            label.setObjectName("simpleStatus")
            label.setMinimumHeight(26)
            label.setWordWrap(True)
            status_layout.addWidget(label)

        right_column = QVBoxLayout()
        right_column.setSpacing(4)
        right_column.addWidget(status_group)
        right_column.addStretch(1)

        reconnect = self._action_button(
            "exhibition_reconnect", "↻  Переподключить всё"
        )
        reconnect.setObjectName("reconnectButton")
        reconnect.setToolTip("")
        reconnect.setMinimumHeight(36)
        right_column.addWidget(reconnect)
        open_viewer = QPushButton("▣  Открыть Robot POV")
        open_viewer.setObjectName("openPovButton")
        open_viewer.setMinimumHeight(36)
        open_viewer.clicked.connect(self.open_viewer)
        right_column.addWidget(open_viewer)

        dashboard.addLayout(left_column, 0, 0)
        dashboard.addWidget(video_group, 0, 1)
        dashboard.addLayout(right_column, 0, 2)
        dashboard.setColumnStretch(0, 2)
        dashboard.setColumnStretch(1, 5)
        dashboard.setColumnStretch(2, 2)
        dashboard.setRowStretch(0, 1)
        layout.addLayout(dashboard, 1)

        self.action_timing_status = QLabel("Последнее действие: —")
        self.action_timing_status.setObjectName("footerStatus")
        self.action_timing_status.setProperty("statusRole", "timing")
        self.action_timing_status.setMinimumHeight(28)
        self.action_timing_status.setWordWrap(True)

        self.operator_instruction = QLabel(
            "Включите робота и очки  →  дождитесь «ПОДКЛЮЧЕНО»  →  выберите LOCK или RUN"
        )
        self.operator_instruction.setObjectName("operatorInstruction")
        self.operator_instruction.setAlignment(Qt.AlignCenter)
        self.operator_instruction.setWordWrap(True)
        self.operator_instruction.setMaximumHeight(40)
        footer = QHBoxLayout()
        footer.setSpacing(4)
        footer.addWidget(self.action_timing_status, 1)
        footer.addWidget(self.operator_instruction, 2)
        layout.addLayout(footer)

        self.exhibition_voice_status = QLabel("● Голос: проверяется")
        self.exhibition_voice_status.setObjectName("simpleStatus")
        self.exhibition_voice_status.setMinimumHeight(24)
        self.exhibition_voice_status.setParent(tab)
        self.exhibition_voice_status.setVisible(False)

        self.exhibition_tab_hint = self.controller_action_hint.text()
        self.exhibition_tab = tab
        self.tabs.addTab(tab, "Главный экран")
        self._refresh_exhibition_status()

    def _connect_controller(self) -> None:
        self.controller.output.connect(self._on_output)
        self.controller.started.connect(self._on_started)
        self.controller.finished.connect(self._on_finished)
        self.controller.failed.connect(
            self._on_failed
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
        self.summary_health = self._card("Realtime health", "Проверка ещё не запускалась")
        cards.addWidget(self.summary_project, 0, 0)
        cards.addWidget(self.summary_network, 0, 1)
        cards.addWidget(self.summary_video, 1, 0)
        cards.addWidget(self.summary_mode, 1, 1)
        cards.addWidget(self.summary_processes, 2, 0, 1, 2)
        cards.addWidget(self.summary_health, 3, 0, 1, 2)
        layout.addLayout(cards)
        explanation = QLabel(
            "Начните с «Проверить всё». Команды движения требуют отдельного "
            "чек-листа и остаются под защитой safety gates проекта."
        )
        explanation.setWordWrap(True)
        explanation.setObjectName("hint")
        layout.addWidget(explanation)
        layout.addStretch(1)
        self.service_tabs.addTab(tab, "Общий статус")

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
            ("writer_check", "Проверить writer/feedback"),
            ("voice_check", "Проверить голос Добрыни"),
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
        self.service_tabs.addTab(tab, "Сеть и подключения")

    def _add_robot_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox("Сервис / аварийное управление")
        grid = QGridLayout(group)
        for index, (key, text) in enumerate(
            [
                ("prepare", "Подготовить робота"),
                ("stand", "Включить stand/balance"),
                ("exhibition_stop", "STOP — остановить сеанс"),
                ("kill", "Аварийная остановка"),
                ("reset_kill", "Снять аварийную остановку"),
            ]
        ):
            button = self._action_button(key, text)
            if key == "kill":
                button.setObjectName("killButton")
            if key in ("stop", "exhibition_stop"):
                button.setObjectName("stopButton")
            grid.addWidget(button, index // 2, index % 2)
        layout.addWidget(group)
        self.service_zero_torque_button = QPushButton(
            "ZERO TORQUE / РАССЛАБИТЬ — завершить процессы и снять удержание"
        )
        self.service_zero_torque_button.setObjectName("zeroTorqueServiceButton")
        self.service_zero_torque_button.setMinimumHeight(48)
        self.service_zero_torque_button.clicked.connect(self.request_zero_torque)
        layout.addWidget(self.service_zero_torque_button)
        note = QLabel(
            "B на правом VR-контроллере и кнопка «Аварийная остановка» немедленно "
            "обнуляют ходьбу и фиксируют KILL. Для возврата используйте «Снять "
            "аварийную остановку» или RUN на главном экране: обе кнопки повторяют "
            "полную проверку и robot-prepare."
        )
        note.setWordWrap(True)
        note.setObjectName("hint")
        layout.addWidget(note)
        layout.addStretch(1)
        self.service_tabs.addTab(tab, "Аварийное")

    def _add_vr_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox("VR и очки")
        grid = QGridLayout(group)
        for index, (key, text) in enumerate(
            [
                ("head_calibrate", "Калибровать голову"),
                ("dry_run", "Запустить VR dry-run"),
                ("vr_check", "Проверить VR-шлем"),
            ]
        ):
            grid.addWidget(self._action_button(key, text), index // 2, index % 2)
        layout.addWidget(group)
        hint = QLabel(
            "Сначала запустите arm VR-сеанс, отпустите Deadman и держите HMD и "
            "оба контроллера в нейтрали. Кнопка сохранит эту позу без команд роботу."
        )
        hint.setWordWrap(True)
        hint.setObjectName("hint")
        layout.addWidget(hint)
        layout.addStretch(1)
        self.service_tabs.addTab(tab, "VR и очки")

    def _add_voice_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox("Голосовой ассистент «Добрыня»")
        grid = QGridLayout(group)
        self.voice_status = QLabel(
            "ASR: локальный Vosk\n"
            "LLM: локальная Ollama\n"
            "TTS: Unitree RPC / мужской голос\n"
            "Wake-name: Добрыня"
        )
        self.voice_status.setObjectName("cardText")
        self.voice_status.setWordWrap(True)
        grid.addWidget(self.voice_status, 0, 0, 1, 2)
        grid.addWidget(self._action_button("voice_check", "Проверить ASR / LLM / TTS"), 1, 0)
        layout.addWidget(group)
        hint = QLabel(
            "Проверка read-only: микрофон не записывается, команды роботу не "
            "создаются. Wake-name gate и краткие ответы описаны в "
            "docs/r1_voice_offline.md."
        )
        hint.setWordWrap(True)
        hint.setObjectName("hint")
        layout.addWidget(hint)
        layout.addStretch(1)
        self.service_tabs.addTab(tab, "Голос")

    def _add_command_capture_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        group = QGroupBox("Запись кнопок Unitree Explore")
        form = QFormLayout(group)
        self.capture_action = QComboBox()
        self.capture_action.setEditable(True)
        self.capture_action.addItems(
            [
                "Run",
                "Lock",
                "Damping",
                "Zero Torque",
                "Handshake",
                "High Five",
                "Hug",
                "High Wave",
                "Clap",
                "Face Wave",
            ]
        )
        self.capture_duration = QSpinBox()
        self.capture_duration.setRange(4, 30)
        self.capture_duration.setValue(12)
        self.capture_duration.setSuffix(" с")
        form.addRow("Какую кнопку записываем", self.capture_action)
        form.addRow("Окно записи", self.capture_duration)
        buttons = QHBoxLayout()
        self.capture_start_button = QPushButton("● Начать запись")
        self.capture_start_button.setObjectName("captureButton")
        self.capture_start_button.clicked.connect(self.start_command_capture)
        buttons.addWidget(self.capture_start_button)
        self.capture_stop_button = QPushButton("Остановить запись")
        self.capture_stop_button.clicked.connect(
            lambda: self.controller.stop("command_capture")
        )
        buttons.addWidget(self.capture_stop_button)
        form.addRow(buttons)
        self.capture_status = QLabel(
            "Выберите название, начните запись и один раз нажмите соответствующую "
            "кнопку в Unitree Explore. Панель только слушает DDS и ничего не отправляет."
        )
        self.capture_status.setWordWrap(True)
        self.capture_status.setObjectName("cardText")
        form.addRow(self.capture_status)
        layout.addWidget(group)
        layout.addStretch(1)
        self.service_tabs.addTab(tab, "Запись кнопок")

    def start_command_capture(self) -> None:
        label = self.capture_action.currentText().strip()
        if not label:
            QMessageBox.warning(self, "Нет названия", "Укажите название кнопки.")
            return
        if self.controller.is_running("command_capture"):
            self.capture_status.setText("Запись уже идёт — нажмите кнопку в Unitree Explore.")
            return
        duration = self.capture_duration.value()
        self.last_capture_path = ""
        command = (
            "exec ./scripts/r1-app-command-capture --label "
            f"{shlex.quote(label)} --duration-sec {duration}"
        )
        self.capture_status.setText(
            f"Запись «{label}» началась. Нажмите эту кнопку в Unitree Explore один раз."
        )
        self.controller.start(
            "command_capture",
            self.specs["command_capture"],
            shell_command=command,
        )

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
        self.service_tabs.addTab(tab, "Видео глазами робота")

    def _add_motion_tab(self) -> None:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        arms = QGroupBox("Руки")
        arms_grid = QGridLayout(arms)
        arms_grid.addWidget(
            self._action_button("arms_live", "Повторять движения рук из VR"), 0, 0
        )
        arms_grid.addWidget(
            self._action_button("head_calibrate", "Калибровать голову (смотреть прямо)"),
            1,
            0,
        )
        arms_grid.addWidget(
            self._action_button("prepare", "Подготовить руки (Deadman зажат)"),
            2,
            0,
        )
        arms_grid.addWidget(
            self._action_button("arms_running_live", "Руки + Running (без стиков)"),
            3,
            0,
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
            "slow/suspended; KILL остаётся доступен сверху. На текущей прошивке "
            "переход в Running / FSM 811 сам поворачивает голову вправо даже при "
            "выключенном head-control. Не пытайтесь исправлять это калибровкой "
            "HMD: физическое управление головой пока заблокировано."
        )
        warning.setWordWrap(True)
        warning.setObjectName("warning")
        layout.addWidget(warning)
        layout.addStretch(1)
        self.service_tabs.addTab(tab, "Руки / ноги / teleop")

    def _add_logs_tab(self) -> None:
        tab = QWidget()
        self.logs_tab = tab
        layout = QVBoxLayout(tab)
        toolbar = QHBoxLayout()
        show_latest = QPushButton("Показать последние логи")
        show_latest.clicked.connect(self._show_logs)
        toolbar.addWidget(show_latest)
        clear = QPushButton("Очистить экран логов")
        clear.clicked.connect(lambda: self.log_view.clear())
        toolbar.addWidget(clear)
        open_logs = QPushButton("Открыть папку логов")
        open_logs.clicked.connect(self.open_logs)
        toolbar.addWidget(open_logs)
        export = QPushButton("Экспортировать отчёт")
        export.clicked.connect(self.export_report)
        toolbar.addWidget(export)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QPlainTextEdit.NoWrap)
        # A POV/ROS process can run for hours; keep the panel responsive while
        # retaining enough recent output for diagnosis.
        self.log_view.setMaximumBlockCount(5000)
        self.log_view.setFont(QFont("Monospace", 9))
        layout.addWidget(self.log_view)
        self.service_tabs.addTab(tab, "Логи")

    def _show_logs(self) -> None:
        self.tabs.setCurrentWidget(self.service_tabs)
        self.service_tabs.setCurrentWidget(self.logs_tab)

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

    @staticmethod
    def _set_simple_status(label: QLabel, text: str, state: str) -> None:
        colors = {
            "ok": ("#173523", "#30d158"),
            "wait": ("#3a2d0e", "#ff9f0a"),
            "off": ("#3a2022", "#ff6961"),
        }
        background, foreground = colors[state]
        label.setText(text)
        label.setStyleSheet(
            "padding: 4px 8px; border-radius: 5px; "
            f"background: {background}; color: {foreground}; font-weight: bold;"
        )

    def _begin_action(self, key: str) -> bool:
        """Start one debounced click-to-result measurement."""
        started = self.__dict__.setdefault("action_started_at", {})
        if key in started:
            if "log_view" in self.__dict__:
                self._append_log(
                    key,
                    "[INFO] Действие уже выполняется; повторный экземпляр не создан.\n",
                )
            return False
        if key in {
            "exhibition_static",
            "exhibition_control",
            "exhibition_stand",
        }:
            for previous in (
                "exhibition_static",
                "exhibition_control",
                "exhibition_stand",
            ):
                if previous != key and previous in started:
                    self._finish_action(
                        previous, "отменено", "оператор выбрал другой режим"
                    )
        started[key] = time.monotonic()
        generations = self.__dict__.setdefault("action_generations", {})
        generation = int(generations.get(key, 0)) + 1
        generations[key] = generation
        title = MAIN_ACTION_TITLES.get(key, key)
        timeout = MAIN_ACTION_TIMEOUTS_SEC.get(key, 30.0)
        if "action_timing_status" in self.__dict__:
            self.action_timing_status.setText(
                f"Выполняется: {title} • тайм-аут {timeout:.0f} с"
            )
            self.action_timing_status.setStyleSheet(
                "padding: 4px 8px; border-radius: 5px; "
                "background: #3a2d0e; color: #ff9f0a; font-weight: bold;"
            )
        if "status_timer" in self.__dict__:
            QTimer.singleShot(
                int(timeout * 1000),
                lambda k=key, g=generation: self._action_timed_out(k, g),
            )
        return True

    def _finish_action(self, key: str, result: str, detail: str = "") -> Optional[float]:
        """Finish an active measurement and expose a compact audit line."""
        started = self.__dict__.setdefault("action_started_at", {})
        started_at = started.pop(key, None)
        if started_at is None:
            return None
        elapsed = max(0.0, time.monotonic() - started_at)
        title = MAIN_ACTION_TITLES.get(key, key)
        normalized = result.strip().lower()
        ok = normalized in {"готово", "ok", "успешно", "уже готово"}
        record = {
            "elapsed_sec": round(elapsed, 3),
            "result": result,
            "detail": detail,
        }
        self.__dict__.setdefault("last_action_timings", {})[key] = record
        suffix = f" • {detail}" if detail else ""
        if "action_timing_status" in self.__dict__:
            self.action_timing_status.setText(
                f"{title}: {result} за {elapsed:.2f} с{suffix}"
            )
            colors = (
                ("#173523", "#30d158")
                if ok
                else ("#3a2022", "#ff6961")
            )
            self.action_timing_status.setStyleSheet(
                "padding: 4px 8px; border-radius: 5px; font-weight: bold; "
                f"background: {colors[0]}; color: {colors[1]};"
            )
        if "log_view" in self.__dict__:
            level = "OK" if ok else "BLOCKED"
            self._append_log(
                key,
                f"[{level}] BUTTON_TIMING result={result} "
                f"elapsed_sec={elapsed:.3f} detail={detail or '-'}\n",
            )
        return elapsed

    def _action_timed_out(self, key: str, generation: int) -> None:
        if self.__dict__.setdefault("action_generations", {}).get(key) != generation:
            return
        if key not in self.__dict__.setdefault("action_started_at", {}):
            return
        title = MAIN_ACTION_TITLES.get(key, key)
        self._finish_action(
            key,
            "тайм-аут",
            "операция не подтверждена; процесс не был принудительно остановлен",
        )
        if "operator_instruction" in self.__dict__:
            self.operator_instruction.setText(
                f"Нет подтверждения «{title}». Проверьте причину в логах и "
                "нажмите эту же кнопку ещё раз."
            )

    def _request_status_refresh(self, *, fast: bool = False) -> bool:
        spec = self.specs.get("panel_status")
        process_key = "panel_status_fast" if fast else "panel_status"
        if not spec or self.controller.is_running(process_key):
            return False
        return self.controller.start(
            process_key,
            spec,
            env_overrides={"R1_PANEL_STATUS_FAST": "1"} if fast else None,
        )

    @staticmethod
    def _action_for_process(key: str) -> Optional[str]:
        mapping = {
            "exhibition_lock": "exhibition_static",
            "exhibition_rearm": "exhibition_control",
            "exhibition_service:exhibition_reconnect": "exhibition_reconnect",
            "zero_torque": "zero_torque",
        }
        return mapping.get(key)

    def _show_retry_reason(self, action_key: Optional[str], reason: str) -> None:
        if not action_key or not reason:
            return
        title = MAIN_ACTION_TITLES.get(action_key, action_key)
        clean = reason.strip().replace("\n", " ")[:280]
        if "operator_instruction" in self.__dict__:
            self.operator_instruction.setText(
                f"Ошибка «{title}»: {clean}. После устранения причины нажмите "
                "эту же кнопку ещё раз."
            )

    def _on_failed(self, key: str, message: str) -> None:
        self._append_log(key, f"[FAIL] {message}\n")
        action = self._action_for_process(key)
        if key == "exhibition":
            action = self.__dict__.get("exhibition_requested_mode")
        self._show_retry_reason(action, message)
        if action:
            self._finish_action(action, "ошибка", message)

    def start_video_preview(self) -> None:
        if "video_preview" in self.__dict__:
            self.video_preview.start()

    def start_background_services(self) -> None:
        """Start read-only camera, battery and voice discovery."""
        self.background_services_started = True
        self.start_video_preview()
        if (
            self.config.auto_start_bridge
            and not self.controller.is_running("connection_ensure")
        ):
            self.controller.start(
                "connection_ensure", self.specs["connection_ensure"]
            )
        elif (
            self.config.auto_start_video
            and not self.config.auto_start_bridge
            and not self.controller.is_running("video_robot")
        ):
            # A video-only deployment may opt out of the bridge. Start just
            # the read-only POV process in that case; RUN remains manual.
            self.controller.start("video_robot", self.specs["video_robot"])
        if not self.controller.is_running("battery_monitor"):
            self.controller.start(
                "battery_monitor", self.specs["battery_monitor"]
            )
        if (
            not self.controller.is_running("sdk_warmup")
            and not self.__dict__.get("pending_warmup_key")
            and not self.__dict__.get("warmup_restart_pending", False)
        ):
            self.sdk_warmup_cache_available = False
            self.controller.start("sdk_warmup", self.specs["sdk_warmup"])
        if not self.controller.is_running("voice:voice_remote"):
            self.controller.start(
                "voice:voice_remote", self.specs["voice_remote"]
            )

    def auto_connect(self, _checked: bool = False, *, track_action: bool = True) -> None:
        if track_action and not self._begin_action("connect"):
            self._request_status_refresh(fast=True)
            return
        self.zero_torque_idle = False
        if "status_timer" in self.__dict__ and not self.status_timer.isActive():
            self.status_timer.start()
        already_connected = (
            self.connection_state == "connected"
            and self.status_values.get("robot", "").upper()
            in {"OK", "FOUND", "RUNNING", "READY"}
        )
        self.connection_state = "connected" if already_connected else "searching"
        headset_ip = _discover_headset_ip() if self.config.vr_transport == "lan" else None
        if headset_ip and headset_ip != self.config.vr_headset_ip:
            # DHCP changes at exhibitions are normal. Keep the discovered
            # address in memory for this session; the next launch discovers it
            # again instead of pinning a stale address in the config file.
            self.config.vr_headset_ip = headset_ip
            self.controller.config = self.config
            self._append_log(
                "connection_ensure",
                f"[OK] VR-шлем найден автоматически: {headset_ip}\n",
            )
        if "robot_name_label" in self.__dict__:
            if not already_connected:
                self.robot_name_label.setText(
                    f"◌  {self.config.robot_name}  — подключение…"
                )
                self.robot_name_label.setProperty("connectionState", "searching")
                self.connect_button.setText("ПОИСК И ПОДКЛЮЧЕНИЕ…")
            # Keep the control clickable. Repeated clicks are debounced in
            # _begin_action and request only a fresh read-only status snapshot.
            self.connect_button.setEnabled(True)
        self.start_background_services()
        self._request_status_refresh(fast=True)
        if already_connected:
            self.last_status = "Зелёный • существующее Ethernet-соединение используется"
            self._finish_action(
                "connect", "уже готово", "повторный полный поиск не запускался"
            )
        else:
            self.last_status = "Жёлтый • локальное подключение проверяется"
        self._refresh_summary()

    def _run_main_reconnect(self) -> None:
        """Reconnect the complete operator path with one button.

        A reconnect never changes the robot mode. With an active manager it
        repairs only failed managed components. Without one it refreshes the
        existing read-only Ethernet/USB/video services.
        """
        if not self._begin_action("exhibition_reconnect"):
            return
        self._ensure_main_services()
        existing = active_session_snapshot()
        if self.controller.is_running("exhibition") or existing.get("mode") in {
            "static", "control"
        }:
            process_key = "exhibition_service:exhibition_reconnect"
            if not self.controller.is_running(process_key):
                started = self.controller.start(
                    process_key, self.specs["exhibition_reconnect"]
                )
                if not started:
                    self._finish_action(
                        "exhibition_reconnect", "ошибка", "не удалось запустить helper"
                    )
            return
        self._append_log(
            "exhibition_reconnect",
            "[OK] Активного управляющего сеанса нет; режим робота не меняю. "
            "Обновляю только локальные статусы Ethernet, USB и видео.\n",
        )
        self._request_status_refresh(fast=True)

    def _run_main_calibration(self) -> None:
        """Retained service path; intentionally absent from the main screen."""
        existing = active_session_snapshot()
        control_active = (
            self.controller.is_running("exhibition")
            and self.exhibition_requested_mode == "exhibition_control"
        ) or (
            existing.get("mode") == "control"
            and existing.get("status") in {"ready", "locked", "degraded"}
        )
        if not control_active:
            self._append_log(
                "exhibition_calibrate",
                "[BLOCKED] Калибровка доступна только в уже запущенном control-сеансе; "
                "автоматический RUN не выполнялся.\n",
            )
            return
        process_key = "exhibition_service:exhibition_calibrate"
        if not self.controller.is_running(process_key):
            self.controller.start(
                process_key, self.specs["exhibition_calibrate"]
            )

    def _ensure_main_services(self) -> None:
        """Make every main-screen action self-starting after panel launch."""
        # Lightweight __new__ instances used by unit tests do not have a Qt
        # timer yet; those callers intentionally exercise only the mode logic.
        if "status_timer" not in self.__dict__:
            return
        if not self.status_timer.isActive():
            self.status_timer.start()
        if self.connection_state == "offline":
            self.auto_connect(track_action=False)
        elif not self.background_services_started:
            self.start_background_services()

    def toggle_window_maximized(self) -> None:
        """Toggle the dashboard between a normal window and a maximized window."""
        if self.isMaximized():
            self.showNormal()
            if self._normal_geometry is not None:
                self.setGeometry(self._normal_geometry)
        else:
            self._normal_geometry = self.geometry()
            self.showMaximized()
        self._update_fullscreen_button()

    def toggle_fullscreen(self) -> None:
        """Toggle the operator dashboard between windowed and full-screen modes."""
        if self.isFullScreen():
            self.showNormal()
            if self._normal_geometry is not None:
                self.setGeometry(self._normal_geometry)
        else:
            self._normal_geometry = self.geometry()
            self.showFullScreen()
        self._update_fullscreen_button()

    def exit_fullscreen(self) -> None:
        """Leave full-screen mode without affecting any robot process."""
        if self.isFullScreen():
            self.toggle_fullscreen()

    def resizeEvent(self, event) -> None:  # noqa: N802 - Qt virtual method
        """Keep the header readable while the dashboard changes width."""
        super().resizeEvent(event)
        self._refresh_header_controls()

    def _refresh_header_controls(self) -> None:
        """Use icon-sized header actions when a shorter window is narrow."""
        if not {
            "service_toggle",
            "help_button",
            "settings_button",
        }.issubset(self.__dict__):
            return
        compact = self.width() < 1450
        service_visible = bool(
            "tabs" in self.__dict__ and self.tabs.currentIndex() == 1
        )
        if compact:
            self.service_toggle.setText("←" if service_visible else "☰")
            self.help_button.setText("?")
            self.settings_button.setText("⚙")
        else:
            self.service_toggle.setText(
                "←  Главный экран"
                if service_visible
                else "⚙  Расширенные настройки"
            )
            self.help_button.setText("?  Как запустить")
            self.settings_button.setText("⚙  Настройки соединения")
        for button in (
            self.service_toggle,
            self.help_button,
            self.settings_button,
        ):
            button.updateGeometry()
        if "header_actions" in self.__dict__:
            self.header_actions.updateGeometry()
            if self.header_actions.layout() is not None:
                self.header_actions.layout().invalidate()
        if "header_card" in self.__dict__:
            self.header_card.updateGeometry()
            if self.header_card.layout() is not None:
                self.header_card.layout().invalidate()
                self.header_card.layout().activate()

    def _update_fullscreen_button(self) -> None:
        if "fullscreen_button" not in self.__dict__:
            return
        expanded = self.isFullScreen() or self.isMaximized()
        self.fullscreen_button.setText("❐" if expanded else "□")
        self.fullscreen_button.setAccessibleName(
            "Вернуть обычный размер" if expanded else "Развернуть окно"
        )
        self.fullscreen_button.setToolTip(
            "Вернуть обычный размер окна"
            if expanded
            else "Развернуть окно на весь рабочий стол"
        )

    def toggle_service_view(self) -> None:
        """Keep the normal operator on one screen; expose service tools on demand."""
        service_visible = self.tabs.currentIndex() == 1
        self.tabs.setCurrentIndex(0 if service_visible else 1)
        self._refresh_header_controls()

    def show_quick_start(self) -> None:
        QMessageBox.information(
            self,
            "Как запустить Unitree R1",
            "1. Поставьте робота устойчиво и освободите место вокруг него.\n"
            "2. Включите робота и VR-шлем с контроллерами.\n"
            "3. Дождитесь зелёного «ПОДКЛЮЧЕНО», изображения с камеры и "
            "зелёных статусов VR.\n"
            "4. Для показа видео нажмите «LOCK / СТАТИЧНЫЙ РЕЖИМ».\n"
            "5. Для головы, рук и ходьбы закройте экран управления Unitree "
            "Explore и нажмите «RUN / ПОЛНОЕ УПРАВЛЕНИЕ». Панель сама "
            "переведёт робота в Run.\n"
            "6. X слева возвращает руки в нейтраль; B справа выполняет "
            "аварийную остановку.\n\n"
            "Первый RUN подготавливает управление. Для обычной паузы используйте "
            "LOCK, затем RUN с отпущенными стиками: полный перезапуск не нужен. "
            "STOP, KILL и Zero Torque находятся в разделе «Аварийное».",
        )

    def _on_video_state(self, text: str, state: str) -> None:
        self.video_state = state
        if state == "online":
            self.status_values["video"] = "OK"
            self._finish_action(
                "viewer", "готово", "локальный видеопоток подтверждён"
            )
        elif state in {"searching", "offline"}:
            self.status_values["video"] = "RECONNECTING"
        self._refresh_exhibition_status()

    def _refresh_exhibition_status(self) -> None:
        """Render the compact, Russian exhibition status from read-only data."""
        # Several focused unit tests construct the Python side without calling
        # QMainWindow.__init__; querying Qt attributes with hasattr() then
        # raises from SIP, so inspect the plain instance dictionary instead.
        if "exhibition_robot_status" not in self.__dict__:
            return
        robot = self.status_values.get("robot", "UNKNOWN").upper()
        pc2 = self.status_values.get("pc2", "UNKNOWN").upper()
        vr = self.status_values.get("vr", "UNKNOWN").upper()
        controllers = self.status_values.get("controllers", "").upper()
        deadman = self.status_values.get("deadman", "UNKNOWN").upper()
        video = self.status_values.get("video", "UNKNOWN").upper()
        voice = self.status_values.get("voice", "UNKNOWN").upper()

        if robot in {"OK", "FOUND", "RUNNING", "READY"}:
            self._set_simple_status(self.exhibition_robot_status, "● Робот найден", "ok")
            self.connection_state = "connected"
            if "robot_name_label" in self.__dict__:
                self.robot_name_label.setText(
                    f"●  {self.config.robot_name}    ETHERNET"
                )
                self.robot_name_label.setProperty("connectionState", "connected")
                self.connect_button.setText("✓  ПОДКЛЮЧЕНО")
                self.connect_button.setEnabled(True)
                self.operator_instruction.setText(
                    "Робот подключён. Проверьте видео и VR ниже, затем выберите режим."
                )
        elif pc2 in {"OK", "FOUND", "RUNNING", "READY"}:
            self._set_simple_status(
                self.exhibition_robot_status,
                "◐ PC2 по кабелю; управление не отвечает",
                "wait",
            )
            self.connection_state = "searching"
            if "robot_name_label" in self.__dict__:
                self.robot_name_label.setText(
                    f"◐  {self.config.robot_name}  — PC2 по кабелю"
                )
                self.robot_name_label.setProperty("connectionState", "searching")
                self.connect_button.setText("…  УПРАВЛЕНИЕ НЕ В СЕТИ")
                self.connect_button.setEnabled(True)
                self.operator_instruction.setText(
                    "Кабель и PC2 работают, но управляющий модуль "
                    "192.168.123.161 не отвечает. Run и камера появятся "
                    "автоматически после его загрузки."
                )
        else:
            self._set_simple_status(
                self.exhibition_robot_status, "○ Робот не найден", "off"
            )
            if self.connection_state != "searching":
                self.connection_state = "offline"
                if "robot_name_label" in self.__dict__:
                    self.robot_name_label.setText(
                        f"○  {self.config.robot_name}  — не в сети"
                    )
                    self.robot_name_label.setProperty("connectionState", "offline")
                    self.connect_button.setText("↻  НАЙТИ И ПОДКЛЮЧИТЬ")
                    self.connect_button.setEnabled(True)
                    self.operator_instruction.setText(
                        "Включите робота и проверьте кабель TP-Link. "
                        "Поиск продолжится автоматически."
                    )
        if "robot_name_label" in self.__dict__:
            self.robot_name_label.style().unpolish(self.robot_name_label)
            self.robot_name_label.style().polish(self.robot_name_label)

        if vr in {"OK", "FOUND", "RUNNING", "READY"}:
            self._set_simple_status(self.exhibition_vr_status, "● Очки найдены", "ok")
        elif vr in {"RECOVERING", "RECONNECTING", "STALE"}:
            self._set_simple_status(self.exhibition_vr_status, "… Очки переподключаются", "wait")
        else:
            self._set_simple_status(self.exhibition_vr_status, "○ Очки не найдены", "off")

        if not controllers:
            if vr in {"OK", "FOUND", "RUNNING", "READY"} and deadman != "UNKNOWN":
                controllers = "OK"
            elif vr in {"OK", "FOUND", "RUNNING", "READY"}:
                controllers = "RECOVERING"
            else:
                controllers = "OFFLINE"
        if controllers in {"OK", "FOUND", "RUNNING", "READY"}:
            self._set_simple_status(
                self.exhibition_controllers_status, "● Контроллеры найдены", "ok"
            )
        elif controllers in {"RECOVERING", "RECONNECTING", "STALE", "HOLD"}:
            self._set_simple_status(
                self.exhibition_controllers_status,
                "… Контроллеры восстанавливаются",
                "wait",
            )
        else:
            self._set_simple_status(
                self.exhibition_controllers_status, "○ Контроллеры не найдены", "off"
            )

        if video in {"OK", "RUNNING", "READY"}:
            self._set_simple_status(self.exhibition_video_status, "● Видео работает", "ok")
        elif video in {"RECOVERING", "RECONNECTING", "STARTING"}:
            self._set_simple_status(self.exhibition_video_status, "… Видео запускается", "wait")
        else:
            self._set_simple_status(self.exhibition_video_status, "○ Нет видео", "off")

        if "exhibition_voice_status" in self.__dict__:
            if voice in {"OK", "RUNNING", "READY", "ACTIVE"}:
                self._set_simple_status(
                    self.exhibition_voice_status, "● Голос Добрыня работает", "ok"
                )
            elif voice in {"STARTING", "RECONNECTING", "UNKNOWN"}:
                self._set_simple_status(
                    self.exhibition_voice_status, "… Голос подключается", "wait"
                )
            else:
                self._set_simple_status(
                    self.exhibition_voice_status, "○ Голос не подключён", "off"
                )

        reported_mode = self.status_values.get("mode", "").strip().lower()
        kill = self.status_values.get("kill", "").strip().upper()
        emergency = self.status_values.get("emergency", "").strip().upper()
        mode_names = {
            "static": "Статичный",
            "control": "Управление",
            "stopped": "Остановлен",
            "starting": "Запускается",
        }
        mode = mode_names.get(reported_mode, self.exhibition_mode)
        if (
            reported_mode == "static"
            and self.__dict__.get("exhibition_requested_mode") == "exhibition_stand"
        ):
            mode = "Стойка"
        mode_state = "ok" if mode in {"Статичный", "Управление"} else "wait"
        if mode == "Стойка":
            mode_state = "ok"
        if reported_mode == "control":
            sequential_names = {
                "arms": "Управление руками и головой",
                "releasing_arms": "Передача управления для ходьбы…",
                "waiting_811": "Ожидание режима ходьбы…",
                "walking": "Ходьба — руки и голова под управлением робота",
                "stopping": "Остановка перед возвратом VR…",
                "fault": "Ошибка переключения режима",
            }
            mode = sequential_names.get(
                self.status_values.get("sequential_phase", ""), mode
            )
        manager_status = self.status_values.get("exhibition", "")
        if manager_status and manager_status not in {"ready", "locked", "stopped"}:
            mode = {
                "degraded": "Нужно восстановление — нажмите RUN после проверки связи",
                "blocked": "Запуск заблокирован — см. причину в логах",
                "reconnecting": "Восстановление связи…",
                "resuming": "Возврат управления…",
                "rearming": "Подготовка после остановки…",
            }.get(manager_status, "Подготовка — жду подтверждения робота…")
            mode_state = "wait"
        if emergency == "RIGHT_B":
            mode = "Аварийная остановка нажата на правом контроллере B"
            mode_state = "off"
            if "operator_instruction" in self.__dict__:
                self.operator_instruction.setText(mode)
        elif kill == "LATCHED" and (
            reported_mode == "control"
            or manager_status
            in {
                "ready",
                "locked",
                "degraded",
                "reconnecting",
                "resuming",
                "rearming",
            }
        ):
            mode = "Управление остановлено (KILL)"
            mode_state = "off"
        if mode == "Остановлен":
            mode_state = "off"
        self._set_simple_status(
            self.exhibition_mode_status, f"Режим: {mode}", mode_state
        )

    def _refresh_summary(self) -> None:
        self.mode_label.setText("РЕЖИМ: " + self.exhibition_mode.upper())
        self.status_label.setText(self.last_status)
        if self.last_status.startswith("Зелёный"):
            background, foreground, border = "#173523", "#30d158", "#245d39"
        elif self.last_status.startswith("Жёлтый"):
            background, foreground, border = "#3a2d0e", "#ff9f0a", "#654d16"
        elif self.last_status.startswith("Красный"):
            background, foreground, border = "#3a2022", "#ff6961", "#6b3438"
        else:
            background, foreground, border = "#232326", "#aeb0b8", "#3a3a3c"
        self.status_label.setStyleSheet(
            "padding: 6px 10px; border-radius: 8px; "
            f"background: {background}; color: {foreground}; "
            f"border: 1px solid {border};"
        )
        self.active_label.setText(
            f"Активных процессов: {len(self.controller.active_keys())}"
        )
        active = self.controller.active_keys()
        self.summary_processes._value_label.setText(  # type: ignore[attr-defined]
            ", ".join(active) if active else "Нет активных процессов"
        )
        self.summary_mode._value_label.setText(self._mode_text())  # type: ignore[attr-defined]
        if self.status_values:
            labels = {
                "ethernet": "Ethernet", "robot": "робот", "vr": "VR",
                "hmd": "HMD", "hands": "руки", "sticks": "стики",
                "deadman": "deadman", "kill": "kill", "writer": "writer",
                "feedback": "feedback", "video": "video",
            }
            health = "  |  ".join(
                f"{labels.get(key, key)}: {value}"
                for key, value in self.status_values.items()
                if key != "project"
            )
            self.summary_health._value_label.setText(  # type: ignore[attr-defined]
                f"{health}\nROS_DOMAIN_ID="
                f"{self.status_values.get('ros_domain', self.config.ros_domain_id)}"
            )
        self._refresh_exhibition_status()

    def run_key(self, key: str) -> None:
        if key == "reset_kill":
            # This is intentionally the same reviewed path as an explicit RUN;
            # it never exposes the writer's low-level reset service directly.
            self._run_exhibition_mode("exhibition_control")
            return
        spec = self.specs[key]
        if not spec.implemented:
            QMessageBox.information(self, spec.title, spec.note)
            return
        if key in ("stop", "kill", "exhibition_stop"):
            # A human STOP/KILL always cancels an already queued mode switch.
            # Otherwise the stop helper could finish and immediately launch
            # the pending live mode behind the operator's back.
            self.pending_exhibition_key = None
            self.pending_exhibition_environment = None
            self.pending_warmup_key = None
            self.pending_warmup_environment = None
            self.warmup_handoff_waiting = False
            self.warmup_handoff_generation = int(
                self.__dict__.get("warmup_handoff_generation", 0)
            ) + 1
            self.warmup_restart_pending = False
            self.exhibition_switch_in_progress = False
            self.exhibition_stop_complete = False
            self.exhibition_manager_stopped = False
            self.pending_zero_torque = False
            self._clear_live_confirmation()
        if key == "check_all":
            self._run_check_all()
            return
        if key in ("exhibition_static", "exhibition_control", "exhibition_stand"):
            self._run_exhibition_mode(key)
            return
        if key == "exhibition_reconnect":
            self._run_main_reconnect()
            return
        if key == "exhibition_calibrate":
            self._run_main_calibration()
            return
        live_environment = None
        if spec.live:
            if not self._confirm_live(
                spec, require_run_mode=spec.requires_run_mode
            ):
                return
            live_environment = self._live_environment(
                run_mode_confirmed=spec.requires_run_mode
            )
        if key == "exhibition_stop":
            process_key = "exhibition_stop"
        elif spec.category == "video":
            process_key = "video"
        elif spec.category in ("arms", "legs", "full", "vr"):
            process_key = "motion"
        elif key in ("prepare", "stand"):
            process_key = "robot:prepare"
        else:
            process_key = spec.category + ":" + key
        if spec.category == "robot" and key in ("stop", "kill"):
            process_key = key
        self.controller.start(
            process_key,
            spec,
            env_overrides=live_environment,
        )

    def request_zero_torque(self) -> None:
        """Stop every physical owner, then request confirmed Damping -> FSM 0."""
        dialog = QMessageBox(self)
        dialog.setIcon(QMessageBox.Warning)
        dialog.setWindowTitle("Zero Torque — робот потеряет удержание")
        dialog.setText(
            "После Zero Torque суставы перестанут удерживать робота. "
            "Он может резко сложиться или упасть."
        )
        dialog.setInformativeText(
            "Продолжайте только если робот лежит или надёжно поддержан. "
            "Панель сначала завершит VR и управляющие процессы, затем "
            "подтвердит Damping и включит Zero Torque."
        )
        relax = dialog.addButton("РАССЛАБИТЬ РОБОТА", QMessageBox.DestructiveRole)
        cancel = dialog.addButton("Отмена", QMessageBox.RejectRole)
        dialog.setDefaultButton(cancel)
        dialog.exec_()
        if dialog.clickedButton() is not relax:
            return
        if not self._begin_action("zero_torque"):
            return

        self.pending_exhibition_key = None
        self.pending_exhibition_environment = None
        self.exhibition_switch_in_progress = False
        self.exhibition_stop_complete = False
        self.exhibition_manager_stopped = False
        self.pending_zero_torque = True
        self.pending_warmup_key = None
        self.pending_warmup_environment = None
        self.warmup_restart_pending = False
        self.zero_torque_deadline = time.monotonic() + 75.0
        self.zero_torque_handoff_started = False
        self.zero_torque_handoff_complete = False
        self._clear_live_confirmation()
        self._append_log(
            "zero_torque",
            "[INFO] Сначала завершаю все физические управляющие процессы.\n",
        )

        if self.controller.is_running("exhibition"):
            if not self.controller.is_running("exhibition_stop"):
                self.controller.start(
                    "exhibition_stop", self.specs["exhibition_stop"]
                )
        elif any(
            self.controller.is_running(key)
            for key in ("motion", "robot:prepare")
        ):
            if not self.controller.is_running("stop"):
                self.controller.start("stop", self.specs["stop"])
        self._continue_zero_torque_when_idle()

    def _continue_zero_torque_when_idle(self) -> None:
        """Wait asynchronously for SDK owners to exit before FSM commands."""
        if not self.pending_zero_torque:
            return
        physical_keys = {
            "exhibition", "exhibition_stop", "motion", "robot:prepare",
            "stop", "kill",
        }
        active = set(self.controller.active_keys())
        if active.intersection(physical_keys):
            if time.monotonic() >= self.zero_torque_deadline:
                self.pending_zero_torque = False
                QMessageBox.critical(
                    self,
                    "Zero Torque не выполнен",
                    "Управляющие процессы не завершились вовремя. "
                    "Команда Zero Torque роботу не отправлялась.",
                )
                self._finish_action(
                    "zero_torque", "тайм-аут", "управляющие процессы не завершились"
                )
                return
            QTimer.singleShot(250, self._continue_zero_torque_when_idle)
            return

        # The requested end state is deliberately quiet: after physical
        # owners have released the SDK, stop every panel helper as well. The
        # operator can start discovery again with «НАЙТИ И ПОДКЛЮЧИТЬ».
        helpers = active.difference({"zero_torque", ZERO_TORQUE_HANDOFF_KEY})
        if helpers:
            self.status_timer.stop()
            if "video_preview" in self.__dict__:
                # Request the HTTP reader to stop without waiting on its
                # socket/thread from the Qt event loop.  The exact offline
                # handoff below remains the authoritative bounded gate before
                # any robot-facing Zero Torque helper may start.
                self.video_preview.stop(wait_ms=0)
            for key in helpers:
                self.controller.stop(
                    key, graceful_timeout_ms=1200, wait=False
                )
            self.background_services_started = False
            QTimer.singleShot(0, self._continue_zero_torque_when_idle)
            return

        # The offline video/bridge session is usually a user systemd unit,
        # therefore it is not necessarily present in ``active_keys()``.  Hand
        # it off explicitly and wait for its exact ports to disappear before
        # opening the SDK channel for FSM 1 -> 0.  This closes the race where
        # the next RUN inherits a stale DDS/UDP owner from the previous run.
        if not self.zero_torque_handoff_complete:
            if not self.zero_torque_handoff_started:
                self.zero_torque_handoff_started = True
                self._append_log(
                    "zero_torque",
                    "[INFO] Останавливаю offline bridge/video и жду освобождения "
                    "портов 8080/9090/9091.\n",
                )
                handoff_spec = CommandSpec(
                    ZERO_TORQUE_HANDOFF_KEY,
                    "Остановить offline bridge перед Zero Torque",
                )
                if not self.controller.start(
                    ZERO_TORQUE_HANDOFF_KEY,
                    handoff_spec,
                    shell_command=ZERO_TORQUE_HANDOFF_COMMAND,
                ):
                    self.zero_torque_handoff_started = False
                    self.pending_zero_torque = False
                    QMessageBox.critical(
                        self,
                        "Zero Torque отменён",
                        "Не удалось запустить handoff offline-сервиса. "
                        "Damping и Zero Torque роботу не отправлялись.",
                    )
                    self._finish_action(
                        "zero_torque", "ошибка", "offline handoff не запустился"
                    )
                return
            # The handoff process has either not emitted finished yet or its
            # finished callback is queued.  Do not start another copy and do
            # not open the robot SDK while it is still active.
            if self.controller.is_running(ZERO_TORQUE_HANDOFF_KEY):
                return
            # A non-zero exit is handled in _on_finished().  Reaching this
            # branch without the completion flag is therefore a fail-closed
            # guard for mocked controllers and unexpected process removal.
            self.pending_zero_torque = False
            QMessageBox.critical(
                self,
                "Zero Torque отменён",
                "Offline bridge не подтвердил освобождение портов. "
                "Damping и Zero Torque роботу не отправлялись.",
            )
            self._finish_action(
                "zero_torque", "ошибка", "offline bridge не освободил порты"
            )
            return

        self.pending_zero_torque = False
        self.zero_torque_idle = True
        environment = {
            "ROBOT_CONFIRM_ZERO_TORQUE": "1",
            "ROBOT_CONFIRM_ROBOT_SUPPORTED": "1",
            "ROBOT_ZERO_TORQUE_TOKEN": secrets.token_urlsafe(24),
        }
        if not self.controller.start(
            "zero_torque",
            self.specs["zero_torque"],
            env_overrides=environment,
        ):
            self._finish_action(
                "zero_torque", "ошибка", "Zero Torque helper не запустился"
            )

    def _run_exhibition_mode(self, key: str) -> None:
        spec = self.specs[key]
        if not self._begin_action(key):
            return
        # A click on RUN/LOCK/STAND is an explicit operator request.  If a
        # recovered-link warmup is already stopping, its finished callback
        # must continue with this physical request instead of silently
        # starting another background worker first.
        self.warmup_restart_pending = False
        if self.__dict__.get(
            "pending_zero_torque", False
        ) or self.controller.is_running("zero_torque"):
            self._append_log(
                key,
                "[INFO] Дождитесь завершения Zero Torque; затем нажмите "
                "нужный режим.\n",
            )
            self._finish_action(key, "заблокировано", "выполняется Zero Torque")
            return
        # Never run a fast rearm against the owner already being stopped.
        # Only an existing operator-requested switch may have its destination
        # replaced. An explicit STOP cannot queue or resurrect movement.
        if self.controller.is_running("exhibition_stop") or self.__dict__.get(
            "exhibition_switch_in_progress", False
        ):
            if not self.__dict__.get("pending_exhibition_key"):
                self._append_log(
                    key,
                    "[INFO] Выполняется STOP; после завершения нажмите "
                    "нужный режим.\n",
                )
                self._finish_action(key, "заблокировано", "выполняется STOP")
                return
            if not self._confirm_live(
                spec, automatic_preflight=True, require_vr=key == "exhibition_control",
                session_arm=key == "exhibition_control", require_run_mode=spec.requires_run_mode,
                skip_checklist=True,
            ):
                self._finish_action(key, "отменено", "подтверждение оператора не получено")
                return
            self.pending_exhibition_key = key
            self.pending_exhibition_environment = self._live_environment(
                run_mode_confirmed=spec.requires_run_mode)
            self._append_log(
                key,
                "[INFO] Следующий режим обновлён; жду безопасного завершения "
                "текущего.\n",
            )
            return
        self._ensure_main_services()
        existing = active_session_snapshot()
        if (
            key in {"exhibition_static", "exhibition_stand"}
            and existing.get("mode") == "static"
            and existing.get("status") == "ready"
        ):
            self.exhibition_mode = (
                "Стойка" if key == "exhibition_stand" else "Статичный"
            )
            self._refresh_summary()
            self._finish_action(
                key,
                "уже готово",
                (
                    "подтверждённая стойка уже активна"
                    if key == "exhibition_stand"
                    else "подтверждённый статичный режим уже активен"
                ),
            )
            return
        external_control = (
            existing.get("mode") == "control"
            and existing.get("session_mode") == "session_arm"
            and existing.get("status") in {"ready", "locked", "degraded"}
        )
        if (
            key != "exhibition_stand" and external_control
            and not self.controller.is_running("exhibition")
        ):
            # The CLI validates the actual owner and its acknowledgement.
            # Never start a competing graph just because QProcess did not
            # launch this session (panel reopen / terminal / service restart).
            action = (
                "exhibition_lock"
                if key == "exhibition_static"
                else "exhibition_rearm"
            )
            if self.controller.is_running(action):
                return
            self.exhibition_requested_mode = "exhibition_control"
            self.exhibition_mode = (
                "Переход в LOCK…"
                if key == "exhibition_static"
                else "Возврат управления…"
            )
            self._refresh_summary()
            if not self.controller.start(action, self.specs[action]):
                self._finish_action(key, "ошибка", "helper уже занят или не запустился")
            return
        fast_lock = (
            key == "exhibition_static"
            and self.controller.is_running("exhibition")
            and (
                self.exhibition_requested_mode == "exhibition_control"
                or self.status_values.get("mode", "").strip().lower()
                == "control"
            )
        )
        if fast_lock:
            # LOCK is a normal operating action, not a teardown.  Keep the
            # already checked SDK graph, POV and writer alive; the bridge's
            # pause service continuously holds the upper body and publishes
            # zero locomotion.  Full STOP/KILL remains in the service tab.
            if self.controller.is_running("exhibition_lock"):
                return
            self.exhibition_mode = "Переход в LOCK…"
            self._refresh_summary()
            if not self.controller.start(
                "exhibition_lock", self.specs["exhibition_lock"]
            ):
                self._finish_action(key, "ошибка", "LOCK helper не запустился")
            return
        same_mode = (
            key != "exhibition_stand"
            and self.controller.is_running("exhibition")
            and self.exhibition_requested_mode == key
        )
        if same_mode:
            # A writer-side safeguard can latch KILL while the healthy graph,
            # video and VR bridge remain alive.  Reusing that graph avoids a
            # minute-long SDK restart and gives RUN the same quick recovery
            # behavior an operator expects from Unitree Explore.
            environment = self._live_environment(
                run_mode_confirmed=spec.requires_run_mode
            )
            self.exhibition_mode = "Повторный запуск…"
            self._refresh_summary()
            if not self.controller.start(
                "exhibition_rearm",
                self.specs["exhibition_rearm"],
                env_overrides=environment,
            ):
                self._finish_action(key, "ошибка", "RUN helper не запустился")
            return
        if self.controller.is_running("exhibition_stop"):
            self._append_log(key, "[INFO] Дождитесь завершения безопасной остановки.\n")
            self._finish_action(key, "заблокировано", "безопасная остановка ещё выполняется")
            return

        require_vr = key == "exhibition_control"
        if not self._confirm_live(
            spec,
            automatic_preflight=True,
            require_vr=require_vr,
            session_arm=require_vr,
            require_run_mode=spec.requires_run_mode,
            # LOCK and RUN are the two normal exhibition actions.  They run
            # the automatic preflight inside the selected mode and must stay
            # one-click for an untrained operator; do not open the legacy
            # per-item acknowledgement dialog here.
            skip_checklist=True,
        ):
            self._finish_action(key, "отменено", "подтверждение оператора не получено")
            return
        self.exhibition_session_confirmed = True
        environment = self._live_environment(
            run_mode_confirmed=spec.requires_run_mode
        )

        # Stop the read-only worker asynchronously and wait for its owned
        # temporary reader cleanup. The manager will check any completed
        # attestation or perform its authoritative preflight itself.
        if (
            not self.controller.is_running("exhibition")
            and self.controller.is_running("sdk_warmup")
        ):
            was_pending = bool(self.__dict__.get("pending_warmup_key"))
            self.pending_warmup_key = key
            self.pending_warmup_environment = environment
            warmup_phase = str(
                self.status_values.get("sdk_warmup", "")
            ).strip().upper()
            cache_available = bool(
                self.__dict__.get("sdk_warmup_cache_available", False)
            )
            if warmup_phase == "CHECKING" and not cache_available:
                # The worker has already invalidated every old attestation and
                # is obtaining fresh physical telemetry. Cancelling it here
                # made the manager repeat the same 30-40 second read-only path
                # from zero. Await only this already-running bounded check;
                # READY still has to be followed by confirmed worker cleanup
                # before any physical graph may start.
                self.exhibition_mode = "Жду уже идущую SDK-проверку…"
                self._refresh_summary()
                if not self.__dict__.get("warmup_handoff_waiting", False):
                    self.warmup_handoff_waiting = True
                    generation = int(
                        self.__dict__.get("warmup_handoff_generation", 0)
                    ) + 1
                    self.warmup_handoff_generation = generation
                    self._append_log(
                        key,
                        "[INFO] Свежая read-only SDK-проверка уже выполняется; "
                        "сохраняю её прогресс вместо повторного запуска.\n",
                    )
                    QTimer.singleShot(
                        SDK_WARMUP_HANDOFF_WAIT_MS,
                        lambda current=generation: self._expire_warmup_handoff(
                            current
                        ),
                    )
                return

            self.warmup_handoff_waiting = False
            self.exhibition_mode = "Завершение фоновой проверки…"
            self._refresh_summary()
            if not was_pending:
                self.controller.stop("sdk_warmup", graceful_timeout_ms=12000, wait=False)
            return

        if self.controller.is_running("exhibition") or (
            bool(existing) and key in {"exhibition_stand", "exhibition_control"}
        ):
            self.pending_exhibition_key = key
            self.pending_exhibition_environment = environment
            self.exhibition_switch_in_progress = True
            self.exhibition_stop_complete = False
            self.exhibition_manager_stopped = False
            self.exhibition_mode = "Переключение…"
            self._refresh_summary()
            self._append_log(
                key,
                "[INFO] Сначала безопасно останавливаю текущий выставочный режим.\n",
            )
            self.controller.start(
                "exhibition_stop", self.specs["exhibition_stop"]
            )
            return
        self._start_exhibition_mode(key, environment)

    def _start_exhibition_mode(
        self, key: str, environment: Optional[Dict[str, str]] = None
    ) -> None:
        self.exhibition_requested_mode = key
        self.exhibition_mode = "Запускается…"
        started = self.controller.start(
            "exhibition", self.specs[key], env_overrides=environment
        )
        if not started:
            self.exhibition_mode = "Остановлен"
            self._finish_action(key, "ошибка", "процесс режима не запустился")
        self._refresh_summary()

    def _start_pending_exhibition_mode(self) -> None:
        key = self.pending_exhibition_key
        environment = self.pending_exhibition_environment
        self.pending_exhibition_key = None
        self.pending_exhibition_environment = None
        self.exhibition_switch_in_progress = False
        self.exhibition_stop_complete = False
        self.exhibition_manager_stopped = False
        if key:
            self._start_exhibition_mode(key, environment)

    def _start_pending_warmup_mode(self) -> None:
        if self.controller.is_running("sdk_warmup"):
            return
        self.warmup_handoff_waiting = False
        key = self.pending_warmup_key
        environment = self.pending_warmup_environment
        self.pending_warmup_key = None
        self.pending_warmup_environment = None
        if not key:
            return
        if (
            self.__dict__.get("pending_zero_torque", False)
            or self.__dict__.get("close_after_stop", False)
            or self.__dict__.get("exhibition_switch_in_progress", False)
            or any(self.controller.is_running(action) for action in (
                "exhibition", "exhibition_stop", "stop", "kill", "zero_torque"))
            or active_session_snapshot()
        ):
            self._append_log(
                key,
                "[BLOCKED] Состояние изменилось во время фоновой проверки; "
                "повторите выбор режима после завершения текущего действия.\n",
            )
            return
        self._start_exhibition_mode(key, environment)

    def _expire_warmup_handoff(self, generation: int) -> None:
        """Bound a pending warmup without blocking or bypassing preflight."""
        if generation != int(
            self.__dict__.get("warmup_handoff_generation", 0)
        ):
            return
        if not self.__dict__.get("warmup_handoff_waiting", False):
            return
        key = self.__dict__.get("pending_warmup_key")
        if not key:
            self.warmup_handoff_waiting = False
            return
        self.warmup_handoff_waiting = False
        self._append_log(
            key,
            "[WARN] Фоновая SDK-проверка не завершилась за 45 секунд; "
            "передаю запуск штатному manager preflight.\n",
        )
        if self.controller.is_running("sdk_warmup"):
            self.controller.stop(
                "sdk_warmup", graceful_timeout_ms=12000, wait=False
            )
        else:
            QTimer.singleShot(0, self._start_pending_warmup_mode)

    def _background_warmup_restart_blocked(self) -> bool:
        """Return whether a recovered-link warmup must yield to physical work."""
        if (
            self.__dict__.get("close_after_stop", False)
            or self.__dict__.get("pending_zero_torque", False)
            or self.__dict__.get("zero_torque_idle", False)
            or self.__dict__.get("pending_warmup_key")
            or self.__dict__.get("pending_exhibition_key")
            or self.__dict__.get("exhibition_switch_in_progress", False)
        ):
            return True
        physical_processes = (
            "exhibition",
            "exhibition_stop",
            "exhibition_lock",
            "exhibition_rearm",
            "motion",
            "robot:prepare",
            "stop",
            "kill",
            "zero_torque",
            ZERO_TORQUE_HANDOFF_KEY,
        )
        if any(self.controller.is_running(key) for key in physical_processes):
            return True
        return bool(active_session_snapshot())

    def _start_recovered_sdk_warmup(self) -> None:
        """Start one fresh read-only worker after its predecessor has exited."""
        if not self.__dict__.get("warmup_restart_pending", False):
            return
        if self._background_warmup_restart_blocked():
            self.warmup_restart_pending = False
            return
        if self.controller.is_running("sdk_warmup"):
            return
        self.warmup_restart_pending = False
        self.sdk_warmup_cache_available = False
        if not self.controller.start("sdk_warmup", self.specs["sdk_warmup"]):
            self.status_values["sdk_warmup"] = "RESTART_FAILED"
            self._append_log(
                "sdk_warmup",
                "[WARN] Не удалось запустить свежий read-only SDK warmup; "
                "панель повторит попытку при следующем опросе.\n",
            )

    def _request_recovered_sdk_warmup(self) -> None:
        """Replace a stale warmup asynchronously after OFFLINE -> healthy."""
        if self._background_warmup_restart_blocked():
            self.warmup_restart_pending = False
            return
        if self.__dict__.get("warmup_restart_pending", False):
            return
        self.warmup_restart_pending = True
        self.status_values["sdk_warmup"] = "RESTARTING"
        self.sdk_warmup_cache_available = False
        self._append_log(
            "sdk_warmup",
            "[INFO] Ethernet восстановлен; перезапускаю только read-only SDK "
            "warmup для свежего адреса и кэша.\n",
        )
        if self.controller.is_running("sdk_warmup"):
            self.controller.stop(
                "sdk_warmup", graceful_timeout_ms=12000, wait=False
            )
            return
        QTimer.singleShot(0, self._start_recovered_sdk_warmup)

    def _poll_panel_status(self) -> None:
        """Refresh the read-only health snapshot without blocking the UI."""
        if self.zero_torque_idle:
            return
        if not self.background_services_started:
            self.start_background_services()
        if not self.controller.is_running("battery_monitor"):
            self.controller.start(
                "battery_monitor", self.specs["battery_monitor"]
            )
        if (
            not self.controller.is_running("exhibition")
            and not self.controller.is_running("sdk_warmup")
            and not self.__dict__.get("pending_warmup_key")
            and not self.__dict__.get("warmup_restart_pending", False)
            and not active_session_snapshot()
        ):
            self.sdk_warmup_cache_available = False
            self.controller.start("sdk_warmup", self.specs["sdk_warmup"])
        self._request_status_refresh()

    def _parse_status_output(self, text: str) -> None:
        updated = set()
        robot_recovered = False
        for line in text.splitlines():
            if not line.startswith("STATUS ") or "=" not in line:
                continue
            key, value = line[7:].split("=", 1)
            self.status_values[key.strip()] = value.strip()
            updated.add(key.strip())
            if key.strip() == "robot" and value.strip() == "OFFLINE":
                # A previously successful check must not authorize a live
                # session after the robot disappears from the LAN.
                self.preflight_ok = False
                self.pending_warmup_key = None
                self.pending_warmup_environment = None
                self.warmup_handoff_waiting = False
                self.sdk_warmup_cache_available = False
                self.warmup_robot_offline_observed = True
                self._clear_live_confirmation()
            elif key.strip() == "robot" and value.strip().upper() in {
                "OK",
                "FOUND",
                "RUNNING",
                "READY",
            }:
                robot_recovered = robot_recovered or bool(
                    self.__dict__.get("warmup_robot_offline_observed", False)
                )
                self.warmup_robot_offline_observed = False
            # A latched software KILL is the required startup state for a
            # physical writer. Keep it visible, but do not invalidate a fresh
            # read-only preflight merely because the fail-closed latch is set.
        # Both fields must come from this snapshot, not a previous session.
        if {"exhibition", "mode"} <= updated and not self.__dict__.get(
            "exhibition_switch_in_progress", False
        ):
            status = self.status_values["exhibition"]
            mode = self.status_values["mode"]
            if status == "ready":
                self.exhibition_mode = "Управление" if mode == "control" else "Стойка"
            elif status == "locked":
                self.exhibition_mode = "LOCK — удержание позы"
            self._apply_exhibition_result(
                mode,
                status,
                self.status_values.get("exhibition_detail", ""),
            )
        if "robot" in updated:
            robot = self.status_values.get("robot", "OFFLINE").upper()
            healthy = robot in {"OK", "FOUND", "RUNNING", "READY"}
            if "connect" in self.__dict__.setdefault("action_started_at", {}):
                if healthy:
                    self._finish_action(
                        "connect", "готово", "последний рабочий Ethernet-адрес отвечает"
                    )
                else:
                    self._finish_action(
                        "connect", "не найден", "Ethernet-проверка завершена без ответа робота"
                    )
                    self._show_retry_reason(
                        "connect", "робот не отвечает по существующему Ethernet"
                    )
            if (
                "exhibition_reconnect"
                in self.__dict__.setdefault("action_started_at", {})
                and not active_session_snapshot()
            ):
                result = "готово" if healthy else "не найден"
                detail = (
                    "локальные службы сохранены; Ethernet отвечает"
                    if healthy
                    else "локальные службы запущены, робот пока не отвечает"
                )
                self._finish_action("exhibition_reconnect", result, detail)
            if robot_recovered:
                self._request_recovered_sdk_warmup()
        self._refresh_exhibition_status()

    def _apply_exhibition_result(self, mode: str, status: str, detail: str) -> None:
        """Map one manager acknowledgement to the pending main-screen action."""
        mode = str(mode).strip().lower()
        status = str(status).strip().lower()
        if status == "ready" and mode == "control":
            self.exhibition_mode = "Управление"
            self._finish_action("exhibition_control", "готово", detail)
        elif status == "ready" and mode == "static":
            action = (
                "exhibition_stand"
                if "exhibition_stand"
                in self.__dict__.setdefault("action_started_at", {})
                else "exhibition_static"
            )
            self.exhibition_mode = "Стойка" if action == "exhibition_stand" else "Статичный"
            self._finish_action(action, "готово", detail)
        elif status == "locked":
            self.exhibition_mode = "LOCK — удержание позы"
            self._finish_action("exhibition_static", "готово", detail)
        elif status in {"blocked", "degraded"} or (
            status == "stopped"
            and not self.controller.is_running("exhibition")
            and not self.__dict__.get("pending_warmup_key")
            and not self.__dict__.get("exhibition_switch_in_progress", False)
        ):
            pending = self.__dict__.setdefault("action_started_at", {})
            for action in (
                "exhibition_control",
                "exhibition_stand",
                "exhibition_static",
            ):
                if action in pending:
                    reason = detail or f"manager status={status}"
                    self._finish_action(action, "ошибка", reason)
                    self._show_retry_reason(action, reason)
                    break

    def _consume_exhibition_events(
        self, key: str, text: str, *, final: bool = False
    ) -> None:
        for raw in self._complete_output_lines(
            f"exhibition:{key}", text, final=final
        ):
            line = raw.strip()
            if not line.startswith("EXHIBITION_STATE "):
                continue
            try:
                payload = json.loads(line.split(" ", 1)[1])
            except (ValueError, TypeError, json.JSONDecodeError):
                continue
            if not isinstance(payload, dict):
                continue
            mode = str(payload.get("mode", ""))
            status = str(payload.get("status", ""))
            detail = str(payload.get("detail", ""))
            self.status_values["mode"] = mode
            self.status_values["exhibition"] = status
            self.status_values["exhibition_detail"] = detail
            self._apply_exhibition_result(mode, status, detail)
            self._refresh_exhibition_status()

    def _complete_output_lines(
        self, buffer_key: str, text: str, *, final: bool = False
    ) -> list[str]:
        """Return complete lines while retaining one fragmented QProcess line."""

        buffers = self.__dict__.setdefault("output_line_buffers", {})
        combined = buffers.get(buffer_key, "") + text
        lines = combined.splitlines(keepends=True)
        buffers[buffer_key] = ""
        if (
            not final
            and lines
            and not lines[-1].endswith(("\n", "\r"))
        ):
            buffers[buffer_key] = lines.pop()
        if final and combined and not lines:
            lines = [combined]
        if final:
            buffers.pop(buffer_key, None)
        return lines

    def _consume_sdk_warmup_events(
        self, text: str, *, final: bool = False
    ) -> None:
        """Parse warmup state lines even when Qt splits one shell ``echo``."""

        for raw in self._complete_output_lines(
            "warmup:sdk_warmup", text, final=final
        ):
            line = raw.strip()
            if not line.startswith("SDK_WARMUP state="):
                continue
            phase = line.split("state=", 1)[1].split()[0]
            self.status_values["sdk_warmup"] = phase
            if phase in {"READY", "REFRESHING"}:
                self.sdk_warmup_cache_available = True
            elif phase in {"WAITING_FOR_ROBOT", "RETRY"}:
                self.sdk_warmup_cache_available = False
            if (
                self.__dict__.get("pending_warmup_key")
                and self.__dict__.get("warmup_handoff_waiting", False)
                and phase == "READY"
            ):
                self.warmup_handoff_waiting = False
                self._append_log(
                    self.pending_warmup_key,
                    "[OK] Фоновая SDK-проверка готова; жду подтверждённой "
                    "очистки reader и запускаю выбранный режим.\n",
                )
                self.controller.stop(
                    "sdk_warmup", graceful_timeout_ms=12000, wait=False
                )

    def _consume_process_diagnostics(
        self, key: str, text: str, *, final: bool = False
    ) -> None:
        """Remember the concrete failure line across fragmented process output."""

        saw_warning = False
        saw_success = False
        for raw in self._complete_output_lines(
            f"diagnostic:{key}", text, final=final
        ):
            line = raw.strip()
            if "[FAIL]" in line or "[BLOCKED]" in line:
                reason = re.sub(
                    r"^.*?\[(?:FAIL|BLOCKED)\]\s*", "", line
                ).strip()
                if not reason:
                    reason = line[:280]
                self.__dict__.setdefault("last_process_errors", {})[key] = reason
                self.last_status = "Красный • обнаружена ошибка или блокировка"
                action = self._action_for_process(key)
                if key == "exhibition":
                    action = self.__dict__.get("exhibition_requested_mode")
                self._show_retry_reason(action, reason)
            elif "[WARN]" in line:
                saw_warning = True
            elif "[OK]" in line or "[PASS]" in line:
                saw_success = True
        if self.__dict__.setdefault("last_process_errors", {}).get(key):
            return
        if saw_warning:
            self.last_status = "Жёлтый • есть предупреждения"
        elif saw_success:
            self.last_status = "Зелёный • последняя проверка прошла"

    def _run_check_all(self) -> None:
        # A fresh run must re-open the gate only after all read-only checks
        # complete successfully; stale success cannot authorize a new session.
        self.preflight_ok = False
        command = (
            "status=0; "
            "run_check() { \"$@\"; rc=$?; "
            "if (( rc > status )); then status=$rc; fi; return 0; }; "
            "run_advisory() { \"$@\"; rc=$?; "
            "if (( rc > 0 )); then echo '[WARN] advisory check returned '\"$rc\"; fi; "
            "return 0; }; "
            "echo '=== Сеть и Ethernet ==='; run_check make r1-lan-preflight; "
            "echo '=== Робот / VR / ROS2 ==='; run_check make robot-preflight; "
            "echo '=== Камера и DDS (диагностика) ==='; "
            "run_advisory make r1-camera-preflight; "
            "echo '=== Сводный статус ==='; run_check make r1-teleoperation-status; "
            "exit \"$status\""
        )
        spec = self.specs["check_all"]
        self.controller.start("check_all", spec, shell_command=command)

    def _confirm_live(
        self,
        spec: CommandSpec,
        *,
        automatic_preflight: bool = False,
        require_vr: bool = True,
        session_arm: bool = False,
        require_run_mode: bool = False,
        skip_checklist: bool = False,
    ) -> bool:
        if (
            self.config.require_preflight
            and not self.preflight_ok
            and not automatic_preflight
        ):
            QMessageBox.warning(
                self, "Сначала preflight",
                "Live-сеанс заблокирован. Нажмите «Проверить всё» и дождитесь "
                "успешного завершения.",
            )
            return False
        if not self.config.allow_live:
            QMessageBox.warning(
                self,
                "Live-режим отключён",
                "В настройках включите «Разрешить кнопкам запрашивать live-сеанс». "
                "По умолчанию панель работает в dry-run.",
            )
            return False
        if self.config.dry_run:
            QMessageBox.warning(
                self,
                "Панель в dry-run",
                "Для физического запуска выключите «Безопасный dry-run по "
                "умолчанию» в настройках панели.",
            )
            return False
        # USB authorization/discovery is enforced by usb_link.control before
        # it starts any manager. Do not block the Qt thread on a 15-second ADB
        # timeout here: STOP and the other buttons must remain responsive.
        if require_vr and self.config.vr_transport == "lan" and not _is_headset_lan_address(
            self.config.vr_headset_ip.strip()
        ):
            QMessageBox.warning(
                self,
                "Укажите IP VR-шлема",
                "В настройках панели нужен фиксированный приватный IPv4-адрес "
                "VR-шлема, например 192.168.8.129.",
            )
            return False
        if skip_checklist:
            return True
        dialog = QDialog(self)
        dialog.setWindowTitle("Проверка безопасности перед live")
        layout = QVBoxLayout(dialog)
        layout.addWidget(QLabel(f"Перед запуском: {spec.title}"))
        checks = []
        for text in self._live_checklist_items(
            require_vr, session_arm, require_run_mode
        ):
            check = QCheckBox(text)
            layout.addWidget(check)
            checks.append(check)
        note_text = (
            "После полного чек-листа панель создаёт одноразовый commissioning "
            "token для этой выставочной сессии. Повторный чек-лист при смене "
            "режима не потребуется. Остальные KILL/STOP и safety gates остаются "
            "обязательными."
        )
        if automatic_preflight:
            note_text += " Preflight выполнится автоматически внутри выбранного режима."
        if session_arm:
            note_text += (
                " Управление использует session-arm до Статичного режима, STOP "
                "или KILL; постоянно удерживать Deadman не требуется."
            )
        note = QLabel(note_text)
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
        self.exhibition_session_confirmed = True
        try:
            record_acknowledgement(self.config)
        except OSError as exception:
            self._append_log(
                "live_ack",
                f"[WARN] Не удалось сохранить локальное подтверждение: {exception}\n",
            )
        return True

    def _clear_live_confirmation(self) -> None:
        """Invalidate the short-lived checklist acknowledgement."""
        self.exhibition_session_confirmed = False
        try:
            clear_acknowledgement()
        except OSError as exception:
            self._append_log(
                "live_ack",
                f"[WARN] Не удалось удалить локальное подтверждение: {exception}\n",
            )

    @staticmethod
    def _live_checklist_items(
        require_vr: bool,
        session_arm: bool,
        require_run_mode: bool = False,
    ) -> tuple[str, ...]:
        items = [
            "Робот снят с зарядки, батарея установлена и питание стабильно",
            "Робот стоит устойчиво на полу или надёжно подвешен для slow-теста",
            "Вокруг робота свободная зона",
            "Человек рядом готов немедленно выключить питание батареи при опасности",
        ]
        if require_vr:
            if session_arm:
                items.append(
                    "VR-шлем и контроллеры готовы; примите любую удобную позу и "
                    "не двигайтесь несколько секунд во время автокалибровки; session-arm "
                    "останется активен до Статичного режима, STOP или KILL"
                )
            else:
                items.append("VR-шлем и контроллеры готовы, deadman проверен")
        if require_run_mode:
            items.append(
                "Unitree Explore не управляет роботом: закройте экран управления "
                "и не используйте виртуальный стик телефона; панель автоматически "
                "переведёт робота в Run (FSM 811)"
            )
        return tuple(items)

    def _live_environment(self, *, run_mode_confirmed: bool = False) -> Dict[str, str]:
        """Build acknowledgements for one explicitly confirmed panel session."""
        token = getattr(self, "live_session_token", "")
        if len(token) < 16:
            token = secrets.token_urlsafe(24)
            self.live_session_token = token
        environment = {
            "ROBOT_DRY_RUN": "0",
            "ROBOT_ENABLE_ACTUATION": "1",
            "ROBOT_CONFIRM_OFF_CHARGER": "1",
            "ROBOT_CONFIRM_CLEAR_AREA": "1",
            "ROBOT_CONFIRM_ESTOP_READY": "1",
            "ROBOT_CONFIRM_COMMISSIONING": "1",
            "ROBOT_COMMISSIONING_TOKEN": token,
            "ROBOT_VR_SOURCE_IP": (
                "127.0.0.1" if self.config.vr_transport == "usb"
                else self.config.vr_headset_ip.strip()
            ),
            "R1_VR_TRANSPORT": self.config.vr_transport,
        }
        if run_mode_confirmed:
            environment["ROBOT_CONFIRM_RUN_MODE"] = "1"
            environment["ROBOT_CONFIRM_NO_PHONE_CONTROL"] = "1"
        return environment

    def _on_output(self, key: str, text: str) -> None:
        self._append_log(key, text)
        if key == "exhibition":
            self._consume_exhibition_events(key, text)
        if key == "sdk_warmup":
            self._consume_sdk_warmup_events(text)
        if key in {"panel_status", "panel_status_fast"}:
            self._parse_status_output(text)
        if key == "voice_check" and hasattr(self, "voice_status"):
            self.voice_status.setText(
                "ASR/LLM/TTS preflight:\n" + text[-1600:].strip()
            )
        if key == "voice:voice_remote":
            if "VOICE status=ACTIVE" in text:
                self.status_values["voice"] = "ACTIVE"
            elif "VOICE status=" in text:
                self.status_values["voice"] = "OFFLINE"
        if key == "connection_ensure" and "CONNECT state=" in text:
            if "CONNECT state=ERROR" in text:
                self.connection_state = "offline"
            elif "CONNECT state=" in text and self.connection_state != "connected":
                self.connection_state = "searching"
        if key == "battery_monitor":
            for line in text.splitlines():
                if not line.startswith("BATTERY soc="):
                    continue
                try:
                    percent = int(line.split("soc=", 1)[1].split()[0])
                except (ValueError, IndexError):
                    continue
                if 0 <= percent <= 100:
                    self.status_values["battery"] = str(percent)
                    if "battery_label" in self.__dict__:
                        self.battery_label.setText(f"{percent}%")
                        self.battery_label.setProperty(
                            "batteryLow", percent <= 20
                        )
                        self.battery_label.style().unpolish(self.battery_label)
                        self.battery_label.style().polish(self.battery_label)
        if key == "command_capture" and "CAPTURE saved=" in text:
            for line in text.splitlines():
                if line.startswith("CAPTURE saved="):
                    self.last_capture_path = line.split("=", 1)[1].strip()
                    self.capture_status.setText(
                        "Сохранено: " + self.last_capture_path
                    )
        self._consume_process_diagnostics(key, text)
        self._refresh_summary()

    def _append_log(self, key: str, text: str) -> None:
        stamp = _datetime.datetime.now().strftime("%H:%M:%S")
        for line in text.splitlines(True):
            self.log_view.appendPlainText(f"[{stamp}] [{key}] {line.rstrip()}")

    def _on_started(self, key: str) -> None:
        self.__dict__.setdefault("last_process_errors", {}).pop(key, None)
        buffers = self.__dict__.setdefault("output_line_buffers", {})
        for prefix in ("diagnostic", "warmup", "exhibition"):
            buffers.pop(f"{prefix}:{key}", None)
        self._append_log(key, "[INFO] процесс запущен\n")
        if key == "exhibition":
            # Starting the Python process does not confirm the robot's FSM.
            self.exhibition_mode = "Подготовка — жду подтверждения робота…"
        self._refresh_summary()

    def _on_finished(self, key: str, code: int, _status: int) -> None:
        if key == "exhibition":
            self._consume_exhibition_events(key, "", final=True)
        if key == "sdk_warmup":
            self._consume_sdk_warmup_events("", final=True)
        self._consume_process_diagnostics(key, "", final=True)
        level = "OK" if code == 0 else "WARN"
        self._append_log(key, f"[{level}] процесс завершён с кодом {code}\n")
        action = self._action_for_process(key)
        if action and key != "zero_torque":
            error_detail = self.__dict__.setdefault(
                "last_process_errors", {}
            ).get(key)
            self._finish_action(
                action,
                "готово" if code == 0 else "ошибка",
                (
                    "команда подтверждена"
                    if code == 0
                    else error_detail or f"helper завершился с кодом {code}"
                ),
            )
        if key == "sdk_warmup":
            self.warmup_handoff_waiting = False
            if self.__dict__.get("pending_warmup_key"):
                self.warmup_restart_pending = False
                if code == 0:
                    QTimer.singleShot(0, self._start_pending_warmup_mode)
                else:
                    self.pending_warmup_key = None
                    self.pending_warmup_environment = None
                    self.exhibition_mode = (
                        "Фоновая проверка завершилась с ошибкой — повторите "
                        "выбор режима"
                    )
                    self._append_log(
                        key,
                        "[BLOCKED] Штатное завершение фоновой проверки не "
                        "подтверждено; отложенный запуск отменён.\n",
                    )
            elif self.__dict__.get("warmup_restart_pending", False):
                if code == 0:
                    QTimer.singleShot(0, self._start_recovered_sdk_warmup)
                else:
                    # Keep the pending guard latched.  A non-zero owner exit
                    # does not prove that its temporary SDK reader was reaped,
                    # so the normal status poll must not start a replacement.
                    self.status_values["sdk_warmup"] = "RESTART_FAILED"
                    self._append_log(
                        key,
                        "[BLOCKED] Очистка старого SDK warmup не подтверждена; "
                        "автоматический перезапуск заблокирован.\n",
                    )
        if key == ZERO_TORQUE_HANDOFF_KEY:
            if self.__dict__.get("pending_zero_torque", False):
                if code == 0:
                    self.zero_torque_handoff_complete = True
                    self._append_log(
                        "zero_torque",
                        "[OK] offline bridge остановлен, порты освобождены; "
                        "продолжаю к Damping → Zero Torque.\n",
                    )
                else:
                    self.pending_zero_torque = False
                    self.zero_torque_handoff_complete = False
                    self._finish_action(
                        "zero_torque", "ошибка", f"offline handoff завершился с кодом {code}"
                    )
                    QMessageBox.critical(
                        self,
                        "Zero Torque отменён",
                        "Offline bridge не завершился штатно или его порты остались заняты. "
                        "Damping и Zero Torque роботу не отправлялись.",
                    )
        if (
            self.__dict__.get("pending_zero_torque", False)
            and key in ("stop", "kill", "exhibition_stop")
            and code != 0
        ):
            self.pending_zero_torque = False
            QMessageBox.critical(
                self,
                "Zero Torque отменён",
                "Управляющий сеанс не удалось штатно остановить. "
                "Damping и Zero Torque роботу не отправлялись.",
            )
        if key == "command_capture":
            if code == 0:
                if self.last_capture_path:
                    self.capture_status.setText(
                        "Запись сохранена: " + self.last_capture_path
                    )
                else:
                    self.capture_status.setText(
                        "Запись сохранена. Можно выбрать следующую кнопку."
                    )
            else:
                self.capture_status.setText(
                    "Запись не завершилась — проверьте Ethernet и повторите."
                )
        if key in ("stop", "kill", "exhibition_stop"):
            # STOP/KILL completes the writer's bounded cleanup first. Then
            # close the owning launch group so SDK readers/writers cannot stay
            # connected on a charger or contaminate the next fresh session.
            exhibition_was_running = self.controller.is_running("exhibition")
            self.controller.stop("motion", wait=False)
            self.controller.stop("robot:prepare", wait=False)
            self.controller.stop(
                "exhibition",
                graceful_timeout_ms=EXHIBITION_GRACEFUL_STOP_MS,
                wait=False,
            )
            self.preflight_ok = False
            self._append_log(
                key,
                "[INFO] STOP/KILL path completed; waiting for physical process "
                "owners to finish child cleanup before the next live session\n",
            )
            pending_switch = (
                key == "exhibition_stop"
                and code == 0
                and bool(self.pending_exhibition_key)
            )
            if pending_switch:
                self.exhibition_stop_complete = True
                self.exhibition_mode = "Переключение…"
                # Reuse the logical slot only after both the reviewed STOP
                # helper and the manager's own child cleanup have completed.
                if not exhibition_was_running or self.exhibition_manager_stopped:
                    QTimer.singleShot(0, self._start_pending_exhibition_mode)
            else:
                self.pending_exhibition_key = None
                self.pending_exhibition_environment = None
                self.exhibition_switch_in_progress = False
                self.exhibition_stop_complete = False
                self.exhibition_manager_stopped = False
                self.exhibition_requested_mode = ""
                self.exhibition_mode = "Остановлен"
                self._clear_live_confirmation()
        elif key == "exhibition":
            if self.exhibition_switch_in_progress and self.pending_exhibition_key:
                self.exhibition_manager_stopped = True
                self.exhibition_mode = "Переключение…"
                if self.exhibition_stop_complete:
                    QTimer.singleShot(0, self._start_pending_exhibition_mode)
            else:
                failed_action = self.exhibition_requested_mode
                self.exhibition_requested_mode = ""
                self.exhibition_mode = "Остановлен"
                if code != 0 and failed_action:
                    failure_detail = self.__dict__.setdefault(
                        "last_process_errors", {}
                    ).get("exhibition")
                    if not failure_detail and self.status_values.get(
                        "exhibition"
                    ) in {"blocked", "degraded"}:
                        failure_detail = self.status_values.get(
                            "exhibition_detail", ""
                        )
                    self._finish_action(
                        failed_action,
                        "ошибка",
                        failure_detail
                        or f"manager завершился с кодом {code}; причина не была выведена",
                    )
                # A local startup failure does not change the conditions that
                # the operator just confirmed.  Keep the bounded runtime
                # acknowledgement for an immediate retry; robot loss,
                # STOP/KILL, Zero Torque and expiry still invalidate it.
        elif key == "exhibition_rearm":
            self.exhibition_mode = (
                "Управление" if code == 0 else "Возврат управления не подтверждён — см. статус"
            )
        elif key == "exhibition_lock":
            self.exhibition_mode = (
                "Статичный" if code == 0 else "Управление"
            )
        if self.__dict__.get("pending_zero_torque", False):
            QTimer.singleShot(0, self._continue_zero_torque_when_idle)
        if key == "zero_torque":
            self.exhibition_requested_mode = ""
            self.exhibition_mode = "Zero Torque" if code == 0 else "Остановлен"
            self.preflight_ok = False
            self.background_services_started = False
            self._finish_action(
                "zero_torque",
                "готово" if code == 0 else "ошибка",
                "FSM 0 подтверждён" if code == 0 else f"helper завершился с кодом {code}",
            )
            if code == 0:
                QMessageBox.information(
                    self,
                    "Zero Torque включён",
                    "Управляющие процессы завершены. Робот расслаблен, "
                    "удерживающий момент отключён.",
                )
            else:
                QMessageBox.critical(
                    self,
                    "Zero Torque не подтверждён",
                    "Команда не была подтверждена роботом. Подробности находятся "
                    "в разделе «Расширенные настройки → Логи».",
                )
        self._refresh_summary()
        if key == "check_all":
            self.preflight_ok = code == 0
            robot = self.status_values.get("robot", "OFFLINE").upper()
            self.connection_state = (
                "connected"
                if robot in {"OK", "FOUND", "RUNNING", "READY"}
                else "offline"
            )
            self._append_log(
                key,
                "[OK] preflight gate открыт\n" if self.preflight_ok
                else "[BLOCKED] preflight gate остаётся закрыт\n",
            )
            self._refresh_summary()
        if self.close_after_stop and not self.controller.active_keys():
            QApplication.instance().quit()

    def open_viewer(self) -> None:
        if not self._begin_action("viewer"):
            return
        # Reuse the existing local stream and watchdog. Opening a browser must
        # not run the complete robot/VR preflight or start a second video server.
        self._ensure_main_services()
        url = self.config.video_url.strip() or "http://127.0.0.1:8080/"
        opened = QDesktopServices.openUrl(QUrl(url))
        if not opened:
            self._finish_action("viewer", "ошибка", "система не открыла локальный URL")
            self._show_retry_reason("viewer", "не удалось открыть локальный URL")
            return
        if self.video_state == "online":
            self._finish_action(
                "viewer", "готово", "использован уже работающий локальный поток"
            )
        else:
            self._append_log(
                "viewer",
                f"[INFO] открываю {url}; видеопоток ещё запускается, "
                "основное окно не блокируется\n",
            )

    def open_logs(self) -> None:
        path = Path(self.config.project_dir) / self.config.logs_dir
        path.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def export_report(self) -> None:
        """Write a local JSON snapshot useful for support without secrets."""
        destination = Path(self.config.project_dir) / self.config.logs_dir / (
            "operator-report-" + _datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".json"
        )
        destination.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "generated_at": _datetime.datetime.now(_datetime.timezone.utc).isoformat(),
            "config": {
                "robot_ip": self.config.robot_ip,
                "pc2_ip": self.config.pc2_ip,
                "interface": self.config.robot_interface,
                "allow_half_duplex_adapter": (
                    self.config.allow_half_duplex_adapter
                ),
                "ros_domain_id": self.config.ros_domain_id,
                "video_profile": self.config.video_profile,
                "locomotion_profile": self.config.locomotion_profile,
                "dry_run": self.config.dry_run,
            },
            "preflight_ok": self.preflight_ok,
            "active_processes": self.controller.active_keys(),
            "health": self.status_values,
            "button_timings": self.last_action_timings,
        }
        destination.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        self._append_log("report", f"[OK] отчёт сохранён: {destination}\n")

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
        background_only = {
            "battery_monitor",
            "connection_ensure",
            "voice:voice_remote",
            "panel_status",
            "check_all",
            "sdk_warmup",
        }
        if active and set(active).issubset(background_only):
            event.ignore()
            if not self.close_after_stop:
                self._begin_close_cleanup()
            return
        if not active:
            if "video_preview" in self.__dict__:
                self.video_preview.stop()
            event.accept()
            return
        if self.close_after_stop:
            # A second window-close gesture must not bypass the cleanup that is
            # already disarming/stopping the physical session.
            event.ignore()
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
            event.ignore()
            self._begin_close_cleanup()
            return
        if "video_preview" in self.__dict__:
            self.video_preview.stop()
        event.accept()

    def _begin_close_cleanup(self) -> None:
        """Close only after the reviewed STOP and manager cleanup complete."""
        self.close_after_stop = True
        self.pending_zero_torque = False
        self.status_timer.stop()
        if "video_preview" in self.__dict__:
            self.video_preview.stop()
        self.pending_exhibition_key = None
        self.pending_exhibition_environment = None
        self.pending_warmup_key = None
        self.pending_warmup_environment = None
        self.warmup_restart_pending = False
        self.exhibition_switch_in_progress = False
        self.exhibition_stop_complete = False
        self.exhibition_manager_stopped = False

        active = set(self.controller.active_keys())
        physical = bool(
            active.intersection(
                {"exhibition", "exhibition_stop", "motion", "robot:prepare"}
            )
        )
        # Diagnostics/video can stop independently.  Physical owners remain
        # alive until exhibition-stop asserts the reviewed robot STOP path.
        for key in active.difference(
            {"exhibition", "exhibition_stop", "motion", "robot:prepare"}
        ):
            self.controller.stop(
                key, graceful_timeout_ms=12000 if key == "sdk_warmup" else 1200,
                wait=False,
            )

        if physical:
            if not self.controller.is_running("exhibition_stop"):
                self.controller.start(
                    "exhibition_stop", self.specs["exhibition_stop"]
                )
            return

        # No physical graph exists; the remaining helpers have been stopped.
        if not self.controller.active_keys():
            app = QApplication.instance()
            if app is not None:
                QTimer.singleShot(0, app.quit)


def build_app(config: Optional[OperatorConfig] = None) -> QApplication:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(
        """
        * { outline: none; }
        QWidget { font-family: "Inter", "SF Pro Display", "Noto Sans", sans-serif;
                  font-size: 10pt; }
        QMainWindow, #rootPanel { background: #0f1115; color: #f5f5f7; }
        QDialog#settingsDialog { background: #1c1c1e; color: #f5f5f7; }
        #settingsDialog QLabel, #settingsDialog QCheckBox {
            background: transparent; color: #f5f5f7;
        }
        #settingsScroll, #settingsViewport, #settingsContent {
            background: #1c1c1e; border: none;
        }
        #headerCard { background: #1c1c1e; border: 1px solid #343438;
                      border-radius: 14px; }
        QGroupBox { background: #1c1c1e; border: 1px solid #343438;
                    border-radius: 14px; margin-top: 10px;
                    padding: 8px 8px 6px; }
        QGroupBox::title { subcontrol-origin: margin; left: 12px;
                           padding: 0 7px; color: #aeb0b8;
                           background: #0f1115; }
        QPushButton { background: #2c2c2e; border: 1px solid #48484a;
                      border-radius: 10px; padding: 8px 12px;
                      color: #f5f5f7; }
        QPushButton:hover { background: #3a3a3c; border-color: #636366; }
        QPushButton:pressed { background: #1f1f21; }
        QPushButton:disabled { background: #202023; color: #636366;
                               border-color: #343438; }
        QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
            background: #1c1c1e; border: 1px solid #3a3a3c;
            border-radius: 9px; padding: 7px 9px; color: #f5f5f7;
        }
        QLineEdit:focus, QComboBox:focus, QSpinBox:focus,
        QDoubleSpinBox:focus { border-color: #0a84ff; }
        QTabWidget::pane { border: 1px solid #343438; border-radius: 12px; }
        QTabBar::tab { background: #1c1c1e; border: 1px solid #343438;
                       padding: 10px 14px; margin-right: 3px; }
        QTabBar::tab:selected { background: #2c2c2e; color: #f5f5f7;
                                border-color: #0a84ff; }
        QPushButton[liveAction="true"] { border-color: #ff9f0a; }
        QPushButton[unavailable="true"] { color: #8e8e93;
                                           border-color: #48484a; }
        #stopButton { background: #5a3211; border-color: #ff9f0a;
                      font-weight: bold; }
        #killButton { background: #5a2025; border-color: #ff453a;
                      font-weight: bold; }
        #appTitle { color: #f5f5f7; letter-spacing: 1px; }
        #brandSubtitle { color: #8e8e93; font-size: 9pt; letter-spacing: 1px; }
        #modeLabel { color: #ff9f0a; background: #2c2c2e;
                     border: 1px solid #48484a; border-radius: 10px;
                     font-weight: bold; padding: 8px 12px; }
        #headerAction { min-width: 42px; }
        #statusLabel { padding: 8px 12px; background: #232326;
                       border: 1px solid #343438; border-radius: 10px; }
        #activeLabel { color: #aeb0b8; padding: 4px 6px; }
        #exhibitionTitle { color: #f5f5f7; letter-spacing: 1px; }
        #hint { color: #8e8e93; padding: 4px 6px; }
        #warning { color: #ff9f0a; padding: 8px; }
        #cardText { color: #d1d1d6; padding: 10px; }
        #operatorInstruction { background: #1c1c1e; color: #aeb0b8;
                               border: 1px solid #343438; border-radius: 10px;
                               padding: 6px 10px; font-size: 10pt; }
        #footerStatus { background: #1c1c1e; color: #d1d1d6;
                        border: 1px solid #343438; border-radius: 10px;
                        padding: 6px 10px; }
        #controllerHint { background: #232326; color: #aeb0b8;
                          border-radius: 9px; padding: 6px 10px;
                          font-size: 9.5pt; }
        #videoCanvas { background: #000000; border: 1px solid #3a3a3c;
                       border-radius: 10px; color: #8e8e93; }
        #videoGroup::title { subcontrol-origin: margin;
                             subcontrol-position: top center; padding: 0 10px; }
        #videoStatus { background: #1c1c1e; color: #64d2ff;
                       border-radius: 8px; padding: 7px; }
        #robotName { padding: 8px; color: #d1d1d6; }
        #robotName[connectionState="searching"] { color: #ff9f0a; }
        #robotName[connectionState="connected"] { color: #30d158; }
        #deviceGroup, #modeGroup, #statusGroup { border-color: #343438; }
        #safetyGroup { border-color: #5a3035; }
        #connectButton { background: #0a84ff; border-color: #64b5ff;
                         font-weight: bold; }
        #connectButton:hover { background: #409cff; }
        #staticModeButton { background: #2c2c2e; border-color: #636366;
                            font-weight: bold; }
        #staticModeButton:hover { background: #3a3a3c; }
        #controlModeButton { background: #0a84ff; border-color: #64b5ff;
                             font-weight: bold; }
        #controlModeButton:hover { background: #409cff; }
        #standModeButton { background: #2c2c2e; border-color: #636366;
                           font-weight: bold; }
        #standModeButton:hover { background: #3a3a3c; }
        #reconnectButton { background: #2c2c2e; border-color: #636366;
                           font-weight: bold; }
        #openPovButton { background: #2c2c2e; border-color: #48484a; }
        #zeroTorqueButton { background: #5a2025; border-color: #ff453a;
                            color: white; font-weight: bold; }
        #zeroTorqueButton:hover { background: #733039; }
        #modeHint, #safetyHint { color: #8e8e93; font-size: 9.5pt;
                                 padding: 2px 4px; }
        #simpleStatus { background: #232326; color: #d1d1d6;
                        border: 1px solid #3a3a3c; border-radius: 8px;
                        padding: 4px 8px; }
        #simpleStatus[statusRole="timing"] { color: #64d2ff; }
        QPlainTextEdit { background: #1c1c1e; color: #d1d1d6;
                         border: 1px solid #343438; border-radius: 10px; }
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
    else:
        # Automatic startup remains read-only: bridge/video, battery, voice
        # discovery and preflight. Physical motion begins only from STATIC/RUN.
        QTimer.singleShot(250, panel.auto_connect)
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
