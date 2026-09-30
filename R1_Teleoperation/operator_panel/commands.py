"""Command catalogue used by the Russian operator panel."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence


@dataclass(frozen=True)
class CommandSpec:
    key: str
    title: str
    target: Optional[str] = None
    category: str = "diagnostic"
    live: bool = False
    long_running: bool = False
    implemented: bool = True
    note: str = ""
    requires_run_mode: bool = False


def command_catalog() -> List[CommandSpec]:
    """
    Return all actions exposed by the panel.

    Targets are existing Makefile targets.  Keeping this list declarative
    makes it easy to show an honest "not implemented" state in the UI.
    """
    return [
        CommandSpec(
            "exhibition_static",
            "Статичный режим",
            "exhibition-static",
            "exhibition",
            True,
            True,
            note="Автоматическая проверка, устойчивый режим и Robot POV без управления.",
        ),
        CommandSpec(
            "exhibition_stand",
            "Стойка с удержанием моторами",
            "exhibition-static",
            "exhibition",
            True,
            True,
            note=(
                "Штатная стойка без VR и команд ходьбы. "
                "Моторы удерживают позу; отключение внутренней стабилизации "
                "прошивки не обещается."
            ),
        ),
        CommandSpec(
            "exhibition_control",
            "Режим управления",
            "exhibition-control",
            "exhibition",
            True,
            True,
            note=(
                "Полный VR-режим: голова, руки и ноги. Использует штатный "
                "канал виртуального пульта и автоматически включает Run/FSM 811."
            ),
            requires_run_mode=True,
        ),
        CommandSpec(
            "exhibition_reconnect",
            "Переподключить всё",
            "exhibition-reconnect",
            "exhibition_service",
        ),
        CommandSpec(
            "exhibition_rearm",
            "Повторно запустить RUN",
            "exhibition-rearm",
            "exhibition_service",
            True,
            note="Повторно калибрует и подготавливает уже работающий граф после KILL.",
            requires_run_mode=True,
        ),
        CommandSpec(
            "exhibition_lock",
            "Быстрый LOCK",
            "exhibition-lock",
            "exhibition_service",
            note=(
                "Мгновенно обнуляет ноги и переводит уже работающий "
                "выставочный control graph в удержание, не перезапуская SDK."
            ),
        ),
        CommandSpec(
            "exhibition_calibrate",
            "Перекалибровать руки",
            "exhibition-calibrate",
            "exhibition_service",
        ),
        CommandSpec(
            "exhibition_stop",
            "Остановить выставочный режим",
            "exhibition-stop",
            "robot",
        ),
        CommandSpec("check_all", "Проверить всё", category="diagnostic"),
        CommandSpec("network", "Проверить сеть", "r1-lan-preflight"),
        CommandSpec("ethernet", "Проверить Ethernet к роботу", "r1-camera-preflight"),
        CommandSpec("robot_check", "Проверить робота", "robot-preflight"),
        CommandSpec("vr_check", "Проверить VR-шлем", "r1-teleoperation-status"),
        CommandSpec("controllers_check", "Проверить контроллеры", "r1-teleoperation-status"),
        CommandSpec("camera_check", "Проверить камеры", "r1-camera-preflight"),
        CommandSpec("video_check", "Проверить видео", "pov-preflight"),
        CommandSpec("ros_check", "Проверить ROS2", "r1-teleoperation-status"),
        CommandSpec("sdk_check", "Проверить Unitree SDK", "r1-sdk-readonly"),
        CommandSpec("writer_check", "Диагностика writer/feedback", "r1-arm-sdk-traffic-check"),
        CommandSpec("voice_check", "Проверить голос Добрыни", "r1-voice-preflight"),
        CommandSpec(
            "voice_remote",
            "Подключить голос Добрыни",
            "r1-voice-ensure",
            "voice",
        ),
        CommandSpec(
            "connection_ensure",
            "Автоматическое подключение",
            "r1-connection-ensure",
            "connection",
            long_running=True,
            note="Запускает только offline bridge и Robot POV, без команд роботу.",
        ),
        CommandSpec(
            "sdk_warmup",
            "Фоновая read-only проверка SDK",
            "r1-sdk-warmup",
            "connection",
            long_running=True,
            note="Заранее проверяет feedback без команд роботу для быстрого RUN.",
        ),
        CommandSpec(
            "battery_monitor",
            "Монитор батареи",
            "r1-battery-monitor",
            "connection",
            long_running=True,
        ),
        CommandSpec(
            "command_capture",
            "Запись кнопки Unitree Explore",
            category="diagnostic",
            long_running=True,
            note="Read-only запись DDS-запросов и виртуального пульта.",
        ),
        CommandSpec("panel_status", "Снимок realtime-статуса", "r1-panel-status"),
        CommandSpec(
            "offline_session",
            "Запустить offline bridge + POV",
            "r1-offline-session",
            "video",
            long_running=True,
        ),
        CommandSpec("prepare", "Подготовить робота", "robot-prepare", "robot", True),
        CommandSpec("stand", "Включить stand/balance", "robot-prepare", "robot", True),
        CommandSpec("stop", "Остановить робота", "robot-stop", "robot"),
        CommandSpec("kill", "Аварийная остановка", "robot-kill", "robot"),
        CommandSpec(
            "zero_torque",
            "Расслабить робота (Zero Torque)",
            "robot-zero-torque",
            "robot",
            note=(
                "Завершает управление, подтверждает Damping/FSM 1 и затем "
                "отключает удерживающий момент через Zero Torque/FSM 0."
            ),
        ),
        CommandSpec(
            "reset_kill",
            "Снять аварийную остановку",
            "exhibition-rearm",
            category="robot",
            live=True,
            note=(
                "Повторяет проверенный RUN/re-arm: очищает VR emergency latch, "
                "калибрует и снимает KILL только после robot-prepare."
            ),
            requires_run_mode=True,
        ),
        CommandSpec(
            "vr_calibrate",
            "Калибровать HMD и руки",
            "arms-calibrate",
            category="calibration",
            note="Требует активный arm VR-сеанс и отпущенный Deadman.",
        ),
        CommandSpec(
            "head_calibrate",
            "Калибровать голову",
            "head-calibrate",
            category="calibration",
            note="Сохраняет текущую нейтраль HMD через локальный dry/live head-сеанс.",
        ),
        CommandSpec(
            "hands_calibrate",
            "Калибровать руки",
            "arms-calibrate",
            category="calibration",
            note="Требует активный arm VR-сеанс и отпущенный Deadman.",
        ),
        CommandSpec(
            "dry_run",
            "Запустить VR dry-run",
            "teleop-dry-run",
            "vr",
            long_running=True,
        ),
        CommandSpec(
            "arms_live",
            "Повторять движения рук из VR",
            "arms-live",
            "arms",
            True,
            True,
        ),
        CommandSpec(
            "arms_running_live",
            "Руки + Running (без стиков)",
            "arms-running-live",
            "arms",
            True,
            True,
        ),
        CommandSpec(
            "legs_live",
            "Включить движение ног",
            "legs-live",
            "legs",
            True,
            True,
            note=(
                "Использует штатный канал виртуального пульта Unitree и "
                "автоматически включает Run/FSM 811."
            ),
            requires_run_mode=True,
        ),
        CommandSpec(
            "teleop_dry_run",
            "Полный VR dry-run",
            "teleop-dry-run",
            "full",
            long_running=True,
        ),
        CommandSpec(
            "teleop_live",
            "Полный VR teleop",
            "teleop-live",
            "full",
            True,
            True,
            note=(
                "Полный VR-режим: голова, руки и ноги."
            ),
            requires_run_mode=True,
        ),
        CommandSpec("teleop_stop", "Остановить VR teleop", "robot-stop", "full"),
        CommandSpec(
            "video_robot",
            "Запустить видео глазами робота",
            "pov-vr",
            "video",
            long_running=True,
        ),
        CommandSpec(
            "video_vr",
            "VR-вид от лица робота",
            "pov-vr",
            "video",
            long_running=True,
        ),
        CommandSpec(
            "video_stereo",
            "Stereo video (демо)",
            "pov-stereo-demo",
            "video",
            long_running=True,
        ),
        CommandSpec(
            "video_mono", "Mono video", "pov-vr", "video", long_running=True
        ),
        CommandSpec(
            "video_low",
            "Профиль: низкая задержка",
            "pov-low-latency",
            "video",
            long_running=True,
        ),
        CommandSpec(
            "video_high",
            "Профиль: высокое качество",
            "pov-high-quality",
            "video",
            long_running=True,
        ),
        CommandSpec(
            "video_exhibition",
            "Профиль: выставка",
            "pov-exhibition",
            "video",
            long_running=True,
        ),
        CommandSpec("video_preflight", "Проверить видеопуть", "pov-preflight", "video"),
    ]


def spec_by_key(key: str) -> CommandSpec:
    for spec in command_catalog():
        if spec.key == key:
            return spec
    raise KeyError(key)


def required_make_targets() -> Sequence[str]:
    return tuple(spec.target for spec in command_catalog() if spec.target)
