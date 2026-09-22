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


def command_catalog() -> List[CommandSpec]:
    """Return all actions exposed by the panel.

    Targets are existing Makefile targets.  Keeping this list declarative
    makes it easy to show an honest "not implemented" state in the UI.
    """

    return [
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
        CommandSpec("prepare", "Подготовить робота", "robot-prepare", "robot", True),
        CommandSpec("stand", "Включить stand/balance", "robot-prepare", "robot", True),
        CommandSpec("stop", "Остановить робота", "robot-stop", "robot"),
        CommandSpec("kill", "Аварийная остановка", "robot-kill", "robot"),
        CommandSpec(
            "reset_kill",
            "Снять аварийную остановку",
            category="robot",
            implemented=False,
            note="В проекте нет безопасной отдельной команды снятия kill latch.",
        ),
        CommandSpec(
            "vr_calibrate",
            "Калибровать VR",
            category="vr",
            implemented=False,
            note="Калибровка выполняется внутри активного ROS/VR-сеанса.",
        ),
        CommandSpec(
            "hands_calibrate",
            "Калибровать руки",
            category="vr",
            implemented=False,
            note="Сервис калибровки доступен только в запущенном ROS-сеансе.",
        ),
        CommandSpec("dry_run", "Запустить VR dry-run", "teleop-dry-run", "vr", long_running=True),
        CommandSpec("arms_live", "Повторять движения рук из VR", "arms-live", "arms", True, True),
        CommandSpec("legs_live", "Включить движение ног", "legs-live", "legs", True, True),
        CommandSpec("teleop_dry_run", "Полный VR dry-run", "teleop-dry-run", "full", long_running=True),
        CommandSpec("teleop_live", "Полный VR teleop", "teleop-live", "full", True, True),
        CommandSpec("teleop_stop", "Остановить VR teleop", "robot-stop", "full"),
        CommandSpec("video_robot", "Запустить видео глазами робота", "pov-vr", "video", long_running=True),
        CommandSpec("video_vr", "VR-вид от лица робота", "pov-vr", "video", long_running=True),
        CommandSpec("video_stereo", "Stereo video (демо)", "pov-stereo-demo", "video", long_running=True),
        CommandSpec("video_mono", "Mono video", "pov-vr", "video", long_running=True),
        CommandSpec("video_low", "Профиль: низкая задержка", "pov-low-latency", "video", long_running=True),
        CommandSpec("video_high", "Профиль: высокое качество", "pov-high-quality", "video", long_running=True),
        CommandSpec("video_exhibition", "Профиль: выставка", "pov-exhibition", "video", long_running=True),
        CommandSpec("video_preflight", "Проверить видеопуть", "pov-preflight", "video"),
    ]


def spec_by_key(key: str) -> CommandSpec:
    for spec in command_catalog():
        if spec.key == key:
            return spec
    raise KeyError(key)


def required_make_targets() -> Sequence[str]:
    return tuple(spec.target for spec in command_catalog() if spec.target)
