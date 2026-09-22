"""Configuration for the Unitree R1 operator panel.

The panel deliberately keeps the default in dry-run mode.  Live operation is
an explicit configuration choice and still has to pass the project's existing
fail-closed scripts and safety gates.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Any, Dict, Optional


PROJECT_DIR = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_DIR / "config" / "operator_panel.json"


@dataclass
class OperatorConfig:
    project_dir: str = str(PROJECT_DIR)
    robot_ip: str = "192.168.123.161"
    pc2_ip: str = "192.168.123.164"
    robot_interface: str = "enxb4b024be59fe"
    laptop_robot_ip: str = "192.168.123.162"
    ros_domain_id: int = 89
    vr_headset_ip: str = ""
    video_url: str = "http://192.168.8.9:8080/"
    logs_dir: str = "logs"
    dry_run: bool = True
    allow_live: bool = False
    video_profile: str = "low-latency"
    locomotion_profile: str = "slow-safe"

    def as_environment(self) -> Dict[str, str]:
        """Return project environment overrides used by child processes."""

        env = {
            "R1_OPERATOR_PANEL": "1",
            # The R1 LAN preflight calls the robot's PC2 host "robot_ip",
            # while the SDK/DDS and videohub use the native control address.
            "R1_ROBOT_IP": self.pc2_ip,
            "R1_CONTROL_IP": self.robot_ip,
            "R1_ROBOT_INTERFACE": self.robot_interface,
            "R1_CAMERA_INTERFACE": self.robot_interface,
            "R1_LAPTOP_ROBOT_IP": self.laptop_robot_ip,
            "R1_PC2_IP": self.pc2_ip,
            "R1_TELEOP_HARDWARE_DOMAIN_ID": str(self.ros_domain_id),
            "R1_TELEOP_DOMAIN_ID": str(self.ros_domain_id),
            "R1_LIVE_ROS_DOMAIN_ID": str(self.ros_domain_id),
            "ROS_DOMAIN_ID": str(self.ros_domain_id),
            "R1_LIVE_NETWORK_INTERFACE": self.robot_interface,
            "ROBOT_POV_UNITREE_INTERFACE": self.robot_interface,
            "ROBOT_POV_ROBOT_IP": self.robot_ip,
            "ROBOT_POV_PROFILE": self.video_profile,
            "R1_LOCOMOTION_PROFILE": self.locomotion_profile,
        }
        if self.vr_headset_ip.strip():
            env["ROBOT_VR_SOURCE_IP"] = self.vr_headset_ip.strip()
            env["R1_DRY_RUN_VR_ALLOWED_SOURCE_IP"] = self.vr_headset_ip.strip()
        # These overrides are the panel's safe default.  They are intentionally
        # not set to live acknowledgements; the live wrappers must remain the
        # authority for physical commissioning.
        if self.dry_run:
            env.update(
                {
                    "ROBOT_DRY_RUN": "1",
                    "ROBOT_ENABLE_ACTUATION": "0",
                    "ROBOT_CONFIRM_OFF_CHARGER": "0",
                    "ROBOT_CONFIRM_CLEAR_AREA": "0",
                    "ROBOT_CONFIRM_ESTOP_READY": "0",
                    "ROBOT_CONFIRM_COMMISSIONING": "0",
                }
            )
        return env


def _merge_values(data: Dict[str, Any]) -> OperatorConfig:
    allowed = {field.name for field in fields(OperatorConfig)}
    values = {key: value for key, value in data.items() if key in allowed}
    config = OperatorConfig(**values)
    config.project_dir = str(Path(config.project_dir).expanduser())
    config.ros_domain_id = int(config.ros_domain_id)
    return config


def load_config(path: Optional[Path] = None) -> OperatorConfig:
    """Load JSON configuration, falling back to safe defaults."""

    path = Path(path or CONFIG_PATH).expanduser()
    if not path.is_file():
        return OperatorConfig()
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("configuration root must be an object")
        return _merge_values(data)
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        # A malformed operator config must never prevent the panel from opening
        # in its safe defaults.
        return OperatorConfig()


def save_config(config: OperatorConfig, path: Optional[Path] = None) -> Path:
    """Atomically save configuration and return the destination path."""

    path = Path(path or CONFIG_PATH).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(asdict(config), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path
