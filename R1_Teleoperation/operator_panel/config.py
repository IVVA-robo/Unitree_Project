"""Configuration for the Unitree R1 operator panel.

The panel deliberately keeps the default in dry-run mode.  Live operation is
an explicit configuration choice and still has to pass the project's existing
fail-closed scripts and safety gates.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Any, Dict, Optional


PROJECT_DIR = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_DIR / "config" / "operator_panel.json"


def resolve_robot_interface(preferred: str) -> str:
    """Return a currently present Ethernet interface, tolerating USB renames."""
    requested = str(preferred or "").strip()
    net_root = Path("/sys/class/net")
    if requested and requested.lower() != "auto" and (net_root / requested).exists():
        return requested
    candidates = []
    try:
        names = sorted(item.name for item in net_root.iterdir())
    except OSError:
        names = []
    for name in names:
        if name == "lo" or not (name.startswith("en") or name.startswith("eth")):
            continue
        path = net_root / name
        try:
            carrier = (path / "carrier").read_text().strip() == "1"
        except OSError:
            carrier = False
        try:
            up = (path / "operstate").read_text().strip() == "up"
        except OSError:
            up = False
        candidates.append((not (carrier and up), name))
    if candidates:
        candidates.sort()
        return candidates[0][1]
    return requested or "auto"


# The panel exposes only the small exhibition tuning envelope reviewed for the
# current R1 setup.  These are operator ergonomics, not replacements for the
# downstream joint/workspace and slow-safe velocity limits.
TUNING_RANGES = {
    "shoulder_height_offset_m": (-0.15, 0.15),
    "shoulder_forward_offset_m": (-0.12, 0.08),
    "shoulder_width_m": (0.25, 0.55),
    "arm_motion_scale": (0.50, 1.20),
    "turn_sensitivity": (0.10, 1.00),
    "leg_speed_scale": (0.10, 1.00),
}


def _finite_in_range(name: str, value: Any) -> float:
    """Convert one tuning value and reject non-finite/out-of-envelope data."""

    result = float(value)
    lower, upper = TUNING_RANGES[name]
    if not math.isfinite(result) or not lower <= result <= upper:
        raise ValueError(f"{name} must be finite and within [{lower}, {upper}]")
    return result


@dataclass
class OperatorConfig:
    project_dir: str = str(PROJECT_DIR)
    robot_name: str = "R1_03079"
    robot_ip: str = "192.168.123.161"
    pc2_ip: str = "192.168.123.164"
    robot_interface: str = "enxb4b024be59fe"
    # The current TP-Link USB 10/100 adapter identifies as cdc_ether and
    # reports a synthetic half-duplex value. The SDK preflight accepts this
    # opt-in only for that exact driver and still requires clean counters,
    # ping, LowState, motor health, and all normal commissioning gates.
    allow_half_duplex_adapter: bool = True
    laptop_robot_ip: str = "192.168.123.162"
    # Physical/live ROS graph.  The laptop-only dry-run wrapper deliberately
    # keeps its own default (89), so the panel must point live actions at the
    # shared commissioning graph used by r1-live-session (88).
    ros_domain_id: int = 88
    vr_headset_ip: str = ""
    vr_transport: str = "lan"
    # Local desktop viewer.  A headset must use the numeric LAN URL printed by
    # Robot POV, since 127.0.0.1 inside the headset is the headset itself.
    video_url: str = "http://127.0.0.1:8080/"
    logs_dir: str = "logs"
    dry_run: bool = True
    allow_live: bool = False
    video_profile: str = "low-latency"
    locomotion_profile: str = "slow-safe"
    require_preflight: bool = True
    # The read-only video path is safe to start as soon as the panel opens;
    # motion remains behind the explicit RUN action and its existing gates.
    auto_start_video: bool = True
    auto_start_bridge: bool = True
    status_poll_sec: float = 5.0
    response_profile: str = "exhibition"
    # Shoulder geometry is captured into the next body calibration.  Motion
    # scale applies to every mapped hand delta without rewriting calibration.
    shoulder_height_offset_m: float = 0.0
    shoulder_forward_offset_m: float = -0.02
    shoulder_width_m: float = 0.40
    arm_motion_scale: float = 0.50
    # Multipliers may only reduce the immutable slow-safe locomotion ceilings.
    turn_sensitivity: float = 1.0
    leg_speed_scale: float = 1.0

    def validate_tuning(self) -> None:
        """Fail closed before invalid UI/JSON tuning reaches a child process."""

        for name in TUNING_RANGES:
            _finite_in_range(name, getattr(self, name))
        if self.response_profile not in {"standard", "exhibition"}:
            raise ValueError("response_profile must be standard or exhibition")

    def as_environment(self) -> Dict[str, str]:
        """Return project environment overrides used by child processes."""

        self.validate_tuning()
        if self.vr_transport not in {"lan", "usb"}:
            raise ValueError("vr_transport must be lan or usb")

        env = {
            "R1_VR_TRANSPORT": self.vr_transport,
            "R1_RESPONSE_PROFILE": self.response_profile,
            "R1_OPERATOR_PANEL": "1",
            # The R1 LAN preflight calls the robot's PC2 host "robot_ip",
            # while the SDK/DDS and videohub use the native control address.
            "R1_ROBOT_IP": self.pc2_ip,
            "R1_CONTROL_IP": self.robot_ip,
            "R1_LIVE_CONTROL_IP": self.robot_ip,
            # The control computer and PC2 are different hosts. Falling back
            # to PC2 makes a cable look like a healthy motion/camera endpoint
            # even though the R1 services are absent.
            "R1_CONTROL_IP_CANDIDATES": self.robot_ip,
            "R1_ROBOT_INTERFACE": resolve_robot_interface(self.robot_interface),
            "R1_CAMERA_INTERFACE": resolve_robot_interface(self.robot_interface),
            "R1_LAPTOP_ROBOT_IP": self.laptop_robot_ip,
            "R1_PC2_IP": self.pc2_ip,
            "R1_TELEOP_HARDWARE_DOMAIN_ID": str(self.ros_domain_id),
            "R1_TELEOP_DOMAIN_ID": str(self.ros_domain_id),
            "R1_LIVE_ROS_DOMAIN_ID": str(self.ros_domain_id),
            "ROS_DOMAIN_ID": str(self.ros_domain_id),
            "ROS_LOCALHOST_ONLY": "1",
            "R1_LIVE_NETWORK_INTERFACE": resolve_robot_interface(self.robot_interface),
            "R1_SDK_ALLOW_HALF_DUPLEX": (
                "1" if self.allow_half_duplex_adapter else "0"
            ),
            "ROBOT_POV_UNITREE_INTERFACE": self.robot_interface,
            # This address is a diagnostic ping target. The camera client
            # discovers videohub through the DDS interface/domain above.
            "ROBOT_POV_ROBOT_IP": self.robot_ip,
            "ROBOT_POV_ROBOT_IP_CANDIDATES": self.robot_ip,
            "ROBOT_POV_PROFILE": self.video_profile,
            "R1_LOCOMOTION_PROFILE": self.locomotion_profile,
            "R1_OPERATOR_REQUIRE_PREFLIGHT": "1" if self.require_preflight else "0",
            "R1_OPERATOR_AUTOSTART_VIDEO": "1" if self.auto_start_video else "0",
            "R1_OPERATOR_AUTOSTART_BRIDGE": "1" if self.auto_start_bridge else "0",
            "R1_SHOULDER_HEIGHT_OFFSET_M": format(
                self.shoulder_height_offset_m, ".6g"
            ),
            "R1_SHOULDER_FORWARD_OFFSET_M": format(
                self.shoulder_forward_offset_m, ".6g"
            ),
            "R1_SHOULDER_WIDTH_M": format(self.shoulder_width_m, ".6g"),
            "R1_ARM_MOTION_SCALE": format(self.arm_motion_scale, ".6g"),
            "R1_TURN_SENSITIVITY": format(self.turn_sensitivity, ".6g"),
            "R1_LEG_SPEED_SCALE": format(self.leg_speed_scale, ".6g"),
        }
        if self.vr_transport == "usb":
            env["ROBOT_VR_SOURCE_IP"] = "127.0.0.1"
            env["R1_DRY_RUN_VR_ALLOWED_SOURCE_IP"] = "127.0.0.1"
        elif self.vr_headset_ip.strip():
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
    boolean_fields = (
        "dry_run",
        "allow_live",
        "allow_half_duplex_adapter",
        "require_preflight",
        "auto_start_video",
        "auto_start_bridge",
    )
    for name in boolean_fields:
        value = getattr(config, name)
        if isinstance(value, str):
            setattr(config, name, value.strip().lower() in {"1", "true", "yes", "on"})
    config.status_poll_sec = max(1.0, float(config.status_poll_sec))
    for name in TUNING_RANGES:
        setattr(config, name, _finite_in_range(name, getattr(config, name)))
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

    config.validate_tuning()
    path = Path(path or CONFIG_PATH).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(asdict(config), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)
    return path
