"""Short-lived local acknowledgement for one supervised live work period."""

from __future__ import annotations

import json
import os
from pathlib import Path
import time
from typing import Mapping, Optional

from .config import OperatorConfig


LIVE_ACK_MAX_AGE_SEC = 30 * 60


def acknowledgement_path(
    environment: Optional[Mapping[str, str]] = None,
) -> Path:
    environment = os.environ if environment is None else environment
    runtime = Path(
        environment.get(
            "XDG_RUNTIME_DIR", f"/tmp/r1-operator-panel-{os.getuid()}"
        )
    )
    return runtime / "r1-operator-panel" / "live-ack.json"


def _identity(config: OperatorConfig) -> dict[str, str]:
    return {
        "robot_ip": config.robot_ip.strip(),
        "robot_interface": config.robot_interface.strip(),
    }


def acknowledgement_valid(
    config: OperatorConfig,
    *,
    path: Optional[Path] = None,
    now: Optional[float] = None,
) -> bool:
    destination = path or acknowledgement_path()
    current = time.time() if now is None else now
    try:
        payload = json.loads(destination.read_text(encoding="utf-8"))
        issued_at = float(payload["issued_at"])
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        return False
    return (
        payload.get("schema") == 1
        and payload.get("identity") == _identity(config)
        and 0.0 <= current - issued_at <= LIVE_ACK_MAX_AGE_SEC
    )


def record_acknowledgement(
    config: OperatorConfig,
    *,
    path: Optional[Path] = None,
    now: Optional[float] = None,
) -> Path:
    destination = path or acknowledgement_path()
    destination.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(destination.parent, 0o700)
    payload = {
        "schema": 1,
        "issued_at": time.time() if now is None else now,
        "identity": _identity(config),
    }
    temporary = destination.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.chmod(temporary, 0o600)
    os.replace(temporary, destination)
    return destination


def clear_acknowledgement(*, path: Optional[Path] = None) -> None:
    try:
        (path or acknowledgement_path()).unlink()
    except FileNotFoundError:
        pass

