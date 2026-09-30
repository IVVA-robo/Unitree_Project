"""
Long-running, fail-closed exhibition process manager.

This module deliberately stays above the ROS/SDK safety boundary.  It starts
the already reviewed Make targets, supervises the video-only POV process, and
records non-secret status for the operator panel.  It never publishes a ROS
message, constructs a Unitree client, fabricates a Deadman sample, or clears a
KILL latch by itself.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Dict, Iterable, List, Mapping, Optional, Sequence

from .process_owner import local_control_process_exists, owner_alive, parent_usb_owner
from .actions import run_helper


PROJECT_DIR = Path(__file__).resolve().parents[1]


def _env_bool(
    environment: Mapping[str, str], name: str, default: bool
) -> bool:
    raw = environment.get(name)
    if raw is None:
        return default
    normalized = raw.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{name} must be a boolean value")


def _positive_float(
    environment: Mapping[str, str], name: str, default: float, maximum: float
) -> float:
    try:
        value = float(environment.get(name, str(default)))
    except ValueError as exception:
        raise ValueError(f"{name} must be numeric") from exception
    if not 0.0 < value <= maximum:
        raise ValueError(f"{name} must be in (0, {maximum}]")
    return value


def _nonnegative_int(
    environment: Mapping[str, str], name: str, default: int, maximum: int
) -> int:
    try:
        value = int(environment.get(name, str(default)))
    except ValueError as exception:
        raise ValueError(f"{name} must be an integer") from exception
    if not 0 <= value <= maximum:
        raise ValueError(f"{name} must be in [0, {maximum}]")
    return value


def _positive_int(
    environment: Mapping[str, str], name: str, default: int, maximum: int
) -> int:
    try:
        value = int(environment.get(name, str(default)))
    except ValueError as exception:
        raise ValueError(f"{name} must be an integer") from exception
    if not 1 <= value <= maximum:
        raise ValueError(f"{name} must be in [1, {maximum}]")
    return value


def _command(
    environment: Mapping[str, str], name: str, default: Sequence[str]
) -> List[str]:
    """Read one argv override without ever invoking a command shell."""
    raw = environment.get(name, "").strip()
    value = shlex.split(raw) if raw else list(default)
    if not value or any("\x00" in item for item in value):
        raise ValueError(f"{name} must contain a valid command")
    return value


@dataclass(frozen=True)
class ExhibitionSettings:
    """Validated process and retry settings for one exhibition session."""

    project_dir: Path
    runtime_dir: Path
    log_dir: Path
    mock: bool
    pov_restart_limit: int
    restart_initial_sec: float
    restart_max_sec: float
    ready_timeout_sec: int
    stop_wait_sec: int
    preflight_timeout_sec: float
    calibrate_on_start: bool
    session_mode: str
    tracking_grace_sec: float
    recovery_blend_sec: float
    commands: Mapping[str, Sequence[str]]

    @classmethod
    def from_environment(
        cls, environment: Optional[Mapping[str, str]] = None
    ) -> "ExhibitionSettings":
        """Load and validate one complete exhibition configuration."""
        environment = dict(os.environ if environment is None else environment)
        project_dir = Path(
            environment.get("R1_EXHIBITION_PROJECT_DIR", str(PROJECT_DIR))
        ).expanduser().resolve()
        runtime_default = Path(
            environment.get(
                "XDG_RUNTIME_DIR", f"/tmp/r1-exhibition-{os.getuid()}"
            )
        )
        # XDG_RUNTIME_DIR normally points at the user's general runtime
        # folder; use a private child so state cannot collide with another
        # application.
        if "R1_EXHIBITION_RUNTIME_DIR" in environment:
            runtime_dir = Path(environment["R1_EXHIBITION_RUNTIME_DIR"])
        elif "XDG_RUNTIME_DIR" in environment:
            runtime_dir = runtime_default / "r1-exhibition"
        else:
            runtime_dir = runtime_default
        runtime_dir = runtime_dir.expanduser().resolve()
        log_dir = Path(
            environment.get(
                "R1_EXHIBITION_LOG_DIR",
                str(project_dir / "logs" / "exhibition"),
            )
        ).expanduser().resolve()
        mock = _env_bool(environment, "R1_EXHIBITION_MOCK", False)
        session_mode = environment.get(
            "R1_EXHIBITION_SESSION_MODE", "session_arm"
        ).strip().lower()
        if session_mode not in {"deadman", "session_arm"}:
            raise ValueError(
                "R1_EXHIBITION_SESSION_MODE must be deadman or session_arm"
            )
        tracking_grace_sec = _positive_float(
            environment, "R1_EXHIBITION_TRACKING_GRACE_SEC", 2.0, 3.0
        )
        if tracking_grace_sec < 1.0:
            raise ValueError(
                "R1_EXHIBITION_TRACKING_GRACE_SEC must be in [1, 3]"
            )
        recovery_blend_sec = _positive_float(
            environment, "R1_EXHIBITION_RECOVERY_BLEND_SEC", 0.50, 2.0
        )
        if recovery_blend_sec < 0.1:
            raise ValueError(
                "R1_EXHIBITION_RECOVERY_BLEND_SEC must be in [0.1, 2]"
            )

        make = ["/usr/bin/make", "--no-print-directory"]
        mock_pov_port = "18080"
        mock_vr_port = "19090"
        mock_discovery_port = "19091"
        # The always-on offline service owns the normal viewer port (8080).
        # A manual mock exhibition session must remain side-effect free, so it
        # uses a separate local viewer instead of stopping that real service.
        pov_command = (
            [
                str(project_dir / "scripts" / "robot-pov"),
                "mock",
                "--profile",
                "balanced",
                "--port",
                mock_pov_port,
            ]
            if mock
            else [
                "/usr/bin/env",
                # The live VR bridge is the sole owner of UDP 9091.  The
                # video process remains active for the headset, but must not
                # create a second discovery responder during RUN.
                "ROBOT_POV_DISCOVERY_ENABLED=false",
                *make,
                "pov-vr",
            ]
        )
        preflight_command = (
            [
                str(project_dir / "scripts" / "r1-exhibition-mock-preflight"),
                mock_pov_port,
                mock_vr_port,
                mock_discovery_port,
            ]
            if mock
            else [str(project_dir / "scripts" / "r1-exhibition-preflight")]
        )
        control_command = (
            [
                "/usr/bin/env",
                f"R1_DRY_RUN_UDP_PORT={mock_vr_port}",
                f"R1_DRY_RUN_DISCOVERY_PORT={mock_discovery_port}",
                "R1_DRY_RUN_UDP_BIND_ADDRESS=127.0.0.1",
                *make,
                "teleop-dry-run",
            ]
            if mock
            else [*make, "teleop-live"]
        )
        offline_handoff = str(
            project_dir / "scripts" / "r1-exhibition-offline-handoff"
        )
        ready_helper = str(
            project_dir / "scripts" / "r1-exhibition-wait-ready"
        )
        session_gate = str(
            project_dir / "scripts" / "r1-exhibition-session-gate"
        )
        defaults = {
            "panel": [str(project_dir / "scripts" / "r1-operator-panel")],
            "preflight": preflight_command,
            "pov": pov_command,
            "control": control_command,
            "static_writer": (
                []
                if mock
                else [
                    str(project_dir / "scripts" / "r1-live-session"),
                    "static",
                ]
            ),
            "ready_static": [] if mock else [ready_helper, "static"],
            "ready_control": [] if mock else [ready_helper, "control"],
            "prepare": [] if mock else [*make, "robot-prepare"],
            "stop": [] if mock else [*make, "robot-stop"],
            "kill": [] if mock else [*make, "robot-kill"],
            "calibrate": [
                str(project_dir / "scripts" / (
                    "r1-head-calibrate" if (
                        environment.get("R1_RESPONSE_PROFILE") == "exhibition"
                        and (project_dir / "config/operator_arm_range.exhibition.json").is_file()
                    ) else "r1-exhibition-calibrate"))
            ],
            # The installed offline service owns the same bridge/video ports
            # (UDP 9090 and TCP 8080) as an exhibition graph.  Manage only
            # that exact user unit and restore it after exhibition teardown.
            "offline_status": (
                [] if mock else [offline_handoff, "status"]
            ),
            "offline_stop": [] if mock else [offline_handoff, "stop"],
            "offline_start": [] if mock else [offline_handoff, "start"],
            "arm_session": (
                []
                if mock or session_mode != "session_arm"
                else [session_gate, "arm"]
            ),
            "disarm_session": (
                []
                if mock or session_mode != "session_arm"
                else [session_gate, "disarm"]
            ),
            "pause_session": (
                []
                if mock or session_mode != "session_arm"
                else [session_gate, "pause"]
            ),
            "resume_session": (
                []
                if mock or session_mode != "session_arm"
                else [session_gate, "resume"]
            ),
            "warm_resume": (
                [] if mock or session_mode != "session_arm"
                else [session_gate, "resume_ready"]
            ),
            "clear_emergency": (
                []
                if mock or session_mode != "session_arm"
                else [session_gate, "clear_emergency"]
            ),
        }
        commands: Dict[str, Sequence[str]] = {}
        for key, default in defaults.items():
            variable = f"R1_EXHIBITION_{key.upper()}_COMMAND"
            if not default and not environment.get(variable, "").strip():
                commands[key] = []
            else:
                commands[key] = _command(environment, variable, default)

        return cls(
            project_dir=project_dir,
            runtime_dir=runtime_dir,
            log_dir=log_dir,
            mock=mock,
            pov_restart_limit=_nonnegative_int(
                environment, "R1_EXHIBITION_POV_RESTART_LIMIT", 0, 1000
            ),
            restart_initial_sec=_positive_float(
                environment, "R1_EXHIBITION_RESTART_INITIAL_SEC", 0.5, 30.0
            ),
            restart_max_sec=_positive_float(
                environment, "R1_EXHIBITION_RESTART_MAX_SEC", 8.0, 120.0
            ),
            ready_timeout_sec=_positive_int(
                environment, "R1_EXHIBITION_READY_TIMEOUT_SEC", 90, 120
            ),
            stop_wait_sec=_positive_int(
                environment, "R1_EXHIBITION_STOP_WAIT_SEC", 45, 120
            ),
            preflight_timeout_sec=_positive_float(
                environment, "R1_EXHIBITION_PREFLIGHT_TIMEOUT_SEC", 180.0, 600.0
            ),
            # A physical exhibition control session captures the operator's
            # current neutral before session-arm is enabled.  Calibration is
            # read-only with respect to the robot and prevents a stale saved
            # body pose from becoming the first live arm target.  Hardware-
            # free mock sessions keep their shorter historical sequence unless
            # a test explicitly opts in.
            calibrate_on_start=_env_bool(
                environment, "R1_EXHIBITION_CALIBRATE_ON_START", not mock
            ),
            session_mode=session_mode,
            tracking_grace_sec=tracking_grace_sec,
            recovery_blend_sec=recovery_blend_sec,
            commands=commands,
        )


def build_plan(mode: str, settings: ExhibitionSettings) -> dict:
    """Return an auditable plan containing no environment values or secrets."""
    if mode not in {"static", "control"}:
        raise ValueError("mode must be static or control")
    steps = ["preflight", "pov"]
    if mode == "static":
        steps.extend(["static_writer", "ready_static", "prepare"])
    else:
        steps.extend(["control", "ready_control"])
        if settings.calibrate_on_start:
            steps.append("calibrate")
        steps.extend(["arm_session", "prepare"])
    return {
        "mode": mode,
        "mock": settings.mock,
        "steps": [
            {
                "name": name,
                "argv": list(settings.commands[name]),
                "restart": name == "pov",
            }
            for name in steps
            if settings.commands[name]
        ],
        "physical_profile": (
            "static-stand" if mode == "static" else "slow-safe"
        ),
        "session_mode": (
            "none"
            if mode == "static"
            else settings.session_mode
        ),
        "tracking_grace_sec": settings.tracking_grace_sec,
        "recovery_blend_sec": settings.recovery_blend_sec,
        "live_writer_restart": False,
        "offline_service_policy": "stop-restore",
        "reuse_offline_bridge": False,
        "pov_restart_limit": settings.pov_restart_limit,
    }


class ExhibitionManager:
    """Own one foreground session and supervise safe-to-restart helpers."""

    def __init__(self, settings: ExhibitionSettings, mode: str):
        """Create an idle manager for one validated static/control mode."""
        if mode not in {"static", "control"}:
            raise ValueError("mode must be static or control")
        self.settings = settings
        self.mode = mode
        self.session_id = uuid.uuid4().hex
        self.transport_owner = parent_usb_owner()
        self.children: Dict[str, subprocess.Popen] = {}
        self.stop_requested = False
        self.reconnect_requested = False
        self.lock_requested = False
        self.run_requested = False
        self.pov_restarts = 0
        self._lock_file = None
        self._log_file = None
        self._status = "starting"
        self._detail = "initializing"
        self._log_path: Optional[Path] = None
        self.offline_service_was_active = False
        self.offline_service_stopped = False
        self.offline_service_restored = False
        self._safe_stop_completed = False
        self._safe_stop_confirmed = False

    @property
    def state_path(self) -> Path:
        """Return the private, non-secret runtime status path."""
        return self.settings.runtime_dir / "state.json"

    def _prepare_paths(self) -> None:
        self.settings.runtime_dir.mkdir(
            mode=0o700, parents=True, exist_ok=True
        )
        os.chmod(self.settings.runtime_dir, 0o700)
        self.settings.log_dir.mkdir(mode=0o750, parents=True, exist_ok=True)
        lock_path = self.settings.runtime_dir / "session.lock"
        self._lock_file = lock_path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(
                self._lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB
            )
        except BlockingIOError as exception:
            raise RuntimeError(
                "another exhibition session is already active"
            ) from exception
        if owner_alive(_read_state(self.settings).get("transport_owner")):
            self._lock_file.close()
            self._lock_file = None
            raise RuntimeError("previous USB session is still cleaning up; retry after STOP completes")
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self._log_path = self.settings.log_dir / f"{self.mode}-{stamp}.log"
        self._log_file = self._log_path.open(
            "a", encoding="utf-8", buffering=1
        )

    def _log(self, message: str) -> None:
        timestamp = datetime.now().astimezone().isoformat(timespec="seconds")
        line = f"[{timestamp}] {message}"
        print(line, flush=True)
        if self._log_file:
            print(line, file=self._log_file, flush=True)

    def _write_state(self, status: str, detail: str) -> None:
        self._status = status
        self._detail = detail
        payload = {
            "schema": 1,
            "session_id": self.session_id,
            "pid": os.getpid(),
            "transport_owner": self.transport_owner,
            "mode": self.mode,
            "mock": self.settings.mock,
            "session_mode": self.settings.session_mode,
            "status": status,
            "detail": detail,
            "log": str(self._log_path) if self._log_path else "",
            "pov_restarts": self.pov_restarts,
            # This becomes true only after the reviewed STOP path or its KILL
            # fallback returns success.  Process disappearance by itself is
            # never evidence that the robot accepted either request.
            "safe_stop_confirmed": self._safe_stop_confirmed,
            "offline_session_handoff": {
                "was_active": self.offline_service_was_active,
                "stopped_for_exhibition": self.offline_service_stopped,
                "restored": self.offline_service_restored,
            },
            "children": {
                name: process.pid
                for name, process in self.children.items()
                if process.poll() is None
            },
            "updated_at": datetime.now(timezone.utc).isoformat(),
        }
        temporary = self.state_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, self.state_path)

    def _command_environment(self, name: str) -> Dict[str, str]:
        """Build mode-scoped child env without weakening another mode."""
        environment = os.environ.copy()
        if name in {"ready_static", "ready_control"}:
            # The live child performs the full robot-facing read-only SDK
            # preflight before publishing its ROS services.  Pass the
            # manager's validated timeout to the readiness helper so the two
            # bounded waits cannot disagree (the old 30/40-second defaults
            # raced a healthy 45-60-second hardware preflight).
            environment["R1_EXHIBITION_READY_TIMEOUT_SEC"] = str(
                self.settings.ready_timeout_sec
            )
        if self.mode == "static" and name in {
            "static_writer",
            "ready_static",
            "prepare",
        }:
            environment["ROBOT_CONFIRM_STATIC_PREPARE"] = "1"
            environment["R1_EXHIBITION_SESSION_MODE"] = "deadman"
            environment.pop("ROBOT_EXHIBITION_SESSION", None)
            if name == "prepare":
                # Static startup has the same completed SDK preflight + typed
                # graph gate as control. Keep token/current feedback checks,
                # but do not repeat the full preflight a second time.
                environment["R1_EXHIBITION_FAST_PREPARE"] = "1"
        elif self.mode == "control" and name in {
            "control",
            "ready_control",
            "arm_session",
            "prepare",
            "pause_session",
            "resume_session",
            "warm_resume",
            "disarm_session",
            "clear_emergency",
        }:
            environment["R1_EXHIBITION_SESSION_MODE"] = (
                self.settings.session_mode
            )
            if self.settings.session_mode == "session_arm":
                environment["ROBOT_EXHIBITION_SESSION"] = "1"
                environment["SAFETY_PROFILE"] = "exhibition"
            if name == "prepare":
                # The live child has just completed the full SDK preflight and
                # the manager has confirmed its typed graph.  Bind the shorter
                # duplicate-check path to the same commissioning token; the
                # prepare helper still verifies fresh feedback, VR commands,
                # dormant ArmSdk traffic and both KILL latches before StandUp.
                environment["R1_EXHIBITION_FAST_PREPARE"] = "1"
        return environment

    def _run_checked(self, name: str, timeout: float) -> bool:
        self.last_action_exit_code = 2
        cleanup = name in {"stop", "kill", "disarm_session", "offline_start", "offline_stop"}
        if self.stop_requested and not cleanup:
            self._log(f"{name}: cancelled because session stop was requested")
            return False
        argv = list(self.settings.commands[name])
        if not argv:
            self.last_action_exit_code = 0
            return True
        self._log(f"{name}: starting {shlex.join(argv)}")
        started_at = time.monotonic()
        try:
            completed = run_helper(
                argv,
                cwd=self.settings.project_dir,
                timeout=timeout,
                env=self._command_environment(name),
                cancelled=lambda: self.stop_requested and not cleanup,
            )
        except OSError as exception:
            self._log(f"{name}: could not start: {exception}")
            return False
        except subprocess.TimeoutExpired as exception:
            output = exception.stdout or ""
            if isinstance(output, bytes):
                output = output.decode("utf-8", errors="replace")
            if output and self._log_file:
                self._log_file.write(output)
            self._log(f"{name}: timed out after {timeout:.1f} s")
            return False
        self.last_action_exit_code = completed.returncode
        if completed.stdout:
            print(completed.stdout, end="", flush=True)
            if self._log_file:
                self._log_file.write(completed.stdout)
        if completed.returncode:
            self._log(f"{name}: failed with exit {completed.returncode}")
            return False
        self._log(f"{name}: complete elapsed_sec={time.monotonic() - started_at:.3f}")
        return True

    def _start_child(self, name: str) -> bool:
        argv = list(self.settings.commands[name])
        if not argv:
            return True
        existing = self.children.get(name)
        if existing and existing.poll() is None:
            return True
        self._log(f"{name}: starting {shlex.join(argv)}")
        try:
            process = subprocess.Popen(
                argv,
                cwd=self.settings.project_dir,
                stdout=self._log_file,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
                env=self._command_environment(name),
            )
        except OSError as exception:
            self._log(f"{name}: could not start: {exception}")
            return False
        self.children[name] = process
        return True

    def _probe_offline_service(self) -> Optional[bool]:
        """Return True/False for active/inactive, or None on probe failure."""
        argv = list(self.settings.commands["offline_status"])
        if not argv:
            return False
        self._log(f"offline_status: starting {shlex.join(argv)}")
        try:
            completed = subprocess.run(
                argv,
                cwd=self.settings.project_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=8.0,
                check=False,
                env=os.environ.copy(),
            )
        except (OSError, subprocess.TimeoutExpired) as exception:
            self._log(f"offline_status: could not complete: {exception}")
            return None
        if completed.stdout:
            output = completed.stdout.rstrip()
            if output:
                self._log(f"offline_status: {output}")
        if completed.returncode == 0:
            return True
        if completed.returncode == 3:
            return False
        self._log(
            "offline_status: unexpected exit "
            f"{completed.returncode}; refusing to start a competing graph"
        )
        return None

    def _take_offline_service_handoff(self) -> bool:
        """Stop an active compatible offline unit before binding its ports."""
        active = self._probe_offline_service()
        if active is None:
            return False
        self.offline_service_was_active = active
        if not active:
            self._log("offline handoff: service was not active")
            return True
        self._write_state(
            "handoff", "stopping the compatible offline LAN service"
        )
        if not self._run_checked("offline_stop", 20.0):
            return False
        # The helper itself waits until systemd reports inactive. Record
        # ownership immediately so every later probe/startup failure restores
        # a service we successfully stopped.
        self.offline_service_stopped = True
        still_active = self._probe_offline_service()
        if still_active is not False:
            self._log("offline handoff: service did not become inactive")
            return False
        self._log("offline handoff: ports released for exhibition graph")
        return True

    def _restore_offline_service(self) -> bool:
        """Restore the prior unit after every exhibition child has stopped."""
        if not (
            self.offline_service_was_active
            and self.offline_service_stopped
        ):
            return True
        if not self._run_checked("offline_start", 20.0):
            self._log("offline handoff: failed to restore prior service")
            return False
        active = self._probe_offline_service()
        if active is not True:
            self._log("offline handoff: restored service is not active")
            return False
        self.offline_service_restored = True
        self._log("offline handoff: prior service restored after STOP")
        return True

    def _terminate_child(self, name: str) -> None:
        process = self.children.pop(name, None)
        if not process or process.poll() is not None:
            return
        self._log(f"{name}: stopping process group {process.pid}")
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._log(f"{name}: process group did not report exit")

    def _safe_stop(self) -> bool:
        """Request the reviewed STOP path; use KILL only as its fallback."""
        if self._safe_stop_completed:
            return self._safe_stop_confirmed
        # This only closes the bridge's exhibition session latch.  The
        # independent writer/supervisor STOP remains mandatory below.
        self._run_checked("disarm_session", 8.0)
        if self.settings.mock:
            self._safe_stop_confirmed = True
            self._safe_stop_completed = True
            return True
        if self._run_checked("stop", 20.0):
            self._safe_stop_confirmed = True
            self._safe_stop_completed = True
            return True
        self._log("STOP did not confirm; requesting independent KILL")
        if self._run_checked("kill", 12.0):
            self._safe_stop_confirmed = True
            self._safe_stop_completed = True
            return True
        # Keep the attempt retryable.  The finally path gets one more bounded
        # chance, and an external ``exhibition-stop`` then owns its independent
        # STOP/KILL fallback.  Never turn an unconfirmed stop into permission
        # to restart a physical writer.
        self._safe_stop_confirmed = False
        return False

    def _wait_for_graph(self, child_name: str, ready_name: str) -> bool:
        """Wait for typed ROS services and prove the child stayed alive."""
        child = self.children.get(child_name)
        if child is None or child.poll() is not None:
            self._log(f"{child_name}: exited before readiness")
            return False
        # The helper enforces its own deadline, but its final Fast DDS
        # discovery iteration can legitimately extend past that deadline by
        # the bounded service/topic/parameter command timeouts.  Leave enough
        # parent-side grace for the helper to print the actual missing items
        # instead of terminating it just before its diagnostic result.
        if not self._run_checked(
            ready_name, float(self.settings.ready_timeout_sec + 20)
        ):
            return False
        if child.poll() is not None:
            self._log(f"{child_name}: exited while readiness was checked")
            return False
        return True

    def _start_static_writer(self) -> bool:
        if not self.settings.commands["static_writer"]:
            return True
        self._safe_stop_completed = False
        self._safe_stop_confirmed = False
        self._write_state(
            "starting_static", "starting featureless static writer"
        )
        if not self._start_child("static_writer"):
            return False
        if not self._wait_for_graph("static_writer", "ready_static"):
            return False
        if not self._run_checked(
            "prepare", self.settings.preflight_timeout_sec
        ):
            return False
        return True

    def _start_control_writer(self) -> bool:
        self._safe_stop_completed = False
        self._safe_stop_confirmed = False
        self._write_state(
            "starting_control", "starting fail-closed live graph"
        )
        if not self._start_child("control"):
            return False
        if not self._wait_for_graph("control", "ready_control"):
            return False
        # Calibration must happen while /vr/teleop/active is false.  It saves
        # the current HMD/controller neutral and sends no command to the robot.
        # Only after that snapshot succeeds may session-arm make VR active.
        if self.settings.calibrate_on_start and not self._run_checked(
            "calibrate", 20.0
        ):
            return False
        # Session arm replaces the initial physical Deadman press, but it does
        # not bypass writer prepare. The writer performs its own anti-stale
        # re-arm internally; an external disarm after prepare would latch KILL.
        if not self._run_checked("arm_session", 12.0):
            return False
        if not self._run_checked(
            "prepare", self.settings.preflight_timeout_sec
        ):
            return False
        return True

    def _start_mode(self) -> bool:
        self._write_state("handoff", "checking offline LAN service ownership")
        if not self._take_offline_service_handoff():
            self._write_state(
                "blocked", "offline LAN service handoff failed"
            )
            return False
        self._write_state("preflight", "read-only checks")
        if not self._run_checked(
            "preflight", self.settings.preflight_timeout_sec
        ):
            self._write_state("blocked", "preflight failed")
            return False

        if not self._start_child("pov"):
            self._write_state("blocked", "POV process could not start")
            return False
        if self.mode == "static":
            if not self._start_static_writer():
                self._write_state(
                    "blocked", "static writer readiness/prepare failed"
                )
                self._safe_stop()
                return False
            detail = (
                "static stand prepared; POV active"
                if self.settings.commands["static_writer"]
                else "mock static: POV active; no physical writer"
            )
            self._write_state("ready", detail)
            return True

        if not self._start_control_writer():
            self._write_state(
                "blocked", "control readiness/session-arm/prepare failed"
            )
            self._safe_stop()
            return False
        self._write_state("ready", "control active; slow-safe profile")
        return True

    def _restart_pov(self, mark_ready: bool = True) -> bool:
        previous_status = self._status
        self._terminate_child("pov")
        self.pov_restarts += 1
        limit = self.settings.pov_restart_limit
        if limit and self.pov_restarts > limit:
            self._write_state("degraded", "POV restart limit reached")
            return False
        delay = min(
            self.settings.restart_initial_sec
            * (2 ** min(16, max(0, self.pov_restarts - 1))),
            self.settings.restart_max_sec,
        )
        self._write_state("reconnecting", f"POV restart {self.pov_restarts}")
        self._log(f"POV reconnect backoff: {delay:.2f} s")
        deadline = time.monotonic() + delay
        while time.monotonic() < deadline and not self.stop_requested:
            time.sleep(0.05)
        if self.stop_requested:
            return False
        if not self._start_child("pov"):
            self._write_state("degraded", "POV process could not restart")
            return False
        if mark_ready:
            # A camera restart is not proof that a blocked/paused writer is
            # healthy and must never silently turn a failed RUN green.
            status = previous_status
            detail = (
                "control paused; locomotion zero; pose held; POV restarted"
                if status == "locked"
                else f"{self.mode}; POV restarted"
            )
            self._write_state(status, detail)
        else:
            self._write_state(
                "reconnecting", "POV restarted; rebuilding writer graph"
            )
        return True

    def _reconnect(self, *, restart_pov: bool = True) -> bool:
        self.reconnect_requested = False
        self._log("operator requested reconnect")
        writer_name = (
            "static_writer" if self.mode == "static" else "control"
        )
        have_writer = bool(self.settings.commands[writer_name])
        if have_writer:
            # A physical writer is never silently restarted. The explicit
            # action first takes reviewed STOP/KILL, then builds a fresh graph
            # which must pass typed readiness and robot-prepare again.
            self._write_state(
                "reconnecting", f"stopping {self.mode} writer first"
            )
            if not self._safe_stop():
                self._write_state(
                    "blocked",
                    "STOP/KILL unconfirmed; writer reconnect aborted",
                )
                return False
            self._terminate_child(writer_name)
        if restart_pov and not self._restart_pov(mark_ready=False):
            self._write_state("degraded", "POV reconnect failed")
            return False
        if not self._run_checked(
            "preflight", self.settings.preflight_timeout_sec
        ):
            self._write_state("blocked", "reconnect preflight failed")
            return False
        started = (
            self._start_static_writer()
            if self.mode == "static"
            else self._start_control_writer()
        )
        if not started:
            self._safe_stop()
            self._write_state(
                "blocked", f"{self.mode} reconnect readiness/prepare failed"
            )
            return False
        detail = (
            "static reconnected; stand prepared"
            if self.mode == "static" and have_writer
            else (
                "static reconnected; mock POV only"
                if self.mode == "static"
                else "control reconnected; slow-safe profile"
            )
        )
        self._write_state("ready", detail)
        return True

    def _resume_control(self) -> None:
        """Handle explicit RUN with the original owner's environment/token."""
        self._write_state("resuming", "checking the existing control graph")
        if self._run_checked("warm_resume", 8.0):
            self._write_state("ready", "warm control graph resumed; no prepare")
            return
        if self.last_action_exit_code != 3:
            # Missing tracking or an unavailable service is a hold condition,
            # not permission to clear KILL or cycle the robot's FSM.
            self._write_state("degraded", "RUN waiting for fresh VR/writer readiness")
            return
        self._write_state("rearming", "explicit RUN recovering a latched stop")
        # An initialized SDK transport retains its rt/arm_sdk publisher even
        # after KILL releases ownership. Preparing that same process would
        # fail the mandatory one-writer idle gate. Do not weaken that gate or
        # try to pause a potentially disarmed bridge: retire our old graph via
        # reviewed STOP/KILL, then perform fresh preflight/readiness/arm/prepare.
        # Only this explicit RUN may rebuild; healthy RUN/LOCK stays warm.
        # The USB owner/relay and existing POV need not restart.
        self._reconnect(restart_pov=False)

    def run(self) -> int:
        """Run the selected exhibition mode until STOP or a critical fault."""
        self._prepare_paths()
        previous_handlers = {
            sig: signal.getsignal(sig)
            for sig in (
                signal.SIGTERM,
                signal.SIGINT,
                signal.SIGHUP,
                signal.SIGUSR1,
                signal.SIGUSR2,
            )
        }

        def request_stop(_signum, _frame):
            self.stop_requested = True

        def request_reconnect(_signum, _frame):
            self.reconnect_requested = True

        def request_lock(_signum, _frame):
            self.lock_requested = True
            self.run_requested = False

        def request_run(_signum, _frame):
            self.run_requested = True
            self.lock_requested = False

        signal.signal(signal.SIGTERM, request_stop)
        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGHUP, request_reconnect)
        signal.signal(signal.SIGUSR1, request_lock)
        signal.signal(signal.SIGUSR2, request_run)
        self._log(
            f"session={self.session_id} mode={self.mode} "
            f"mock={self.settings.mock}"
        )
        self._log(f"log={self._log_path}")
        exit_code = 0
        try:
            started = self._start_mode()
            if not started:
                exit_code = 2
            while started and not self.stop_requested:
                if self.lock_requested:
                    self.lock_requested = False
                    if (
                        self.mode != "control"
                        or self.settings.session_mode != "session_arm"
                    ):
                        self._write_state(
                            "degraded",
                            "fast LOCK requires a session_arm control graph",
                        )
                    elif self._run_checked(
                        "pause_session", 8.0
                    ):
                        self._write_state(
                            "locked",
                            "control paused; locomotion zero; pose held",
                        )
                    else:
                        self._write_state(
                            "degraded", "fast LOCK pause was not confirmed"
                        )
                if self.run_requested:
                    self.run_requested = False
                    if (
                        self.mode != "control"
                        or self.settings.session_mode != "session_arm"
                    ):
                        self._write_state(
                            "degraded",
                            "fast RUN requires a session_arm control graph",
                        )
                    else:
                        self._resume_control()
                if self.reconnect_requested and not self._reconnect():
                    exit_code = 2
                    break
                pov = self.children.get("pov")
                if pov and pov.poll() is not None:
                    self._log(f"POV exited with code {pov.returncode}")
                    if not self._restart_pov():
                        if self.mode == "static":
                            exit_code = 1
                            break
                writer_name = (
                    "static_writer" if self.mode == "static" else "control"
                )
                if self.settings.commands[writer_name]:
                    writer = self.children.get(writer_name)
                    if writer is None or writer.poll() is not None:
                        code = None if writer is None else writer.returncode
                        self._log(
                            f"{writer_name} exited unexpectedly: {code}"
                        )
                        self._write_state(
                            "blocked",
                            f"{writer_name} exited; STOP requested",
                        )
                        self._safe_stop()
                        exit_code = 1
                        break
                time.sleep(0.20)
        finally:
            stop_confirmed = True
            if self.mode == "control" or "static_writer" in self.children:
                stop_confirmed = self._safe_stop()
                if not stop_confirmed:
                    exit_code = max(exit_code, 1)
            for name in list(self.children):
                self._terminate_child(name)
            restored = self._restore_offline_service()
            detail = "session processes stopped"
            if not stop_confirmed:
                detail += "; STOP/KILL unconfirmed"
            if not restored:
                detail += "; offline service restore failed"
            self._write_state("stopped", detail)
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)
            if self._log_file:
                self._log_file.close()
            if self._lock_file:
                fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
                self._lock_file.close()
        return exit_code


def _read_state(settings: ExhibitionSettings) -> dict:
    try:
        data = json.loads(
            (settings.runtime_dir / "state.json").read_text("utf-8")
        )
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return {"status": "stopped", "detail": "no active exhibition session"}
    return data if isinstance(data, dict) else {"status": "unknown"}


def _active_pid(state: Mapping[str, object]) -> Optional[int]:
    try:
        pid = int(state.get("pid", 0))
    except (TypeError, ValueError):
        return None
    if pid <= 1:
        return None
    try:
        command_line = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return None
    # The shell wrapper uses exec, so the persistent process normally appears
    # as ``python3 -m exhibition.orchestrator``.  Accept the wrapper spelling
    # too for direct/future packaging, but reject unrelated reused PIDs.
    if not any(
        marker in command_line
        for marker in (b"r1-exhibition", b"exhibition.orchestrator")
    ):
        return None
    return pid


def active_session_snapshot() -> dict:
    """Read-only desktop discovery, including sessions started in a terminal."""
    try:
        state = _read_state(ExhibitionSettings.from_environment())
        return state if _active_pid(state) is not None else {}
    except (OSError, ValueError):
        return {}


def _run_action(
    settings: ExhibitionSettings, name: str, timeout: float
) -> int:
    argv = list(settings.commands[name])
    if not argv:
        return 0
    try:
        return subprocess.run(
            argv,
            cwd=settings.project_dir,
            timeout=timeout,
            check=False,
        ).returncode
    except OSError as exception:
        print(
            f"[BLOCKED] {name} could not start: {exception}",
            file=sys.stderr,
        )
        return 2
    except subprocess.TimeoutExpired:
        print(
            f"[BLOCKED] {name} timed out after {timeout:.1f} s",
            file=sys.stderr,
        )
        return 2


def _wait_for_manager_stop(
    settings: ExhibitionSettings,
    state: Mapping[str, object],
    pid: int,
) -> bool:
    """Wait for the signalled manager to own one complete STOP sequence."""
    session_id = state.get("session_id")
    deadline = time.monotonic() + settings.stop_wait_sec
    while time.monotonic() < deadline:
        current = _read_state(settings)
        if (
            current.get("session_id") == session_id
            and current.get("status") == "stopped"
        ):
            return bool(
                settings.mock
                or current.get("safe_stop_confirmed") is True
            )
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            # A crashed/killed manager may disappear without calling the
            # robot's STOP services.  Let the caller execute its independent
            # STOP/KILL fallback instead of reporting successful cleanup.
            return False
        except PermissionError:
            return False
        time.sleep(0.05)
    return False


def _wait_for_transport_cleanup(settings, state) -> bool:
    """STOP is confirmed already; a USB timeout must NOT issue another STOP."""
    deadline = time.monotonic() + settings.stop_wait_sec
    while owner_alive(state.get("transport_owner")):
        if time.monotonic() >= deadline:
            print("[BLOCKED] Robot STOP confirmed, but USB cleanup is still running; "
                  "next mode was not started. Retry after cleanup.", file=sys.stderr)
            return False
        time.sleep(0.05)
    return True


def _confirmed_cleanup_is_idle(settings, state) -> bool:
    """A repeated STOP may be a no-op only after a reviewed, fully retired session.

    Hold the same lock as a new manager while rechecking state and processes;
    a stale JSON file alone is never evidence that control is stopped.
    """
    if not (
        state.get('status') == 'stopped'
        and state.get('safe_stop_confirmed') is True
        and state.get('children') == {}
        and state.get('mock') is settings.mock
        and isinstance(state.get('session_id'), str) and state['session_id']
        and isinstance(state.get('updated_at'), str) and state['updated_at']
    ):
        return False
    try:
        with (settings.runtime_dir / 'session.lock').open('r+') as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            current = _read_state(settings)
            if current != state or _active_pid(current) is not None:
                return False
            if owner_alive(current.get('transport_owner')) or local_control_process_exists():
                return False
            return _read_state(settings) == current
    except (OSError, ValueError):
        return False


def main(argv: Optional[Iterable[str]] = None) -> int:
    """Run one CLI action and return its process exit status."""
    parser = argparse.ArgumentParser(
        description="Safe, offline Unitree R1 exhibition process manager"
    )
    parser.add_argument(
        "action",
        choices=(
            "panel",
            "static",
            "control",
            "lock",
            "reconnect",
            "rearm",
            "calibrate",
            "stop",
            "status",
            "plan",
        ),
    )
    parser.add_argument("mode", nargs="?", choices=("static", "control"))
    parser.add_argument(
        "--mock", action="store_true", help="force hardware-free mock commands"
    )
    arguments = parser.parse_args(list(argv) if argv is not None else None)
    if "R1_EXHIBITION_SESSION_MODE" not in os.environ:
        requested_mode = (
            arguments.action
            if arguments.action in {"static", "control"}
            else arguments.mode
        )
        if requested_mode == "static":
            os.environ["R1_EXHIBITION_SESSION_MODE"] = "deadman"
        elif requested_mode == "control":
            os.environ["R1_EXHIBITION_SESSION_MODE"] = "session_arm"
    if arguments.mock:
        os.environ["R1_EXHIBITION_MOCK"] = "1"
    try:
        settings = ExhibitionSettings.from_environment()
    except ValueError as exception:
        print(
            f"[BLOCKED] invalid exhibition configuration: {exception}",
            file=sys.stderr,
        )
        return 2

    if arguments.action == "panel":
        os.environ["R1_OPERATOR_EXHIBITION"] = "1"
        os.chdir(settings.project_dir)
        os.execvpe(
            settings.commands["panel"][0],
            list(settings.commands["panel"]),
            os.environ.copy(),
        )
    if arguments.action in {"static", "control"}:
        try:
            return ExhibitionManager(settings, arguments.action).run()
        except RuntimeError as exception:
            print(f"[BLOCKED] {exception}", file=sys.stderr)
            return 2
    if arguments.action == "plan":
        mode = arguments.mode or "static"
        print(
            json.dumps(
                build_plan(mode, settings), ensure_ascii=False, indent=2
            )
        )
        return 0

    state = _read_state(settings)
    if arguments.action == "status":
        state["active"] = _active_pid(state) is not None
        print(json.dumps(state, ensure_ascii=False, indent=2))
        return 0

    pid = _active_pid(state)
    if arguments.action == "lock":
        if (
            pid is None
            or state.get("mode") != "control"
            or state.get("session_mode") != "session_arm"
        ):
            print(
                "[BLOCKED] fast LOCK requires an active session_arm "
                "control graph",
                file=sys.stderr,
            )
            return 2
        if state.get("status") == "locked":
            print("[OK] Existing control graph is already in LOCK.")
            return 0
        session_id = state.get("session_id")
        previous_update = state.get("updated_at")
        try:
            os.kill(pid, signal.SIGUSR1)
        except (ProcessLookupError, PermissionError) as exception:
            print(
                f"[BLOCKED] fast LOCK signal failed: {exception}",
                file=sys.stderr,
            )
            return 2
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            current = _read_state(settings)
            if current.get("session_id") != session_id:
                break
            status = current.get("status")
            if status == "locked":
                print(
                    "[OK] Existing control graph entered LOCK; "
                    "SDK, writer and video stayed warm."
                )
                return 0
            if (current.get('updated_at') != previous_update
                    and status in {"degraded", "blocked", "stopped"}):
                print(
                    "[BLOCKED] fast LOCK was not confirmed: "
                    f"{current.get('detail', status)}",
                    file=sys.stderr,
                )
                return 2
            time.sleep(0.05)
        print(
            "[BLOCKED] fast LOCK acknowledgement timed out",
            file=sys.stderr,
        )
        return 2
    if arguments.action == "reconnect":
        if pid is None:
            print(
                "[BLOCKED] no active exhibition session to reconnect",
                file=sys.stderr,
            )
            return 2
        os.kill(pid, signal.SIGHUP)
        mode = state.get("mode", "unknown")
        print(f"Reconnect requested for {mode} session.")
        return 0
    if arguments.action == "rearm":
        if pid is None or state.get("mode") != "control":
            print(
                "[BLOCKED] fast RUN requires an active control session",
                file=sys.stderr,
            )
            return 2
        if state.get("session_mode") != "session_arm":
            print(
                "[BLOCKED] fast RUN requires the session_arm control graph",
                file=sys.stderr,
            )
            return 2
        if settings.session_mode != "session_arm":
            print(
                "[BLOCKED] fast RUN lacks matching exhibition configuration",
                file=sys.stderr,
            )
            return 2
        session_id = state.get("session_id")
        previous_update = state.get("updated_at")
        try:
            os.kill(pid, signal.SIGUSR2)
        except (ProcessLookupError, PermissionError):
            print("[BLOCKED] control manager unavailable", file=sys.stderr)
            return 2
        # The owner performs recovery with its original commissioning token.
        # A reopened panel must neither expose/copy that token nor replace it.
        deadline = time.monotonic() + settings.preflight_timeout_sec + 40.0
        while time.monotonic() < deadline:
            current = _read_state(settings)
            if current.get("session_id") != session_id:
                break
            if current.get("updated_at") != previous_update:
                if current.get("status") == "ready":
                    print("[OK] Existing control graph is ready: " +
                          str(current.get("detail", "")))
                    return 0
                if current.get("status") in {"degraded", "blocked", "stopped"}:
                    print("[BLOCKED] " + str(current.get("detail", "RUN rejected")))
                    return 2
            time.sleep(0.05)
        print("[BLOCKED] RUN acknowledgement unavailable", file=sys.stderr)
        return 2
    if arguments.action == "calibrate":
        if pid is None or state.get("mode") != "control":
            print(
                "[BLOCKED] calibration requires an active control session",
                file=sys.stderr,
            )
            return 2
        if state.get("session_mode") == "session_arm":
            if settings.session_mode != "session_arm":
                print(
                    "[BLOCKED] active session uses session_arm but this "
                    "action lacks matching exhibition configuration",
                    file=sys.stderr,
                )
                return 2
            if _run_action(settings, "pause_session", 8.0):
                return 2
            result = _run_action(
                settings, "calibrate", settings.preflight_timeout_sec
            )
            resume_result = _run_action(settings, "resume_session", 12.0)
            return result or resume_result
        return _run_action(
            settings, "calibrate", settings.preflight_timeout_sec
        )

    if pid is not None:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pid = None
        if pid is not None and _wait_for_manager_stop(
            settings, state, pid
        ):
            if not _wait_for_transport_cleanup(settings, state):
                return 2
            print(
                f"Exhibition session {pid} completed reviewed STOP cleanup."
            )
            return 0
        if pid is not None:
            print(
                "[WARN] manager did not confirm STOP in time; using the "
                "independent STOP/KILL fallback.",
                file=sys.stderr,
            )

    if (pid is None and state.get("status") == "stopped"
            and state.get("safe_stop_confirmed") is True
            and owner_alive(state.get("transport_owner"))):
        return 0 if _wait_for_transport_cleanup(settings, state) else 2

    if pid is None and _confirmed_cleanup_is_idle(settings, state):
        print('[OK] Previous reviewed STOP cleanup is complete; no local control owner/writer remains.')
        return 0

    # STOP/KILL stays usable if manager state was lost, stale, or its bounded
    # cleanup did not finish. Closing session_arm is best-effort and never
    # replaces the independent writer/supervisor stop.
    if state.get("session_mode") == "session_arm":
        _run_action(settings, "disarm_session", 8.0)
    stop_code = _run_action(settings, "stop", 20.0)
    if stop_code and not settings.mock:
        _run_action(settings, "kill", 12.0)
    if pid is None:
        print(
            "No active exhibition manager; reviewed STOP/KILL path requested."
        )
    else:
        print("Independent STOP/KILL fallback requested.")
    return 0 if stop_code == 0 else stop_code


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
