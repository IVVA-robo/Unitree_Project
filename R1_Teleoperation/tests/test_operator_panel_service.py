import json
import os
from pathlib import Path
import subprocess
import time


PROJECT_DIR = Path(__file__).resolve().parents[1]
STOP_HELPER = PROJECT_DIR / "scripts" / "r1-operator-panel-service-stop"
UNIT = PROJECT_DIR / "systemd" / "user" / "r1-operator-panel.service"
DESKTOP = PROJECT_DIR / "desktop" / "Unitree R1 Панель оператора.desktop"
PANEL_STATUS = PROJECT_DIR / "scripts" / "r1-panel-status"
TELEOPERATION_STATUS = PROJECT_DIR / "scripts" / "r1-teleoperation-status"


def _fake_exhibition(tmp_path: Path, active_payload) -> tuple[Path, Path]:
    calls = tmp_path / "calls"
    helper = tmp_path / "r1-exhibition"
    helper.write_text(
        "#!/usr/bin/env python3\n"
        "import json, os, pathlib, sys\n"
        f"calls = pathlib.Path({str(calls)!r})\n"
        "with calls.open('a', encoding='utf-8') as stream:\n"
        "    stream.write(sys.argv[1] + '\\n')\n"
        "if sys.argv[1] == 'status':\n"
        f"    print({json.dumps(json.dumps(active_payload))})\n"
        "    raise SystemExit(0)\n"
        "if sys.argv[1] == 'stop':\n"
        "    raise SystemExit(int(os.environ.get('FAKE_STOP_CODE', '0')))\n"
        "raise SystemExit(2)\n",
        encoding="utf-8",
    )
    helper.chmod(0o755)
    return helper, calls


def _run_stop_helper(helper: Path, tmp_path: Path, **extra_environment):
    environment = os.environ.copy()
    environment["R1_OPERATOR_PANEL_EXHIBITION_COMMAND"] = str(helper)
    cgroup_procs = tmp_path / "cgroup.procs"
    cgroup_procs.write_text(f"{os.getpid()}\n", encoding="ascii")
    environment["R1_OPERATOR_PANEL_CGROUP_PROCS"] = str(cgroup_procs)
    environment.update(extra_environment)
    return subprocess.run(
        [str(STOP_HELPER)],
        cwd=PROJECT_DIR,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )


def test_service_stop_skips_robot_path_without_active_manager(tmp_path):
    helper, calls = _fake_exhibition(tmp_path, {"active": False})

    result = _run_stop_helper(helper, tmp_path)

    assert result.returncode == 0
    assert calls.read_text(encoding="utf-8").splitlines() == ["status"]
    assert "no active exhibition manager" in result.stdout


def test_service_stop_waits_on_reviewed_stop_for_active_manager(tmp_path):
    helper, calls = _fake_exhibition(tmp_path, {"active": True})

    result = _run_stop_helper(helper, tmp_path)

    assert result.returncode == 0
    assert calls.read_text(encoding="utf-8").splitlines() == ["status", "stop"]
    assert "reviewed STOP cleanup" in result.stdout


def test_service_stop_propagates_failed_reviewed_stop(tmp_path):
    helper, calls = _fake_exhibition(tmp_path, {"active": True})

    result = _run_stop_helper(helper, tmp_path, FAKE_STOP_CODE="7")

    assert result.returncode == 7
    assert calls.read_text(encoding="utf-8").splitlines() == ["status", "stop"]


def test_service_stop_detects_advanced_live_owner_without_manager(tmp_path):
    helper, calls = _fake_exhibition(tmp_path, {"active": False})
    live_owner = subprocess.Popen(
        [
            "python3",
            "-c",
            "import time; time.sleep(30)",
            "r1_teleop_live.launch.py",
            "transport:=sdk",
            "send_commands:=true",
        ]
    )
    try:
        time.sleep(0.05)
        cgroup_procs = tmp_path / "live.procs"
        cgroup_procs.write_text(f"{live_owner.pid}\n", encoding="ascii")
        result = _run_stop_helper(
            helper,
            tmp_path,
            R1_OPERATOR_PANEL_CGROUP_PROCS=str(cgroup_procs),
        )
    finally:
        live_owner.terminate()
        live_owner.wait(timeout=2)

    assert result.returncode == 0
    assert calls.read_text(encoding="utf-8").splitlines() == ["status", "stop"]
    assert "advanced live process owner" in result.stdout


def test_unit_and_desktop_use_service_owned_safe_paths():
    unit_text = UNIT.read_text(encoding="utf-8")
    desktop_text = DESKTOP.read_text(encoding="utf-8")

    assert "PartOf=graphical-session.target" in unit_text
    assert "ExecStop=-/home/unitree/Unitree_Project/R1_Teleoperation/scripts/r1-operator-panel-service-stop" in unit_text
    assert "TimeoutStopSec=120s" in unit_text
    assert "KillMode=control-group" in unit_text
    assert "scripts/r1-operator-panel-launch" in desktop_text
    assert "StartupWMClass=__main__.py" in desktop_text
    assert "assets/icons/unitree-r1-robot.png" in desktop_text
    assert (PROJECT_DIR / "assets" / "icons" / "unitree-r1-robot.png").is_file()


def test_panel_status_ros_graph_queries_never_start_ros2_daemon():
    script = PANEL_STATUS.read_text(encoding="utf-8")

    expected_queries = (
        "ros2 topic list --no-daemon --spin-time 0.5",
        "ros2 topic echo --once /vr/teleop/active \\\n"
        "      std_msgs/msg/Bool --no-daemon",
        "ros2 topic echo --once /vr/teleop/status \\\n"
        "      std_msgs/msg/String --field data --no-daemon",
        "ros2 topic echo --once /r1/safety/kill \\\n"
        "      std_msgs/msg/Bool --no-daemon",
        "ros2 topic echo --once /r1/live_writer/status \\\n"
        "      std_msgs/msg/String --field data --qos-durability transient_local \\\n"
        "      --no-daemon",
    )
    for query in expected_queries:
        assert query in script

    # The panel runs this snapshot repeatedly.  A daemon-backed graph query
    # can detach from its short-lived QProcess and then hold the systemd
    # service cgroup open during stop/restart.
    assert script.count("ros2 topic ") == len(expected_queries)


def test_panel_status_marks_locked_manager_degraded_when_kill_is_latched():
    script = PANEL_STATUS.read_text(encoding="utf-8")

    assert "${manager_value:-stopped} == locked" in script
    assert 'emit exhibition degraded' in script
    assert (
        'emit exhibition_detail "writer safety latched; link recovered but '
        'explicit RUN is required"'
    ) in script


def test_fast_panel_status_checks_control_once_and_skips_ros_and_pc2(
    tmp_path,
):
    calls = tmp_path / "calls.log"
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()

    def helper(name: str, body: str) -> None:
        path = fake_bin / name
        path.write_text("#!/usr/bin/env bash\n" + body, encoding="utf-8")
        path.chmod(0o755)

    helper(
        "ping",
        f"printf 'ping %s\\n' \"$*\" >> {str(calls)!r}\nexit 0\n",
    )
    helper(
        "curl",
        f"printf 'curl %s\\n' \"$*\" >> {str(calls)!r}\n"
        "printf '{\"ready\":false}\\n'\n",
    )
    helper(
        "ros2",
        f"printf 'ros2 %s\\n' \"$*\" >> {str(calls)!r}\nexit 99\n",
    )

    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "R1_PANEL_STATUS_FAST": "1",
            "R1_ROBOT_INTERFACE": "lo",
            "R1_CONTROL_IP": "192.0.2.161",
            "R1_PC2_IP": "192.0.2.164",
            "R1_TELEOP_STATUS_VIDEO_URL": "http://127.0.0.1:9/readyz",
            "R1_EXHIBITION_RUNTIME_DIR": str(tmp_path / "runtime"),
        }
    )

    result = subprocess.run(
        [str(PANEL_STATUS)],
        cwd=PROJECT_DIR,
        env=environment,
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    logged = calls.read_text(encoding="utf-8").splitlines()
    assert sum("192.0.2.161" in line for line in logged) == 1
    assert all("192.0.2.164" not in line for line in logged)
    assert all(not line.startswith("ros2 ") for line in logged)
    assert "STATUS robot=OK" in result.stdout
    assert "STATUS pc2=SKIPPED" in result.stdout


def test_usb_static_status_uses_pico_controller_service_without_starting_bridge(
    tmp_path,
):
    calls = tmp_path / "calls.log"
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()

    def helper(name: str, body: str) -> None:
        path = fake_bin / name
        path.write_text("#!/usr/bin/env bash\n" + body, encoding="utf-8")
        path.chmod(0o755)

    helper("ping", "exit 1\n")
    helper("curl", "printf '{\"ready\":false}\\n'\n")
    helper("ros2", "exit 0\n")
    helper(
        "adb",
        f"printf 'adb %s\\n' \"$*\" >> {str(calls)!r}\n"
        "if [[ \"$*\" == 'devices -l' ]]; then\n"
        "  printf 'List of devices attached\\n'\n"
        "  printf 'test-pico device usb:3-2 model:A9210\\n'\n"
        "elif [[ \"$*\" == '-s test-pico shell dumpsys pxrcontrollerservice' ]]; then\n"
        "  printf '    left controller:\\n      state: online\\n'\n"
        "  printf '    right controller:\\n      state: online\\n'\n"
        "else\n"
        "  exit 2\n"
        "fi\n",
    )

    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "R1_VR_TRANSPORT": "usb",
            "R1_ROBOT_INTERFACE": "lo",
            "R1_CONTROL_IP": "192.0.2.161",
            "R1_PC2_IP": "192.0.2.164",
            "R1_TELEOP_STATUS_VIDEO_URL": "http://127.0.0.1:9/readyz",
            "R1_EXHIBITION_RUNTIME_DIR": str(tmp_path / "runtime"),
            "ROS_DOMAIN_ID": "230",
            "R1_OFFLINE_ROS_DOMAIN_ID": "230",
        }
    )

    result = subprocess.run(
        [str(PANEL_STATUS)],
        cwd=PROJECT_DIR,
        env=environment,
        capture_output=True,
        text=True,
        timeout=8,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "STATUS vr=OK" in result.stdout
    assert "STATUS controllers=OK" in result.stdout
    assert "-s test-pico shell dumpsys pxrcontrollerservice" in calls.read_text(
        encoding="utf-8"
    )


def test_usb_static_status_does_not_claim_one_online_controller_is_ready(
    tmp_path,
):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()

    def helper(name: str, body: str) -> None:
        path = fake_bin / name
        path.write_text("#!/usr/bin/env bash\n" + body, encoding="utf-8")
        path.chmod(0o755)

    helper("ping", "exit 1\n")
    helper("curl", "printf '{\"ready\":false}\\n'\n")
    helper("ros2", "exit 0\n")
    helper(
        "adb",
        "if [[ \"$*\" == 'devices -l' ]]; then\n"
        "  printf 'List of devices attached\\n'\n"
        "  printf 'test-pico device usb:3-2 model:A9210\\n'\n"
        "else\n"
        "  printf '    left controller:\\n      state: online\\n'\n"
        "  printf '    right controller:\\n      state: offline\\n'\n"
        "fi\n",
    )

    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "R1_VR_TRANSPORT": "usb",
            "R1_ROBOT_INTERFACE": "lo",
            "R1_CONTROL_IP": "192.0.2.161",
            "R1_PC2_IP": "192.0.2.164",
            "R1_TELEOP_STATUS_VIDEO_URL": "http://127.0.0.1:9/readyz",
            "R1_EXHIBITION_RUNTIME_DIR": str(tmp_path / "runtime"),
            "ROS_DOMAIN_ID": "230",
            "R1_OFFLINE_ROS_DOMAIN_ID": "230",
        }
    )

    result = subprocess.run(
        [str(PANEL_STATUS)],
        cwd=PROJECT_DIR,
        env=environment,
        capture_output=True,
        text=True,
        timeout=8,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "STATUS controllers=OFFLINE" in result.stdout
    assert "STATUS controllers=OK" not in result.stdout


def test_teleoperation_status_echoes_never_start_ros2_daemon():
    script = TELEOPERATION_STATUS.read_text(encoding="utf-8")
    assert "ros2 topic echo --once /vr/teleop/active --no-daemon" in script
    assert "ros2 topic echo --once /vr/cmd_vel --no-daemon" in script
