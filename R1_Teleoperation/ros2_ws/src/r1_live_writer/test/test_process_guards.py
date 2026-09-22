"""Offline regressions for process-entry and asynchronous prepare guards."""

import os
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
NODE = (ROOT / "src" / "r1_live_writer_node.cpp").read_text(encoding="utf-8")
TRANSPORT = (ROOT / "src" / "transport.cpp").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    """Return a C++ function body for order-sensitive safety assertions."""
    signature_index = source.index(signature)
    opening_brace = source.index("{", signature_index)
    depth = 0
    for index in range(opening_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening_brace + 1:index]
    raise AssertionError(f"unterminated C++ function: {signature}")


def blocked_process_environment(rmw: str | None) -> dict[str, str]:
    """Build an isolated test environment without authorizing actuation."""
    environment = os.environ.copy()
    environment["R1_PHYSICAL_SDK_SESSION"] = "1"
    environment["ROS_DOMAIN_ID"] = "232"
    environment["ROS_LOCALHOST_ONLY"] = "1"
    for name in (
        "ROBOT_DRY_RUN",
        "ROBOT_ENABLE_ACTUATION",
        "ROBOT_CONFIRM_OFF_CHARGER",
        "ROBOT_CONFIRM_CLEAR_AREA",
        "ROBOT_CONFIRM_ESTOP_READY",
        "ROBOT_CONFIRM_COMMISSIONING",
        "ROBOT_COMMISSIONING_TOKEN",
        "ROBOT_VR_SOURCE_IP",
        "ROBOT_CONFIRM_HEAD_RECENTER",
        "ROBOT_CONFIRM_HEAD_OWNERSHIP_PROBE",
    ):
        environment.pop(name, None)
    if rmw is None:
        environment.pop("RMW_IMPLEMENTATION", None)
    else:
        environment["RMW_IMPLEMENTATION"] = rmw

    vendor_directory = os.environ.get("R1_UNITREE_TEST_LIBRARY_DIR", "")
    if vendor_directory and Path(vendor_directory).is_dir():
        previous = environment.get("LD_LIBRARY_PATH", "")
        environment["LD_LIBRARY_PATH"] = (
            vendor_directory
            if not previous
            else f"{vendor_directory}:{previous}"
        )
    return environment


@pytest.mark.parametrize("rmw", [None, "rmw_cyclonedds_cpp"])
def test_physical_marker_rejects_wrong_rmw_before_rclcpp_init(rmw):
    """Execute only the pre-init rejection path in a loopback-only netns."""
    executable_text = os.environ.get("R1_LIVE_WRITER_TEST_EXECUTABLE", "")
    assert executable_text, "CMake must provide the built writer executable"
    executable = Path(executable_text)
    assert executable.is_file()

    unshare = shutil.which("unshare")
    ip = shutil.which("ip")
    bash = shutil.which("bash")
    assert unshare and ip and bash

    # Even a regression that reaches rclcpp cannot see the robot-facing NIC.
    namespace_script = r"""
set -euo pipefail
"$1" link set dev lo up
links="$("$1" -o link show)"
test "$(printf '%s\n' "$links" | wc -l)" -eq 1
printf '%s\n' "$links" | grep -Eq '^[0-9]+: lo:'
exec "$2"
"""
    command = [
        unshare,
        "--user",
        "--map-root-user",
        "--net",
        "--",
        bash,
        "-c",
        namespace_script,
        "r1-preinit-rmw-test",
        ip,
        str(executable),
    ]
    completed = subprocess.run(
        command,
        env=blocked_process_environment(rmw),
        text=True,
        capture_output=True,
        timeout=5,
        check=False,
    )

    assert completed.returncode == 2, completed.stderr
    assert (
        "[BLOCKED] physical r1_live_writer requires "
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp before rclcpp init"
    ) in completed.stderr
    combined_output = completed.stdout + completed.stderr
    assert "startup fail_closed=true" not in combined_output
    assert "selected interface" not in combined_output
    assert "Unitree SDK transport initialized" not in combined_output


def test_prepare_worker_ownership_covers_launch_completion_and_shutdown():
    """The node must advertise SDK ownership for the worker's full lifetime."""
    prepare = function_body(NODE, "void on_prepare(")
    ownership_set = prepare.index("prepare_in_progress_ = true")
    async_launch = prepare.index(
        "std::async(std::launch::async", ownership_set)
    worker_call = prepare.index("transport_->prepare(", async_launch)
    launch_catch = prepare.index("catch (const std::exception", worker_call)
    launch_clear = prepare.index("prepare_in_progress_ = false", launch_catch)
    launch_stop = prepare.index("safe_stop_outputs(", launch_clear)
    assert ownership_set < async_launch < worker_call
    assert worker_call < launch_catch < launch_clear < launch_stop

    completion = function_body(NODE, "void finish_prepare_if_ready()")
    future_get = completion.index("prepare_future_.get()")
    typed_catch = completion.index("catch (const std::exception", future_get)
    unknown_catch = completion.index("catch (...)", typed_catch)
    ownership_clear = completion.index(
        "prepare_in_progress_ = false", unknown_catch)
    cancel_exchange = completion.index(
        "prepare_cancel_requested_.exchange(false)", ownership_clear)
    cancelled_branch = completion.index("if (cancelled)", cancel_exchange)
    result_branch = completion.index("if (!result.ok)", cancelled_branch)
    assert future_get < typed_catch < unknown_catch < ownership_clear
    assert ownership_clear < cancel_exchange < cancelled_branch < result_branch

    destructor = function_body(NODE, "~R1LiveWriter() override")
    shutdown_cancel = destructor.index("prepare_cancel_requested_.store(true)")
    shutdown_wait = destructor.index("prepare_future_.wait()", shutdown_cancel)
    shutdown_get = destructor.index("prepare_future_.get()", shutdown_wait)
    shutdown_clear = destructor.index(
        "prepare_in_progress_ = false", shutdown_get)
    shutdown_stop = destructor.index("safe_stop_outputs(", shutdown_clear)
    assert shutdown_cancel < shutdown_wait < shutdown_get
    assert shutdown_get < shutdown_clear < shutdown_stop


def test_cancelled_prepare_retains_stop_debt_until_joined_cleanup():
    """Cancellation after StandUp must keep its StopMove debt."""
    sdk_prepare = function_body(
        TRANSPORT, "TransportResult SdkTransport::prepare(")
    debt_set = sdk_prepare.index("prepare_maybe_active = true")
    stand_up = sdk_prepare.index("StandUp()", debt_set)
    first_poll_cancel = sdk_prepare.index(
        'return failure("R1 StandUp FSM confirmation cancelled")', stand_up)
    stable_clear = sdk_prepare.index("prepare_maybe_active = false", stand_up)
    assert debt_set < stand_up < first_poll_cancel < stable_clear
    assert sdk_prepare.count("prepare_maybe_active = false") == 1

    safe_stop = function_body(NODE, "void safe_stop_outputs(")
    cancel_request = safe_stop.index("prepare_cancel_requested_.store(true)")
    ownership_guard = safe_stop.index("!prepare_in_progress_", cancel_request)
    transport_calls = (
        "transport_->stop_locomotion()",
        "transport_->hold_head(",
        "transport_->release_head(",
    )
    assert all(
        safe_stop.index(call) > ownership_guard for call in transport_calls)

    completion = function_body(NODE, "void finish_prepare_if_ready()")
    joined = completion.index("prepare_future_.get()")
    ownership_clear = completion.index("prepare_in_progress_ = false", joined)
    cancelled_branch = completion.index("if (cancelled)", ownership_clear)
    deferred_cleanup = completion.index(
        'safe_stop_outputs("prepare_cancelled result="', cancelled_branch)
    assert joined < ownership_clear < cancelled_branch < deferred_cleanup

    sdk_stop = function_body(
        TRANSPORT, "TransportResult SdkTransport::stop_locomotion()")
    stop_move = sdk_stop.index("StopMove()")
    successful_result = sdk_stop.index("if (result == 0)", stop_move)
    debt_clear = sdk_stop.index(
        "prepare_maybe_active = false", successful_result)
    assert stop_move < successful_result < debt_clear
