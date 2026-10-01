"""Single-participant ROS client for the final, already-authorized prepare.

This helper does not perform discovery policy or ArmSdk traffic checks.  The
reviewed ``r1-robot-prepare`` wrapper must complete those checks first and is
the only supported caller.  The helper preserves the release -> 0.3 second
propagation -> local reset -> prepare ordering while avoiding four separate
ROS CLI participants.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


SERVICES = {
    "release": "/r1/safety/set_kill",
    "reset": "/r1/live_writer/reset_kill",
    "prepare": "/r1/live_writer/prepare",
}


def prepare_confirmed(status: str, expected_rearm: str) -> bool:
    """Return whether a structured writer status confirms prepare."""
    return (
        f"prepare_rearm={expected_rearm}" in status
        and ("prepare_result=" in status or "prepared=true" in status)
    )


def prepare_failed(status: str) -> bool:
    """Return whether writer status unambiguously reports failed prepare."""
    return (
        "fail_closed reason=prepare_" in status
        or "prepare_cancelled" in status
        or (
            "prepare_in_progress=false" in status
            and "prepared=false" in status
            and "kill_clear=false" in status
        )
    )


def head_center_confirmed(status: str) -> bool:
    return any(
        marker in status
        for marker in (
            "head_auto_center=complete",
            "head_auto_center_complete=true",
            "head_tracking_seed_verified=true",
        )
    )


def head_center_failed(status: str) -> bool:
    return any(
        marker in status
        for marker in (
            "fail_closed reason=head_auto_center",
            "fail_closed reason=head_recenter",
        )
    )


class PrepareClient:
    """Own one ROS participant for all final prepare services and status."""

    def __init__(self, context: Context) -> None:
        self.node = rclpy.create_node(
            f"r1_prepare_client_{os.getpid()}", context=context
        )
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        self.cancelled = False
        self.phase = "initializing"
        self.status = ""
        self.last_printed_status = ""
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.node.create_subscription(
            String,
            "/r1/live_writer/status",
            self._on_status,
            status_qos,
        )
        self.release_client = self.node.create_client(
            SetBool, SERVICES["release"]
        )
        self.reset_client = self.node.create_client(
            Trigger, SERVICES["reset"]
        )
        self.prepare_client = self.node.create_client(
            Trigger, SERVICES["prepare"]
        )

    def _on_status(self, message: String) -> None:
        self.status = str(message.data)
        if self.status and self.status != self.last_printed_status:
            print(f"writer status: {self.status}", flush=True)
            self.last_printed_status = self.status

    def diagnostics(self) -> str:
        return f"phase={self.phase} status={self.status or '<unavailable>'}"

    def spin_until(self, predicate, deadline: float) -> bool:
        while time.monotonic() < deadline:
            if self.cancelled:
                return False
            if predicate():
                return True
            self.executor.spin_once(
                timeout_sec=min(
                    0.05, max(0.0, deadline - time.monotonic())
                )
            )
        return not self.cancelled and predicate()

    def wait_for_services(self, deadline: float) -> bool:
        self.phase = "service_discovery"
        print("[WAIT] final prepare services", flush=True)
        clients = (
            self.release_client,
            self.reset_client,
            self.prepare_client,
        )
        return self.spin_until(
            lambda: all(client.service_is_ready() for client in clients),
            deadline,
        )

    def call(self, client, request, name: str, deadline: float):
        self.phase = f"response:{name}"
        future = client.call_async(request)
        if not self.spin_until(future.done, deadline):
            raise RuntimeError(f"service timed out: {name}")
        if future.exception() is not None:
            raise RuntimeError(f"service failed: {name}: {future.exception()}")
        response = future.result()
        if response is None:
            raise RuntimeError(f"service returned no response: {name}")
        print("response:", flush=True)
        print(f"  service: {name}", flush=True)
        print(
            f"  success: {'true' if response.success else 'false'}",
            flush=True,
        )
        print(f"  message: {response.message}", flush=True)
        if not response.success:
            raise RuntimeError(
                f"service rejected request: {name}: {response.message}"
            )
        return response

    def settle_kill_release(self, deadline: float) -> None:
        self.phase = "kill_release_propagation"
        settle_deadline = time.monotonic() + 0.3
        if settle_deadline > deadline:
            raise RuntimeError("no time remains for kill-release propagation")
        if not self.spin_until(
            lambda: time.monotonic() >= settle_deadline,
            settle_deadline + 0.01,
        ):
            raise RuntimeError("kill-release propagation was interrupted")

    def close(self) -> None:
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=0.1)
        self.node.destroy_node()


def run_prepare(
    client: PrepareClient,
    *,
    deadline: float,
    expected_rearm: str,
    wait_head_auto_center: bool,
) -> int:
    if not client.wait_for_services(deadline):
        print(
            "[BLOCKED] final prepare services were not discovered: "
            + client.diagnostics(),
            file=sys.stderr,
            flush=True,
        )
        return 2

    client.call(
        client.release_client,
        SetBool.Request(data=False),
        SERVICES["release"],
        deadline,
    )
    client.settle_kill_release(deadline)
    client.call(
        client.reset_client,
        Trigger.Request(),
        SERVICES["reset"],
        deadline,
    )
    client.call(
        client.prepare_client,
        Trigger.Request(),
        SERVICES["prepare"],
        deadline,
    )

    client.status = ""
    client.phase = "prepare_confirmation"
    if not client.spin_until(
        lambda: (
            prepare_confirmed(client.status, expected_rearm)
            or prepare_failed(client.status)
        ),
        deadline,
    ):
        print(
            "[BLOCKED] timed out waiting for stable FSM prepare confirmation: "
            + client.diagnostics(),
            file=sys.stderr,
            flush=True,
        )
        return 2
    if prepare_failed(client.status):
        print(
            "[BLOCKED] writer reported failed prepare: " + client.status,
            file=sys.stderr,
            flush=True,
        )
        return 2

    if wait_head_auto_center:
        client.phase = "head_auto_center_confirmation"
        if not client.spin_until(
            lambda: (
                head_center_confirmed(client.status)
                or head_center_failed(client.status)
            ),
            deadline,
        ):
            print(
                "[BLOCKED] timed out waiting for automatic head centering: "
                + client.diagnostics(),
                file=sys.stderr,
                flush=True,
            )
            return 2
        if head_center_failed(client.status):
            print(
                "[BLOCKED] writer reported failed automatic head centering: "
                + client.status,
                file=sys.stderr,
                flush=True,
            )
            return 2

    print(
        "[OK] final prepare services and structured writer confirmation "
        "completed with one ROS participant.",
        flush=True,
    )
    return 0


def _environment_authorized() -> bool:
    exact = {
        "R1_PREPARE_CLIENT_AUTH": "post-traffic-gate",
        "ROBOT_DRY_RUN": "0",
        "ROBOT_ENABLE_ACTUATION": "1",
        "ROBOT_CONFIRM_OFF_CHARGER": "1",
        "ROBOT_CONFIRM_CLEAR_AREA": "1",
        "ROBOT_CONFIRM_ESTOP_READY": "1",
        "ROBOT_CONFIRM_COMMISSIONING": "1",
    }
    exact_match = all(
        os.environ.get(name) == value for name, value in exact.items()
    )
    return exact_match and len(
        os.environ.get("ROBOT_COMMISSIONING_TOKEN", "")
    ) >= 16


def main(argv=None) -> int:
    sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--expected-rearm",
        choices=("ready", "await_deadman_release"),
        required=True,
    )
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--wait-head-auto-center", action="store_true")
    arguments = parser.parse_args(argv)
    if not 1.0 <= arguments.timeout_sec <= 120.0:
        parser.error("--timeout-sec must be 1..120")
    if not _environment_authorized():
        print(
            "[BLOCKED] prepare client requires the reviewed post-traffic-gate "
            "wrapper and all live acknowledgements.",
            file=sys.stderr,
        )
        return 2

    context = Context()
    client = None
    interrupted = False

    def cancel(_signum, _frame):
        nonlocal interrupted
        interrupted = True
        if client is not None:
            client.cancelled = True

    previous_term = signal.signal(signal.SIGTERM, cancel)
    try:
        rclpy.init(
            context=context,
            signal_handler_options=SignalHandlerOptions.NO,
        )
        client = PrepareClient(context)
        client.cancelled = interrupted
        return run_prepare(
            client,
            deadline=time.monotonic() + arguments.timeout_sec,
            expected_rearm=arguments.expected_rearm,
            wait_head_auto_center=arguments.wait_head_auto_center,
        )
    except (KeyboardInterrupt, SystemExit):
        return 130
    except Exception as exception:
        print(
            f"[BLOCKED] final prepare client failed: {exception}",
            file=sys.stderr,
            flush=True,
        )
        return 2
    finally:
        if client is not None:
            if interrupted:
                print(
                    "[BLOCKED] final prepare client interrupted: "
                    + client.diagnostics(),
                    flush=True,
                )
            client.close()
        if context.ok():
            context.shutdown()
        signal.signal(signal.SIGTERM, previous_term)


if __name__ == "__main__":
    raise SystemExit(main())
