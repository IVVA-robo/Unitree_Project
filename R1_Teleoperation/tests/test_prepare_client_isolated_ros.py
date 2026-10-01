"""Prepare-client integration only in a loopback-only network namespace."""

import os
import socket
import threading
import time

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("R1_ISOLATED_ROS_TEST") != "1",
    reason="requires dedicated loopback-only network namespace",
)


def test_final_prepare_uses_one_client_and_preserves_service_order():
    assert {name for _, name in socket.if_nameindex()} == {"lo"}
    assert os.environ.get("ROS_DOMAIN_ID") == "231"

    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger

    from exhibition.prepare_client import PrepareClient, run_prepare

    context = Context()
    rclpy.init(context=context)
    fake = rclpy.create_node("offline_fake_prepare_writer", context=context)
    group = ReentrantCallbackGroup()
    status_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    status_publisher = fake.create_publisher(
        String, "/r1/live_writer/status", status_qos
    )
    events = []
    prepared = {"value": False}

    def release(request, response):
        events.append(("release", time.monotonic(), request.data))
        response.success = request.data is False
        response.message = "all live environment interlocks are set"
        return response

    def reset(_request, response):
        events.append(("reset", time.monotonic(), None))
        response.success = bool(events and events[0][0] == "release")
        response.message = "local kill latch reset"
        return response

    def prepare(_request, response):
        events.append(("prepare", time.monotonic(), None))
        prepared["value"] = True
        response.success = True
        response.message = "prepare started asynchronously"
        return response

    def publish_status():
        text = (
            "prepare_result=R1 StandUp confirmed by stable FSM 4 samples=5; "
            "prepare_rearm=ready"
            if prepared["value"]
            else "prepare_in_progress=false prepared=false kill_clear=true"
        )
        status_publisher.publish(String(data=text))

    services = (
        fake.create_service(
            SetBool,
            "/r1/safety/set_kill",
            release,
            callback_group=group,
        ),
        fake.create_service(
            Trigger,
            "/r1/live_writer/reset_kill",
            reset,
            callback_group=group,
        ),
        fake.create_service(
            Trigger,
            "/r1/live_writer/prepare",
            prepare,
            callback_group=group,
        ),
    )
    timer = fake.create_timer(0.05, publish_status, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(fake)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    client = PrepareClient(context)
    started = time.monotonic()
    try:
        assert run_prepare(
            client,
            deadline=started + 5.0,
            expected_rearm="ready",
            wait_head_auto_center=False,
        ) == 0
        assert [event[0] for event in events] == [
            "release", "reset", "prepare"
        ]
        assert events[0][2] is False
        assert events[1][1] - events[0][1] >= 0.28
        assert time.monotonic() - started < 5.0
    finally:
        client.close()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        fake.destroy_timer(timer)
        for service in services:
            fake.destroy_service(service)
        fake.destroy_node()
        context.shutdown()
