"""Opt-in ROS integration ONLY inside a network namespace with loopback alone."""
import os
from pathlib import Path
import runpy
import socket
import threading
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('R1_ISOLATED_ROS_TEST') != '1',
                                reason='requires dedicated loopback-only network namespace')


@pytest.fixture
def graph():
    # sysfs may remain mounted from the parent namespace; query the current
    # kernel network namespace, not that inherited directory view.
    assert {name for _, name in socket.if_nameindex()} == {'lo'}, 'refusing real network interfaces'
    assert os.environ.get('ROS_DOMAIN_ID') == '231'
    import rclpy
    from rclpy.context import Context
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger

    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('offline_fake_bridge', context=context)
    active = node.create_publisher(Bool, '/vr/teleop/active', 5)
    armed = node.create_publisher(Bool, '/vr/teleop/session_armed',
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    writer = node.create_publisher(Bool, '/r1/live_writer/armed', 5)
    kill = node.create_publisher(Bool, '/r1/safety/kill', 5)
    state = {'active': False, 'armed': True, 'kill': False, 'pause_calls': 0,
             'delay_first_pause': False, 'publish': True, 'resume_calls': 0}

    def publish():
        if state['publish']:
            active.publish(Bool(data=state['active']))
            armed.publish(Bool(data=state['armed']))
            writer.publish(Bool(data=state['active'] and not state['kill']))
            kill.publish(Bool(data=state['kill']))

    def pause(_request, response):
        state['pause_calls'] += 1
        state['active'] = False
        if state['delay_first_pause'] and state['pause_calls'] == 1:
            time.sleep(2.2)
        response.success, response.message = True, 'session_paused'
        return response

    def resume(_request, response):
        state['resume_calls'] += 1
        state['active'] = True
        response.success, response.message = True, 'session_resumed'
        return response

    group = ReentrantCallbackGroup()
    services = [node.create_service(Trigger, '/vr/teleop/pause_session', pause, callback_group=group),
                node.create_service(Trigger, '/vr/teleop/resume_session', resume, callback_group=group)]
    timer = node.create_timer(0.05, publish, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    module = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts/r1-exhibition-ros-client'))
    client = module['ExhibitionClient'](context)
    try:
        yield client, state, module['run_gate']
    finally:
        client.close()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        node.destroy_node()
        context.shutdown()


def test_repeated_run_lock_uses_real_local_ros_messages(graph):
    client, state, run_gate = graph
    for _ in range(5):
        assert run_gate(client, 'resume_ready', time.monotonic() + 8) == 0
        assert state['active'] is True
        assert run_gate(client, 'pause', time.monotonic() + 8) == 0
        assert state['active'] is False
    assert state['resume_calls'] == 5
    assert state['pause_calls'] == 5


def test_delayed_lock_reply_retries_without_movement_or_extended_budget(graph):
    client, state, run_gate = graph
    state.update(active=True, delay_first_pause=True)
    started = time.monotonic()
    assert run_gate(client, 'pause', started + 8) == 0
    assert time.monotonic() - started < 8
    assert state['pause_calls'] == 2
    assert state['resume_calls'] == 0
    assert state['active'] is False


def test_kill_never_auto_resumes_on_local_ros(graph):
    client, state, run_gate = graph
    state['kill'] = True
    assert run_gate(client, 'resume_ready', time.monotonic() + 8) == 3
    assert state['resume_calls'] == 0


def test_missing_confirmation_cannot_pass_on_service_reply_alone(graph):
    client, state, run_gate = graph
    state['publish'] = False
    assert run_gate(client, 'pause', time.monotonic() + 4) == 2
    assert state['active'] is False
