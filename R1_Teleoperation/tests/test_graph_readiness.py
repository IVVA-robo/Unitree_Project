"""Typed readiness tests; fake ROS services, no Unitree clients or actuation."""

from types import SimpleNamespace
import time

import pytest

from exhibition.readiness import missing_graph, parameters_match, wait_ready


def test_graph_rejects_missing_and_conflicting_types():
    expected = [('/state', 'std_msgs/msg/Bool')]
    assert missing_graph([], [('/state', ['std_msgs/msg/Bool'])], [], expected) == []
    assert missing_graph([], [], [], expected)
    assert missing_graph([], [('/state', ['std_msgs/msg/String'])], [], expected)
    assert missing_graph([], [('/state', ['std_msgs/msg/Bool', 'std_msgs/msg/String'])], [], expected)


@pytest.mark.parametrize('values', [[], [SimpleNamespace(type=4, bool_value=False)],
                                   [SimpleNamespace(type=1, bool_value=True)]])
def test_static_policy_requires_complete_typed_boolean_values(values):
    assert not parameters_match(values, {'enable_head': False})
    assert parameters_match([SimpleNamespace(type=1, bool_value=False)], {'enable_head': False})


@pytest.mark.parametrize('head_enabled,domain', [(False, 94), (True, 95), ('false', 96)])
def test_actual_ros_graph_queries_only_read_parameters(monkeypatch, head_enabled, domain):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger

    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = Context()
    rclpy.init(context=context, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    fake = rclpy.create_node('r1_live_writer', context=context)
    reader = rclpy.create_node('r1_test_readiness', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(fake)
    executor.add_node(reader)
    motion_calls = []

    def forbidden(request, response):
        motion_calls.append(request)
        response.success = False
        return response

    fake.declare_parameter('enable_head', head_enabled)
    fake.declare_parameter('static_prepare_mode', True)
    fake.create_service(Trigger, '/r1/live_writer/prepare', forbidden)
    fake.create_publisher(Bool, '/r1/safety/kill', 10)
    try:
        started = time.monotonic()
        okay, missing = wait_ready(
            reader, executor, [('/r1/live_writer/prepare', 'std_srvs/srv/Trigger')],
            [('/r1/safety/kill', 'std_msgs/msg/Bool')],
            {'static_prepare_mode': True, 'enable_head': False}, started + 2.5)
        assert okay is (head_enabled is False), missing
        assert motion_calls == []
    finally:
        executor.shutdown(timeout_sec=0.1)
        reader.destroy_node()
        fake.destroy_node()
        context.shutdown()
