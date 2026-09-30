"""Fake read-only readers on isolated ROS domains; never load Unitree SDK."""

from types import SimpleNamespace
import time

import pytest

from exhibition.reader_probe import POLICY, finite_feedback, local_reader_exists, probe_reader


def feedback(**changes):
    return SimpleNamespace(**dict({'name': [f'joint_{i}' for i in range(26)],
                                   'position': [0.0] * 26}, **changes))


def test_feedback_rejects_short_duplicate_missing_and_nonfinite_samples():
    assert finite_feedback(feedback())
    for values in ({'name': ['joint'] * 26}, {'name': [''] * 26}, {'position': []},
                   {'position': [float('nan')] * 26}, {'position': [float('inf')] * 26}):
        assert not finite_feedback(feedback(**values))


def test_process_guard_is_domain_scoped_and_rejects_unknown_domain(tmp_path):
    reader = tmp_path / '123'
    reader.mkdir()
    (reader / 'cmdline').write_bytes(b'/opt/test/r1_sdk_transport\0--ros-args\0')
    (reader / 'environ').write_bytes(b'ROS_DOMAIN_ID=88\0')
    assert local_reader_exists(88, tmp_path)
    assert not local_reader_exists(97, tmp_path)
    (reader / 'environ').write_bytes(b'ROS_DOMAIN_ID=bad\0')
    assert local_reader_exists(88, tmp_path)


@pytest.mark.parametrize('kind,domain,expected', [
    ('healthy', 97, 10), ('absent', 98, 0), ('unsafe', 99, 2),
    ('stale', 100, 2), ('nan', 101, 2), ('undiscovered', 102, 2),
])
def test_reader_requires_real_typed_policy_and_fresh_sample(monkeypatch, kind, domain, expected):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    import exhibition.reader_probe as module

    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setattr(module, 'local_reader_exists', lambda _: kind == 'undiscovered')
    context = Context()
    rclpy.init(context=context, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor(context=context)
    probe = rclpy.create_node('r1_test_reader_probe', context=context)
    executor.add_node(probe)
    fake = None
    try:
        if kind not in {'absent', 'undiscovered'}:
            fake = rclpy.create_node('r1_sdk_transport', context=context)
            executor.add_node(fake)
            for name, value in POLICY.items():
                fake.declare_parameter(name, True if kind == 'unsafe' and name == 'arm_writer_enabled' else value)
            publisher = fake.create_publisher(JointState, '/r1/sdk/joint_states', 10)
            message = JointState()
            message.name = feedback().name
            message.position = [float('nan') if kind == 'nan' else 0.0] * 26
            if kind != 'stale':
                fake.create_timer(0.02, lambda: publisher.publish(message))
        code, detail = probe_reader(probe, executor, time.monotonic() + 2.5,
                                    domain, absent_grace=0.6)
        assert code == expected, detail
    finally:
        executor.shutdown(timeout_sec=0.1)
        probe.destroy_node()
        if fake is not None:
            fake.destroy_node()
        context.shutdown()
