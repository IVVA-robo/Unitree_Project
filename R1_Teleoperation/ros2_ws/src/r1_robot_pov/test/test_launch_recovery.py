"""Validate process-local Robot POV crash recovery."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.actions import Node


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / 'launch'
    / 'robot_pov.launch.py'
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location('robot_pov_launch', LAUNCH_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_video_child_respawns_without_restarting_parent_graph():
    module = _load_launch_module()
    context = LaunchContext()
    context.launch_configurations.update({
        'env_file': '/tmp/robot-pov-test.env',
        'source': 'mock',
        'profile': 'low-latency',
        'transport': 'mjpeg',
        'layout': 'mono',
        'host': '127.0.0.1',
        'port': '18080',
        'enable_mdns': 'false',
        'enable_discovery': 'false',
    })

    actions = module._video_only_node(context)

    assert len(actions) == 1
    assert isinstance(actions[0], Node)
    assert actions[0]._ExecuteLocal__respawn is True
    assert actions[0]._ExecuteLocal__respawn_delay == 2.0
