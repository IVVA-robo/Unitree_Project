"""Validate the unified launch description without starting any process."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / 'launch'
    / 'r1_teleoperation.launch.py'
)


def _load_launch_module():
    """Load the launch module directly from the source tree."""
    spec = importlib.util.spec_from_file_location('r1_unified_launch', LAUNCH_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_unified_launch_declares_safe_defaults():
    """Ensure the one-command launch exposes the expected safety defaults."""
    description = _load_launch_module().generate_launch_description()
    assert isinstance(description, LaunchDescription)

    def default_text(action):
        value = action.default_value
        if isinstance(value, list) and len(value) == 1:
            value = value[0]
        return str(getattr(value, 'text', value))

    arguments = {
        action.name: default_text(action)
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert arguments['hardware_enabled'] == 'false'
    assert arguments['ros_localhost_only'] == '1'
    assert arguments['headset_relative_enabled'] == 'true'
    assert arguments['video_source'] == 'ros'
    assert arguments['start_bridge'] == 'true'
    assert arguments['start_simulation'] == 'true'
    assert arguments['start_video'] == 'true'
