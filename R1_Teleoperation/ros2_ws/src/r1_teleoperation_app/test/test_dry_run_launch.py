"""Contract tests for the standalone, actuator-free dry-run launch."""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / 'launch'
    / 'r1_teleop_dry_run.launch.py'
)


def _load_launch_module():
    """Load the source launch module without resolving ROS package shares."""
    spec = importlib.util.spec_from_file_location('r1_dry_run_launch', LAUNCH_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _text(value):
    """Return the scalar text represented by a launch argument value."""
    if isinstance(value, (list, tuple)):
        assert len(value) == 1
        value = value[0]
    return str(getattr(value, 'text', value))


def test_dry_run_launch_is_fail_closed_without_physical_robot():
    """Verify safe defaults and forced environment interlocks by contract."""
    description = _load_launch_module().generate_launch_description()
    assert isinstance(description, LaunchDescription)

    arguments = {
        action.name: _text(action.default_value)
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert arguments['ros_domain_id'] == '89'
    assert arguments['start_bridge'] == 'true'
    assert arguments['start_head'] == 'true'
    assert arguments['start_locomotion'] == 'true'
    assert arguments['locomotion_mode'] == 'slow-safe'
    assert arguments['start_arm_pipeline'] == 'false'
    assert arguments['vr_allowed_source_ip'] == ''

    environment = {
        _text(action.name): _text(action.value)
        for action in description.entities
        if isinstance(action, SetEnvironmentVariable)
    }
    assert environment['ROS_LOCALHOST_ONLY'] == '1'
    assert environment['ROBOT_DRY_RUN'] == '1'
    assert environment['ROBOT_ENABLE_ACTUATION'] == '0'
    assert environment['ROBOT_CONFIRM_OFF_CHARGER'] == '0'
    assert environment['ROBOT_CONFIRM_CLEAR_AREA'] == '0'
    assert environment['ROBOT_CONFIRM_ESTOP_READY'] == '0'
