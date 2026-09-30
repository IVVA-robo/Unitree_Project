"""Static safety contract for the physical-writer launch composition."""

import importlib.util
import os
import platform
from pathlib import Path
from types import SimpleNamespace

import pytest

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / 'launch'
    / 'r1_teleop_live.launch.py'
)
PACKAGE_XML = Path(__file__).resolve().parents[1] / 'package.xml'
PROJECT_LIVE_WRAPPER = LAUNCH_PATH.parents[4] / 'scripts' / 'r1-live-session'
BRIDGE_LAUNCH = (
    LAUNCH_PATH.parents[2]
    / 'vr_teleop_bridge'
    / 'launch'
    / 'vr_bridge.launch.py'
)
KINEMATICS_LAUNCH = (
    LAUNCH_PATH.parents[2]
    / 'r1_kinematics_control'
    / 'launch'
    / 'r1_kinematics_control.launch.py'
)


def _load_launch_module():
    """Load the launch source without starting or resolving any package."""
    spec = importlib.util.spec_from_file_location(
        'r1_live_launch', LAUNCH_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _text(value):
    """Return the scalar text represented by a launch substitution."""
    if isinstance(value, (list, tuple)):
        assert len(value) == 1
        value = value[0]
    return str(getattr(value, 'text', value))


def _architecture_directory():
    """Return the Unitree directory name for the test host."""
    return {
        'x86_64': 'x86_64',
        'amd64': 'x86_64',
        'aarch64': 'aarch64',
        'arm64': 'aarch64',
    }[platform.machine().lower()]


def _fake_sdk_environment(tmp_path):
    """Create a file-only fake vendor runtime; no library is ever loaded."""
    sdk_root = tmp_path / 'unitree_sdk2'
    vendor = sdk_root / 'thirdparty' / 'lib' / _architecture_directory()
    vendor.mkdir(parents=True)
    for name in ('libddsc.so.0', 'libddscxx.so.0'):
        (vendor / name).touch()
    return sdk_root, vendor, {
        'R1_UNITREE_SDK_ROOT': str(sdk_root),
        'LD_LIBRARY_PATH': '/opt/ros/humble/lib:/tmp/unrelated',
    }


def test_live_launch_defaults_cannot_send_commands():
    """No default argument may construct a physical SDK reader or writer."""
    description = _load_launch_module().generate_launch_description()
    assert isinstance(description, LaunchDescription)
    arguments = {
        action.name: _text(action.default_value)
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert arguments['transport'] == 'mock'
    assert arguments['send_commands'] == 'false'
    assert arguments['enable_head'] == 'false'
    assert arguments['enable_arms'] == 'false'
    assert arguments['enable_locomotion'] == 'false'
    assert arguments['enable_prepare'] == 'false'
    assert arguments['prepare_enter_locomotion'] == 'false'
    assert arguments['head_recenter_enabled'] == 'false'
    assert arguments['head_recenter_confirmed'] == 'false'
    assert arguments['head_ownership_probe_only'] == 'true'
    assert arguments['head_ownership_probe_confirmed'] == 'false'
    assert arguments['start_safety_supervisor'] == 'true'
    assert arguments['start_readonly_reader'] == 'false'
    assert arguments['commissioning_confirmed'] == 'false'
    assert arguments['commissioning_token'] == ''
    assert arguments['vr_source_ip'] == ''
    assert arguments['discovery_port'] == '9091'
    assert arguments['motor_health_topic'] == \
        '/r1/sdk_transport/motors_healthy'
    assert arguments['shoulder_height_offset_m'] == '0.0'
    assert arguments['shoulder_forward_offset_m'] == '-0.02'
    assert arguments['shoulder_width_m'] == '0.40'
    assert arguments['arm_motion_scale'] == '1.15'
    assert arguments['max_forward_mps'] == '0.20'
    assert arguments['max_lateral_mps'] == '0.12'
    assert arguments['max_yaw_rps'] == '0.35'
    environment = {
        _text(action.name): _text(action.value)
        for action in description.entities
        if isinstance(action, SetEnvironmentVariable)
    }
    assert environment['ROS_LOCALHOST_ONLY'] == '1'
    source = LAUNCH_PATH.read_text()
    assert "'head_ownership_probe_only': LaunchConfiguration(" in source
    assert "'head_ownership_probe_confirmed': LaunchConfiguration(" in source
    assert source.count("'motor_health_topic': motor_health_topic") == 2
    assert 'ROBOT_ENABLE_ACTUATION' not in source
    assert 'ROBOT_CONFIRM_OFF_CHARGER' not in source


def test_readonly_sdk_reader_uses_the_same_explicit_opt_in():
    """The include and its sdk_enabled parameter share the false default."""
    description = _load_launch_module().generate_launch_description()
    reader_includes = [
        action for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
        and 'r1_sdk_transport' in str(
            action.launch_description_source.location
        )
    ]
    assert len(reader_includes) == 1
    reader = reader_includes[0]
    assert isinstance(reader.condition, IfCondition)

    arguments = dict(reader.launch_arguments)
    sdk_enabled = arguments['sdk_enabled']
    assert isinstance(sdk_enabled, LaunchConfiguration)
    assert _text(sdk_enabled.variable_name) == 'start_readonly_reader'

    predicate = vars(reader.condition)['_IfCondition__predicate_expression']
    assert predicate == [sdk_enabled]


def test_staged_live_wrapper_explicitly_owns_reader_opt_in():
    """Only the reviewed wrapper may opt in after its mandatory preflight."""
    wrapper = PROJECT_LIVE_WRAPPER.read_text(encoding='utf-8')
    assert 'START_READER=true' in wrapper
    assert 'start_readonly_reader:="${START_READER}"' in wrapper
    assert wrapper.index('"${SCRIPT_DIR}/r1-sdk-preflight"') \
        < wrapper.index('start_readonly_reader:="${START_READER}"')


def test_exhibition_tuning_is_validated_and_forwarded_end_to_end():
    """Panel tuning cannot raise the reviewed physical speed ceilings."""
    wrapper = PROJECT_LIVE_WRAPPER.read_text(encoding='utf-8')
    launch = LAUNCH_PATH.read_text(encoding='utf-8')

    for required in (
        'R1_SHOULDER_HEIGHT_OFFSET_M',
        'R1_SHOULDER_FORWARD_OFFSET_M',
        'R1_SHOULDER_WIDTH_M',
        'R1_ARM_MOTION_SCALE',
        'R1_TURN_SENSITIVITY',
        'R1_LEG_SPEED_SCALE',
        '0.20 * leg_scale',
        '0.12 * leg_scale',
        '0.35 * turn_scale',
        'shoulder_height_offset_m:="${SHOULDER_HEIGHT_OFFSET_M}"',
        'shoulder_forward_offset_m:="${SHOULDER_FORWARD_OFFSET_M}"',
        'shoulder_width_m:="${SHOULDER_WIDTH_M}"',
        'arm_motion_scale:="${ARM_MOTION_SCALE}"',
        'max_forward_mps:="${MAX_FORWARD_MPS}"',
        'max_lateral_mps:="${MAX_LATERAL_MPS}"',
        'max_yaw_rps:="${MAX_YAW_RPS}"',
    ):
        assert required in wrapper

    for argument in (
        'shoulder_height_offset_m',
        'shoulder_forward_offset_m',
        'shoulder_width_m',
        'arm_motion_scale',
    ):
        assert f"'{argument}': LaunchConfiguration(" in launch
        assert f"                        '{argument}'" in launch
    for argument in (
        'max_forward_mps',
        'max_lateral_mps',
        'max_yaw_rps',
    ):
        assert launch.count(f"LaunchConfiguration('{argument}')") >= 2

    bridge = BRIDGE_LAUNCH.read_text(encoding='utf-8')
    for argument in ('max_forward_mps', 'max_lateral_mps', 'max_yaw_rps'):
        assert f"DeclareLaunchArgument(\n            '{argument}'" in bridge
        assert f"'{argument}': ParameterValue(" in bridge

    kinematics = KINEMATICS_LAUNCH.read_text(encoding='utf-8')
    for argument in (
        'shoulder_height_offset_m',
        'shoulder_forward_offset_m',
        'shoulder_width_m',
        'arm_motion_scale',
        'max_forward_mps',
        'max_lateral_mps',
        'max_yaw_rps',
    ):
        assert argument in kinematics


def test_loader_guard_runs_before_every_process_action():
    """A direct launch must establish its DDS ABI boundary before children."""
    description = _load_launch_module().generate_launch_description()
    entities = list(description.entities)
    guards = [
        index for index, action in enumerate(entities)
        if isinstance(action, OpaqueFunction)
    ]
    processes = [
        index for index, action in enumerate(entities)
        if isinstance(action, (IncludeLaunchDescription, TimerAction))
    ]
    assert len(guards) == 1
    assert processes
    assert guards[0] < min(processes)


def test_package_declares_the_required_fastdds_runtime():
    """Installing the app must also install the pinned ROS middleware."""
    package_xml = PACKAGE_XML.read_text(encoding='utf-8')
    assert '<exec_depend>rmw_fastrtps_cpp</exec_depend>' in package_xml


def test_direct_launch_pins_fastdds_and_complete_vendor_pair(
        tmp_path, monkeypatch):
    """Stale ROS loader settings are replaced before any SDK-linked child."""
    module = _load_launch_module()
    sdk_root, vendor, source_environment = _fake_sdk_environment(tmp_path)
    fastdds = tmp_path / 'librmw_fastrtps_cpp.so'
    fastdds.touch()
    monkeypatch.setattr(module, '_FAST_DDS_RMW_LIBRARY', fastdds)
    source_environment.update({
        'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
        'R1_PHYSICAL_SDK_SESSION': '0',
    })

    actions = module._configure_physical_runtime(
        SimpleNamespace(environment=source_environment)
    )
    pinned = {
        _text(action.name): _text(action.value)
        for action in actions
        if isinstance(action, SetEnvironmentVariable)
    }
    assert pinned['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
    assert pinned['R1_PHYSICAL_SDK_SESSION'] == '1'
    assert pinned['R1_UNITREE_SDK_ROOT'] == str(sdk_root)
    assert pinned['R1_UNITREE_DDS_LIB_DIR'] == str(vendor)
    loader_paths = pinned['LD_LIBRARY_PATH'].split(os.pathsep)
    assert loader_paths[0] == str(vendor)
    assert loader_paths[1:] == [
        '/opt/ros/humble/lib', '/tmp/unrelated'
    ]


@pytest.mark.parametrize('missing_library', [
    'libddsc.so.0',
    'libddscxx.so.0',
])
def test_direct_launch_blocks_an_incomplete_vendor_pair(
        tmp_path, monkeypatch, missing_library):
    """Neither half of a mixed CycloneDDS pair may reach process startup."""
    module = _load_launch_module()
    _, vendor, source_environment = _fake_sdk_environment(tmp_path)
    fastdds = tmp_path / 'librmw_fastrtps_cpp.so'
    fastdds.touch()
    monkeypatch.setattr(module, '_FAST_DDS_RMW_LIBRARY', fastdds)
    (vendor / missing_library).unlink()

    with pytest.raises(RuntimeError, match=r'^\[BLOCKED\].*incomplete'):
        module._physical_runtime_environment(source_environment)


def test_direct_launch_blocks_missing_fastdds_before_process_start(
        tmp_path, monkeypatch):
    """A host without Fast DDS cannot fall back to ROS CycloneDDS."""
    module = _load_launch_module()
    _, _, source_environment = _fake_sdk_environment(tmp_path)
    monkeypatch.setattr(
        module, '_FAST_DDS_RMW_LIBRARY', tmp_path / 'missing-fastdds.so'
    )

    with pytest.raises(RuntimeError, match=r'^\[BLOCKED\].*Fast DDS'):
        module._physical_runtime_environment(source_environment)


def test_direct_launch_rejects_relative_sdk_root(tmp_path, monkeypatch):
    """Loader paths may not depend on the launch process working directory."""
    module = _load_launch_module()
    fastdds = tmp_path / 'librmw_fastrtps_cpp.so'
    fastdds.touch()
    monkeypatch.setattr(module, '_FAST_DDS_RMW_LIBRARY', fastdds)

    with pytest.raises(RuntimeError, match=r'^\[BLOCKED\].*absolute'):
        module._physical_runtime_environment({
            'R1_UNITREE_SDK_ROOT': 'relative/unitree_sdk2',
        })
