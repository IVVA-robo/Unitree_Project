"""Offline process tests for the r1_sdk_transport DDS ABI guard."""

import os
from pathlib import Path
import shutil
import subprocess
import sysconfig

import pytest


_REQUIRED_BUILD_ENV = (
    'R1_SDK_TRANSPORT_BINARY',
    'R1_ARM_SDK_OBSERVER_BINARY',
    'R1_MOTION_SWITCHER_PROBE_BINARY',
    'R1_LOCO_STATE_PROBE_BINARY',
    'R1_WIRELESS_CONTROLLER_OBSERVER_BINARY',
    'R1_WIRELESS_CONTROLLER_PROBE_BINARY',
    'R1_SDK_TRANSPORT_VENDOR_DDS_DIR',
)
_MISSING_BUILD_ENV = [name for name in _REQUIRED_BUILD_ENV if not os.environ.get(name)]
if _MISSING_BUILD_ENV:
    pytest.skip(
        'SDK ABI guard requires the CMake-built test environment: '
        + ', '.join(_MISSING_BUILD_ENV),
        allow_module_level=True,
    )


BINARY = Path(os.environ['R1_SDK_TRANSPORT_BINARY'])
OBSERVER_BINARY = Path(os.environ['R1_ARM_SDK_OBSERVER_BINARY'])
MOTION_SWITCHER_PROBE_BINARY = Path(
    os.environ['R1_MOTION_SWITCHER_PROBE_BINARY'])
LOCO_STATE_PROBE_BINARY = Path(os.environ['R1_LOCO_STATE_PROBE_BINARY'])
WIRELESS_CONTROLLER_OBSERVER_BINARY = Path(
    os.environ['R1_WIRELESS_CONTROLLER_OBSERVER_BINARY'])
WIRELESS_CONTROLLER_PROBE_BINARY = Path(
    os.environ['R1_WIRELESS_CONTROLLER_PROBE_BINARY'])
VENDOR_DDS_DIR = Path(os.environ['R1_SDK_TRANSPORT_VENDOR_DDS_DIR'])
MATCH_STUB_SOURCE = Path(__file__).with_name('match_discovery_stub.c')


def run_binary(arguments, library_dirs, rmw='rmw_fastrtps_cpp'):
    """Run the transport with a controlled RMW and loader search path."""
    environment = os.environ.copy()
    environment['ROS_LOCALHOST_ONLY'] = '1'
    requested = [str(path) for path in library_dirs]
    inherited = environment.get('LD_LIBRARY_PATH', '').split(os.pathsep)
    remainder = [
        entry for entry in inherited if entry and entry not in requested
    ]
    environment['LD_LIBRARY_PATH'] = os.pathsep.join([*requested, *remainder])
    if rmw is None:
        environment.pop('RMW_IMPLEMENTATION', None)
    else:
        environment['RMW_IMPLEMENTATION'] = rmw
    return subprocess.run(
        [str(BINARY), *arguments],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
        env=environment,
    )


def run_guard(library_dirs, rmw='rmw_fastrtps_cpp'):
    """Run only the pre-init guard; this mode never initializes ROS or SDK."""
    return run_binary(['--abi-check-only'], library_dirs, rmw)


def combined_output(result):
    return result.stdout + result.stderr


@pytest.fixture(scope='module')
def match_discovery_stub(tmp_path_factory):
    """Build a private getter-only fixture; it creates no DDS publisher."""
    output = tmp_path_factory.mktemp('match_stub') / 'match_discovery_stub.so'
    subprocess.run(
        [
            'cc', '-std=c11', '-D_GNU_SOURCE', '-shared', '-fPIC',
            str(MATCH_STUB_SOURCE), '-o', str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )
    return output


def run_observer_with_match_stub(stub, *, expected, mode='stable'):
    """Exercise match classification on loopback with getter-only evidence."""
    environment = os.environ.copy()
    environment['LD_PRELOAD'] = str(stub)
    environment['LD_LIBRARY_PATH'] = os.pathsep.join([
        str(VENDOR_DDS_DIR), environment.get('LD_LIBRARY_PATH', '')
    ]).rstrip(os.pathsep)
    environment['R1_MATCH_STUB_EXPECTED'] = str(expected)
    environment['R1_MATCH_STUB_MODE'] = mode
    return subprocess.run(
        [
            str(OBSERVER_BINARY), '--interface', 'lo',
            '--duration-sec', '2.1', '--fresh-sec', '0.5',
            '--action-fresh-sec', '0.5', '--min-samples', '3',
            '--expected-writers', str(expected),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
        env=environment,
    )


def test_arm_sdk_observer_help_is_inert_and_documents_gate_statuses():
    """Help must exit before ChannelFactory and expose machine-usable codes."""
    result = subprocess.run(
        [str(OBSERVER_BINARY), '--help'],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, combined_output(result)
    assert (
        'Read only rt/arm_sdk and rt/arm/action/state for a bounded interval'
        in result.stdout
    )
    assert '10=fresh active traffic' in result.stdout
    assert '11=traffic observed but not fresh at exit' in result.stdout
    assert '12=action state missing' in result.stdout
    assert '15=typed writer set absent/unexpected/unstable' in result.stdout
    assert 'requires the phase-specific typed rt/arm_sdk writer set' in result.stdout


def test_motion_switcher_probe_help_is_inert_and_read_only():
    """Help exits before DDS initialization and states the API boundary."""
    result = subprocess.run(
        [str(MOTION_SWITCHER_PROBE_BINARY), '--help'],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, combined_output(result)
    assert 'Read-only Unitree MotionSwitcher diagnostic.' in result.stdout
    assert 'internal Noop API 2 handshake' in result.stdout
    assert 'then sends one explicit API 1001 query' in result.stdout
    assert '--get-silent adds one API 1005 query' in result.stdout
    assert 'No state-changing MotionSwitcher API (1002, 1003, or 1004)' in result.stdout


def test_loco_state_probe_help_is_inert_and_read_only():
    """Help exits before DDS initialization and states the narrow API boundary."""
    result = subprocess.run(
        [str(LOCO_STATE_PROBE_BINARY), '--help'],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, combined_output(result)
    assert 'Read-only Unitree R1 locomotion-state diagnostic.' in result.stdout
    assert 'internal Noop API 2 handshake' in result.stdout
    assert 'then sends one explicit API 7001 query' in result.stdout
    assert '--get-fsm-mode adds one API 7002 query' in result.stdout
    assert (
        'No state-changing locomotion API (7101, 7105, or 7107)'
        in result.stdout
    )


def test_wireless_controller_observer_help_is_inert_and_read_only():
    """Help exits before DDS initialization and documents its receive boundary."""
    result = subprocess.run(
        [str(WIRELESS_CONTROLLER_OBSERVER_BINARY), '--help'],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, combined_output(result)
    assert (
        'Read only rt/wirelesscontroller for a bounded interval'
        in result.stdout
    )
    assert 'No DDS writer, RPC client, locomotion call' in result.stdout
    assert '10=typed publisher matched but silent' in result.stdout
    assert '11=no typed publisher matched' in result.stdout


def test_wireless_controller_probe_help_is_inert_and_live_gated():
    """Help exits before DDS and documents the exact bounded live pulse."""
    result = subprocess.run(
        [str(WIRELESS_CONTROLLER_PROBE_BINARY), '--help'],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, combined_output(result)
    assert 'Default: publish zero axes/buttons' in result.stdout
    assert 'ly=0.15 pulse for 0.40 s' in result.stdout
    assert 'BOUNDED_R1_FORWARD_STICK_PROBE' in result.stdout
    assert 'R2+A Running Mode combination' in result.stdout
    assert 'BOUNDED_R1_RUNNING_AND_FORWARD_STICK_PROBE' in result.stdout


def test_wireless_controller_probe_accepts_exact_ack_before_runtime_checks():
    """The documented acknowledgement reaches interface validation."""
    result = subprocess.run(
        [
            str(WIRELESS_CONTROLLER_PROBE_BINARY),
            '--interface', 'nosuch0',
            '--forward-probe', '--acknowledge',
            'BOUNDED_R1_FORWARD_STICK_PROBE',
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 21, combined_output(result)
    assert 'network interface does not exist: nosuch0' in combined_output(result)
    assert 'invalid --acknowledge value' not in combined_output(result)


@pytest.mark.parametrize('expected', [1, 2])
def test_observer_accepts_stable_phase_specific_typed_set_offline(
        match_discovery_stub, expected):
    """Stable mocked discovery reaches match-clear but action state still blocks."""
    result = run_observer_with_match_stub(
        match_discovery_stub, expected=expected)
    output = combined_output(result)
    assert result.returncode == 12, output
    assert 'ARM_SDK_MATCH status=TYPED_STABLE' in output
    assert f'expected_writers={expected}' in output
    assert f'current_count={expected}' in output
    assert f'handle_count={expected}' in output
    assert f'total_count={expected}' in output
    assert 'metadata_valid=true churn=false' in output
    assert 'ARM_SDK_TRAFFIC status=CLEAR_NO_SAMPLES' in output
    assert 'ARM_ACTION_STATE status=BLOCKED_MISSING_OR_STALE' in output


def test_observer_blocks_handle_and_history_churn_offline(
        match_discovery_stub):
    """Any post-baseline match change gets the dedicated fail-closed exit."""
    result = run_observer_with_match_stub(
        match_discovery_stub, expected=1, mode='churn')
    output = combined_output(result)
    assert result.returncode == 15, output
    assert 'ARM_SDK_MATCH status=BLOCKED_COUNT_OR_CHURN' in output
    assert 'max_total_count=2' in output
    assert 'churn=true' in output


def resolved_library(name):
    candidates = sorted(VENDOR_DDS_DIR.glob(f'{name}.so.0*'))
    assert candidates, f'missing {name} under {VENDOR_DDS_DIR}'
    return candidates[0].resolve(strict=True)


def ros_cyclonedds_directory():
    """Find the ROS CycloneDDS C library used in the original mixed ABI."""
    multiarch = sysconfig.get_config_var('MULTIARCH')
    candidates = []
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if not prefix:
            continue
        lib_dir = Path(prefix) / 'lib'
        if multiarch:
            candidates.append(lib_dir / multiarch / 'libddsc.so.0')
        candidates.append(lib_dir / 'libddsc.so.0')
    for candidate in candidates:
        if candidate.exists():
            return candidate.parent
    pytest.skip('ROS CycloneDDS libddsc.so.0 is not installed')


def test_guard_accepts_compiled_vendor_pair_without_initializing_ros():
    result = run_guard([VENDOR_DDS_DIR])
    assert result.returncode == 0, combined_output(result)
    assert (
        'DDS ABI guard accepted explicit FastRTPS and vendor libraries'
        in result.stdout
    )


def test_guard_requires_explicit_fastrtps():
    unset = run_guard([VENDOR_DDS_DIR], rmw=None)
    assert unset.returncode != 0
    assert (
        'requires explicit RMW_IMPLEMENTATION=rmw_fastrtps_cpp; got <unset>'
        in combined_output(unset)
    )

    cyclone = run_guard([VENDOR_DDS_DIR], rmw='rmw_cyclonedds_cpp')
    assert cyclone.returncode != 0
    assert (
        'requires explicit RMW_IMPLEMENTATION=rmw_fastrtps_cpp'
        in combined_output(cyclone)
    )


def test_guard_rejects_ddsc_loaded_outside_compiled_vendor_directory(tmp_path):
    shutil.copy2(resolved_library('libddsc'), tmp_path / 'libddsc.so.0')
    result = run_guard([tmp_path, VENDOR_DDS_DIR])
    assert result.returncode != 0
    output = combined_output(result)
    assert 'DDS ABI guard rejected loaded libraries' in output
    assert f'libddsc={tmp_path}' in output


def test_guard_rejects_ddscxx_outside_compiled_vendor_directory(tmp_path):
    shutil.copy2(resolved_library('libddscxx'), tmp_path / 'libddscxx.so.0')
    result = run_guard([tmp_path, VENDOR_DDS_DIR])
    assert result.returncode != 0
    output = combined_output(result)
    assert 'DDS ABI guard rejected loaded libraries' in output
    assert f'libddscxx={tmp_path}' in output


def test_guard_rejects_real_ros_ddsc_vendor_ddscxx_mix():
    ros_dds_dir = ros_cyclonedds_directory()
    result = run_guard([ros_dds_dir, VENDOR_DDS_DIR])
    assert result.returncode != 0
    output = combined_output(result)
    assert 'DDS ABI guard rejected loaded libraries' in output
    assert f'libddsc={ros_dds_dir}' in output
    assert f'libddscxx={VENDOR_DDS_DIR}' in output


def test_rmw_check_only_initializes_ros_on_loopback_without_sdk():
    result = run_binary(['--rmw-check-only'], [VENDOR_DDS_DIR])
    assert result.returncode == 0, combined_output(result)
    assert (
        'accepted active FastRTPS after ROS initialization'
        in result.stdout
    )


def test_invalid_ros_arguments_fail_cleanly_without_sigabrt():
    result = run_binary(
        ['--ros-args', '--definitely-invalid'], [VENDOR_DDS_DIR])
    assert result.returncode == 1, combined_output(result)
    assert result.returncode != -6
    assert 'r1_sdk_transport runtime failure:' in result.stderr
