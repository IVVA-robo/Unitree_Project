"""Static safety contract for the read-only MotionSwitcher diagnostic."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / 'src' / 'r1_motion_switcher_probe.cpp').read_text()
CMAKE = (PACKAGE / 'CMakeLists.txt').read_text()
CODE = '\n'.join(line.split('//', 1)[0] for line in SOURCE.splitlines())


def test_probe_declares_the_mandatory_internal_noop_and_only_read_only_mode_apis():
    """Client construction has API 2; mode inspection cannot change a mode."""
    assert 'ROBOT_API_ID_INTERNAL_API_NOOP == 2' in CODE
    assert 'SDK client construction' in SOURCE
    assert 'MOTION_SWITCHER_API_ID_CHECK_MODE' in CODE
    assert 'MOTION_SWITCHER_API_ID_GET_SILENT' in CODE
    # These are explicit MotionSwitcher calls.  The base SDK Client separately
    # makes its mandatory internal Noop (API 2) during construction.
    assert CODE.count('RegistApi(') == 2
    assert CODE.count('Call(') == 2
    for forbidden in (
        'MOTION_SWITCHER_API_ID_SELECT_MODE',
        'MOTION_SWITCHER_API_ID_RELEASE_MODE',
        'MOTION_SWITCHER_API_ID_SET_SILENT',
        'SelectMode(',
        'ReleaseMode(',
        'SetSilent(',
        'CreateSendChannel',
        'ChannelPublisher',
        'ArmSdk',
        'LocoClient',
    ):
        assert forbidden not in CODE


def test_probe_bounds_all_external_interaction():
    """The CLI requires a real interface and has no unbounded wait path."""
    for token in (
        '--interface is required',
        'if_nametoindex(options.interface.c_str())',
        '--timeout-sec must be within 0.1..10.0',
        'client.SetTimeout(static_cast<float>(options.timeout_sec));',
        'ChannelFactory::Instance()->Init(0, options.interface);',
        'MOTION_SWITCHER_CHECK status=',
    ):
        assert token in SOURCE
    assert 'std::thread' not in CODE
    assert 'while (' not in CODE


def test_probe_is_built_installed_and_runtime_tested():
    """The installed executable remains covered by its inert --help test."""
    assert 'add_executable(r1_motion_switcher_probe' in CMAKE
    assert 'set(R1_MOTION_SWITCHER_PROBE_HAS_SDK OFF)' in CMAKE
    assert 'motion_switcher/motion_switcher_api.hpp' in CMAKE
    assert 'r1_motion_switcher_probe' in CMAKE[
        CMAKE.index('install(TARGETS'):CMAKE.index('install(DIRECTORY')]
    assert 'test_motion_switcher_probe_contract' in CMAKE
    assert 'R1_MOTION_SWITCHER_PROBE_BINARY=$<TARGET_FILE:r1_motion_switcher_probe>' in CMAKE
