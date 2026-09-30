"""Static safety contract for the read-only R1 locomotion-state probe."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / 'src' / 'r1_loco_state_probe.cpp').read_text()
CMAKE = (PACKAGE / 'CMakeLists.txt').read_text()
CODE = '\n'.join(line.split('//', 1)[0] for line in SOURCE.splitlines())


def test_probe_registers_only_internal_noop_and_read_only_loco_apis():
    """The executable cannot express an FSM, velocity, or speed command."""
    assert 'ROBOT_API_ID_INTERNAL_API_NOOP == 2' in CODE
    assert 'SDK Client construction' in SOURCE
    assert 'ROBOT_API_ID_LOCO_GET_FSM_ID' in CODE
    assert 'ROBOT_API_ID_LOCO_GET_FSM_MODE' in CODE
    # These are the only explicit calls. The SDK base Client separately sends
    # its mandatory internal Noop (API 2) during construction.
    assert CODE.count('RegistApi(') == 2
    assert CODE.count('Call(') == 2
    for forbidden in (
        'ROBOT_API_ID_LOCO_SET_FSM_ID',
        'ROBOT_API_ID_LOCO_SET_VELOCITY',
        'ROBOT_API_ID_LOCO_SET_SPEED_MODE',
        'SetFsmId(',
        'SetVelocity(',
        'SetSpeedMode(',
        'StandUp(',
        'Start(',
        'StopMove(',
        'Damp(',
        'ZeroTorque(',
        'CreateSendChannel',
        'ChannelPublisher',
        'ArmSdk',
        'LocoClient',
    ):
        assert forbidden not in CODE


def test_probe_bounds_every_external_query():
    """The CLI requires a real interface and has no unbounded wait path."""
    for token in (
        '--interface is required',
        'if_nametoindex(options.interface.c_str())',
        '--timeout-sec must be within 0.1..10.0',
        'client.SetTimeout(static_cast<float>(options.timeout_sec));',
        'ChannelFactory::Instance()->Init(0, options.interface);',
        'client.GetApiVersion()',
        'client.GetServerApiVersion()',
        'R1_LOCO_API_VERSION local=',
        'R1_LOCO_FSM_ID status=',
        'api=7001',
        'api=7002',
    ):
        assert token in SOURCE
    assert 'std::thread' not in CODE
    assert 'while (' not in CODE


def test_fsm_mode_is_opt_in_and_follows_a_successful_fsm_id_query():
    """The default performs one state query; API 7002 requires an explicit flag."""
    parse_flag = SOURCE.index('argument == "--get-fsm-mode"')
    id_call = SOURCE.index('client.get_fsm_id(fsm_id)')
    id_failure = SOURCE.index('return kExitFsmIdFailed', id_call)
    mode_guard = SOURCE.index('if (!options.get_fsm_mode)', id_failure)
    mode_call = SOURCE.index('client.get_fsm_mode(fsm_mode)', mode_guard)
    assert parse_flag < id_call < id_failure < mode_guard < mode_call


def test_probe_is_built_installed_and_runtime_tested():
    """The installed executable remains covered by its inert --help test."""
    assert 'add_executable(r1_loco_state_probe' in CMAKE
    assert 'set(R1_LOCO_STATE_PROBE_HAS_SDK OFF)' in CMAKE
    assert 'r1/loco/r1_loco_api.hpp' in CMAKE
    assert 'r1_loco_state_probe' in CMAKE[
        CMAKE.index('install(TARGETS'):CMAKE.index('install(DIRECTORY')]
    assert 'test_loco_state_probe_contract' in CMAKE
    assert 'R1_LOCO_STATE_PROBE_BINARY=$<TARGET_FILE:r1_loco_state_probe>' in CMAKE
