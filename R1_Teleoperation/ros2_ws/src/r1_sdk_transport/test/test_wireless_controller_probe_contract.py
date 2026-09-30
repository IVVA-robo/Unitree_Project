"""Static contract for the deliberately narrow live controller probe."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / 'src' / 'r1_wireless_controller_probe.cpp').read_text()
CMAKE = (PACKAGE / 'CMakeLists.txt').read_text()
CODE = '\n'.join(line.split('//', 1)[0] for line in SOURCE.splitlines())


def test_live_command_is_fixed_bounded_and_acknowledged():
    for token in (
        'constexpr float kForwardLy = 0.15F;',
        'BOUNDED_R1_RUNNING_AND_FORWARD_STICK_PROBE',
        '(1U << 4U) | (1U << 8U)',
        'constexpr double kRunningKeyPulseSec = 0.20;',
        'constexpr double kRunningSettleSec = 2.00;',
        'constexpr double kPulseSec = 0.40;',
        'constexpr double kPreZeroSec = 0.50;',
        'constexpr double kPostZeroSec = 1.00;',
        'BOUNDED_R1_FORWARD_STICK_PROBE',
        'options.acknowledgement != kRunningLiveAcknowledgement',
        'WirelessController_(0.0F, ly, 0.0F, 0.0F, keys)',
    ):
        assert token in SOURCE
    assert '--ly' not in SOURCE
    assert '--duration' not in SOURCE
    assert '--keys' not in SOURCE


def test_probe_has_no_rpc_or_low_level_actuation_path():
    assert 'CreateSendChannel' in CODE
    assert 'rt/wirelesscontroller' in SOURCE
    for forbidden in (
        'LocoClient', 'SportClient', 'SetVelocity(', 'SetFsmId(',
        'SetSpeedMode(', 'LowCmd_', 'ArmSdk', 'rt/lowcmd', 'rt/arm_sdk',
    ):
        assert forbidden not in CODE


def test_probe_requires_a_typed_subscriber_before_any_write():
    match = SOURCE.index('const MatchResult match =')
    reject = SOURCE.index('return kExitUnmatched;', match)
    first_write = SOURCE.index('publish_for(channel', reject)
    assert match < reject < first_write
    assert 'dds_get_publication_matched_status' in SOURCE
    assert 'dds_get_matched_subscriptions' in SOURCE


def test_probe_is_built_installed_and_runtime_tested():
    assert 'add_executable(r1_wireless_controller_probe' in CMAKE
    assert 'r1_wireless_controller_probe' in CMAKE[
        CMAKE.index('install(TARGETS'):CMAKE.index('install(DIRECTORY')]
    assert 'test_wireless_controller_probe_contract' in CMAKE
    assert 'R1_WIRELESS_CONTROLLER_PROBE_BINARY=' in CMAKE
