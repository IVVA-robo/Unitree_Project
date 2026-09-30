"""Static safety contract for the read-only controller-channel observer."""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / 'src' / 'r1_wireless_controller_observer.cpp').read_text()
CMAKE = (PACKAGE / 'CMakeLists.txt').read_text()
CODE = '\n'.join(line.split('//', 1)[0] for line in SOURCE.splitlines())


def test_observer_can_only_receive_the_official_typed_channel():
    assert 'rt/wirelesscontroller' in SOURCE
    assert 'WirelessController_' in SOURCE
    assert 'CreateRecvChannel' in CODE
    for forbidden in (
        'CreateSendChannel', 'ChannelPublisher', 'LocoClient', 'SportClient',
        'SetVelocity(', 'SetFsmId(', 'SetSpeedMode(', 'LowCmd_', 'ArmSdk',
    ):
        assert forbidden not in CODE


def test_observer_is_bounded_and_reports_discovery_and_samples():
    for token in (
        '--interface is required',
        '--duration-sec must be within 0.5..30.0',
        'if_nametoindex(options.interface.c_str())',
        'dds_get_subscription_matched_status',
        'dds_get_matched_publications',
        'WIRELESS_CONTROLLER status=',
        'keys_latest=0x%04x',
        'rate_hz=%.3f',
    ):
        assert token in SOURCE


def test_observer_is_built_installed_and_runtime_tested():
    assert 'add_executable(r1_wireless_controller_observer' in CMAKE
    assert 'r1_wireless_controller_observer' in CMAKE[
        CMAKE.index('install(TARGETS'):CMAKE.index('install(DIRECTORY')]
    assert 'test_wireless_controller_observer_contract' in CMAKE
    assert (
        'R1_WIRELESS_CONTROLLER_OBSERVER_BINARY=' in CMAKE
    )
