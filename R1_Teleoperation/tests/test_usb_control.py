"""USB mode preserves physical permissions and tunnel ownership."""

import subprocess

import pytest

from operator_panel.config import OperatorConfig
from usb_link.control import ensure_mappings, live_environment


def acknowledged():
    return dict(ROBOT_DRY_RUN='0', ROBOT_ENABLE_ACTUATION='1',
                ROBOT_CONFIRM_OFF_CHARGER='1', ROBOT_CONFIRM_CLEAR_AREA='1',
                ROBOT_CONFIRM_ESTOP_READY='1', ROBOT_CONFIRM_COMMISSIONING='1',
                ROBOT_CONFIRM_RUN_MODE='1', ROBOT_CONFIRM_NO_PHONE_CONTROL='1',
                ROBOT_COMMISSIONING_TOKEN='unit-test-no-physical-runtime')


def test_usb_adds_transport_without_creating_acknowledgements():
    source = acknowledged()
    result = live_environment(source, 'test-pico')
    assert result['ROBOT_VR_SOURCE_IP'] == '127.0.0.1'
    assert result['R1_VR_TRANSPORT'] == 'usb'
    assert result['R1_VR_ADB_SERIAL'] == 'test-pico'
    assert all(result[key] == value for key, value in source.items())
    assert 'R1_VR_TRANSPORT' not in source


@pytest.mark.parametrize('key', list(acknowledged()))
def test_each_missing_ack_blocks_usb_run(key):
    env = acknowledged()
    del env[key]
    with pytest.raises(RuntimeError):
        live_environment(env, 'test-pico')


def test_usb_panel_ignores_saved_wifi_address_and_keeps_safety_defaults():
    config = OperatorConfig(vr_transport='usb', vr_headset_ip='192.168.8.42')
    env = config.as_environment()
    assert env['ROBOT_VR_SOURCE_IP'] == '127.0.0.1'
    assert env['ROBOT_DRY_RUN'] == '1'
    assert env['ROBOT_ENABLE_ACTUATION'] == '0'
    config.vr_transport = 'lan'
    assert config.as_environment()['ROBOT_VR_SOURCE_IP'] == '192.168.8.42'


def test_usb_checks_all_tunnel_conflicts_before_any_write(monkeypatch):
    calls = []

    def run(*args):
        calls.append(args)
        return 'UsbFfs tcp:8080 tcp:18080'

    monkeypatch.setattr('usb_link.control.run', run)
    owned = set()
    with pytest.raises(RuntimeError):
        ensure_mappings(['adb', '-s', 'pico'], owned, initial=True)
    assert owned == set()
    assert len(calls) == 1


def test_usb_partial_acquisition_only_owns_successful_mapping(monkeypatch):
    def run(*args):
        if args[-1] == '--list':
            return ''
        if args[-1] == 'tcp:8080':
            raise subprocess.CalledProcessError(1, args)
        return ''

    monkeypatch.setattr('usb_link.control.run', run)
    owned = set()
    with pytest.raises(subprocess.CalledProcessError):
        ensure_mappings(['adb'], owned, initial=True)
    assert owned == {'tcp:19092'}


def test_usb_restores_only_missing_tunnel_after_reconnect(monkeypatch):
    calls = []

    def run(*args):
        calls.append(args)
        return 'UsbFfs tcp:8080 tcp:8080' if args[-1] == '--list' else ''

    monkeypatch.setattr('usb_link.control.run', run)
    owned = {'tcp:8080'}
    ensure_mappings(['adb'], owned)
    assert owned == {'tcp:8080', 'tcp:19092'}
    assert calls[-1][-2:] == ('tcp:19092', 'tcp:19092')


def test_service_has_no_late_global_stop_and_preserves_limits():
    from usb_link.service import service_command

    env = dict(acknowledged(), R1_LEG_SPEED_SCALE='1.0', SAFETY_PROFILE='slow-safe')
    command = service_command(env)
    assert not any('ExecStop' in arg or 'r1-exhibition' in arg for arg in command)
    assert '--property=Restart=no' in command
    assert '--property=KillMode=mixed' in command
    assert '--property=SendSIGKILL=no' in command
    assert '--property=TimeoutStopSec=infinity' in command
    assert '--setenv=R1_LEG_SPEED_SCALE=1.0' in command
    assert '--setenv=SAFETY_PROFILE=slow-safe' in command
    assert command[-3:] == ['-m', 'usb_link.control', 'control']
    assert command != service_command(env)  # no shared unit name


@pytest.mark.parametrize('key', list(acknowledged()))
def test_service_cannot_supply_missing_permissions(key):
    from usb_link.service import service_command

    env = acknowledged()
    del env[key]
    with pytest.raises(RuntimeError):
        service_command(env)


def test_transport_lock_covers_cleanup_and_blocks_second_wrapper(tmp_path, monkeypatch):
    import fcntl
    from usb_link import control

    for key, value in acknowledged().items():
        monkeypatch.setenv(key, value)
    monkeypatch.setenv('R1_EXHIBITION_RUNTIME_DIR', str(tmp_path))
    calls = []

    def fake_run():
        with (tmp_path / 'usb-transport.lock').open('a+') as other:
            with pytest.raises(BlockingIOError):
                fcntl.flock(other, fcntl.LOCK_EX | fcntl.LOCK_NB)
        calls.append('run-and-cleanup')
        return 0

    monkeypatch.setattr(control, 'run_control', fake_run)
    assert control.main(['control']) == 0
    assert control.main(['control']) == 0
    assert len(calls) == 2
    with (tmp_path / 'usb-transport.lock').open('a+') as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(RuntimeError, match='cleaning up'):
            control.main(['control'])
    assert len(calls) == 2


def test_missing_usb_cannot_start_manager_even_without_ui_probe(tmp_path, monkeypatch):
    from usb_link import control
    for key, value in acknowledged().items():
        monkeypatch.setenv(key, value)
    monkeypatch.setenv('R1_EXHIBITION_RUNTIME_DIR', str(tmp_path))
    monkeypatch.setattr(control, 'assert_no_live_session', lambda: None)
    def absent(*args):
        raise RuntimeError('headset absent')
    monkeypatch.setattr(control, 'usb_device', absent)
    monkeypatch.setattr(control.subprocess, 'Popen', lambda *args, **kwargs: pytest.fail('manager before USB readiness'))
    with pytest.raises(RuntimeError, match='headset absent'):
        control.main(['control'])
