"""
Own USB transport around the existing exhibition control manager.

This wrapper never creates physical acknowledgements or a robot client.
The normal manager remains responsible for readiness, prepare, STOP and KILL.
"""

import fcntl
import os
import signal
import socket
import subprocess
import sys
import time

from .session import ACTIVITY, PACKAGE, ROOT, assert_no_live_session, run, usb_device
from exhibition.orchestrator import ExhibitionSettings
from .recovery import RelayRecovery, pause_for_recovery


def live_environment(environment, serial):
    """Keep all physical permissions explicit; add only transport settings."""
    required = {
        'ROBOT_DRY_RUN': '0', 'ROBOT_ENABLE_ACTUATION': '1',
        'ROBOT_CONFIRM_OFF_CHARGER': '1', 'ROBOT_CONFIRM_CLEAR_AREA': '1',
        'ROBOT_CONFIRM_ESTOP_READY': '1', 'ROBOT_CONFIRM_COMMISSIONING': '1',
        'ROBOT_CONFIRM_RUN_MODE': '1', 'ROBOT_CONFIRM_NO_PHONE_CONTROL': '1',
    }
    missing = [key for key, value in required.items() if environment.get(key) != value]
    if len(environment.get('ROBOT_COMMISSIONING_TOKEN', '')) < 16:
        missing.append('ROBOT_COMMISSIONING_TOKEN')
    if missing:
        raise RuntimeError('Start USB RUN from the operator panel; missing: ' + ', '.join(missing))
    if environment.get('R1_EXHIBITION_SESSION_MODE', 'session_arm') != 'session_arm':
        raise RuntimeError('USB RUN requires session_arm for explicit resume after cable loss')
    if environment.get('R1_LIVE_UDP_PORT', '9090') != '9090':
        raise RuntimeError('USB RUN uses fixed local UDP 9090')
    return dict(environment, R1_VR_TRANSPORT='usb', R1_USB_MANAGED='1',
                R1_VR_ADB_SERIAL=serial, R1_VR_AUTO_DISCOVERY='0',
                ROBOT_VR_SOURCE_IP='127.0.0.1', R1_LIVE_UDP_PORT='9090',
                PYTHONUNBUFFERED='1')


def ensure_mappings(adb, owned, *, initial=False):
    existing = {}
    for row in run(*adb, 'reverse', '--list').splitlines():
        fields = row.split()
        if len(fields) == 3:
            existing[fields[1]] = fields[2]
    desired = [('tcp:19092', 'tcp:19092'), ('tcp:8080', 'tcp:8080')]
    for device, host in desired:
        if device in existing:
            if initial or existing[device] != host:
                raise RuntimeError(f'USB port {device} belongs to another session')
    for device, host in desired:
        if device not in existing:
            run(*adb, 'reverse', '--no-rebind', device, host)
            owned.add(device)


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if args != ['control']:
        raise RuntimeError('USB control wrapper accepts only the control action')
    # Check acknowledgements before even starting ADB or changing the headset.
    live_environment(os.environ, '')
    runtime = ExhibitionSettings.from_environment().runtime_dir
    runtime.mkdir(mode=0o700, parents=True, exist_ok=True)
    # Cover acquisition AND final ADB cleanup, not just the child manager.
    with (runtime / 'usb-transport.lock').open('a+') as transport_lock:
        try:
            fcntl.flock(transport_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError('Previous USB session is still active or cleaning up') from error
        return run_control()


def run_control():
    assert_no_live_session()
    serial = usb_device(os.environ.get('R1_VR_ADB_SERIAL'))
    env = live_environment(os.environ, serial)
    adb = ['adb', '-s', serial]
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(('127.0.0.1', 19092))
    # Reject conflicts before acquiring any mapping, then own these two only.
    current = run(*adb, 'reverse', '--list').split()
    if 'tcp:19092' in current or 'tcp:8080' in current:
        raise RuntimeError('Close the USB diagnostic session before starting RUN')
    wifi_on = run(*adb, 'shell', 'settings', 'get', 'global', 'wifi_on') == '1'
    manager = relay = None
    stopping = False
    owned = set()
    wifi_changed = False
    app_started = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True
        if manager is not None and manager.poll() is None:
            # Signal only the manager, which owns its robot STOP sequence.
            manager.send_signal(signal.SIGTERM)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        ensure_mappings(adb, owned, initial=True)
        relay = subprocess.Popen([sys.executable, '-m', 'usb_link.relay', '--udp-port', '9090'],
                                 cwd=ROOT, env=env, start_new_session=True)
        if wifi_on:
            wifi_changed = True
            run(*adb, 'shell', 'svc', 'wifi', 'disable')
        run(*adb, 'shell', 'am', 'force-stop', PACKAGE)
        app_started = True
        run(*adb, 'shell', 'input', 'keyevent', 'KEYCODE_WAKEUP')
        run(*adb, 'shell', 'am', 'start', '-n', ACTIVITY, '--ez', 'r1_usb', 'true')
        if stopping:
            return 0
        manager = subprocess.Popen([sys.executable, '-m', 'exhibition.orchestrator', 'control'],
                                   cwd=ROOT, env=env, start_new_session=True)
        print(f'[OK] USB RUN: Pico {serial}; source=127.0.0.1; Wi-Fi disabled', flush=True)
        print('[INFO] Cable/pose loss pauses control; '
              'explicit RUN is required after recovery.', flush=True)
        was_connected = True
        recovery = RelayRecovery()
        while manager.poll() is None:
            if stopping:
                time.sleep(0.25)
                continue
            if relay.poll() is not None:
                recovery.admit(time.monotonic())
                pause_for_recovery(manager, ExhibitionSettings.from_environment(), lambda: stopping)
                if stopping or manager.poll() is not None:
                    continue
                relay = subprocess.Popen(
                    [sys.executable, '-m', 'usb_link.relay', '--udp-port', '9090'],
                    cwd=ROOT, env=env, start_new_session=True)
                print('[WARN] USB relay restarted in LOCK. Press RUN explicitly after recovery.', flush=True)
            try:
                connected = usb_device(serial) == serial
            except (RuntimeError, subprocess.SubprocessError):
                connected = False
            try:
                if connected:
                    ensure_mappings(adb, owned)
            except subprocess.SubprocessError:
                # The cable may disappear between discovery and reverse.
                # Keep the manager alive and let the bridge latch its pause.
                connected = False
            if connected != was_connected:
                print('[INFO] USB cable restored; press RUN to resume.' if connected else
                      '[WARN] USB cable absent; bridge watchdog pauses control.', flush=True)
                was_connected = connected
            time.sleep(1.0)
        return manager.returncode
    finally:
        if manager is not None and manager.poll() is None:
            manager.send_signal(signal.SIGTERM)
            # Keep relay alive while the existing manager performs STOP/KILL.
            # Never SIGKILL a robot owner to make a timeout look successful.
            while manager.poll() is None:
                try:
                    manager.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print('[INFO] Waiting for exhibition manager to finish robot STOP.',
                          flush=True)
        if relay is not None and relay.poll() is None:
            relay.terminate()
            try:
                relay.wait(timeout=5)
            except subprocess.TimeoutExpired:
                relay.kill()  # pose-only local relay; manager is already stopped
                relay.wait()
        cleanup = [['shell', 'am', 'force-stop', PACKAGE]] if app_started else []
        cleanup += [['reverse', '--remove', port] for port in sorted(owned)]
        if wifi_changed:
            cleanup.append(['shell', 'svc', 'wifi', 'enable'])
        for action in cleanup:
            try:
                run(*adb, *action)
            except (OSError, subprocess.SubprocessError):
                print(f'[WARN] Pico cleanup needs cable: {action}', flush=True)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(f'[BLOCKED] USB RUN: {error}', file=sys.stderr)
        sys.exit(2)
