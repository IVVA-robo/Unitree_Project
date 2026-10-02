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

from .session import ROOT, assert_no_live_session, run, usb_device
from exhibition.orchestrator import ExhibitionSettings
from .recovery import RelayRecovery, pause_for_recovery
from .video import (
    CONTROL_DEVICE_ENDPOINT,
    CONTROL_HOST_ENDPOINT,
    VIDEO_DEVICE_ENDPOINT,
    VIDEO_HOST_ENDPOINT,
    ensure_usb_app,
    reverse_mappings,
)


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
    existing = reverse_mappings(adb, runner=run)
    desired = [
        (CONTROL_DEVICE_ENDPOINT, CONTROL_HOST_ENDPOINT, True),
        (VIDEO_DEVICE_ENDPOINT, VIDEO_HOST_ENDPOINT, False),
    ]
    for device, host, exclusive in desired:
        if device not in existing:
            continue
        if existing[device] != host or (initial and exclusive):
            raise RuntimeError(f'USB port {device} belongs to another session')
    for device, host, exclusive in desired:
        if device not in existing:
            run(*adb, 'reverse', '--no-rebind', device, host)
            if exclusive:
                owned.add(device)


def wait_for_previous_control_cleanup(
    environment=None,
    *,
    checker=None,
    clock=time.monotonic,
    sleeper=time.sleep,
):
    """Wait briefly for the previous mode's writer process to disappear.

    The panel starts a requested mode only after the previous manager and its
    reviewed STOP path have completed.  Linux can still expose a terminating
    writer in ``/proc`` for a short time after that handoff.  Treat that as a
    bounded cleanup phase, while keeping a genuinely active writer blocking.
    """

    environment = os.environ if environment is None else environment
    checker = assert_no_live_session if checker is None else checker
    try:
        wait_sec = float(environment.get('R1_USB_LIVE_CLEANUP_WAIT_SEC', '12'))
    except ValueError as error:
        raise RuntimeError('R1_USB_LIVE_CLEANUP_WAIT_SEC must be numeric') from error
    if not 0 <= wait_sec <= 30:
        raise RuntimeError('R1_USB_LIVE_CLEANUP_WAIT_SEC must be in [0, 30]')

    deadline = clock() + wait_sec
    waiting = False
    while True:
        try:
            checker()
        except RuntimeError as error:
            if clock() >= deadline:
                raise RuntimeError(
                    'Previous robot control cleanup did not finish within '
                    f'{wait_sec:g}s; wait for STOP cleanup before RUN'
                ) from error
            if not waiting:
                print(
                    '[INFO] Waiting for the previous robot control process '
                    'to finish cleanup before USB RUN.',
                    flush=True,
                )
                waiting = True
            sleeper(0.05)
            continue
        if waiting:
            print('[OK] Previous robot control cleanup completed.', flush=True)
        return


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
            wait_sec = float(os.environ.get('R1_USB_TRANSPORT_LOCK_WAIT_SEC', '5'))
        except ValueError as error:
            raise RuntimeError('R1_USB_TRANSPORT_LOCK_WAIT_SEC must be numeric') from error
        if not 0 <= wait_sec <= 10:
            raise RuntimeError('R1_USB_TRANSPORT_LOCK_WAIT_SEC must be in [0, 10]')
        deadline = time.monotonic() + wait_sec
        while True:
            try:
                fcntl.flock(transport_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError as error:
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        'Previous USB session is still active or cleaning up'
                    ) from error
                time.sleep(0.05)
        return run_control()


def run_control():
    wait_for_previous_control_cleanup()
    runtime = ExhibitionSettings.from_environment().runtime_dir
    serial = usb_device(os.environ.get('R1_VR_ADB_SERIAL'))
    env = live_environment(os.environ, serial)
    adb = ['adb', '-s', serial]
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(('127.0.0.1', 19092))
    wifi_on = run(*adb, 'shell', 'settings', 'get', 'global', 'wifi_on') == '1'
    manager = relay = None
    stopping = False
    owned = set()
    wifi_changed = False

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
        app_state = ensure_usb_app(adb, runtime, serial)
        if stopping:
            return 0
        manager = subprocess.Popen([sys.executable, '-m', 'exhibition.orchestrator', 'control'],
                                   cwd=ROOT, env=env, start_new_session=True)
        print(
            f'[OK] USB RUN: Pico {serial}; source=127.0.0.1; '
            f'Wi-Fi disabled; app={app_state}; video=persistent',
            flush=True,
        )
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
        # Keep the USB-mode APK and the read-only 8080 tunnel alive across
        # RUN -> LOCK -> STAND. Only the exclusive pose tunnel belongs to this
        # control session and is removed here.
        cleanup = [['reverse', '--remove', port] for port in sorted(owned)]
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
