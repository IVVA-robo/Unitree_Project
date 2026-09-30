"""Reversible USB diagnostic session: poses, POV and optional Wi-Fi-off proof."""

import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = 'com.ionosrobots.unitree.telepresencedemo'
ACTIVITY = PACKAGE + '/com.unity3d.player.UnityPlayerActivity'


def run(*args, check=True):
    return subprocess.run(args, check=check, capture_output=True, text=True, timeout=15).stdout.strip()


def usb_device(serial=None):
    rows = [row.split() for row in run('adb', 'devices', '-l').splitlines()[1:]]
    devices = [row[0] for row in rows if len(row) > 2 and row[1] == 'device'
               and any(item.startswith('usb:') for item in row[2:])]
    if serial and serial in devices:
        return serial
    if serial or len(devices) != 1:
        raise RuntimeError('Connect one authorized USB Pico, or select it with --serial')
    return devices[0]


def assert_no_live_session():
    for proc in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            argv = proc.read_bytes().split(b'\0')
        except OSError:
            continue
        if any(Path(os.fsdecode(arg)).name in
               {'r1_live_writer', 'r1_live_writer_node', 'r1-live-session', 'r1-robot-prepare'} for arg in argv if arg):
            raise RuntimeError('Stop the active robot control session before USB diagnostics')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial')
    parser.add_argument('--video-source', choices=['mock', 'unitree'], default='mock')
    parser.add_argument('--wifi-off', action='store_true',
                        help='Temporarily disable headset Wi-Fi; restore on exit')
    parser.add_argument('--duration', type=float, default=0,
                        help='Stop after this many seconds; 0 runs until Ctrl-C')
    args = parser.parse_args()
    assert_no_live_session()
    serial = usb_device(args.serial)
    adb = ['adb', '-s', serial]
    # Verify all dedicated endpoints before changing the headset.
    for kind, port in [(socket.SOCK_STREAM, 19092), (socket.SOCK_STREAM, 18080),
                       (socket.SOCK_DGRAM, 19090), (socket.SOCK_DGRAM, 19091)]:
        with socket.socket(socket.AF_INET, kind) as sock:
            if kind == socket.SOCK_STREAM:
                # Match asyncio's listener: a previous session's TIME_WAIT
                # connections are not an active listener or a port conflict.
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('127.0.0.1', port))
    reverse = run(*adb, 'reverse', '--list')
    if any(port in reverse.split() for port in ['tcp:8080', 'tcp:19092']):
        raise RuntimeError('ADB reverse 8080/19092 is already owned; close its session first')
    wifi_on = run(*adb, 'shell', 'settings', 'get', 'global', 'wifi_on') == '1'
    children, mappings = [], []
    wifi_changed = False
    app_started = False
    env = dict(os.environ, PYTHONUNBUFFERED='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
               R1_DRY_RUN_DOMAIN_ID='91', R1_DRY_RUN_UDP_BIND_ADDRESS='127.0.0.1',
               R1_DRY_RUN_UDP_PORT='19090', R1_DRY_RUN_DISCOVERY_PORT='19091',
               R1_DRY_RUN_VR_ALLOWED_SOURCE_IP='127.0.0.1')

    def start(command):
        child = subprocess.Popen(command, cwd=ROOT, env=env, start_new_session=True)
        children.append(child)

    def stopped(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stopped)
    signal.signal(signal.SIGINT, stopped)
    try:
        start([sys.executable, '-m', 'usb_link.relay'])
        start([str(ROOT / 'scripts/r1-teleop-dry-run')])
        # r1-teleoperation supplies the matched SDK environment for real video.
        video_env = ROOT / 'ros2_ws/src/r1_robot_pov/config' / (
            'robot_pov.r1.env' if args.video_source == 'unitree' else 'robot_pov.mock.env')
        env.update(R1_TELEOP_START_BRIDGE='false', R1_TELEOP_START_SIMULATION='false',
                   R1_TELEOP_START_VIDEO='true', R1_TELEOP_VIDEO_SOURCE=args.video_source,
                   R1_TELEOP_VIDEO_ENV_FILE=str(video_env), R1_TELEOP_DOMAIN_ID='91',
                   R1_TELEOP_VIDEO_HOST='127.0.0.1', R1_TELEOP_VIDEO_PORT='18080',
                   R1_TELEOP_VIDEO_PROFILE='bad-wifi', R1_TELEOP_VIDEO_LAYOUT='mono',
                   R1_TELEOP_VIDEO_MDNS='false', ROBOT_POV_DISCOVERY_ENABLED='false')
        start([str(ROOT / 'scripts/r1-teleoperation')])
        for device_port, host_port in [('19092', '19092'), ('8080', '18080')]:
            run(*adb, 'reverse', '--no-rebind', 'tcp:' + device_port, 'tcp:' + host_port)
            mappings.append('tcp:' + device_port)
        if args.wifi_off and wifi_on:
            wifi_changed = True
            run(*adb, 'shell', 'svc', 'wifi', 'disable')
        run(*adb, 'shell', 'am', 'force-stop', PACKAGE)
        app_started = True
        run(*adb, 'shell', 'input', 'keyevent', 'KEYCODE_WAKEUP')
        run(*adb, 'shell', 'am', 'start', '-n', ACTIVITY, '--ez', 'r1_usb', 'true')
        print('USB diagnostic session: ROS domain 91; video http://127.0.0.1:18080/', flush=True)
        print('Robot actuation is disabled. Ctrl-C closes owned tunnels and restores headset Wi-Fi.', flush=True)
        start_time = time.monotonic()
        while not args.duration or time.monotonic() - start_time < args.duration:
            if any(child.poll() is not None for child in children):
                raise RuntimeError('A USB session component exited; closing session')
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
        for child in children:
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        # Each action is independent: a disconnected headset must not skip
        # cleanup of laptop processes, or hide the need to restore Wi-Fi.
        cleanup = []
        if app_started:
            cleanup.append(['shell', 'am', 'force-stop', PACKAGE])
        cleanup += [['reverse', '--remove', mapping] for mapping in mappings]
        if wifi_changed:
            cleanup.append(['shell', 'svc', 'wifi', 'enable'])
        for action in cleanup:
            try:
                run(*adb, *action)
            except (OSError, subprocess.SubprocessError) as error:
                print(f'USB cleanup pending: {action}: {error}', file=sys.stderr)
        if wifi_changed:
            print('If the cable was removed, reconnect it and run: adb shell svc wifi enable', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print(f'USB session: {error}', file=sys.stderr)
        sys.exit(1)
