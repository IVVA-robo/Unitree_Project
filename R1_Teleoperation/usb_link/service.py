"""Start a persistent USB owner without any global, late ExecStop callback."""

import os
import subprocess
import sys
import uuid

from .control import live_environment
from .session import ROOT


def service_command(environment):
    # Validate but never mint permissions or change the caller's speed limits.
    live_environment(environment, environment.get('R1_VR_ADB_SERIAL', ''))
    argv = [
        'systemd-run', '--user', '--collect',
        '--unit=r1-usb-run-' + uuid.uuid4().hex,
        '--property=Type=exec', '--property=Restart=no',
        '--property=KillMode=mixed', '--property=SendSIGKILL=no',
        '--property=TimeoutStopSec=infinity',
        '--working-directory=' + str(ROOT),
    ]
    for key, value in sorted(environment.items()):
        if key.startswith(('R1_', 'ROBOT_', 'ROS_', 'RMW_', 'CYCLONEDDS_')) or key in {
            'SAFETY_PROFILE', 'PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH',
            'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
        }:
            argv.append(f'--setenv={key}={value}')
    # SIGTERM goes to the wrapper, which signals only its own child manager.
    # Natural exit must do NOTHING to any subsequent exhibition session.
    argv += [sys.executable, '-m', 'usb_link.control', 'control']
    return argv


def main():
    if sys.argv[1:]:
        raise RuntimeError('This launcher takes no arguments; use the existing acknowledged environment')
    return subprocess.run(service_command(os.environ), check=False).returncode


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, OSError) as error:
        print(f'[BLOCKED] USB service: {error}', file=sys.stderr)
        sys.exit(2)
