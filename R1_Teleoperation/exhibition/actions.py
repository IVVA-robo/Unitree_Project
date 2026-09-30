"""Cancellable short-lived helpers; never used to terminate a robot writer."""

import os
import signal
import subprocess
import time


def run_helper(argv, *, cwd, env, timeout, cancelled, cleanup_grace=12.0):
    """Bound helper groups, including grandchildren of shell/ROS CLI scripts.

    SIGTERM lets prepare's existing emergency-stop trap run. Its bounded grace
    exceeds that trap's 4 + 5 seconds; the owning manager still performs its
    independent reviewed STOP/KILL afterwards. Cleanup actions are not cancelled.
    """
    with subprocess.Popen(argv, cwd=cwd, env=env, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True,
                          start_new_session=True) as process:
        deadline = time.monotonic() + timeout
        while True:
            interrupted = cancelled()
            expired = time.monotonic() >= deadline
            if interrupted or expired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    output, _ = process.communicate(timeout=cleanup_grace)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    output, _ = process.communicate(timeout=2.0)
                reason = 'cancelled by STOP' if interrupted else 'timed out'
                return subprocess.CompletedProcess(
                    argv, 130 if interrupted else 124,
                    (output or '') + f'\n[BLOCKED] helper {reason}\n')
            try:
                output, _ = process.communicate(timeout=min(0.1, max(0.001, deadline - time.monotonic())))
                return subprocess.CompletedProcess(argv, process.returncode, output)
            except subprocess.TimeoutExpired:
                pass
