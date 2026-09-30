"""Process-only fault injection; no ROS, ADB, network or robot commands."""

import os
import sys
import time

from exhibition.actions import run_helper


def test_cancellation_runs_helper_term_cleanup_without_waiting_full_timeout(tmp_path):
    marker = tmp_path / 'cleaned'
    ready = tmp_path / 'ready'
    code = (
        'import signal,time; from pathlib import Path; '
        f'signal.signal(signal.SIGTERM, lambda *_: (Path({str(marker)!r}).touch(), exit(0))); '
        f'Path({str(ready)!r}).touch(); time.sleep(60)'
    )
    started = time.monotonic()
    result = run_helper([sys.executable, '-c', code], cwd=tmp_path, env=os.environ.copy(),
                        timeout=60, cancelled=ready.exists)
    assert time.monotonic() - started < 2.0
    assert result.returncode == 130
    assert marker.exists()


def test_timeout_terminates_grandchild_that_ignores_term(tmp_path):
    pid_file = tmp_path / 'child'
    code = (
        'import subprocess,sys,time; from pathlib import Path; '
        "p=subprocess.Popen([sys.executable,'-c','import signal,time; "
        "signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(60)']); "
        f'Path({str(pid_file)!r}).write_text(str(p.pid)); time.sleep(60)'
    )
    started = time.monotonic()
    result = run_helper([sys.executable, '-c', code], cwd=tmp_path, env=os.environ.copy(),
                        timeout=0.3, cancelled=lambda: False, cleanup_grace=0.2)
    assert result.returncode == 124
    assert time.monotonic() - started < 2.0
    from pathlib import Path
    proc = Path('/proc') / pid_file.read_text() / 'stat'
    try:
        state = proc.read_text().rsplit(')', 1)[1].split()[0]
    except (FileNotFoundError, ProcessLookupError):
        state = None
    assert state in {None, 'Z'}


def test_normal_helper_keeps_output_and_exit_code(tmp_path):
    result = run_helper([sys.executable, '-c', "print('checked'); exit(7)"],
                        cwd=tmp_path, env=os.environ.copy(), timeout=1,
                        cancelled=lambda: False)
    assert result.returncode == 7
    assert result.stdout == 'checked\n'
