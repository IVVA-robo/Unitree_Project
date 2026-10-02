"""The warmup parent must await preflight cleanup before it reports exit."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


@pytest.mark.parametrize('launcher', ['direct', 'panel'])
def test_warmup_stop_waits_for_its_preflight_owned_cleanup(tmp_path, launcher, monkeypatch):
    root = Path(__file__).resolve().parents[1]
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    warmup = scripts / 'r1-sdk-warmup'
    warmup.write_text((root / 'scripts/r1-sdk-warmup').read_text())
    warmup.chmod(0o755)
    (scripts / 'r1-network-autodetect').write_text('r1_resolve_control_ip() { echo 127.0.0.1; }\n')
    for name, code in [('ping', 0), ('pgrep', 1)]:
        path = scripts / name
        path.write_text(f'#!/bin/sh\nexit {code}\n')
        path.chmod(0o755)
    attestation = scripts / 'r1-sdk-preflight-attestation'
    attestation.write_text(
        '#!/bin/sh\n[ "$1" = invalidate ] && exit 0\nexit 3\n'
    )
    attestation.chmod(0o755)
    ready, cleaned = tmp_path / 'ready', tmp_path / 'cleaned'
    terminating = tmp_path / 'terminating'
    child = scripts / 'r1-sdk-preflight'
    child.write_text(
        f'#!{sys.executable}\nimport signal,time\nfrom pathlib import Path\n'
        f'def stop(*args):\n    Path({str(terminating)!r}).touch()\n    time.sleep(0.25)\n'
        f'    Path({str(cleaned)!r}).touch()\n    raise SystemExit(0)\n'
        'signal.signal(signal.SIGTERM, stop)\n'
        f'Path({str(ready)!r}).touch()\nwhile True: time.sleep(0.1)\n')
    child.chmod(0o755)
    environment_path = str(scripts) + ':' + os.environ['PATH']
    if launcher == 'panel':
        from PyQt5.QtWidgets import QApplication
        from operator_panel.commands import command_catalog
        from operator_panel.config import OperatorConfig
        from operator_panel.processes import ProcessController

        app = QApplication.instance() or QApplication([])
        monkeypatch.setenv('PATH', environment_path)
        (tmp_path / 'Makefile').write_text('r1-sdk-warmup:\n\t./scripts/r1-sdk-warmup\n')
        controller = ProcessController(OperatorConfig(project_dir=str(tmp_path)))
        finished = []
        controller.finished.connect(lambda key, code, status: finished.append((code, cleaned.exists())))
        spec = next(spec for spec in command_catalog() if spec.key == 'sdk_warmup')
        assert controller.start('sdk_warmup', spec)
        try:
            deadline = time.monotonic() + 3
            while not ready.exists() and time.monotonic() < deadline:
                app.processEvents()
                time.sleep(0.01)
            assert ready.exists()
            controller.stop('sdk_warmup', graceful_timeout_ms=12000, wait=False)
            deadline = time.monotonic() + 2
            while not terminating.exists() and time.monotonic() < deadline:
                app.processEvents()
                time.sleep(0.01)
            assert terminating.exists()
            # STOP followed by close/Zero Torque must not interrupt cleanup.
            controller.stop('sdk_warmup', graceful_timeout_ms=12000, wait=False)
            deadline = time.monotonic() + 4
            while not finished and time.monotonic() < deadline:
                app.processEvents()
                time.sleep(0.01)
            assert finished == [(0, True)]
        finally:
            if controller.is_running('sdk_warmup'):
                controller.stop('sdk_warmup', graceful_timeout_ms=4000)
            # Old make may exit before the owned cleanup. Reap the isolated
            # test worker before the temporary fixture is discarded.
            deadline = time.monotonic() + 4
            while ready.exists() and not cleaned.exists() and time.monotonic() < deadline:
                app.processEvents()
                time.sleep(0.01)
        return

    process = subprocess.Popen([str(warmup)], cwd=tmp_path,
                               env=dict(os.environ, PATH=environment_path),
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 3
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert ready.exists()
        os.killpg(process.pid, signal.SIGTERM)
        output, _ = process.communicate(timeout=4)
        assert process.returncode == 0, output
        assert cleaned.exists()
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.communicate(timeout=4)


def test_warmup_refresh_keeps_recent_cache_until_replacement_is_ready(tmp_path):
    """A click during healthy refresh must not recreate the cold-cache gap."""
    root = Path(__file__).resolve().parents[1]
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    warmup = scripts / 'r1-sdk-warmup'
    warmup.write_text((root / 'scripts/r1-sdk-warmup').read_text())
    warmup.chmod(0o755)
    (scripts / 'r1-network-autodetect').write_text(
        'r1_resolve_control_ip() { echo 127.0.0.1; }\n'
    )
    for name in ('ping', 'pgrep'):
        path = scripts / name
        path.write_text('#!/bin/sh\nexit ' + ('0' if name == 'ping' else '1') + '\n')
        path.chmod(0o755)

    cache = tmp_path / 'attestation'
    attestation = scripts / 'r1-sdk-preflight-attestation'
    attestation.write_text(
        '#!/bin/sh\n'
        f'cache={str(cache)!r}\n'
        'case "$1" in\n'
        '  check) [ -f "$cache" ] ;;\n'
        '  record) : > "$cache" ;;\n'
        '  invalidate) rm -f -- "$cache" ;;\n'
        'esac\n'
    )
    attestation.chmod(0o755)

    count = tmp_path / 'preflight-count'
    refreshing = tmp_path / 'refreshing'
    cleaned = tmp_path / 'cleaned'
    child = scripts / 'r1-sdk-preflight'
    child.write_text(
        f'#!{sys.executable}\n'
        'import signal, time\n'
        'from pathlib import Path\n'
        f'count = Path({str(count)!r})\n'
        'attempt = int(count.read_text()) + 1 if count.exists() else 1\n'
        'count.write_text(str(attempt))\n'
        'if attempt == 1:\n'
        '    raise SystemExit(0)\n'
        f'refreshing = Path({str(refreshing)!r})\n'
        f'cleaned = Path({str(cleaned)!r})\n'
        'def stop(*_args):\n'
        '    cleaned.touch()\n'
        '    raise SystemExit(0)\n'
        'signal.signal(signal.SIGTERM, stop)\n'
        'refreshing.touch()\n'
        'while True:\n'
        '    time.sleep(0.05)\n'
    )
    child.chmod(0o755)

    environment = dict(
        os.environ,
        PATH=str(scripts) + ':' + os.environ['PATH'],
        R1_LIVE_NETWORK_INTERFACE='lo',
        R1_SDK_WARMUP_RETRY_SEC='1',
        R1_SDK_WARMUP_REFRESH_SEC='1',
    )
    process = subprocess.Popen(
        [str(warmup)], cwd=tmp_path, env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        deadline = time.monotonic() + 5
        while not refreshing.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert refreshing.exists()
        assert cache.exists(), 'healthy cache disappeared during refresh'

        os.killpg(process.pid, signal.SIGTERM)
        output, _ = process.communicate(timeout=4)
        assert process.returncode == 0, output
        assert cleaned.exists()
        assert cache.exists(), 'handoff lost the preceding healthy attestation'
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.communicate(timeout=4)
