"""Read-only Linux process identity for USB cleanup handoff (never signal it)."""

import os
from pathlib import Path


def usb_owner(pid):
    try:
        pid = int(pid)
        if pid <= 1:
            return {}
        proc = Path(f'/proc/{pid}')
        argv = proc.joinpath('cmdline').read_bytes().split(b'\0')
        if b'usb_link.control' not in argv:
            return {}
        fields = proc.joinpath('stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return {}
        return {'pid': pid, 'start_ticks': fields[19]}
    except (OSError, ValueError, TypeError, IndexError):
        return {}


def parent_usb_owner():
    return usb_owner(os.getppid())


def owner_alive(owner):
    return isinstance(owner, dict) and bool(owner) and usb_owner(owner.get('pid')) == owner


CONTROL_WRAPPERS = frozenset({
    'r1-teleop-live', 'r1-live-session', 'r1-live-auto-run',
    'r1-arms-live-test', 'r1-head-live-test', 'r1-locomotion-live-test',
    'r1-arms-running-live-test', 'r1-head-running-live-test',
})


def local_control_process_exists(root=Path('/proc')):
    """Conservative read-only guard for repeated STOP, across ROS domains.

    Do not inspect/log process environments. An unknown/unreadable process
    prevents the no-op shortcut. The reviewed STOP fallback remains available.
    This only identifies this project's control stack, not arbitrary robots.
    """
    try:
        processes = list(root.iterdir())
    except OSError:
        return True
    for proc in processes:
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            state = (proc / 'stat').read_text().rsplit(')', 1)[1].split()[0]
            if state == 'Z':
                continue
            argv = [os.fsdecode(part) for part in (proc / 'cmdline').read_bytes().split(b'\0') if part]
            if not argv:
                continue
            names = {Path(arg).name for arg in argv[:3]}
            if 'r1_live_writer_node' in names or names & CONTROL_WRAPPERS:
                return True
            if '-m' in argv:
                index = argv.index('-m') + 1
                module = argv[index] if index < len(argv) else ''
                action = argv[index + 1] if index + 1 < len(argv) else ''
                if module == 'usb_link.control':
                    return True
                if module == 'exhibition.orchestrator' and action in {'static', 'control'}:
                    return True
            if any(Path(arg).name == 'r1-exhibition' for arg in argv[:2]):
                if any(arg in {'static', 'control'} for arg in argv[1:]):
                    return True
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (OSError, ValueError, IndexError):
            return True
    return False
