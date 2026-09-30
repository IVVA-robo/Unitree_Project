#!/usr/bin/env python3
"""Build a reviewable local export; never push, start services or delete originals."""

import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import tarfile
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[2]
EXCLUDED_DIRS = {'.git', '.ssh', '.gnupg', '.nx', 'Library', 'Temp',
                 'UserSettings', '__pycache__', '.pytest_cache', 'build', 'install'}
SECRETS = [
    re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----'),
    re.compile(rb'github_pat_[A-Za-z0-9_]{30,}'),
    re.compile(rb'gh[pousr]_[A-Za-z0-9]{30,}'),
    re.compile(rb'sk-(?:proj-|svcacct-)[A-Za-z0-9_-]{30,}'),
    re.compile(rb'AKIA[0-9A-Z]{16}'),
]
TOKEN = re.compile(rb'(?i)(commissioning_token[\s"\x27:=]+)([A-Za-z0-9_-]{16,})')


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def forbidden(path):
    p = PurePosixPath(str(path))
    return (p.is_absolute() or '..' in p.parts
            or bool(set(p.parts) & EXCLUDED_DIRS)
            or p.name in {'.env', '.bash_history', '.git-credentials', 'id_rsa',
                          'id_ed25519', 'r1_pc2_ed25519'}
            or p.name.endswith(('.local.env', '.venue.env'))
            or p.suffix in {'.key', '.pem', '.p12', '.jks', '.keystore', '.partial'})


def sanitize(data):
    if any(pattern.search(data) for pattern in SECRETS):
        return None
    return TOKEN.sub(rb'\1REDACTED_COMMISSIONING_TOKEN', data)


def save_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def pack(destination, entries, report):
    """Stream files into an archive; skip credentials and all symlinks."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(destination, 'w:gz', compresslevel=1) as archive:
        for path, name in entries:
            if path.is_symlink() or not path.is_file() or forbidden(name):
                report.append({'path': str(name), 'reason': 'excluded path/link'})
                continue
            before = path.stat()
            data = path.read_bytes()
            cleaned = sanitize(data)
            if cleaned is None:
                report.append({'path': str(name), 'reason': 'credential pattern'})
                continue
            if cleaned != data:
                report.append({'path': str(name), 'reason': 'session token redacted'})
            after = path.stat()
            if before.st_size != after.st_size or before.st_mtime_ns != after.st_mtime_ns:
                report.append({'path': str(name), 'reason': 'live file changed during snapshot'})
            info = tarfile.TarInfo(str(name))
            info.size = len(cleaned)
            info.mode = before.st_mode & 0o777
            info.mtime = int(before.st_mtime)
            archive.addfile(info, io.BytesIO(cleaned))


def entries_under(base):
    for directory, dirs, files in os.walk(base, followlinks=False):
        dirs[:] = sorted(d for d in dirs if d not in EXCLUDED_DIRS)
        for name in sorted(files):
            path = Path(directory) / name
            yield path, path.relative_to(base)


def repack(source, destination, report):
    """Review old tar members without extracting paths or executing content."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(source, 'r:*') as old, tarfile.open(destination, 'w:gz', compresslevel=1) as new:
        for member in old:
            name = PurePosixPath(member.name)
            # Offline dependencies are exported once, separately. Generated ROS
            # trees and old diagnostics are not part of a source recovery archive.
            if (not member.isfile() or forbidden(name)
                    or {'.cache', 'logs', 'log', 'Builds'} & set(name.parts)):
                continue
            data = old.extractfile(member).read()
            cleaned = sanitize(data)
            if cleaned is None:
                report.append({'archive': str(source.relative_to(ROOT)),
                               'path': member.name, 'reason': 'credential pattern'})
                continue
            if cleaned != data:
                report.append({'archive': str(source.relative_to(ROOT)),
                               'path': member.name, 'reason': 'session token redacted'})
            member.size = len(cleaned)
            member.uid = member.gid = 0
            member.uname = member.gname = ''
            new.addfile(member, io.BytesIO(cleaned))


def build(output):
    if output.exists():
        raise SystemExit('Output exists; choose a new directory to preserve the earlier snapshot')
    output.mkdir(parents=True)
    report, inventory = [], []
    # Complete size/mtime inventory, including generated files (not .git).
    for directory, dirs, files in os.walk(ROOT, followlinks=False):
        dirs[:] = sorted(d for d in dirs if d != '.git' and Path(directory) / d != output)
        for name in sorted(files):
            p = Path(directory) / name
            if p.is_symlink():
                inventory.append({'path': str(p.relative_to(ROOT)), 'symlink': os.readlink(p)})
            elif p.is_file():
                st = p.stat()
                inventory.append({'path': str(p.relative_to(ROOT)), 'bytes': st.st_size,
                                  'mtime_ns': st.st_mtime_ns})
    save_json(output / 'inventory.json', inventory)
    print('Inventory:', len(inventory), 'files', flush=True)

    builds = ROOT / 'Unity_Projects/Unitree_VR_Controller/Builds'
    for source in sorted(builds.rglob('*.apk')):
        target = output / 'apk' / source.relative_to(builds)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    print('APK copies ready', flush=True)

    for base in [ROOT / 'Backups', ROOT / 'R1_Teleoperation/backups']:
        for source in sorted(base.rglob('*')):
            if source.is_file() and source.name.endswith(('.tar', '.tar.gz')):
                target = output / 'source-backups' / source.relative_to(ROOT)
                if source.name.endswith('.tar'):
                    target = target.with_suffix('.tar.gz')
                repack(source, target, report)
                print('Reviewed backup:', source.relative_to(ROOT), flush=True)

    cache = ROOT / 'R1_Teleoperation/.cache'
    for directory in ('robot-pov-wheelhouse', 'voice-models'):
        base = cache / directory
        if base.exists():
            # Do not publish failed/partial model downloads.
            entries = [(p, n) for p, n in entries_under(base)
                       if p.stat().st_size > 16384 or p.suffix not in {'.zip', '.partial'}]
            pack(output / 'offline' / (directory + '.tar.gz'), entries, report)
    # Model weights over GitHub's per-object limit are kept as ordered 512 MiB
    # parts, with the original checksum for lossless reconstruction.
    for source in sorted((cache / 'ollama-models').glob('*.gguf')):
        model_dir = output / 'offline' / source.name
        model_dir.mkdir(parents=True)
        parts = []
        with source.open('rb') as stream:
            index = 0
            while block := stream.read(512 * 1024 * 1024):
                part = model_dir / f'part-{index:04d}'
                part.write_bytes(block)
                parts.append({'file': part.name, 'bytes': len(block), 'sha256': sha256(part)})
                index += 1
        save_json(model_dir / 'manifest.json', {'filename': source.name,
                  'bytes': source.stat().st_size, 'sha256': sha256(source), 'parts': parts})
        print('Offline model split:', source.name, flush=True)

    pack(output / 'diagnostics.tar.gz', entries_under(ROOT / 'R1_Teleoperation/logs'), report)
    extras = []
    for base in (ROOT / 'Backups', ROOT / 'R1_Teleoperation/backups'):
        for p, name in entries_under(base):
            if not p.name.endswith(('.tar', '.tar.gz')):
                extras.append((p, p.relative_to(ROOT)))
    pack(output / 'backup-supporting-files.tar.gz', extras, report)
    save_json(output / 'export-review.json', {'created_at': datetime.now(timezone.utc).isoformat(),
        'robot_snapshot': 'NOT_COLLECTED_PC2_POWERED_OFF',
        'excluded_directory_names': sorted(EXCLUDED_DIRS),
        'source_archive_omissions': ['.cache', 'logs', 'log', 'Builds'],
        'review': report})
    sums = []
    for p in sorted(output.rglob('*')):
        if p.is_file():
            sums.append(f'{sha256(p)}  {p.relative_to(output)}')
    (output / 'SHA256SUMS').write_text('\n'.join(sums) + '\n')
    print('Export ready:', output, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    build(args.output.resolve())


if __name__ == '__main__':
    main()
