#!/usr/bin/env python3
"""Read-only scan of tracked/publication candidates, including nested archives.

Reports filenames/categories, never matched credentials. This is a bounded
heuristic, not a proof that every secret or personal datum has been removed.
Large binary payloads are checksum-verified separately; archive text members
up to 32 MiB and nested archive containers up to 256 MiB are inspected.
"""
import io
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import tarfile
import zipfile

ROOT = Path(__file__).resolve().parents[2]
PATTERNS = {
    'private-key': re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----(?:\r?\n|\\n)[A-Za-z0-9+/]{32,}'),
    'github-token': re.compile(rb'(?:github_pat_[A-Za-z0-9_]{30,}|gh[pousr]_[A-Za-z0-9]{30,})'),
    'openai-token': re.compile(rb'sk-(?:proj-|svcacct-)[A-Za-z0-9_-]{30,}'),
    'aws-access-key': re.compile(rb'\bAKIA[0-9A-Z]{16}\b'),
    'sshpass-literal': re.compile(rb'sshpass\s+-p\s+[\x22\x27]?[a-zA-Z0-9]{4,}'),
}
ARCHIVES = ('.zip', '.apk', '.aar', '.whl', '.tar', '.tar.gz', '.tgz')
TEXT_SUFFIXES = ('.log', '.txt', '.md', '.json', '.jsonl', '.yaml', '.yml', '.py', '.sh', '.env')
FINDINGS, SKIPPED = [], []
TEXT_MAX, ARCHIVE_MAX = 32 * 1024**2, 256 * 1024**2


def check_name(name, label):
    p = PurePosixPath(name)
    if (p.is_absolute() or '..' in p.parts or set(p.parts) & {'.ssh', '.gnupg', '.nx'}
            or p.name in {'.env', '.git-credentials', 'id_rsa', 'id_ed25519', 'r1_pc2_ed25519'}
            or p.name.endswith(('.local.env', '.venue.env'))
            or p.suffix in {'.key', '.p12', '.jks', '.keystore'}):
        FINDINGS.append({'path': label, 'category': 'private-or-unsafe-path'})


def scan_data(data, label):
    for category, pattern in PATTERNS.items():
        if pattern.search(data):
            FINDINGS.append({'path': label, 'category': category})


def scan_stream(stream, name, label, depth=0):
    if not name.lower().endswith(ARCHIVES):
        overlap = b''
        while block := stream.read(4 * 1024**2):
            scan_data(overlap + block, label)
            overlap = block[-4096:]
        return
    if depth > 4:
        SKIPPED.append({'path': label, 'reason': 'nested-depth'})
        return
    try:
        if name.lower().endswith(('.zip', '.apk', '.aar', '.whl')):
            with zipfile.ZipFile(stream) as archive:
                for info in archive.infolist():
                    member = label + '!' + info.filename
                    check_name(info.filename, member)
                    if info.is_dir():
                        continue
                    bound = ARCHIVE_MAX if info.filename.lower().endswith(ARCHIVES) else TEXT_MAX
                    if info.file_size > bound:
                        SKIPPED.append({'path': member, 'reason': 'large-binary-member'})
                        continue
                    with archive.open(info) as entry:
                        data = entry.read(bound + 1)
                    scan_stream(io.BytesIO(data), info.filename, member, depth + 1)
        else:
            with tarfile.open(fileobj=stream, mode='r:*') as archive:
                for info in archive:
                    member = label + '!' + info.name
                    check_name(info.name, member)
                    if not info.isfile():
                        continue
                    bound = ARCHIVE_MAX if info.name.lower().endswith(ARCHIVES) else TEXT_MAX
                    if info.size > bound and info.name.lower().endswith(TEXT_SUFFIXES):
                        with archive.extractfile(info) as entry:
                            scan_stream(entry, info.name, member, depth + 1)
                        continue
                    if info.size > bound:
                        SKIPPED.append({'path': member, 'reason': 'large-binary-member'})
                        continue
                    with archive.extractfile(info) as entry:
                        data = entry.read(bound + 1)
                    scan_stream(io.BytesIO(data), info.name, member, depth + 1)
    except (OSError, ValueError, tarfile.TarError, zipfile.BadZipFile, RuntimeError) as error:
        FINDINGS.append({'path': label, 'category': 'unreadable-archive', 'type': type(error).__name__})


def main():
    raw = subprocess.check_output(['git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard'], cwd=ROOT)
    paths = sorted(set(p.decode() for p in raw.split(b'\0') if p))
    for name in paths:
        path = ROOT / name
        check_name(name, name)
        if path.is_symlink() or not path.is_file():
            continue
        with path.open('rb') as stream:
            scan_stream(stream, name, name)
    print(json.dumps({'files': len(paths), 'findings': FINDINGS, 'not_fully_inspected': SKIPPED}, indent=2))
    return bool(FINDINGS)


if __name__ == '__main__':
    raise SystemExit(main())
