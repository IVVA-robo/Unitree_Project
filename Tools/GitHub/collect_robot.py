#!/usr/bin/env python3
"""Download explicit R1 PC2 project paths over SSH; no remote writes or service calls."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

DEFAULT_PATHS = [
    '/home/unitree/VoiceAssistant', '/opt/R1Service',
    '/home/unitree/Unitree_Project', '/home/unitree/unitree_sdk2',
    '/home/unitree/unitree_sdk2_python', '/home/unitree/ros2_ws',
    '/home/unitree/.config/systemd/user',
    '/etc/systemd/system/robot-voice-service.service',
    '/etc/systemd/system/robot-voice-service.service.d',
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='unitree@192.168.123.164')
    parser.add_argument('--identity', type=Path)
    parser.add_argument('--path', action='append', dest='paths')
    parser.add_argument('--output', required=True, type=Path,
                        help='NEW local private directory; review archive before publishing')
    args = parser.parse_args()
    if args.host.startswith('-'):
        parser.error('invalid SSH host')
    paths = args.paths or DEFAULT_PATHS
    if any(not p.startswith(('/home/unitree/', '/opt/', '/etc/systemd/system/'))
           or '..' in Path(p).parts for p in paths):
        parser.error('only explicit robot project/service paths are supported')
    ssh = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
           '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=5',
           '-o', 'ServerAliveCountMax=3']
    if args.identity:
        ssh += ['-i', str(args.identity)]
    ssh += [args.host, 'python3 -']
    inventory_code = '''import json,os,socket
paths = PATHS
print(json.dumps({'hostname':socket.gethostname(),'paths':[{'path':p,'exists':os.path.exists(p),'readable':os.access(p,os.R_OK)} for p in paths]}))
'''.replace('PATHS', repr(paths))
    result = subprocess.run(ssh, input=inventory_code, text=True, capture_output=True, check=True)
    inventory = json.loads(result.stdout)
    present = [item['path'] for item in inventory['paths'] if item['exists'] and item['readable']]
    if not present:
        raise RuntimeError('No readable project paths; nothing was copied')
    args.output.mkdir(mode=0o700, parents=True, exist_ok=False)
    (args.output / 'inventory.json').write_text(json.dumps(inventory, indent=2) + '\n')
    # tar streamed to stdout. Symlinks are stored, never followed. Raw .env,
    # SSH keys, caches and command histories stay on PC2. No sudo is used.
    code = '''import sys,tarfile
from pathlib import PurePosixPath
paths = PATHS
def allow(info):
 p=PurePosixPath(info.name)
 if set(p.parts)&{'.git','.ssh','.gnupg','.cache','__pycache__','build','install'}:return None
 if p.name in {'.env','.bash_history','.git-credentials','id_rsa','id_ed25519','r1_pc2_ed25519'} or p.suffix in {'.pem','.key','.p12','.keystore','.jks'} or p.name.endswith(('.local.env','.venue.env')):return None
 return info
with tarfile.open(fileobj=sys.stdout.buffer,mode='w|gz') as t:
 for p in paths:t.add(p,arcname=p.lstrip('/'),recursive=True,filter=allow)
'''.replace('PATHS', repr(present))
    archive = args.output / 'robot-projects.tar.gz'
    with archive.open('xb') as stream:
        subprocess.run(ssh, input=code.encode(), stdout=stream, check=True)
    digest = hashlib.sha256()
    with archive.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    (args.output / 'SHA256SUMS').write_text(digest.hexdigest() + '  robot-projects.tar.gz\n')
    (args.output / 'COMPLETE.json').write_text(json.dumps({
        'transfer_complete': True, 'publication_review_complete': False,
        'present_paths': present,
        'missing_or_unreadable': [item for item in inventory['paths']
                                  if not item['exists'] or not item['readable']],
    }, indent=2) + '\n')
    print('Copied to', args.output, '; inspect inventory/missing paths and scan contents before publication')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print('Robot export incomplete:', error, file=sys.stderr)
        sys.exit(2)
