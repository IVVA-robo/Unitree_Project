#!/usr/bin/env python3
"""Reconstruct an exported GGUF into a NEW file, validating every checksum."""
import argparse
import hashlib
import json
from pathlib import Path


def restore(directory, output):
    manifest = json.loads((directory / 'manifest.json').read_text())
    parts = manifest['parts']
    if not parts or len({p['file'] for p in parts}) != len(parts):
        raise ValueError('Missing or duplicate parts')
    for part in parts:
        name = part['file']
        if Path(name).name != name or name in {'.', '..'}:
            raise ValueError('Unsafe part name')
        source = directory / name
        if source.is_symlink() or not source.is_file():
            raise ValueError('Part must be a regular file')
    total, digest = 0, hashlib.sha256()
    # Never overwrite an existing model. An incomplete output is deliberately
    # retained for diagnosis on failure and must not be used as a model.
    with output.open('xb') as target:
        for part in parts:
            part_digest, size = hashlib.sha256(), 0
            with (directory / part['file']).open('rb') as source:
                for block in iter(lambda: source.read(1024 * 1024), b''):
                    part_digest.update(block)
                    digest.update(block)
                    size += len(block)
                    target.write(block)
            if size != part['bytes'] or part_digest.hexdigest() != part['sha256']:
                raise ValueError('Part mismatch; output is INCOMPLETE: ' + part['file'])
            total += size
    if total != manifest['bytes'] or digest.hexdigest() != manifest['sha256']:
        raise ValueError('Model mismatch; output is INVALID')
    return digest.hexdigest()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    print('Verified:', restore(args.directory, args.output))
