#!/usr/bin/env python3
"""Check/install a frozen course overlay into a matching, fresh upstream clone."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

MATERIALS = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(project, apply=False):
    project = Path(project).resolve()
    manifest = json.loads((MATERIALS / 'overlay-manifest.json').read_text())
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=project, text=True).strip()
    if head != manifest['upstream_commit']:
        raise ValueError('Use the exact upstream commit in overlay-manifest.json')
    pending = []
    for row in manifest['files']:
        relative = Path(row['path'])
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Unsafe overlay path')
        source, target = MATERIALS / 'overlay' / relative, project / relative
        if sha(source) != row['sha256']:
            raise ValueError(f'Course source changed: {relative}')
        if target.is_symlink() or not target.resolve().is_relative_to(project):
            raise ValueError(f'Symlink/escaping destination: {relative}')
        if target.exists():
            actual = sha(target)
            if actual == row['sha256']:
                continue
            if actual != row['upstream_sha256']:
                raise ValueError(f'Preserve local edits; refusing to overwrite: {relative}')
        elif row['upstream_sha256'] is not None:
            raise ValueError(f'Missing upstream file: {relative}')
        pending.append((source, target))
    if apply:
        for source, target in pending:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    return {'mode': 'applied' if apply else 'dry-run', 'files_to_copy': len(pending),
            'upstream_commit': head, 'project': str(project)}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, required=True)
    parser.add_argument('--apply', action='store_true', help='Without this flag, inspect only')
    args = parser.parse_args()
    print(json.dumps(install(args.project, args.apply), indent=2))
