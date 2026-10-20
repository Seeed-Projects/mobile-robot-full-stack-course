#!/usr/bin/env python3
"""Read-only course integrity/source check; no model imports or network access."""
import argparse
import ast
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def check(root=ROOT):
    root = Path(root)
    manifest = json.loads((root / 'overlay-manifest.json').read_text())
    errors = []
    for row in manifest['files']:
        p = root / 'overlay' / row['path']
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != row['sha256']:
            errors.append('Source missing/changed: ' + row['path'])
        elif p.suffix == '.py':
            ast.parse(p.read_text(), filename=str(p))
    checksum = root / 'SHA256SUMS.txt'
    if checksum.is_file():
        for line in checksum.read_text().splitlines():
            expected, name = line.split('  ', 1)
            path = root / name
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                errors.append('Material missing/changed: ' + name)
    else:
        errors.append('SHA256SUMS.txt missing')
    return {'ok': not errors, 'overlay_files': len(manifest['files']),
            'upstream_commit': manifest['upstream_commit'], 'errors': errors,
            'not_tested': ['fresh-board installation', 'GPU inference', 'new navigation']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    result = check(parser.parse_args().root)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['ok'] else 1)
