#!/usr/bin/env python3
"""Verify a separately supplied CAD bundle; relocate only an offline copy."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import tarfile

MATERIALS = Path(__file__).resolve().parents[1]


def prepare(project, bundle, apply=False):
    project, bundle = Path(project).resolve(), Path(bundle).resolve()
    manifest = json.loads((MATERIALS / 'configs/asset-manifest.json').read_text())
    expected = {r['file']: r['sha256'] for r in manifest['files']}
    contents = {}
    with tarfile.open(bundle) as archive:
        for member in archive.getmembers():
            if member.isdir():
                continue
            name = Path(member.name)
            if not member.isfile() or name.is_absolute() or '..' in name.parts or len(name.parts) != 2 or name.parts[0] != 'meshes':
                raise ValueError('Unexpected asset member: ' + member.name)
            key = name.name
            if key not in expected or key in contents:
                raise ValueError('Unexpected/duplicate mesh: ' + key)
            if member.size > 100_000_000:
                raise ValueError('Unexpectedly large mesh')
            contents[key] = archive.extractfile(member).read()
            if hashlib.sha256(contents[key]).hexdigest() != expected[key]:
                raise ValueError('CAD hash mismatch: ' + key)
    if set(contents) != set(expected):
        raise ValueError('Incomplete CAD bundle')
    destination = project / 'course-assets'
    if destination.exists():
        raise ValueError('Keep existing assets; choose a new clone or inspect them manually')
    chassis = json.loads((MATERIALS / 'configs/chassis.original.json').read_text())
    sdk = json.loads((MATERIALS / 'configs/sdk-chassis.json').read_text())
    if chassis['geometry'] != sdk['geometry']:
        raise ValueError('Frozen geometry configuration mismatch')
    chassis['sdk_chassis_config'] = str(destination / 'dm-h65/sdk-chassis.json')
    task = json.loads((MATERIALS / 'configs/task.original.json').read_text())
    task['config_path'] = str(destination / 'dm-h65/config.json')
    # This is a relocated course copy, not the original registration artifact.
    task['config_sha256'] = hashlib.sha256(json.dumps(chassis, indent=2).encode()).hexdigest()
    task['course_relocation'] = True
    if apply:
        meshdir = destination / 'dm-h65'
        meshdir.mkdir(parents=True)
        for name, content in contents.items():
            (meshdir / name).write_bytes(content)
        (meshdir / 'manifest.json').write_text(json.dumps(manifest, indent=2))
        (meshdir / 'config.json').write_text(json.dumps(chassis, indent=2))
        (meshdir / 'sdk-chassis.json').write_text(json.dumps(sdk, indent=2))
        (destination / 'task.json').write_text(json.dumps(task, indent=2))
        (destination / 'preparation.json').write_text(json.dumps({
            'source_bundle_sha256': hashlib.sha256(bundle.read_bytes()).hexdigest(),
            'mesh_count': len(contents), 'relocated_copy': True,
            'original_config_sha256': hashlib.sha256((MATERIALS / 'configs/chassis.original.json').read_bytes()).hexdigest()
        }, indent=2))
    return {'mode': 'applied' if apply else 'dry-run', 'mesh_count': len(contents),
            'destination': str(destination)}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, required=True)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    print(json.dumps(prepare(args.project, args.bundle, args.apply), indent=2))
