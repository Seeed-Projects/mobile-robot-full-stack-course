#!/usr/bin/env python3
"""Read-only URDF inventory. No conversion, simulator import or device access."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

def inspect(path, packages=None):
    path = Path(path).resolve()
    packages = {k: Path(v).resolve() for k, v in (packages or {}).items()}
    root = ET.fromstring(path.read_bytes())
    if root.tag != 'robot':
        raise ValueError('Expected a URDF robot root')
    links = root.findall('link')
    names = [link.get('name') for link in links]
    if not names or any(not name for name in names) or len(set(names)) != len(names):
        raise ValueError('Missing or duplicate link name')
    joints, parents, children, joint_names = [], {}, {}, set()
    for joint in root.findall('joint'):
        parent, child = joint.find('parent'), joint.find('child')
        if parent is None or child is None:
            raise ValueError('Joint requires parent and child')
        p, c = parent.get('link'), child.get('link')
        name = joint.get('name')
        if not name or name in joint_names:
            raise ValueError('Missing or duplicate joint name')
        joint_names.add(name)
        if p not in names or c not in names or c in parents:
            raise ValueError('Invalid joint links or multiple parents')
        parents[c] = p
        children.setdefault(p, []).append(c)
        origin, axis = joint.find('origin'), joint.find('axis')
        joints.append(dict(name=name, type=joint.get('type'), parent=p, child=c,
                           xyz='0 0 0' if origin is None else origin.get('xyz', '0 0 0'),
                           rpy='0 0 0' if origin is None else origin.get('rpy', '0 0 0'),
                           axis='1 0 0' if axis is None else axis.get('xyz', '1 0 0')))
    roots = [name for name in names if name not in parents]
    if len(roots) != 1:
        raise ValueError('Expected one connected root')
    visited = set()
    def visit(name):
        if name in visited:
            raise ValueError('Cycle in link tree')
        visited.add(name)
        for child in children.get(name, []):
            visit(child)
    visit(roots[0])
    if len(visited) != len(names):
        raise ValueError('Disconnected or cyclic links')
    meshes, errors = [], []
    for link in links:
        for section in ('visual', 'collision'):
            for item in link.findall(section):
                mesh = item.find('geometry/mesh')
                if mesh is None:
                    continue
                uri = mesh.get('filename', '')
                resolved = None
                if uri.startswith('package://'):
                    package, sep, rel = uri[10:].partition('/')
                    if sep and package in packages:
                        base = packages[package]
                        candidate = (base / rel).resolve()
                        if candidate.is_relative_to(base):
                            resolved = candidate
                elif uri and '://' not in uri:
                    resolved = (path.parent / uri).resolve()
                exists = resolved is not None and resolved.is_file()
                row = dict(link=link.get('name'), role=section, uri=uri,
                           scale=mesh.get('scale', '1 1 1'),
                           resolved=None if resolved is None else str(resolved), exists=exists)
                if exists:
                    row.update(bytes=resolved.stat().st_size,
                               sha256=hashlib.sha256(resolved.read_bytes()).hexdigest())
                else:
                    errors.append('Unresolved mesh: ' + uri)
                meshes.append(row)
    return dict(robot=root.get('name'), urdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                root=roots[0], link_count=len(names), links=names, joints=joints,
                meshes=meshes, errors=errors, valid_inventory=not errors,
                limitation='Inventory only; no unit, dimensions, collision, dynamics or VLN qualification')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('--package', action='append', default=[], metavar='NAME=PATH')
    args = parser.parse_args()
    packages = {}
    for item in args.package:
        name, sep, value = item.partition('=')
        if not sep or not name or not value or name in packages:
            parser.error('Each --package requires a unique NAME=PATH')
        packages[name] = value
    result = inspect(args.urdf, packages)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result['valid_inventory'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
