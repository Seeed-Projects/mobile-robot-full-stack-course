#!/usr/bin/env python3
"""Check mesh provenance, a connected URDF tree, physical inertias and wheel contact."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]


def origin(element):
    result = np.eye(4)
    if element is None:
        return result
    result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
    r, p, y = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                      [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    return result


def validate():
    root = ET.parse(PACKAGE / 'urdf/j501_chassis.urdf').getroot()
    manifest = json.loads((PACKAGE / 'config/cad_manifest.json').read_text())
    for mesh in manifest['meshes']:
        path = PACKAGE / mesh['file']
        if hashlib.sha256(path.read_bytes()).hexdigest() != mesh['sha256']:
            raise ValueError(f'Mesh checksum mismatch: {path}')
    if sorted(p['id'] for p in manifest['parts']) != list(range(359)):
        raise ValueError('CAD part inventory is incomplete or duplicated')
    links = {link.get('name'): link for link in root.findall('link')}
    joints = root.findall('joint')
    if len(links) != len(root.findall('link')) or len(joints) != len(links)-1:
        raise ValueError('Invalid URDF tree size')
    transforms = {'base_footprint': np.eye(4)}
    pending = list(joints)
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in transforms]
        if not ready:
            raise ValueError('Disconnected or cyclic URDF')
        for joint in ready:
            child = joint.find('child').get('link')
            if child in transforms or child not in links:
                raise ValueError('Duplicate or missing child link')
            transforms[child] = transforms[joint.find('parent').get('link')] @ origin(joint.find('origin'))
            pending.remove(joint)
    total_mass = 0
    for name, link in links.items():
        inertial = link.find('inertial')
        if inertial is None:
            if name != 'base_footprint':
                raise ValueError(f'Missing inertia: {name}')
            continue
        mass = float(inertial.find('mass').get('value'))
        i = {key: float(value) for key, value in inertial.find('inertia').attrib.items()}
        tensor = np.array([[i['ixx'], i['ixy'], i['ixz']], [i['ixy'], i['iyy'], i['iyz']], [i['ixz'], i['iyz'], i['izz']]])
        eigenvalues = np.linalg.eigvalsh(tensor)
        if not math.isfinite(mass) or mass <= 0 or not np.isfinite(eigenvalues).all() or min(eigenvalues) <= 0:
            raise ValueError(f'Invalid physical inertia: {name}')
        if eigenvalues[2] > eigenvalues[0] + eigenvalues[1] + 1e-12:
            raise ValueError(f'Inertia triangle inequality failed: {name}')
        total_mass += mass
    contact = {}
    for name, link in links.items():
        for collision in link.findall('collision'):
            cylinder = collision.find('geometry/cylinder')
            if cylinder is None:
                continue
            world = transforms[name] @ origin(collision.find('origin'))
            axis_z = abs(world[2, 2])
            half_z = float(cylinder.get('length'))/2*axis_z + float(cylinder.get('radius'))*math.sqrt(max(0, 1-axis_z**2))
            bottom = float(world[2, 3]-half_z)
            contact[name] = bottom
            if abs(bottom) > 1e-6:
                raise ValueError(f'Wheel contact is not on the ground: {name}: {bottom}')
    if len(contact) != 6:
        raise ValueError('Expected two drive tyres and four casters')
    return {'passed': True, 'cad_part_instances': manifest['part_instances'],
            'visual_meshes': len(manifest['meshes']), 'visual_triangles': sum(m['triangles'] for m in manifest['meshes']),
            'links': len(links), 'joints': len(joints), 'continuous_joints': sum(j.get('type') == 'continuous' for j in joints),
            'estimated_total_mass_kg': total_mass, 'wheel_contact_z_m': contact,
            'checks': ['mesh checksums', 'complete CAD instance mapping', 'connected URDF tree',
                       'positive physical inertia tensors', 'six nominal ground contacts']}


if __name__ == '__main__':
    report = validate()
    output = PACKAGE / 'validation/model_checks.json'
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
