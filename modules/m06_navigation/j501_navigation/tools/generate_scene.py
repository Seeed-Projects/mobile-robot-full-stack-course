#!/usr/bin/python3
"""Generate matching Gazebo geometry and a static occupancy grid from one scene."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


def write_frustum(path, radius0, radius1, z0, z1, segments=48):
    """Closed STL cone/frustum; compatible with Fortress's SDF 1.8 mesh geometry."""
    triangles = []
    for i in range(segments):
        a, b = 2*math.pi*i/segments, 2*math.pi*(i+1)/segments
        p, q = (radius0*math.cos(a), radius0*math.sin(a), z0), (radius0*math.cos(b), radius0*math.sin(b), z0)
        r, s = (radius1*math.cos(a), radius1*math.sin(a), z1), (radius1*math.cos(b), radius1*math.sin(b), z1)
        triangles.extend([(p, q, r), ((0, 0, z0), q, p)])
        if radius1 > 0:
            triangles.extend([(q, s, r), ((0, 0, z1), r, s)])
    lines = ['solid traffic_cone']
    for triangle in triangles:
        p, q, r = np.array(triangle)
        normal = np.cross(q-p, r-p)
        normal /= np.linalg.norm(normal)
        lines.extend(['  facet normal ' + ' '.join(map(str, normal)), '    outer loop'])
        lines.extend('      vertex ' + ' '.join(map(str, point)) for point in triangle)
        lines.extend(['    endloop', '  endfacet'])
    path.write_text('\n'.join([*lines, 'endsolid traffic_cone']) + '\n')


def traffic_cone(cone):
    name, radius, height, base = cone['name'], cone['radius'], cone['height'], cone['base_height']
    folder = PACKAGE / 'models' / name
    (folder / 'meshes').mkdir(parents=True, exist_ok=True)
    write_frustum(folder / 'meshes/cone.stl', radius, 0.0, base, height)
    sdf = ET.Element('sdf', version='1.8')
    model = ET.SubElement(sdf, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='link')
    for kind in ['collision', 'visual']:
        shape = ET.SubElement(link, kind, name='base')
        ET.SubElement(shape, 'pose').text = f'0 0 {base/2} 0 0 0'
        geom = ET.SubElement(ET.SubElement(shape, 'geometry'), 'box')
        ET.SubElement(geom, 'size').text = f'{cone["size"][0]} {cone["size"][1]} {base}'
        if kind == 'visual':
            material = ET.SubElement(shape, 'material')
            for key in ['ambient', 'diffuse']:
                ET.SubElement(material, key).text = '.9 .22 .02 1'
        shape = ET.SubElement(link, kind, name='cone')
        mesh = ET.SubElement(ET.SubElement(shape, 'geometry'), 'mesh')
        ET.SubElement(mesh, 'uri').text = f'model://{name}/meshes/cone.stl'
        if kind == 'visual':
            material = ET.SubElement(shape, 'material')
            for key in ['ambient', 'diffuse']:
                ET.SubElement(material, key).text = '1 .3 .02 1'
    for i, (lower, upper) in enumerate([(0.29, 0.40), (0.58, 0.69)]):
        z0, z1 = base+(height-base)*lower, base+(height-base)*upper
        filename = f'band_{i}.stl'
        write_frustum(folder / 'meshes' / filename, radius*(1-lower)+0.001, radius*(1-upper)+0.001, z0, z1)
        shape = ET.SubElement(link, 'visual', name=f'white_band_{i}')
        mesh = ET.SubElement(ET.SubElement(shape, 'geometry'), 'mesh')
        ET.SubElement(mesh, 'uri').text = f'model://{name}/meshes/{filename}'
        material = ET.SubElement(shape, 'material')
        for key in ['ambient', 'diffuse']:
            ET.SubElement(material, key).text = '.95 .95 .95 1'
    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(folder / 'model.sdf', encoding='unicode', xml_declaration=True)
    (folder / 'model.config').write_text(f'''<?xml version="1.0"?>
<model><name>{name}</name><version>1.0</version><sdf version="1.8">model.sdf</sdf>
<description>Orange and white traffic cone, generated from scenes/office.yaml.</description></model>
''')


def generate():
    scene = yaml.safe_load((PACKAGE / 'scenes/office.yaml').read_text())
    for folder in ['maps', 'worlds', 'assets', 'validation']:
        (PACKAGE / folder).mkdir(exist_ok=True)
    resolution = scene['resolution']
    width, height = [round(v/resolution) for v in scene['size']]
    x = scene['origin'][0] + (np.arange(width)+0.5)*resolution
    y = scene['origin'][1] + (np.arange(height)+0.5)*resolution
    xx, yy = np.meshgrid(x, y)
    pixels = np.full((height, width), 205, dtype=np.uint8)
    pixels[(abs(xx) < 5.9) & (abs(yy) < 4.9)] = 254
    world = ET.Element('sdf', version='1.8')
    w = ET.SubElement(world, 'world', name=scene['name'])
    physics = ET.SubElement(w, 'physics', name='default', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_factor').text = '1'
    ET.SubElement(w, 'gravity').text = '0 0 -9.81'
    for library, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                          ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'), ('imu', 'Imu')]:
        plugin = ET.SubElement(w, 'plugin', filename=f'ignition-gazebo-{library}-system', name=f'ignition::gazebo::systems::{name}')
        if name == 'Sensors':
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    light = ET.SubElement(w, 'light', name='sun', type='directional')
    for key, value in [('pose', '0 0 10 0 0 0'), ('diffuse', '.9 .9 .9 1'), ('direction', '-.4 -.5 -1')]:
        ET.SubElement(light, key).text = value
    floor = ET.SubElement(w, 'model', name='ground')
    ET.SubElement(floor, 'static').text = 'true'
    link = ET.SubElement(floor, 'link', name='link')
    for kind in ['collision', 'visual']:
        shape = ET.SubElement(link, kind, name='floor')
        plane = ET.SubElement(ET.SubElement(shape, 'geometry'), 'plane')
        ET.SubElement(plane, 'normal').text = '0 0 1'
        ET.SubElement(plane, 'size').text = '20 20'
        if kind == 'visual':
            material = ET.SubElement(shape, 'material')
            ET.SubElement(material, 'ambient').text = '.65 .67 .7 1'
            ET.SubElement(material, 'diffuse').text = '.65 .67 .7 1'
    for box in scene['boxes']:
        cx, cy = box['center']
        sx, sy = box['size']
        pixels[(abs(xx-cx) <= sx/2) & (abs(yy-cy) <= sy/2)] = 0
        model = ET.SubElement(w, 'model', name=box['name'])
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = f'{cx} {cy} 0.6 0 0 0'
        link = ET.SubElement(model, 'link', name='link')
        for kind in ['collision', 'visual']:
            shape = ET.SubElement(link, kind, name='box')
            geom = ET.SubElement(ET.SubElement(shape, 'geometry'), 'box')
            ET.SubElement(geom, 'size').text = f'{sx} {sy} 1.2'
            if kind == 'visual':
                material = ET.SubElement(shape, 'material')
                color = '.35 .48 .62 1' if 'shelf' in box['name'] else '.75 .72 .65 1'
                ET.SubElement(material, 'ambient').text = color
                ET.SubElement(material, 'diffuse').text = color
    for cone in scene.get('unmapped_cones', []):
        traffic_cone(cone)
        include = ET.SubElement(w, 'include')
        ET.SubElement(include, 'uri').text = f'model://{cone["name"]}'
        ET.SubElement(include, 'name').text = cone['name']
        ET.SubElement(include, 'pose').text = f'{cone["center"][0]} {cone["center"][1]} 0 0 0 0'
    map_file = PACKAGE / 'maps/office.pgm'
    map_file.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + pixels[::-1].tobytes())
    metadata = {'image': 'office.pgm', 'mode': 'trinary', 'resolution': resolution,
                'origin': [*scene['origin'], 0.0], 'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}
    (PACKAGE / 'maps/office.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    ET.indent(world, space='  ')
    ET.ElementTree(world).write(PACKAGE / 'worlds/office.sdf', encoding='unicode', xml_declaration=True)
    # Survey the permanent environment first. Add movable cones only after saving the map.
    for include in list(w.findall('include')):
        if include.findtext('name') in {cone['name'] for cone in scene.get('unmapped_cones', [])}:
            w.remove(include)
    ET.ElementTree(world).write(PACKAGE / 'worlds/office_mapping.sdf', encoding='unicode', xml_declaration=True)
    report = {'resolution_m': resolution, 'size_cells': [width, height], 'static_boxes': len(scene['boxes']),
              'occupied_cells': int((pixels == 0).sum()), 'free_cells': int((pixels == 254).sum()),
              'unknown_cells': int((pixels == 205).sum()), 'source': 'scenes/office.yaml',
              'unmapped_cones': scene.get('unmapped_cones', [])}
    (PACKAGE / 'validation/scene.json').write_text(json.dumps(report, indent=2) + '\n')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.imshow(pixels, origin='lower', cmap='gray', vmin=0, vmax=255,
                  extent=[x[0]-resolution/2, x[-1]+resolution/2, y[0]-resolution/2, y[-1]+resolution/2])
        ax.scatter(*scene['initial_pose'][:2], c='green', s=70, label='Start')
        for name, pose in scene['goals'].items():
            ax.scatter(*pose[:2], marker='x', s=70)
            ax.annotate(name, pose[:2], xytext=(8, 8), textcoords='offset points')
        ax.scatter(-3.3, -3.15, marker='s', c='orange', s=60, label='Optional box (not in map)')
        for cone in scene.get('unmapped_cones', []):
            ax.add_patch(plt.Rectangle((cone['center'][0]-cone['size'][0]/2, cone['center'][1]-cone['size'][1]/2),
                                      *cone['size'], facecolor='orange', edgecolor='darkorange', label='Arrival cone (not in map)'))
            ax.annotate(cone['name'], cone['center'], xytext=(8, -18), textcoords='offset points')
        ax.set(xlabel='map X (m)', ylabel='map Y (m)', title='J501 multi-room navigation lab')
        ax.legend(loc='lower left')
        fig.tight_layout()
        fig.savefig(PACKAGE / 'assets/office_map.png', dpi=150)
        plt.close(fig)
    except ImportError:
        pass
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    generate()
