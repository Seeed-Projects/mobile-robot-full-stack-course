#!/usr/bin/env python3
"""Export this course's pinned STEP assembly, retaining all 359 part instances.

Requires requirements-cad.txt, but no ROS, FreeCAD GUI, or ROS 1 converter.
Joint ownership is an explicit mapping for this STEP, not inferred CAD mates.
"""

import argparse
import hashlib
import json
import tempfile
from pathlib import Path

import numpy as np
import trimesh
import xacro
import yaml
from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepTools import BRepTools
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Shape
from OCP.XCAFDoc import XCAFDoc_DocumentTool

SOURCE_SHA256 = '93d7e688bdc57ac7ca547eb50eddfc30ff215267561c3df3195cc89f002c7efd'
PACKAGE = Path(__file__).resolve().parents[1]
# CAD +X is the provisional front; the source contains no verified vehicle-front datum.
ORIGIN_MM = np.array([76.23092482, -166.9341003, 215.54363585])
WHEELS = {'left': 96, 'right': 46}
CARRIERS = {'left': [97, 99, 102], 'right': [44, 45, 48]}
CASTERS = {'rear_left': 40, 'rear_right': 50, 'front_left': 51, 'front_right': 53}


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read_parts(source, cache):
    inventory = cache / 'inventory.json'
    marker = cache / 'source.sha256'
    if inventory.exists() and marker.exists() and marker.read_text().strip() == SOURCE_SHA256:
        return json.loads(inventory.read_text())
    doc = TDocStd_Document(TCollection_ExtendedString('BinXCAF'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(False)
    reader.SetLayerMode(False)
    reader.SetGDTMode(False)
    reader.SetMatMode(False)
    print('Reading and transferring STEP assembly...', flush=True)
    if reader.ReadFile(str(source)) != IFSelect_RetDone or not reader.Transfer(doc):
        raise RuntimeError('STEP import failed')
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    rows = []

    def name(label):
        attr = TDataStd_Name()
        return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else '(unnamed)'

    def walk(label, loc, path):
        if tool.IsReference_s(label):
            target = TDF_Label()
            if not tool.GetReferredShape_s(label, target):
                raise RuntimeError('Unresolved STEP instance')
            walk(target, loc.Multiplied(tool.GetLocation_s(label)), path + [name(label)])
        elif tool.IsAssembly_s(label):
            seq = TDF_LabelSequence()
            tool.GetComponents_s(label, seq)
            for index in range(1, seq.Length() + 1):
                walk(seq.Value(index), loc, path + [name(label)])
        elif tool.IsSimpleShape_s(label):
            shape = tool.GetShape_s(label).Moved(loc)
            box = Bnd_Box()
            BRepBndLib.AddOptimal_s(shape, box, False, False)
            props = GProp_GProps()
            BRepGProp.VolumeProperties_s(shape, props)
            center = props.CentreOfMass()
            file = cache / f'part_{len(rows):03d}.brep'
            BRepTools.Write_s(shape, str(file))
            rows.append({'id': len(rows), 'label': name(label), 'path': path,
                         'bounds_mm': list(box.Get()), 'volume_mm3': props.Mass(),
                         'center_mm': [center.X(), center.Y(), center.Z()], 'brep': str(file)})

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for index in range(1, roots.Length() + 1):
        walk(roots.Value(index), TopLoc_Location(), [])
    inventory.write_text(json.dumps(rows, ensure_ascii=False, indent=2) + '\n')
    marker.write_text(SOURCE_SHA256 + '\n')
    return rows


def mesh_parts(rows, cache):
    meshes = {}
    for row in rows:
        index = row['id']
        stl = cache / f'part_{index:03d}.stl'
        if not stl.exists():
            shape = TopoDS_Shape()
            BRepTools.Read_s(shape, row['brep'], BRep_Builder())
            BRepMesh_IncrementalMesh(shape, 0.6, False, 0.25, True).Perform()
            writer = StlAPI_Writer()
            writer.ASCIIMode = False
            if not writer.Write(shape, str(stl)):
                raise RuntimeError(f'Meshing failed for part {index}')
        mesh = trimesh.load_mesh(stl, process=True)
        mesh.apply_scale(0.001)
        meshes[index] = mesh
        if index % 50 == 0:
            print(f'Meshed {index + 1}/{len(rows)}', flush=True)
    return meshes


def export(source, cache, output):
    if sha256(source) != SOURCE_SHA256:
        raise ValueError('STEP hash differs from the mapped assembly; inspect and update joint mapping first')
    rows = read_parts(source, cache)
    if len(rows) != 359 or any(rows[index]['label'] != 'NONE' for index in WHEELS.values()):
        raise ValueError('STEP part inventory differs from the joint mapping')
    meshes = mesh_parts(rows, cache)
    origin = ORIGIN_MM / 1000
    geometry = {'cad_origin_mm': ORIGIN_MM.tolist(), 'front': 'provisional CAD +X', 'wheels': {}, 'casters': []}
    # Infer tread width from outer tyre vertices, excluding the protruding motor shaft.
    for side, index in WHEELS.items():
        mesh = meshes[index]
        radial = np.linalg.norm(mesh.vertices[:, [0, 2]] - origin[[0, 2]], axis=1)
        tread = mesh.vertices[radial > 0.075]
        center = np.array([origin[0], (tread[:, 1].min() + tread[:, 1].max()) / 2, origin[2]])
        geometry['wheels'][side] = {'center_m': (center - origin).tolist(), 'radius_m': float(radial.max()),
                                    'width_m': float(np.ptp(tread[:, 1]))}
    geometry['wheel_radius_m'] = max(w['radius_m'] for w in geometry['wheels'].values())
    geometry['track_width_m'] = geometry['wheels']['left']['center_m'][1] - geometry['wheels']['right']['center_m'][1]
    for name, index in CASTERS.items():
        box = np.array(rows[index]['bounds_mm']) / 1000
        y = (box[1] + box[4]) / 2
        # Top-plate pivot, through-axle centre, and 15 mm radius identified from CAD cylinders.
        pivot = np.array([box[0] + 0.015, y, box[5]])
        axle = pivot + np.array([0.0135, 0, -0.027])
        geometry['casters'].append({'name': name, 'pivot_m': (pivot - origin).tolist(),
                                    'axle_offset_m': (axle - pivot).tolist(), 'radius_m': 0.015,
                                    'width_m': 0.020, 'cad_part': index})
    caster_ground = min(c['pivot_m'][2] + c['axle_offset_m'][2] - c['radius_m'] for c in geometry['casters'])
    geometry['suspension_compression_m'] = geometry['wheel_radius_m'] + caster_ground
    geometry['body_size_m'] = [0.49, 0.49, 0.156]
    geometry['body_collision_center_m'] = [0, 0, 0.07798]
    groups = {}
    for side, index in WHEELS.items():
        groups[f'{side}_wheel'] = ([index], np.array(geometry['wheels'][side]['center_m']) + origin, 10000)
        groups[f'{side}_carrier'] = (CARRIERS[side], np.array(geometry['wheels'][side]['center_m']) + origin, 4000)
    for caster in geometry['casters']:
        groups[f"caster_{caster['name']}"] = ([caster['cad_part']], np.array(caster['pivot_m']) + origin, 9000)
    moving = {index for indices, _, _ in groups.values() for index in indices}
    electronics = set(range(2, 34)) | set(range(61, 93)) | set(range(104, 359))
    groups['electronics'] = (sorted(electronics), origin, 55000)
    groups['frame'] = (sorted(set(range(359)) - moving - electronics), origin, 40000)
    assigned = [index for indices, _, _ in groups.values() for index in indices]
    if sorted(assigned) != list(range(359)):
        raise ValueError('Each CAD instance must belong to exactly one visual group')
    (output / 'meshes').mkdir(parents=True, exist_ok=True)
    report = {'source_file': source.name, 'source_sha256': SOURCE_SHA256, 'part_instances': len(rows),
              'units': 'meters', 'linear_deflection_mm': 0.6, 'angular_deflection_rad': 0.25,
              'simplification': 'visual-only quadric decimation; not a collision mesh or error bound',
              'meshes': [], 'parts': []}
    for name, (indices, center, target) in groups.items():
        mesh = trimesh.util.concatenate([meshes[index] for index in indices])
        mesh.apply_translation(-center)
        before = len(mesh.faces)
        if before > target:
            mesh = mesh.simplify_quadric_decimation(face_count=target)
        mesh.remove_unreferenced_vertices()
        file = output / 'meshes' / f'{name}.stl'
        mesh.export(file)
        report['meshes'].append({'file': 'meshes/' + file.name, 'sha256': sha256(file),
                                 'triangles_before': before, 'triangles': len(mesh.faces),
                                 'bounds_m': mesh.bounds.tolist(), 'cad_parts': indices})
        print(f'{name}: {before} -> {len(mesh.faces)} triangles', flush=True)
        report['parts'].extend({'id': index, 'label': rows[index]['label'], 'group': name,
                                'bounds_mm': rows[index]['bounds_mm']} for index in indices)
    (output / 'config' / 'geometry.yaml').write_text(yaml.safe_dump(geometry, sort_keys=False))
    (output / 'config' / 'cad_manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    xacro_file = output / 'urdf' / 'j501_chassis.urdf.xacro'
    for name, mappings in [('j501_chassis.urdf', {}),
                           ('j501_chassis_cad.urdf', {'simulation': 'false', 'add_lidar': 'false', 'suspension_compression': '0'})]:
        document = xacro.process_file(str(xacro_file), mappings=mappings)
        (output / 'urdf' / name).write_text(document.toprettyxml(indent='  '))
    print(json.dumps({k: geometry[k] for k in ['wheel_radius_m', 'track_width_m', 'suspension_compression_m']}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('step', type=Path)
    parser.add_argument('--output', type=Path, default=PACKAGE)
    parser.add_argument('--cache-dir', type=Path)
    args = parser.parse_args()
    if args.cache_dir:
        args.cache_dir.mkdir(parents=True, exist_ok=True)
        export(args.step.resolve(), args.cache_dir.resolve(), args.output.resolve())
    else:
        with tempfile.TemporaryDirectory(prefix='j501-step-') as cache:
            export(args.step.resolve(), Path(cache), args.output.resolve())


if __name__ == '__main__':
    main()
