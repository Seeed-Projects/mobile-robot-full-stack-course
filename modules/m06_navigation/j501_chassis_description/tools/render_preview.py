#!/usr/bin/env python3
"""Render the actual exported meshes in CAD and ground-contact configurations."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import vtk

PACKAGE = Path(__file__).resolve().parents[1]


def pose(origin):
    transform = np.eye(4)
    if origin is None:
        return transform
    transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    r, p, y = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    transform[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    return transform


def link_poses(root):
    transforms = {'base_footprint': np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in transforms]
        if not ready:
            raise ValueError('URDF is not a connected tree')
        for joint in ready:
            transforms[joint.find('child').get('link')] = transforms[joint.find('parent').get('link')] @ pose(joint.find('origin'))
            pending.remove(joint)
    return transforms


def renderer(file, title, viewport):
    root = ET.parse(file).getroot()
    transforms = link_poses(root)
    materials = {m.get('name'): [float(x) for x in m.find('color').get('rgba').split()[:3]] for m in root.findall('material')}
    ren = vtk.vtkRenderer()
    ren.SetViewport(*viewport)
    ren.SetBackground(0.14, 0.16, 0.19)
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            cylinder = visual.find('geometry/cylinder')
            if mesh is not None:
                source = vtk.vtkSTLReader()
                source.SetFileName(str(PACKAGE / 'meshes' / Path(mesh.get('filename')).name))
                rotation = np.eye(4)
            elif cylinder is not None:
                source = vtk.vtkCylinderSource()
                source.SetRadius(float(cylinder.get('radius')))
                source.SetHeight(float(cylinder.get('length')))
                source.SetResolution(48)
                rotation = pose(ET.Element('origin', rpy=f'{math.pi/2} 0 0'))
            else:
                continue
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(source.GetOutputPort())
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            matrix = vtk.vtkMatrix4x4()
            world = transforms[link.get('name')] @ pose(visual.find('origin')) @ rotation
            for i in range(4):
                for j in range(4):
                    matrix.SetElement(i, j, world[i, j])
            actor.SetUserMatrix(matrix)
            actor.GetProperty().SetColor(*materials[visual.find('material').get('name')])
            actor.GetProperty().SetSpecular(0.25)
            ren.AddActor(actor)
    plane = vtk.vtkPlaneSource()
    plane.SetOrigin(-0.65, -0.65, -0.001)
    plane.SetPoint1(0.65, -0.65, -0.001)
    plane.SetPoint2(-0.65, 0.65, -0.001)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(plane.GetOutputPort())
    floor = vtk.vtkActor()
    floor.SetMapper(mapper)
    floor.GetProperty().SetColor(0.31, 0.34, 0.37)
    ren.AddActor(floor)
    caption = vtk.vtkTextActor()
    caption.SetInput(title)
    caption.GetTextProperty().SetFontSize(24)
    caption.GetTextProperty().SetColor(0.94, 0.95, 0.97)
    caption.SetPosition(24, 26)
    ren.AddActor2D(caption)
    camera = ren.GetActiveCamera()
    camera.SetPosition(0.78, -0.95, 0.64)
    camera.SetFocalPoint(0, 0, 0.1)
    camera.SetViewUp(0, 0, 1)
    camera.ParallelProjectionOn()
    camera.SetParallelScale(0.39)
    ren.ResetCameraClippingRange()
    return ren


if __name__ == '__main__':
    window = vtk.vtkRenderWindow()
    window.SetSize(1600, 800)
    window.SetOffScreenRendering(1)
    window.AddRenderer(renderer(PACKAGE / 'urdf/j501_chassis_cad.urdf', 'Original CAD assembly pose', (0, 0, 0.5, 1)))
    window.AddRenderer(renderer(PACKAGE / 'urdf/j501_chassis.urdf', 'Simulation contact pose + virtual LiDAR', (0.5, 0, 1, 1)))
    window.Render()
    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.Update()
    target = PACKAGE / 'assets/chassis_preview.png'
    target.parent.mkdir(exist_ok=True)
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(target))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()
    print(target)
