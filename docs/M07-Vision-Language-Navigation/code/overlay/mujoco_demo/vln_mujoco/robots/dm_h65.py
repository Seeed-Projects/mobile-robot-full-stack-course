"""Genuine CAD / finite-torque dynamics baseline. No SDK or CAN dependency.

The exported assembly has two drive wheels and four compound caster assemblies.
Caster joints are inferred from CAD geometry, and remain a calibration baseline.
Use independent model/physics validation before any interactive integration.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from ..chassis_drive import DifferentialGeometry
from ..collision import CollisionMonitor
from ..model import compile_model, load_scene_xml
from .base import RobotState


def _numbers(values):
    return ' '.join(f'{float(x):.17g}' for x in values)


def load_config(path):
    path = Path(path).resolve()
    cfg = json.loads(path.read_text())
    if type(cfg.get('schema')) is not int or cfg['schema'] != 1:
        raise ValueError('DM-H65 config requires schema=1')
    geometry = DifferentialGeometry.from_config(cfg['sdk_chassis_config'])
    own = DifferentialGeometry(**cfg['geometry'])
    if geometry != own:
        raise ValueError('CAD asset geometry differs from authoritative SDK profile')
    manifest = json.loads((path.parent / cfg['asset_manifest']).read_text())
    for row in manifest['files']:
        file = (path.parent / row['file']).resolve()
        if file.parent != path.parent or not file.is_file():
            raise ValueError('Missing or escaping CAD mesh')
        if hashlib.sha256(file.read_bytes()).hexdigest() != row['sha256']:
            raise ValueError(f'CAD mesh hash mismatch: {file.name}')
    if len(cfg['links']) != 43 or sum(bool(l['wheel']) for l in cfg['links']) != 2:
        raise ValueError('Unexpected frozen CAD link/drive-wheel inventory')
    if not all((path.parent / m).is_file() for l in cfg['links'] for m in l['meshes']):
        raise ValueError('A CAD visual mesh is missing')
    return path, cfg, geometry


def build_model(path, *, scene=False, substeps=None):
    path, cfg, geometry = load_config(path)
    sim = cfg['simulation']
    count = substeps if substeps is not None else sim['substeps']
    if type(count) is not int or count not in (1, 5) or sim['outer_dt'] != .005:
        raise ValueError('Only 5ms outer steps, with 1 or 5 physics substeps, are supported')
    if scene:
        root = load_scene_xml()
    else:
        root = ET.fromstring('''<mujoco model="dm_h65_flat_validation">
          <compiler angle="radian"/><option gravity="0 0 -9.81"/>
          <visual><global offwidth="480" offheight="270"/></visual>
          <asset/><worldbody><light pos="0 0 4"/>
          <geom name="floor" type="plane" size="3 3 .1" rgba=".55 .58 .60 1" friction=".9 .005 .0001"/>
          </worldbody></mujoco>''')
    option = root.find('option')
    option.set('timestep', str(.005 / count))
    option.set('integrator', 'implicitfast')
    option.set('gravity', '0 0 -9.81')
    compiler = root.find('compiler')
    if compiler is None: compiler = ET.SubElement(root, 'compiler')
    compiler.set('angle', 'radian')
    asset = root.find('asset')
    if asset is None: asset = ET.SubElement(root, 'asset')
    est = cfg['estimates']
    for key in ('base_mass_kg', 'wheel_mass_kg', 'wheel_servo_kv', 'wheel_torque_limit_nm'):
        if not math.isfinite(est[key]) or est[key] <= 0:
            raise ValueError(f'Invalid estimated {key}')
    # A small enclosing tyre tolerance is explicit; control geometry remains the
    # frozen SDK/CAD nominal value, not a measured effective rolling radius.
    tyre_radius = geometry.wheel_radius_m + .0005
    root_height = tyre_radius + .0005
    xy = sim['spawn_xy'] if scene else [0., 0.]
    yaw = sim['initial_yaw']
    body = ET.SubElement(root.find('worldbody'), 'body', name='dm_h65_base',
                         pos=_numbers([*xy, root_height]),
                         quat=_numbers([math.cos(yaw/2), 0, 0, math.sin(yaw/2)]))
    ET.SubElement(body, 'freejoint', name='base_joint')
    ET.SubElement(body, 'inertial', pos=_numbers(est['base_com_m']),
                  mass=str(est['base_mass_kg']), diaginertia=_numbers(est['base_diaginertia_kg_m2']))
    allowed_floor = []
    for index, link in enumerate(cfg['links']):
        target = body
        if link['wheel']:
            side = 'left' if link['center'][1] > 0 else 'right'
            target = ET.SubElement(body, 'body', name=f'{side}_wheel', pos=_numbers(link['center']))
            ET.SubElement(target, 'joint', name=f'{side}_wheel_joint', type='hinge', axis='0 1 0',
                          damping=str(est['wheel_damping']), armature='.001')
            # Isolated wheel estimate: physically valid cylinder inertia.
            mass = est['wheel_mass_kg']; half = (link['hi'][1]-link['lo'][1])/2+.0001
            axial = .5 * mass * tyre_radius**2
            radial = mass * (3*tyre_radius**2 + (2*half)**2)/12
            ET.SubElement(target, 'inertial', pos='0 0 0', mass=str(mass),
                          diaginertia=_numbers([radial, axial, radial]))
            name = f'{side}_tyre_collision'; allowed_floor.append(name)
            ET.SubElement(target, 'geom', name=name, type='cylinder',
                          size=_numbers([tyre_radius, half]), euler=f'{-math.pi/2} 0 0',
                          friction=_numbers(est['wheel_friction']), rgba='.1 .1 .1 0',
                          group='3', mass='0', solref='.01 1')
        elif 'caster' in link:
            ci = link['caster']; caster = cfg['casters'][ci]
            pivot = np.asarray(caster['pivot_root_m']); axle = np.asarray(caster['axle_root_m'])
            target = ET.SubElement(body, 'body', name=f'caster_{ci}_swivel', pos=_numbers(pivot))
            ET.SubElement(target, 'joint', name=f'caster_{ci}_swivel_joint', type='hinge', axis='0 0 1',
                          damping=str(est['caster_swivel_damping']))
            ET.SubElement(target, 'inertial', pos='0 0 -.01', mass=str(est['caster_fork_mass_kg']),
                          diaginertia='.00005 .00005 .00005')
            lo=np.asarray(caster['fork_collision_lo_root_m']);hi=np.asarray(caster['fork_collision_hi_root_m'])
            ET.SubElement(target, 'geom', name=f'caster_{ci}_fork_collision', type='box',
                          pos=_numbers((lo+hi)/2-pivot), size=_numbers((hi-lo)/2+.001),
                          mass='0', group='3', rgba='.8 .2 .1 0', friction='.8 .005 .0001')
            wheel = ET.SubElement(target,'body',name=f'caster_{ci}_wheel',pos=_numbers(axle-pivot))
            ET.SubElement(wheel,'joint',name=f'caster_{ci}_roll_joint',type='hinge',axis='0 1 0',
                          damping=str(est['caster_roll_damping']))
            mass=est['caster_wheel_mass_kg']; r=caster['collision_radius_m'];half=caster['collision_half_width_m']
            axial=.5*mass*r*r;radial=mass*(3*r*r+4*half*half)/12
            ET.SubElement(wheel,'inertial',pos='0 0 0',mass=str(mass),diaginertia=_numbers([radial,axial,radial]))
            name=f'caster_{ci}_tyre_collision';allowed_floor.append(name)
            ET.SubElement(wheel,'geom',name=name,type='cylinder',size=_numbers([r,half]),
                          euler=f'{-math.pi/2} 0 0',mass='0',group='3',rgba='.1 .1 .1 0',
                          friction=_numbers(est['wheel_friction']),solref='.01 1')
        else:
            lo, hi = np.asarray(link['lo']), np.asarray(link['hi'])
            ET.SubElement(body, 'geom', name=f'cad_collision_{index:02d}', type='box',
                          pos=_numbers((lo+hi)/2), size=_numbers((hi-lo)/2+.001),
                          mass='0', group='3', rgba='.8 .2 .1 0', friction='.8 .005 .0001', solref='.01 1')
        for part, file in enumerate(link['meshes']):
            mesh_name = f'cad_mesh_{index:02d}_{part}'
            ET.SubElement(asset, 'mesh', name=mesh_name, file=str((path.parent/file).resolve()))
            color = '.08 .08 .08 1' if link['wheel'] else '.60 .64 .68 1'
            if link['link'] == 'part_1_6': color = '.10 .12 .16 1'
            extra = {'pos': _numbers(-np.asarray(cfg['casters'][link['caster']]['pivot_root_m']))} if 'caster' in link else {}
            ET.SubElement(target, 'geom', name=f'cad_visual_{index:02d}_{part}',
                          type='mesh', mesh=mesh_name, mass='0', contype='0', conaffinity='0',
                          group='1', rgba=color, **extra)
    cam = cfg['camera']
    if cam['width'] != 480 or cam['height'] != 270:
        raise ValueError('Camera resolution differs from the frozen baseline')
    ET.SubElement(body, 'camera', name='robot_rgb', mode='fixed',
                  pos=_numbers(cam['position_root_m']), xyaxes='0 -1 0 0 0 1', fovy=str(cam['fovy']))
    ET.SubElement(body, 'camera', name='robot_third_person', mode='fixed',
                  pos='-1.2 0 .85', xyaxes='0 -1 0 .45 0 .893', fovy='65')
    actuator = root.find('actuator')
    if actuator is None: actuator = ET.SubElement(root, 'actuator')
    for side in ('left', 'right'):
        ET.SubElement(actuator, 'velocity', name=f'{side}_wheel_drive', joint=f'{side}_wheel_joint',
                      kv=str(est['wheel_servo_kv']), ctrllimited='true',
                      ctrlrange=_numbers([-geometry.max_wheel_speed_rad_s, geometry.max_wheel_speed_rad_s]),
                      forcelimited='true', forcerange=_numbers([-est['wheel_torque_limit_nm'], est['wheel_torque_limit_nm']]))
    return compile_model(root), root, cfg, geometry, count, allowed_floor


class DMH65Backend:
    name = 'DM-H65 dynamics baseline / virtual camera / estimated parameters'
    camera_name = 'robot_rgb'
    # No configuration switch bypasses the pending dynamics/braking qualification.
    automatic_motion_approved = False
    automatic_motion_blocked_reason = 'DM-H65 G1/G2/G4 dynamics and braking qualification not completed'

    def __init__(self, config, *, scene=False, substeps=None):
        self.model, self.xml, self.config, self.geometry, self.substeps, allowed = build_model(
            config, scene=scene, substeps=substeps)
        self.data = mujoco.MjData(self.model)
        self._base_joint = self.model.joint('base_joint').id
        self._base_qpos = int(self.model.jnt_qposadr[self._base_joint])
        self._base_dof = int(self.model.jnt_dofadr[self._base_joint])
        self._base_body = int(self.model.jnt_bodyid[self._base_joint])
        self._wheel_dofs = [int(self.model.jnt_dofadr[self.model.joint(f'{s}_wheel_joint').id]) for s in ('left','right')]
        self._actuators = [self.model.actuator(f'{s}_wheel_drive').id for s in ('left','right')]
        self.allowed_ground = {self.model.geom(name).id for name in allowed}
        self.collision = CollisionMonitor(self.model, self.data, self._base_joint)
        self.fault = None
        self.last_raw_command = (0., 0.)
        self.last_applied_command = (0., 0.)
        self.wheel_targets = (0., 0.)
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.collision.reset()
        self.fault = None
        self.last_raw_command = self.last_applied_command = self.wheel_targets = (0., 0.)

    def _check_safety(self):
        if self.fault is not None: return
        if not (np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()):
            self.fault = {'reason':'NONFINITE_STATE', 'sim_time':float(self.data.time)}
            return
        for c in self.data.contact:
            if c.dist > 0: continue
            a,b = int(c.geom1),int(c.geom2)
            if bool(self.collision.robot_mask[a]) == bool(self.collision.robot_mask[b]): continue
            robot, env = (a,b) if self.collision.robot_mask[a] else (b,a)
            if self.model.geom(env).name == 'floor' and robot in self.allowed_ground: continue
            self.fault = {'reason':'FORBIDDEN_CONTACT', 'sim_time':float(self.data.time),
                          'geoms':[self.model.geom(robot).name,self.model.geom(env).name],
                          'distance_m':float(c.dist),'position':list(map(float,c.pos))}
            return
        tilt = math.acos(float(np.clip(self.data.xmat[self._base_body].reshape(3,3)[2,2],-1,1)))
        if tilt > .50:
            self.fault = {'reason':'EXCESSIVE_TILT','tilt_rad':tilt,'sim_time':float(self.data.time)}

    def step(self, command):
        if not all(math.isfinite(x) for x in command):
            self.data.ctrl[self._actuators] = 0
            raise ValueError('DM-H65 command must be finite')
        self.last_raw_command = tuple(command)
        self._check_safety()
        v,w = command
        lim = self.config['simulation']
        v = max(-lim['v_limit'],min(lim['v_limit'],v))
        w = max(-lim['w_limit'],min(lim['w_limit'],w))
        if self.fault is not None: v=w=0.
        wheels = self.geometry.from_twist(v,w)
        self.last_applied_command = (v,w)
        self.wheel_targets = (wheels.left_rad_s,wheels.right_rad_s)
        self.data.ctrl[self._actuators] = self.wheel_targets
        # Normal driving NEVER writes root pose or wheel velocity.
        for _ in range(self.substeps):
            mujoco.mj_step(self.model,self.data)
            mujoco.mj_forward(self.model,self.data)
            self._check_safety()
            if self.fault is not None:
                self.data.ctrl[self._actuators] = 0.

    def state(self):
        p = self.data.qpos[self._base_qpos:self._base_qpos+7]
        mat = self.data.xmat[self._base_body].reshape(3,3)
        yaw = math.atan2(mat[1,0],mat[0,0])
        linear = float(np.dot(self.data.qvel[self._base_dof:self._base_dof+3],mat[:,0]))
        omega_world = mat @ self.data.qvel[self._base_dof+3:self._base_dof+6]
        angular = float(omega_world[2])
        wheels = tuple(float(self.data.qvel[d]) for d in self._wheel_dofs)
        tilt = math.acos(float(np.clip(mat[2,2],-1,1)))
        return RobotState((*map(float,p[:3]),yaw),(linear,angular),telemetry={
            'finite':bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()),
            'fault':self.fault,'tilt_rad':tilt,
            'planar_speed_m_s':float(np.linalg.norm(self.data.qvel[self._base_dof:self._base_dof+2])),
            'angular_velocity_world_rad_s':list(map(float,omega_world)),'wheel_target_rad_s':self.wheel_targets,
            'wheel_measured_rad_s':wheels,'encoder_twist':self.geometry.encoder_twist(*wheels),
            'actuator_torque_nm':[float(self.data.actuator_force[a]) for a in self._actuators],
            'raw_command':self.last_raw_command,'applied_command':self.last_applied_command,
            'root_quaternion':list(map(float,p[3:7])), 'sources':self.config['sources'],
            'automatic_motion_approved':False,'physics_dt_s':float(self.model.opt.timestep),
            'outer_dt_s':.005,'substeps':self.substeps,'virtual_camera':True})

    def third_person_camera(self, _camera):
        return 'robot_third_person'
