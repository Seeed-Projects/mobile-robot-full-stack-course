"""Independent Simulation Demo Mode: kinematics, async real inference, no CAN.

The strict V2 controller is deliberately not instantiated or imported here.
"""
from __future__ import annotations
import asyncio
from collections import deque
from dataclasses import asdict
import hashlib
import io
import json
import math
from pathlib import Path
import queue
import shutil
import struct
import threading
import time
import traceback

from aiohttp import web, WSMsgType
import mujoco
import numpy as np
from PIL import Image

from .robots.dm_h65 import DMH65Backend
from .simulation import CameraFrame
from .dm_h65_history import OrderedHistory, classify
from .sim_demo_core import DT, V_MAX, W_MAX, PATH_TTL, FastCollision, Reference, RotationProgress, integrate, world_path, wrap

from .sim_demo_replan import repair_path, path_signature, MAX_ENDPOINT_SHIFT, PLAN_TIMEOUT

MAX_REPLAN_ATTEMPTS = 3

WEB = Path(__file__).parent/'sim_demo_web'


def dump(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2))


def geom_vertices(model, data, g):
    kind = int(model.geom_type[g]); size = model.geom_size[g]
    if kind == int(mujoco.mjtGeom.mjGEOM_MESH):
        mesh = int(model.geom_dataid[g]); start = int(model.mesh_vertadr[mesh])
        vertices = np.asarray(model.mesh_vert[start:start+int(model.mesh_vertnum[mesh])])
    else:
        if kind == int(mujoco.mjtGeom.mjGEOM_PLANE): return None
        if kind == int(mujoco.mjtGeom.mjGEOM_BOX): half = size
        elif kind == int(mujoco.mjtGeom.mjGEOM_CYLINDER): half = [size[0], size[0], size[1]]
        elif kind == int(mujoco.mjtGeom.mjGEOM_SPHERE): half = [size[0]]*3
        elif kind == int(mujoco.mjtGeom.mjGEOM_CAPSULE): half = [size[0], size[0], size[0]+size[1]]
        else: raise ValueError(f'Unsupported collision geom {g}: {kind}')
        vertices = np.asarray([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])*half
    return vertices @ data.geom_xmat[g].reshape(3,3).T + data.geom_xpos[g]


class Evidence:
    def __init__(self, root):
        self.root = root; self.queue = queue.Queue(maxsize=8192); self.error = ''
        self.thread = threading.Thread(target=self.run, name='demo-evidence', daemon=True)
        self.thread.start()

    def add(self, file, data):
        try: self.queue.put_nowait((file, data))
        except queue.Full: self.error = 'EVIDENCE_QUEUE_OVERFLOW'

    def run(self):
        streams = {}
        try:
            while True:
                item = self.queue.get()
                try:
                    if item is None: break
                    file, data = item
                    if isinstance(data, bytes): (self.root/file).write_bytes(data)
                    else:
                        if file not in streams: streams[file] = (self.root/file).open('a', buffering=1)
                        streams[file].write(json.dumps(data, ensure_ascii=False)+'\n')
                finally: self.queue.task_done()
        except Exception as error: self.error = f'EVIDENCE_WRITE_ERROR:{error}'
        finally:
            for stream in streams.values(): stream.close()

    def close(self):
        self.queue.put(None); self.thread.join(10)


class DemoSimulation:
    def __init__(self, config, output, initial, resolution=(960,540)):
        self.robot = DMH65Backend(config, scene=True)
        self.model, self.data = self.robot.model, self.robot.data
        self.model.vis.global_.offwidth = max(960, int(self.model.vis.global_.offwidth))
        self.model.vis.global_.offheight = max(540, int(self.model.vis.global_.offheight))
        self.render_data = mujoco.MjData(self.model)
        self.probe_data = mujoco.MjData(self.model)
        self.base = self.robot._base_qpos; self.dof = self.robot._base_dof
        self.wheel_qpos=[int(self.model.jnt_qposadr[self.model.joint(f'{side}_wheel_joint').id]) for side in ('left','right')]
        self.height = float(self.data.qpos[self.base+2])
        self.initial = np.asarray(initial); self.lock = threading.RLock()
        self._scene_epoch = 0; self.generation = 0
        self.reference = None; self.state = 'READY'; self.reason = ''; self.fault = None
        self.velocity = np.zeros(2); self.target = np.zeros(2); self.command_at = 0.
        self.active = False; self.frame_value = None; self.first = self.third = None
        self.rotation_progress = None
        self.resolution = resolution; self.quit = threading.Event(); self.physics_thread = self.render_thread = None
        self.history_fifo = None; self.history_error = ''; self.history_token = 0
        self.history_next = 0.; self.history_last = None
        self.frames = self.third_frames = self.ticks = 0
        self.start_time = time.monotonic(); self.last_frame_time = 0.; self.last_tick = 0.
        self.render_times = deque(maxlen=600); self.tick_times = deque(maxlen=2000)
        self.render_events = deque(maxlen=600); self.third_events = deque(maxlen=600); self.tick_events = deque(maxlen=2000)
        self.evidence = Evidence(output); self.output = output
        (output/'video-frames').mkdir()
        self.task_id = ''; self.task_started = 0.; self.task_ticks = self.moving_ticks = 0
        self.actual = deque(maxlen=10000); self.last_checked = {}; self.pending_command = None
        self.browser_metrics = {}; self.resource = {}; self.last_step_cost = 0.
        self.reset()
        self.geometry = self.build_geometry()
        dump(output/'geometry.json', {'motion_model':'simplified_differential_kinematics',
            'original_robot_collision_geoms':len(self.robot.collision.robot_ids),
            'caster_swivel_envelopes':4, 'footprint':self.geometry.footprint.tolist(),
            'obstacles':len(self.geometry.obstacles), 'margin_m':.020,
            'description':'Conservative 2.5D hull and static obstacle convex projections; not dynamics certification'})

    def reset(self):
        with self.lock:
            self.generation += 1; self._scene_epoch += 1
            mujoco.mj_resetData(self.model, self.data)
            self.pose = self.initial.copy(); self.write_pose(self.data, self.pose)
            self.velocity[:] = self.target[:] = 0.; self.reference = None
            self.active = False; self.state = 'READY'; self.reason = ''; self.fault = None
            self.actual.clear(); self.robot.collision.reset()
            self.frame_value = None; self.first = self.third = None
            self.history_fifo = None; self.history_error = ''

    def write_pose(self, data, pose):
        x,y,a = pose; data.qpos[self.base:self.base+3] = [x,y,self.height]
        data.qpos[self.base+3:self.base+7] = [math.cos(a/2),0,0,math.sin(a/2)]
        mujoco.mj_forward(self.model, data)

    def build_geometry(self):
        root = self.data.qpos[self.base:self.base+3].copy()
        # Construct at yaw zero so geometry is expressed in the root frame.
        self.write_pose(self.probe_data, [0,0,0])
        robot_vertices = []
        for g in self.robot.collision.robot_ids:
            robot_vertices.extend(geom_vertices(self.model,self.probe_data,int(g)))
        rv = np.asarray(robot_vertices); footprint = list(rv[:, :2])
        for c in self.robot.config['casters']:
            pivot = np.asarray(c['pivot_root_m']); lo=np.asarray(c['fork_collision_lo_root_m']); hi=np.asarray(c['fork_collision_hi_root_m'])
            radius = max(np.linalg.norm(np.array([x,y])-pivot[:2]) for x in [lo[0]-.001,hi[0]+.001] for y in [lo[1]-.001,hi[1]+.001])
            radius = max(radius, np.linalg.norm(np.asarray(c['axle_root_m'])[:2]-pivot[:2])+c['collision_radius_m']+c['collision_half_width_m'])
            # Square encloses every swivel angle, including the tyre and fork.
            footprint.extend([pivot[:2]+[sx*radius,sy*radius] for sx in [-1,1] for sy in [-1,1]])
        obstacles=[]
        for g in self.robot.collision.environment_ids:
            vertices=geom_vertices(self.model,self.probe_data,int(g))
            if vertices is None: raise ValueError(f'Unbounded obstacle {g}')
            if vertices[:,2].max() < rv[:,2].min()-1e-6 or vertices[:,2].min() > rv[:,2].max(): continue
            try:
                from .sim_demo_core import hull
                points=hull(vertices)
            except ValueError:
                lo=vertices[:,:2].min(0)-1e-5;hi=vertices[:,:2].max(0)+1e-5
                points=np.array([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]])
            obstacles.append(({'geom_id':int(g),'name':self.model.geom(int(g)).name},points))
        self.write_pose(self.probe_data,self.initial)
        return FastCollision(footprint,obstacles)

    def begin_history_capture(self,capacity=1024):
        with self.lock:
            self.history_token += 1; self.history_fifo=deque(maxlen=capacity)
            self.history_error='';self.history_next=time.monotonic();self.history_last=None
            return self.history_token

    def drain_history_capture(self, token):
        with self.lock:
            if token != self.history_token or self.history_fifo is None: raise RuntimeError('HISTORY_CANCELLED')
            if self.history_error: raise RuntimeError(self.history_error)
            frames=list(self.history_fifo);self.history_fifo.clear();return frames

    def end_history_capture(self,token):
        with self.lock:
            if token==self.history_token:self.history_fifo=None

    def begin_motion_guard(self):
        # Async demo expressly allows motion during inference. World-frame path
        # conversion plus freshness/remaining-route checks replace parking drift.
        pass

    def frame(self):
        with self.lock:return self.frame_value

    def cancel(self,reason,fault=False):
        with self.lock:
            self.generation+=1;self.reference=None;self.active=False
            self.target[:]=0.;self.velocity[:]=0.
            self.data.qvel[:]=0.;self.state='FAULT' if fault else 'STOPPED';self.reason=reason
            if fault:self.fault=reason
            self.evidence.add('events.jsonl',{'event':'STOP','reason':reason,'fault':fault,'time':time.monotonic(),'pose':self.pose.tolist(),'task_id':self.task_id})

    def start(self):
        self.start_time=time.monotonic();self.ticks=0
        self.physics_thread=threading.Thread(target=self.physics,name='demo-kinematics',daemon=True)
        self.render_thread=threading.Thread(target=self.render,name='demo-render',daemon=True)
        self.physics_thread.start();self.render_thread.start()

    def physics(self):
        next_tick=time.monotonic();next_control=0.;next_idle=0.
        try:
            while not self.quit.is_set():
                now=time.monotonic()
                if now<next_tick:self.quit.wait(min(next_tick-now,.002));continue
                started=time.monotonic()
                with self.lock:
                    if self.active and now-self.command_at>.35:self.cancel('CONTROL_WATCHDOG_350MS')
                    command=(0.,0.)
                    if self.active and self.reference:
                        if now>=next_control:
                            command,status=self.reference.command(self.pose,now)
                            self.target[:]=command;next_control=now+.02
                            self.state=status
                            if status in {'PATH_EXPIRED','TRACKING_DEVIATION'}:
                                self.reference=None;self.target[:]=self.velocity[:]=0.;self.reason=status
                        command=self.target
                    else:self.target[:]=0.
                    if self.active and self.rotation_progress and self.rotation_progress.observe(self.pose,now):
                        self.cancel('ROTATION_WITHOUT_PROGRESS')
                    # Rate limits smooth reference switches. Safety STOP bypasses smoothing.
                    self.velocity += np.clip(self.target-self.velocity,[-.30*DT,-1.0*DT],[.30*DT,1.0*DT])
                    candidate=integrate(self.pose,self.velocity)
                    bound=float(np.linalg.norm(candidate[:2]-self.pose[:2])+self.geometry.radius*abs(wrap(candidate[2]-self.pose[2])))
                    clearance,obstacle=self.geometry.clearance(candidate)
                    if bound>0 and clearance-bound<.020:
                        self.cancel('RUNTIME_COLLISION_GUARD',fault=True)
                        self.evidence.add('events.jsonl',{'event':'MOTION_BLOCKED','obstacle':obstacle,'clearance_m':clearance,'candidate':candidate.tolist(),'time':now})
                    else:
                        self.pose=candidate
                        self.write_pose(self.data,self.pose)
                        self.data.qvel[:]=0.
                        v,w=self.velocity
                        self.data.qvel[self.dof:self.dof+2]=[v*math.cos(self.pose[2]),v*math.sin(self.pose[2])]
                        self.data.qvel[self.dof+5]=w
                        wheels=self.robot.geometry.from_twist(v,w)
                        for q,d,value in zip(self.wheel_qpos,self.robot._wheel_dofs,[wheels.left_rad_s,wheels.right_rad_s]):
                            self.data.qpos[q]+=value*DT;self.data.qvel[d]=value
                        for i,c in enumerate(self.robot.config['casters']):
                            pivot=c['pivot_root_m'];vx=v-w*pivot[1];vy=w*pivot[0]
                            sj=self.model.joint(f'caster_{i}_swivel_joint');rj=self.model.joint(f'caster_{i}_roll_joint')
                            speed=math.hypot(vx,vy);qa=int(self.model.jnt_qposadr[sj.id])
                            if speed>1e-6:
                                old=float(self.data.qpos[qa]);delta=wrap(math.atan2(vy,vx)-old)
                                self.data.qpos[qa]+=max(-2*DT,min(2*DT,delta))
                            self.data.qpos[int(self.model.jnt_qposadr[rj.id])]+=speed/max(c['wheel_radius_m'],.001)*DT
                        self.data.time+=DT;mujoco.mj_forward(self.model,self.data)
                        contact=self.robot.collision.check()
                        if contact:self.cancel('ACTUAL_CONTACT',fault=True)
                    self.ticks+=1;self.last_tick=now;self.tick_events.append(now)
                    if self.active:
                        self.task_ticks+=1;self.moving_ticks+=int(abs(self.velocity[0])>.01 or abs(self.velocity[1])>.03)
                    if self.active or now>=next_idle:
                        self.evidence.add('state.jsonl',{'time':now,'sim_time':float(self.data.time),'task_id':self.task_id,
                            'active':self.active,'pose':self.pose.tolist(),'velocity':self.velocity.tolist(),
                            'command':self.target.tolist(),'phase':self.state,'path_id':self.reference.path_id if self.reference else None,
                            'progress_m':self.reference.progress if self.reference else None,'clearance_m':clearance,
                            'contact':self.robot.collision.first})
                        if self.ticks%20==0 or not self.active:self.actual.append(self.pose.tolist())
                        next_idle=now+1.
                self.last_step_cost=time.monotonic()-started;self.tick_times.append(self.last_step_cost)
                next_tick+=DT
                if now-next_tick>.10:
                    # No fast-forward or disguised replay when a wall-clock tick is missed.
                    next_tick=now;self.evidence.add('events.jsonl',{'event':'SIMULATION_LAG','time':now})
        except Exception as error:
            self.cancel(f'SIMULATION_ERROR:{error}',fault=True);self.evidence.add('events.jsonl',{'traceback':traceback.format_exc()})
            self.quit.set()

    def render(self):
        renderer=small=None
        try:
            width,height=self.resolution
            renderer=mujoco.Renderer(self.model,height=height,width=width)
            small=mujoco.Renderer(self.model,height=270,width=480)
            camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera)
            next_render=time.monotonic();next_third=next_render;sequence=0
            while not self.quit.is_set():
                now=time.monotonic()
                if now<next_render:self.quit.wait(min(next_render-now,.005));continue
                started=time.monotonic()
                with self.lock:
                    mujoco.mj_copyData(self.render_data,self.model,self.data)
                    pose=self.pose.copy();epoch=self._scene_epoch;task_id=self.task_id
                    sample=self.history_fifo is not None and now>=self.history_next
                    recording=self.active
                capture_ns=time.monotonic_ns();stamp_ns=time.time_ns()
                renderer.update_scene(self.render_data,camera='robot_rgb')
                rgb=np.asarray(renderer.render(),dtype=np.uint8).copy()
                out=io.BytesIO();Image.fromarray(rgb).save(out,format='JPEG',quality=82)
                first={'jpeg':out.getvalue(),'seq':sequence,'time':capture_ns/1e9,'pose':pose.tolist(),'epoch':epoch}
                third=None
                if now>=next_third:
                    camera.type=mujoco.mjtCamera.mjCAMERA_FREE
                    camera.lookat[:]=[pose[0],pose[1],self.height+.08]
                    camera.distance=1.65;camera.azimuth=math.degrees(pose[2])+180.;camera.elevation=-38.
                    small.update_scene(self.render_data,camera=camera)
                    third_rgb=np.asarray(small.render(),dtype=np.uint8).copy()
                    out=io.BytesIO();Image.fromarray(third_rgb).save(out,format='JPEG',quality=82)
                    third={'jpeg':out.getvalue(),'seq':sequence,'time':capture_ns/1e9,'epoch':epoch}
                    next_third=now+1/16.
                frame=None
                if sample:
                    model_rgb=Image.fromarray(rgb).resize((480,270),Image.Resampling.LANCZOS)
                    buf=io.BytesIO();model_rgb.save(buf,format='JPEG',quality=82)
                    cid=self.model.camera('robot_rgb').id
                    frame=CameraFrame(stamp_ns=stamp_ns,rgb=model_rgb.tobytes(),jpeg=buf.getvalue(),
                        pose=tuple(pose),monotonic_ns=capture_ns,
                        camera_position=tuple(map(float,self.render_data.cam_xpos[cid])),
                        camera_orientation=tuple(map(float,self.render_data.cam_xmat[cid])),
                        camera_fovy=float(self.model.cam_fovy[cid]),
                        root_position=(float(pose[0]),float(pose[1]),self.height))
                with self.lock:
                    if epoch!=self._scene_epoch:continue
                    cid=self.model.camera('robot_rgb').id
                    first.update(camera_position=self.render_data.cam_xpos[cid].tolist(),
                                 camera_orientation=self.render_data.cam_xmat[cid].tolist(),
                                 camera_fovy=float(self.model.cam_fovy[cid]))
                    self.first=first;self.last_frame_time=capture_ns/1e9;self.frames+=1;self.render_events.append(time.monotonic())
                    if third:self.third=third;self.third_frames+=1;self.third_events.append(time.monotonic())
                    if frame and self.history_fifo is not None:
                        if self.history_last and frame.monotonic_ns-self.history_last>.40e9:self.history_error='MISSED_REAL_4HZ_CAPTURE'
                        if len(self.history_fifo)==self.history_fifo.maxlen:self.history_error='HISTORY_FIFO_OVERFLOW'
                        else:self.history_fifo.append(frame)
                        self.frame_value=frame;self.history_last=frame.monotonic_ns
                        self.history_next+=.25
                sequence+=1;self.render_times.append(time.monotonic()-started)
                if recording:
                    first_file=f'video-frames/{task_id}-first-{sequence:06d}.jpg'
                    self.evidence.add(first_file,first['jpeg'])
                    self.evidence.add('video-timestamps.jsonl',{'view':'first','file':first_file,'time':capture_ns/1e9,'task_id':task_id,'seq':sequence})
                    if third:
                        third_file=f'video-frames/{task_id}-third-{sequence:06d}.jpg'
                        self.evidence.add(third_file,third['jpeg'])
                        self.evidence.add('video-timestamps.jsonl',{'view':'third','file':third_file,'time':capture_ns/1e9,'task_id':task_id,'seq':sequence})
                next_render=now+1/32.
        except Exception as error:
            self.cancel(f'RENDER_ERROR:{error}',fault=True);self.evidence.add('events.jsonl',{'traceback':traceback.format_exc()})
            self.quit.set()
        finally:
            if renderer:renderer.close()
            if small:small.close()

    def stats(self):
        def rate(events):return (len(events)-1)/(events[-1]-events[0]) if len(events)>1 and events[-1]>events[0] else 0.
        with self.lock:
            return {'pose':self.pose.tolist(),'velocity':self.velocity.tolist(),'phase':self.state,'reason':self.reason,
                'fault':self.fault,'active':self.active,'generation':self.generation,'epoch':self._scene_epoch,
                'motion_model':'simplified_differential_kinematics','virtual_camera_height_m':1.,
                'simulation_hz':rate(self.tick_events),'simulation_rtf':self.ticks*DT/max(time.monotonic()-self.start_time,.001),
                'camera_fps':rate(self.render_events),'third_person_fps':rate(self.third_events),
                'render_ms_p95':float(np.percentile(list(self.render_times),95)*1000) if self.render_times else None,
                'step_ms_p95':float(np.percentile(list(self.tick_times),95)*1000) if self.tick_times else None,
                'physics_alive':bool(self.physics_thread and self.physics_thread.is_alive()),
                'render_alive':bool(self.render_thread and self.render_thread.is_alive()),
                'frame_age_s':time.monotonic()-self.last_frame_time if self.last_frame_time else None,
                'motion_duty_cycle':self.moving_ticks/max(1,self.task_ticks),
                'path_id':self.reference.path_id if self.reference else None,
                'path_age_s':time.monotonic()-self.reference.capture_time if self.reference else None,
                'path_progress_m':self.reference.progress if self.reference else None,
                'path_length_m':float(self.reference.cumulative[-1]) if self.reference else None,
                'path_world':self.reference.points.tolist() if self.reference else [],
                'actual_trace':list(self.actual)[-1500:],'resource':self.resource,'writer_error':self.evidence.error,
                'browser':self.browser_metrics,'last_geometry_check':self.last_checked}

    def close(self):
        self.cancel('SERVER_STOP');self.quit.set()
        for thread in [self.physics_thread,self.render_thread]:
            if thread:thread.join(10)
        self.evidence.close()


class DemoApp:
    def __init__(self,config,output,task,resolution=(960,540)):
        self.output=output;self.task_config=task
        initial=task['initial_pose'];self.sim=DemoSimulation(config,output,[initial['x'],initial['y'],initial['yaw']],resolution)
        self.owner=None;self.sockets=set();self.navigation=None;self.task=None;self.history=None
        self.prediction=None;self.prediction_jpeg=None;self.model_ready=False;self.request_count=0
        self.latencies=[];self.prediction_events=[];self.next_task=0;self.watch_task=None
        self.control_task=None
        self.telemetry_error=None
        self.browser_heartbeat=0.;self.closed=False
        # Target location is held only by the independent evaluator, never tracker/history.
        self.score_target=np.asarray(task['target_position'][:2]) if 'target_position' in task else np.asarray(task['target_xyz'][:2]) if 'target_xyz' in task else None
        if self.score_target is None:
            target=self.sim.model.body(task['target_body']).id
            self.score_target=self.sim.data.xpos[target,:2].copy()

    def health(self):
        state=self.sim.stats()
        ready=bool(state['physics_alive'] and state['render_alive'] and state['frame_age_s'] is not None and state['frame_age_s']<1. and not state['fault'] and not state['writer_error'])
        return {'ok':ready,'mode':'sim-demo','model_ready':self.model_ready,'simulation':state,
            'navigation':self.navigation,'prediction':self.prediction,
            'telemetry_error':self.telemetry_error,
            'telemetry_alive':bool(self.watch_task and not self.watch_task.done()),
            'task_context':{'registered_instruction':self.task_config['instruction'],
                'strict_scoring_enabled':not self.navigation or self.navigation['instruction']==self.task_config['instruction']},
            'inference_hz':((len(self.prediction_events)-1)/(self.prediction_events[-1]-self.prediction_events[0]) if len(self.prediction_events)>1 else 0.),
            'inference_latency_ms':self.latencies[-1]*1000 if self.latencies else None,
            'limits':{'v_max':V_MAX,'w_max':W_MAX,'path_ttl_s':PATH_TTL,'task_s':300,'predictions':40,
                      'max_replan_attempts':MAX_REPLAN_ATTEMPTS,
                      'rotation_progress_distance_m':.15,'rotation_window_s':30.,'rotation_budget_rad':2*math.pi,
                      'local_plan_timeout_s':PLAN_TIMEOUT,'max_endpoint_shift_m':MAX_ENDPOINT_SHIFT},
            'strict_v2_modified':False,'no_CAN':True}

    def pointing_anchors(self,frame,data):
        """Display only: ray depth comes from MuJoCo, not from the model."""
        matrix=np.asarray(frame.camera_orientation).reshape(3,3)
        origin=np.asarray(frame.camera_position);fy=270/(2*math.tan(math.radians(frame.camera_fovy)/2))
        result={}
        # Separate mjData: never mutate the running simulator to recover a frame.
        probe=self.sim.probe_data
        self.sim.write_pose(probe,frame.pose)
        for key in ['apos_px','opos_px']:
            pixel=(data.get('pointing') or {}).get(key)
            if pixel is None:continue
            direction=matrix @ np.array([(pixel[0]-240)/fy,-(pixel[1]-135)/fy,-1.])
            direction/=np.linalg.norm(direction);geom=np.array([-1],dtype=np.int32)
            depth=mujoco.mj_ray(self.sim.model,probe,origin,direction,None,1,self.sim.robot._base_body,geom)
            if depth>=0 and geom[0]>=0 and not self.sim.robot.collision.robot_mask[geom[0]]:
                result[key]={'world_point':(origin+depth*direction).tolist(),'geom_id':int(geom[0]),
                    'depth_source':'MuJoCo scene ray intersection; model output is 2D pixel only'}
        return result

    def stop(self,reason):
        self.sim.cancel(reason)
        if self.task and not self.task.done():self.task.cancel()

    async def start_task(self,socket,instruction):
        if self.task and not self.task.done():raise ValueError('TASK_ALREADY_RUNNING')
        if not instruction.strip() or len(instruction)>1000:raise ValueError('INVALID_INSTRUCTION')
        h=self.health()
        if not h['ok'] or not self.model_ready:raise ValueError('DEMO_NOT_READY')
        if self.sim.resource.get('memory_bytes',0)<3*1024**3 or self.sim.resource.get('disk_bytes',0)<3.5*1024**3:raise ValueError('RESOURCE_START_GUARD')
        if self.owner is not None and self.owner is not socket:raise ValueError('CONTROL_ALREADY_OWNED')
        self.owner=socket;self.browser_heartbeat=time.monotonic()
        self.task=asyncio.create_task(self.navigate(instruction))

    async def navigate(self,instruction):
        self.next_task+=1;out=self.output/f'navigation-{self.next_task:04d}';out.mkdir()
        history_dir=out/'history';history_dir.mkdir()
        self.sim.task_id=out.name;self.sim.task_ticks=self.sim.moving_ticks=0
        self.sim.task_started=time.monotonic();started=self.sim.task_started
        self.sim.rotation_progress=RotationProgress(self.sim.pose,started)
        generation=self.sim.generation;initial=self.sim.pose.copy();distances=[];errors='';strict=False
        total_replan_attempts=0;consecutive_replan_attempts=0;last_replan_reason='';replan_limit_reached=False
        local_repair_count=0;failed_repairs={}
        self.navigation={'task_id':out.name,'instruction':instruction,'phase':'OBSERVING','predictions':0,
                         'started':started,'remaining_s':300.,'outcome':None,
                         'replan_attempts':0,'consecutive_replan_attempts':0,
                         'last_replan_reason':None,'retained_path_id':None,
                         'replan_limit_reached':False,'local_repair_count':0,'repeated_path_count':0}
        self.latencies=[];self.prediction_events=[]
        dump(out/'registration.json',{'instruction':instruction,'initial_pose':initial.tolist(),'motion_model':'simplified_differential_kinematics',
            'task_budget_s':300,'predictions_max':40,'v_max':V_MAX,'w_max':W_MAX,'path_ttl_s':PATH_TTL,
            'scoring_only_target':self.score_target.tolist(),'strict_scoring_enabled':instruction==self.task_config['instruction'],
            'max_replan_attempts':MAX_REPLAN_ATTEMPTS})
        def check():
            if generation!=self.sim.generation:raise RuntimeError(self.sim.reason or 'CANCELLED_OR_STALE')
            if time.monotonic()-started>=300:raise RuntimeError('TASK_BUDGET_EXHAUSTED')
            if self.sim.fault or self.sim.evidence.error:raise RuntimeError(self.sim.fault or self.sim.evidence.error)
        self.history=OrderedHistory(self.sim,history_dir,instruction,check)
        try:
            self.sim.active=True;self.sim.command_at=time.monotonic()
            await self.history.start()
            while self.navigation['predictions']<40:
                check();self.navigation['phase']='INFERENCE'
                answer=await self.history.predict();check()
                self.navigation['predictions']+=1;index=self.navigation['predictions']
                reply=answer['reply'];data=reply['data'];frame=answer['frame']
                stage=out/f'prediction-{index:02d}';stage.mkdir()
                await asyncio.to_thread(dump,stage/'raw.json',{'response':reply,'elapsed_s':answer['elapsed_s'],
                    'capture':{k:v for k,v in asdict(frame).items() if k not in {'rgb','jpeg'}},
                    'jpeg_sha256':hashlib.sha256(frame.jpeg).hexdigest(),'sequence':answer['sequence']})
                await asyncio.to_thread((stage/'input.jpg').write_bytes,frame.jpeg)
                self.prediction={'data':data,'capture_pose':frame.pose,'capture_time':frame.monotonic_ns/1e9,
                    'capture_stamp_ns':str(frame.stamp_ns),'index':index,'task_id':out.name,'classification':classify(data)}
                self.prediction['camera']={'camera_position':frame.camera_position,'camera_orientation':frame.camera_orientation,'camera_fovy':frame.camera_fovy}
                self.prediction_jpeg=frame.jpeg;self.latencies.append(answer['elapsed_s']);self.prediction_events.append(time.monotonic())
                self.prediction['display_anchors']=await asyncio.to_thread(self.pointing_anchors,frame,data)
                await asyncio.to_thread(dump,stage/'display-anchors.json',self.prediction['display_anchors'])
                if data.get('rc')!=0:raise RuntimeError(classify(data))
                age=time.monotonic()-frame.monotonic_ns/1e9
                if age>=PATH_TTL:raise RuntimeError('PREDICTION_TOO_OLD')
                if data.get('stop') is True:
                    with self.sim.lock:
                        check()
                        self.sim.reference=None;self.sim.target[:]=self.sim.velocity[:]=0.;self.sim.state='MODEL_STOP'
                    self.navigation['phase']='MODEL_STOP'
                    # STOP halts even when old path is valid. Score does not drive motion.
                    stop_started=time.monotonic()
                    while time.monotonic()-stop_started<2.:
                        check();dist=float(np.linalg.norm(self.sim.pose[:2]-self.score_target));distances.append(dist)
                        if dist>.75 or abs(self.sim.velocity[0])>.05 or abs(self.sim.velocity[1])>.1:break
                        await asyncio.sleep(.02)
                    strict=bool(time.monotonic()-stop_started>=2. and instruction==self.task_config['instruction'])
                    errors=('STRICT_SUCCESS' if strict else 'MODEL_STOP_OUTSIDE_SUCCESS_CONDITIONS') if instruction==self.task_config['instruction'] else 'MODEL_STOP_UNSCORED'
                    break
                local=np.asarray(data.get('actions',{}).get('actions'),dtype=float)
                points=world_path(local,frame.pose)
                self.prediction['world_path']=points.tolist()
                self.navigation['phase']='PATH_PRECHECK'
                result=await asyncio.to_thread(self.sim.geometry.check,points)
                self.prediction['path_check']=result
                await asyncio.to_thread(dump,stage/'full-path-check.json',{'world_path':points.tolist(),'check':result})
                check()
                if not result['safe'] and result.get('reason') != 'PATH_COLLISION_RISK':
                    raise RuntimeError(result['reason'])
                # Pause only for path handoff/local planning. During inference the
                # old certified Reference still runs. Clearing it under the physics
                # lock makes certification and installation refer to one pose.
                with self.sim.lock:
                    check()
                    parked_reference=self.sim.reference
                    self.sim.reference=None;self.sim.target[:]=self.sim.velocity[:]=0.
                    self.sim.state='PATH_PRECHECK'
                    current=self.sim.pose.copy()
                reference=Reference(points,frame.monotonic_ns/1e9,f'{out.name}:{index}')
                remaining=reference.remaining(current)
                mismatch=reference.locate(current)[0]>.20
                if mismatch:
                    # The observation is no longer on this route. In particular,
                    # a short turn predicted before a long inference must not be
                    # repaired into a trip BACK to its old capture position.
                    # Stay parked; the next request uses a new real camera frame.
                    total_replan_attempts+=1;consecutive_replan_attempts+=1
                    last_replan_reason='NEW_PATH_CAPTURE_MISMATCH'
                    replan_limit_reached=consecutive_replan_attempts>=MAX_REPLAN_ATTEMPTS
                    self.sim.reason=last_replan_reason;self.sim.state='REOBSERVING'
                    self.navigation.update(phase='REOBSERVING',replan_attempts=total_replan_attempts,
                        consecutive_replan_attempts=consecutive_replan_attempts,
                        last_replan_reason=last_replan_reason,retained_path_id=None,
                        replan_limit_reached=replan_limit_reached)
                    await asyncio.to_thread(dump,stage/'reobserve.json',{
                        'reason':last_replan_reason,'capture_pose':list(frame.pose),
                        'current_pose':current.tolist(),'attempt':consecutive_replan_attempts,
                        'action':'park_and_capture_fresh_observation'})
                    if replan_limit_reached:
                        errors=last_replan_reason;break
                    continue
                connector=None
                if result['safe'] and not mismatch:
                    connector=await asyncio.to_thread(self.sim.geometry.check,remaining)
                    await asyncio.to_thread(dump,stage/'remaining-check.json',connector)
                    check()
                    if not connector['safe'] and connector.get('reason') != 'PATH_COLLISION_RISK':
                        raise RuntimeError(connector['reason'])
                repair_needed=not result['safe'] or mismatch or (connector is not None and not connector['safe'])
                source='model'
                if repair_needed:
                    self.navigation['phase']='LOCAL_PLANNING';self.sim.state='LOCAL_PLANNING'
                    signature=path_signature(remaining)
                    if signature in failed_repairs:
                        local_repair={**failed_repairs[signature],'repeated_path':True}
                        self.navigation['repeated_path_count']+=1
                    else:
                        local_repair=await asyncio.to_thread(repair_path,self.sim.geometry,remaining,
                            lambda:generation!=self.sim.generation or bool(self.sim.fault or self.sim.evidence.error))
                        if local_repair['reason']=='NO_SAFE_LOCAL_PATH':
                            failed_repairs[signature]=local_repair
                    local_repair['signature']=signature
                    local_repair['trigger']='NEW_PATH_CAPTURE_MISMATCH' if mismatch else 'PATH_COLLISION_RISK'
                    self.prediction['local_repair']={k:v for k,v in local_repair.items() if k not in {'attempts','tracker_check'}}
                    self.prediction['local_repair']['tracker_check']={k:v for k,v in local_repair.get('tracker_check',{}).items() if k!='trace'}
                    await asyncio.to_thread(dump,stage/'local-replan.json',local_repair)
                    check()
                    if local_repair['safe']:
                        # Freshness remains tied to the original camera capture.
                        reference=Reference(np.asarray(local_repair['world_path']),frame.monotonic_ns/1e9,
                                            f'{out.name}:{index}:local')
                        result=local_repair['check'];source='local_repair'
                        local_repair_count+=1;self.navigation['local_repair_count']=local_repair_count
                    else:
                        if local_repair['reason']=='CANCELLED_OR_STALE':raise RuntimeError(local_repair['reason'])
                        # A failed repair never authorizes movement. Restore an old
                        # reference only after checking its complete remaining route.
                        retained_path_id=None
                        if parked_reference is not None and time.monotonic()-parked_reference.capture_time<PATH_TTL:
                            old_remaining=parked_reference.remaining(current)
                            old_check=await asyncio.to_thread(self.sim.geometry.check,old_remaining)
                            check()
                            with self.sim.lock:
                                check()
                                if old_check['safe'] and time.monotonic()-parked_reference.capture_time<PATH_TTL:
                                    self.sim.reference=parked_reference;retained_path_id=parked_reference.path_id
                        total_replan_attempts+=1;consecutive_replan_attempts+=1
                        last_replan_reason='PATH_COLLISION_RISK'
                        self.sim.reason=last_replan_reason;self.sim.state='REPLANNING'
                        replan_limit_reached=consecutive_replan_attempts>=MAX_REPLAN_ATTEMPTS
                        self.navigation.update(phase='REPLANNING',replan_attempts=total_replan_attempts,
                            consecutive_replan_attempts=consecutive_replan_attempts,last_replan_reason=last_replan_reason,
                            retained_path_id=retained_path_id,replan_limit_reached=replan_limit_reached)
                        await asyncio.to_thread(dump,stage/'replan.json',{
                            'reason':last_replan_reason,'local_reason':local_repair['reason'],
                            'attempt':consecutive_replan_attempts,'total_attempts':total_replan_attempts,
                            'retained_path_id':retained_path_id,'retained_path_valid':retained_path_id is not None})
                        if replan_limit_reached:
                            errors=last_replan_reason;break
                        await asyncio.sleep(.05)
                        continue
                with self.sim.lock:
                    check()
                    if time.monotonic()-reference.capture_time>=PATH_TTL:raise RuntimeError('PATH_EXPIRED_BEFORE_INSTALL')
                    if np.linalg.norm(current[:2]-self.sim.pose[:2])>.03 or abs(wrap(current[2]-self.sim.pose[2]))>.05:raise RuntimeError('PRECHECK_STATE_CHANGED')
                    self.sim.reference=reference;self.sim.last_checked=result;self.sim.reason='';self.sim.state='TRACKING'
                self.prediction['installed_path']=reference.points.tolist()
                self.prediction['path_source']=source
                consecutive_replan_attempts = 0
                self.navigation.update(phase='TRACKING_WITH_ASYNC_INFERENCE',
                                       consecutive_replan_attempts=0,
                                       last_replan_reason=last_replan_reason,
                                       retained_path_id=reference.path_id,
                                       replan_limit_reached=False)
                self.sim.evidence.add('events.jsonl',{'event':'PATH_INSTALLED','time':time.monotonic(),'path_id':reference.path_id,
                    'capture_time':reference.capture_time,'progress_m':reference.progress,
                    'source':source,'world_path':reference.points.tolist(),'check':result})
                distances.append(float(np.linalg.norm(self.sim.pose[:2]-self.score_target)))
                # Exactly one serial model request. Previous safe path keeps tracking.
                await asyncio.sleep(.05)
            else:errors='PREDICTION_BUDGET_EXHAUSTED'
        except asyncio.CancelledError:errors=self.sim.reason or 'USER_CANCELLED'
        except Exception as error:errors=str(error)
        finally:
            # Clear motion immediately on any terminal result, before disk/network cleanup.
            final=self.sim.pose.copy();before=self.sim.stats()
            self.sim.cancel(errors or 'TASK_FINISHED')
            if self.history:
                await self.history.close();self.history=None
            outcome={'reason':errors,'strict_success':strict,'predictions':self.navigation['predictions'],
                'replan_attempts':total_replan_attempts,'last_replan_reason':last_replan_reason,
                'replan_limit_reached':replan_limit_reached,'local_repair_count':local_repair_count,
                'elapsed_s':time.monotonic()-started,'initial_pose':initial.tolist(),'final_pose':final.tolist(),
                'initial_distance_m':float(np.linalg.norm(initial[:2]-self.score_target)),
                'final_distance_m':float(np.linalg.norm(final[:2]-self.score_target)),
                'motion_duty_cycle':before['motion_duty_cycle'],'statistics':before,'latencies_s':self.latencies.copy()}
            await asyncio.to_thread(dump,out/'outcome.json',outcome)
            self.navigation.update(phase='FINISHED',outcome=errors,remaining_s=0.,strict_success=strict,
                                   replan_attempts=total_replan_attempts,
                                   replan_limit_reached=replan_limit_reached)

    async def control_lease(self):
        """Renew control independently of potentially slow telemetry clients."""
        while not self.closed:
            now=time.monotonic()
            if self.sim.active:
                if self.owner is None or self.owner.closed or now-self.browser_heartbeat>2.:
                    self.stop('BROWSER_DISCONNECTED_OR_HEARTBEAT_EXPIRED')
                else:self.sim.command_at=now
                if self.navigation:
                    self.navigation['remaining_s']=max(0,300-(now-self.navigation['started']))
                    if self.navigation['remaining_s']<=0:self.stop('TASK_BUDGET_EXHAUSTED')
            await asyncio.sleep(.05)

    async def broadcast_state(self):
        message={'type':'state',**self.health()}
        async def publish(socket):
            if socket.closed:return
            try:await asyncio.wait_for(socket.send_json(message),.3)
            except Exception:
                try:await asyncio.wait_for(socket.close(),.3)
                except Exception:pass
        await asyncio.gather(*(publish(socket) for socket in list(self.sockets)))

    async def watch(self):
        while not self.closed:
            try:
                self.sim.resource={'disk_bytes':shutil.disk_usage(self.output).free,
                    'memory_bytes':int(next(s.split()[1] for s in Path('/proc/meminfo').read_text().splitlines() if s.startswith('MemAvailable:')))*1024}
                if min(self.sim.resource.values())<3*1024**3 and self.sim.active:self.stop('RESOURCE_GUARD')
                await self.broadcast_state()
                self.telemetry_error=None
            except Exception as error:
                self.telemetry_error=str(error)
                self.sim.evidence.add('events.jsonl',{'event':'TELEMETRY_ERROR','reason':str(error),'traceback':traceback.format_exc()})
            await asyncio.sleep(.10)

    async def socket(self,request):
        socket=web.WebSocketResponse(heartbeat=5,max_msg_size=1024**2);await socket.prepare(request);self.sockets.add(socket)
        try:
            async for message in socket:
                if message.type!=WSMsgType.TEXT:continue
                try:
                    data=json.loads(message.data);kind=data.get('type')
                    if kind=='heartbeat':
                        if self.owner is socket:self.browser_heartbeat=time.monotonic()
                    elif kind=='start_sim_navigation':await self.start_task(socket,str(data.get('instruction','')))
                    elif kind=='stop':self.stop('USER_STOP')
                    elif kind=='reset':
                        self.stop('USER_RESET')
                        if self.task:await self.task
                        self.sim.reset();self.prediction=None;self.prediction_jpeg=None;self.navigation=None
                    elif kind=='browser_metrics':self.sim.browser_metrics=data.get('metrics',{})
                    else:raise ValueError('UNSUPPORTED_DEMO_COMMAND')
                except Exception as error:await socket.send_json({'type':'error','message':str(error)})
        finally:
            self.sockets.discard(socket)
            if self.owner is socket:self.stop('CONTROL_DISCONNECTED');self.owner=None
        return socket

    async def frames(self,request):
        view=request.match_info['view'];resp=web.StreamResponse(headers={'Content-Type':'multipart/x-mixed-replace; boundary=frame','Cache-Control':'no-store'})
        await resp.prepare(request);last=-1
        try:
            while not self.closed:
                frame=self.sim.first if view=='first' else self.sim.third
                if frame and frame['seq']!=last:
                    payload=frame['jpeg'];meta={k:v for k,v in frame.items() if k!='jpeg'}
                    header=f'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(payload)}\r\nX-Frame-Meta: {json.dumps(meta)}\r\n\r\n'.encode()
                    await asyncio.wait_for(resp.write(header+payload+b'\r\n'),1.);last=frame['seq']
                await asyncio.sleep(.005)
        except (ConnectionError,asyncio.TimeoutError,asyncio.CancelledError):pass
        return resp

    async def video(self,request):
        """H.264 display transport; real model input remains the original JPEG.

        Each packet carries MP4 bytes and exact source-frame extrinsics keyed by
        encoder PTS. No navigation state is interpolated by the video transport.
        """
        if not shutil.which('ffmpeg'):raise web.HTTPServiceUnavailable(text='H264_ENCODER_UNAVAILABLE')
        view=request.match_info['view'];fps=30 if view=='first' else 15
        process=await asyncio.create_subprocess_exec('ffmpeg','-hide_banner','-loglevel','error',
            '-f','mjpeg','-framerate',str(fps),'-i','pipe:0','-an','-c:v','libx264','-preset','ultrafast',
            '-tune','zerolatency','-threads','2','-pix_fmt','yuv420p','-profile:v','baseline','-level','3.1',
            '-bf','0','-g',str(fps),'-crf','24','-movflags','frag_keyframe+empty_moov+default_base_moof',
            '-frag_duration','100000','-flush_packets','1','-f','mp4','pipe:1',
            stdin=asyncio.subprocess.PIPE,stdout=asyncio.subprocess.PIPE,stderr=asyncio.subprocess.DEVNULL)
        resp=web.StreamResponse(headers={'Content-Type':'application/octet-stream','Cache-Control':'no-store'})
        await resp.prepare(request);metadata=[]
        async def writer():
            last=-1;index=0;next_frame=time.monotonic()
            while not self.closed:
                now=time.monotonic()
                if now<next_frame:
                    await asyncio.sleep(min(next_frame-now,.005));continue
                frame=self.sim.first if view=='first' else self.sim.third
                if frame:
                    # Keep the codec clock constant, without inventing new camera
                    # captures. Duplicate transport frames retain the original seq
                    # and extrinsics and do not count as unique presented frames.
                    metadata.append({'pts':index/fps,'transport_duplicate':frame['seq']==last,
                                     **{k:v for k,v in frame.items() if k!='jpeg'}})
                    process.stdin.write(frame['jpeg']);await process.stdin.drain()
                    last=frame['seq'];index+=1
                next_frame+=1/fps
                if time.monotonic()-next_frame>.10:next_frame=time.monotonic()
        writing=asyncio.create_task(writer())
        try:
            while not self.closed:
                payload=await process.stdout.read(32768)
                if not payload:break
                meta=json.dumps(metadata).encode();metadata.clear()
                await asyncio.wait_for(resp.write(struct.pack('!II',len(meta),len(payload))+meta+payload),2.)
        except (ConnectionError,asyncio.TimeoutError,asyncio.CancelledError):pass
        finally:
            writing.cancel()
            try:await writing
            except (asyncio.CancelledError,ConnectionError,BrokenPipeError):pass
            if process.returncode is None:
                process.terminate()
                try:await asyncio.wait_for(process.wait(),2)
                except asyncio.TimeoutError:process.kill();await process.wait()
        return resp

    async def jpeg(self,request):
        view=request.match_info['view'];frame=self.sim.first if view=='first' else self.sim.third
        if not frame:raise web.HTTPServiceUnavailable()
        return web.Response(body=frame['jpeg'],content_type='image/jpeg',headers={'Cache-Control':'no-store','X-Frame-Meta':json.dumps({k:v for k,v in frame.items() if k!='jpeg'})})

    async def startup(self,app):
        self.sim.start()
        self.watch_task=asyncio.create_task(self.watch())
        self.control_task=asyncio.create_task(self.control_lease())
        from aiohttp import ClientSession
        try:
            async with ClientSession() as session:
                async with session.ws_connect('ws://127.0.0.1:8050',timeout=3) as ws:
                    await ws.send_json({'action':'login','data':{'clientId':'sim-demo-health-'+str(time.time_ns())}})
                    reply=await asyncio.wait_for(ws.receive_json(),3)
                    self.model_ready=reply.get('data',{}).get('rc')==0
        except Exception:self.model_ready=False

    async def shutdown(self,app):
        self.closed=True;self.stop('SERVER_STOP')
        if self.task:await self.task
        if self.watch_task:self.watch_task.cancel()
        if self.control_task:
            self.control_task.cancel()
            try:await self.control_task
            except asyncio.CancelledError:pass
        await asyncio.to_thread(self.sim.close)

    async def stopping(self,app):
        self.closed=True;self.stop('SERVER_STOP')
        for socket in list(self.sockets):await socket.close()

    def app(self):
        app=web.Application()
        app.router.add_get('/',lambda r:web.FileResponse(WEB/'index.html'))
        app.router.add_get('/api/health',lambda r:web.json_response(self.health()))
        app.router.add_get('/api/prediction.jpg',lambda r:web.Response(body=self.prediction_jpeg or b'',content_type='image/jpeg',headers={'Cache-Control':'no-store'}))
        app.router.add_get('/api/map',lambda r:web.json_response({'polygons':[p.tolist() for _,p,_ in self.sim.geometry.obstacles]}))
        app.router.add_get('/api/{view:first|third}.jpg',self.jpeg)
        app.router.add_get('/stream/{view:first|third}',self.frames)
        app.router.add_get('/video/{view:first|third}',self.video)
        app.router.add_get('/ws',self.socket);app.router.add_static('/static',WEB)
        app.on_startup.append(self.startup);app.on_shutdown.append(self.stopping);app.on_cleanup.append(self.shutdown)
        return app
