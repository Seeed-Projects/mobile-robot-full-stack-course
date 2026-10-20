"""CPU-only fault tests with explicitly fake model replies, never demo evidence."""
import asyncio
from pathlib import Path
import threading
from types import SimpleNamespace
import json
import numpy as np
import pytest

from vln_mujoco.simulation import CameraFrame
import vln_mujoco.sim_demo_runtime as runtime


class FakeSim:
    def __init__(self):
        self.generation=1;self.pose=np.array([0.,0.,0.]);self.lock=threading.RLock()
        self.active=False;self.reference=None;self.target=np.zeros(2);self.velocity=np.zeros(2)
        self.fault=None;self.reason='';self.evidence=SimpleNamespace(error='',add=lambda *args:None)
        self.geometry=SimpleNamespace(check=lambda points:{'safe':False,'reason':'PATH_COLLISION_RISK'})
    def cancel(self,reason):
        self.generation+=1;self.active=False;self.reference=None;self.reason=reason;self.velocity[:]=0
    def stats(self):return {'motion_duty_cycle':0.,'active':self.active}


def run_case(tmp_path,monkeypatch,kind):
    sim=FakeSim()
    monkeypatch.setattr(runtime,'repair_path',lambda *args:{'safe':False,'reason':'NO_SAFE_LOCAL_PATH'})
    if kind=='late':
        def cancel_during_check(points):
            sim.cancel('USER_STOP');return {'safe':True}
        sim.geometry.check=cancel_during_check
    class FakeHistory:
        def __init__(self,*args):pass
        async def start(self):pass
        async def predict(self):
            if kind in {'timeout','sample'}:
                raise RuntimeError('MODEL_REQUEST_TIMEOUT:next' if kind=='timeout' else 'MISSED_REAL_4HZ_CAPTURE')
            return {'reply':{'data':{'rc':0,'stop':kind=='stop',
                    'actions':{'actions':[[.1*i,0,0] for i in range(1,11)]}}},
                'frame':CameraFrame(1,b'',b'original',(0.,0.,0.),int(runtime.time.monotonic()*1e9)),
                'elapsed_s':.1,'sequence':4}
        async def close(self):
            # Motion must be invalidated BEFORE potentially slow evidence cleanup.
            assert not sim.active
            assert sim.reference is None
    monkeypatch.setattr(runtime,'OrderedHistory',FakeHistory)
    app=runtime.DemoApp.__new__(runtime.DemoApp)
    app.sim=sim;app.next_task=0;app.output=tmp_path;app.score_target=np.array([2.,0.])
    app.task_config={'instruction':'go'};app.latencies=[];app.prediction_events=[]
    app.pointing_anchors=lambda *args:{}
    asyncio.run(app.navigate('go'))
    assert not sim.active and sim.reference is None
    return app.navigation['outcome']


@pytest.mark.parametrize('kind,expected',[
    ('late','USER_STOP'),
    ('timeout','MODEL_REQUEST_TIMEOUT:next'),('sample','MISSED_REAL_4HZ_CAPTURE'),
    ('stop','MODEL_STOP_OUTSIDE_SUCCESS_CONDITIONS')])
def test_terminal_results_never_resume_motion(tmp_path,monkeypatch,kind,expected):
    assert run_case(tmp_path,monkeypatch,kind)==expected


def test_collision_risk_is_replanned_three_times_then_stops(tmp_path,monkeypatch):
    sim=FakeSim()
    monkeypatch.setattr(runtime,'repair_path',lambda *args:{'safe':False,'reason':'NO_SAFE_LOCAL_PATH'})
    class RiskHistory:
        def __init__(self,*args): self.calls=0
        async def start(self): pass
        async def predict(self):
            self.calls += 1
            return {'reply':{'data':{'rc':0,'stop':False,
                    'actions':{'actions':[[.1*i,0,0] for i in range(1,11)]}}},
                'frame':CameraFrame(1,b'',b'original',(0.,0.,0.),int(runtime.time.monotonic()*1e9)),
                'elapsed_s':.1,'sequence':self.calls}
        async def close(self):
            assert not sim.active
            assert sim.reference is None
    monkeypatch.setattr(runtime,'OrderedHistory',RiskHistory)
    app=runtime.DemoApp.__new__(runtime.DemoApp)
    app.sim=sim;app.next_task=0;app.output=tmp_path;app.score_target=np.array([2.,0.])
    app.task_config={'instruction':'go'};app.latencies=[];app.prediction_events=[]
    app.pointing_anchors=lambda *args:{}
    asyncio.run(app.navigate('go'))
    assert app.navigation['outcome']=='PATH_COLLISION_RISK'
    assert app.navigation['replan_attempts']==runtime.MAX_REPLAN_ATTEMPTS
    assert app.navigation['replan_limit_reached']
    assert not sim.active and sim.reference is None


def test_stop_cancels_pending_task():
    async def exercise():
        app=runtime.DemoApp.__new__(runtime.DemoApp);app.sim=FakeSim()
        app.task=asyncio.create_task(asyncio.sleep(10));app.stop('USER_STOP')
        with pytest.raises(asyncio.CancelledError):await app.task
        assert app.sim.reason=='USER_STOP' and not app.sim.active
    asyncio.run(exercise())


@pytest.mark.parametrize('mode',['move','cancel','expire'])
def test_local_repair_handoff_moves_or_rejects_stale_result(tmp_path,monkeypatch,mode):
    from vln_mujoco.sim_demo_core import FastCollision, integrate
    from vln_mujoco.sim_demo_replan import repair_path as real_repair
    sim=FakeSim()
    footprint=np.array([[-.25,-.25],[.25,-.25],[.25,.25],[-.25,.25]])
    wall=np.array([[.85,.2],[1.3,.2],[1.3,.8],[.85,.8]])
    sim.geometry=FastCollision(footprint,[(259,wall)])
    events=[];sim.evidence.add=lambda file,row:events.append(row)
    class History:
        def __init__(self,*args):self.calls=0
        async def start(self):pass
        async def predict(self):
            self.calls+=1
            if self.calls>1:
                assert sim.reference is not None
                assert sim.reference.path_id.endswith(':local')
                start=sim.pose.copy()
                for tick in range(200):
                    command,_=sim.reference.command(sim.pose,runtime.time.monotonic())
                    sim.velocity+=np.clip(np.array(command)-sim.velocity,[-.0015,-.005],[.0015,.005])
                    sim.pose=integrate(sim.pose,sim.velocity)
                assert np.linalg.norm(sim.pose[:2]-start[:2])>.05
                raise RuntimeError('TEST_COMPLETED_MOVEMENT')
            return {'reply':{'data':{'rc':0,'stop':False,
                    'actions':{'actions':[[.1*i,0,0] for i in range(1,11)]}}},
                'frame':CameraFrame(1,b'',b'original',(0.,0.,0.),int(runtime.time.monotonic()*1e9)),
                'elapsed_s':.1,'sequence':self.calls}
        async def close(self):
            assert sim.reference is None and not sim.active
    def repair(*args):
        result=real_repair(*args)
        assert result['safe']
        if mode=='cancel':sim.cancel('USER_STOP')
        elif mode=='expire':
            monkeypatch.setattr(runtime,'PATH_TTL',0.)
        return result
    monkeypatch.setattr(runtime,'OrderedHistory',History)
    monkeypatch.setattr(runtime,'repair_path',repair)
    app=runtime.DemoApp.__new__(runtime.DemoApp)
    app.sim=sim;app.next_task=0;app.output=tmp_path;app.score_target=np.array([2.,0.])
    app.task_config={'instruction':'go'};app.latencies=[];app.prediction_events=[]
    app.pointing_anchors=lambda *args:{}
    asyncio.run(app.navigate('go'))
    expected={'move':'TEST_COMPLETED_MOVEMENT','cancel':'USER_STOP','expire':'PATH_EXPIRED_BEFORE_INSTALL'}
    assert app.navigation['outcome']==expected[mode]
    installed=[e for e in events if e.get('event')=='PATH_INSTALLED']
    assert len(installed)==(1 if mode=='move' else 0)
    if installed:
        assert installed[0]['source']=='local_repair'
        assert installed[0]['check']['minimum_m']>=.02
        original=json.loads((tmp_path/'navigation-0001/prediction-01/full-path-check.json').read_text())
        assert not original['check']['safe']
    assert not app.navigation['strict_success']


def test_identical_failed_repairs_are_not_recomputed(tmp_path,monkeypatch):
    calls=[]
    def failed(*args):
        calls.append(1)
        return {'safe':False,'reason':'NO_SAFE_LOCAL_PATH'}
    monkeypatch.setattr(runtime,'repair_path',failed)
    # run_case installs its own fake planner; reproduce its risk history here.
    sim=FakeSim()
    class History:
        def __init__(self,*args):pass
        async def start(self):pass
        async def predict(self):
            return {'reply':{'data':{'rc':0,'stop':False,
                'actions':{'actions':[[.1*i,0,0] for i in range(1,11)]}}},
                'frame':CameraFrame(1,b'',b'original',(0.,0.,0.),int(runtime.time.monotonic()*1e9)),
                'elapsed_s':.1,'sequence':4}
        async def close(self):assert not sim.active and sim.reference is None
    monkeypatch.setattr(runtime,'OrderedHistory',History)
    app=runtime.DemoApp.__new__(runtime.DemoApp)
    app.sim=sim;app.next_task=0;app.output=tmp_path;app.score_target=np.array([2.,0.])
    app.task_config={'instruction':'go'};app.pointing_anchors=lambda *args:{}
    asyncio.run(app.navigate('go'))
    assert len(calls)==1
    assert app.navigation['predictions']==3
    assert app.navigation['repeated_path_count']==2
    assert app.navigation['outcome']=='PATH_COLLISION_RISK'


def test_render_history_sample_uses_actual_camera_frame_schema(monkeypatch):
    """Exercise first history capture; idle rendering alone missed this regression."""
    from collections import deque
    sim=runtime.DemoSimulation.__new__(runtime.DemoSimulation)
    sim.lock=threading.RLock();sim.quit=threading.Event()
    sim.model=SimpleNamespace(camera=lambda name:SimpleNamespace(id=0),cam_fovy=[70.])
    sim.render_data=SimpleNamespace(cam_xpos=np.array([[1.,2.,3.]]),cam_xmat=np.array([np.eye(3).ravel()]))
    sim.data=None;sim.resolution=(640,360);sim.pose=np.array([4.,5.,.6])
    sim.height=1.;sim._scene_epoch=2;sim.task_id='test';sim.active=False
    sim.history_fifo=deque(maxlen=10);sim.history_next=0.;sim.history_last=None
    sim.history_error='';sim.frames=sim.third_frames=0
    sim.render_events=deque();sim.third_events=deque();sim.render_times=deque()
    errors=[];sim.cancel=lambda *args,**kwargs:errors.append(args)
    sim.evidence=SimpleNamespace(add=lambda *args:None)
    class Renderer:
        def __init__(self,model,height,width):self.shape=(height,width,3)
        def update_scene(self,*args,**kwargs):pass
        def render(self):
            sim.quit.set()
            return np.zeros(self.shape,dtype=np.uint8)
        def close(self):pass
    monkeypatch.setattr(runtime.mujoco,'Renderer',Renderer)
    monkeypatch.setattr(runtime.mujoco,'mj_copyData',lambda *args:None)
    sim.render()
    assert not errors
    frame=sim.history_fifo[0]
    assert isinstance(frame,CameraFrame)
    assert len(frame.rgb)==480*270*3 and frame.jpeg.startswith(b'\xff\xd8')
    assert frame.monotonic_ns>0 and frame.pose==(4.,5.,.6)
    assert frame.camera_position==(1.,2.,3.)
    assert frame.camera_orientation==tuple(np.eye(3).ravel())
    assert frame.camera_fovy==70. and frame.root_position==(4.,5.,1.)


def test_stale_short_turn_parks_and_reobserves_without_returning_to_capture(tmp_path,monkeypatch):
    sim=FakeSim();sim.pose=np.array([.55,0.,0.])
    sim.geometry.check=lambda *args:{'safe':True,'reason':'CLEAR'}
    events=[];sim.evidence.add=lambda file,row:events.append(row)
    def unexpected_repair(*args):
        raise AssertionError('Do not repair a stale short turn back to its capture')
    monkeypatch.setattr(runtime,'repair_path',unexpected_repair)
    class History:
        def __init__(self,*args):self.calls=0
        async def start(self):pass
        async def predict(self):
            self.calls+=1
            if self.calls==2:
                assert sim.reference is None
                assert np.all(sim.velocity==0)
                assert sim.state=='REOBSERVING'
            if self.calls==3:
                assert sim.reference is not None
                assert np.allclose(sim.reference.points[0],sim.pose)
                raise RuntimeError('FRESH_PATH_INSTALLED')
            capture=(0.,0.,0.) if self.calls==1 else tuple(sim.pose)
            return {'reply':{'data':{'rc':0,'stop':False,
                'actions':{'actions':[[.002*i,0.,.1*i] for i in range(1,11)]}}},
                'frame':CameraFrame(1,b'',b'original',capture,int(runtime.time.monotonic()*1e9)),
                'elapsed_s':4.,'sequence':self.calls}
        async def close(self):assert not sim.active and sim.reference is None
    monkeypatch.setattr(runtime,'OrderedHistory',History)
    app=runtime.DemoApp.__new__(runtime.DemoApp)
    app.sim=sim;app.next_task=0;app.output=tmp_path;app.score_target=np.array([2.,0.])
    app.task_config={'instruction':'go'};app.pointing_anchors=lambda *args:{}
    asyncio.run(app.navigate('go'))
    assert app.navigation['outcome']=='FRESH_PATH_INSTALLED'
    assert (tmp_path/'navigation-0001/prediction-01/reobserve.json').exists()
    assert [e['path_id'] for e in events if e.get('event')=='PATH_INSTALLED']==['navigation-0001:2']


def test_free_instruction_stop_is_unscored_not_registered_task_failure(tmp_path,monkeypatch):
    sim=FakeSim()
    class History:
        def __init__(self,*args):pass
        async def start(self):pass
        async def predict(self):
            return {'reply':{'data':{'rc':0,'stop':True}},
                'frame':CameraFrame(1,b'',b'original',(0.,0.,0.),int(runtime.time.monotonic()*1e9)),
                'elapsed_s':.1,'sequence':1}
        async def close(self):assert not sim.active
    monkeypatch.setattr(runtime,'OrderedHistory',History)
    app=runtime.DemoApp.__new__(runtime.DemoApp)
    app.sim=sim;app.next_task=0;app.output=tmp_path;app.score_target=np.array([2.,0.])
    app.task_config={'instruction':'registered doorframe'};app.pointing_anchors=lambda *args:{}
    asyncio.run(app.navigate('find refrigerator'))
    assert app.navigation['outcome']=='MODEL_STOP_UNSCORED'
    assert not app.navigation['strict_success']


def test_control_renewal_is_independent_of_telemetry_and_expires_with_owner():
    async def exercise():
        app=runtime.DemoApp.__new__(runtime.DemoApp);app.sim=FakeSim()
        app.closed=False;app.task=None;app.navigation=None
        app.owner=SimpleNamespace(closed=False);app.browser_heartbeat=runtime.time.monotonic()
        app.sim.active=True;app.sim.command_at=0.
        control=asyncio.create_task(app.control_lease())
        try:
            # No telemetry broadcast runs here; a slow viewer cannot withhold renewal.
            await asyncio.sleep(.12)
            assert app.sim.active
            assert runtime.time.monotonic()-app.sim.command_at<.10
            app.browser_heartbeat=runtime.time.monotonic()-3.
            await asyncio.sleep(.07)
            assert not app.sim.active
            assert app.sim.reason=='BROWSER_DISCONNECTED_OR_HEARTBEAT_EXPIRED'
        finally:
            app.closed=True
            await control
    asyncio.run(exercise())


def test_slow_viewer_close_does_not_block_other_viewers_or_broadcast_loop():
    async def exercise():
        app=runtime.DemoApp.__new__(runtime.DemoApp)
        async def hanging():await asyncio.sleep(30)
        slow=SimpleNamespace(closed=False,send_json=lambda message:hanging(),close=hanging)
        messages=[]
        async def receive(message):messages.append(message)
        fast=SimpleNamespace(closed=False,send_json=receive)
        app.sockets=[slow,fast];app.health=lambda:{'navigation':None,'prediction':None}
        await asyncio.wait_for(app.broadcast_state(),1.)
        assert len(messages)==1 and messages[0]['navigation'] is None
        app.sockets=[fast]
        await app.broadcast_state()
        assert len(messages)==2
    asyncio.run(exercise())


def test_telemetry_exception_is_recorded_and_next_update_recovers(tmp_path,monkeypatch):
    monkeypatch.setattr(runtime.shutil,'disk_usage',lambda _:SimpleNamespace(free=5*1024**3))
    monkeypatch.setattr(runtime,'Path',lambda _:SimpleNamespace(read_text=lambda:'MemAvailable: 5000000 kB'))
    async def exercise():
        app=runtime.DemoApp.__new__(runtime.DemoApp);app.sim=FakeSim()
        app.closed=False;app.output=tmp_path;app.sockets=[];app.telemetry_error=None
        events=[];app.sim.evidence.add=lambda name,row:events.append(row)
        calls=[]
        def health():
            calls.append(1)
            if len(calls)==1:raise RuntimeError('TELEMETRY_TEST')
            app.closed=True
            return {'navigation':None}
        app.health=health
        await asyncio.wait_for(app.watch(),1.)
        assert len(calls)==2 and app.telemetry_error is None
        assert events[0]['event']=='TELEMETRY_ERROR'
        assert events[0]['reason']=='TELEMETRY_TEST'
    asyncio.run(exercise())
