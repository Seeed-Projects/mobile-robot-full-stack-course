import math
import numpy as np
import pytest
from vln_mujoco.sim_demo_core import FastCollision, Reference, integrate, world_path, separation, transform

BOX=np.array([[-.25,-.25],[.25,-.25],[.25,.25],[-.25,.25]])


def test_cumulative_capture_transform():
    local=np.array([[i*.1,0,0] for i in range(1,11)])
    path=world_path(local,(2,3,math.pi/2))
    assert np.allclose(path[-1],[2,4,math.pi/2])
    assert np.allclose(path[0],[2,3,math.pi/2])


@pytest.mark.parametrize('shape',[(9,3),(10,2),(11,3)])
def test_invalid_path(shape):
    with pytest.raises(ValueError):world_path(np.zeros(shape),(0,0,0))


def test_nonfinite():
    local=np.zeros((10,3));local[2,1]=np.nan
    with pytest.raises(ValueError):world_path(local,(0,0,0))


def test_thin_obstacle_between_points_rejected():
    wall=np.array([[.499,-1],[.501,-1],[.501,1],[.499,1]])
    geometry=FastCollision(BOX,[(7,wall)])
    assert not geometry.check(np.array([[0,0,0],[1,0,0]]))['safe']


def test_rotation_sweep_not_only_endpoints():
    body=np.array([[-.5,-.1],[.5,-.1],[.5,.1],[-.5,.1]])
    wall=np.array([[.32,.32],[.36,.32],[.36,.36],[.32,.36]])
    geometry=FastCollision(body,[(8,wall)])
    assert not geometry.check(np.array([[0,0,0],[0,0,math.pi/2]]))['safe']


def test_margin_and_safe_corridor():
    geometry=FastCollision(BOX,[(9,BOX+[0,1])])
    assert geometry.check(np.array([[0,0,0],[1,0,0]]))['safe']
    assert not geometry.check(np.array([[0,.49,0],[1,.49,0]]))['safe']


def test_ttl_and_progress_do_not_reanchor():
    points=world_path(np.array([[i*.1,0,0] for i in range(1,11)]),(2,3,0))
    ref=Reference(points,100,'x');ref.command((2.3,3,0),101)
    saved=ref.progress;ref.command((2.2,3,0),102)
    assert ref.progress>=saved
    assert np.allclose(ref.points[-1],[3,3,0])
    assert ref.command((2.3,3,0),110)[1]=='PATH_EXPIRED'


def test_no_target_or_synthetic_stop():
    ref=Reference(np.array([[0,0,0],[.5,0,0]]),100,'x')
    assert ref.command((.5,0,0),101)[1]=='PATH_EXHAUSTED'
    assert ref.command((0,1,0),101)[1]=='TRACKING_DEVIATION'


def test_signed_motion_and_wrap():
    assert integrate((0,0,0),(.1,0),1)[0]==pytest.approx(.1)
    assert integrate((0,0,0),(0,.5),1)[2]==pytest.approx(.5)
    assert -math.pi<=integrate((0,0,3.14),(0,.5),1)[2]<=math.pi


def test_geometry_timeout_never_approves():
    geometry=FastCollision(BOX,[(9,BOX+[0,1])])
    assert geometry.check(np.array([[0,0,0],[1,0,0]]),timeout=-1)['reason']=='FAST_CHECK_TIMEOUT'


def test_separation_is_conservative():
    assert separation(BOX,BOX+[1,1])<=math.sqrt(.5)
    assert separation(BOX,BOX)<0


def test_rotation_progress_bounds_oscillations_without_blocking_one_turn():
    from vln_mujoco.sim_demo_core import RotationProgress
    guard=RotationProgress((0,0,0),0.)
    # A useful 150-degree observation turn is allowed, even at 30 seconds.
    for i in range(1,151):
        assert not guard.observe((0,0,np.deg2rad(i)),i*.2)
    # Repeated left/right searching counts actual rotation, not net yaw.
    stopped=False
    for i in range(151,400):
        yaw=np.deg2rad(300-i)
        stopped=guard.observe((0,0,yaw),i*.2)
        if stopped:break
    assert stopped


def test_translation_resets_rotation_progress_budget():
    from vln_mujoco.sim_demo_core import RotationProgress
    guard=RotationProgress((0,0,0),0.)
    for i in range(1,401):
        assert not guard.observe((i*.002,0,i*.025),i*.1)
