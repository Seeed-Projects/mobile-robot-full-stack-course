"""Actual geometry/controller tests, synthetic obstacles (not navigation evidence)."""
import math
import time

import numpy as np
import pytest

from vln_mujoco.sim_demo_core import FastCollision, MARGIN, world_path
from vln_mujoco.sim_demo_replan import repair_path, path_signature, tracker_rollout

BODY=np.array([[-.25,-.25],[.25,-.25],[.25,.25],[-.25,.25]])


def box(x1,x2,y1,y2):
    return np.array([[x1,y1],[x2,y1],[x2,y2],[x1,y2]])


@pytest.mark.parametrize('side',[1.,-1.])
def test_blocked_terminal_has_certified_lateral_correction(side):
    wall=box(.85,1.3,.20,.8)*[1,side]
    geometry=FastCollision(BODY,[(259,wall)])
    route=world_path([[i*.1,0.,0.] for i in range(1,11)],(0,0,0))
    assert not geometry.check(route)['safe']
    result=repair_path(geometry,route,timeout=5)
    assert result['safe'],result
    planned=np.array(result['world_path'])
    assert result['selected']['offset_m']*side<0
    assert 0<result['selected']['endpoint_shift_m']<=.4+1e-9
    assert np.allclose(planned[0],route[0])
    assert planned[-1,0]==pytest.approx(route[-1,0])  # no safe-prefix truncation
    assert geometry.check(planned)['safe']
    assert result['tracker_check']['minimum_m']>=MARGIN
    trace=np.array(result['tracker_check']['trace'])
    assert np.linalg.norm(trace[-1,:2]-trace[0,:2])>.9
    assert geometry.check(trace,timeout=5)['safe']


def test_no_route_does_not_authorize_prefix_or_lower_margin():
    geometry=FastCollision(BODY,[(8,box(.85,1.2,-2,2))])
    route=world_path([[i*.1,0.,0.] for i in range(1,11)],(0,0,0))
    result=repair_path(geometry,route)
    assert not result['safe']
    assert result['reason']=='NO_SAFE_LOCAL_PATH'
    assert 'world_path' not in result


def test_interior_detour_keeps_terminal_when_possible():
    body=BODY*.3
    geometry=FastCollision(body,[(8,box(.4,.5,-.025,.025))])
    route=world_path([[i*.1,0.,0.] for i in range(1,11)],(0,0,0))
    result=repair_path(geometry,route,timeout=5)
    assert result['safe'],result
    assert result['selected']['kind']=='detour'
    assert np.allclose(result['world_path'][-1],route[-1])


def test_cancellation_timeout_and_invalid_route_never_install():
    geometry=FastCollision(BODY,[])
    route=np.array([[0.,0,0],[1,0,0]])
    assert repair_path(geometry,route,cancelled=lambda:True)['reason']=='CANCELLED_OR_STALE'
    assert repair_path(geometry,route,timeout=0)['reason']=='LOCAL_PLAN_TIMEOUT'
    assert repair_path(geometry,[[0,0,0],[float('nan'),0,0]])['reason']=='INVALID_LOCAL_PATH'


def test_signature_includes_root_and_heading_and_groups_identical_replies():
    route=np.array([[0.,0,0],[1,0,0]])
    assert path_signature(route)==path_signature(route.copy())
    other=route.copy();other[0,0]=.1
    assert path_signature(route)!=path_signature(other)
    other=route.copy();other[-1,2]=.1
    assert path_signature(route)!=path_signature(other)


def test_rotating_body_rollout_cannot_cross_obstacle():
    body=np.array([[-.5,-.1],[.5,-.1],[.5,.1],[-.5,.1]])
    geometry=FastCollision(body,[(8,box(.32,.36,.32,.36))])
    route=np.array([[0.,0,0],[0,0,math.pi/2]])
    result=tracker_rollout(route,geometry,time.monotonic()+5)
    assert not result['safe']
    assert result['reason']=='TRACKER_COLLISION_RISK'
