"""Bounded local path repair for the kinematic Demo only (no task coordinates).

The model supplies the route and terminal pose. We try smooth lateral detours,
then nearby lateral terminal corrections when that pose itself is obstructed.
Every returned route AND its closed-loop tracker rollout retain the same margin.
This is a finite candidate search, not a completeness or dynamics guarantee.
"""
from __future__ import annotations

import hashlib
import time

import numpy as np

from .sim_demo_core import MARGIN, Reference, integrate, wrap

MAX_ENDPOINT_SHIFT = .40
PLAN_TIMEOUT = 2.5


def path_signature(points):
    """Stable across identical stationary model replies; includes world root."""
    return hashlib.sha256(np.round(np.asarray(points), 3).astype('<f8').tobytes()).hexdigest()


def candidates(points):
    points = np.asarray(points, dtype=float)
    length = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))]
    if length[-1] < .05:
        return
    samples = np.linspace(0., length[-1], max(12, int(np.ceil(length[-1]/.05))+1))
    # Remove duplicate positions before interpolating; preserve the terminal yaw.
    keep = np.r_[np.diff(length) > 1e-9, True]
    base = np.column_stack([np.interp(samples, length[keep], points[keep, i]) for i in range(2)])
    t = samples/length[-1]
    direction = points[-1, :2]-points[0, :2]
    if np.linalg.norm(direction) < .05:
        return
    normal = np.array([-direction[1], direction[0]])/np.linalg.norm(direction)
    # Fixed-end detours first; terminal corrections never move along the path
    # (no execution of a truncated safe prefix or hidden task-goal substitution).
    for mode in ('detour', 'terminal_correction'):
        for magnitude in (.04, .08, .12, .16, .20, .24, .30, .36, MAX_ENDPOINT_SHIFT):
            for side in (1., -1.):
                offset = side*magnitude
                shape = np.sin(np.pi*t)**2 if mode == 'detour' else t*t*(3-2*t)
                xy = base+shape[:, None]*offset*normal
                tangent = np.gradient(xy, axis=0)
                yaw = np.arctan2(tangent[:, 1], tangent[:, 0])
                route = np.column_stack((xy, yaw))
                route[0] = points[0]
                route[-1, 2] = points[-1, 2]
                yield route, {'kind':mode, 'offset_m':offset,
                              'endpoint_shift_m':float(np.linalg.norm(route[-1, :2]-points[-1, :2]))}


def tracker_rollout(points, geometry, deadline, cancelled=lambda: False):
    """Check the actual 50Hz steering law, acceleration and swept motion.

    The virtual clock intentionally stays fresh to certify the *whole* route,
    even if a real Reference expires sooner. Runtime still enforces its 10s TTL.
    Integrate the same 5ms acceleration updates and 20ms steering refresh as the
    runtime. Bound each batch by its total translation and absolute rotation.
    """
    reference = Reference(points, 0., 'offline-local-repair')
    pose = np.asarray(points[0]).copy()
    velocity = np.zeros(2)
    trace = [pose.tolist()]
    minimum = float('inf')
    for index in range(3000):
        if cancelled():
            return {'safe':False, 'reason':'CANCELLED_OR_STALE'}
        if time.monotonic() >= deadline:
            return {'safe':False, 'reason':'LOCAL_PLAN_TIMEOUT'}
        command, status = reference.command(pose, 0.)
        if status == 'PATH_EXHAUSTED' and np.linalg.norm(velocity) < 1e-6 and np.linalg.norm(pose[:2]-points[-1, :2]) <= .04:
            return {'safe':True, 'reason':'CLEAR', 'minimum_m':minimum,
                    'simulated_s':index*.02, 'samples':len(trace), 'trace':trace}
        if status not in {'TRACKING', 'PATH_EXHAUSTED'}:
            return {'safe':False, 'reason':status}
        candidate = pose.copy()
        bound = 0.
        for _ in range(4):
            velocity += np.clip(np.asarray(command)-velocity, [-.0015, -.005], [.0015, .005])
            next_pose = integrate(candidate, velocity)
            bound += float(np.linalg.norm(next_pose[:2]-candidate[:2])+geometry.radius*abs(wrap(next_pose[2]-candidate[2])))
            candidate = next_pose
        clearance, obstacle = geometry.clearance(candidate)
        certificate = clearance-bound
        minimum = min(minimum, certificate)
        if certificate < MARGIN:
            return {'safe':False, 'reason':'TRACKER_COLLISION_RISK',
                    'minimum_m':minimum, 'obstacle':obstacle, 'risk_pose':candidate.tolist()}
        pose = candidate
        trace.append(pose.tolist())
    return {'safe':False, 'reason':'TRACKER_NOT_CONVERGED'}


def repair_path(geometry, points, cancelled=lambda: False, timeout=PLAN_TIMEOUT):
    """Return a certified complete candidate or a reason, never a safe prefix."""
    started = time.monotonic()
    deadline = started+timeout
    points = np.asarray(points, dtype=float)
    attempts = []
    def result(reason, **values):
        return {'safe':reason == 'LOCAL_PATH_REPAIRED', 'reason':reason,
                'elapsed_s':time.monotonic()-started, 'attempts':attempts,
                'max_endpoint_shift_m':MAX_ENDPOINT_SHIFT, **values}
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
        return result('INVALID_LOCAL_PATH')
    for route, identity in candidates(points):
        if cancelled():
            return result('CANCELLED_OR_STALE')
        if time.monotonic() >= deadline:
            return result('LOCAL_PLAN_TIMEOUT')
        # An unsafe endpoint makes an otherwise expensive whole-route test moot.
        endpoint_clearance, endpoint_obstacle = geometry.clearance(route[-1])
        if endpoint_clearance < MARGIN:
            attempts.append({**identity, 'reason':'ENDPOINT_BLOCKED',
                             'minimum_m':endpoint_clearance, 'obstacle':endpoint_obstacle})
            continue
        check = geometry.check(route, timeout=max(0., min(.5, deadline-time.monotonic())))
        attempt = {**identity, **check}
        attempts.append(attempt)
        if not check['safe']:
            continue
        rollout = tracker_rollout(route, geometry, deadline, cancelled)
        attempt['tracker_check'] = {k:v for k,v in rollout.items() if k != 'trace'}
        if rollout['safe'] and not cancelled() and time.monotonic() < deadline:
            return result('LOCAL_PATH_REPAIRED', world_path=route.tolist(),
                          selected=identity, check=check, tracker_check=rollout)
    return result('NO_SAFE_LOCAL_PATH')
