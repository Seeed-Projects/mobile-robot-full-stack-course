"""Pure-simulation geometry and tracking. No hardware or strict-mode imports."""
from __future__ import annotations
import math
import time
import numpy as np

DT = .005
V_MAX, W_MAX = .15, .50
PATH_TTL = 10.
MARGIN = .020


class RotationProgress:
    """Bound repeated turning using measured poses, without goal coordinates."""
    def __init__(self, pose, now):
        self.origin = np.asarray(pose[:2]).copy()
        self.last_yaw = float(pose[2])
        self.started = now
        self.rotation = 0.

    def observe(self, pose, now):
        self.rotation += abs(wrap(pose[2]-self.last_yaw))
        self.last_yaw = float(pose[2])
        if np.linalg.norm(np.asarray(pose[:2])-self.origin) >= .15:
            self.origin = np.asarray(pose[:2]).copy()
            self.started = now
            self.rotation = 0.
        return now-self.started >= 30. and self.rotation >= 2*math.pi


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def hull(points):
    pts = sorted(set(map(tuple, np.asarray(points)[:, :2])))
    if len(pts) < 3:
        raise ValueError('Degenerate collision polygon')
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 1e-12: lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 1e-12: hi.pop()
        hi.append(p)
    return np.asarray(lo[:-1]+hi[:-1], dtype=float)


def axes(poly):
    edge = np.roll(poly, -1, axis=0)-poly
    out = np.column_stack((-edge[:, 1], edge[:, 0]))
    return out/np.linalg.norm(out, axis=1)[:, None]


def transform(poly, pose):
    x, y, a = pose
    c, s = math.cos(a), math.sin(a)
    return poly @ np.array([[c, s], [-s, c]]) + [x, y]


def separation(a, b, b_axes=None):
    """Separating-axis clearance lower bound; <=0 means projected overlap."""
    directions = np.concatenate((axes(a), axes(b) if b_axes is None else b_axes))
    pa, pb = a @ directions.T, b @ directions.T
    return float(np.maximum(pb.min(0)-pa.max(0), pa.min(0)-pb.max(0)).max())


def world_path(local, capture):
    local = np.asarray(local, dtype=float)
    if local.shape != (10, 3) or not np.isfinite(local).all():
        raise ValueError('Expected finite cumulative (10,3) path')
    result = np.empty_like(local)
    result[:, :2] = transform(local[:, :2], capture)
    result[:, 2] = [wrap(capture[2]+a) for a in local[:, 2]]
    return np.vstack((capture, result))


def swept_rows(points, radius, step=.01):
    yield np.asarray(points[0]), 0.
    for a, b in zip(points[:-1], points[1:]):
        delta = np.asarray(b)-a
        delta[2] = wrap(delta[2])
        bound = float(np.linalg.norm(delta[:2])+radius*abs(delta[2]))
        count = max(1, math.ceil(bound/step), math.ceil(abs(delta[2])/math.radians(1)))
        for i in range(1, count+1):
            yield np.asarray(a)+delta*i/count, bound/count


class FastCollision:
    """Conservative 2.5D convex projections of existing collision geoms.

    Original geom IDs are retained. The whole robot hull covers its 47 collision
    geoms and all caster swivel positions; z-overlapping environment geoms are
    projected conservatively. This may refuse a safe path, never shrinks it.
    """
    def __init__(self, footprint, obstacles):
        self.footprint = hull(footprint)
        self.radius = float(np.linalg.norm(self.footprint, axis=1).max())
        self.obstacles = []
        for identity, points in obstacles:
            polygon = hull(points)
            self.obstacles.append((identity, polygon, axes(polygon)))
        self.bounds = np.asarray([(p.min(0), p.max(0)) for _, p, _ in self.obstacles])

    def clearance(self, pose, limit=.10):
        robot = transform(self.footprint, pose)
        if not len(self.obstacles): return limit, None
        lo, hi = robot.min(0), robot.max(0)
        gap = np.maximum(0, np.maximum(self.bounds[:, 0]-hi, lo-self.bounds[:, 1]))
        candidates = np.flatnonzero(np.linalg.norm(gap, axis=1) <= limit)
        minimum, nearest = limit, None
        for i in candidates:
            identity, poly, direction = self.obstacles[i]
            distance = separation(robot, poly, direction)
            if not math.isfinite(distance): raise ValueError('Nonfinite geometry distance')
            if distance < minimum: minimum, nearest = distance, identity
        return minimum, nearest

    def check(self, points, margin=MARGIN, timeout=.5, step=.01):
        started = time.monotonic()
        minimum, nearest, count = math.inf, None, 0
        for pose, bound in swept_rows(np.asarray(points), self.radius, step):
            count += 1
            d, obj = self.clearance(pose)
            certificate = d-bound
            if certificate < minimum: minimum, nearest = certificate, obj
            if certificate < margin:
                return dict(safe=False, reason='PATH_COLLISION_RISK', minimum_m=minimum,
                            obstacle=nearest, risk_pose=pose.tolist(), samples=count,
                            elapsed_s=time.monotonic()-started)
            if time.monotonic()-started > timeout:
                return dict(safe=False, reason='FAST_CHECK_TIMEOUT', samples=count,
                            elapsed_s=time.monotonic()-started)
        return dict(safe=True, reason='CLEAR', minimum_m=minimum, obstacle=nearest,
                    samples=count, elapsed_s=time.monotonic()-started)


class Reference:
    def __init__(self, points, capture_time, path_id):
        self.points = np.asarray(points)
        self.capture_time, self.path_id = capture_time, path_id
        self.lengths = np.linalg.norm(np.diff(self.points[:, :2], axis=0), axis=1)
        self.cumulative = np.r_[0., np.cumsum(self.lengths)]
        self.progress = 0.

    def locate(self, pose):
        p = np.asarray(pose[:2]); best = None
        for i, length in enumerate(self.lengths):
            if self.cumulative[i+1]+1e-6 < self.progress: continue
            a, b = self.points[i:i+2, :2]
            t = float(np.clip(np.dot(p-a, b-a)/max(length*length, 1e-12), 0, 1))
            if length > 1e-9:
                t = max(t, float(np.clip((self.progress-self.cumulative[i])/length,0,1)))
            s = self.cumulative[i]+t*length
            if s+1e-6 < self.progress: continue
            distance = float(np.linalg.norm(p-(a+t*(b-a))))
            if best is None or distance < best[0]: best = distance, s, i
        if best is None: return float(np.linalg.norm(p-self.points[-1,:2])), len(self.points)-2
        self.progress = max(self.progress, best[1])
        return best[0], best[2]

    def remaining(self, pose):
        _, i = self.locate(pose)
        return np.vstack((pose, self.points[i+1:]))

    def command(self, pose, now):
        if now-self.capture_time >= PATH_TTL: return (0., 0.), 'PATH_EXPIRED'
        cross, _ = self.locate(pose)
        if cross > .20: return (0., 0.), 'TRACKING_DEVIATION'
        remaining = self.cumulative[-1]-self.progress
        final = self.points[-1]
        if remaining < .025:
            error = wrap(final[2]-pose[2])
            if abs(error) < .04: return (0., 0.), 'PATH_EXHAUSTED'
            return (0., float(np.clip(1.5*error, -W_MAX, W_MAX))), 'TRACKING'
        target_s = min(self.cumulative[-1], self.progress+.22)
        i = min(len(self.lengths)-1, int(np.searchsorted(self.cumulative, target_s, side='right')-1))
        t = (target_s-self.cumulative[i])/max(self.lengths[i], 1e-12)
        target = self.points[i, :2]*(1-t)+self.points[i+1, :2]*t
        delta = target-np.asarray(pose[:2])
        alpha = wrap(math.atan2(delta[1], delta[0])-pose[2])
        # Geometric lookahead follows only the model path, without target coordinates.
        if abs(alpha) > .7: return (0., float(np.clip(1.5*alpha, -W_MAX, W_MAX))), 'TRACKING'
        v = min(V_MAX, max(.025, remaining*.8))*max(.15, math.cos(alpha))
        w = float(np.clip(2*v*math.sin(alpha)/max(np.linalg.norm(delta), .05), -W_MAX, W_MAX))
        v *= max(.3, 1.-abs(w)/W_MAX*.6)
        return (v, w), 'TRACKING'


def integrate(pose, command, dt=DT):
    x, y, a = pose; v, w = command
    if abs(w) < 1e-9: return np.array([x+v*math.cos(a)*dt, y+v*math.sin(a)*dt, wrap(a+w*dt)])
    return np.array([x+v/w*(math.sin(a+w*dt)-math.sin(a)),
                     y-v/w*(math.cos(a+w*dt)-math.cos(a)), wrap(a+w*dt)])
