"""Footprint-to-box clearance for independent ground-truth evaluation."""
import math


def rectangle(x, y, yaw, sx, sy):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*dx-s*dy, y+s*dx+c*dy) for dx, dy in
            [(-sx/2, -sy/2), (sx/2, -sy/2), (sx/2, sy/2), (-sx/2, sy/2)]]


def point_segment(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0, min(1, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def polygon_distance(a, b):
    separated = False
    for poly in [a, b]:
        for p, q in zip(poly, poly[1:]+poly[:1]):
            axis = q[1]-p[1], p[0]-q[0]
            pa = [x*axis[0]+y*axis[1] for x, y in a]
            pb = [x*axis[0]+y*axis[1] for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                separated = True
    if not separated:
        return 0.0
    return min(point_segment(p, q, r) for first, second in [(a, b), (b, a)]
               for p in first for q, r in zip(second, second[1:]+second[:1]))


def clearance(pose, boxes):
    robot = rectangle(*pose, 0.49, 0.49)
    return min(polygon_distance(robot, rectangle(*box['center'], 0.0, *box['size'])) for box in boxes)
