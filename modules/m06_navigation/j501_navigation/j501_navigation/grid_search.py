"""Readable 8-connected Dijkstra, A*, and Theta* teaching implementations.

Nav2 experiments use its production plugins. This module runs independently of ROS
to expose search and line-of-sight logic on the same generated occupancy map.
"""
import heapq
import math
from pathlib import Path
import time

import numpy as np
import yaml


class Grid:
    def __init__(self, yaml_file, radius=0.36):
        yaml_file = Path(yaml_file)
        self.meta = yaml.safe_load(yaml_file.read_text())
        with (yaml_file.parent / self.meta['image']).open('rb') as stream:
            if stream.readline().strip() != b'P5':
                raise ValueError('Teaching loader expects the generated binary PGM (P5)')
            line = stream.readline()
            while line.startswith(b'#'):
                line = stream.readline()
            width, height = map(int, line.split())
            if stream.readline().strip() != b'255':
                raise ValueError('Expected an 8-bit PGM')
            self.pixels = np.frombuffer(stream.read(), dtype=np.uint8).reshape(height, width)[::-1]
        self.resolution = self.meta['resolution']
        self.origin = self.meta['origin'][:2]
        self.blocked = self.pixels < 250  # Occupied AND unknown are excluded.
        raw = self.blocked.copy()
        cells = math.ceil(radius/self.resolution)
        for dy in range(-cells, cells+1):
            for dx in range(-cells, cells+1):
                if math.hypot(dx, dy)*self.resolution > radius:
                    continue
                y0, y1 = max(0, dy), min(height, height+dy)
                x0, x1 = max(0, dx), min(width, width+dx)
                self.blocked[y0:y1, x0:x1] |= raw[y0-dy:y1-dy, x0-dx:x1-dx]

    def cell(self, point):
        return tuple(math.floor((point[i]-self.origin[i])/self.resolution) for i in range(2))

    def world(self, cell):
        return [(cell[i]+0.5)*self.resolution+self.origin[i] for i in range(2)]

    def free(self, cell):
        x, y = cell
        return 0 <= y < self.blocked.shape[0] and 0 <= x < self.blocked.shape[1] and not self.blocked[y, x]

    def visible(self, start, goal):
        """Supercover grid traversal; diagonal corner contact checks both side cells."""
        if not self.free(start) or not self.free(goal):
            return False
        x, y = start
        dx, dy = goal[0]-x, goal[1]-y
        sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
        tx = 0.5/abs(dx) if dx else math.inf
        ty = 0.5/abs(dy) if dy else math.inf
        stepx, stepy = (1/abs(dx) if dx else math.inf), (1/abs(dy) if dy else math.inf)
        while (x, y) != goal:
            if abs(tx-ty) < 1e-12:
                if not self.free((x+sx, y)) or not self.free((x, y+sy)):
                    return False
                x, y, tx, ty = x+sx, y+sy, tx+stepx, ty+stepy
            elif tx < ty:
                x, tx = x+sx, tx+stepx
            else:
                y, ty = y+sy, ty+stepy
            if not self.free((x, y)):
                return False
        return True

    def search(self, start_xy, goal_xy, algorithm='astar'):
        if algorithm not in ('astar', 'dijkstra', 'theta_star'):
            raise ValueError('Unknown algorithm')
        start, goal = self.cell(start_xy), self.cell(goal_xy)
        if not self.free(start) or not self.free(goal):
            raise ValueError('Start or goal is occupied, unknown, or too close to an obstacle')
        beginning = time.perf_counter()
        distance = lambda a, b: math.hypot(a[0]-b[0], a[1]-b[1])
        heuristic = lambda p: 0 if algorithm == 'dijkstra' else distance(p, goal)
        costs, parents, closed = {start: 0.0}, {start: start}, set()
        queue = [(heuristic(start), start)]
        while queue:
            _, current = heapq.heappop(queue)
            if current in closed:
                continue
            if current == goal:
                cells = [goal]
                while cells[-1] != start:
                    cells.append(parents[cells[-1]])
                cells.reverse()
                points = [self.world(cell) for cell in cells]
                return {'algorithm': algorithm, 'points': points, 'expanded': len(closed),
                        'planning_time_s': time.perf_counter()-beginning,
                        'length_m': sum(math.dist(a, b) for a, b in zip(points, points[1:])),
                        'source': 'standalone teaching search, not Nav2 plugin output'}
            closed.add(current)
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    neighbor = current[0]+dx, current[1]+dy
                    if (dx == dy == 0) or neighbor in closed or not self.visible(current, neighbor):
                        continue
                    parent = parents[current] if algorithm == 'theta_star' and self.visible(parents[current], neighbor) else current
                    cost = costs[parent] + distance(parent, neighbor)
                    if cost < costs.get(neighbor, math.inf):
                        costs[neighbor], parents[neighbor] = cost, parent
                        heapq.heappush(queue, (cost+heuristic(neighbor), neighbor))
        raise ValueError('No collision-free route on the inflated map')
