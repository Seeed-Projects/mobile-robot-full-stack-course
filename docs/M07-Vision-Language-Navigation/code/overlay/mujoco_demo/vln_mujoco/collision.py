"""Robot/environment contacts, excluding self contacts and the named scene floor."""
from __future__ import annotations

import math

import mujoco
import numpy as np


class CollisionMonitor:
    def __init__(self, model, data, base_joint):
        self.model, self.data = model, data
        bodies = set()
        if base_joint is not None:
            root = int(model.jnt_bodyid[base_joint])
            for body in range(model.nbody):
                ancestor = body
                while ancestor and ancestor != root:
                    ancestor = int(model.body_parentid[ancestor])
                if ancestor == root:
                    bodies.add(body)
        self.robot_mask = np.asarray([int(body) in bodies for body in model.geom_bodyid], dtype=bool)
        self.floor_mask = np.asarray([model.geom(index).name == "floor" for index in range(model.ngeom)], dtype=bool)
        collidable = (model.geom_contype != 0) | (model.geom_conaffinity != 0)
        self.robot_ids = np.flatnonzero(self.robot_mask & collidable)
        self.environment_ids = np.flatnonzero(~self.robot_mask & ~self.floor_mask & collidable)
        self.first = None

    def reset(self):
        self.first = None

    def check(self):
        if self.first is not None:
            return self.first
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            left, right = int(contact.geom1), int(contact.geom2)
            if contact.dist > 0 or self.floor_mask[left] or self.floor_mask[right]:
                continue
            if bool(self.robot_mask[left]) == bool(self.robot_mask[right]):
                continue
            self.first = {"detected": True, "sim_time": float(self.data.time),
                          "geoms": [self.model.geom(left).name, self.model.geom(right).name],
                          "distance_m": float(contact.dist), "position": list(map(float, contact.pos))}
            return self.first
        return None

    def near_sweep(self, travel_bound):
        """Conservative sphere broad phase followed by actual geometry distance."""
        if not len(self.robot_ids) or not len(self.environment_ids):
            return False
        robot = self.robot_ids
        environment = self.environment_ids
        distances = np.linalg.norm(self.data.geom_xpos[robot, None] - self.data.geom_xpos[environment], axis=2)
        distances -= self.model.geom_rbound[robot, None] + self.model.geom_rbound[environment]
        for row, column in np.argwhere(distances <= travel_bound + 0.002):
            distance = mujoco.mj_geomDistance(self.model, self.data, int(robot[row]),
                                              int(environment[column]), travel_bound + 0.002, None)
            if not math.isfinite(distance) or distance <= travel_bound + 0.001:
                return True
        return False
