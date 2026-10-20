from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from ..collision import CollisionMonitor
from ..model import PHYSICS_DT, SPAWN, compile_model, load_scene_xml
from .base import RenderCamera, RobotState, Twist2D

CAMERA_NAME = "robot_rgb"
THIRD_PERSON_CAMERA_NAME = "robot_third_person"
WHEEL_RADIUS = 0.033
WHEEL_TRACK = 0.160


def _robot_body() -> ET.Element:
    return ET.fromstring(
        f"""
        <body name="base_link" pos="{SPAWN[0]} {SPAWN[1]} {WHEEL_RADIUS}">
          <freejoint name="base_joint" />
          <inertial pos="0 0 0.055" mass="2.5" diaginertia="0.018 0.018 0.025" />
          <geom name="base_collision" type="cylinder" size="0.092 0.028" pos="0 0 0.030"
                rgba="0.08 0.11 0.13 1" friction="0.9 0.02 0.002" />
          <geom name="lower_plate" type="cylinder" size="0.088 0.010" pos="0 0 0.063"
                rgba="0.05 0.08 0.09 1" contype="0" conaffinity="0" />
          <geom name="upper_plate" type="cylinder" size="0.078 0.010" pos="0 0 0.145"
                rgba="0.08 0.68 0.77 1" contype="0" conaffinity="0" />
          <geom name="mast" type="cylinder" size="0.010 0.052" pos="-0.020 0 0.105"
                rgba="0.65 0.72 0.74 1" contype="0" conaffinity="0" />
          <geom name="lidar" type="cylinder" size="0.042 0.018" pos="0.015 0 0.178"
                rgba="0.05 0.08 0.09 1" contype="0" conaffinity="0" />
          <geom name="camera_case" type="box" size="0.022 0.030 0.018" pos="0.067 0 0.162"
                rgba="0.14 0.18 0.19 1" contype="0" conaffinity="0" />
          <camera name="{CAMERA_NAME}" mode="fixed" pos="0.090 0 0.165"
                  xyaxes="0 -1 0 0 0 1" fovy="79.865" />
          <camera name="{THIRD_PERSON_CAMERA_NAME}" mode="fixed" pos="-0.65 0 0.50"
                  xyaxes="0 -1 0 0.38 0 0.925" fovy="65" />
          <body name="left_wheel" pos="0 0.080 0">
            <joint name="left_wheel_joint" type="hinge" axis="0 1 0" />
            <geom name="left_wheel_geom" type="cylinder" size="{WHEEL_RADIUS} 0.012"
                  euler="{math.pi / 2} 0 0" mass="0.08" rgba="0.035 0.045 0.048 1"
                  friction="2.0 0.01 0.001" solref="0.004 1" />
          </body>
          <body name="right_wheel" pos="0 -0.080 0">
            <joint name="right_wheel_joint" type="hinge" axis="0 1 0" />
            <geom name="right_wheel_geom" type="cylinder" size="{WHEEL_RADIUS} 0.012"
                  euler="{math.pi / 2} 0 0" mass="0.08" rgba="0.035 0.045 0.048 1"
                  friction="2.0 0.01 0.001" solref="0.004 1" />
          </body>
          <body name="caster" pos="-0.073 0 -0.018">
            <geom name="caster_geom" type="sphere" size="0.015" mass="0.025"
                  rgba="0.32 0.36 0.37 1" friction="0.05 0.001 0.0001" />
          </body>
        </body>
        """
    )


def build_model(source: Path | None = None) -> mujoco.MjModel:
    root = load_scene_xml(source)
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("MolmoSpaces scene has no worldbody")
    worldbody.append(_robot_body())
    return compile_model(root)


class TurtleBotBackend:
    name = "TurtleBot"
    camera_name = CAMERA_NAME

    def __init__(self, source: Path | None = None) -> None:
        self.model = build_model(source)
        self.data = mujoco.MjData(self.model)
        self._left_joint = self.model.joint("left_wheel_joint").id
        self._right_joint = self.model.joint("right_wheel_joint").id
        self._base_joint = self.model.joint("base_joint").id
        self._base_qpos = self.model.jnt_qposadr[self._base_joint]
        self._base_dof = self.model.jnt_dofadr[self._base_joint]
        self._left_qpos = self.model.jnt_qposadr[self._left_joint]
        self._right_qpos = self.model.jnt_qposadr[self._right_joint]
        self._left_dof = self.model.jnt_dofadr[self._left_joint]
        self._right_dof = self.model.jnt_dofadr[self._right_joint]
        self._yaw = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.collision = CollisionMonitor(self.model, self.data, self._base_joint)

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self._yaw = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.collision.reset()

    def step(self, command: Twist2D) -> None:
        linear, angular = command
        if self.collision.first is not None:
            linear = angular = 0.0
        previous_qpos = self.data.qpos.copy()
        travel_bound = abs(linear) * PHYSICS_DT + abs(angular) * PHYSICS_DT * 0.25
        sweep = self.collision.near_sweep(travel_bound) if travel_bound > 0 else False
        previous_yaw = self._yaw
        self._yaw = math.atan2(
            math.sin(previous_yaw + angular * PHYSICS_DT),
            math.cos(previous_yaw + angular * PHYSICS_DT),
        )
        heading = previous_yaw + angular * PHYSICS_DT / 2.0
        self.data.qpos[self._base_qpos] += linear * math.cos(heading) * PHYSICS_DT
        self.data.qpos[self._base_qpos + 1] += linear * math.sin(heading) * PHYSICS_DT
        self.data.qpos[self._base_qpos + 3 : self._base_qpos + 7] = (
            math.cos(self._yaw / 2.0),
            0.0,
            0.0,
            math.sin(self._yaw / 2.0),
        )

        left = (linear - angular * WHEEL_TRACK / 2.0) / WHEEL_RADIUS
        right = (linear + angular * WHEEL_TRACK / 2.0) / WHEEL_RADIUS
        self.data.qpos[self._left_qpos] += left * PHYSICS_DT
        self.data.qpos[self._right_qpos] += right * PHYSICS_DT
        self.data.qvel.fill(0.0)
        self.data.qvel[self._base_dof] = linear * math.cos(self._yaw)
        self.data.qvel[self._base_dof + 1] = linear * math.sin(self._yaw)
        self.data.qvel[self._base_dof + 5] = angular
        self.data.qvel[self._left_dof] = left
        self.data.qvel[self._right_dof] = right
        self.data.time += PHYSICS_DT
        target_qpos = self.data.qpos.copy()
        steps = max(1, math.ceil(max(abs(linear) * PHYSICS_DT, abs(angular) * PHYSICS_DT) / 0.001)) if sweep else 1
        for step in range(1, steps + 1):
            fraction = step / steps
            self.data.qpos[:] = previous_qpos + fraction * (target_qpos - previous_qpos)
            yaw = previous_yaw + angular * PHYSICS_DT * fraction
            self.data.qpos[self._base_qpos + 3:self._base_qpos + 7] = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
            mujoco.mj_forward(self.model, self.data)
            if self.collision.check():
                self._yaw = yaw
                self.data.qvel.fill(0)
                mujoco.mj_forward(self.model, self.data)
                break

    def state(self) -> RobotState:
        qpos = self.data.qpos
        x, y, z = (
            float(value)
            for value in qpos[self._base_qpos : self._base_qpos + 3]
        )
        qw, qx, qy, qz = (
            float(value)
            for value in qpos[self._base_qpos + 3 : self._base_qpos + 7]
        )
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        left = float(self.data.qvel[self._left_dof])
        right = float(self.data.qvel[self._right_dof])
        linear = WHEEL_RADIUS * (left + right) / 2.0
        angular = WHEEL_RADIUS * (right - left) / WHEEL_TRACK
        return RobotState((x, y, z, yaw), (linear, angular),
                          telemetry={"collision": self.collision.first or {"detected": False},
                                     "finite": bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all())})

    def third_person_camera(self, _camera: mujoco.MjvCamera) -> RenderCamera:
        return THIRD_PERSON_CAMERA_NAME
