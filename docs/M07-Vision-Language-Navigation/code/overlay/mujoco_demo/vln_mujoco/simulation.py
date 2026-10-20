from __future__ import annotations

import io
import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass

import mujoco
import numpy as np
from PIL import Image

from .collision import CollisionMonitor
from .model import CAMERA_HEIGHT, CAMERA_WIDTH, PHYSICS_DT
from .mpc import TRACK_V_MAX, W_MAX
from .recording import RunRecorder
from .robots.base import RobotBackend
from .robots.turtlebot import TurtleBotBackend

COMMAND_TIMEOUT_S = 0.35
FRAME_PERIOD_S = 0.05


@dataclass(frozen=True)
class CameraFrame:
    stamp_ns: int
    rgb: bytes
    jpeg: bytes
    pose: tuple[float, float, float]
    monotonic_ns: int = 0
    camera_position: tuple[float, ...] = ()
    camera_orientation: tuple[float, ...] = ()
    scene_epoch: int = 0
    camera_fovy: float = 0.0
    root_position: tuple[float, ...] = ()


class Simulation:
    def __init__(self, robot: RobotBackend | None = None, *, recorder: RunRecorder | None = None) -> None:
        self.robot = robot or TurtleBotBackend()
        self.model = self.robot.model
        self.data = self.robot.data
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._command = (0.0, 0.0)
        self._command_at = 0.0
        self._frame: CameraFrame | None = None
        self._third_person_frame: CameraFrame | None = None
        self._frame_count = 0
        self._started_at = time.monotonic()
        self.recorder = recorder
        self._error = ""
        self._physics_step_count = 0
        self._last_step_ns: int | None = None
        self._last_physics_ms: float | None = None
        self._physics_max_lag_ms = 0.0
        self._lag_reset_count = 0
        self._watchdog_timeout_count = 0
        self._last_watchdog_command_at = 0.0
        self._reset_count = 0
        self._scene_epoch = 0
        self._motion_trace = None
        self._motion_trace_overflow = False
        self._physics_window = deque(maxlen=1000)
        self._stationary_since_ns = None
        self._physics_safety_error = ""
        self._collision = CollisionMonitor(self.model, self.data, getattr(self.robot, "_base_joint", None))

    @property
    def robot_name(self) -> str:
        return self.robot.name

    def _on_frame_published(self, frame: CameraFrame) -> None:
        """Subclass hook; called by the physical renderer at actual publication."""

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._error = ""
        self._thread = threading.Thread(target=self._run, name="mujoco", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=5.0)
            if thread.is_alive():
                self._error = "simulation thread did not stop within 5 seconds"
                raise RuntimeError(self._error)
        if self.recorder is not None:
            self.recorder.record_runtime(self.snapshot(), event="stop")
            self.recorder.close()
        self._thread = None

    def reset(self, yaw: float | None = None) -> None:
        with self._lock:
            self.robot.reset()
            if yaw is not None:
                if not math.isfinite(yaw):
                    raise ValueError("initial yaw must be finite")
                base = self.robot._base_qpos
                self.data.qpos[base + 3:base + 7] = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
                # TurtleBot caches its heading; MicroDuck's _yaw is a quaternion helper.
                if isinstance(getattr(self.robot, "_yaw", None), (int, float)):
                    self.robot._yaw = yaw
                self.data.qvel.fill(0)
                mujoco.mj_forward(self.model, self.data)
            self._command = (0.0, 0.0)
            self._command_at = 0.0
            self._reset_count += 1
            self._scene_epoch += 1
            self._frame = self._third_person_frame = None
            self._motion_trace = None
            self._physics_window.clear()
            self._stationary_since_ns = None
            self._physics_safety_error = ""
            self._collision.reset()
            if self.recorder is not None:
                self.recorder.record_runtime(self.snapshot(), event="reset")

    def set_velocity(self, linear: float, angular: float) -> None:
        # Physical sanity bound only; per-mode VLN limits live in the MPC.
        linear = max(-TRACK_V_MAX, min(TRACK_V_MAX, float(linear)))
        angular = max(-W_MAX, min(W_MAX, float(angular)))
        with self._lock:
            self._command = (linear, angular)
            self._command_at = time.monotonic()

    def frame(self) -> CameraFrame | None:
        with self._lock:
            return self._frame

    def third_person_frame(self) -> CameraFrame | None:
        with self._lock:
            return self._third_person_frame

    def physics_window(self) -> dict:
        with self._lock:
            return {"scene_epoch": self._scene_epoch, "error": self._physics_safety_error,
                    "samples": list(self._physics_window)}

    @staticmethod
    def _encode_frame(
        stamp_ns: int,
        rgb: np.ndarray,
        pose: tuple[float, float, float],
        monotonic_ns: int = 0,
        camera_position: tuple[float, ...] = (),
        camera_orientation: tuple[float, ...] = (),
        scene_epoch: int = 0,
        camera_fovy: float = 0.0,
        root_position: tuple[float, ...] = (),
    ) -> CameraFrame:
        output = io.BytesIO()
        Image.fromarray(rgb, "RGB").save(output, format="JPEG", quality=82)
        return CameraFrame(stamp_ns, rgb.tobytes(), output.getvalue(), pose, monotonic_ns,
                           camera_position, camera_orientation, scene_epoch, camera_fovy, root_position)

    def begin_motion_guard(self) -> None:
        with self._lock:
            self._motion_trace = deque(maxlen=15000)
            self._motion_trace_overflow = False

    def end_motion_guard(self) -> None:
        with self._lock:
            self._motion_trace = None

    def motion_guard_result(self, frame: CameraFrame) -> dict:
        with self._lock:
            samples = [sample for sample in (self._motion_trace or ()) if sample[0] >= frame.monotonic_ns]
            if self._motion_trace_overflow or not samples or frame.scene_epoch != self._scene_epoch:
                return {"valid": False, "error": "Missing, overflowing or wrong-epoch motion trace"}
            if (len(frame.root_position) != 3 or any(len(sample) != 5 for sample in samples)
                    or len(frame.camera_position) != 3 or len(frame.camera_orientation) != 9):
                return {"valid": False, "error": "Missing actual capture root/camera extrinsics"}
            roots = np.asarray([sample[4] for sample in samples])
            poses = np.asarray([sample[1] for sample in samples])
            positions = np.asarray([sample[2] for sample in samples])
            rotations = np.asarray([sample[3] for sample in samples]).reshape(-1, 3, 3)
            reference = np.asarray(frame.camera_orientation).reshape(3, 3)
            relative = np.einsum("ij,njk->nik", reference.T, rotations)
            angles = np.arccos(np.clip((np.trace(relative, axis1=1, axis2=2) - 1) / 2, -1, 1))
            yaw_delta = poses[:, 2] - frame.pose[2]
            maxima = {"root_xyz_m": float(np.linalg.norm(roots - frame.root_position, axis=1).max()),
                      "root_xy_m": float(np.linalg.norm(poses[:, :2] - frame.pose[:2], axis=1).max()),
                      "root_yaw_rad": float(np.abs(np.arctan2(np.sin(yaw_delta), np.cos(yaw_delta))).max()),
                      "camera_translation_m": float(np.linalg.norm(positions - frame.camera_position, axis=1).max()),
                      "camera_rotation_rad": float(angles.max()), "physics_samples": len(samples)}
            valid = (maxima["root_xyz_m"] <= 0.03 and maxima["root_yaw_rad"] <= 0.05
                     and maxima["camera_translation_m"] <= 0.02 and maxima["camera_rotation_rad"] <= 0.05)
            return {"valid": valid, **maxima}

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            state = self.robot.state()
            x, y, z, yaw = state.pose
            linear, angular = state.velocity
            base_dof = getattr(self.robot, "_base_dof", None)
            planar = float(np.linalg.norm(self.data.qvel[base_dof:base_dof + 2])) if base_dof is not None else abs(linear)
            command = self._command
            frame = self._frame
            elapsed = max(time.monotonic() - self._started_at, 1e-6)
            snapshot = {
                "pose": {"x": x, "y": y, "z": z, "yaw": yaw},
                "velocity": {"linear": linear, "angular": angular, "planar": planar},
                "scene_epoch": self._scene_epoch,
                "stationary_since_monotonic_ns": str(self._stationary_since_ns) if self._stationary_since_ns is not None else None,
                "collision": self._collision.first or {"detected": False},
                "command": {"linear": command[0], "angular": command[1]},
                "sim_time": float(self.data.time),
                "thread": {
                    "alive": self._thread is not None and self._thread.is_alive(),
                    "error": self._error,
                },
                "timing": {
                    "physics_step_count": self._physics_step_count,
                    "last_step_monotonic_ns": self._last_step_ns,
                    "last_physics_ms": self._last_physics_ms,
                    "physics_max_lag_ms": self._physics_max_lag_ms,
                    "lag_reset_count": self._lag_reset_count,
                    "watchdog_timeout_count": self._watchdog_timeout_count,
                    "reset_count": self._reset_count,
                },
                "camera": {
                    "ready": frame is not None,
                    "stamp_ns": frame.stamp_ns if frame is not None else None,
                    "fps": self._frame_count / elapsed,
                    "frame_count": self._frame_count,
                    "monotonic_ns": frame.monotonic_ns if frame is not None else None,
                },
            }
            if state.telemetry:
                snapshot["backend"] = state.telemetry
            if self.recorder is not None:
                snapshot["recording"] = self.recorder.snapshot()
            snapshot["physics_safety"] = {"valid": not self._physics_safety_error,
                                           "error": self._physics_safety_error}
            return snapshot

    def _advance(self, now: float) -> None:
        command = self._command
        if self._command_at and now - self._command_at > COMMAND_TIMEOUT_S:
            command = (0.0, 0.0)
            self._command = command
            if self._last_watchdog_command_at != self._command_at:
                self._watchdog_timeout_count += 1
                self._last_watchdog_command_at = self._command_at
        self.robot.step(command)
        if self._collision.check():
            self._command = (0.0, 0.0)
        state = self.robot.state()
        telemetry = state.telemetry or {}
        finite = bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all())
        if not finite or telemetry.get("trunk_height_m", 1) < 0.065 or telemetry.get("tilt_rad", 0) > 0.9:
            self._physics_safety_error = self._physics_safety_error or "Non-finite state or fall observed at physics step"
        dof = getattr(self.robot, "_base_dof", None)
        planar = float(np.linalg.norm(self.data.qvel[dof:dof + 2])) if dof is not None else abs(state.velocity[0])
        sample_ns = time.monotonic_ns()
        stable = finite and planar <= 0.02 and abs(state.velocity[1]) <= 0.05
        if self._last_step_ns is not None and sample_ns - self._last_step_ns > 100_000_000:
            self._stationary_since_ns = None
        if not stable:
            self._stationary_since_ns = None
        elif self._stationary_since_ns is None:
            self._stationary_since_ns = sample_ns
        self._physics_window.append({"monotonic_ns": str(sample_ns),
            "step": self._physics_step_count, "pose": state.pose if finite else None,
            "planar_speed": planar if finite else None, "angular_speed": state.velocity[1] if finite else None,
            "valid": not self._physics_safety_error and self._collision.first is None})
        if self._motion_trace is not None:
            camera_id = self.model.camera(self.robot.camera_name).id
            x, y, z, yaw = state.pose
            if len(self._motion_trace) == self._motion_trace.maxlen:
                self._motion_trace_overflow = True
            self._motion_trace.append((time.monotonic_ns(), (x, y, yaw),
                                       tuple(self.data.cam_xpos[camera_id]), tuple(self.data.cam_xmat[camera_id]), (x, y, z)))
        self._physics_step_count += 1

    def _run(self) -> None:
        renderer = None
        try:
            renderer = mujoco.Renderer(
                self.model,
                height=CAMERA_HEIGHT,
                width=CAMERA_WIDTH,
            )
            third_person_camera = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(third_person_camera)
            next_step = time.monotonic()
            next_frame = next_step
            while not self._stop.is_set():
                now = time.monotonic()
                if now < next_step:
                    time.sleep(min(next_step - now, 0.002))
                    continue
                with self._lock:
                    started_ns = time.monotonic_ns()
                    lag_ms = max(0.0, now - next_step) * 1000.0
                    self._physics_max_lag_ms = max(self._physics_max_lag_ms, lag_ms)
                    self._advance(now)
                    self._last_step_ns = time.monotonic_ns()
                    self._last_physics_ms = (self._last_step_ns - started_ns) / 1e6
                next_step += PHYSICS_DT
                if now - next_step > 0.25:
                    next_step = now
                    with self._lock:
                        self._lag_reset_count += 1
                if now >= next_frame:
                    with self._lock:
                        renderer.update_scene(
                            self.data,
                            camera=self.robot.camera_name,
                        )
                        rgb = np.asarray(renderer.render(), dtype=np.uint8).copy()
                        render_camera = self.robot.third_person_camera(
                            third_person_camera
                        )
                        renderer.update_scene(self.data, camera=render_camera)
                        third_person_rgb = np.asarray(
                            renderer.render(), dtype=np.uint8
                        ).copy()
                        x, y, z, yaw = self.robot.state().pose
                        capture_pose = (x, y, yaw)
                        camera_id = self.model.camera(self.robot.camera_name).id
                        camera_position = tuple(float(value) for value in self.data.cam_xpos[camera_id])
                        camera_orientation = tuple(float(value) for value in self.data.cam_xmat[camera_id])
                        epoch = self._scene_epoch
                        stamp_ns = time.time_ns()
                        monotonic_ns = time.monotonic_ns()
                    frame = self._encode_frame(stamp_ns, rgb, capture_pose, monotonic_ns,
                                               camera_position, camera_orientation, epoch,
                                               float(self.model.cam_fovy[camera_id]), (x, y, z))
                    third_person_frame = self._encode_frame(
                        stamp_ns,
                        third_person_rgb,
                        capture_pose,
                        monotonic_ns,
                    )
                    with self._lock:
                        if epoch != self._scene_epoch:
                            continue
                        self._frame = frame
                        self._third_person_frame = third_person_frame
                        self._frame_count += 1
                        self._on_frame_published(frame)
                        if self.recorder is not None:
                            self.recorder.record_runtime(self.snapshot())
                    next_frame = now + FRAME_PERIOD_S
        except Exception as exc:
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
                self._command = (0.0, 0.0)
                self._stop.set()
                if self.recorder is not None:
                    try:
                        self.recorder.record_runtime(self.snapshot(), event="error")
                    except RuntimeError:
                        pass
            logging.getLogger("vln_mujoco.simulation").exception("simulation thread failed")
        finally:
            if renderer is not None:
                renderer.close()
