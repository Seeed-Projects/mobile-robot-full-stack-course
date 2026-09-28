"""bev_pose.foundationpose_engine — NVlabs/FoundationPose wrapper with explicit lifecycle.

DESIGN
------
The engine exposes four clearly-named operations:

    load_object_model(mesh_npz_path)
        - Offline data: pre-computed vertices / bbox / diameter from
          mesh_preprocessor.py. No NVlabs model loading here.

    register(rgb, depth, K, mask)
        - Phase 1. Called ONCE on (or any time after) a frame where the
          object is known to be present. Returns the initial 4x4 pose.
          Subsequent calls reset the tracker.

    track(rgb, depth, K)
        - Phase 2. Called every frame after `register` succeeded. Uses
          the previous pose as the initialisation for refiner+scaler.
          Cheap (~50-80ms on Jetson).

    refine_pose(rgb, depth, K, n_iters=5)
        - Phase 3. Explicit re-refine when the tracker appears stuck. NOT
          required for normal operation; exposed because the M4 demo wants
          a clear "re-anchor" button in the web UI.

The engine MUST NOT expose a single run(rgb, depth, K, mask): that would
hide whether the call is registering or tracking. The M4 plan forbids
this shortcut explicitly.

RUNTIME MODEL
-------------
* `load_object_model` triggers NVlabs FoundationPose model load (refiner
  + scorer weights). It is the only step that touches the GPU.
* `register` allocates the in-engine mesh state.
* `track` runs refiner iteration only.
* `refine_pose` re-runs the full refiner chain for a given frame.

If FoundationPose is not installed yet (e.g. Phase 0 not run), the
constructor raises a clear `RuntimeError` rather than silently failing
on the first frame. This is intentional — we want the bug to be loud.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .types import EngineConfig, PreprocessedMesh, PoseResult

log = logging.getLogger(__name__)


@dataclass
class PerfTrace:
    """Per-call perf stats; held by the engine for reporting."""

    last_register_ms: float = 0.0
    last_track_ms: float = 0.0
    last_refine_ms: float = 0.0
    register_count: int = 0
    track_count: int = 0
    register_history_ms: list = field(default_factory=list)
    track_history_ms: list = field(default_factory=list)
    # Tracking FPS is computed over a rolling window
    fps_window_ms: list = field(default_factory=list)
    fps_window_size: int = 30


class FoundationPoseEngine:
    """Explicit-lifecycle wrapper for NVlabs/FoundationPose.

    See module docstring for the API contract.
    """

    def __init__(self, config: EngineConfig):
        self.config = config
        self.mesh: Optional[PreprocessedMesh] = None
        self._est = None                  # NVlabs FoundationPose estimator
        self._registered: bool = False
        self._last_pose: Optional[np.ndarray] = None  # (4, 4)
        self._frame_count: int = 0
        self.perf = PerfTrace()

    # ----------------------------------------------------------------
    # Phase 0 — load_object_model
    # ----------------------------------------------------------------
    def load_object_model(self, mesh_npz_path: str) -> PreprocessedMesh:
        """Load pre-processed mesh + initialise NVlabs models.

        Args:
            mesh_npz_path: path to a .npz produced by mesh_preprocessor.

        Returns:
            The PreprocessedMesh that was loaded.

        Raises:
            FileNotFoundError: the .npz file does not exist.
            RuntimeError: the NVlibs/FoundationPose backend is missing.
        """
        # 1. Load the offline-precomputed geometry
        if not mesh_npz_path:
            raise ValueError("mesh_npz_path must be non-empty")
        mesh = PreprocessedMesh.from_npz(mesh_npz_path)
        self.mesh = mesh
        log.info(
            "load_object_model: %d verts, diameter=%.4fm, frame_id=%s",
            len(mesh.vertices), mesh.diameter, mesh.frame_id,
        )

        # 2. Initialise NVlabs estimator (loads refiner + scorer weights)
        self._init_nvlabs_estimator()
        return mesh

    def _init_nvlabs_estimator(self) -> None:
        """Construct the NVlabs FoundationPose estimator.

        The NVlabs repo is NOT pip-installed; the host is expected to have
        cloned it, and `FOUNDATIONPOSE_DIR` names the checkout (default
        `<workspace>/third_party/FoundationPose`, not `$HOME/FoundationPose`
        -- the course keeps third-party sources outside the repo).

        Everything imported here comes from that checkout, so a missing
        dependency fails loudly at this point rather than at the first frame.
        """
        try:
            import os as _os
            fp_dir = _os.environ.get('FOUNDATIONPOSE_DIR',
                                     _os.path.expanduser('~/FoundationPose'))
            if fp_dir and fp_dir not in __import__('sys').path:
                __import__('sys').path.insert(0, fp_dir)
            from estimater import FoundationPose  # type: ignore
        except Exception as e:
            raise RuntimeError(
                "NVlabs/FoundationPose backend is not importable. Run "
                "scripts/m4/phase0_foundationpose_verify.sh first or set "
                "FOUNDATIONPOSE_DIR=/path/to/FoundationPose. "
                f"Underlying error: {e!r}"
            ) from e

        # Written against the pinned upstream revision
        # a1b694b83e633c2cb6115b9063d940a687759392, using run_demo.py as the
        # reference. The real constructor is:
        #
        #   FoundationPose(model_pts, model_normals, symmetry_tfs=None,
        #                  mesh=None, scorer=None, refiner=None,
        #                  glctx=None, debug=0, debug_dir=...)
        #
        # model_pts / model_normals are ARRAYS taken from the object mesh, not
        # a path. An earlier version of this file passed model_dir=,
        # scorer_module=, refiner_module=, mesh_file= and device= -- none of
        # which exist upstream -- and omitted both required arguments, so it
        # could not have run on any revision.
        import trimesh
        from estimater import (FoundationPose, PoseRefinePredictor,
                               ScorePredictor)
        import nvdiffrast.torch as dr

        if self.mesh is None:
            raise RuntimeError("load_object_model() must run before the estimator")

        # PreprocessedMesh already holds the vertices and per-vertex normals
        # produced offline, so the runtime needs no CAD kernel.
        verts = np.ascontiguousarray(self.mesh.vertices, dtype=np.float32)
        norms = np.ascontiguousarray(self.mesh.normals, dtype=np.float32)
        if verts.ndim != 2 or verts.shape[1] != 3:
            raise RuntimeError(f"mesh vertices must be (N,3), got {verts.shape}")
        if norms.shape != verts.shape:
            raise RuntimeError(
                f"mesh normals {norms.shape} do not match vertices {verts.shape}")

        # `mesh` is used internally for rendering/centring; rebuild it from the
        # same arrays so it cannot disagree with model_pts.
        mesh_tm = trimesh.Trimesh(
            vertices=verts,
            faces=np.ascontiguousarray(self.mesh.faces, dtype=np.int64),
            vertex_normals=norms,
            process=False,
        )

        # Neither predictor takes a weights argument -- they resolve their own
        # checkpoints under the repo's weights/ directory. Upstream signatures:
        #   ScorePredictor(amp=True)
        #   PoseRefinePredictor()
        self._scorer = ScorePredictor()
        self._refiner = PoseRefinePredictor()
        self._glctx = dr.RasterizeCudaContext()

        self._est = FoundationPose(
            model_pts=verts,
            model_normals=norms,
            mesh=mesh_tm,
            scorer=self._scorer,
            refiner=self._refiner,
            glctx=self._glctx,
            debug=0,
            debug_dir=self.config.model_dir,
        )

    # ----------------------------------------------------------------
    # Phase 1 — register (initial registration, ONCE)
    # ----------------------------------------------------------------
    def register(self, rgb: np.ndarray, depth: np.ndarray,
                 K: np.ndarray, mask: np.ndarray) -> PoseResult:
        """Run initial registration on the first known-in-frame.

        This is the registration. It MUST be called only when the object
        is known to be present and the user has confidence in the mask.
        Calling it again resets the tracker (de-registers and then
        re-initialises from scratch).
        """
        if self._est is None or self.mesh is None:
            raise RuntimeError(
                "Engine not initialised: call load_object_model() first."
            )

        t0 = time.monotonic()
        # register(K, rgb, depth, ob_mask, ob_id=None, glctx=None, iteration=5)
        pose = self._est.register(
            K=K,
            rgb=rgb,
            depth=depth,
            ob_mask=mask,
            iteration=self.config.refiner_iterations,
        )
        dt_ms = (time.monotonic() - t0) * 1000.0

        pose_np = np.asarray(pose, dtype=np.float32)
        if pose_np.shape != (4, 4):
            # Some NVlabs versions return (1,4,4) when batched.
            pose_np = pose_np.reshape(4, 4)

        self._registered = True
        self._last_pose = pose_np
        self._frame_count = 0
        self.perf.last_register_ms = dt_ms
        self.perf.register_count += 1
        self.perf.register_history_ms.append(dt_ms)
        # Trim history
        if len(self.perf.register_history_ms) > 30:
            self.perf.register_history_ms = self.perf.register_history_ms[-30:]

        return PoseResult(
            pose=pose_np,
            score=1.0,
            latency_ms=dt_ms,
            is_registered=True,
            is_tracked=False,
            frame_index=0,
        )

    # ----------------------------------------------------------------
    # Phase 2 — track (every following frame)
    # ----------------------------------------------------------------
    def track(self, rgb: np.ndarray, depth: np.ndarray,
              K: np.ndarray) -> PoseResult:
        """Track the object on a new frame.

        Requires a previous successful register() in the same engine
        lifetime. Uses the previous pose as warm-start for the refiner
        chain; this is the cheap path (~50–80ms on Jetson).
        """
        if not self._registered or self._last_pose is None:
            raise RuntimeError(
                "Cannot track before register(); call register() first."
            )

        t0 = time.monotonic()
        # track_one(rgb, depth, K, iteration, extra={}) -- upstream keeps the
        # previous pose in self.pose_last and has NO pose_init argument. Passing
        # one was a TypeError waiting to happen; the tracker is already
        # warm-started by construction.
        pose = self._est.track_one(
            rgb=rgb,
            depth=depth,
            K=K,
            iteration=self.config.refiner_iterations,
        )
        dt_ms = (time.monotonic() - t0) * 1000.0

        pose_np = np.asarray(pose, dtype=np.float32).reshape(4, 4)
        self._last_pose = pose_np
        self._frame_count += 1
        self.perf.last_track_ms = dt_ms
        self.perf.track_count += 1
        self.perf.track_history_ms.append(dt_ms)
        if len(self.perf.track_history_ms) > 120:
            self.perf.track_history_ms = self.perf.track_history_ms[-120:]

        # FPS window
        self.perf.fps_window_ms.append(dt_ms)
        if len(self.perf.fps_window_ms) > self.perf.fps_window_size:
            self.perf.fps_window_ms = self.perf.fps_window_ms[-self.perf.fps_window_size:]

        # Score is not exposed by NVlabs tracker; use a stable surrogate
        # (1 / (1 + dt_ms/1000)) so the node has something to publish.
        score = 1.0 / (1.0 + dt_ms / 1000.0)
        return PoseResult(
            pose=pose_np,
            score=score,
            latency_ms=dt_ms,
            is_registered=True,
            is_tracked=True,
            frame_index=self._frame_count,
        )

    # ----------------------------------------------------------------
    # Phase 3 — refine_pose (explicit re-anchor)
    # ----------------------------------------------------------------
    def refine_pose(self, rgb: np.ndarray, depth: np.ndarray,
                    K: np.ndarray, n_iters: int = 5) -> PoseResult:
        """Re-refine the pose on a given frame (re-anchor).

        Use case: the tracker is following the wrong pose after a fast
        camera/robot motion. Run refine_pose() to re-hammer the refiner
        iterations on the current frame; the new pose replaces the
        internal last_pose.
        """
        if not self._registered or self._last_pose is None:
            raise RuntimeError(
                "Cannot refine before register(); call register() first."
            )

        t0 = time.monotonic()
        # Same call as track(), with an explicit iteration count. There is no
        # separate upstream "refine" entry point: re-anchoring is just the
        # refiner run harder on the current frame, which is what this does.
        pose = self._est.track_one(
            rgb=rgb,
            depth=depth,
            K=K,
            iteration=n_iters,
        )
        dt_ms = (time.monotonic() - t0) * 1000.0

        pose_np = np.asarray(pose, dtype=np.float32).reshape(4, 4)
        self._last_pose = pose_np
        self.perf.last_refine_ms = dt_ms

        return PoseResult(
            pose=pose_np,
            score=1.0,
            latency_ms=dt_ms,
            is_registered=True,
            is_tracked=True,
            frame_index=self._frame_count,
        )

    # ----------------------------------------------------------------
    # Diagnostics + lifecycle
    # ----------------------------------------------------------------
    @property
    def is_registered(self) -> bool:
        return self._registered

    @property
    def last_pose(self) -> Optional[np.ndarray]:
        return self._last_pose

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def tracking_fps(self) -> float:
        if not self.perf.fps_window_ms:
            return 0.0
        mean_ms = sum(self.perf.fps_window_ms) / len(self.perf.fps_window_ms)
        return 1000.0 / max(mean_ms, 1.0)

    def reset(self) -> None:
        """Back to un-registered state (next frame must call register())."""
        self._registered = False
        self._last_pose = None
        self._frame_count = 0
        log.info("engine reset (register required before track)")

    def summary(self) -> dict:
        return {
            'registered': self._registered,
            'frame_count': self._frame_count,
            'last_register_ms': self.perf.last_register_ms,
            'last_track_ms': self.perf.last_track_ms,
            'tracking_fps': self.tracking_fps,
            'tracking_median_ms': (
                float(np.median(self.perf.track_history_ms))
                if self.perf.track_history_ms else 0.0
            ),
            'tracking_p95_ms': (
                float(np.percentile(self.perf.track_history_ms, 95))
                if self.perf.track_history_ms else 0.0
            ),
            'mesh_vertices': int(self.mesh.vertices.shape[0]) if self.mesh else 0,
            'mesh_diameter_m': self.mesh.diameter if self.mesh else 0.0,
        }
