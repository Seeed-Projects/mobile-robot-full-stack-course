"""High-level trajectory-token inference: frames + instruction -> (H, 3) waypoints.

This is the public, websocket-free core. ``TrackingAgent`` wraps the inference
engine + a stateful frame buffer (``NavigationPolicy``) and turns the model's
trajectory-token output into waypoints: a ``<traj_k>`` id looked up in the
trajectory vocabulary (flat), or ``<act_l*>`` codes decoded through an RVQ bundle.

Waypoint convention: each ``(H, 3)`` row is robot-local
``[forward_m, lateral_m, yaw_rad]`` with +lateral = LEFT and +yaw = CCW, matching
Habitat's ``velocity_control``.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path

import numpy as np

from lightnav.eval_config import load_eval_config, resolve_asset_path
from lightnav.inference.config import InferenceConfig
from lightnav.inference.engine import build_engine
from lightnav.inference.policies import NavigationPolicy
from lightnav.velocity import is_stop_centroid
from lightnav.vln_utils import (
    DEFAULT_TRAJ_HORIZON,
    DEFAULT_TRAJ_K,
    decode_target_pos,
    parse_rvq_action_tokens,
    parse_traj_token,
    safe_parse_tpos_token,
)

logger = logging.getLogger("lightnav.tracking")

# RVQ "stop" tolerance: the stop action's per-level codeword sum only decodes to
# ~0 (k-means residual), unlike flat's exact-zero centroid[0], so treat a
# near-zero rvq decode as an explicit stop -> exact-zero waypoints.
_RVQ_STOP_ATOL = 5e-3


def load_centroids(traj_vocab_path: str | Path, K: int, horizon: int) -> np.ndarray:
    """Load ``centroids_whole_chunk_K{K}_h{horizon}.npy`` -> (K, H, 3) float32.

    A directory or a direct ``.npy`` path are both accepted.
    """
    p = Path(traj_vocab_path)
    if p.is_dir():
        p = p / f"centroids_whole_chunk_K{K}_h{horizon}.npy"
    if not p.exists():
        raise FileNotFoundError(
            f"centroids file not found: {p}. The trajectory vocabulary is shipped "
            f"alongside the checkpoint; point --traj_vocab_path at its directory."
        )
    arr = np.load(p)
    logger.info("Loaded centroids %s shape=%s", p, arr.shape)
    return arr


class TrackingAgent(NavigationPolicy):
    """NavigationPolicy extended to decode trajectory-token outputs into waypoint chunks.

    The buffer / video tensor / engine call all stay identical to the base
    policy; the only specialization is at decode time. Two action-tokenizer modes:

    * flat (default): parse the ``<traj_k>`` id and look up the centroid chunk.
    * rvq (opt-in; pass ``rvq_bundle``): parse the D coarse->fine ``<act_l*>``
      codes and decode them through the RVQ bundle (SE(2)-composed waypoints).

    Either way it returns (H, 3) waypoints in robot-local frame
    [forward, lateral, yaw].
    """

    def __init__(
        self,
        engine,
        centroids: np.ndarray | None = None,
        num_history_frames: int = 64,
        *,
        rvq_bundle=None,
        stop_atol: float = _RVQ_STOP_ATOL,
        client_id: str | None = None,
    ):
        super().__init__(engine, num_history_frames=num_history_frames, predict_horizon=1)
        if (centroids is None) == (rvq_bundle is None):
            raise ValueError(
                "TrackingAgent needs exactly one of `centroids` (flat) or `rvq_bundle` (rvq)"
            )
        self.rvq = rvq_bundle
        self.last_decode_info = None
        self._stop_atol = float(stop_atol)
        self.client_id = client_id
        if rvq_bundle is not None:
            self.centroids = None
            self.K = None
            self.H = int(rvq_bundle.horizon)
        else:
            if centroids.ndim != 3 or centroids.shape[-1] != 3:
                raise ValueError(f"centroids shape must be (K, H, 3), got {centroids.shape}")
            self.centroids = np.asarray(centroids, dtype=np.float32)
            self.K, self.H = int(centroids.shape[0]), int(centroids.shape[1])

    def decode_waypoints(self, text: str) -> tuple[np.ndarray, str]:
        """Parse model output text into (H, 3) waypoints. No engine/GPU.

        Newer checkpoints emit ``<tpos_K><traj_K>``; older ones ``<traj_K>`` only.
        ``safe_parse_tpos_token`` returns None when absent so both are accepted.
        Raises ``ValueError`` if the traj id is missing or out of vocab range.
        """
        tpos_id = safe_parse_tpos_token(text)
        if tpos_id is not None:
            info = decode_target_pos(tpos_id)
            if info["visible"]:
                logger.info(
                    "tpos id=%d  visible=1  az_bin=%d (%.2f rad)  d_bin=%d (%.2f m)",
                    tpos_id,
                    info["az_bin"],
                    info["az_center"],
                    info["d_bin"],
                    info["d_center"],
                )
            else:
                logger.info("tpos id=%d  visible=0 (target not seen)", tpos_id)

        if self.rvq is not None:
            self.last_decode_info = None
            strict = os.environ.get("LIGHTNAV_STRICT_RVQ", "0") == "1"
            if strict:
                from lightnav.serving.strict_decode import validate_decoded, validate_tokens

                info = validate_tokens(text)
            # RVQ: D <act_l*> tokens -> per-level codes -> codeword sum -> SE(2)
            # waypoints. Stop = the explicit stop tuple or a near-zero decode.
            codes = parse_rvq_action_tokens(text)
            levels = self.rvq.levels
            if len(codes) != len(levels):
                raise ValueError(
                    f"got {len(codes)} act levels, expected {len(levels)} from {text!r}"
                )
            for lvl, c in enumerate(codes):
                if not (0 <= c < levels[lvl]):
                    raise ValueError(f"rvq code {c} at level {lvl} out of range [0, {levels[lvl]})")
            if self.rvq.is_stop(codes):
                wp = np.zeros((self.H, 3), dtype=np.float32)
                if strict:
                    self.last_decode_info = validate_decoded(text, wp, info, explicit_stop=True)
                return wp, text
            wp = self.rvq.decode_waypoints(codes)
            if is_stop_centroid(wp, atol=self._stop_atol):
                wp = np.zeros((self.H, 3), dtype=np.float32)
            wp = np.asarray(wp, dtype=np.float32)
            if strict:
                self.last_decode_info = validate_decoded(text, wp, info, explicit_stop=False)
            return wp, text

        traj_id = parse_traj_token(text)
        if not (0 <= traj_id < self.K):
            raise ValueError(f"traj id {traj_id} out of vocab range [0, {self.K})")
        return self.centroids[traj_id].copy(), text

    def predict_waypoints(
        self, instruction: str, task_type: str = "tracking"
    ) -> tuple[np.ndarray, str, float]:
        """Run a single inference step on the current frame buffer.

        ``task_type`` selects the prompt family ("tracking" or "vlnce_traj") and is
        forwarded to ``engine.generate_from_frames``. Returns
        ``(waypoints, raw_text, latency_ms)``. Raises ``ValueError`` if the model
        output cannot be parsed as a trajectory token (caller can fall back).
        """
        if self._buffer_len == 0:
            raise RuntimeError("predict_waypoints called with empty observation buffer")

        video = self._get_video_tensor()
        text, latency_ms = self.engine.generate_from_frames(
            video,
            instruction,
            predict_horizon=1,
            frame_ids=self._history_frame_ids,
            task_type=task_type,
        )
        wp, raw = self.decode_waypoints(text)
        return wp, raw, latency_ms


_CENTROIDS_FILENAME = "centroids_whole_chunk_K{K}_h{H}.npy"


def _manifest_horizon(bundle_dir: Path) -> int | None:
    try:
        import json

        return int(json.loads((bundle_dir / "manifest.json").read_text()).get("horizon") or 0) or None
    except (OSError, ValueError):
        return None


def resolve_action_decoder_from_config(model_path: str | Path, task_key: str) -> dict | None:
    """Find the action decoder a checkpoint ships or references, without any CLI flag.

    ``task_key`` is the ``eval_config.json`` task entry to prefer (``"trackvla"`` for
    tracking, ``"vlnce"`` for navigation). Resolution order:

    1. the ``eval_config.json`` snapshot of that task (then of the other tasks):
       ``action_tokenizer.bundle_path`` (RVQ) or ``traj_vocab_path`` + ``traj_vocab_K``
       (flat), with relative paths taken relative to the checkpoint directory;
    2. sibling directories next to the checkpoint: ``action_tokenizer/<task_key>``,
       ``action_tokenizer`` (RVQ) or ``traj_vocab`` (flat).

    Returns ``{"method": "rvq", "bundle_path": Path, "horizon": int}`` or
    ``{"method": "flat", "traj_vocab_path": Path, "K": int, "horizon": int}``, or None.
    """
    model_path = str(model_path)
    saved = load_eval_config(model_path) or {}
    tasks_map = {k: v for k, v in (saved.get("tasks") or {}).items() if isinstance(v, dict)}
    ordered = ([tasks_map[task_key]] if task_key in tasks_map else []) + [
        t for k, t in tasks_map.items() if k != task_key
    ]
    for task in ordered:
        horizon = int(task.get("predict_horizon") or 0)
        at = task.get("action_tokenizer") or {}
        if at.get("method") == "rvq" and at.get("bundle_path"):
            bundle_dir = resolve_asset_path(model_path, at["bundle_path"])
            if (bundle_dir / "manifest.json").is_file():
                return {
                    "method": "rvq",
                    "bundle_path": bundle_dir,
                    "horizon": horizon or _manifest_horizon(bundle_dir) or DEFAULT_TRAJ_HORIZON,
                }
        elif task.get("traj_vocab_path") and task.get("traj_vocab_K") and horizon:
            vocab_dir = resolve_asset_path(model_path, task["traj_vocab_path"])
            k = int(task["traj_vocab_K"])
            if (vocab_dir / _CENTROIDS_FILENAME.format(K=k, H=horizon)).is_file():
                return {"method": "flat", "traj_vocab_path": vocab_dir, "K": k, "horizon": horizon}

    preferred = tasks_map.get(task_key) or (ordered[0] if ordered else {})
    horizon = int(preferred.get("predict_horizon") or 0)
    model_dir = Path(model_path).expanduser().resolve()
    for sibling in (model_dir / "action_tokenizer" / task_key, model_dir / "action_tokenizer"):
        if (sibling / "manifest.json").is_file():
            return {
                "method": "rvq",
                "bundle_path": sibling,
                "horizon": horizon or _manifest_horizon(sibling) or DEFAULT_TRAJ_HORIZON,
            }
    k = int(preferred.get("traj_vocab_K") or DEFAULT_TRAJ_K)
    h = horizon or DEFAULT_TRAJ_HORIZON
    vocab_dir = model_dir / "traj_vocab"
    if (vocab_dir / _CENTROIDS_FILENAME.format(K=k, H=h)).is_file():
        return {"method": "flat", "traj_vocab_path": vocab_dir, "K": k, "horizon": h}
    return None


def build_tracking_agent(
    model_path: str,
    traj_vocab_path: str | Path | None = None,
    K: int | None = None,
    horizon: int | None = None,
    *,
    backend: str = "vllm_local",
    num_history_frames: int | None = None,
    max_new_tokens: int = 8,
    device: str = "cuda",
    gpu_memory_utilization: float = 0.85,
    pool_spatial: int | None = None,
    action_tokenizer_bundle: str | Path | None = None,
    task_key: str = "trackvla",
    aspect_mode: str = "stretch",
) -> TrackingAgent:
    """One-call constructor: checkpoint + trajectory vocab -> ready TrackingAgent.

    ``backend`` is ``"vllm_local"`` (in-process vLLM, fastest) or ``"hf"``
    (transformers.generate). Processing params (video_size / pooling / slowfast
    tiers / history window) are auto-read from the checkpoint's eval_config.json.

    ``action_tokenizer_bundle`` opts into RVQ decoding: point it at an RVQ bundle
    dir (``manifest.json`` + codebooks) and the agent decodes the checkpoint's
    ``<act_l*>`` output instead of ``<traj_k>``. ``traj_vocab_path`` / ``K`` are
    then ignored (``horizon`` is still used, to validate the bundle), and
    ``max_new_tokens`` is raised to fit the D level tokens if needed.
    """
    if action_tokenizer_bundle is not None and traj_vocab_path:
        raise ValueError("pass only one of traj_vocab_path (flat) and action_tokenizer_bundle (rvq)")
    if action_tokenizer_bundle is None and not traj_vocab_path:
        # No decoder given: use what the checkpoint ships / references (eval_config.json
        # snapshot or a sibling action_tokenizer/ / traj_vocab/ directory).
        resolved = resolve_action_decoder_from_config(model_path, task_key)
        if resolved is None:
            raise FileNotFoundError(
                "no action decoder found for this checkpoint: pass traj_vocab_path (+K, horizon) "
                "or action_tokenizer_bundle, or ship them next to the weights as referenced by "
                "eval_config.json"
            )
        logger.info("Action decoder resolved from the checkpoint: %s", resolved)
        if resolved["method"] == "rvq":
            action_tokenizer_bundle = resolved["bundle_path"]
        else:
            traj_vocab_path, K = resolved["traj_vocab_path"], resolved["K"]
        horizon = horizon or int(resolved["horizon"])
    rvq_bundle = None
    if action_tokenizer_bundle is not None:
        from lightnav.traj_vocab import load_rvq_bundle

        horizon = horizon or _manifest_horizon(Path(action_tokenizer_bundle)) or DEFAULT_TRAJ_HORIZON
        rvq_bundle = load_rvq_bundle(
            Path(action_tokenizer_bundle), int(horizon), num_frames=0, load_cluster_ids=False
        )
        # rvq emits 1 tpos + D level tokens + eos; the flat default (~8) may be too small.
        max_new_tokens = max(max_new_tokens, 1 + len(rvq_bundle.levels) + 1)

    cfg = InferenceConfig(
        model_path=model_path,
        backend=backend,
        max_new_tokens=max_new_tokens,
        device=device,
        gpu_memory_utilization=gpu_memory_utilization,
        aspect_mode=aspect_mode,
    )
    if pool_spatial is not None:
        cfg.pool_spatial = pool_spatial
    # The engine's processing params come from the SAME eval_config.json task entry the
    # decoder came from: a checkpoint whose tasks.vlnce and tasks.trackvla differ (history
    # window, slowfast tiers, video_fps) would otherwise be served the wrong one -- e.g. a
    # VLN-only checkpoint used to get INFERENCE_FALLBACK_DEFAULTS (num_history_frames=16,
    # no slowfast tiers) because the engine was always built from "trackvla".
    # A checkpoint that does not carry the requested task keeps whichever entry it has:
    # resolve_inference_params has no cross-task fallback, so the alternative is silently
    # dropping to those defaults.
    engine_task = task_key
    available_tasks = (load_eval_config(model_path) or {}).get("tasks") or {}
    if available_tasks and engine_task not in available_tasks:
        engine_task = next(iter(available_tasks))
        logger.info(
            "eval_config.json has no '%s' task entry; building the engine from '%s'",
            task_key,
            engine_task,
        )
    engine, bundle = build_engine(cfg, task_type=engine_task, max_new_tokens=max_new_tokens)
    # None (the default) = the checkpoint's own history window, as lightnav-serve does;
    # an explicit value overrides it.
    num_history_frames = int(num_history_frames or bundle.num_history_frames)
    if rvq_bundle is not None:
        return TrackingAgent(engine, num_history_frames=num_history_frames, rvq_bundle=rvq_bundle)
    centroids = load_centroids(
        traj_vocab_path, int(K or DEFAULT_TRAJ_K), int(horizon or DEFAULT_TRAJ_HORIZON)
    )
    return TrackingAgent(engine, centroids, num_history_frames=num_history_frames)
