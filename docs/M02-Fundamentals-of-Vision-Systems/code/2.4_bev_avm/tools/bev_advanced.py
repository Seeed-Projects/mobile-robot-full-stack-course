#!/usr/bin/env python3
"""Advanced, testable helpers for multi-position AVM calibration and rendering."""
from __future__ import annotations

import hashlib
import itertools
import json
import os
import shutil
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


SESSION_VERSION = 1
# v3 stores geometric sector seams and rejects content-aware GraphCut LUTs.
# Older files are retained as backups rather than silently reused.
SEAM_LUT_VERSION = 3


def owner_and_weights(supports, scores, directions, transition_px, allowed=None,
                      hard_mask=None):
    """Choose only positively-scored directional candidates; never fill unknown."""
    stacked = np.stack([
        scores[d] if allowed is None else scores[d] * allowed[d]
        for d in directions], axis=0)
    owner = np.argmax(stacked, axis=0).astype(np.int16)
    valid = np.max(stacked, axis=0) > 0
    owner[~valid] = -1
    labels = {d: (owner == index).astype(np.float32)
              for index, d in enumerate(directions)}
    if transition_px > 0:
        sigma = max(0.35, float(transition_px) / 3.0)
        labels = {d: cv2.GaussianBlur(value, (0, 0), sigmaX=sigma,
                                      sigmaY=sigma)
                  for d, value in labels.items()}
    weights = {d: labels[d] * supports[d].astype(np.float32)
               for d in directions}
    total = np.maximum(sum(weights.values()), 1e-6)
    for d in directions:
        weights[d] /= total
        weights[d][~valid] = 0.0
    # A hard mask, if present, disables blending on selected pixels.  Geometric
    # seams pass an empty mask so the page-controlled transition width remains.
    if hard_mask is not None:
        hard_mask = np.asarray(hard_mask, bool) & valid
        for index, d in enumerate(directions):
            weights[d][hard_mask] = (owner[hard_mask] == index).astype(np.float32)
    return owner, weights


def angle_weight(direction, gx, gy, cam_axis, inner_deg=50.0, outer_deg=75.0):
    """Soft-clip a camera to its ground sector so H cannot extrapolate across-view."""
    ax, ay = cam_axis[direction]
    norm = np.hypot(gx, gy)
    dot = (gx * ax + gy * ay) / np.maximum(norm, 1e-6)
    soft = np.clip((dot - np.cos(np.deg2rad(outer_deg))) /
                   (np.cos(np.deg2rad(inner_deg)) - np.cos(np.deg2rad(outer_deg))),
                   0.0, 1.0)
    return soft.astype(np.float32)


def sector_scores(supports, directions, cam_axis, gx, gy):
    return {d: angle_weight(d, gx, gy, cam_axis) * np.asarray(supports[d], np.float32)
            for d in directions}


def geometric_seam(supports, scores, directions, transition_px):
    """Fixed sector ownership with a page-controlled blend and no content freeze."""
    allowed = geometric_seam_allow(supports, directions)
    hard_mask = empty_hard_seam_mask(supports, directions)
    owner, weights = owner_and_weights(
        supports, scores, directions, transition_px, allowed, hard_mask)
    return allowed, hard_mask, owner, weights


def geometric_seam_allow(supports, directions):
    """Keep every trustworthy observation; do not freeze content-aware cuts."""
    return {d: np.asarray(supports[d], bool).copy() for d in directions}


def empty_hard_seam_mask(supports, directions):
    """Geometric seams always keep the page-controlled blend width."""
    first = np.asarray(supports[directions[0]], bool)
    return np.zeros(first.shape, bool)


def seam_pair_ids(seam_pairs):
    return [f"{a}+{b}" for a, b in seam_pairs]


def seam_pair_coverage(supports, seam_pairs, min_overlap=300):
    """Return adjacent pairs that still have enough real overlap."""
    used = []
    for a, b in seam_pairs:
        overlap = np.asarray(supports[a], bool) & np.asarray(supports[b], bool)
        if int(overlap.sum()) >= int(min_overlap):
            used.append(f"{a}+{b}")
    return used


def owner_boundary_length(owner):
    """Count 4-connected owner transitions used as a smoothness proxy."""
    owner = np.asarray(owner)
    vertical = np.count_nonzero(owner[1:] != owner[:-1])
    horizontal = np.count_nonzero(owner[:, 1:] != owner[:, :-1])
    return int(vertical + horizontal)


def geometric_seam_quality(owner, reference_owner, supports, seam_pairs,
                           max_length_ratio=1.15, min_overlap=300):
    """Accept a seam only when it stays close to the directional diagonals."""
    coverage_before = np.any(np.stack([np.asarray(supports[d], bool)
                                       for d in supports]), axis=0)
    pairs = seam_pair_coverage(supports, seam_pairs, min_overlap=min_overlap)
    expected = set(seam_pair_ids(seam_pairs))
    if set(pairs) != expected:
        raise RuntimeError("四组相邻接缝未全部有效，结果未保存")
    reference_len = max(owner_boundary_length(reference_owner), 1)
    seam_len = owner_boundary_length(owner)
    length_ratio = float(seam_len) / float(reference_len)
    if length_ratio > float(max_length_ratio):
        raise RuntimeError("几何接缝边界过长，结果未保存")
    return {
        "reason_code": "optimized",
        "pairs": pairs,
        "coverage_lost_px": 0,
        "changed_px": 0,
        "boundary_px": seam_len,
        "reference_boundary_px": reference_len,
        "boundary_length_ratio": round(length_ratio, 3),
    }


def calibration_fingerprint(results_path: Path, directions) -> str:
    """Hash every geometry input used by the BEV renderer."""
    digest = hashlib.sha256()
    for name in [*(f"{d}.json" for d in directions), "extrinsics.json"]:
        path = Path(results_path) / name
        digest.update(name.encode("ascii"))
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, data: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)


def default_layout(direction: str) -> list[dict]:
    depths = (0.55, 1.20) if direction in ("front", "back") else (0.40, 1.05)
    return [
        {"id": f"{direction}-{depth:.2f}-{lateral:+.2f}",
         "near_m": depth, "lateral_m": lateral, "orient": "long-lateral"}
        for depth in depths for lateral in (-0.55, 0.0, 0.55)
    ]


def new_session(directions, intrinsics_fingerprint: str = "") -> dict:
    now = datetime.now().isoformat(timespec="seconds")
    return {
        "version": SESSION_VERSION,
        "id": datetime.now().strftime("%Y%m%d-%H%M%S"),
        "state": "capturing",
        "created_at": now,
        "updated_at": now,
        "layout": {d: default_layout(d) for d in directions},
        "observations": {d: [] for d in directions},
        "solutions": {},
        "intrinsics_fingerprint": str(intrinsics_fingerprint),
        "last_error": "",
    }


def save_session(session_dir: Path, session: dict) -> None:
    session["updated_at"] = datetime.now().isoformat(timespec="seconds")
    atomic_json(Path(session_dir) / "session.json", session)


def load_session(session_dir: Path, directions) -> dict:
    path = Path(session_dir) / "session.json"
    if not path.is_file():
        return new_session(directions)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("version") != SESSION_VERSION:
            raise ValueError("unsupported session version")
        for d in directions:
            data.setdefault("observations", {}).setdefault(d, [])
            data.setdefault("layout", {}).setdefault(d, default_layout(d))
        data.setdefault("solutions", {})
        data.setdefault("intrinsics_fingerprint", "")
        data.setdefault("last_error", "")
        return data
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return new_session(directions)


def _target_options(hom, direction, placement, cols, rows, square, scale, canvas):
    """Return both physically possible 180-degree checkerboard orderings."""
    g4, _ = hom.ground_corners(
        direction, placement["near_m"], placement.get("lateral_m", 0.0),
        placement.get("orient", "long-lateral"), cols, rows, square)
    g4 = np.asarray(g4, np.float64)
    grid_pos = [(0, 0), (0, cols - 1), (rows - 1, cols - 1),
                (rows - 1, 0)]
    cx, cy = float(canvas[0]) / 2.0, float(canvas[1]) / 2.0
    # 8x6 has distinguishable long/short axes, leaving only a 180-degree ambiguity.
    rotations = (0, 2) if placement.get("orient", "long-lateral") == "long-lateral" else (1, 3)
    choices = []
    for k in rotations:
        ordered = [g4[(k + i) % 4] for i in range(4)]
        mapping = {grid_pos[i]: ordered[i] for i in range(4)}
        ground = hom.bilinear_ground(
            mapping[(0, 0)], mapping[(0, cols - 1)],
            mapping[(rows - 1, 0)], mapping[(rows - 1, cols - 1)], cols, rows)
        targets = np.asarray([
            hom.ground_to_canvas(point[0], point[1], scale, cx, cy)
            for point in ground], np.float64)
        ros = np.zeros((len(ground), 3), np.float64)
        ros[:, 0] = ground[:, 1]
        ros[:, 1] = -ground[:, 0]
        choices.append((targets, ros, ground))
    return choices


def _fit_h(src, dst, threshold=3.0):
    H, mask = cv2.findHomography(
        np.asarray(src, np.float64), np.asarray(dst, np.float64), cv2.RANSAC,
        float(threshold), maxIters=5000, confidence=0.999)
    if H is None:
        raise ValueError("RANSAC failed to find a homography")
    mask = mask.ravel().astype(bool) if mask is not None else np.ones(len(src), bool)
    if int(mask.sum()) < 12:
        raise ValueError("too few global homography inliers")
    refined, _ = cv2.findHomography(np.asarray(src)[mask], np.asarray(dst)[mask], 0)
    if refined is not None:
        H = refined
    if not np.isfinite(H).all() or abs(float(np.linalg.det(H))) < 1e-12:
        raise ValueError("global homography is non-finite or degenerate")
    return H / H[2, 2], mask


def solve_multi_position(hom, observations, direction, cols, rows, square,
                         scale, canvas, new_k):
    """Fit one H and planar 6DoF pose from multiple measured board positions."""
    if len(observations) < 4:
        raise ValueError("at least four distinct positions are required")
    source_parts, option_parts = [], []
    position_ids = set()
    for obs in observations:
        corners = np.asarray(obs["undist_corners"], np.float64).reshape(-1, 2)
        if corners.shape != (cols * rows, 2):
            raise ValueError(f"observation {obs.get('id')} has invalid corners")
        source_parts.append(corners)
        option_parts.append(_target_options(
            hom, direction, obs["placement"], cols, rows, square, scale, canvas))
        position_ids.add(obs.get("position_id") or obs.get("id"))
    if len(position_ids) < 4:
        raise ValueError("at least four distinct physical positions are required")
    source = np.concatenate(source_parts)
    best = None
    # Six required positions produce only 64 combinations. Joint selection is
    # essential because a symmetric checkerboard cannot resolve 180 degrees in
    # isolation, while one global ground homography resolves it unambiguously.
    for selection in itertools.product((0, 1), repeat=len(observations)):
        target_try = np.concatenate([
            option_parts[i][choice][0] for i, choice in enumerate(selection)])
        try:
            H_try, inliers_try = _fit_h(source, target_try)
        except ValueError:
            continue
        pred_try = cv2.perspectiveTransform(
            source.reshape(-1, 1, 2), H_try).reshape(-1, 2)
        values = np.linalg.norm(pred_try - target_try, axis=1)
        score = float(np.percentile(values, 95))
        if best is None or score < best[0]:
            best = (score, selection, H_try, inliers_try)
    if best is None:
        raise ValueError("no globally consistent checkerboard orientation")
    _, selection, H, inliers = best
    target_parts = [option_parts[i][choice][0]
                    for i, choice in enumerate(selection)]
    ros_parts = [option_parts[i][choice][1]
                 for i, choice in enumerate(selection)]
    ground_parts = [option_parts[i][choice][2]
                    for i, choice in enumerate(selection)]
    target = np.concatenate(target_parts)
    projected = cv2.perspectiveTransform(
        source.reshape(-1, 1, 2).astype(np.float64), H).reshape(-1, 2)
    error = np.linalg.norm(projected - target, axis=1)
    per_position = []
    offset = 0
    for obs, part in zip(observations, source_parts):
        current = error[offset:offset + len(part)]
        per_position.append({
            "id": obs["id"], "position_id": obs.get("position_id"),
            "rms_px": float(np.sqrt(np.mean(current ** 2))),
            "p95_px": float(np.percentile(current, 95)),
            "max_px": float(np.max(current)),
        })
        offset += len(part)

    holdouts = []
    for index, (test_src, test_dst) in enumerate(zip(source_parts, target_parts)):
        train_src = np.concatenate([x for i, x in enumerate(source_parts) if i != index])
        train_dst = np.concatenate([x for i, x in enumerate(target_parts) if i != index])
        H_leave, _ = _fit_h(train_src, train_dst)
        pred = cv2.perspectiveTransform(
            test_src.reshape(-1, 1, 2).astype(np.float64), H_leave).reshape(-1, 2)
        values = np.linalg.norm(pred - test_dst, axis=1)
        holdouts.append({"id": observations[index]["id"],
                         "p95_px": float(np.percentile(values, 95)),
                         "rms_px": float(np.sqrt(np.mean(values ** 2)))})

    ros_all = np.concatenate(ros_parts)
    pose_result = hom.solve_camera_pose(source, ros_all, new_k)
    pose = {}
    if pose_result is not None:
        R, t = pose_result
        pose = {"R": np.asarray(R).tolist(),
                "t": [float(x) for x in np.asarray(t).ravel()]}

    train_rms = float(np.sqrt(np.mean(error ** 2)))
    max_position_p95 = max(item["p95_px"] for item in per_position)
    max_holdout_p95 = max(item["p95_px"] for item in holdouts)
    passed = (train_rms <= 3.0 and max_position_p95 <= 6.0
              and max_holdout_p95 <= 8.0 and bool(pose))
    ground = np.concatenate(ground_parts)
    ax = np.asarray({"front": (0.0, 1.0), "back": (0.0, -1.0),
                     "left": (-1.0, 0.0), "right": (1.0, 0.0)}[direction])
    lateral_axis = np.asarray((-ax[1], ax[0]))
    forward = ground @ ax
    lateral = ground @ lateral_axis
    envelope = {
        "forward_min_m": max(0.0, float(np.min(forward)) - 0.20),
        "forward_max_m": float(np.max(forward)) + 0.65,
        "lateral_abs_max_m": float(np.max(np.abs(lateral))) + 0.35,
    }
    return {
        "passed": passed, "H": H.tolist(), "pose": pose,
        "train_rms_px": train_rms,
        "inlier_ratio": float(np.mean(inliers)),
        "max_position_p95_px": max_position_p95,
        "max_holdout_p95_px": max_holdout_p95,
        "per_position": per_position, "holdouts": holdouts,
        "ground_envelope": envelope,
        "thresholds": {"train_rms_px": 3.0, "position_p95_px": 6.0,
                       "holdout_p95_px": 8.0},
    }


def save_seam_lut(path: Path, fingerprint: str, cache: dict, directions) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp.npz")
    payload = {
        "version": np.asarray([SEAM_LUT_VERSION], np.int32),
        "fingerprint": np.asarray([fingerprint]),
        "canvas": np.asarray(cache["canvas"], np.int32),
        "view_m": np.asarray([cache["view_m"]], np.float64),
        "gains": np.asarray([cache["gains"].get(d, 1.0) for d in directions],
                            np.float32),
        "quality_json": np.asarray([json.dumps(cache.get("seam_quality", {}),
                                                 ensure_ascii=False)]),
    }
    for d in directions:
        payload[f"allow_{d}"] = cache["seam_allow"][d].astype(np.uint8)
    payload["hard_seam_mask"] = np.asarray(
        cache.get("hard_seam_mask", np.zeros_like(cache["seam_allow"][directions[0]])),
        np.uint8)
    if path.exists():
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        backup = path.with_name(f"{path.stem}.v{SEAM_LUT_VERSION - 1}-{stamp}.bak.npz")
        shutil.copy2(path, backup)
    np.savez_compressed(tmp, **payload)
    os.replace(tmp, path)


def load_seam_lut(path: Path, fingerprint: str, cache: dict, directions):
    path = Path(path)
    if not path.is_file():
        return None, "missing"
    try:
        with np.load(path, allow_pickle=False) as data:
            version = int(data["version"][0])
            if version != SEAM_LUT_VERSION:
                return None, f"LUT version {version} needs re-optimization"
            stored = str(data["fingerprint"][0])
            if stored != fingerprint:
                return None, "calibration fingerprint changed"
            if tuple(int(x) for x in data["canvas"]) != tuple(cache["canvas"]):
                return None, "canvas changed"
            if abs(float(data["view_m"][0]) - float(cache["view_m"])) > 1e-6:
                return None, "view range changed"
            allowed = {}
            for d in directions:
                mask = np.asarray(data[f"allow_{d}"], np.uint8).astype(bool)
                if mask.shape != cache["supports"][d].shape:
                    return None, "mask shape changed"
                allowed[d] = mask & cache["supports"][d]
            hard_mask = np.asarray(data["hard_seam_mask"], np.uint8).astype(bool)
            if hard_mask.shape != cache["supports"][directions[0]].shape:
                return None, "hard seam shape changed"
            support_union = np.any(np.stack(
                [cache["supports"][d] for d in directions]), axis=0)
            allowed_union = np.any(np.stack(
                [allowed[d] for d in directions]), axis=0)
            if np.any(support_union & ~allowed_union):
                return None, "stored seam removes trustworthy coverage"
            gains = {d: float(data["gains"][i]) for i, d in enumerate(directions)}
            quality = json.loads(str(data["quality_json"][0])) \
                if "quality_json" in data else {}
        return {"allowed": allowed, "gains": gains, "quality": quality,
                "hard_mask": hard_mask}, "ok"
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return None, f"invalid: {exc}"


def _single_json_item(item: dict) -> dict:
    return {
        "H": np.asarray(item["H"], dtype=np.float64).tolist(),
        "qc": item["qc"],
        "rms": float(item["rms"]),
        "burst": item.get("burst", {}),
        "pose": item.get("pose", {}),
        "pose_qc": item.get("pose_qc", {}),
        "placement": item.get("placement", {}),
        "meas": item.get("meas"),
    }


def save_single_pending(path: Path, pending: dict, directions) -> None:
    """Persist single-position candidates without touching formal extrinsics."""
    data = {
        "version": 1,
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "directions": {d: _single_json_item(pending[d])
                       for d in directions if d in pending},
    }
    atomic_json(Path(path), data)


def load_single_pending(path: Path, directions) -> dict:
    path = Path(path)
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("version") != 1:
            return {}
        return {d: item for d, item in data.get("directions", {}).items()
                if d in directions}
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return {}


def commit_single_position(results_path: Path, pending: dict, directions,
                           bev_cfg: dict, pattern, square: float) -> Path:
    """Atomically replace all four direct-H results from one-position captures."""
    if set(pending) != set(directions):
        raise ValueError("all four single-position candidates are required")
    for d in directions:
        item = pending[d]
        H = np.asarray(item.get("H"), dtype=np.float64)
        if H.shape != (3, 3) or not np.isfinite(H).all():
            raise ValueError(f"{d} homography is invalid")
        if item.get("qc", {}).get("status") == "bad":
            raise ValueError(f"{d} homography failed QC")
        rms = float(item.get("rms", float("nan")))
        if not np.isfinite(rms) or rms > 3.0:
            raise ValueError(f"{d} RMS is invalid")

    if "left" in pending and "right" in pending:
        sigmas = {}
        for d in ("left", "right"):
            sigmas[d] = float(np.linalg.svd(
                np.asarray(pending[d]["H"], dtype=np.float64)[:2, :2],
                compute_uv=False)[0])
        ratio = max(sigmas.values()) / max(min(sigmas.values()), 1e-9)
        if ratio > 4.0:
            raise ValueError(
                f"left/right homography scale ratio {ratio:.2f} exceeds 4.0")

    results_path = Path(results_path)
    path = results_path / "extrinsics.json"
    if not path.is_file():
        raise ValueError("formal extrinsics.json is missing")
    old = json.loads(path.read_text(encoding="utf-8"))
    backup_dir = results_path / "backups"
    backup_dir.mkdir(parents=True, exist_ok=True)
    backup = backup_dir / f"extrinsics.pre-single.{datetime.now():%Y%m%d-%H%M%S-%f}.json"
    shutil.copy2(path, backup)

    data = dict(old)
    data["pattern_size"] = [int(pattern[0]), int(pattern[1])]
    data["square_size_m"] = float(square)
    data["scale_px_per_meter"] = float(bev_cfg["scale_px_per_meter"])
    data["canvas_size"] = [int(x) for x in bev_cfg["canvas_size"]]
    data["vehicle_center"] = [float(x) for x in bev_cfg["vehicle_center"]]
    data["balance"] = float(bev_cfg["balance"])
    data["extrinsic_balance"] = float(bev_cfg["extrinsic_balance"])
    data["_meta"] = {
        "scale_px_per_meter": data["scale_px_per_meter"],
        "canvas_size": data["canvas_size"],
        "extrinsic_balance": data["extrinsic_balance"],
    }
    data["homographies"] = {
        d: np.asarray(pending[d]["H"], dtype=np.float64).tolist()
        for d in directions}
    data["rms_errors"] = {d: float(pending[d]["rms"]) for d in directions}
    data["homography_qc"] = {d: pending[d]["qc"] for d in directions}
    data["homography_source"] = {
        d: "single_position_direct_h" for d in directions}
    data["burst"] = {d: pending[d].get("burst", {}) for d in directions}
    data["poses"] = {d: pending[d]["pose"] for d in directions
                     if pending[d].get("pose")}
    data["pose_qc"] = {d: pending[d].get("pose_qc", {}) for d in directions}
    data["placements"] = {d: pending[d].get("placement", {}) for d in directions}
    data["board_measured_bev"] = {
        d: pending[d].get("meas") for d in directions}
    data["seam_refined"] = []
    data["verifications"] = []
    data.pop("ground_envelopes", None)
    data.pop("multi_position", None)
    data["single_position"] = {
        "committed_at": datetime.now().isoformat(timespec="seconds"),
        "source": "direct_h",
        "atomic_four_camera_commit": True,
        "backup": str(backup),
    }
    atomic_json(path, data)
    return backup


def commit_multi_session(results_path: Path, session: dict, directions) -> Path:
    """Atomically replace all four extrinsics after every candidate passes."""
    solutions = session.get("solutions") or {}
    if set(solutions) != set(directions) or not all(solutions[d].get("passed") for d in directions):
        raise ValueError("all four camera solutions must pass before commit")
    path = Path(results_path) / "extrinsics.json"
    old = json.loads(path.read_text(encoding="utf-8"))
    backup_dir = Path(results_path) / "backups"
    backup_dir.mkdir(parents=True, exist_ok=True)
    backup = backup_dir / f"extrinsics.pre-multi.{datetime.now():%Y%m%d-%H%M%S-%f}.json"
    shutil.copy2(path, backup)
    data = dict(old)
    data["homographies"] = {d: solutions[d]["H"] for d in directions}
    data["homography_source"] = {
        d: "multi_position_direct_h" for d in directions}
    data["poses"] = {d: solutions[d]["pose"] for d in directions}
    data["rms_errors"] = {d: solutions[d]["train_rms_px"] for d in directions}
    data["homography_qc"] = {d: solutions[d]["qc"] for d in directions}
    data["board_measured_bev"] = {
        d: solutions[d]["board_measured_bev"] for d in directions}
    data["ground_envelopes"] = {d: solutions[d]["ground_envelope"] for d in directions}
    data["multi_position"] = {
        "session_id": session["id"], "committed_at": datetime.now().isoformat(timespec="seconds"),
        "quality": {d: {k: solutions[d][k] for k in (
            "train_rms_px", "max_position_p95_px", "max_holdout_p95_px",
            "inlier_ratio")} for d in directions},
        "observation_count": {d: len(session["observations"][d]) for d in directions},
    }
    atomic_json(path, data)
    return backup
