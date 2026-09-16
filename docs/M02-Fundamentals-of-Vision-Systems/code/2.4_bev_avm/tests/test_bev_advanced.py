import json
import sys
import types
from pathlib import Path

import cv2
import numpy as np

import bev_advanced as advanced

if "j501_avm_calib.config" not in sys.modules:
    config_stub = types.ModuleType("j501_avm_calib.config")
    config_stub.CAM_AXIS = {"front": (0.0, 1.0), "back": (0.0, -1.0),
                            "left": (-1.0, 0.0), "right": (1.0, 0.0)}
    sys.modules["j501_avm_calib.config"] = config_stub
from j501_avm_calib import homography

DIRECTIONS = ("front", "back", "left", "right")


def test_owner_keeps_no_direction_unknown():
    shape = (20, 20)
    supports = {d: np.ones(shape, bool) for d in DIRECTIONS}
    scores = {d: np.zeros(shape, np.float32) for d in DIRECTIONS}
    owner, weights = advanced.owner_and_weights(
        supports, scores, DIRECTIONS, 4)
    assert np.all(owner == -1)
    assert all(np.count_nonzero(weights[d]) == 0 for d in DIRECTIONS)


def test_owner_uses_only_positive_direction_score():
    shape = (20, 20)
    supports = {d: np.ones(shape, bool) for d in DIRECTIONS}
    scores = {d: np.zeros(shape, np.float32) for d in DIRECTIONS}
    scores["front"][:10] = 1.0
    scores["back"][10:] = 1.0
    owner, _ = advanced.owner_and_weights(supports, scores, DIRECTIONS, 0)
    assert np.all(owner[:10] == DIRECTIONS.index("front"))
    assert np.all(owner[10:] == DIRECTIONS.index("back"))


def test_seam_lut_roundtrip_and_fingerprint_rejection(tmp_path):
    shape = (24, 24)
    supports = {d: np.ones(shape, bool) for d in DIRECTIONS}
    cache = {"canvas": (24, 24), "view_m": 4.0, "supports": supports,
             "seam_allow": {d: np.ones(shape, bool) for d in DIRECTIONS},
             "gains": {d: 1.0 + i * 0.01 for i, d in enumerate(DIRECTIONS)}}
    path = tmp_path / "seam_lut.npz"
    advanced.save_seam_lut(path, "abc", cache, DIRECTIONS)
    loaded, reason = advanced.load_seam_lut(
        path, "abc", cache, DIRECTIONS)
    assert reason == "ok"
    assert loaded is not None
    stale, reason = advanced.load_seam_lut(
        path, "different", cache, DIRECTIONS)
    assert stale is None
    assert "fingerprint" in reason


def test_multi_position_global_solution_passes_synthetic_data():
    cols, rows, square = 8, 6, 0.025
    scale, canvas = 250.0, (1000, 1000)
    direction = "front"
    true_h = np.array([[1.10, 0.04, -240.0],
                       [-0.03, 0.85, -110.0],
                       [0.00008, -0.00015, 1.0]], np.float64)
    rng = np.random.default_rng(3)
    observations = []
    for index, placement in enumerate(advanced.default_layout(direction)):
        g4, _ = homography.ground_corners(
            direction, placement["near_m"], placement["lateral_m"],
            placement["orient"], cols, rows, square)
        ground = homography.bilinear_ground(
            g4[0], g4[1], g4[3], g4[2], cols, rows)
        target = np.asarray([
            homography.ground_to_canvas(p[0], p[1], scale, 500.0, 500.0)
            for p in ground], np.float64)
        source = cv2.perspectiveTransform(
            target.reshape(-1, 1, 2), np.linalg.inv(true_h)).reshape(-1, 2)
        source += rng.normal(0.0, 0.08, source.shape)
        if index % 2:
            source = source[::-1]
        observations.append({"id": placement["id"],
                             "position_id": placement["id"],
                             "placement": placement,
                             "undist_corners": source.tolist()})
    new_k = np.array([[700.0, 0.0, 960.0], [0.0, 700.0, 768.0],
                      [0.0, 0.0, 1.0]])
    result = advanced.solve_multi_position(
        homography, observations, direction, cols, rows, square, scale,
        canvas, new_k)
    assert result["passed"]
    assert result["train_rms_px"] < 1.0
    assert result["max_holdout_p95_px"] < 2.0


def test_commit_requires_all_passing_solutions(tmp_path):
    (tmp_path / "extrinsics.json").write_text(
        json.dumps({"homographies": {}}), encoding="utf-8")
    session = advanced.new_session(DIRECTIONS)
    try:
        advanced.commit_multi_session(tmp_path, session, DIRECTIONS)
    except ValueError as exc:
        assert "all four" in str(exc)
    else:
        raise AssertionError("incomplete session was committed")

CAM_AXIS = {"front": (0.0, 1.0), "back": (0.0, -1.0),
            "left": (-1.0, 0.0), "right": (1.0, 0.0)}
SEAM_PAIRS = (("front", "left"), ("front", "right"),
              ("back", "left"), ("back", "right"))


def _sector_canvas(size=121):
    xx, yy = np.meshgrid(np.arange(size, dtype=np.float32),
                         np.arange(size, dtype=np.float32))
    cx = cy = (size - 1) / 2.0
    gx = xx - cx
    gy = -(yy - cy)
    supports = {d: np.ones((size, size), bool) for d in DIRECTIONS}
    scores = advanced.sector_scores(supports, DIRECTIONS, CAM_AXIS, gx, gy)
    return gx, gy, supports, scores


def test_geometric_seam_follows_sector_diagonals():
    gx, gy, supports, scores = _sector_canvas()
    allowed, hard_mask, owner, weights = advanced.geometric_seam(
        supports, scores, DIRECTIONS, 0)
    assert not np.any(hard_mask)
    assert all(np.array_equal(allowed[d], supports[d]) for d in DIRECTIONS)
    front, back, left, right = (DIRECTIONS.index(d) for d in DIRECTIONS)
    hypot = np.maximum(np.hypot(gx, gy), 1e-6)
    inner = np.cos(np.deg2rad(30.0))
    far = hypot > 8.0
    assert np.all(owner[((gy / hypot) > inner) & far] == front)
    assert np.all(owner[((-gy / hypot) > inner) & far] == back)
    assert np.all(owner[((-gx / hypot) > inner) & far] == left)
    assert np.all(owner[((gx / hypot) > inner) & far] == right)
    quality = advanced.geometric_seam_quality(
        owner, owner, supports, SEAM_PAIRS)
    assert quality["boundary_length_ratio"] == 1.0
    assert set(quality["pairs"]) == {f"{a}+{b}" for a, b in SEAM_PAIRS}


def test_geometric_seam_keeps_page_blend_width():
    _, _, supports, scores = _sector_canvas(81)
    _, hard_mask, _, weights = advanced.geometric_seam(
        supports, scores, DIRECTIONS, 6)
    mixed = ((weights["front"] > 0.02) & (weights["right"] > 0.02))
    assert not np.any(hard_mask)
    assert np.any(mixed)


def test_zigzag_owner_is_rejected_by_quality_gate():
    shape = (80, 80)
    supports = {d: np.ones(shape, bool) for d in DIRECTIONS}
    reference = np.zeros(shape, np.int16)
    reference[:, 40:] = 1
    zigzag = np.indices(shape).sum(axis=0).astype(np.int16) % 2
    try:
        advanced.geometric_seam_quality(zigzag, reference, supports, SEAM_PAIRS)
    except RuntimeError as exc:
        assert "过长" in str(exc)
    else:
        raise AssertionError("zigzag seam was accepted")


def test_seam_lut_v3_roundtrip_rejects_v2(tmp_path):
    shape = (24, 24)
    supports = {d: np.ones(shape, bool) for d in DIRECTIONS}
    cache = {"canvas": (24, 24), "view_m": 4.0, "supports": supports,
             "seam_allow": {d: np.ones(shape, bool) for d in DIRECTIONS},
             "gains": {d: 1.0 + i * 0.01 for i, d in enumerate(DIRECTIONS)},
             "hard_seam_mask": np.zeros(shape, bool)}
    path = tmp_path / "seam_lut.npz"
    advanced.save_seam_lut(path, "abc", cache, DIRECTIONS)
    with np.load(path) as data:
        assert int(data["version"][0]) == 3
        assert int(np.asarray(data["hard_seam_mask"]).sum()) == 0
    loaded, reason = advanced.load_seam_lut(path, "abc", cache, DIRECTIONS)
    assert reason == "ok"
    assert loaded is not None
    assert not np.any(loaded["hard_mask"])
    tmp = tmp_path / "seam_lut.v2.npz"
    payload = dict(np.load(path))
    payload["version"] = np.asarray([2], np.int32)
    np.savez_compressed(tmp, **payload)
    stale, reason = advanced.load_seam_lut(tmp, "abc", cache, DIRECTIONS)
    assert stale is None
    assert "version" in reason
