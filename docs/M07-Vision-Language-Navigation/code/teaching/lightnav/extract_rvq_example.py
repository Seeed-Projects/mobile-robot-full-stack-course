#!/usr/bin/env python3
"""Freeze one RVQ decode example from a released ``action_tokenizer`` bundle.

This is the *optional* provenance step: it reads the published bundle once, picks three
code ids, and writes ``rvq-teaching-example.json`` with every intermediate quantity the
animation draws (per-level codeword -> motion deltas, the sum, and the composed
waypoints). ``build_visuals.py`` and ``verify_examples.py`` only read that frozen JSON, so
the animation can be rebuilt on a machine without any checkpoint.

The arithmetic is the same order the released decoder uses
(``RVQBundle.decode_waypoints``): sum the per-level codewords, divide by the Jacobian
weights, reshape to ``(horizon, 3)``, then SE(2)-compose. Nothing here is a new
definition -- it is a longer-hand transcription with the intermediates kept.

Usage:
    python3 extract_rvq_example.py \
        --bundle /path/to/checkpoint/action_tokenizer \
        --out rvq-teaching-example.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
# Fixed choice, documented in the animation: a level-0 code whose composed path is a
# gentle forward arc, plus the two residual codes that refine it. Recorded in the output
# so a rebuild cannot silently pick different ids.
CODES = [14, 42, 30]
CANDIDATE_IDS = 40  # how many nearby l0 ids the picker reports for context


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(bundle: Path):
    man = json.loads((bundle / "manifest.json").read_text())
    if man.get("method") != "rvq":
        raise SystemExit(f"{bundle}: manifest.method={man.get('method')!r}, expected 'rvq'")
    levels = list(man["levels"])
    dim = int(man["feature_dim"])
    books = []
    for i, fn in enumerate(man["codebook_files"]):
        cb = np.load(bundle / fn).astype(np.float32)
        if cb.shape != (levels[i], dim):
            raise SystemExit(f"codebook l{i} shape {cb.shape} != ({levels[i]}, {dim})")
        books.append(cb)
    w = np.load(bundle / man["jacobian_weights_file"]).astype(np.float32)
    if w.shape != (dim,):
        raise SystemExit(f"jacobian_weights shape {w.shape} != ({dim},)")
    return man, levels, dim, books, w


def compose(deltas: np.ndarray) -> np.ndarray:
    """Local copy of the released composition, float64 accumulation, same order."""
    out = np.empty((deltas.shape[0], 3), dtype=np.float64)
    x = y = th = 0.0
    for j in range(deltas.shape[0]):
        dx, dy, dth = float(deltas[j, 0]), float(deltas[j, 1]), float(deltas[j, 2])
        x = x + np.cos(th) * dx - np.sin(th) * dy
        y = y + np.sin(th) * dx + np.cos(th) * dy
        th = (th + dth + np.pi) % (2.0 * np.pi) - np.pi
        out[j] = (x, y, th)
    return out


def round_list(a, nd=6):
    return np.round(np.asarray(a, dtype=float), nd).tolist()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bundle", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=HERE / "rvq-teaching-example.json")
    args = ap.parse_args()
    bundle = args.bundle
    man, levels, dim, books, w = load(bundle)
    horizon = int(man["horizon"])
    assert dim == 3 * horizon, (dim, horizon)
    assert all(v == 256 for v in levels), levels

    # Per-level motion deltas: what each of the three codes contributes on its own.
    per_level = []
    for lvl, code in enumerate(CODES):
        feat = (books[lvl][code] / w).reshape(horizon, 3)
        per_level.append(feat)

    recon = sum(books[lvl][code] for lvl, code in enumerate(CODES))
    feat_sum = (recon / w).reshape(horizon, 3)
    for lvl in range(3):
        assert np.allclose(sum(per_level[: lvl + 1]), (sum(books[i][CODES[i]] for i in range(lvl + 1)) / w).reshape(horizon, 3))

    stop_l0 = int((man.get("stop") or {}).get("l0", -1))
    assert stop_l0 >= 0, "manifest has no stop code"
    stop_codes = [int(c) for c in (man.get("stop") or {}).get("codes", [])]
    stop_feat = (books[0][stop_l0] / w).reshape(horizon, 3)

    # Level-0 only / +L1 / +L2: how much path error each residual removes, measured as the
    # distance between that level's decoded path and the full three-level path.
    wanted = ["l0", "l0+l1", "l0+l1+l2"]
    partial = [compose(np.array(per_level[0])), compose(sum(per_level[:2])), compose(feat_sum)]
    refine = []
    for name, path in zip(wanted, partial):
        de = float(np.linalg.norm((path - partial[-1])[:, :2], axis=1).mean())
        dh = float(np.abs(path[:, 2] - partial[-1][:, 2]).mean())
        refine.append({"levels": name, "mean_position_gap_m": round(de, 4), "mean_heading_gap_rad": round(dh, 4)})

    # per-step and per-chunk magnitudes, averaged over every codeword of each level
    level_stats = []
    for lvl in range(3):
        feat = (books[lvl] / w).reshape(levels[lvl], horizon, 3)
        step = np.linalg.norm(feat[:, :, :2], axis=2)
        dth = np.abs(feat[:, :, 2])
        net = np.array([np.linalg.norm(compose(f)[-1, :2]) for f in feat])
        level_stats.append({
            "level": lvl,
            "entries": int(levels[lvl]),
            "per_step_xy_mean_m": round(float(step.mean()), 4),
            "per_step_xy_p90_m": round(float(np.percentile(step, 90)), 4),
            "per_step_heading_mean_rad": round(float(dth.mean()), 4),
            "chunk_net_mean_m": round(float(net.mean()), 4),
            "chunk_net_median_m": round(float(np.median(net)), 4),
            "chunk_net_p90_m": round(float(np.percentile(net, 90)), 4),
        })

    payload = {
        "source": "released action_tokenizer bundle, decoded offline",
        "bundle_manifest_sha256": sha256(bundle / "manifest.json"),
        "codebook_sha256": [sha256(bundle / fn) for fn in man["codebook_files"]],
        "jacobian_weights_sha256": sha256(bundle / man["jacobian_weights_file"]),
        "horizon": horizon,
        "levels": levels,
        "feature_dim": dim,
        "representation": man["representation"],
        "feature_space": man["feature_space"],
        "objective": man["objective"],
        "encode": man["encode"],
        "codes": CODES,
        "token_literals": [f"<act_l{lvl}_{code}>" for lvl, code in enumerate(CODES)],
        "combo_count": int(np.prod(levels)),
        "codeword_motion_deltas": [round_list(f) for f in per_level],
        "summed_motion_deltas": round_list(feat_sum),
        "codeword_sum_feature": round_list(recon),
        "waypoints_full": round_list(compose(feat_sum)),
        "waypoints_partial": [round_list(p) for p in partial],
        "refinement": refine,
        "level_stats": level_stats,
        "stop": {
            "l0": stop_l0,
            "codes": stop_codes,
            "codeword_abs_max": round(float(np.abs(stop_feat).max()), 9),
            "codeword_net_m": round(float(np.linalg.norm(compose(np.array(stop_feat))[-1, :2])), 9),
            "codeword_motion_deltas": round_list(stop_feat),
        },
        "measured_metrics": {
            "per_step_xy": "mean norm of (dx, dy) per waypoint of that level codeword alone",
            "chunk_net": "norm of the composed endpoint of that level codeword alone",
            "refinement_gap": "mean per-point distance to the three-level decoded path",
        },
        "not_execution_error": True,
        "teaching_only": True,
    }
    args.out.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"wrote {args.out} ({args.out.stat().st_size} bytes)")
    for row in level_stats:
        print(f"  L{row['level']}: step {row['per_step_xy_mean_m']:.4f} m, net {row['chunk_net_mean_m']:.4f} m")
    print("  refine:", [r["mean_position_gap_m"] for r in refine])
    print(f"  stop l0={stop_l0} codes={stop_codes} net={payload['stop']['codeword_net_m']:.2e} m")


if __name__ == "__main__":
    main()
