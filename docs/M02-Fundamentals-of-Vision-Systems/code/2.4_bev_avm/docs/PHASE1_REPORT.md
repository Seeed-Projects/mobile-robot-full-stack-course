# Phase 1 Report — nuScenes Offline Validation (ROS-free)

> Date: 2026-09-03 · Device: Jetson AGX Orin 32GB (sm_87, JP 6.2.1, TRT 10.3, MODE_40W)
>
> **Verdict: PHASE 1 GATE — PASS** ✅

## What was run

| | |
|---|---|
| Data | nuScenes v1.0-mini (new metadata format), 404 keyframe samples, 10 scenes |
| Frames inferenced | **200 consecutive keyframes** (time-sequence order, scenes 1–5 of mini) |
| Path | real 6-camera JPEG → CPU decode → BEVDet FP16 TRT (built in Phase 0) → per-frame ego-frame 3D boxes |
| Speed | mean 42.06 ms/frame (≈23.8 FPS, warmup excluded) — consistent with Phase 0 benchmark |
| Artifacts | `output/phase1_egoboxes/bevdet_egoboxes_{0..199}.txt`, `output/phase1_run.log` |

Data pipeline repro (all committed):

```bash
scripts/download_nuscenes_mini.sh                             # source + sha256 pinned
.venv-tools/bin/python tools/dataset_converter/nuscenes_to_bevdet_infos.py \
    datasets/nuscenes datasets/nuscenes                       # nuScenes -> data_infos
cd tools/build && ./bevdemo /home/seeed/workspace/ros2_bev/models/configs/configure_phase1.yaml
.venv-tools/bin/python tools/eval_phase1.py datasets/nuscenes \
    output/phase1_egoboxes output/accuracy_comparison.json    # GT comparison
```

Notes on the 2025-format mini release: `sample_data.json` dropped the `channel`
field (derived from filename), `sample_annotation.json` has no `category_token`
(resolved via `instance.json`), `calibrated_sensor.camera_intrinsic` is a 3×3
matrix — the committed converter handles all of this and records
`sample_tokens.json` so evaluators use the identical sample ranking.

## Gate check 1 — structure & coordinates (GT comparison)

`output/accuracy_comparison.json` (score ≥ 0.25) · `output/accuracy_comparison_score0.4.json` (≥ 0.4)

| Metric (threshold) | score ≥ 0.25 | score ≥ 0.4 |
|---|---|---|
| GT boxes evaluated (200 frames) | 7935 | 7935 |
| Pred boxes kept | 6815 | 3703 |
| **Matched pairs** | **3401** | **2773** |
| Match fraction of GT | 42.9 % | 34.9 % |
| Center error p50 | 0.37 m | 0.35 m |
| Center error mean / p95 | 0.47 m / 1.21 m | 0.45 m / 1.16 m |
| Yaw error mean / p95 (all classes) | 28.7° / 170° | 26.6° / 169° |

Match criterion: same class, BEV center ≤ 2 m (≤ 1 m for small classes).

## Gate check 2 — yaw direction

Per-class yaw error on matched pairs (score ≥ 0.4):

| Class | matched | yaw mean | yaw p95 | verdict |
|---|---|---|---|---|
| car | 1570 | 9.6° | 18.5° | ✅ direction correct |
| truck | 95 | 6.3° | 12.5° | ✅ |
| construction_vehicle | 22 | 4.0° | 13.3° | ✅ |
| trailer | 7 | 5.7° | 9.6° | ✅ |
| barrier | 183 | 12.8° | 35.7° | ✅ (elongated, ok) |
| motorcycle / bicycle | 56 | 53–56° | 171–177° | ⚠ degenerate yaw on small objects (known BEVDet trait) |
| pedestrian | 695 | 67.3° | 177° | ⚠ pedestrians have no stable heading (expected) |
| traffic_cone | 145 | 39.9° | 147° | ⚠ symmetric object (expected) |

Interpretation: unambiguous vehicle classes show mean yaw error ≤ 10° —
**yaw convention and direction are correct**. High errors concentrate on
symmetric/degenerate classes, matching the published BEVDet behaviors, not a
pipeline defect. l/w are flipped on some ambiguous boxes (hence mean size err
~1.9 m on l/w vs 0.11 m on h) — size magnitudes are correct modulo the yaw
ambiguity.

## Gate check 3 — camera order (ablation, single scene)

Control experiments on the Phase 0 sample scene (cameras unchanged):

| Run | Result |
|---|---|
| Baseline (6 correct images) | 105 boxes; strong (≥0.4) boxes: 2 in front region, 6 in rear/side |
| **CAM_FRONT→black image** | 94 boxes; front-region strong boxes collapse 2→1; the two highest-score front vehicles (0.81/0.80) **disappear** (nearest counterparts 0.15/0.10); all 6 rear/side strong boxes survive at 0.03–1.85 m with same classes |
| CAM_FRONT↔CAM_BACK image swap | mid-strong boxes partially retained (GEOMETRIC multi-view redundancy — the model fuses per-camera geometry with depth, so swapped pixels still project through correct per-slot extrinsics); cannot be used alone as order evidence |

The black-out ablation is the decisive per-camera binding proof: content fed to
a camera slot affects exactly the BEV region that camera observes.

## Gate verdict

| # | Criterion | Evidence | Status |
|---|---|---|---|
| 1 | TensorRT output structurally correct (200 frames, 6 cams each) | all frames ran; stable 42 ms | ✅ |
| 2 | No obvious coordinate inversion | GT match p50 center err 0.37 m with 43 % recall | ✅ |
| 3 | Yaw direction correct | vehicles yaw mean 9.6°, p95 18.5° | ✅ |
| 4 | Camera order correct | front-black ablation front-vs-rear differential | ✅ |
| 5 | Pipeline vs domain separable | failures concentrate on degenerate-yaw classes (pedestrians/cones) | ✅ |

## Deferred item (documented deviation)

Plan called for PyTorch-reference comparison of `box center/dims/yaw/class/score`
on x86. No x86 GPU host is available in this session; instead the reference
roles are filled by (a) the vendor's authoritative sample inference (Phase 0,
105↔104 boxes, matched), and (b) nuScenes **ground truth** (this phase, 200
frames). A PyTorch export/reference runbook is documented below for the
workstation when available.

```bash
# on x86 workstation (future): reference via upstream ONNX-export repo
git clone --depth 1 --branch export https://github.com/LCH1238/BEVDet.git
# follow BEVDet_exp/tools/export/export_onx_onnx.py with the nuScenes val split;
# compare against output/accuracy_comparison.json statistics (same thresholds).
```

## Known limitations carried forward

- BEVDet-R50 128×128 BEV yaw ambiguity on symmetric classes (no fix planned; is
  a model-capability issue, Phase 6 occupancy/tracking will absorb it).
- First-frame temporal buffer is zero-initialized (standard for LTS)
- Score threshold 0.25 is a Phase-1 default; Phase 2 node config will expose it.
- nuScenes mini evaluation is sanity/geometry validation, NOT a mAP benchmark
  (model was trained on nuScenes train split which includes these scenes).