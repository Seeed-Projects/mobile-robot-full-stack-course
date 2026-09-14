# Datasets

## nuScenes v1.0-mini (Phase 0/1 validation)

| | |
|---|---|
| Source | https://www.nuscenes.org/data/v1.0-mini.tgz (also `scripts/download_nuscenes_mini.sh`) |
| License | CC BY-NC-SA 4.0 (non-commercial), nuScenes terms of use apply |
| sha256 (of this mirror, recorded 2026-09-03) | `943037abbb3b26b3070dc76504a43eb440503b00baf9ac2f1538d9c03fc9298f` |
| Kept on device | `v1.0-mini/*.json` (meta) + `samples/` 706 MB + `maps/` 5.6 MB |
| Discarded (re-downloadable) | `sweeps/` 4.4 GB — BEVDet consumes 2 Hz keyframes only |

Layout on device:

```
datasets/nuscenes/
├── v1.0-mini/            # metadata JSON (new 2025 schema, no 'channel' field)
├── samples/CAM_*/        # per-camera keyframe JPEGs
├── maps/
└── data_infos/           # generated: bevdet_vendor format (gitignored)
    ├── time_sequence.yaml
    ├── sample_tokens.json         # canonical ranking for evaluators
    └── samples_info/sample%04d.yaml
```

Regenerate `data_infos`:

```bash
.venv-tools/bin/python tools/dataset_converter/nuscenes_to_bevdet_infos.py \
    datasets/nuscenes datasets/nuscenes
```

## Format quirks (2025 nuScenes mini release)

1. `sample_data.json` has **no `channel` field** → derive from filename
   (`__CAM_XX__` pattern).
2. `sample_annotation.json` has **no `category_token`** → resolve
   `instance_token → instance.json → category_token`.
3. `calibrated_sensor.camera_intrinsic` is a **3×3 matrix** (not a flat list);
   some per-sweep calibrations carry an empty intrinsic (only keyframe
   calibrations are populated).
4. `lidar2ego` translation in this release is `[0.9858, 0, 1.8402]` (the
   classic release had `[0.9437, 0, 1.8402]`); read it from the meta, never
   hardcode.

## Own-dataset format (Phase 3+)

Planned nuScenes-like tree (see master spec §30):

```
datasets/own_dataset/
└── samples/
    ├── CAM_FRONT/ ... (6 dirs)
metadata: timestamp, intrinsic, extrinsic, ego_pose (JSON sidecar)
3D annotations: class, center xyz, size lwh, yaw
```

`tools/dataset_converter` will gain a `rosbag → own_dataset` mode in Phase 3.