# M4 source snapshot

| Field | Value |
| --- | --- |
| Source repository | Jetson `/home/seeed/workspace/ros2_bev` |
| Source branch | `main` |
| Source commits | `6683e08dca3edb7137494d7f84fccc93f5826623` for current M4.4 official Mustard status and runner; `6b83f5b959e487a36addada0462c00131d86e993` for the earlier adapted M4.4 files; `7e945912a299ec36e0a335f508b2a45d53a54060` for the older M4.5 snapshot |
| Snapshot date | 2026-09-24 |
| Direction | Jetson runtime source to course snapshot |

## Path mapping

| Jetson source | Course destination |
| --- | --- |
| `modules/m04-ai-vision-and-edge-acceleration/4.1-yolo-object-detection/` | `code/4.1-yolo-object-detection/` |
| `modules/m04-ai-vision-and-edge-acceleration/4.2-multi-object-tracking/` | `code/4.2-multi-object-tracking/` |
| `modules/m04-ai-vision-and-edge-acceleration/4.3-semantic-segmentation/` | `code/4.3-semantic-segmentation/` |
| `modules/m04-ai-vision-and-edge-acceleration/4.4-foundationpose/` | `code/4.4-foundationpose/` |
| `modules/m04-ai-vision-and-edge-acceleration/4.4-isaac-ros-foundationpose/` | `code/4.4-isaac-ros-foundationpose/` |
| `modules/m04-ai-vision-and-edge-acceleration/scripts/m4/` | `code/scripts/m4/` |
| `modules/m04-ai-vision-and-edge-acceleration/PROJECT_STATUS.md` | `code/PROJECT_STATUS.md` |

`code/README.md`, this file, `code/common/README.md`, `code/.gitignore`, and
`code/scripts/setup_workspace.sh` are course packaging files. Runtime output,
model binaries, build trees, caches and machine-local configuration are not
included in the course snapshot.

## M4.5 MVP addition

The 2026-09-24 snapshot adds the native Chapter 4.5 standalone entry points:

- `scripts/m4/run_m4_5_native_mvp.py`
- `scripts/m4/run_m4_5_native_mvp.sh`
- `scripts/m4/phase0_foundationpose_verify.sh` as a compatibility wrapper

The runner uses the pinned NVlabs FoundationPose checkout and official recorded
Mustard RGB-D sequence, saving pose matrices, annotated frames and a JSON timing
report under `output/m4/m45_native_mvp/` on the Jetson.
