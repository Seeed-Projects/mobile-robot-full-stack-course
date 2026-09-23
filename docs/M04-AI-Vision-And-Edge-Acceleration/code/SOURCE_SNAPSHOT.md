# M4 source snapshot

| Field | Value |
| --- | --- |
| Source repository | Jetson `/home/seeed/workspace/ros2_bev` |
| Source branch | `main` |
| Source commit | `7e945912a299ec36e0a335f508b2a45d53a54060` for the M4.4/M4.5 files listed below |
| Snapshot date | 2026-09-23 |
| Direction | A → C only; never copy course code back to the Jetson |

## Path mapping

| Jetson source | Course destination |
| --- | --- |
| `modules/m04-ai-vision-and-edge-acceleration/4.1-yolo-object-detection/` | `code/4.1-yolo-object-detection/` |
| `modules/m04-ai-vision-and-edge-acceleration/4.2-multi-object-tracking/` | `code/4.2-multi-object-tracking/` |
| `modules/m04-ai-vision-and-edge-acceleration/4.3-semantic-segmentation/` | `code/4.3-semantic-segmentation/` |
| `modules/m04-ai-vision-and-edge-acceleration/4.4-foundationpose/` | `code/4.4-foundationpose/` |
| `modules/m04-ai-vision-and-edge-acceleration/4.4-isaac-ros-foundationpose/README.md` | `code/4.4-isaac-ros-foundationpose/README.md` |
| `modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_4_isaacros_quickstart.sh` | `code/scripts/m4/run_m4_4_isaacros_quickstart.sh` |
| `modules/m04-ai-vision-and-edge-acceleration/common/ros2/m4_demo_bringup/` | `code/common/ros2/m4_demo_bringup/` |
| `modules/common/ros2/bev_interfaces/` | `code/common/ros2/bev_interfaces/` |
| `modules/m04-ai-vision-and-edge-acceleration/models/m4/` | `code/models/m4/` |
| `modules/m04-ai-vision-and-edge-acceleration/scripts/{m4,regression}/` | `code/scripts/{m4,regression}/` |
| `modules/m04-ai-vision-and-edge-acceleration/PROJECT_STATUS.md` | `code/PROJECT_STATUS.md` |

`code/README.md`, this file, `code/common/README.md`, `code/.gitignore`, and
`code/scripts/setup_workspace.sh` are course packaging files. In particular,
`setup_workspace.sh` creates the standalone course `ros2_ws`; the Jetson runtime
does not use it and builds directly with `scripts/build.sh --base-paths modules`.

## Verified status at this commit

- M4.1: **PASS**
- M4.2: **PASS**
- M4.3: see the canonical status file; this M4.4/M4.5 sync does not update its source
- M4.4 Isaac ROS FoundationPose: **PARTIAL**; assets and two engines present, no pose output
- M4.5 native NVlabs FoundationPose: **BLOCKED**

The M4.4/M4.5 files above were copied from the source commit. Other paths in
this course code tree retain their earlier snapshot provenance; this update
does not copy concurrent M4.1/M4.3 work.

Detailed commands, evidence and blockers are in [`PROJECT_STATUS.md`](PROJECT_STATUS.md).

## Exclusions

The snapshot intentionally excludes build/install/log/output trees, caches,
model binaries, CAD binaries, machine-local configuration, credentials,
internal Agent handoff files and product requirement drafts.
