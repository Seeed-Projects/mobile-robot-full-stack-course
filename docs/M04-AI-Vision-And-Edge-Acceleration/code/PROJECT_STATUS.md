# M4 Project Status

This is the only runtime-status source of truth for M4. Course pages and
handoff notes may summarize it, but must link here for evidence and numbers.

Last verified: **2026-09-22** on Seeed reComputer Robotics J501, AGX Orin
32 GB, JetPack 6.2.1 / L4T R36.4.4, CUDA 12.6, TensorRT 10.3 and ROS 2
Humble.

Allowed states: `PASS`, `VERIFIED`, `PARTIAL`, `BLOCKED`, `PLANNED`.

| Module | Status | Runtime evidence | Current blocker |
| --- | --- | --- | --- |
| M4.1 YOLO11n TensorRT | **PASS** | Build and 28 production-path GTests pass; physical camera and web overlay previously measured at about 29 FPS | None for the documented scope |
| M4.2 ByteTrack | **PASS** | 30 pytest tests pass; physical camera tracking previously measured at about 30 FPS | None for the documented scope |
| M4.3 SegFormer TensorRT | **VERIFIED** | Unit, engine gate and full suite pass; parity and geometry have physical evidence | Evidence covers the verified model and sampled scenes, not a dataset survey |
| M4.4 FoundationPose | **BLOCKED** | Package scaffold installs; no successful inference exists | Runtime/weights incomplete and no supported RGB-D camera is attached |
| M4.5 Isaac ROS | **PLANNED** | Documentation and acceptance design only | No runnable implementation exists |

## M4.1 — YOLO11n TensorRT

- **Model:** YOLO11n FP16 TensorRT engine.
- **Entry:** `modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_1_demo.sh`.
- **Topics:** image input `/perception/cameras/front/image`; detections
  `/perception/detections`; preview `/perception/demo/m4_1`.
- **Tests:** `colcon test --packages-select bev_detection` completed with
  28 tests, 0 errors, 0 failures and 0 skipped on 2026-09-22.
- **Benchmark:** the last physical single-module run was approximately 29 FPS.
  This consolidation did not manufacture a newer benchmark number.
- **Evidence:** tests exercise the same `YoloPostprocess`, NMS, IoU, bbox
  restore and letterbox implementation used by runtime inference.
- **Known limitation:** model and engine binaries are target-generated and are
  intentionally not distributable through the course snapshot.

## M4.2 — ByteTrack

- **Software:** `supervision.ByteTrack` 0.27.0.
- **Entry:** `modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_2_demo.sh`.
- **Topics:** `/perception/detections` to `/perception/tracks`; stable track id
  is written to `Detection2D.id`.
- **Tests:** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q` completed
  with 30 passed on 2026-09-22.
- **Benchmark:** the last physical single-module run was approximately 30 FPS.
- **Known limitation:** pytest plugin autoload must stay disabled on this image
  because the user-site `anyio` plugin is incompatible with the system pytest.

## M4.3 — SegFormer TensorRT

- **Model:** SegFormer-B0 FP16 TensorRT with Cityscapes labels.
- **Entry:** `modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_3_demo.sh`.
- **Topics:** semantic mask `/perception/semantic_mask`, drivable mask
  `/perception/drivable_mask`, preview `/perception/demo/m4_3`.
- **Tests:** six unit-labelled CTest targets and two engine-gate targets pass;
  the full suite has seven CTest targets and 29 GTests, all passing on
  2026-09-22.
- **Parity evidence:** PyTorch/TensorRT pixel agreement 0.9966 and mean IoU
  0.9898 from the last verified replay.
- **Geometry evidence:** fixed inverse-letterbox path agrees 1.0000 with the
  published mask; the reproduced old path agrees 0.8101.
- **Benchmark:** the last idle `trtexec` measurement was 58.88 qps. The managed
  Hub caps segmentation at 10 FPS by default to protect the shared pipeline.

## M4.4 — FoundationPose

- **Status:** `BLOCKED`; no inference has ever been accepted.
- **Entry:** `run_m4_4_demo.sh` is scaffolding, not proof of a working runtime.
- **Available:** package skeleton and a CAD target description.
- **Missing:** complete FoundationPose runtime/weights, supported RGB-D driver
  and attached Orbbec Gemini 2 hardware.
- **Acceptance required:** one real RGB-D inference, pose/TF publication and
  repeatable physical evidence before this state can change.

## M4.5 — Isaac ROS

- **Status:** `PLANNED`.
- **Implementation:** no runnable M4.5 package or verified acceleration graph.
- **Course scope:** principles, migration plan, experiment design and future
  acceptance criteria only.

## Shared Hub and input control

- One shared camera, web server and browser session remain alive while exactly
  one selected inference pipeline runs.
- Hub regression, runtime ownership and video-input lifecycle tests passed on
  2026-09-22. New video sessions clear stale frame/loop telemetry; first-frame
  and stall failures persist their reason and fall back to camera.
- The production camera publisher is installed by `m4_demo_bringup`; it no
  longer lives in a test directory.
- The supported supervisor shutdown signal is `SIGTERM`. Detached processes
  can inherit `SIGINT` as ignored, so Ctrl+C claims are not used as evidence.

## Known cross-cutting limits

- The older all-modules-at-once architecture measured about 15 FPS because it
  was CPU-bound. The managed Hub avoids that mode; this is not evidence of a
  faster individual model.
- M4.4 object naming/configuration still needs reconciliation when M4.4 resumes.
- No Git remote is configured on the Jetson. The verified P0 bundle and dirty
  worktree recovery archive are stored on the Mac under
  `m4_code_upgrade/backups/P0_20260922-162759/`.
