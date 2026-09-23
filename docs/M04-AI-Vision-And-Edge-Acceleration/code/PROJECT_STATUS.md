# M4 Project Status

This is the only runtime-status source of truth for M4. Course pages and
handoff notes may summarize it, but must link here for evidence and numbers.

Last verified: **2026-09-23** on Seeed reComputer Robotics J501, AGX Orin
32 GB, JetPack 6.2.1 / L4T R36.4.4, CUDA 12.6, TensorRT 10.3 and ROS 2
Humble.

Allowed states: `PASS`, `VERIFIED`, `PARTIAL`, `BLOCKED`, `PLANNED`.

| Module | Status | Runtime evidence | Current blocker |
| --- | --- | --- | --- |
| M4.1 YOLO11n TensorRT | **PASS** | Build and 28 production-path GTests pass; physical camera and web overlay previously measured at about 29 FPS | None for the documented scope |
| M4.2 ByteTrack | **PASS** | 30 pytest tests pass; physical camera tracking previously measured at about 30 FPS | None for the documented scope |
| M4.3 SegFormer TensorRT | **PARTIAL** | Bilinear preprocessing and logits restoration are implemented; CUDA and CPU paths agree; 8/8 CTests and the engine gate pass; Hub semantic output averaged about 11.3 FPS after optimization | Hub preview was 6.7–10.4 FPS in a 30-second page-rate sample, so stable 10 FPS is not accepted; ADE20K indoor candidate remains unevaluated and Cityscapes stays default |
| M4.4 Isaac ROS FoundationPose | **PARTIAL** | Isaac ROS 3.2 packages, official Mustard bag/mesh, ONNX models, FP32 refine and FP16 RT-DETR engines are present; no pose output accepted | Score engine build fails at the official max profile; official bag replay remains incomplete |
| M4.5 NVlabs FoundationPose | **BLOCKED** | Legacy `bev_pose` scaffold installs; no successful inference exists | Native runtime, weights, supported RGB-D driver and physical camera remain incomplete |
| Shared Hub / video input | **PARTIAL** | Video repair is committed; build, 3 Hub regression cycles and 9 video state tests pass; a fresh three-upload run reached `playing` each time | The final run used a synthetic camera and did not exercise a physical-camera session or injected stall |

## M4.1 — YOLO11n TensorRT

- **Model:** YOLO11n FP16 TensorRT engine.
- **Entry:** `modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_1_demo.sh`.
- **Topics:** image input `/perception/cameras/front/image`; detections
  `/perception/detections`; preview `/perception/demo/m4_1`.
- **Tests:** `colcon test --packages-select bev_detection` completed with
  28 tests, 0 errors, 0 failures and 0 skipped on 2026-09-23.
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
  with 30 passed on 2026-09-23.
- **Benchmark:** the last physical single-module run was approximately 30 FPS.
- **Known limitation:** pytest plugin autoload must stay disabled on this image
  because the user-site `anyio` plugin is incompatible with the system pytest.

## M4.3 — SegFormer TensorRT

- **Model:** SegFormer-B0 FP16 TensorRT with Cityscapes labels.
- **Entry:** `modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_3_demo.sh`.
- **Topics:** semantic mask `/perception/semantic_mask`, drivable mask
  `/perception/drivable_mask`, preview `/perception/demo/m4_3`.
- **Tests:** seven unit-labelled CTest targets and the engine-existence gate
  pass; the full package has 8/8 CTest targets passing on 2026-09-23.
- **Geometry implementation:** input sampling now uses bilinear pixel-centre
  mapping. Postprocessing restores logits through the letterbox geometry before
  argmax. The CPU path is row/class parallel; the Jetson CUDA path produces the
  same mask as the scalar reference in the fixed regression case.
- **Runtime evidence:** with `SEGMENTATION_MAX_FPS=15`, the semantic topic
  measured roughly 11.3 FPS over a long sample. The Hub page rate varied from
  6.7 to 12.5 FPS (30 samples, mean 10.38), so the stable 10 FPS gate failed.
- **Final video-source check:** with the Hub at its default 10 FPS cap and a
  synthetic camera, M4.3 preview reported about 6.8–8.1 FPS. Four consecutive
  preview frames had distinct content and `frame_id=m4_video_input`.
- **Indoor model decision:** ADE20K-B0 (`nvidia/segformer-b0-finetuned-ade-512-512`,
  revision `489d5cd81a0b59fab9b7ea758d3548ebe99677da`, license marker
  `other` with no clear terms) was not promoted. No ADE20K engine or binary is
  distributed, and Cityscapes remains the formal default.
- **Evidence limit:** the earlier temporary set of 20 indoor JPEGs is no longer
  on this device. No fixed-frame Cityscapes/ADE20K quality score is available.
- **Parity evidence:** PyTorch/TensorRT pixel agreement 0.9966 and mean IoU
  0.9898 from the last verified replay.
- **Geometry evidence:** fixed inverse-letterbox path agrees 1.0000 with the
  published mask; the reproduced old path agrees 0.8101.
- **Benchmark:** the last idle `trtexec` measurement was 58.88 qps. The
  managed Hub default remains 10 FPS; the 15 FPS run was diagnostic only and
  did not change the release gate.

## M4.4 — Isaac ROS FoundationPose

- **Status:** `PARTIAL` for setup only; no 6D pose inference is accepted.
- **Runtime:** separate `m4-isaacros-foundationpose` container with
  `ros-humble-isaac-ros-foundationpose 3.2.14` and
  `ros-humble-isaac-ros-examples 3.2.5` on JetPack 6.2.1 / ROS 2 Humble.
- **Assets:** official NGC `isaac_ros_foundationpose_assets` 3.2.0 Mustard
  textured mesh, 640x480 interface spec and `quickstart.bag`; the bag has one
  RGB, depth and camera-info message each. FoundationPose `1.0.0_onnx`
  refine/score models are in `/home/seeed/workspace/isaac_ros_assets/models/foundationpose`.
- **Engine evidence:** FP32 refine engine built with TensorRT 10.3 using
  `--maxAuxStreams=0 --builderOptimizationLevel=0 --memPoolSize=workspace:4096`;
  the resulting 92 MB plan passed `trtexec` inference (13.8 ms median for
  batch 1). The official SyntheticaDETR grasp ONNX was downloaded, parsed and
  built as a 94 MB FP16 plan; its `trtexec` run passed at 41.1 qps while the
  shared Hub was active. The score plan did not build: both the concurrent and
  later idle retries requested a 2190 MB tactic while reporting about 1.4 GB
  available, including a 12 GB workspace retry.
- **Entry:** `scripts/m4/run_m4_4_isaacros_quickstart.sh` guards against an
  active M4.1 or M4.3 inference node, builds missing FoundationPose engines and
  launches the official `foundationpose` fragment. It uses the separate
  SyntheticaDETR grasp RT-DETR engine and has not yet produced a pose message.
- **Next gate:** on an agreed GPU maintenance window, build the score engine,
  replay the official bag and capture one valid `output` pose. A
  one-frame bag cannot establish FPS or physical-camera acceptance. Project
  integration also needs a real object-instance mask and RGB/depth alignment;
  M4.3's semantic mask is not an instance mask.

## M4.5 — NVlabs FoundationPose

- **Status:** `BLOCKED`; native `bev_pose` has no accepted inference.
- **Legacy physical paths:** `4.4-foundationpose/`, `run_m4_4_demo.sh` and
  `m4_4_demo.launch.py` still identify the original scaffold. They are not the
  M4.4 Isaac ROS entry and must not be used as evidence for it.
- **Missing:** validated NVlabs runtime/weights, supported RGB-D driver and
  attached Orbbec Gemini 2. Native acceptance remains separate from the
  Isaac ROS quickstart.

## Shared Hub and input control

- One shared camera, web server and browser session remain alive while exactly
  one selected inference pipeline runs.
- The 5-package build, 3 Hub regression cycles, 14 video/sync tests, M4.1
  (28), M4.2 (30), and M4.3 unit/engine/full tests passed on 2026-09-23.
  New video sessions clear stale frame/loop telemetry; first-frame and stall
  failures persist their reason and fall back to camera.
- **Live input acceptance is PARTIAL.** The isolated Hub regression passed 3/3
  cycles and the video state suite passes 9/9. The repaired mux recreates the
  selected subscription on the ROS executor thread, preserves configured
  topics, records publisher/received/forwarded counters, and reports failure
  stages without calling them decoder failures. Earlier physical runs also
  recorded three uploads, loop counter increments, `frame_id=m4_video_input`,
  first-frame timeout recovery, and 3-second stall recovery. In a fresh test
  with a synthetic camera and M4.3 active, three uploads each changed from
  `starting` to `playing` without error; three camera/video switches recovered,
  `/perception/cameras/front/image` retained `frame_id=m4_video_input`, and
  `loop_count` increased from 0 to 1 after a full 50.4-second replay. The
  source was synthetic and no fresh forced-stall test was run, so the overall
  status remains `PARTIAL`.
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
