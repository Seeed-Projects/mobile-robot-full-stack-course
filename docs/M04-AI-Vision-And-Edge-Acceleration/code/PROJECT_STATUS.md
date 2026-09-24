# M4 Project Status

This is the only runtime-status source of truth for M4. Course pages and
handoff notes may summarize it, but must link here for evidence and numbers.

Last verified: **2026-09-24** for M4.4 visualization; other module evidence is
from 2026-09-23. Platform: Seeed reComputer Robotics J501, AGX Orin
32 GB, JetPack 6.2.1 / L4T R36.4.4, CUDA 12.6, TensorRT 10.3 and ROS 2
Humble.

Allowed states: `PASS`, `VERIFIED`, `PARTIAL`, `BLOCKED`, `PLANNED`.

| Module | Status | Runtime evidence | Current blocker |
| --- | --- | --- | --- |
| M4.1 YOLO11n TensorRT | **PASS** | Build and 28 production-path GTests pass; physical camera and web overlay previously measured at about 29 FPS | None for the documented scope |
| M4.2 ByteTrack | **PASS** | 30 pytest tests pass; physical camera tracking previously measured at about 30 FPS | None for the documented scope |
| M4.3 SegFormer TensorRT | **PARTIAL** | Bilinear preprocessing and logits restoration are implemented; CUDA and CPU paths agree; 8/8 CTests and the engine gate pass; Hub semantic output averaged about 11.3 FPS after optimization | Hub preview was 6.7–10.4 FPS in a 30-second page-rate sample, so stable 10 FPS is not accepted; ADE20K indoor candidate remains unevaluated and Cityscapes stays default |
| M4.4 Isaac ROS FoundationPose | **PARTIAL** | Official FP32/252 Mustard graph and separate FP32/42 adaptation both produced valid `Detection3DArray` poses on `/output` | Orbbec Gemini 2 is unavailable, so physical RGB-D acceptance remains open |
| M4.5 NVlabs FoundationPose | **PARTIAL** | Native standalone MVP completed 8-frame Mustard register/tracking run with finite poses, annotated frames and timing report | This MVP uses the official recorded sequence; live RGB-D camera integration is a later extension |
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

- **Status:** `PARTIAL` overall. The **official FP32/252 Mustard single-frame
  example passed**, as did the separate 42-candidate adaptation. The physical
  RGB-D gate has not passed.
- **Runtime:** separate `m4-isaacros-foundationpose` container with
  `ros-humble-isaac-ros-foundationpose 3.2.14` and
  `ros-humble-isaac-ros-examples 3.2.5` on JetPack 6.2.1 / ROS 2 Humble.
- **Assets:** official NGC `isaac_ros_foundationpose_assets` 3.2.0 Mustard
  textured mesh, 640x480 interface spec and `quickstart.bag`; the bag has one
  RGB, depth and camera-info message each. FoundationPose `1.0.0_onnx`
  refine/score models are in `/home/seeed/workspace/isaac_ros_assets/models/foundationpose`.
- **Official engine evidence:** an idle FP32 score build inside the container
  used NVIDIA's 1/1/252 min/opt/max shapes and failed when a 2190 MB tactic
  found only 1405 MB available (`score_trtexec_official_fp32.log`). The same
  1/1/252 FP32 build on the Jetson host, with TensorRT 10.3 and no custom
  workspace/optimization flags, **passed** in 213 seconds. It used
  `--skipInference` only to separate build from runtime verification. The
  persistent plan is `score_trt_engine.plan`; build log
  `score_trtexec_252_host_fp32.log`. The container then deserialized that plan
  at the 252 maximum shape (`score_trtexec_252_load_max.log`, PASSED). The FP32
  refine and separate FP16 SyntheticaDETR grasp engines also passed `trtexec`.
- **Adapted engine and graph:** FP32 score profile min/opt/max 1/1/42 built
  and passed TensorRT inference, including a maximum-shape deserialize check.
  Its independent plan is `score_trt_engine_42_fp32.plan`, with build log
  `score_trtexec_42_fp32.log`. The project config sets `max_hypothesis: 42`;
  the custom launch sets `fixed_axis_angles=['z_0']` so the sampled grid fits
  that cap, at the cost of narrower orientation coverage. Runtime ROS
  parameters confirmed the config path, angle constraint and 42-profile plan.
  The successful pose proves the runtime batch fits that engine profile.
- **Reproduce:** from `/home/seeed/workspace/ros2_bev`, run
  `M44_MODE=official modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_4_isaacros_quickstart.sh`.
  If the official score plan is absent, the runner builds it with host
  TensorRT before entering the container; `M44_MODE=adapted` retains the
  independent 42-profile path. The runner blocks active M4.1/M4.3 GPU
  nodes, loops the one-frame bag, validates `vision_msgs/Detection3DArray`,
  and stops its launch and bag process groups on exit. Its actual output topic
  is `/output` (the node's logged remapping), not the example namespace path.
- **Accepted adapted pose:** 2026-09-23 run
  `20260923-100715-14266-adapted`: `frame_id=tf_camera`, position in metres
  `[-0.4713481963, 0.0929617882, 0.8295211196]`, quaternion xyzw
  `[0.2184645543, -0.3925129918, 0.0745772909, 0.8903061370]`, norm
  `1.0`. The verifier saw one message, rejected none and returned `valid_pose`.
  Logs are under `/home/seeed/workspace/isaac_ros_assets/m4_4_logs/` with
  this run ID. With `POSE_TOPIC=/m44_absent_topic M44_POSE_TIMEOUT=2`, the
  runner returned a timeout and exit 8; a synthetic active-node process
  triggered the GPU guard and exit 3. After success and timeout, no launch,
  rosbag or component-container process remained.
- **Accepted official pose:** 2026-09-23 run
  `20260923-112313-14976-official`: `frame_id=tf_camera`, position in metres
  `[-0.4350625575, 0.1339290440, 0.7972502112]`, quaternion xyzw
  `[0.7753970849, -0.3331845536, 0.3022323303, -0.4431738174]`, norm
  `1.0`; the verifier returned `valid_pose`, with no rejected messages. The
  launch, bag and pose logs have this run ID under `m4_4_logs/`.
- **Visualization follow-up (2026-09-24):** the user's grid-only RViz view had
  the Camera dock hidden, and the acceptance runner stopped playback as soon
  as `valid_pose` arrived. The visual runner now opens RViz, focuses the 3D
  view near the recorded Mustard pose, and holds the official graph and looping
  bag for 120 seconds by default. The GNOME desktop icon
  `/home/seeed/Desktop/m4_4_mustard_demo.desktop` calls it with a 300-second
  hold. On the physical 1024x600 Jetson display, launching that icon through
  the graphical session produced a visible Mustard RGB frame in RViz's Camera
  dock and a red 3D detection. Run `20260924-014511-25838-official` returned
  `valid_pose`; `/output` had one publisher and one RViz subscriber, and
  `/rgb/image_rect_color` had one publisher and four subscribers during the
  hold. The separate local viewer image `m44-foundationpose-viewer:local`
  (image ID `5e56c210b462`) was made from the installed Isaac ROS container
  so RViz could access NVIDIA's display through the local X11 socket. Sending
  an interrupt to the launcher's process group removed its viewer container,
  launch, rosbag and component container. A separate `M44_VIEW_SECONDS=3`
  normal-exit run (`20260924-014935-27114-official`) returned `valid_pose`
  and also left none of those live processes. A second visual launch is
  rejected with exit 3 while its per-user lock is held. This visualization does not change
  the single-frame evidence limit or M4.4's `PARTIAL` physical-camera status.
- **Next gate:** the one-frame bag cannot establish FPS or physical-camera
  performance. The user confirmed that Orbbec Gemini 2 is not currently
  available; physical RGB-D, camera calibration/alignment and an object
  instance mask remain unaccepted. M4.3's semantic mask is not one.

## M4.5 — NVlabs FoundationPose

- **Status:** `PARTIAL`; the standalone native FoundationPose MVP completed an
  8-frame official Mustard sequence on 2026-09-24.
- **Runtime evidence:** frame 0 used `register` with the 252-candidate global
  grid and frames 1–7 used `track_one`. All eight 4x4 pose matrices were finite.
  The run wrote `report.json`, per-frame matrices and annotated first/last
  frames under `output/m4/m45_native_mvp/` and printed
  `M45_NATIVE_MVP_OK`.
- **Measured timing:** first-frame registration took 11050.90 ms. Tracking took
  89.37–237.11 ms per frame, averaging about 120.43 ms; the report's 8.30 FPS
  is a tracking-only reference and not a full camera-pipeline frame rate.
- **Runtime:** pinned NVlabs commit
  `a1b694b83e633c2cb6115b9063d940a687759392`, Python 3.10, PyTorch 2.11,
  PyTorch3D 0.7.9, nvdiffrast 0.4.0 and the compiled `mycpp` extension.
- **JetPack compatibility:** the runner disables user-site packages and uses a
  local analytic inverse for the 3x3 crop/camera matrices because the current
  PyTorch wheel requests two cuSOLVER symbols newer than JetPack 6.2.1's CUDA
  12.6 library. The upstream FoundationPose checkout is unchanged.
- **Legacy physical paths:** `4.4-foundationpose/`, `run_m4_4_demo.sh` and
  `m4_4_demo.launch.py` still identify the original scaffold. They are not the
  M4.4 Isaac ROS entry and must not be used as evidence for it.
- **Scope:** this release uses the official recorded RGB-D sequence. ROS 2
  wrapping and a live RGB-D camera remain follow-up integration work.

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
- M4.4's adapted Mustard pose is an isolated demonstration, with narrower
  orientation sampling than NVIDIA's 252-candidate graph.
- No Git remote is configured on the Jetson. The verified P0 bundle and dirty
  worktree recovery archive are stored on the Mac under
  `m4_code_upgrade/backups/P0_20260922-162759/`.
