# M4 Project Status

Factual state of each chapter. Evidence is from a physical run on the target Jetson
(2026-09-18) unless stated otherwise.

| Module | Source | Build | Tests | Model | ROS | Physical Hardware | Web Demo | Status | Next Blocker |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 4.1 YOLO detection | Complete | PASS | 21 GTests, **1 failing** (dead-code path) | YOLO11n FP16 TRT engine present | PASS | **PASS** - 28.7 Hz image -> 28.6 Hz detections | PASS (overlay + H.264 WebRTC) | **PASS** | fix or delete the dead `parseYoloOutput` test |
| 4.2 ByteTrack tracking | Complete | PASS | 30 pytest **PASS** (needs `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`) | n/a (supervision 0.27.0) | PASS | **PASS** - 24.7-28.3 Hz tracks, real ids | PASS | **PASS** | `frame_rate` mis-set (10 vs ~28 Hz actual) |
| 4.3 SegFormer segmentation | Complete (C++) | PASS | 32 GTests, **0 failures, 0 skipped**; engine gate separate and enforced | ONNX + TensorRT FP16 engine + labels all present | **PASS** - 30.0 Hz masks (segmentation alone) | **PASS** - real GMSL camera, geometry proven | **PASS** (side-by-side semantic/drivable, 4.3 tab) | **VERIFIED** | `/m4/3` does not deep-link to the 4.3 tab; broken `m4_3_demo.launch.py` |
| 4.4 FoundationPose 6D | Scaffold, fabricated API | PASS (installs) | none exist | **MISSING** - no repo, no weights; a CAD model now exists (see below) | NOT TESTED | **BLOCKED** - no Orbbec driver, no USB device | BLOCKED | **BLOCKED** | make NVLabs FoundationPose run standalone first |

## 4.3 evidence (VERIFIED)

Measured from the course workspace on 2026-09-18:

```
TensorRT baseline (trtexec, idle box)
    throughput        94.72 qps
    GPU compute       10.51 ms mean / 10.51 ms median
    total latency     11.06 ms
PyTorch <-> TensorRT parity (real 1920x1080 frame)
    pixel agreement   0.9973      (threshold 0.95)
    mean IoU          0.9813      (threshold 0.85)
Geometry, node vs replayed frame
    fixed path        1.0000 agreement with the published mask
    old path          0.7071      (left band 0.5524)
Live topics, segmentation_node alone
    /perception/cameras/front/image   30.00 Hz
    /perception/semantic_mask         30.00 Hz
    /perception/drivable_mask         30.01 Hz
Live topics, full 4.1+4.2+4.3 hub (CPU-bound)
    /perception/semantic_mask         11.6 Hz
    /perception/demo/m4_3             15.3 Hz
```

Two long-standing defects were real and are now fixed: the inverse letterbox
(masks were geometrically wrong for any non-2:1 source) and the silently-skipped
engine tests. A third was found on the way: `build_segformer_engine.sh` aborted
before reaching `trtexec`, so no engine could ever have been built on this machine.

The checkpoint licence was verified and is **unspecified** (`license: other` with
no terms). Not redistributable - `models/m4/segmentation/LICENSE.md`.

## 4.4 evidence (BLOCKED)

- The scaffold called an **invented FoundationPose API** (`model_dir=`,
  `scorer_module=`, `refiner_module=`, `mesh_file=`, `device=`), verified against
  the real upstream `estimater.py`, whose constructor takes `model_pts`,
  `model_normals`, `symmetry_tfs`, `mesh`, `scorer`, `refiner`.
- `scripts/m4/phase0_foundationpose_verify.sh:285` contains a hard Python syntax
  error (`masks[i]=masks[i]` as a keyword name).
- No NVlabs checkout, no weights, no Orbbec driver, no Orbbec hardware.
- A real CAD model for the target object now exists
  (`GL.iNet GL-SFT1200 "Opal"`, measured 111.671 x 80.000 x 75.000 mm), but M4.4
  has not been run.

## Evidence for the 4.1 / 4.2 PASS rows

Measured from the course workspace (`modules/m04-.../ros2_ws`) on 2026-09-18:

```
camera /perception/cameras/front/image   28.675 Hz    (1920x1080 bgr8)
       /perception/detections            28.636 Hz
       /perception/tracks                24.721 Hz
live track sample:  id: '96'  class_id: person  score: 0.479
tracking_node:      frame=4888 in_dets=1 out_tracks=1 latency_ms=1.60
web /healthz:       {"server_ready":true,"ros_frame_ready":true,
                     "peer_ready":true,"transport":"h264_gst"}
```

GPU inference depth is ~3.4 ms and end-to-end ~18 ms (from the node's own 1 Hz
telemetry). Earlier documentation quoted 48.3 Hz / 19 ms / 105 ms from a sample
benchmark that no artifact supports - those numbers have been removed.

## What is NOT proven

- 4.4 has never executed anything: the package installs, but every runtime
  dependency is absent and no depth camera is attached.
- No chapter other than 4.1/4.2/4.3 has been exercised against physical hardware.
- The full 4.1+4.2+4.3 pipeline is **CPU-bound at ~11-16 Hz**, not the ~28 Hz that
  4.1/4.2 achieve on their own. Adding 4.3 costs the other modules throughput.
- "Ctrl+C cleanup" was **not** verified as such: the demo was started detached
  (`setsid nohup` from a non-interactive shell), which makes bash set SIGINT to
  ignore, so a SIGINT test is meaningless in that configuration. SIGTERM - the
  signal the supervisor actually traps - tears everything down cleanly and an
  immediate restart comes back up.