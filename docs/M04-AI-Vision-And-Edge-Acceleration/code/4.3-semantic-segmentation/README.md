# 4.3 Semantic Segmentation + Drivable Area

SegFormer-B0 (Cityscapes, 19 classes) exported to TensorRT, producing a per-pixel
class map and a binary drivable-area mask.

**Status: VERIFIED on hardware (2026-09-18).** The full chain runs on the real
GMSL camera: checkpoint -> ONNX -> TensorRT FP16 -> ROS topics -> web demo.
PyTorch/TensorRT parity passes, the inverse-letterbox geometry is proven against
the running node, and the module appears in the unified M4 web UI as READY.

## Pipeline (as implemented)

```
/perception/cameras/front/image
  -> letterbox to 512x1024, ImageNet normalise, RGB float32 NCHW
  -> SegFormer-B0 TensorRT (FP16)  -> logits [1,19,128,256]
  -> argmax over classes
  -> unletterbox_mask(): strip the letterbox padding, then sample to the
     original resolution
  -> /perception/semantic_mask  (mono8 class ids 0..18)
     /perception/drivable_mask  (mono8, 0/255)
```

`header.stamp` and `header.frame_id` are inherited from the source image.
The mask is published at the source resolution (1920x1080 on the GMSL camera).

## Source

`ros2/bev_segmentation/` - C++ package. `src/segmentation_node.cpp` (ROS glue),
`src/segmentation_engine.cpp` (TensorRT), `src/preprocess.cpp`,
`src/postprocess.cpp` (argmax, `unletterbox_mask`, drivable LUT),
`config/segmentation.yaml`.

## Model artifacts

| Artifact | Path | State |
| --- | --- | --- |
| ONNX export | `models/m4/segmentation/onnx/segformer_b0.onnx` | present (1.3 MB, opset 18) |
| TensorRT engine | `models/m4/segmentation/engines/segformer_b0_fp16.engine` | present (10.8 MB, FP16) |
| Class labels | `models/m4/segmentation/labels/labels.json` | present (19 classes) |

The ONNX and engine are **gitignored** - they are target-specific binaries and the
checkpoint's licence does not permit redistribution (see `LICENSE.md`). Only
`labels.json` is tracked. Reproduce them on the target:

```bash
export HF_ENDPOINT=https://hf-mirror.com   # huggingface.co has no route from this Jetson
./scripts/m4/export_segformer.sh           # checkpoint -> ONNX  (static 1x3x512x1024)
./scripts/m4/build_segformer_engine.sh     # ONNX -> FP16 engine
./scripts/m4/generate_labels_json.sh       # id2label -> labels.json
```

`export_segformer.sh` uses **opset 18**, not 13: the `torch.export`-based exporter
has no `Resize` implementation below 17, so requesting 13 left the file at 18
while printing a misleading conversion failure. TensorRT 10.3 parses 18 natively.

Without the engine the node **throws in its constructor** (`Cannot open engine
file: ...`) - there is no ONNX/PyTorch fallback.

## Not a silent fallback: the engine gate

`test_engine_smoke.cpp` calls `GTEST_SKIP()` when the engine is absent, and gtest
records a skip as a pass - so a workspace with no engine used to report a green
suite while the two model tests had never run. `test/check_engine_gate.sh`
(registered as `engine_existence_gate`) now fails hard instead:

```bash
colcon test --packages-select bev_segmentation --ctest-args -L unit
    # fast path: geometry/algorithm tests, no model artifact needed
colcon test --packages-select bev_segmentation --ctest-args -L engine_gate
    # release gate: RED until the engine exists
```

Verified by moving the engine aside: `-L engine_gate` goes red, `-L unit` stays
green.

## Inverse letterbox (why `unletterbox_mask` exists)

Preprocessing letterboxes the source into the 512x1024 canvas. Restoring the mask
therefore has to run the letterbox **backwards**:

```
argmax at 128x256 -> canvas coordinates -> subtract pad -> original resolution
```

The node previously did `nearest_neighbor_resize(argmax, 128x256 -> orig_h, orig_w)`,
which is only correct at exactly 2:1. At 1920x1080 the canvas is 910x512 with
57 px of pad each side, so 14 mask columns - about 105 output columns - of
padding prediction landed on real image pixels.

`unletterbox_mask()` does this in one fused pass rather than two resizes, and
reuses `compute_letterbox()`'s own `+0.5f` rounding so the two are exact
inverses. Composing with `nearest_neighbor_resize` (which maps with floor) would
introduce a systematic half-pixel bias.

Correctness is pinned two ways:

- `test/test_unletterbox.cpp` - 11 cases over 1920x1080, 1280x720, 640x480 and a
  1920x480 case that exercises the vertical-padding branch. Each test asserts the
  fix **and** the old path's corruption, so the discrimination is part of the
  suite rather than a one-off observation.
- `scripts/m4/verify_m4_3_geometry.py` - replays one real camera frame through the
  engine, restores it both ways, and compares against the mask the node actually
  published. Measured: fixed path agrees **1.0000**, old path **0.7071** (left
  band 1.0000 vs 0.5524).

## Measured performance

TensorRT baseline (`trtexec`, idle box): **94.7 qps**, GPU compute
**10.51 ms** mean / 10.51 ms median, total latency **11.06 ms**.

With only the camera and `segmentation_node` running, both mask topics sustain
**30.0 Hz** - the full camera rate.

With all three modules (4.1 + 4.2 + 4.3) up, the pipeline is **CPU-bound** at
roughly 11-16 Hz per topic (load ~5 on 8 cores, GPU only 35-62%). That is a
whole-pipeline limit, not a segmentation limit. Full numbers in
`output/m4/4.3/benchmark.json`.

## PyTorch <-> TensorRT parity

`scripts/m4/verify_segformer_parity.py` plus `test/parity_runner.cpp`. torch is
only importable from conda py310 and the `tensorrt` bindings only from the system
interpreter, so the check is split across two processes joined over raw buffers.
The runner links the same `bev_segmentation_core` the node uses, so a green result
covers the shipped inference path.

Measured on a real 1920x1080 frame: pixel agreement **0.9973**, mean IoU
**0.9813** (thresholds 0.95 / 0.85).

## QoS

The image subscription and **both mask publishers** use `rclcpp::SensorDataQoS()`
(BEST_EFFORT), matching 4.1/4.2. The publishers previously used a bare depth of
10, i.e. RELIABLE, which was the odd one out.

The only consumer, `m4_demo_bringup/segmentation_visualizer.py`, subscribes with
BEST_EFFORT - so this change removes an odd pairing rather than creating an
incompatibility. A RELIABLE subscriber would be incompatible and receive nothing.

## Drivable-class policy

Default `drivable_class_ids: [0]` = **road only**. Sidewalk (class 1) is *not*
included; pass `[0, 1]` to include it. The mask is a *candidate drivable*
semantic mask and must not be treated as collision-free space.

## Validation evidence

`output/m4/4.3/` (gitignored): `parity.json`, `geometry.json`, `benchmark.json`,
`m4_3_overlay.png`, `m4_3_source.png`, `m4_3_semantic_palette.png`, `logs/`.

## Known debt

- **`/m4/3` does not deep-link to the 4.3 tab.** In hub mode the server renders
  `data-demo="hub"` for every `/m4/<n>` route, so all of them open on the same
  default tab. The 4.3 tab itself works; the URL just does not preselect it.
- **`launch/m4_segmentation.launch.py` declares an `image_republisher` node it
  never adds** to the LaunchDescription (dead code).
- **`m4_3_demo.launch.py` cannot run at all**: its `IfCondition('$(eval ...)')` is
  not valid ROS 2 launch syntax and raises `invalid condition expression`. The
  same bug is in `m4_2_demo.launch.py`. Both demos work because
  `run_m4_web_hub.sh` uses `m4_all_demo.launch.py` instead, but
  `run_m4_3_demo.sh` calls the broken one.
- **`run_m4_3_benchmark.sh` would start a second `segmentation_node`** while the
  hub's is running (duplicate publisher on `/perception/semantic_mask`), and it
  reads `output/m4/4.3/test_input.png`, which has never existed. The numbers above
  were measured from the live pipeline instead.
- Model licence is **unspecified** (`license: other` with no terms). Not
  redistributable; see `models/m4/segmentation/LICENSE.md`.