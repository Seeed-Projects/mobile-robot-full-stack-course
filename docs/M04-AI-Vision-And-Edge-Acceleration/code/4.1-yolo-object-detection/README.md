# 4.1 YOLO Object Detection (TensorRT)

Training -> ONNX -> TensorRT engine -> ROS 2 detection topic, at ~30 Hz on Orin.

**Status: PASS - verified on physical hardware.**

## Pipeline

```
GMSL front camera -> /dev/video0 (V4L2)
   -> GStreamer -> sensor_msgs/Image on /perception/cameras/front/image
   -> yolo_trt_node (C++ / TensorRT 10.3, FP16, 640x640)
   -> vision_msgs/Detection2DArray on /perception/detections
```

## Source

`ros2/bev_detection/` - C++ package.

| Target | Role |
| --- | --- |
| `yolo_engine` (shared lib) | TensorRT runtime: deserialize + bind + infer + decode/NMS |
| `yolo_trt_node` | ROS 2 node: Image in -> Detection2DArray out |
| `camera_adapter_node` | `bev_interfaces/FrameSet` -> front `Image` (legacy GMSL frameset bridge) |

Key files: `src/yolo_engine.cpp` (inference + `YoloPostprocess::parse`),
`src/yolo_trt_node.cpp` (ROS glue), `include/bev_detection/types.hpp`
(`LetterBox::restore`), `config/yolo.yaml`.

## Model preparation

See [`models/m4/detection/README.md`](../models/m4/detection/README.md).
Artifacts (not committed): `models/m4/detection/engines/yolo11n_fp16.engine`
(8,546,556 B), `onnx/yolo11n.onnx`, `labels/coco.names`.

## Launch / config

```bash
ros2 launch bev_detection yolo.launch.py            # detection only
ros2 launch bev_detection m4_detection.launch.py    # + camera sync/adapter (legacy GMSL)
```

Parameters (`config/yolo.yaml`): `model_path`, `class_names_path`, `image_topic`,
`detections_topic`, `input_width/height` (640), `num_classes` (80),
`confidence_threshold` (0.25), `nms_threshold` (0.45), `publish_debug_image`.

## Demo

```bash
./scripts/m4/run_m4_1_demo.sh                 # auto camera resolution
CAMERA_SOURCE=test ./scripts/m4/run_m4_1_demo.sh   # synthetic, no hardware
./scripts/m4/run_m4_1_benchmark.sh            # writes output/m4/4.1/benchmark.json
```

## Measured performance (2026-09-18, course workspace)

| Metric | Value |
| --- | --- |
| Camera rate | 29.7 fps (1920x1080) |
| `/perception/detections` | 27.7-28.6 Hz |
| GPU inference | ~3.4 ms (cudaEvent, GPU only) |
| End-to-end | ~18 ms |

> Historical docs quoted 48.3 Hz / 19 ms / 105 ms. Those came from a sample
> benchmark JSON that no artifact matches (`benchmark.json` actually records
> `inference_rate_hz: 0.0` - a measurement failure). Treat the table above as the
> only supported numbers.

## Contract

- `header.stamp` and `frame_id` are copied from the input image (never re-stamped).
- Empty frames still publish an empty `Detection2DArray` (M4.2 depends on this).
- `Detection2D.id` is deliberately left `""` - M4.2 fills it with the track id.
- Bounding boxes are in **original image pixels** (letterbox is inverted before publish).
- QoS: `SensorDataQoS` (BEST_EFFORT) on the image sub and both publishers.

## Troubleshooting

| Symptom | Cause |
| --- | --- |
| `Engine not ready, skipping frame` (every 2 s) | engine path wrong or deserialization failed - it will NOT fall back to ONNX/PyTorch |
| No boxes but the image is fine | `confidence_threshold` too high, or the objects are not COCO-80 classes |
| `No module named 'std_msgs'` when running tests | source `/opt/ros/humble/setup.bash` first |

## Known technical debt

- `src/preprocessing.cpp::letterboxPreprocess()` and the standalone
  `src/postprocessing.cpp` helpers are **dead code** - the live path re-implements
  that logic inside `yolo_engine.cpp`. The two copies have diverged: the standalone
  `parseYoloOutput` returns empty even for high-confidence input, and
  `test_postprocessing.cpp::ParseYoloOutput.ConfidenceFilterGatesBelowThreshold`
  fails for exactly that reason. Not on the runtime path, but it should be deleted
  or fixed.
- The camera publisher is `test/csi_camera_publisher.py` - a test-tree helper that
  is the de-facto production camera source for 4.1-4.3. It drives a GMSL V4L2 node
  despite the `--source csi` flag name.
