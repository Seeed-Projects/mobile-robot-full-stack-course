# 4.1 YOLO Object Detection: From Training to TensorRT Deployment

**Status: PASS.** Jetson A was revalidated with YOLO11n FP16 TensorRT, the real ROS 2 topics, and 28 GTests. See [`code/PROJECT_STATUS.md`](../code/PROJECT_STATUS.md) for the canonical evidence and environment.

## Course Overview

![Course Overview](./images/ZDIIbrRovoXY93x5OKAczK2NnNd.png)

A camera can hand you a whole image of pixels, yet it does not tell you what is in the frame or where it is. Object detection answers exactly these two questions: what class each target belongs to, and where it sits in pixel coordinates.

By the end of 2.4, you had a bird's-eye view that refreshes in real time; that image answers "how the ground runs", but it cannot answer "what is on the road". M4 starts from detection to add this layer.

This page is not about "training a model", but about "wiring an already-trained model into the robot": the ROS 2 package `bev_detection` loads a YOLO11n TensorRT engine, turns camera images into `vision_msgs/Detection2DArray`, and publishes them on `/perception/detections`, ready for 4.2's tracking to consume directly. The model itself comes from Ultralytics' COCO pretrained weights; this page does not train it, it only deploys it into a reliable detection pipeline.

After walking through it, you will have and understand these things: a detection pipeline you can actually run, an accurate grasp of every contract of `/perception/detections` (message type, QoS, timestamp inheritance, empty-frame behavior), and a reproducible set of performance measurement conventions.

### Before You Start: What This Lesson Will Walk You Through

| Stage | What you will understand | What you can ultimately do |
| --- | --- | --- |
| Read | What the model's `[1, 84, 8400]` output is, and what question confidence, IoU, and mAP each answer | Read detection results, no longer treating post-processing as a black box |
| Deploy | The three-stage PyTorch → ONNX → TensorRT export, and how an engine is bound to hardware and versions | Explain clearly whether one engine can be used on another machine, and know why the input shape is fixed |
| Integrate | How detection results become ROS 2 topics: message type, QoS, timestamp, empty frames | Read every contract of `/perception/detections` and know what downstream relies on |
| Verify | How the detection pipeline's performance should be measured and which conditions to record | Produce a reproducible, comparable measurement record |

### Learning Outcomes

- Explain the meaning of every dimension in the YOLO output tensor `[1, 84, 8400]`, and how the number 8400 is derived.

- Understand Letterbox scaling and padding, and why the inverse transform must reuse the same `s`, `dw`, and `dh`.

- Explain the differences in convention among IoU, mAP@0.5, mAP@0.5:0.95, and Precision / Recall, and know which conditions must accompany any reported mAP.

- Explain what each of the three stages PyTorch → ONNX → TensorRT solves, and why an engine is bound to the GPU architecture and TensorRT version.

- Run the `bev_detection` detection pipeline on the J501, and use `ros2 topic` to verify the output message type, QoS, and timestamp.

- Explain the boundary of responsibility between M4.1 and M4.2: why the detection node does not fill in `id`, and why empty frames must also be published.

- Use the on-device scripts to produce a performance measurement record with its conditions, rather than a lone frame-rate number.

### Hardware and Software Checklist

**Hardware:**

| ![Hardware and Software Checklist](./images/QDs8bua4qovkoAx39qycbjMYnxc.png) | ![Hardware and Software Checklist](./images/UbdAbukKyoS7v8xWel8cSOOunSb.png) | ![Hardware and Software Checklist](./images/TgphbD4TdoCCcvxaLOjci7nhnte.png) |
| --- | --- | --- |

**Software:**

- Platform: reComputer J501 (Jetson AGX Orin 32GB, MAXN mode); JetPack 6.2.1 (L4T 36.4.4), Ubuntu 22.04, CUDA 12.6, TensorRT 10.3.x, ROS 2 Humble

- Inference stack: `bev_detection` is a C++ package that runs the engine directly with the TensorRT runtime, without depending on PyTorch / Ultralytics. The TensorRT version must be on `10.3.x`; `ros2_ws/src/bev_detection/config/yolo.yaml` declares `expected_trt_version: "10.3"` for version checking

- Model artifacts: `models/m4/detection/engines/yolo11n_fp16.engine` (FP16), `models/m4/detection/labels/coco.names` (80 COCO classes). This engine was built by exporting Ultralytics' `yolo11n.pt` to ONNX and then building it; the build commands are recorded in `docs/M4.1_YOLO_TENSORRT.md`; **the model is not retrained on this machine**

- ROS 2 packages depended on: `vision_msgs` (message types), `cv_bridge`, `image_transport`

### Prerequisites

- The camera path already works and you can get a stable video stream: for the bring-up procedure (device tree, `media-ctl`, `v4l2-ctl`, multi-camera FSYNC synchronization), see [2.1 GMSL2: Automotive-Grade Multi-Camera Integration](https://seeedstudio.feishu.cn/docx/Takhd7wo5oPx3mx0ljhcfyxDnEe);

- The system is already flashed: follow [1.2 JetPack 6.2 System Flashing and Basic Configuration](https://seeedstudio.feishu.cn/docx/UweQdPUKYobmfMxYjZkcy1ZpnNh)

- You can use `ros2 topic list` / `ros2 topic echo` / `ros2 topic info -v` to look at topics and QoS. This chapter does not require you to write ROS 2 nodes.

- General basics: basic Python 3 usage (reading tensor shapes, looking at array slices).

## Read First: From an Image to a Detection Box

### What the Model Outputs: Center Point, Width and Height, and Confidence

![What the Model Outputs: Center Point, Width and Height, and Confidence](./images/X0IebHJTPocTj3x7x0LcxEgRnHh.gif)

YOLO's output is not "one box" but a dense prediction map. Taking a 640×640 input and COCO's 80 classes as an example, the model's output tensor has shape `[1, 84, 8400]`: 84 = 4 box parameters + 80 class scores, and 8400 = 80² + 40² + 20², that is, the total number of prediction points at the three strides 8 / 16 / 32.

The box parameters are not the top-left and bottom-right corners directly, but the center point (cx, cy) and the width and height (w, h), in units of pixels of the network input image (640×640). The maximum among the class scores is what is usually called confidence; when it is below the `conf` threshold, this prediction point together with its box is discarded. So the first step of the post-processing you see is always "screen out the vast majority of prediction points by score".

This tensor is laid out **channel-major**: the i-th prediction point of channel c is at `c * 8400 + i`. This is the easiest place to make a mistake when writing parsing code; reading it as `output + i * stride` (anchor-major) gives results that look plausible but are completely wrong.

**One-line memory aid:** confidence answers "is there a target here"; IoU answers "does this box fit the target accurately".

### Metric Definitions: IoU, mAP, Precision / Recall, Confusion Matrix

![Metric Definitions: IoU, mAP, Precision / Recall, Confusion Matrix](./images/THqIbcVPuovArkxO4VAcgyhdngh.png)

Intersection over Union (IoU) measures the degree of overlap between a predicted box and a ground-truth box; it is the threshold for deciding "correct detection / wrong detection", and the source of the threshold in every mAP metric.

$\mathrm{IoU} = \frac{|A \cap B|}{|A \cup B|}$ , **where A is the predicted box and B is the ground-truth box**; the numerator is the area of their intersection and the denominator is the area of their union. The closer IoU is to 1, the more the two boxes coincide; in engineering, IoU ≥ 0.5 is taken as the minimum threshold for "counting as a correct detection", and that is where the 0.5 in mAP@0.5 comes from.

![Metric Definitions: IoU, mAP, Precision / Recall, Confusion Matrix](./images/MTT0bEIBdoVIKuxTsMccUISwnPb.png)

- **mAP@0.5**: fix the IoU threshold at 0.5, compute the area under the Precision–Recall curve for each class (Average Precision, AP), then average the AP over all classes.

- **mAP@0.5:0.95**: take the IoU threshold in steps of 0.05 from 0.5 to 0.95, giving 10 mAP@0.5:x values, and average them. It is much more sensitive to box localization accuracy; the COCO main-line metric uses it, and this page's accuracy comparisons also follow it.

- **Precision**: P = TP / (TP + FP), answering "how many of the boxes you reported are real". It rises as the `conf` threshold increases.

- **Recall**: R = TP / (TP + FN), answering "how many of the targets that truly exist in the frame you found". It falls as the `conf` threshold increases.

- **F1 score**: the harmonic mean of P and R, used to make the trade-off at a single threshold.

$F1 = \frac{2 \cdot P \cdot R}{P + R}$

- **Confusion matrix**: rows are ground-truth classes and columns are predicted classes. The diagonal is the count of correct detections, and the off-diagonal tells you which two classes are being mistaken for each other; after validation, look at this table first, then decide whether to add data or tune the threshold.

There are two hard rules for reading these numbers.

First, an mAP must be reported together with four conditions: the dataset and split, `imgsz`, the precision mode, and whether post-processing is included. The mAP@0.5:0.95 evaluated from the same weights at imgsz=640 and at imgsz=1280 cannot be compared directly.

Second, the mAP computed on the framework side and the mAP computed on the TensorRT engine must be written separately: the engine's input is a tensor, and the evaluation script must reuse the same Letterbox and NMS parameters, otherwise you cannot tell whether a differing number comes from quantization or from post-processing.

A confusion matrix looks like this:

![Metric Definitions: IoU, mAP, Precision / Recall, Confusion Matrix](./images/S7ZpbxddPobXNJxVuDWc01yHnng.png)

### From PyTorch to TensorRT: Three-Stage Export

First, answer a natural question: where does the model come from?

The `yolo11n` used on this page is Ultralytics' released COCO pretrained weights, and does not need to be trained by you. If your scene classes differ greatly from COCO (for example, recognizing specific parts at a workstation), you must first fine-tune on your own data. That is a separate training workflow with no overlap with the deployment pipeline this chapter covers. This chapter starts from "you already have a model".

Deployment has three stages, each of which can be verified on its own; when something goes wrong, first locate which stage it is, then change parameters.

- **PyTorch → ONNX**: `yolo export model=yolo11n.pt format=onnx simplify=True dynamic=False`. `simplify=True` merges redundant nodes, which reduces the chance of operators unsupported at build time; `dynamic=False` fixes the input shape, in exchange for a static engine whose memory allocation can be determined at build time.

- **ONNX → TensorRT Engine**: `trtexec --onnx=yolo11n.onnx --saveEngine=yolo11n_fp16.engine --fp16 --workspace=4096`. What happens here is operator fusion, layer selection, and kernel autotuning, so the engine built from the same ONNX under different TensorRT versions and on different GPUs is not portable.

- **Determine the output contract**: the output of the engine on this machine is `output0: float32[1, 84, 8400]`, that is, the one-to-many path, so post-processing must include NMS. The choices made at export time are baked into the computation graph; passing parameters again at load time will not rebuild the graph.

**An engine is bound to the GPU architecture and the TensorRT version**, so changing devices requires rebuilding it. This is not a configuration problem but the essence of a serialized engine: what it stores is the compiled kernels and scheduling plan, not a portable intermediate representation.

After the export is done, first check that the ONNX itself is valid, then hand it to trtexec: `python3 -c "import onnx; m=onnx.load('yolo11n.onnx'); onnx.checker.check_model(m); print(len(m.graph.node))"`. If the node count differs too much from expectation, `simplify` merged operators it should not have; go back and export once more with `simplify=False`.

**One-line memory aid:** ONNX answers "what the operators are written as"; the engine answers "how to execute fastest on this GPU".

### Precision Modes: FP32 and FP16

| Precision mode | How to build | Cost |
| --- | --- | --- |
| FP32 | trtexec without a precision flag | The accuracy baseline, with the highest latency; used as the reference value for accuracy comparisons |
| FP16 | add `--fp16` | AGX Orin's Tensor Cores natively support half precision, halving both memory access and compute at once; the actual gain depends on the layer structure, so measure it under the same conditions and fill in the table |

The engine on this machine is FP16: `models/m4/detection/engines/yolo11n_fp16.engine`. Before the node loads it, you can first confirm the match between the TensorRT version and the engine; when the versions do not match, TensorRT deserialization fails outright. The `expected_trt_version: "10.3"` in `ros2_ws/src/bev_detection/config/yolo.yaml` exists for exactly this check, so that you do not discover the version mismatch only after inference fails.

To switch precision, just switch the engine file; there is no need to change the node code: `model_path` is a parameter.

### Preprocessing and Post-Processing: Letterbox and Coordinate Restoration

Preprocessing must resolve a contradiction: the network only accepts square inputs of a fixed size, while the camera supplies frames of arbitrary aspect ratio. Stretching directly would change the aspect ratio of the targets and teach box regression the wrong shape, so Letterbox is used (scale while preserving the aspect ratio + gray padding).

First compute the scale, then compute the margins; these two steps determine whether post-processing can restore the original image coordinates.

$s = \min\left(\frac{640}{W_{src}},\ \frac{640}{H_{src}}\right)$

where $W_{src}$ / $H_{src}$ are the width and height of the original image and $s$ is the scale. The new size after scaling is $(\mathrm{round}(W_{src}\cdot s),\ \mathrm{round}(H_{src}\cdot s))$, the remaining part is padded evenly left/right and top/bottom with gray value 114, and the margins are denoted $d_w$ and $d_h$. The inverse transform is simply subtracting the margins from the box coordinates output by the model and then dividing by the scale:

$x_{src} = \frac{x_{model} - d_w}{s}, \quad y_{src} = \frac{y_{model} - d_h}{s}$

The most common mistake here is treating `dw` / `dh` as the scaled size, or forgetting that the network input is laid out CHW and feeding HWC in directly. When writing code, fix these steps into a function and let the inverse transform reuse the same `s`, `dw`, and `dh`; the error then reduces to within 1 px from integer rounding.

```python
import cv2
import numpy as np

def letterbox(img, new_shape=640, color=114):
    h, w = img.shape[:2]
    s = min(new_shape / w, new_shape / h)
    nw, nh = int(round(w * s)), int(round(h * s))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((new_shape, new_shape, 3), color, dtype=np.uint8)
    dw, dh = (new_shape - nw) // 2, (new_shape - nh) // 2
    canvas[dh:dh + nh, dw:dw + nw] = resized
    return canvas, s, dw, dh

def to_nchw_bgr2rgb(canvas):
    x = canvas[:, :, ::-1].astype(np.float32) / 255.0        # BGR -&gt; RGB，归一化到 0-1
    return np.ascontiguousarray(x.transpose(2, 0, 1))[None]  # HWC -&gt; 1CHW
```

Post-processing has four steps, and the on-device `bev_detection` follows exactly this path:

1. **Confidence filtering**: among the 8400 prediction points, the class scores of the i-th point are at `output[4 + c][i]`, not "columns 5 to 84 of row i". Take each point's maximum class score according to this layout, and discard the whole point if it is below the `conf` threshold. On this machine `confidence_threshold` defaults to 0.25. If you have many missed detections, lower it to 0.1 and look again.

2. **Coordinate conversion**: convert (cx, cy, w, h) into top-left / bottom-right corners (x1, y1, x2, y2).

3. **Non-maximum suppression**: sort by score within the same class, and suppress boxes whose IoU with an already-kept box exceeds the `iou` threshold. On this machine `nms_threshold` defaults to 0.45. This step is single-threaded O(n²); with 8400 candidate points, the worst case requires about 35 million IoU computations, making it the cost most worth watching in this pipeline.

4. **Coordinate restoration and clipping**: use the two formulas above to convert back to original image coordinates, then clip to the image bounds so that boxes are not drawn outside the frame.

NMS is performed per class, so two overlapping targets of different classes are both kept; conversely, if the same target is assigned two classes, two boxes will be stacked on top of each other. If you want different classes to suppress each other as well, you can switch to class-agnostic NMS, which this page does not cover.

### What a Detection Pipeline Looks Like: Nodes, Topics, and Three Contracts

With the theory covered, look at what this pipeline becomes when it lands on the robot.

### Nodes and Topics

The `bev_detection` package provides two executables:

| Executable | Role |
| --- | --- |
| `yolo_trt_node` | Main node: subscribes to camera images, runs TensorRT inference, publishes detection results |
| `camera_adapter_node` | Optional forwarding node that converts upstream images into the namespace the detection node expects |

They are connected by three topics:

| Direction | Topic | Type |
| --- | --- | --- |
| Subscribe | `/perception/cameras/front/image` | `sensor_msgs/Image` |
| Publish | `/perception/detections` | `vision_msgs/Detection2DArray` |
| Publish | `/perception/debug/detection_image` | `sensor_msgs/Image` |

`vision_msgs/Detection2DArray` is the standard 2D detection message in the ROS 2 ecosystem. There is a deliberate choice here: detection results **reuse the standard message** rather than a custom message type of our own. The benefit is that any downstream that recognizes this type (tracking, visualization, recording) can connect directly, with no extra translation layer.

### Three Contracts (Downstream Depends on Them; They Are Not Optional)

**Contract 1: The timestamp and coordinate frame must be inherited from the source image.** The output `header.stamp` and `header.frame_id` are taken directly from the source image, and re-stamping with `now()` is **not allowed**. Tracking, depth alignment, pose estimation, sensor fusion, and rosbag playback all depend on this. Once you stamp the time yourself, every timestamp in offline playback becomes the playback moment, and the timing of the entire pipeline is ruined.

**Contract 2: Each frame that completes inference publishes exactly one message, including frames with no detected targets.** Empty frames are published too, because downstream tracking relies on "no observation in this frame" to advance a track's lost count. If the detection node chooses not to publish on an empty frame, what tracking sees is not "the target disappeared" but "there is no message at all", and these two are completely different semantics inside the tracker.

**Contract 3: The detection node does not fill in the `id` field.** Each detection item in `Detection2DArray` has an `id`, and that is reserved for tracking's track ID. If the detection node fills it in, it crosses the boundary between detection and tracking, and 4.2's tracker will be unable to distinguish "this is the same target as the previous frame" from "this is a number the detector made up itself".

### QoS: Why BEST_EFFORT

All three topics use `rclcpp::SensorDataQoS()`, equivalent to Reliability `BEST_EFFORT`, History `KEEP_LAST` (depth 10), Durability `VOLATILE`.

This is the conventional choice for sensor data: the image stream runs at 30 Hz, where dropping frames is acceptable but freshness is required; reliable transport retransmits old frames when the link is congested, which instead lets latency accumulate. The cost is that **the subscriber must also use BEST_EFFORT**; otherwise DDS refuses the connection due to QoS incompatibility, which shows up as "the publisher has data, the subscriber receives nothing".

**One-line memory aid:** detection is responsible for "what is seen in this frame", tracking is responsible for "whether this is the same as the previous frame"; the boundary between them is the `id` field and empty-frame behavior.

## Hands-On: Run the Detection Model into a ROS 2 Topic

Four steps, each with a verifiable output. All commands are executed on the J501, and the working directory is `/home/seeed/workspace/ros2_bev`. Before running, first confirm that the power mode is MAXN, otherwise the numbers measured later are not comparable.

### Step 1: Confirm the Environment, Model Artifacts, and Nodes

First confirm that the model, labels, and node are all ready, so that you do not later mistake an environment problem for an operational error.

```bash
cd /home/seeed/workspace/ros2_bev

# 模型产物
ls -l models/m4/detection/engines/yolo11n_fp16.engine   # 8,546,556 B
ls -l models/m4/detection/labels/coco.names             # COCO 80 类

# 可执行文件
ls -l ros2_ws/install/bev_detection/lib/bev_detection/yolo_trt_node

# 版本自检
python3 -c "import tensorrt as trt; print('trt', trt.__version__)"   # 10.3.0
```

The engine, labels, and executable are all indispensable. `ros2_ws/src/bev_detection/config/yolo.yaml` also declares `expected_trt_version: "10.3"`; the TensorRT on this machine is 10.3.0, so confirm that the two agree before starting. If you have changed the TensorRT version, the engine must be rebuilt.

### Step 2: Launch the Existing Demo

This step actually gets the detection pipeline running.

```bash
cd /home/seeed/workspace/ros2_bev
scripts/m4/run_m4_1_demo.sh
```

The script uses `CAMERA_SOURCE=csi` by default, pulls the stream from `/dev/video0` at 1920×1536@30, starts the camera publishing node itself, and then feeds the images to `yolo_trt_node`. The debug image is remapped to `/perception/demo/m4_1`, so that it can coexist with other pipelines for observation.

Optional switches (all environment variables): `CAMERA_SOURCE`, `CAMERA_DEVICE`, `CAMERA_WIDTH` / `CAMERA_HEIGHT` / `CAMERA_FPS`, `VIEWER`, `DURATION`.

### Step 3: Verify the `/perception/detections` Message

Next, do not just look at whether the topic exists; verify the three interface contracts one by one.

```bash
ros2 topic list | grep perception
ros2 topic info -v /perception/detections
ros2 topic hz /perception/detections
ros2 topic echo /perception/detections --once
```

Check them one by one:

- The **message type** is `vision_msgs/msg/Detection2DArray`;

- The **QoS** is `BEST_EFFORT` / `KEEP_LAST` (depth 10) / `VOLATILE`, corresponding to `SensorDataQoS` on the subscriber side;

- The **timestamp and frame_id** match the source image rather than the current time. Put the `header.stamp` of `/perception/cameras/front/image` and `/perception/detections` side by side; the two should be identical;

- **Empty frames publish messages too**: point the camera at a scene with no detectable targets and confirm that `/perception/detections` still publishes continuously, with `detections` an empty array. Note that you must not test this by "unplugging the camera" — the premise of the contract is "an input frame that completed inference", and with no input frame you cannot test this contract. The repository has a dedicated contract check script `scripts/m4/test_empty_frame_contract.sh`; use it directly when you need strict verification;

- **The `id` field is empty**: `ros2 topic echo /perception/detections --field detections[0].id` should have no valid value. Filling it in is 4.2's job.

### Step 4: Performance Measurement Method

What you finally need is a measurement that records the complete conditions, not a lone frame rate.

```bash
cd /home/seeed/workspace/ros2_bev
scripts/m4/run_m4_1_benchmark.sh 30
```

The script measures this pipeline's frame rate and latency with real camera input, and the results land in `output/m4/4.1`.

**This page does not give a target frame-rate number.** There are two layers to the reason: first, two design documents in the repository give contradictory readings for the same engine, and neither notes its measurement conditions; second, the frame rate itself strongly depends on the power mode, camera resolution, and scene. What you need to do is record the conditions completely, then measure it yourself.

A comparable record must state all seven items at once; with one missing, the number cannot be compared with anyone else's:

| # | Condition |
| --- | --- |
| 1 | Device model (on this machine, reComputer J501 / Jetson AGX Orin 32GB) |
| 2 | JetPack / L4T version (6.2.1 / R36.4.4) |
| 3 | Model and engine precision (YOLO11n / FP16) |
| 4 | Model input size (640×640) |
| 5 | Camera resolution and input frame rate (1920×1536@30) |
| 6 | Power and clock mode (`sudo nvpmodel -m 0` + `sudo jetson_clocks`) |
| 7 | Measurement convention and sampling window (including warm-up length) |

The power mode must be recorded: after switching back to 15W mode, the readings from the same engine will drop, and the two results cannot be mixed in the same table. Otherwise, changing the power mode once will produce a table that looks like "the optimization paid off" but in fact only changed the mode.

## Deliverables and Acceptance Criteria

### Deliverables Checklist

1. A running detection pipeline: `scripts/m4/run_m4_1_demo.sh` starts normally, and `/perception/detections` publishes continuously.

2. Verification records for the three contracts: a screenshot of the message type and QoS, a timestamp consistency comparison, evidence that empty frames are still published, and evidence that `id` is empty.

3. A performance measurement record: the output of `scripts/m4/run_m4_1_benchmark.sh`, plus the seven-condition table.

4. A piece of visualization evidence: a debug image or an excerpt of `ros2 topic echo`, showing detection boxes, classes, and confidences.

### Acceptance Criteria

| Check | Pass criterion | When failing, inspect first |
| --- | --- | --- |
| Environment self-check | All three of the engine, `coco.names`, and `yolo_trt_node` exist; the TensorRT version is 10.3.x | Whether `colcon build` has been run; whether the engine has been replaced with one built by a different TensorRT version |
| Message contract | The type of `/perception/detections` is `vision_msgs/msg/Detection2DArray`; the QoS is `BEST_EFFORT` / `KEEP_LAST` (depth 10) / `VOLATILE` | Whether the subscriber also uses `SensorDataQoS` (see "downstream receives nothing" in troubleshooting) |
| Timestamp inheritance | The detection message's `header.stamp` matches the source image | Whether someone re-stamped the timestamp with `now()` |
| Empty-frame behavior | The topic is still publishing in a scene with no targets, with `detections` an empty array | Whether the node returns early on an empty box set |
| Boundary of responsibility | `detections[i].id` is empty | Whether tracking's ID logic was mistakenly written into the detection node |
| Coordinate restoration | Verify with a target at a known pixel position; the restoration error is within 1 px | Whether `dw` / `dh` used the scaled size; whether `s` was recomputed |
| Measurement record | All seven conditions are stated, with the raw script output attached | Whether only the frame rate was copied and the conditions dropped |

## FAQ and Troubleshooting

### Detection Boxes Are Shifted Overall, or the Scale Is Wrong

- **Symptom**: box positions are systematically shifted to one side, or the boxes are a size larger / smaller than the target, while classes and confidences are all normal.

- **Cause**: the Letterbox inverse transform used the wrong `dw` / `dh`, or the image was fed into the network directly as HWC, or the scale was recomputed using the stretched size.

- **Solution**: first print the shapes of the model's input and output tensors to confirm the layout; then draw a square image of known size (for example a 100×100 pixel right-angle marker) and run it through the whole pipeline, checking whether the restored box returns to its original position, with an error within 1 px. When fixing it, let preprocessing and post-processing share the same `s`, `dw`, and `dh`.

### Frame Rate Far Below Expectation

- **Symptom**: with an engine from elsewhere, or on another machine, the frame rate differs greatly for the same input.

- **Cause**: most likely it is not inference. The common cases are preprocessing resizing frame by frame on the CPU, waiting synchronously for the GPU every frame, actually loading an FP32 engine, or post-processing looping over 8400 prediction points in Python.

- **Solution**: use `jtop` to look at GPU utilization; below 50% you can basically conclude that the bottleneck is the CPU or a synchronous wait. Time the three stages of preprocessing, inference, and post-processing per frame, and fix the longest one first. Confirm that the node loads `yolo11n_fp16.engine` and not an FP32 engine you substituted yourself.

### The Camera Pipeline Drops Frames

- **Symptom**: the video comes out but stutters now and then, or the timestamps of the frames you read jump around a lot.

- **Cause**: too much camera buffering, format conversion done on the CPU, or the actually achievable frame rate being below the configured value.

- **Solution**: first verify the camera pipeline and inference separately: run only the camera publishing node and confirm that it can steadily reach the target frame rate, then add detection on top. If you troubleshoot the two mixed together, you will never be able to tell whether it is a camera problem or an inference problem.

### `/perception/detections` Has Data, but Downstream Receives Nothing

- **Symptom**: `ros2 topic hz /perception/detections` looks normal, but a subscriber you wrote yourself receives not a single message, and reports no error either.

- **Cause**: **QoS incompatibility**. The publisher uses `SensorDataQoS` (BEST_EFFORT); if the subscriber uses the default RELIABLE, DDS decides that the two cannot connect, simply does not establish a connection, and reports no error.

- **Solution**: use `ros2 topic info -v /perception/detections` to confirm the publisher's QoS, and change the subscriber to `rclcpp::SensorDataQoS()` (C++) or `qos_profile_sensor_data` (Python). This is the most common pitfall when subscribing to real-time sensor topics.

### Empty Detection Frames Cause Abnormal Track Counts Downstream

- **Symptom**: after wiring detection into 4.2, a track breaks off immediately when the target is occluded, as if `lost_track_buffer` were not working.

- **Cause**: the detection node chooses not to publish on frames where `detections` is empty, or downstream treats "an empty array" as "no message received".

- **Solution**: confirm that the detection node **publishes every frame**, and that an empty frame publishes an empty array; downstream must also treat an empty array as a valid observation to advance the lost count. This contract has a dedicated check in M4.1's acceptance criteria; when modifying the detection node, do not casually add "early return on an empty box set".

> **Next step:** hand this page's `/perception/detections` to 4.2 multi-object tracking. Tracking does not retrain the detector and does not change detection results; it is only responsible for assigning a stable ID to the boxes of the same target. Three things on the detection side directly affect tracking performance: the coordinate accuracy of the boxes, the `iou` threshold, and whether empty frames are still published. Two boundaries to remember: an engine is bound to the GPU architecture and TensorRT version, so changing devices requires rebuilding it; the detection node does not fill in `id`, which is tracking's responsibility.
