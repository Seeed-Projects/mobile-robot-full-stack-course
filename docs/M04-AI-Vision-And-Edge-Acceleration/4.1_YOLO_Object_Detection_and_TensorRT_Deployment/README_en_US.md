# 4.1 YOLO Object Detection: From a Pretrained Model to TensorRT

## Overview

![Course Overview](./images/ZDIIbrRovoXY93x5OKAczK2NnNd.png)

A camera can hand you a whole image of pixels, yet it does not tell you what is in the frame or where it is. Object detection answers exactly these two questions: what class each target belongs to, and where it sits in pixel coordinates. By the end of 2.4, you had a bird's-eye view that refreshes in real time; that image answers "how the ground runs", but it cannot answer "what is on the road". M4 starts from detection to add this layer, and this chapter takes the widely used YOLO object detector as its entry point.

### What This Lesson Will Walk You Through

| Stage     | What you will understand                                                                              | What you can ultimately do                                                                               |
| --------- | ----------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| Read      | What the model's `[1, 84, 8400]` output is, and what question confidence, IoU, and mAP each answer    | Read detection results, no longer treating post-processing as a black box                                |
| Deploy    | The three-stage PyTorch → ONNX → TensorRT export, and how an engine is bound to hardware and versions | Explain clearly whether one engine can be used on another machine, and know why the input shape is fixed |
| Integrate | How detection results become ROS 2 topics: message type, QoS, timestamp, empty frames                 | Read every key design of `/perception/detections` and know what downstream relies on                     |
| Verify    | How the detection pipeline's performance should be measured and which conditions to record            | Produce a reproducible, comparable measurement record                                                    |

### Learning Outcomes

- Explain the meaning of every dimension in the YOLO output tensor `[1, 84, 8400]`, and how the number 8400 is derived.

- Understand Letterbox scaling and padding, and why the inverse transform must reuse the same `s`, `dw`, and `dh`.

- Explain the differences in convention among IoU, mAP@0.5, mAP@0.5:0.95, and Precision / Recall, and know which conditions must accompany any reported mAP.

- Explain what each of the three stages PyTorch → ONNX → TensorRT solves, and why an engine is bound to the GPU architecture and TensorRT version.

- Run the `bev_detection` detection pipeline on the J501, and use `ros2 topic` to verify the output message type, QoS, and timestamp.

- Explain the boundary of responsibility between M4.1 and M4.2: why the detection node does not fill in `id`, and why empty frames must also be published.

- Use the on-device scripts to produce a performance measurement record with its conditions, rather than a lone frame-rate number.

### Hardware and Software Checklist

- Platform: reComputer Robotics J501 (Jetson AGX Orin 32GB)
- JetPack 6.2.1
- ROS 2 Humble
- GMSL or USB camera

### Prerequisites

- The system is already flashed: follow [1.2 JetPack 6.2 System Flashing and Basic Configuration](https://seeedstudio.feishu.cn/docx/UweQdPUKYobmfMxYjZkcy1ZpnNh)
- You can use `ros2 topic list` / `ros2 topic echo` / `ros2 topic info -v` to look at topics and QoS. This chapter does not require you to write ROS 2 nodes.
- General basics: basic Python 3 usage (reading tensor shapes, looking at array slices).

### Runtime Preview

On the Jetson, run `./modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_1_demo.sh` from `/home/seeed/workspace/ros2_bev` to run detection standalone. To switch modules in a browser, run `./modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_web_hub.sh`, open `http://<Jetson-IP>:8080/m4/1`, and select “4.1 Detection.” Both entries publish `/perception/detections`, so do not start two camera pipelines at the same time.

![M4.1 running on the Jetson's physical indoor camera, with detection labels and scores](./images/m4_runtime_m41_detection.png)

## Read First: From an Image to a Detection Box

### What the Model Outputs: Center Point, Width and Height, and Confidence

![What the Model Outputs: Center Point, Width and Height, and Confidence](./images/X0IebHJTPocTj3x7x0LcxEgRnHh.gif)

YOLO's output is not a handful of final boxes directly; it first produces a large number of candidate predictions at several scales. Taking **YOLO11, a 640×640 input, and COCO's 80 classes** as an example, the usual raw inference output tensor has shape:

`[1, 84, 8400]`

The three dimensions mean:

- `1`: batch size;

- `84 = 4 + 80`: 4 box parameters + 80 class scores;

- `8400`: the total number of prediction positions.

![YOLO11 output layout](./images/yolo11.png)

The 8400 prediction positions come from three different scales: `80×80 + 40×40 + 20×20 = 8400`, corresponding to feature layers at strides 8, 16, and 32. In other words, YOLO predicts on feature maps of different resolutions at the same time: higher-resolution layers are better at finding small targets, while lower-resolution layers have a larger receptive field. For every prediction position, the model ultimately gives:

`[cx, cy, w, h, class_0, class_1, ..., class_79]`

Here `cx` and `cy` are the center point of the predicted box, and `w` and `h` are its width and height. In the usual YOLO11 inference output these box parameters are already decoded and usually correspond to the network input image coordinate system, for example pixel coordinates under a 640×640 input. The 80 values that follow are the **model's predicted scores for each class**. For a candidate box, the largest class score is usually taken as that box's confidence, with the corresponding class as the predicted class. If that score is below the configured `conf` threshold, the candidate box can be discarded directly. So the 8400 candidates do not all reach the final result. A typical post-processing flow is:

`8400 candidates → confidence filtering → NMS → final detection boxes`

### Metric Definitions: IoU, mAP, Precision / Recall, Confusion Matrix

![Metric Definitions: IoU, mAP, Precision / Recall, Confusion Matrix](./images/THqIbcVPuovArkxO4VAcgyhdngh.png)

Intersection over Union (IoU) measures the degree of overlap between a predicted box and a ground-truth box; it is the threshold for deciding "correct detection / wrong detection", and the source of the threshold in every mAP metric.

$\mathrm{IoU} = \frac{|A \cap B|}{|A \cup B|}$, **where A is the predicted box and B is the ground-truth box**; the numerator is the area of their intersection and the denominator is the area of their union. The closer IoU is to 1, the more the two boxes coincide. mAP@0.5 uses an IoU matching threshold of 0.5.

![Metric Definitions: IoU, mAP, Precision / Recall, Confusion Matrix](./images/MTT0bEIBdoVIKuxTsMccUISwnPb.png)

- **mAP@0.5**: fix the IoU threshold at 0.5, compute the area under the Precision–Recall curve for each class (Average Precision, AP), then average the AP over all classes.

- **mAP@0.5:0.95**: average the results over ten IoU thresholds from 0.5 to 0.95 in steps of 0.05. It is more sensitive to box localization accuracy and is a common COCO headline metric.

- **Precision**: P = TP / (TP + FP), answering "how many of the boxes you reported are real". It rises as the `conf` threshold increases.

- **Recall**: R = TP / (TP + FN), answering "how many of the targets that truly exist in the frame you found". It falls as the `conf` threshold increases.

### From PyTorch to TensorRT: Three-Stage Export

First, answer a natural question: where does the model come from? The `yolo11n` used on this page is Ultralytics' released COCO pretrained weights, and does not need to be trained by you. If your scene classes differ greatly from COCO (for example, recognizing specific parts at a workstation), you must first fine-tune on your own data. That is a separate training workflow with no overlap with the deployment pipeline this chapter covers. This chapter starts from "you already have a model".

Deployment has three stages, each of which can be verified on its own; when something goes wrong, first locate which stage it is, then change parameters.

1. **PyTorch → ONNX**:

```bash
yolo export model=yolo11n.pt format=onnx simplify=True dynamic=False
```

- **simplify=True** merges redundant nodes, which reduces the chance of operators unsupported at build time;
- **dynamic=False** fixes the input shape, in exchange for a static engine whose memory allocation can be determined at build time.

2. **ONNX → TensorRT Engine**:

```bash
trtexec --onnx=yolo11n.onnx --saveEngine=yolo11n_fp16.engine --fp16 --memPoolSize=workspace:4096
```

What happens here is operator fusion, layer selection, and kernel autotuning, so the engine built from the same ONNX under different TensorRT versions and on **different GPUs is not portable!**

3. **Determine the output contract**: the output of the engine on this machine is `output0: float32[1, 84, 8400]`, that is, the one-to-many path, so post-processing must include NMS. The choices made at export time are baked into the computation graph; passing parameters again at load time will not rebuild the graph.

**An engine is bound to the GPU architecture and the TensorRT version**, so changing devices requires rebuilding it. This is not a configuration problem but the essence of a serialized engine: what it stores is the compiled kernels and scheduling plan, not a portable intermediate representation.

After the export is done, first check that the ONNX itself is valid, then hand it to trtexec:

```bash
python3 -c "import onnx; m=onnx.load('yolo11n.onnx'); onnx.checker.check_model(m); print(len(m.graph.node))"
```

If the node count differs too much from expectation, `simplify` merged operators it should not have; go back and export once more with `simplify=False`.

**One-line memory aid**: ONNX answers "**what the operators are written as**"; the engine answers "**how to execute fastest on this GPU**".

### Precision Modes: FP32 and FP16

| Precision mode | How to build                     | Cost                                                                                                                                                                                                                   |
| -------------- | -------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| FP32           | trtexec without a precision flag | The accuracy baseline, with the highest latency; used as the reference value for accuracy comparisons                                                                                                                  |
| FP16           | add `--fp16`                     | AGX Orin's Tensor Cores natively support half precision, halving both memory access and compute at once; the actual gain depends on the layer structure, so measure it under the same conditions and fill in the table |

### Preprocessing and Post-Processing: Letterbox and Coordinate Restoration

![Letterbox scaling](./images/resize.png)

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
    x = canvas[:, :, ::-1].astype(np.float32) / 255.0        # BGR -> RGB, normalized to 0-1
    return np.ascontiguousarray(x.transpose(2, 0, 1))[None]  # HWC -> 1CHW
```

Post-processing has four steps, and the on-device `bev_detection` follows exactly this path:

![Non-maximum suppression (NMS)](./images/nms.png)

1. **Confidence filtering**: among the 8400 prediction points, the class scores of the i-th point are at `output[4 + c][i]`, not "columns 5 to 84 of row i". Take each point's maximum class score according to this layout, and discard the whole point if it is below the `conf` threshold. On this machine `confidence_threshold` defaults to 0.25. If you have many missed detections, lower it to 0.1 and look again.

2. **Coordinate conversion**: convert (cx, cy, w, h) into top-left / bottom-right corners (x1, y1, x2, y2).

3. **Non-maximum suppression**: sort by score within the same class, and suppress boxes whose IoU with an already-kept box exceeds the `iou` threshold. On this machine `nms_threshold` defaults to 0.45. This step is single-threaded O(n²); with 8400 candidate points, the worst case requires about 35 million IoU computations, making it the cost most worth watching in this pipeline.

4. **Coordinate restoration and clipping**: use the two formulas above to convert back to original image coordinates, then clip to the image bounds so that boxes are not drawn outside the frame.

NMS is performed per class, so two overlapping targets of different classes are both kept; conversely, if the same target is assigned two classes, two boxes will be stacked on top of each other. If you want different classes to suppress each other as well, you can switch to class-agnostic NMS, which this page does not cover.

### The Full Picture: How This Pipeline Runs

With the theory covered, do not rush into typing commands yet. This figure is the full picture of the `bev_detection` pipeline:

![Detection pipeline overview: the journey of one frame and model preparation](./images/m4_1_detection_pipeline.png)

**1. The camera produces images.** The GMSL camera keeps producing frames from `/dev/video0`, in the raw YUY2 format.

**2. Decode and publish.** `csi_camera_publisher.py` uses GStreamer to decode YUY2 into BGR color images, packs them as `sensor_msgs/Image`, and publishes them on `/perception/cameras/front/image`. From that moment on, any node can subscribe to this image stream. If the upstream is not this script (for example, another module publishes under a different image namespace), you can interpose a `camera_adapter_node`, which only forwards and takes no part in inference.

**3. The detection node takes the order.** Each time `yolo_trt_node` receives a frame, it does three things:

- **Preprocessing (CPU)**: Letterbox scaling to 640×640, gray padding, BGR to RGB — putting a large photo into a square frame, with the scale and margins remembered for restoration later;
- **Inference (GPU)**: the TensorRT engine takes `1×3×640×640` and outputs `1×84×8400` — 8400 candidate positions, each reporting box coordinates and 80 class scores;
- **Post-processing (CPU)**: confidence filtering (default 0.25), NMS de-duplication (default 0.45), then restoring boxes to the original image coordinates using the scale and margins recorded during preprocessing.

**4. Publish the results.** The detected boxes are packed as `vision_msgs/Detection2DArray` and published on `/perception/detections`, the standard ROS 2 2D detection message that downstream can consume directly. This message carries three deliberate designs: the timestamp inherits the source image instead of being re-stamped, an empty array is published even when there is no target, and `id` is left empty. The first two are easy to understand; `id` is reserved for the track ID of the 4.2 tracker — if the detection node filled it in, tracking could not tell "the old target from the previous frame" apart from "a number the detector made up itself".

**5. Hand over to tracking (dashed).** Here the detection node's responsibility ends: it only answers "what was seen in this frame". "Is this the same target as the previous frame" is taken over by 4.2's ByteTrack, which fills in a stable track `id` for every box.

The branch on the right shows where the engine comes from: `yolo11n.pt → yolo11n.onnx → trtexec --fp16 → yolo11n_fp16.engine`. The engine is an artifact compiled for this GPU; changing the device or the TensorRT version requires rebuilding it. On the main path, every frame uses it and never touches the original model again.

One last note on QoS: the image topics all use `SensorDataQoS` (BEST_EFFORT) — dropping one frame does not matter, while retransmitting old frames only lets latency accumulate; the subscriber must use it too, otherwise DDS will not establish a connection, which shows up as "the publisher has data, the subscriber receives nothing". The troubleshooting section has a dedicated item on this.

With the full picture in view, the next step is to get the pipeline running and verify each design with `ros2 topic`.

## Hands-On: Run the Detection Model into a ROS 2 Topic

Four steps, each with a verifiable output. All commands are executed on the J501, and the working directory is `/home/seeed/workspace/ros2_bev`. Before running, first confirm that the power mode is MAXN, otherwise the numbers measured later are not comparable.

### Step 1: Confirm the Environment, Model Artifacts, and Nodes

First confirm that the model, labels, and node are all ready, so that you do not later mistake an environment problem for an operational error.

```bash
cd /home/seeed/workspace/ros2_bev

# Model artifacts
ls -l modules/m04-ai-vision-and-edge-acceleration/models/m4/detection/engines/yolo11n_fp16.engine
ls -l modules/m04-ai-vision-and-edge-acceleration/models/m4/detection/labels/coco.names

# Installed executable
ls -l install/bev_detection/lib/bev_detection/yolo_trt_node

# Check the installed version
python3 -c "import tensorrt as trt; print('trt', trt.__version__)"   # 10.3.0
```

The engine, labels, and executable are all indispensable. The Jetson config at `modules/m04-ai-vision-and-edge-acceleration/4.1-yolo-object-detection/ros2/bev_detection/config/yolo.yaml` also declares `expected_trt_version: "10.3"`; if the version does not match, the engine must be rebuilt on the target device. The course snapshot under `code/ros2_ws` is a different packaging layout and cannot be applied directly to the Jetson runtime tree.

### Step 2: Launch the Existing Demo

This step actually gets the detection pipeline running.

```bash
cd /home/seeed/workspace/ros2_bev
./modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_1_demo.sh
```

The script uses `CAMERA_SOURCE=csi` by default, reads `/dev/video0`, and publishes 1920×1080@30 images by default. It then feeds the images to `yolo_trt_node` and publishes the debug image on `/perception/demo/m4_1`.

Optional switches (all environment variables): `CAMERA_SOURCE`, `CAMERA_DEVICE`, `CAMERA_WIDTH` / `CAMERA_HEIGHT` / `CAMERA_FPS`, `VIEWER`, `DURATION`.

### Step 3: Verify the `/perception/detections` Message

Next, do not just look at whether the topic exists; verify the three deliberate designs from the full picture one by one.

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

- **Empty frames publish messages too**: point the camera at a scene with no detectable targets and confirm that `/perception/detections` still publishes continuously, with `detections` an empty array. Unplugging the camera does not test this design, because its premise is "an input frame that completed inference". For a strict check, use `./modules/m04-ai-vision-and-edge-acceleration/scripts/m4/test_empty_frame_contract.sh`;

- **The `id` field is empty**: `ros2 topic echo /perception/detections --field detections[0].id` should have no valid value. Filling it in is 4.2's job.

### Step 4: Performance Measurement Method

What you finally need is a measurement that records the complete conditions, not a lone frame rate.

```bash
cd /home/seeed/workspace/ros2_bev
./modules/m04-ai-vision-and-edge-acceleration/scripts/m4/run_m4_1_benchmark.sh 30
```

The script measures this pipeline's frame rate and latency with real camera input, and the results land in `output/m4/4.1`.

**This page does not give a target frame-rate number.** There are two layers to the reason: first, two design documents in the repository give contradictory readings for the same engine, and neither notes its measurement conditions; second, the frame rate itself strongly depends on the power mode, camera resolution, and scene. What you need to do is record the conditions completely, then measure it yourself.

### Acceptance Criteria

| Check                      | Pass criterion                                                                                                                             | When failing, inspect first                                                                                              |
| -------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------ |
| Environment self-check     | All three of the engine, `coco.names`, and `yolo_trt_node` exist; the TensorRT version is 10.3.x                                           | Whether `colcon build` has been run; whether the engine has been replaced with one built by a different TensorRT version |
| Message and QoS            | The type of `/perception/detections` is `vision_msgs/msg/Detection2DArray`; the QoS is `BEST_EFFORT` / `KEEP_LAST` (depth 10) / `VOLATILE` | Whether the subscriber also uses `SensorDataQoS` (see "downstream receives nothing" in troubleshooting)                  |
| Timestamp inheritance      | The detection message's `header.stamp` matches the source image                                                                            | Whether someone re-stamped the timestamp with `now()`                                                                    |
| Empty-frame behavior       | The topic is still publishing in a scene with no targets, with `detections` an empty array                                                 | Whether the node returns early on an empty box set                                                                       |
| Boundary of responsibility | `detections[i].id` is empty                                                                                                                | Whether tracking's ID logic was mistakenly written into the detection node                                               |
| Coordinate restoration     | Verify with a target at a known pixel position; the restoration error is within 1 px                                                       | Whether `dw` / `dh` used the scaled size; whether `s` was recomputed                                                     |
| Measurement record         | All seven conditions are stated, with the raw script output attached                                                                       | Whether only the frame rate was copied and the conditions dropped                                                        |

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

- **Solution**: confirm that the detection node **publishes every frame**, and that an empty frame publishes an empty array; downstream must also treat an empty array as a valid observation to advance the lost count. This design has a dedicated check in M4.1's acceptance criteria; when modifying the detection node, do not casually add "early return on an empty box set".

> **Next step:** hand this page's `/perception/detections` to 4.2 multi-object tracking. Tracking does not retrain the detector and does not change detection results; it is only responsible for assigning a stable ID to the boxes of the same target. Three things on the detection side directly affect tracking performance: the coordinate accuracy of the boxes, the `iou` threshold, and whether empty frames are still published. The boundary has already been drawn in the full picture: the detection node does not fill in `id`, which is tracking's job.