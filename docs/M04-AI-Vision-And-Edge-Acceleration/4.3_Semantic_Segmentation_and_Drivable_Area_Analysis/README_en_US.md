# 4.3 Semantic Segmentation and Drivable Area Analysis

**[Not yet implemented]** The model and engine artifacts needed to complete inference are currently missing: under `models/m4/segmentation/` there is only `LICENSE.md`, while `engines/` and `labels/` are both empty.

**[Not yet verified]** The existing code path has not yet completed the correctness gate, and there is no measured latency / FPS either.

This chapter provides **interfaces, a skeleton, and an acceptance method**; it does not promise that end-to-end inference can be completed. The steps below all appear in the form of "checking" rather than "after running, you will see".

## Course Overview

4.1 lets the robot know "what is in the frame", and 4.2 keeps the same target under the same ID across consecutive frames. But what navigation really has to answer is the third question: **where it can go**. Semantic segmentation does exactly this: attach a class label to every pixel in the image, then apply the table of "which classes count as drivable" to obtain a drivable mask.

The on-robot interface defines **two outputs**: one is the 19-class raw semantic map `/perception/semantic_mask`, and the other is the mapped drivable mask `/perception/drivable_mask`. Splitting them into two paths is intentional: if an upper layer wants to switch to a different definition of "what counts as drivable", what it changes is the mapping, without running the network through again.

The on-robot package in this chapter is `bev_segmentation`. It already has the node, preprocessing, postprocessing, the engine wrapper, the launch file, and the tests written, but **the two artifacts needed for inference have not been generated yet**. So the focus of this chapter is: see clearly what this interface looks like, which steps are still missing, and by what standard to accept it once they are filled in.

### Before You Start: What This Lesson Will Walk You Through

| Stage | What you will understand | What you can ultimately do |
| --- | --- | --- |
| Read | The difference between semantic segmentation and instance segmentation, and why navigation needs only the former | Judge which kind of segmentation a task should use |
| See through | How 19-class semantics turn into one 0/255 drivable mask | Read mapping configurations such as `drivable_class_ids` |
| Integrate | The separate roles of the two masks and how each is consumed | Consume `/perception/semantic_mask` and `/perception/drivable_mask` with the correct semantics |
| Accept | What this chapter is still missing before it can really run | Use the correctness gate to judge "when it can be treated as a runnable chapter" |

### Learning Outcomes

- Explain the criterion for semantic, instance, and panoptic segmentation, and state why navigation chooses semantic segmentation.

- Explain what mIoU, Pixel Accuracy, and FW-IoU each answer, and how small classes affect mIoU.

- Explain how SegFormer and DeepLabV3+ differ in encoder, multi-scale context, and decoder, and why the deployment side leans toward SegFormer-B0.

- Read every item of `config/segmentation.yaml`, and explain what the value of `drivable_class_ids` means.

- Restate the boundary that "candidate-drivable is not equal to collision-free", and explain why it must be kept.

- List the three steps `bev_segmentation` still needs to go from skeleton to runnable, and the acceptance criterion for each step.

### Hardware and Software Checklist

| ![Hardware and Software Checklist](./images/R29abyTLHooQaSx8cBYcPRoVncj.png) | ![Hardware and Software Checklist](./images/R45DbADhNoekD2xJmSBch3hLn5c.png) | ![Hardware and Software Checklist](./images/Rti6b89EuoWbfIxA4P5cb3usnLb.png) |
| --- | --- | --- |
| reCImputer mini J501  + GMSL expansion board |   | GMSL camera / USB camera |

### Prerequisites

- 4.1 / 4.2: the detection and tracking pipeline already runs end to end. This chapter shares the same camera image input with them.

- [2.1 GMSL2: automotive-grade multi-camera integration](https://seeedstudio.feishu.cn/docx/Takhd7wo5oPx3mx0ljhcfyxDnEe): confirm the image path and the driver.

- [2.2 Depth cameras and 3D visual perception](https://seeedstudio.feishu.cn/docx/Jj53dSSudoKyEdxk6jHcYnMHnOc): needed when using the Orbbec Gemini 2.

- You can use `ros2 topic` to look at topics and messages. This chapter does not require you to write nodes, nor to train models.

## Read First: Give Every Pixel a "Can It Be Driven On" Label

### Three Segmentation Tasks: Which One Separates Instances, Which Only Knows Classes

There is only one criterion that separates these three tasks: for two objects of the same class, whether the output can tell them apart. What cannot tell them apart is semantic segmentation, what can is instance segmentation, and panoptic segmentation puts "stuff that cannot be told apart" and "things that can be told apart" into the same output.

| Task | Output | Are same-class instances separated | Place on this page |
| --- | --- | --- | --- |
| Semantic segmentation | One class ID per pixel | Not separated; two pedestrians are both person | **Main line**: produces the two masks |
| Instance segmentation | One binary mask per instance, plus an instance ID | Separated; person_1 and person_2 each get their own mask | Not covered on this page |
| Panoptic segmentation | Unified output of the semantic map for stuff + the instance map for things | things separated, stuff not separated | Not covered on this page: two postprocessing stacks overlaid, the edge side must run two engines concurrently |

Navigation cares about "which region can be driven on". Ground, walls, and vegetation belong to stuff, which has no notion of instances, so forcing instance segmentation would only burn extra compute; conversely, a grasping task wants the mask of one specific object, which semantic segmentation cannot provide. Which route to pick depends only on where the output goes: if the output is to enter a costmap, use semantic segmentation; only if it is to enter the pose estimation of grasp planning do you need instance segmentation.

### What Each of the Three Metrics Answers

Choosing the wrong metric makes you misjudge a model problem as a dataset problem.

| Metric | The question it answers | What can mislead it |
| --- | --- | --- |
| mIoU | How much of each class is classified correctly | Small classes (pedestrians, poles) drag down the overall score, hiding that "the large classes are already good enough" |
| Pixel Accuracy | The proportion of all pixels classified correctly | When the ground covers 60%, predicting everything as ground still scores 0.6 |
| FW-IoU | IoU weighted by pixel proportion | Same origin as Pixel Accuracy; it looks good whenever the large classes are good |

The difference between the three is "whose voice is louder": mIoU weighs every class equally, so one small class classified badly clearly pulls the total score down; Pixel Accuracy and FW-IoU are both weighted by pixel count, so whoever has more pixels speaks louder.

The action after reading the numbers is fixed: **if the gap between mIoU and FW-IoU exceeds 0.15, the small classes have essentially not been learned**; at that point add samples or add class weights first, rather than trying quantization first. If FW-IoU is high while mIoU is low, go to the confusion matrix and see which large class the small classes were merged into. On the deployment side you must also look at two binary recall rates separately: **a low ground recall rate means drivable places are judged non-drivable, and a low obstacle recall rate means there is a collision risk**; they are closer to "will the robot crash" than mIoU is.

**One-line memory aid:** mIoU asks "is every class classified correctly", Pixel Accuracy asks "how much is right overall"; the former looks at fairness, the latter at volume.

### Encoder-Decoder: How SegFormer and DeepLabV3+ Differ

Both share the same overall framework, "downsample first to extract semantics, then upsample to restore resolution"; the differences lie in which operators the encoder uses, how multi-scale context is obtained, and how heavy the decoder is.

| Item | SegFormer (MiT-B0) | DeepLabV3+ (ResNet-101 / MobileNetV2) |
| --- | --- | --- |
| Encoder | Hierarchical Transformer (MiT), 4 stages outputting 1/4, 1/8, 1/16, and 1/32 features in turn; 4×4 overlapping patch embedding, no positional encoding | CNN backbone plus atrous convolution, outputting a stride-16 feature map |
| Multi-scale context | Self-attention itself covers the global range, so no extra module is needed | ASPP: atrous convolutions with dilation rates 6, 12, and 18 plus image-level pooling, 5 branches concatenated |
| Decoder | All-MLP: unify the 4 stage features to 256 channels, upsample and concatenate, output 1/4-resolution logits | Depthwise separable convolution, fusing stride-4 low-level features, output 1/4-resolution logits |
| Edge friendliness | The operators are mainly MatMul, LayerNorm, and GELU, with clear kernel choices under FP16 | Large-dilation atrous convolutions use more memory under FP16 as the dilation rate rises, and ASPP has many branches and many layers |

The selection criterion follows deployment: this page's main line uses **SegFormer-B0**, for a direct reason: its decoder output is already H/4 × W/4 logits, so postprocessing needs only one nearest-neighbor upsample; DeepLabV3+ also outputs 1/4 resolution, but ASPP's 5 branches make the engine's layer count and memory footprint larger. Consider switching over only when you already have a CNN training pipeline, or need the explicit multi-scale context that ASPP provides.

**One-line memory aid:** SegFormer replaces ASPP with attention; DeepLabV3+ replaces self-attention with atrous convolution.

### Class Mapping: How 19-Class Semantics Become One Drivable Mask

The segmentation network outputs dataset classes, while the robot wants "can it be driven on". This step is pure table lookup, but if the table is wrong, everything after it is wrong.

The on-robot model is a Cityscapes 19-class SegFormer-B0, and its output `/perception/semantic_mask` is a 19-class uint8 image (values 0 to 18). The drivable mask is derived from `drivable_class_ids` in `config/segmentation.yaml`:

| Source class | Which bucket it falls into | Reason and criterion |
| --- | --- | --- |
| road (trainId 0) | Drivable | **The on-robot default takes only this class** |
| sidewalk (trainId 1) | Optionally added | The config comment states explicitly: `drivable_class_ids` defaults to `[0]` (road only), and sidewalk is id=1 and optional |
| terrain (trainId 9) | Conditionally drivable | Grass and mud depend on the chassis: tracked vehicles can cross, wheeled ones need real-world testing, and treating it as an obstacle by default is more conservative |
| building, wall, fence, pole, traffic light, traffic sign, vegetation (trainId 2 to 8) | Obstacle (static) | Geometric obstacles |
| person, rider, car, truck, bus, train, motorcycle, bicycle (trainId 11 to 18) | Obstacle (dynamic) | Marks only the current position; trajectory prediction needs 4.2's tracking results and is not covered on this page |
| sky (trainId 10) | Ignored | The depth map is invalid in that region, or it is outside the height range of the robot body |

In code, the mapping is written as a constant array whose length equals the number of classes; do not write it as an if-else chain: if the class order changes, the if-else chain silently shifts out of place, whereas a constant array reports a length mismatch outright.

**Note that `drivable_class_ids: [0]` is a conservative default.** It means the sidewalk does not enter the drivable area. Before switching scenes, confirm this table first, rather than tuning inference parameters directly.

### The Two Masks: How semantic_mask and drivable_mask Divide the Work

The on-robot system publishes two masks rather than merging the mapping result into one:

| Topic | Type and value range | Its role |
| --- | --- | --- |
| `/perception/semantic_mask` | `sensor_msgs/Image`, 19-class uint8 (0–18) | The raw semantic result, keeping all class information |
| `/perception/drivable_mask` | `sensor_msgs/Image`, 0/255 uint8 | Derived from `drivable_class_ids`, for downstream consumption directly |

The benefit of splitting them into two paths is that **mapping and inference are decoupled**: when an upper layer wants to change "what counts as drivable", it changes `drivable_class_ids` in the config and does not need to run the network through again; and to debug whether the mapping is right, viewing the two masks side by side pinpoints whether the problem is in the model or in the table.

There is one boundary here that must be preserved verbatim; it is written in the comment of the on-robot config:

> **`drivable_mask` is a candidate-drivable semantic mask, not equivalent to collision-free space (a candidate-drivable semantic mask, not equivalent to collision-free space).**

Where does the difference lie: the mask only answers "does this pixel's class look drivable"; it does not know whether there is an unclassified overhead obstacle ahead, does not know the load-bearing capacity of the ground, and does not know where a dynamic target will be a second later. Treating it directly as "passable" amounts to treating a semantic map as a safety certificate.

**One-line memory aid:** `semantic_mask` answers "what is this", `drivable_mask` answers "by the current definition, does it count as drivable"; neither answers "is it safe to go through".

### Depth Back-Projection: Why This Chapter Does Not Build a Point Cloud

The source draft continued with "back-project the labeled pixels into a 3D obstacle point cloud". Mathematically this chain is very short:

$\begin{bmatrix} X \\ Y \\ Z \end{bmatrix}=d\,K^{-1}\begin{bmatrix} u \\ v \\ 1 \end{bmatrix}$

Expanded, this is $X=(u-c_x)d/f_x$, $Y=(v-c_y)d/f_y$, $Z=d$, where $(u,v)$ is the pixel coordinate, $d$ is that pixel's depth (meters), and $f_x$, $f_y$, $c_x$, $c_y$ come from the K matrix in `camera_info`.

**But the on-robot system has no point cloud topic.** `bev_segmentation` publishes only the two masks above and does not publish any `PointCloud2`. So this section keeps the formula only as extended knowledge, not as hands-on content of this chapter.

To really build a point cloud, four preconditions must hold at the same time: the depth map has undergone D2C alignment (depth pixels correspond one-to-one with color pixels); the depth map and the color image have the same resolution and a consistent `camera_info`; K comes from the same calibration line as the image (this page uses the pinhole `plumb_bob` K of the 2.3 main line; if what you have is the K of the fisheye `equidistant` model, substituting it into this formula gives a systematic offset); and the depth unit conversion is correct (16UC1 encoding generally stores millimeters, so divide by 1000 when reading it in). If any one of these four is missing, the point cloud will be misaligned with the image.

## Hands-On: Check the Skeleton, the Interfaces, and the Acceptance Gates

Three steps. All commands are executed on the J501, with the working directory `/home/seeed/workspace/ros2_bev`.

**To be clear up front**: these three steps are **checking and defining**, not "run it and see the result". This chapter's engine has not been generated yet, so if you follow the steps below, you will get a checklist of "what is still missing", not a segmentation image.

### Step 8: Check the Segmentation Skeleton and Model Asset Status

First see clearly how far the code has come and how far the model has come.

```bash
cd /home/seeed/workspace/ros2_bev

# 代码骨架：节点 / 前后处理 / engine 封装 / launch / 测试是否齐备
find ros2_ws/src/bev_segmentation -type f -name "*.cpp" -o -name "*.hpp" -o -name "*.py" | sort

# 模型资产：这里应当只有 LICENSE.md
find models/m4/segmentation -type f | sort
```

On the code side, **the skeleton files are all present**: `segmentation_node`, `segmentation_engine`, `preprocess`, `postprocess`, plus `config/segmentation.yaml`, `launch/m4_segmentation.launch.py`, and a set of tests. Complete files do not mean the implementation is verified; all judgments later in this chapter are based on actual run results. The model side is empty: under `models/m4/segmentation/` there is only one `LICENSE.md`.

The repository provides three scripts as implementation entry points, in order:

```bash
scripts/m4/export_segformer.sh        # 导出 ONNX
scripts/m4/build_segformer_engine.sh  # 建 TensorRT engine
scripts/m4/generate_labels_json.sh    # 生成 labels.json
```

> **[Not yet verified]** These three scripts themselves have not yet been validated by an end-to-end run. Before running them, read through the script contents once to confirm the path parameters; failing to run is currently expected, not an operating mistake on your part.

### Step 9: Check the Two Output Interfaces

Confirm the semantics by which downstream should consume them.

```bash
cat ros2_ws/src/bev_segmentation/config/segmentation.yaml
```

Check item by item:

- **Input**: `/perception/cameras/front/image` (color image; this chapter does not consume depth).

- **Two outputs**: `/perception/semantic_mask` (19-class uint8) and `/perception/drivable_mask` (0/255 uint8).

- **`drivable_class_ids: [0]`**: by default only road counts as drivable; sidewalk (id=1) must be added explicitly.

- **`publish_debug`**: this item exists in the config and defaults to `false`; its debug output capability is not yet among this chapter's verified items.

There are two usage boundaries here: `drivable_mask` is a candidate-drivable mask, not equivalent to collision-free space; and after the mapping is changed, the two masks must be re-checked together.

### Step 10: Define the Correctness Gate Once the Engine Is Ready

Step 10 defines the acceptance conditions once the engine is ready. This chapter does not currently count as complete, so the output of this step is not a reading but a gate:

```bash
scripts/m4/engine_correctness_gate.sh
scripts/m4/run_m4_3_benchmark.sh
```

Check item by item against the P0 checklist in `docs/M4.3_SEMANTIC_SEGMENTATION.md`; only after all three items are complete can this chapter be upgraded to a runnable chapter:

| Gate | Criterion | Current status |
| --- | --- | --- |
| Engine Correctness Gate | PyTorch and TensorRT outputs aligned, with the threshold filled in from actual runs | **[Not yet verified]** script written, not run |
| Latency / FPS report | Run inference N times on a real camera stream and report P50 / P95 | **[Not yet verified]** script written, not run |
| License verification | See `models/m4/segmentation/LICENSE.md` | **[Not yet verified]** not checked |

Before the gate passes, any number "optimized" out of this chapter does not hold, because there is no baseline to compare against.

## Deliverables and Acceptance Criteria

### Deliverables Checklist

**Currently deliverable (what this chapter can honor)**

1. A code and asset status record: the file list of `bev_segmentation` + the fact that `models/m4/segmentation/` is empty.

2. An interface contract record: the topics, types, and value ranges of the two masks, plus the current value of `drivable_class_ids`.

3. A release gate checklist: Engine Correctness Gate / Latency-FPS report / License verification, each with its status and trigger conditions.

4. (Optional) the log and the verbatim error output of an engine build attempt. This is the most valuable output this chapter currently has.

**Target runtime artifacts (`[Not yet implemented]` / `[Not yet verified]`, which this chapter does not promise to produce)**

- The actual images on the two channels `/perception/semantic_mask` and `/perception/drivable_mask`.

**The execution chain after unlocking** (each step below presumes the corresponding gate has passed; these are not current steps):

Assets successfully generated → correctness gate passed → `[Not yet verified]` `run_m4_3_demo.sh` verifies the two topics → `run_m4_3_benchmark.sh` produces the baseline.

### Acceptance Criteria

| Check | Pass criterion | Check first when failing |
| --- | --- | --- |
| Skeleton check | You can list `bev_segmentation`'s node, preprocessing and postprocessing, engine wrapper, launch, and test files | Whether you have mixed up `bev_segmentation` with another package |
| Asset status | You can state accurately what `models/m4/segmentation/` is currently missing | Whether you have taken `LICENSE.md` for a model file |
| Interface check | The topic names, types, and value ranges of the two masks match the config; you can explain what `drivable_class_ids` does | Whether you remembered only 0/255 and forgot the 19-class path |
| Safety boundary | You can restate candidate-drivable ≠ collision-free and give an example of why | — |
| Incomplete items | All three state their current status and "under what conditions they count as complete" | Whether you have taken "the script exists" for "the function works" |

## FAQ and Troubleshooting

### You Follow the Steps but Not One Frame Comes Out

- **Symptom**: the node does not start, or it starts but produces no output.

- **Cause**: **this is currently expected**. There is no engine and no labels under `models/m4/segmentation/`, so inference cannot begin.

- **Solution**: run the three generation scripts from Step 8 first, and write down the verbatim error output. Before the engine is generated, read this chapter as an "interface and skeleton description", not as a step-by-step tutorial.

### Ground-Class IoU Is Clearly Lower Than Other Classes

- **Symptom**: the overall image looks acceptable, but the ground (road) class is classified very poorly.

- **Cause**: the ground occupies a large share of the frame, has weak texture, and is strongly affected by lighting and reflections; it may also be that the diversity of ground samples in the training set is insufficient. Note that this kind of problem can only be discussed once the engine is ready and evaluation metrics can be produced.

- **Solution**: first look at the confusion matrix to confirm which class the ground was merged into (commonly sidewalk or terrain); then check the camera placement and lighting distribution of the training set. This chapter does not train models, so the direction of this troubleshooting is "swap the upstream weights or add data", not "tune inference parameters".

### The Two Masks Do Not Match

- **Symptom**: something in `semantic_mask` is clearly road, yet `drivable_mask` marks it as 0.

- **Cause**: the value of `drivable_class_ids` does not match expectations, or the two masks come from two different frames (timestamps not aligned).

- **Solution**: first `ros2 topic echo` the `header.stamp` of the two masks to confirm they are the same frame; then go back to `config/segmentation.yaml` and check `drivable_class_ids`. The value of this troubleshooting is that it separates a "model problem" from a "mapping problem".

> **Next step:** 4.4 pose estimation uses the same Orbbec Gemini 2 and **consumes both color and depth** channels at once, which is the first time in this module that `depth` is genuinely needed. The two masks left behind by 4.3 will appear side by side with detection and tracking in the 4.5 integration. Although this chapter cannot be run end to end, its interfaces are fixed: remember the semantics of `/perception/semantic_mask` and `/perception/drivable_mask`, because the following chapters all use them.
