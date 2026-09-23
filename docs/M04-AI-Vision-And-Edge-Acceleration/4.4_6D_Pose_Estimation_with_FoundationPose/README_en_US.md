# 4.4 6D Pose Estimation with FoundationPose

**Status: BLOCKED.** The `bev_pose` scaffolding and interface contract exist, but the FoundationPose runtime, weights, and real RGB-D input are not yet complete, so there is no real inference evidence. See [`code/PROJECT_STATUS.md`](../code/PROJECT_STATUS.md) for the canonical blockers.

What follows is an **implementation contract and acceptance entry point**, not a runnable experiment. "It will not run" is the expected state in this chapter, not an operating mistake on your part.

## Course Overview

4.1 through 4.3 all solve two-dimensional problems: what is in the image, which numbered target it is, and which class each pixel belongs to. Grasping has to answer a different question: **where is this object in three-dimensional space, and which way is it oriented?**

Detection gives you "where to look"; pose estimation gives you "how to place the gripper". Two steps are still missing in between: the detection box must first become a mask, because FoundationPose does not accept a rectangular box; and the quantity computed by pose estimation must be converted from the camera frame to the robot base frame, a step completed by the TF chain.

The theory part of this chapter is the most solid stretch in this module, because the algorithm used on real hardware is exactly the FoundationPose described here. The hands-on part must state honestly: this pipeline still lacks three things before it can run on real hardware.

### Before You Start: What This Lesson Will Walk You Through

| Stage | What you will understand | What you can ultimately do |
| --- | --- | --- |
| Read | What the 6 degrees of freedom of a 6D pose are, and why quaternions rather than matrices are used as the regression target | Read the field meanings of any pose output |
| See through | FoundationPose's three-stage mechanism: hypothesis generation, refinement, selection | Explain clearly why it does not need an external network to guess the initial pose |
| Distinguish | The division of labor with keypoint approaches such as DOPE | Choose a route by "whether you have CAD, whether you can accept second-level initialization" |
| Acceptance | What this chapter still lacks before it can run, and by what standard you judge that "it can run" | List the three prerequisites — dependencies, weights, camera — and define the upgrade conditions |

### Learning Outcomes

- Explain the 6 degrees of freedom of a 6D pose, and where homogeneous transforms, quaternions, and axis-angle each apply.

- Explain the roles of the camera frame, the object frame, and the robot base frame in the pipeline, and the relationship of chained multiplication along the TF chain.

- State what prior each of the four technical routes to 6D pose estimation depends on, and where its cost lies.

- Recount FoundationPose's three-stage mechanism, and explain the often-misread point that "the diffusion model does not take part in inference".

- State the typical manifestation of each of the four sources of pose error (symmetry, occlusion, truncation, depth noise).

- List the three prerequisites `bev_pose` still lacks to go from skeleton to runnable, and how each of them is verified.

### Hardware and Software Checklist

| ![Hardware and Software Checklist](./images/Yv51b1b5UonaTMxQqS0ca0ORnzh.png) | ![Hardware and Software Checklist](./images/Orz9bmlgMo0a9LxVD2TcpbqIntb.png) | ![Hardware and Software Checklist](./images/T6m7b6KHbokACSxomW6cAm3snWf.png) |
| --- | --- | --- |
| reCImputer mini J501  + GMSL expansion board |   | GMSL camera / USB camera |

| Category | Description |
| --- | --- |
| Compute platform | reComputer J501 (Jetson AGX Orin 32GB, MAXN mode); JetPack 6.2.1 (L4T 36.4.4), Ubuntu 22.04, CUDA 12.6, TensorRT 10.3.x, ROS 2 Humble |
| Camera | Orbbec Gemini 2 (USB3, active stereo infrared, integrated 6-axis IMU, supports hardware D2C alignment; the same model as in 2.2 / 4.3). **This chapter is the first in the module to genuinely consume depth**: the color, depth, and `camera_info` streams all three must be aligned |
| Algorithm stack | **NVlabs/FoundationPose** (PyTorch + nvdiffrast), wrapped independently by the `bev_pose` package |
| Model and weights | `model_dir` points to `/home/seeed/FoundationPose/weights`, **which does not currently exist** |
| Dependencies | `torch`, `trimesh`, `nvdiffrast`, **none of which is currently installed** |
| Mesh input | One `.obj` mesh, precomputed into `.npz` by `mesh_preprocessor.py` (parameter `mesh_npz_path`), or hand it the NVlabs source mesh directly (parameter `mesh_obj`) |

### Prerequisites

- 4.1 / 4.3: the detection pipeline and the mask interface are already understood. This chapter uses the mask evolved from the detection box as one of its inputs.

- [2.2 Depth Cameras and 3D Visual Perception](https://seeedstudio.feishu.cn/docx/Jj53dSSudoKyEdxk6jHcYnMHnOc): D2C alignment of the depth map and the color image.

- [2.1 GMSL2: Vehicle-Grade Multi-Camera Access](https://seeedstudio.feishu.cn/docx/Takhd7wo5oPx3mx0ljhcfyxDnEe): confirm the image path.

- Be able to inspect the TF tree with `ros2 run tf2_tools view_frames`.

- Linear algebra basics: being able to read matrix multiplication and to accept that a rotation matrix and a quaternion are two notations for the same rotation is enough.

## Read First: From Detection Box to Object Pose

### Why 6D Pose Is Needed: from "Where Is It" to "How to Grasp It"

Detection answers a two-dimensional localization problem: which patch of pixels the object occupies in the image. That answer is not enough for grasping, because a position in the image carries neither depth nor orientation. 6D pose estimation (6D Pose Estimation) answers a three-dimensional rigid-body localization problem: the object's 3D position in some coordinate frame (3D translation $t$) and its 3D orientation (3D rotation $R$). Three degrees of freedom for position and three for orientation make six in total — that is where "6D" comes from.

Putting these two questions together gives this chapter's main line: detection says "where to look", pose estimation says "how to place the gripper".

### Pose Representations: R, t, Homogeneous Transforms, Quaternions, Axis-Angle

A rotation has only 3 degrees of freedom, yet can be written in more than 4 ways; which one you choose depends on what you are going to do with it: take part directly in coordinate transforms, or feed it to a neural network for regression. First look at how position and orientation combine into one quantity.

Translation and rotation can be combined into a 4×4 homogeneous transform matrix, whose job is to take a point $p_{obj}$ in the object frame to the camera frame:

$T_{cam}^{obj} = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}, \qquad p_{cam} = R\,p_{obj} + t$

$R$ is a 3×3 rotation matrix, $t$ is a 3×1 translation vector, and the fourth row is fixed at $[0\ 0\ 0\ 1]$. The cost of writing it this way is that 16 numbers are packed into 4×4 while there are only 6 degrees of freedom. The benefit is that one matrix multiplication completes rotation and translation at the same time, and multiple transforms can be multiplied together directly.

The four rotation representations and where each one applies are as follows:

| Representation | Parameters and constraints | Its cost | Where it fits |
| --- | --- | --- | --- |
| Rotation matrix $R$ | 9 numbers satisfying $R^{T}R = I$ and $\det(R) = 1$, actually 3 degrees of freedom | Redundant parameters; a directly regressed matrix generally does not satisfy orthogonality and needs a further orthogonalization | Coordinate transforms, TF broadcasting, error computation; not sent directly as a regression target |
| Quaternion $q = [w, x, y, z]$ | 4 numbers with one unit-length constraint, 3 degrees of freedom | Sign ambiguity: $q$ and $-q$ denote the same rotation, so sign alignment is needed during training | Network regression targets; the default storage form for ROS messages, TF, and `geometry_msgs/Quaternion` |
| Axis-angle $r = \theta n$ | A unit axis $n$ plus an angle $\theta$, written together as a 3-dimensional vector, 3 degrees of freedom | When $\theta$ approaches $\pi$ the direction flips and regression is discontinuous | Expressing errors, and as an intermediate for interpolation; not used alone as a wide-range regression target |
| 6D continuous representation | Regress the **first two columns** of the rotation matrix, 6 numbers in total, then restore $R$ with Gram-Schmidt orthogonalization | Requires one orthogonalization post-processing step | Background knowledge: the mainstream choice for deep-learning rotation regression; not covered in this chapter |

Although a quaternion is compact, comparing two poses requires care: $q$ and $-q$ are the same rotation, and subtracting them directly would count them as a 180° error. For the angular error, take $\theta = 2\arccos\big(|\langle q_1, q_2\rangle|\big)$ — taking the absolute value of the inner product first makes the sign ambiguity disappear; in an implementation, clamp the inner product once to keep floating-point error from going out of range.

$R$ and a quaternion are two notations for the same rotation, not two different quantities: the former is used for coordinate transforms, the latter for network regression and ROS messages.

### Frame Conventions: Camera Frame, Object Frame, Base Frame

The same sentence "the object is at (0.4, 0.1, 0.6)" means something entirely different in another frame. Three coordinate frames appear consistently in this chapter's pipeline; distinguish their roles first, and only then will you have an identifiable reference when debugging pose drift later.

| Frame | Definition of origin and axes | Role in this chapter's pipeline |
| --- | --- | --- |
| Camera frame | Optical center as origin, $x$ to the right, $y$ downward, $z$ forward along the optical axis | The frame of the RGB-D data and of the intrinsics $K$; both pose estimation and tracking do projection and rendering here |
| Object frame | Defined by the mesh file you provide; the origin and axes are the mesh's own coordinate frame | The "described object" of the pose estimation result; $T_{cam}^{obj}$ is exactly the transform that takes the object frame to the camera frame |
| Robot base frame | The vehicle body or the robot arm base; the course follows `base_link` | The frame used when planning a grasp; the TF chain converts the pose in the camera frame over to it |

The three are connected by chained multiplication along the TF chain: $T_{base}^{obj} = T_{base}^{cam} \cdot T_{cam}^{obj}$. Once one link of this chain is missing or its timestamp has expired, the symptom in RViz2 is the object TF flying off screen or no longer refreshing, not a slightly off number. So when troubleshooting pose problems, the first step is always `ros2 run tf2_tools view_frames`, to see whether a link in the chain is broken.

$T_{base}^{cam}$ is not a quantity that holds forever after one calibration. When the camera is mounted on the arm's end effector it comes from hand-eye calibration (Eye-in-hand); when the camera is fixed externally it comes from extrinsics. Before wiring the pose straight into MoveIt2, first confirm that this link is a calibrated real number rather than a temporary approximation.

**One-line memory aid:** pose estimation outputs the "object relative to camera" transform; grasping needs the "object relative to base" transform. The difference between the two is exactly one $T_{base}^{cam}$, provided by calibration and TF, not predicted by the model.

### Method Categories: Four Routes and Their Costs

Methods for 6D pose estimation can be divided into four categories by "what prior they depend on and at which step they solve". The choice does not depend on which one is newer, but on whether you have a mesh or reference images at hand, whether the object is symmetric, how heavy the occlusion is, and whether you can accept second-level latency.

| Route | Representative methods | Prior it depends on | Main cost and failure scenarios |
| --- | --- | --- | --- |
| Template matching | LINEMOD and its improvements | CAD model: offline rendering of multi-view templates plus normal and depth gradient features | The template library is large and initialization is slow; sensitive to heavy occlusion and cluttered backgrounds; a textureless object has almost no features to match |
| Direct regression | PoseCNN, CenterSnap | Training data covering the target category; CenterSnap additionally needs depth | Poor generalization to objects outside the training set; the rotation ambiguity of symmetric objects makes the regression target self-contradictory; accuracy is on the centimeter scale |
| Keypoints + PnP | DOPE, PVNet | A CAD model used to generate keypoint annotations and synthetic training data | Fails as soon as keypoints are occluded; PnP is sensitive to outliers among the keypoints; the category is fixed, so a new object requires retraining |
| Pose refinement + pose selection | FoundationPose | The paper supports two settings: giving a textured CAD model (model-based), or giving about 16 reference images (model-free) | Needs an initial pose; one refinement step can travel only a limited distance, and an initial value too far off converges to a wrong local minimum; single-object estimation takes seconds rather than being real-time |

What the first three routes have in common is "a one-shot solution": the network or the matcher gives the answer directly, with no opportunity for iterative correction. The fourth route first needs a rough initial pose, then grinds it accurate by iteration; its accuracy ceiling is set by the refinement process, not by a single regression.

The bottleneck in choosing a route therefore splits into two places: the first three categories are stuck on priors (you need CAD, training data, keypoints), while the refinement route is stuck on the initial value, and on "picking the right one out of a batch of initial values".

### FoundationPose's Real Mechanism: Hypothesis Generation, Refinement, Selection

FoundationPose divides 6D pose into two stages, "estimation" and "tracking", with three modules relaying internally, plus one training-time component that is often misread.

**Step one: pose hypothesis generation.** Sample $N_s = 42$ viewpoints uniformly around the object, and give each viewpoint $N_i = 12$ in-plane rotations, yielding 504 initial pose hypotheses. This batch of hypotheses covers "in what poses the object might be seen", so no external network is needed to guess the first pose.

**Step two: the Pose Refinement Network.** Its input is "the object image rendered under the current pose + the observation crop", and its output is a pose update: a translation increment $\Delta t \in \mathbb{R}^3$ and a rotation increment $\Delta R \in \mathrm{SO}(3)$, pushing the current estimate one step in the direction of observation consistency. The paper's test setting iterates 5 times in the estimation stage and only 1 time in the tracking stage. The corresponding parameter on real hardware is `refiner_iterations` (default 5), matching the paper's estimation stage. Its relationship to classic Iterative Closest Point (ICP) is replacement rather than addition: ICP needs explicit geometric correspondences, and on a textureless plane or a smooth cylinder there are almost no usable correspondences to find; the learned refinement network can use joint color and shape cues to judge "which way to push".

**Step three: the Pose Selection (Ranking) Network.** It performs a two-level hierarchical comparison of $K = 5$ candidate hypotheses, scores them separately, and takes the highest-scoring one as the output. What it answers is not "how to fine-tune" but "which of this batch of hypotheses is most trustworthy".

**A point often misread: the diffusion model does not take part in inference.** The diffusion model in the paper is used only to synthesize training data (together with LLM-guided texture augmentation); its role is to make the training set richer. At inference time, pose screening is done by the ranking network of step three above. So "the diffusion model provides a pose prior" is an architectural misunderstanding — do not draw it that way in explanations or diagrams.

**How the paper's account differs from the real-hardware implementation.** The paper's model-free setting needs about 16 reference images (ablation experiments show that 12 already approaches saturation). On real hardware, `bev_pose` takes a different route: prepare one mesh, precompute it into `.npz` with `mesh_preprocessor.py`, and pass it to the node via `mesh_npz_path`; you can also hand it the NVlabs source mesh (`mesh_obj`) directly. So "no retraining required" means **no need to retrain the network for each new object**, not "no model files of any kind required".

**The speed magnitudes must be discussed separately.** The numbers the paper reports on an RTX 3090 are: single-object estimation about 1.3 s, tracking about 32 Hz. The former is the one-time cost of "starting from an unknown pose", the latter the cost of "maintaining frame by frame once initialized" — two orders of magnitude apart. Judging these two things by the same metric will lead to opposite conclusions.

**Three hard preconditions on the input.** If any one is missing, you will see "it looks like it is tracking, but it is actually drifting". First, the color image is undistorted first according to the mainline `plumb_bob` $(K, k_1, k_2, p_1, p_2, k_3)$, and the depth image is aligned by D2C into the color frame, so that the two land on the same set of parameters; mixing in fisheye `equidistant` (Kannala–Brandt) parameters makes the error grow systematically with field angle. Second, the intrinsics $K$ must match the real hardware, otherwise back-projection is meaningless. Third, the object must not be fully occluded during tracking.

**One-line memory aid:** the refinement network answers "which way the current pose should be fine-tuned"; the selection/ranking network answers "which of this batch of hypotheses is most trustworthy". The former determines accuracy, the latter keeps it from flying off, and the diffusion model has nothing to do with either.

### Comparison with DOPE: When to Switch Approaches

For the same object, whether to use DOPE or FoundationPose depends on whether you are shorter on CAD, shorter on speed, or shorter on tolerance for occlusion. The two differ in inputs, priors, and output form; you cannot compare just one "accuracy" number.

| Dimension | DOPE | FoundationPose |
| --- | --- | --- |
| Input | Monocular RGB image | Undistorted color image + aligned depth + segmentation mask |
| Prior dependency | The target object's CAD model, used to synthesize training data | Paper: textured CAD or about 16 reference images; real hardware: one mesh (`.obj` → `.npz`) |
| Intermediate product | Confidence heatmaps for the object's 8 corners plus 1 centroid, then PnP to solve the pose | 504 initial hypotheses, refined and ranked to give the pose directly |
| Output form | Keypoint heatmaps → pose solved by PnP | A pose matrix, publishable as `PoseStamped` (primary) and `Detection3DArray` (optional) |
| Cost of a new object | Needs that object's CAD and one round of synthetic-data training | Prepare one mesh and you can switch objects, with no need to retrain the network |
| Speed and occlusion | Monocular single forward pass, low latency; once keypoints are occluded the confidence drops sharply and the pose jumps | Estimation takes seconds and tracking runs frame by frame; it keeps refining from the remaining visible surface, so occlusion tolerance is higher |

The choice criterion can be compressed into two sentences. If the object category stays fixed over the long term, you want low latency, and you have CAD on hand, use a keypoint approach such as DOPE; if objects change often, you cannot get CAD but can supply a mesh, occlusion is unavoidable, and you can accept second-level initialization, use FoundationPose. This course chooses the latter as the main line, because the targets in tabletop grasping tasks are almost always specified on the spot.

### Difficulties: Symmetry, Occlusion, Truncation, and Depth Noise

Pose error does not come from a single cause, and the four difficulties each have their own form of manifestation. The purpose of writing them separately is to let you localize which one is at fault when accuracy falls short, rather than tuning parameters blindly.

- **Rotational ambiguity of symmetric objects**: objects such as spheres, cylinders, and boxes have a symmetry transform $S$, and $R_{gt}S$ and $R_{gt}$ cannot be distinguished physically. The estimate may land on the former; comparing directly with ADD will judge it a large error, yet that pose is in fact correct for grasping. Metrically you should use the ADD-S criterion; on real hardware `bev_pose` does not expose a symmetry-prior parameter, so when you encounter a symmetric object you either handle it in the evaluation criterion or restrict the target upstream to an asymmetric one.

- **Occlusion**: once the visible surface shrinks, the geometric information that can constrain the pose shrinks with it, and refinement slides along the unconstrained direction. The visible-pixel ratio is the main basis for judging this one: the lower the ratio, the more pronounced the rise in error. At that point you should consider reselecting the initial hypothesis or raising `refiner_iterations`, and record visibility together with error, rather than shortening the iterations to save compute.

- **Truncation**: the object goes beyond the image boundary, the node can see only part of its surface, which does not match the mesh, and the initial pose is liable to be considerably off. The typical manifestation is "the pose suddenly jumps when the box is at the edge of the image"; the fix is to move the object back to the middle of the image and initialize again.

- **Texturelessness and depth noise**: a textureless surface invalidates the color cues, leaving only geometric cues; depth noise directly pollutes the geometric cues. When the two stack, fix depth first: confirm that the alignment error between depth and color is small enough and that depth has enough valid points within its valid range, then consider changing the mesh or the texture.

The most dangerous failure mode of tracking is not a dropped frame, but "it looks like it is tracking while the pose has already drifted". Real hardware provides no automatic reset option, so you must judge drift yourself: check whether the observations near the pose still support the current estimate (for example, compare the object contour under the predicted pose with the measured depth); as soon as the estimate and the observation start to come apart while the position readings remain smooth, discard that track and go through initialization again. The concrete thresholds for the criteria need to be set after the real hardware runs.

### Accuracy Metrics: ADD, ADD-S, and Rotation/Translation Error

Pose accuracy cannot be described with just one sentence about "how much error", because the metric itself decides what counts as correct and what counts as wrong. Fix the criterion before evaluating, and the evaluation results will be comparable.

- **ADD**: transform the object's model points once by the predicted pose and once by the ground-truth pose, then compute the average distance between the two point sets. It answers "if this pose is measured by geometric error, how far off is it".

- **ADD-S**: for each model point, take the distance to the **nearest point** in the ground-truth point set, then average. It specifically handles symmetric objects: equivalent poses would be judged a large error under ADD but are not misjudged under ADD-S.

- **Rotation / translation error**: split the error apart and look at it; rotation is expressed as an angle (note the "shortest arc" criterion mentioned earlier), and translation directly as Euclidean distance. The benefit of splitting is that you can distinguish "rotated correctly but offset in position" from "positioned correctly but rotated askew" — the troubleshooting directions for these two faults are entirely different.

Once the wrong criterion is used on a symmetric object, a correct pose will be judged a large error; this is also why ADD-S is not a "looser version" of ADD, but a metric aimed at another class of problem.

**But this chapter does not produce these numbers.** The real hardware has no pose evaluation script, nor a test sequence with ground-truth poses. The criteria are kept here so that you know which one to choose when you add evaluation in the future. They are a "future correctness metric", not current measured results.

### The Pose Pipeline on Real Hardware: Nodes, Topics, and the Pose/TF Contract

With the theory covered, look at what this turns into on a robot. The `bev_pose` package provides two nodes:

| Node | Role |
| --- | --- |
| `object_mask_node` | P0 single-object initialization aid: generates an object mask from depth and an optional ROI hint |
| `foundationpose_node` | Main node: consumes color, depth, intrinsics, and mask, and outputs the object pose |

**Input**

| Topic | Type | Description |
| --- | --- | --- |
| `/perception/cameras/front/image` | `sensor_msgs/Image` | `rgb8` or `bgr8` |
| `/perception/cameras/front/depth` | `sensor_msgs/Image` | **`32FC1`, in meters** |
| `/perception/cameras/front/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics K |
| `/perception/object_mask` | `sensor_msgs/Image` | `mono8`, 0/255 |

**Output**

| Topic | Type | Description |
| --- | --- | --- |
| `/perception/object_pose` | `geometry_msgs/PoseStamped` | **Primary output** |
| `/perception/object_poses_3d` | `vision_msgs/Detection3DArray` | Optional (`publish_det3d`) |
| `/tf` | — | Parent frame `camera_front`, child frame determined by a parameter |
| `/perception/mesh_meta` | `std_msgs/String` | latched JSON, for visualization |
| `/perception/foundationpose/stats` | `std_msgs/String` | JSON, 1 Hz |

> **There is one point here you must get exactly right.** The primary output is **`PoseStamped`**, not `Detection3DArray`; and the TF's **parent frame is `camera_front`**. Writing the subscriber side against `Detection3DArray` or against some other parent frame name will leave you receiving no data or finding no transform.

Key parameters (`config/pose_estimation.yaml`): `model_dir` (weights directory), `refiner_iterations` (5), `score_threshold` (0.3), `camera_frame_id` (`camera_front`), `auto_register_on_first_mask` (true), `require_mask_for_register` (true), `mesh_obj` / `mesh_npz_path` (mesh input). `object_mask_node` is a **P0 single-object** aid; this chapter does not cover multi-object scenes.

## Hands-On: Readiness Audit and Acceptance Gates

Three steps. These three steps are **checking and defining**, not "run it and see the result". This chapter currently does not run; follow the steps below and you will get a checklist of "what is still missing".

### Step 11: Readiness Audit

First confirm how each of the three prerequisites falls short, so that you do not later misjudge an environment problem as an operating mistake.

```bash
# 1. 权重目录
ls -d /home/seeed/FoundationPose/weights

# 2. 依赖
python3 -c "import torch; print('torch', torch.__version__)"
python3 -c "import trimesh; print('trimesh ok')"
python3 -c "import nvdiffrast; print('nvdiffrast ok')"

# 3. 相机
ros2 topic list | grep /perception/cameras/front
```

The current state of the three items: the weights directory **does not exist**; `torch` / `trimesh` / `nvdiffrast` **all fail to import**; the camera **is not connected**. Until all three are in place, neither `run_m4_4_demo.sh` nor `run_m4_4_perf_benchmark.sh` will produce meaningful results.

> **[To be implemented]** Write down the output of this step as this chapter's "prerequisite checklist". It is closer to what this chapter can currently deliver than any operating step.

### Step 12: Check the Pose / TF Output Contract

Write the downstream against the **real** contract, not against intuition.

```bash
cat modules/m04-ai-vision-and-edge-acceleration/ros2/bev_pose/config/pose_estimation.yaml
grep -n "camera_frame_id\|pose_topic\|publish_det3d" modules/m04-ai-vision-and-edge-acceleration/ros2/bev_pose/bev_pose/foundationpose_node.py
```

Confirm item by item:

- The primary output is `/perception/object_pose`, of type `geometry_msgs/PoseStamped`;

- `/perception/object_poses_3d` is an **optional** `Detection3DArray` (controlled by `publish_det3d`);

- The TF's **parent frame is `camera_front`**, and the child frame is determined by `<frame_id>`;

- The depth input must be `32FC1` with units of meters; when that contract is not met, the pose result cannot enter valid acceptance.

The most memorable thing in this section is not the parameter values but **the shape of the contract itself**: a pose is one `PoseStamped` plus one TF, not a detection array.

### Step 13: Define the Acceptance Gates for Upgrading to a Runnable Chapter

Finally, set for this chapter "when it really counts as runnable".

```bash
# 只读脚本，核对它定义的验收条件（不执行）
sed -n '1,40p' scripts/m4/run_m4_4_perf_benchmark.sh
```

Check item by item against the acceptance conditions defined at the head of the script:

| Gate | Type | Criterion |
| --- | --- | --- |
| tracking FPS > 10 | Hard | Frame-by-frame maintenance capability once initialized |
| Pose stays stable and tracking is not lost within a 30 s window | Hard | The boundary between drift and lost tracking |
| RGB-D pipeline timestamp deviation < 33 ms | Hard | Alignment quality between color and depth |
| register latency | Soft | First registration on the Jetson is inherently slow, so no hard upper limit is set |

**[To be verified]** All four items become meaningful only after the three prerequisites of step 11 are in place. This step does only two things: **define** the gates and **record** the gates; at present it can be neither executed nor reported as passing. Until the gates pass, no number derived from this chapter holds, because there is no baseline to compare against.

## Deliverables and Acceptance Criteria

### Deliverables Checklist

**Currently deliverable (what this chapter can honor)**

1. A prerequisite checklist: the weights directory, the three dependencies, and the camera path, with each one's status, verification method, and outstanding items.

2. A contract record: four input topics and five output topics (including the PoseStamped primary output and the TF parent frame `camera_front`).

3. An acceptance-gate checklist: the type and trigger condition of each of the four criteria (**definition and record**, not executed in this chapter).

**Target runtime artifacts (`[To be implemented]` / `[To be verified]`, not promised by this chapter)**

- The real-time pose output on `/perception/object_pose`, and the TF following the object in RViz2.

**The execution chain once unlocked** (each step presupposes that the prerequisites are in place; these are not current steps):

Dependencies and weights in place → camera connected → `[To be verified]` `run_m4_4_demo.sh` runs through the node chain → `run_m4_4_perf_benchmark.sh` passes the four criteria.

### Acceptance Criteria

| Check | Pass criterion | Check first when failing |
| --- | --- | --- |
| Prerequisite checklist | You can state the status of all three prerequisites accurately and know how to complete each | Whether a dependency was missed (`torch` / `trimesh` / `nvdiffrast` — all three are needed) |
| Contract check | You can state the primary output type and the TF parent frame accurately | Whether you recorded it as `Detection3DArray` |
| Coordinate frames | You can draw clearly the chained multiplication among the camera frame, the object frame, and the base frame | Whether you treated `T_base^cam` as something the model predicts |
| Mechanism recount | You can explain the three-stage mechanism clearly and point out that the diffusion model does not take part in inference | Whether you treated the diffusion model as a pose prior |
| Acceptance gates | The type and criterion of all four items are written out | Whether you treated a soft metric as a hard gate |

## FAQ and Troubleshooting

### What Should the Mesh Parameters Be Set To

- **Expected error**: when the mesh field is empty the node cannot initialize (the troubleshooting entry is based on the parameter contract, not on a real-hardware run record).

- **Two entry points**: `mesh_obj` (the NVlabs source mesh `.obj`) and `mesh_npz_path` (the `.npz` precomputed by `mesh_preprocessor.py`).

- **Solution**: first preprocess the mesh into `.npz` with `preprocess_mesh.sh`, then fill the path into `config/pose_estimation.yaml`. The mesh has only these two entry points; do not go looking for parameter names from other implementations online.

### The Pose Topic Has Data, but RViz2 Does Not Show the Object Frame

- **Expected symptom**: the pose topic is publishing, but the object TF does not move or refresh (inferred from the TF chain and naming conventions; not reproducible until the real hardware runs).

- **Check first**: check the TF chain first, not the model. If a link is missing between the parent frame `camera_front` and the target child frame, or some segment's timestamp has expired, RViz2 will stop refreshing.

- **Solution**: use `ros2 run tf2_tools view_frames` to look at the whole tree and confirm that the `camera_front` link exists and is continuous. Then confirm whether `/perception/object_pose` is publishing. A stopped pose and a broken TF chain are two different faults; first work out which one it is.

### The Pose Is Offset Overall, and the Error Grows with Distance

- **Expected symptom**: all poses deviate in the same direction, and the farther the distance the larger the deviation (structural reasoning, not a real-hardware observation).

- **Check first**: an error of the "grows linearly with distance" kind points to the coordinate frames or the calibration, not the model: the intrinsics K do not match reality, `T_base^cam` uses an uncalibrated approximation, or the depth unit is not meters.

- **Solution**: first confirm the depth image's encoding and units (this chapter requires `32FC1` with units of meters); then check whether `camera_info` is the same calibration as the actual camera; finally check where `T_base^cam` comes from. This kind of error will not disappear by itself; you must localize the specific link.

> **Next step:** 4.5 puts the models from the previous four chapters into the same pipeline and discusses optimization techniques such as zero-copy, fixed shapes, quantization, and offloading. **Note**: 4.5 likewise has no corresponding implementation on the real hardware; the whole chapter is forward-looking content. As with this chapter, when reading it, first separate "what exists now" from "what is to be done later".
