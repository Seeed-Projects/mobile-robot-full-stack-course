# 7.2 Understanding LightNav-0

[English](README_en_US.md) | [中文](README_zh_CN.md)

Chapter 7.1 introduced VLN through the instruction “Locate the refrigerator in the room and stop beside it.” This chapter explains how LightNav-0 turns camera images and language into a local path a robot can follow.

Keep one idea in mind: **the model reads the instruction and visual history, generates two image-pointing symbols and three action codes, and a decoder reconstructs ten waypoints from those codes.**

![LightNav-0 pipeline: instruction and visual history to ten local waypoints](assets/images/13-lightnav-pipeline-v1.png)

*LightNav-0's core pipeline. APOS and OPOS express action and target pointing; RVQ decoding converts three action codes into ten local cumulative poses.*

## Learning Objectives

- Explain how LightNav uses a vision-language model to produce navigation outputs.
- Describe the roles of SlowFast, APOS/OPOS, and RVQ.
- Distinguish a predicted path, controller execution, and the final task outcome.

## 7.2.1 How Can a Language Model Generate a Navigation Path?

A language model generates **tokens**, symbols in its vocabulary, one at a time. A token may be a text fragment or a specially defined marker. A **Vision-Language Model (VLM)** also converts images into numerical features and processes them with text. It can therefore address questions such as “What is in the image?” or “Which side is the refrigerator on?”

Saying “go toward the refrigerator” is still insufficient for control. The execution layer needs a movement direction, distance, and turning behavior. LightNav adds **navigation-specific symbols with defined spatial meanings** to the model's vocabulary.

Its output contains five task tokens:

| Output | Question it addresses |
| --- | --- |
| APOS: action pointing | Where in the image should the next action be directed? |
| OPOS: object/target pointing | Where is the instructed target in the image? |
| Three action codes | What local path is proposed for this decision? |

The two pointing symbols express image-space intent, while the three action codes jointly represent one geometric path. The model generates symbols in sequence; the decoder converts them into image positions and motion data.

Suppose the refrigerator is in the next room. OPOS can point to the appliance, APOS to the doorway, and the action codes to a path approaching that opening. This makes the spatial intent inspectable and provides more specific movement information than “forward” or “left.”

## 7.2.2 The Overall Architecture

LightNav-0 is based on Qwen3-VL-4B-Instruct. It retains the shared model backbone and autoregressive token generation, learning navigation symbols through training rather than adding a separate continuous-trajectory prediction head.

One request can be understood in five steps:

1. **Select observations.** Choose the current and some earlier camera images from the task's ordered history.
2. **Extract visual features.** Encode those images numerically, then compress spatial detail according to their history tiers.
3. **Interpret the inputs together.** Give the language model the image features, time information, and instruction so it can relate the target, scene, and earlier views.
4. **Generate navigation symbols.** Produce APOS, OPOS, and three action codes in order. Earlier generated symbols also condition later predictions.
5. **Decode the path.** Look up the motion features and compose ten cumulative poses relative to the robot frame at capture time.

![One LightNav decision](assets/lightnav/01-decision-loop.gif)

*Figure 7.2-1. The inputs, five navigation tokens, and ten-waypoint output of a LightNav decision.*

Visual history is the sequence of images captured during this navigation task. It lets the model refer to previously seen furniture, doorways, and targets as well as the current view.

**Why can generating five symbols still take seconds?** The model must process images and the full input context before generating them. Caching can reuse eligible computations. SlowFast reduces how much visual information each request needs to process. Measure actual latency across the complete request.

## 7.2.3 SlowFast: More Detail for Recent Views

Near the refrigerator, the robot needs detail about the doorway, floor, and obstacles. Earlier views may still help it infer the target's direction. Feeding every image at full detail would make computation grow with task duration.

**SlowFast processes visual history at different temporal and spatial resolutions.** Recent views are sampled more densely and retain more detail; older views are sampled more sparsely and represented more coarsely. The name does not describe the robot's driving speed.

### Temporal Compression: Select Fewer Images for This Prediction

At 4 Hz, the camera contributes one image every 0.25 seconds. After roughly a minute, the buffer contains 240 frames. SlowFast selects frames by age instead of sending all 240 to the current prediction.

![SlowFast temporal selection](assets/slowfast/time-selection.gif)

*Figure 7.2-2. SlowFast selects current, recent, medium-term, long-term, and initial observations according to their age.*

The principle is to **retain consecutive current and recent views, sample groups at intervals in the middle, spread selections across older history, and retain initial observations as anchors.**

In this example, the other 158 frames are not selected for this prediction; they can remain in the original history. Selection is repeated relative to the next current frame. It follows time rules, rather than first recognizing which image contains a refrigerator, so it can still miss useful information.

### Spatial Compression: Keep Fewer Features from Older Views

After selection, the visual encoder produces features arranged by spatial location. Imagine a grid over the image, with each cell containing numbers describing local appearance and context. **Spatial pooling** summarizes neighboring cells into fewer cells.

![SlowFast spatial pooling](assets/slowfast/spatial-pooling.gif)

*Figure 7.2-3. Spatial pooling summarizes an 8×8 feature grid into 4×4 or 2×2 grids.*

For the same 8×8 example grid:

| Pooling factor | Operation | Retained spatial locations |
| --- | --- | ---: |
| 1 | Keep cells unchanged | 8×8 = 64 |
| 2 | Summarize each 2×2 region as one cell | 4×4 = 16 |
| 4 | Summarize each 4×4 region as one cell | 2×2 = 4 |

Factor 2 reduces both dimensions, leaving one quarter as many spatial locations.

### Combining the Two

| Observation range | Temporal treatment | Spatial treatment |
| --- | --- | --- |
| Current | Keep the latest views | Retain fine features |
| Recent | Sample densely | Apply moderate pooling |
| Medium- and long-term | Sample at intervals or across the range | Use coarser representations for older views |
| Initial anchors | Retain the starting observations | Summarize them with fewer features |

Temporal compression reduces **how many images enter the request**. Spatial compression reduces **how many feature locations each selected image retains**. The order is **frame selection → visual encoding → spatial pooling of encoded features**. The source calls the last operation `post-ViT pooling`: pooling after the Vision Transformer (ViT) encoder.

See [Advanced Reading](ADVANCED_READING.md#slowfast-configuration-and-time-representation) for the exact tiers and selection rules.

## 7.2.4 APOS and OPOS: The Goal and the Next Action

Seeing a refrigerator does not mean the robot can drive straight toward it. A doorway or table may require a different immediate approach. LightNav expresses this distinction with two image-pointing channels:

- **OPOS points to the target:** where the refrigerator appears in the image.
- **APOS points toward the action:** for example, traversable floor at the doorway.

![APOS and OPOS pointing grid](assets/lightnav/02-pointing-grid.gif)

*Figure 7.2-4. Orange APOS indicates action direction; green OPOS indicates target location.*

To point with one symbol, the implementation divides the image into **48 columns and 27 rows**, assigns a code to each cell, and maps a predicted code back to the cell center.

In a 480×270 image, a cell is 10×10 pixels. With zero-based row and column indices, row 10, column 24 has code `1 + 10×48 + 24 = 505`, corresponding to center pixel `(245,105)`. Code 0 is reserved for a special case.

Pixels do not supply distance, so an OPOS pixel coordinate cannot simply be scaled into meters.

When the target is outside the view, OPOS can indicate that no ordinary target point was provided. APOS also supports special values for left rotation, right rotation, and stopping. Determine whether an output is a grid point or a special state before interpreting it.

For our example, **OPOS on the refrigerator and APOS on the doorway floor can both be appropriate**. An action point on the floor does not certify that the entire chassis fits through. Check the path for collisions, and inspect pointing on its corresponding captured image: an old pixel location cannot be pasted onto a new view after turning.

## 7.2.5 RVQ: How Can Three Codes Describe One Path?

A ten-waypoint path contains ten sets of position and heading: 30 numbers in total. Rather than writing out all 30, LightNav **predicts three codes that a decoder uses to look up and reconstruct motion features**.

### Think of a Base Sketch and Two Corrections

You could draw a curved route by sketching its general shape, correcting the largest differences, and then refining what remains. RVQ works similarly: the first level represents the main component, the second supplies a correction, and the third addresses the remaining difference. They combine into one path.

In the program, both the “sketch” and the “corrections” are arrays of motion features. Their values are added before conversion to a path. The robot does not drive a base route followed by two correction routes.

### A Code Is an Address; the Stored Content Is Added

Start with one number rather than 30. Suppose we want to represent `1.00` using three prepared tables. The model predicts codes `12, 7, 25`, and the decoder looks up:

| Table | Predicted code | Stored value | Running sum |
| --- | ---: | ---: | ---: |
| Base table | 12 | 0.80 | 0.80 |
| First correction table | 7 | 0.15 | 0.95 |
| Second correction table | 25 | 0.05 | 1.00 |

These codes and values are a constructed teaching example with no distance unit. Code 12 means “read entry 12 in the base table.” It means neither 12 meters nor the number 0.80 itself. **The decoder adds the retrieved contents, not the code indices `12 + 7 + 25`.**

![RVQ: Three Codes, One Path](assets/images/14-rvq-three-codes-v1.png)

*Figure 7.2-5. Three codes select table entries whose motion features are added and decoded into one ten-waypoint path.*

### A Residual Is the Part Still Missing

To represent 1.00, the first level supplies 0.80, leaving 0.20. The second adds 0.15, leaving 0.05. The third adds 0.05. The target value minus the current approximation is the **residual**.

Now the terminology has a concrete meaning: a lookup table is a **codebook**, an entry's stored vector is a **codeword**, and successive tables that approximate the remaining difference form **Residual Vector Quantization (RVQ)**. This example reconstructs the number exactly; real quantization usually leaves some error.

### Return to the Ten-Waypoint Path

The actual bundle used here has 256 entries per table, each containing 30 motion-feature values. The model predicts one index per table. The decoder retrieves the three vectors, adds matching elements, and restores motion units according to the bundled rules:

**Three codes → retrieve and sum codewords → recover ten motion increments → compose ten cumulative poses.**

An increment describes how much to move and turn at one step. A cumulative pose describes where that step ends relative to the starting frame. Section 7.2.6 explains the conversion.

During training, a known demonstration path can be encoded into three codes as a learning target. During navigation, the model predicts codes from language and images, and the decoder reconstructs the proposal. Codebooks are prepared with the model, not rebuilt on every request. Lookup is deterministic; suitability for the scene still depends on the prediction and execution checks.

## 7.2.6 Ten Waypoints: After Turning, Which Way Is Forward?

The decoder has recovered ten motion increments, such as “move forward a little” and “turn left a little.” To turn them into a path, ask: **after each step, where is the robot relative to its starting position, and which way does it face?**

### The Robot Turns; the Starting Frame Does Not

Imagine standing at the robot's departure point. Call its initial forward direction X and its initial left direction Y, and keep those axes fixed. After the robot turns, its own “forward” changes, but your recording axes do not.

Position and heading together form a **pose**, written `(x, y, θ)`. The first two numbers locate the robot relative to the start; the third records its angle relative to the initial heading.

### Follow Three Moves

Start at `(0, 0)`, facing X:

1. **Move forward 0.5 m.** The robot reaches `(0.5, 0)`, still facing X.
2. **Turn left 90° in place.** Position remains `(0.5, 0)`, while heading changes to Y.
3. **Move forward another 0.5 m along the new heading.** The robot moves along Y and finishes at `(0.5, 0.5)`, still facing Y.

![The robot turns while the starting axes stay fixed](assets/images/15-waypoints-turning-v1.png)

*Figure 7.2-6. The starting state and the three subsequent poses. X remains the initial forward direction and Y the initial left direction; forward motion after the left turn increases Y.*

The important detail is step 3: **“forward” refers to the robot's heading at that time.** After a left turn it no longer increases the original X coordinate. A turn in place leaves position unchanged but affects the direction of the next movement.

### Increments Say “How Much More”; Waypoints Say “Where Now”

“Move forward another 0.5 m” is an **increment**. “The robot is now at `(0.5, 0.5)`, facing 90° left of its initial heading” is a **cumulative pose**.

Straight motion makes the difference especially clear. Two successive 0.15 m moves have identical increments, but cumulative positions of 0.15 m and 0.30 m. The second position already includes the first move; adding 0.15 m again would count it twice.

For turning motion, the decoder considers the preceding heading before accumulating translation. This composition of planar position and heading is described mathematically by **SE(2)**. The three-move example is enough for the main idea; formulas are in Advanced Reading.

### LightNav's Ten Returned Poses Are Already Cumulative

Each returned point specifies **a proposed position and heading relative to the robot frame when the input image was captured**. The decoder has already composed the increments. The execution layer must not sum the ten returned poses again.

It uses the robot's room position and heading at capture time to place this local path in the simulation's world frame, then gives the result to the controller. Chapter 7.3 shows how to run the integration.

The ten points specify spatial order, not deadlines. They do not assign each point to a particular second. The controller chooses the travel speed, and actual travel must be recorded separately.

## 7.2.7 How Does the Model Learn to Navigate?

Adding symbols to a vocabulary does not teach a model when to use them. The paper describes three training stages that adapt its vision-language capabilities for navigation.

### Stage 1: Strengthen Spatial and Action Understanding

Recognizing a refrigerator is different from understanding how to reach the floor beside it. Embodied-reasoning mid-training uses tasks involving spatial relationships, image pointing, video question answering, and associations across viewpoints. Judging which opening leads into another room, for example, connects visual understanding to possible actions.

### Stage 2: Learn Navigation Outputs from Demonstrations

Supervised fine-tuning pairs instructions and observations with reference APOS, OPOS, and trajectories. Reference trajectories are encoded into RVQ codes, and the model learns to predict these symbols. This connects visual understanding with the action representation.

Learning only from expert routes leaves a gap: the model may drift during its own runs and encounter views absent from demonstrations. **DAgger (Dataset Aggregation)** adds examples from such states. Run the current policy, obtain corrective labels from a reference policy at states it actually visits, and add those examples to training. The aim is to cover the model's own likely states as well as ideal routes.

### Stage 3: Improve Decisions Using Complete Runs

Matching a demonstration at one decision does not ensure the full task succeeds. Online reinforcement learning executes tasks in simulation and uses the resulting returns to update the policy. The paper uses **GRPO (Group Relative Policy Optimization)**, which compares returns from multiple attempts at the same task to favor better behavior.

![LightNav training stages](assets/lightnav/05-training-stages.gif)

*Figure 7.2-7. Embodied reasoning, supervised fine-tuning, and online reinforcement learning build LightNav's navigation capabilities.*

The stages strengthen spatial/action understanding, teach reference outputs, and refine decisions through run-level feedback. Training data and rewards supply learning signals; during deployment the model predicts from images and language without receiving the target's true world coordinates as a hidden hint.

A new room, camera, or chassis can still differ from training. The model may point to the refrigerator correctly but propose a path that cannot fit through the doorway, or decide to stop too early. Evaluate inputs, predictions, and actual execution together.

## Interpreting Stops and Task Outcomes

APOS STOP is a special state of the pointing channel. Action stopping is expressed by action decoding and the response's top-level `stop` field. Read them separately. If APOS STOP accompanies nonzero motion, the local integration rejects the inconsistency and should record a semantic conflict rather than label every such event a decoding failure.

A model stop, a user STOP, and a collision-protection stop have different causes. **Completing the refrigerator task still requires checking actual position, velocity, collisions, and the task's defined stopping conditions.** Model STOP is an observable decision, not a task-success score.

## Review and Self-Check

The algorithm connects **instruction and visual history → SlowFast selection and compression → two pointing symbols and three action codes → RVQ decoding → ten local cumulative poses**. The execution layer transforms them into a world path, tracks it, and provides new images.

1. How can LightNav use language-model token generation for navigation? What do the two pointing symbols and three action codes represent?
2. What do temporal and spatial compression each reduce? Why does factor-2 pooling turn 64 spatial locations into 16?
3. Why can OPOS on the refrigerator and APOS on the doorway both be reasonable?
4. Explain three-level RVQ using the residual example. How does encoding a training trajectory differ from predicting codes during navigation?
5. Why do forward 0.5 m, left 90°, and forward 0.5 m end at `(0.5, 0.5)`? Why should returned poses not be accumulated again?
6. What does each training stage contribute? Why is independent evaluation needed after a model STOP?

Configuration fields need not be memorized. Consult [Advanced Reading](ADVANCED_READING.md) for formulas, exact settings, source locations, and offline checks. To associate real images, tokens, and all ten poses, follow [Lab 2: Interpret a Real Prediction](../7.3_From_Navigation_Prediction_to_Closed_Loop_Control/LAB_GUIDE.md#lab-2-interpret-a-real-prediction).

## References and Next Steps

The paper, fixed source version, model revision, and teaching-asset checks are listed under [Versions and Source](ADVANCED_READING.md#versions-and-source). Animation timing illustrates the mechanisms rather than measuring inference latency or task performance.

[7.1 Introduction to VLN](../7.1_VLN_Fundamentals/README_en_US.md) · [Back to M7](../README.md) · [7.3 Run the Demo](../7.3_From_Navigation_Prediction_to_Closed_Loop_Control/README_en_US.md)
