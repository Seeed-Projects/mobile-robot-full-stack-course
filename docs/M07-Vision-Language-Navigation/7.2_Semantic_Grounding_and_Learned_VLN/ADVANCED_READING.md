# 7.2 Advanced Reading: Formulas, Configuration, and Source

[English](ADVANCED_READING.md) | [中文](ADVANCED_READING_zh_CN.md)

This page supports [chapter 7.2](README_en_US.md) with exact settings, derivations, and offline teaching-asset checks. It is optional on a first reading: the six self-check questions in the main chapter are sufficient preparation for 7.3. Exact rules refer to the fixed release below; conceptual formulas and implementation choices are identified separately.

## Versions and Source

- [Paper v2](https://arxiv.org/html/2608.30935v2): architecture and training.
- [Pinned upstream source](https://github.com/lightorigins/LightNav-0/tree/3015508b70fb6e30bbd66701e94c9b916dcb8df8): the version checked for this chapter.
- [Pinned model release](https://huggingface.co/LightOriginsHQ/LightNav-0/tree/826dc5fbfa37afa8293d2e336d329b6ffc0bfb64): history configuration and action-decoding resources.
- [Pinned protocol](https://github.com/lightorigins/LightNav-0/blob/3015508b70fb6e30bbd66701e94c9b916dcb8df8/docs/PROTOCOL.md): response coordinates, sessions, and stop fields.

| Operation | Source location |
| --- | --- |
| History selection | `src/lightnav/slowfast.py`: `slowfast_segments` |
| Spatial pooling and time representation | `src/lightnav/processing.py`: `post_vit_spatial_pool`, `_calculate_timestamps` |
| Point codes and pixel conversion | `src/lightnav/vln_utils.py`: `encode_point_pixel`, `decode_point_pixel_center` |
| Codebooks, action decoding, and pose composition | `src/lightnav/traj_vocab.py`: `load_rvq_bundle`, `RVQBundle.decode_waypoints`, `compose_to_abs` |

## Generation and Attention

These formulas describe context processing in the shared backbone; understanding the main chapter does not require calculating the network by hand.

### Generating Five Task Tokens

At decision $t$, the model reads the instruction and visual history and generates:

$$
p_\theta(\mathbf y_t\mid I,O_{\leq t})
=\prod_{j=1}^{5}p_\theta(y_{t,j}\mid I,O_{\leq t},y_{t,<j}).
$$

$I$ is the instruction and $O_{\leq t}$ the observations available for this decision. The output is $\mathbf y_t=(y_{t,1},\ldots,y_{t,5})$. Its $j$th token is $y_{t,j}$, while $y_{t,<j}$ denotes tokens already generated in the same decision. Model parameters are $\theta$, and $p_\theta$ is the conditional probability. The product multiplies five probabilities: each next symbol is predicted using both the input and preceding outputs.

The five task tokens are APOS, OPOS, and one action code from each RVQ level, `L0`, `L1`, and `L2`. RVQ names the representation, not a sixth token. Template and end markers are separate. When generating action codes, the pointing tokens are already in context, allowing spatial intent and action to be generated together.

### How Attention Combines Information

The visual encoder extracts features from local image patches. Visual representations and text then enter a context the model can process jointly. A token is not synonymous with an object: a refrigerator covers several regions, and a visual token need not correspond to just one object.

Transformer attention can be summarized as:

$$
\mathrm{Attention}(Q,K,V)
=\mathrm{softmax}\left(\frac{QK^\top}{\sqrt d}\right)V.
$$

$Q$, $K$, and $V$ are query, key, and value vectors calculated by the network; $d$ is the query/key dimension. The product $QK^\top$ gives matching scores. Scaling by $\sqrt d$ adjusts their magnitude, softmax turns them into weights, and the weights combine $V$.

While processing “refrigerator,” the network can combine appearance features with clues from earlier views. The formula describes information association; its accuracy depends on trained weights and the actual inputs.

## SlowFast Configuration and Time Representation

The paper describes the method; the release configuration specifies which tiers, frame budgets, and pooling factors this deployment uses.

| Name | Meaning | Use here |
| --- | --- | --- |
| Checkpoint | A saved model version; this release also includes configuration and decoding resources | Fix weights and companion settings to one version |
| `eval_config.json` | The release's evaluation configuration file | The inference service also reads image-processing and history settings from it |
| `vlnce` | The navigation-task settings group inside that file | The navigation service selects this group, including the tiers below |

The name comes from **VLN-CE: Vision-and-Language Navigation in Continuous Environments**. Contrast an agent that jumps among preconnected viewpoints with one that moves through a 3D scene using turns and forward motion while encountering obstacles. VLN-CE studies the latter. “Continuous” describes the environment and movement, not uninterrupted inference or uninterrupted driving. See the [official VLN-CE introduction](https://github.com/jacobkrantz/VLN-CE).

Here, selecting `vlnce` simply means selecting the navigation settings in the parameter file. It is not an additional model to install. The MuJoCo refrigerator demo is not thereby an official VLN-CE benchmark evaluation.

Find these settings under `eval_config.json` → `tasks` → `vlnce` → `slowfast_tiers`. The service maps its navigation task to that configuration name.

### Tiers in the Fixed Release

Age is measured in native history frames; time below assumes 4 Hz. The example has 240 frames and current index 239.

| Tier | Age range | Time before current | Temporal selection | Spatial factor | Example count |
| --- | --- | --- | --- | ---: | ---: |
| `current` | 0–1 frames | 0–0.25 s | Latest two consecutive frames | 1 | 2 |
| `fast` | 2–33 frames | 0.5–8.25 s | Consecutive frames | 2 | 32 |
| `mid` | 34–89 frames | 8.5–22.25 s | Adjacent pairs at age intervals of 6 frames | 2 | 20 |
| `long` | 90 frames and older, within available history | 22.5 s and older | Spread across the range, up to 14 candidate pairs | 4 | 26 |
| `anchor` | Initial indices 0 and 1 | Depends on task duration | Retain the start, avoiding duplicate selection | 4 | 2 |

The long tier initially produces 14 candidate pairs. Its oldest pair, frames 0 and 1, is already retained by the anchor. Removing that overlap leaves 13 pairs, or 26 frames. The total is $2+32+20+26+2=82$. Other history lengths do not necessarily produce 82 inputs.

The paper also discusses native resolution and pixel budgets. The Jetson example here uses the fixed release settings above.

### Adjacent Pairs and Structural Padding

The visual encoder groups frames temporally, and the implementation makes each segment's frame count even. Consecutive selection is named `dense`; interval-based adjacent pairs are `burst`; pairs spread across an available range are `span`. The latter two already select pairs. At startup, a one-frame or odd-length segment is padded by repeating its latest frame within the processed input.

This **structural padding** differs from creating a false capture record. Logs still contain only actual captures; padding satisfies the encoding format. If initial frames also belong to current or recent tiers, their finer representation takes priority and anchor duplicates are removed. Compute short-history selections from source rather than applying the 240-frame counts to every case.

### Continuous-Age Formulas

For observation age $\Delta T$, the paper describes a decaying temporal sampling rate:

$$
f_s(\Delta T)=f_{\max}\exp(-\Delta T/\tau_s).
$$

$f_{\max}$ is the maximum rate and $\tau_s$ a time scale. Increasing age decreases the exponential term, giving sparser sampling of older observations.

A trend for the spatial pooling factor is:

$$
s(\Delta T)=\max\{1,\lfloor\exp(\Delta T/\tau_p)\rfloor\}.
$$

$\tau_p$ uses the same time unit as $\Delta T$. Greater age tends to produce a larger factor and coarser features. The floor operation defines this factor; rounding output grid dimensions upward during pooling is a separate operation.

These expressions explain the design trend. The checkpoint used here employs discrete tiers and fixed factors, not direct runtime evaluation of these exponentials for each image.

### Frame Index, Frame Age, and Relative Time

- **Absolute frame index** identifies a capture within the task, starting at 0 internally.
- **Frame age** measures how many frames precede the current one and determines the tier.
- **Model time representation** supplies ordering and time-gap cues. The setting `timestamp_relative=true` uses time relative to the current observation, rather than elapsed time from task start.

With current index 239, frame 80 has nominal capture time $80/4=20$ s after task start, but age $(239-80)/4=39.75$ s. The configuration uses the relative-time convention and averages times within each adjacent-frame temporal block. Thus the block containing 238 and 239 corresponds to about 0.125 s ago, rather than assigning every latest block a time of zero.

These model times derive from frame indices and the 4 Hz convention. The integration separately records actual capture time, robot root pose, and camera extrinsics for sampling checks, path transforms, and evidence association. Those records are not all supplied as an extra model state vector.

Selection, pooling, and caching serve different purposes. Selection reduces image count; pooling reduces encoded spatial locations; caching reuses eligible computations. When a spatial grid is not divisible by a factor, the implementation rounds the output dimensions up and uses adaptive average pooling.

## Pointing Codes and Special Values

The implementation divides the image into 48 columns and 27 rows. Ordinary grid codes start at 1, reserving 0:

$$
\mathrm{id}=1+rW_g+c,\qquad W_g=48,\quad H_g=27.
$$

Recover the cell center as:

$$
c=(\mathrm{id}-1)\bmod48,\quad
r=\lfloor(\mathrm{id}-1)/48\rfloor,
\qquad
u=\frac{c+0.5}{48}W,\quad
v=\frac{r+0.5}{27}H.
$$

$r,c$ are zero-based row and column; $W_g,H_g$ are grid dimensions; $W,H$ are the corresponding original image dimensions. Pixel $(u,v)$ starts at the upper-left, with $u$ rightward and $v$ downward. This differs from the robot's forward/left axes.

For a 480×270 frame, each cell is 10×10 pixels. Subtracting 1 from `opos_id=505` gives 504: row 10, column 24, center $(245,105)$. The normalized grid can map to different display sizes; always inspect it on the request's matching image.

The paper's grid equations and the code's reserved-index offset must be read together. Omitting the offset shifts the selected cell.

### Special Values

| Channel | Code | Meaning in this version |
| --- | ---: | --- |
| APOS | 0 | No ordinary image point |
| APOS | 1–1296 | Image grid point |
| APOS | 1297 / 1298 / 1299 | Rotate left / rotate right / stop |
| OPOS | 0 | No visible target location provided |
| OPOS | 1–1296 | Target image grid point |

Interpret `visible=true` as the model providing a target point. To check it, associate `request_id` with the original JPEG, recover the pixel, and inspect whether the point or cell overlaps the actual target region. Record manual visibility assessment separately from the model field.

A pixel does not specify unique depth. Recovering a world position would also need depth, camera parameters, or other geometry. The local action path is decoded directly from RVQ codes, not obtained by treating OPOS pixels as meters.

## RVQ Formulas and Decoding Resources

A common residual-quantization formulation begins with feature $z$ and residual $r^{(0)}=z$. At level $\ell$, select an entry and subtract its contribution:

$$
k_\ell=\arg\min_k d\bigl(r^{(\ell)},e_k^{(\ell)}\bigr),
\qquad
r^{(\ell+1)}=r^{(\ell)}-e_{k_\ell}^{(\ell)}.
$$

- $\ell$ is the level index, here 0, 1, or 2; $k$ indexes a candidate entry.
- $r^{(\ell)}$ is the residual entering that level.
- $e_k^{(\ell)}$ is entry $k$ in its codebook.
- $d$ measures difference; $\arg\min$ chooses the index giving the smallest difference, named $k_\ell$.

This summarizes approximation followed by residual correction. The release configuration also specifies how its tools compare trajectories; this general expression is not the complete implementation of local training.

For decoding, indices are already known:

$$
\hat z=e_{k_0}^{(0)}+e_{k_1}^{(1)}+e_{k_2}^{(2)}.
$$

$\hat z$ is an approximate feature vector, not the final path. For scale coefficient $w_q$, the decoder calculates $u_q=\hat z_q/w_q$ elementwise, then reshapes the 30 restored values into ten motion increments. These coefficients come with the model rather than being selected by the controller.

### The Decoding Bundle

The **bundle** is the collection of codebook arrays, scale coefficients, and metadata. `action_tokenizer/manifest.json` describes the files and representation.

| Meaning | Source field | Value and interpretation |
| --- | --- | --- |
| Poses reconstructed per output | `horizon` | 10 spatial poses, not ten seconds |
| Values in each codeword | `feature_dim` | 30: ten groups of three motion components |
| Codebook levels and sizes | `levels` | `[256,256,256]`: three tables, each with 256 entries |
| Motion representation | `representation` | `se2_diff`: increments relative to the preceding pose |
| Feature representation | `feature_space` | `weighted_diff`: weighted motion increments |
| Coefficient file | `jacobian_weights_file` | `jacobian_weights.npy`: divide the codeword sum elementwise by these weights |
| Encoding comparison settings | `objective`, `encode` | `ade_v1`: average displacement error plus heading weight `heading_weight=0.3` |
| Action token format | `vocab_layout.token_format` | `<act_l{level}_{code}>`, such as the teaching example `<act_l1_7>` |
| Explicit stop identifier | `stop` | First-level code 6; canonical combination `[6,122,174]` |

The fixed decoder identifies an explicit stop from first-level code 6. The canonical stop codeword sum may only be near zero, so exact path zeros are not a substitute for checking that identifier. See “Stop Recognition and Local Handling” below for response processing.

Three tables of 256 entries permit $256^3=16{,}777{,}216$ index combinations. This counts combinations, not feasible or safe paths. The fixed interface requires all three codes; fewer-level outputs discussed in the paper represent another configuration choice.

### Codes Used in the Teaching Examples

The main chapter's scalar example uses codes 12, 7, and 25 for values 0.80, 0.15, and 0.05. The older RVQ animation retains a two-dimensional addition example. Both are constructed feature examples with no distance unit. Its ten-point curves come separately from offline decoding of release codes `[14,42,30]`.

Gray, blue, and cyan curves show successive level contributions. Partial-level curves illustrate refinement; the online interface still requires three codes. The approximately 11.2 cm and 3.6 cm values are mean pointwise differences between partial and full decoded paths, not vehicle tracking errors. Per-level codeword magnitude statistics are codebook averages, not fixed correction sizes for each prediction.

## SE2 Pose Composition

A planar pose is $(x,y,\theta)$, with positions in meters and heading in radians. **SE(2)** is the Special Euclidean Group in two dimensions: planar translations and rotations and their composition. The “2” refers to the plane; a pose still has two position components and one heading component.

The practical issue is to combine motion relative to each preceding robot frame into poses relative to a fixed starting frame.

For increment $(\Delta x_j,\Delta y_j,\Delta\theta_j)$, forward is $\Delta x_j$, left is $\Delta y_j$, and $\Delta\theta_j$ is the heading change. Given preceding cumulative pose $(x_{j-1},y_{j-1},\theta_{j-1})$:

$$
x_j=x_{j-1}+\cos\theta_{j-1}\,\Delta x_j-\sin\theta_{j-1}\,\Delta y_j,
$$

$$
y_j=y_{j-1}+\sin\theta_{j-1}\,\Delta x_j+\cos\theta_{j-1}\,\Delta y_j,
$$

$$
\theta_j=\operatorname{wrap}(\theta_{j-1}+\Delta\theta_j).
$$

Start at $(x_0,y_0,\theta_0)=(0,0,0)$. The first two equations rotate the local translation into the starting frame and add it to the previous position. The third updates heading.

For the last move in the turning example, preceding heading is $\pi/2$ and increment is $(0.5,0,0)$. Since $\cos(\pi/2)=0$ and $\sin(\pi/2)=1$, X remains unchanged and Y increases by 0.5 m, giving $(0.5,0.5,\pi/2)$.

$\operatorname{wrap}$ normalizes equivalent headings. The code uses $[-\pi,\pi)$, so 180° is written as $-\pi$, equivalent to $\pi$. This does not imply an extra revolution. Heading differences must also respect angle equivalence.

The `abs` in `compose_to_abs` means cumulative relative to the capture frame, not absolute scene coordinates. Execution still applies one transform using the robot root pose at capture.

## Additional Training Details

### Supervised Learning and DAgger

**Supervised Fine-tuning (SFT)** pairs inputs with reference answers. Navigation samples pair observations and instructions with pointing and trajectories; trajectories become RVQ codes in the target output. They come from demonstrations or data generation. Visual Question Answering (VQA) and reasoning examples are also mixed into training to retain existing capabilities.

For supervised output positions $M$, cross-entropy is:

$$
\mathcal L_{\mathrm{CE}}
=-\sum_{j\in M}\log p_\theta(y_j\mid x,y_{<j}).
$$

$x$ is the input and $y_j$ the reference token. The negative log penalizes assigning low probability to it. Raising the correct OPOS probability from 0.2 to 0.8 lowers its term from about 1.609 to 0.223. Training fit and full navigation outcomes remain separate measures.

Expert-only routes can cause **distribution shift**: the learner visits positions and views different from those in demonstrations. **DAgger (Dataset Aggregation)** runs the current policy, obtains corrective labels at visited states, and adds them to training. The expert may be a qualified reference policy or planning process, rather than a human operator.

The paper reports DAgger examples from policy-induced states. That explains the intended coverage without establishing which annotator produced every sample. Cross-room and cross-chassis transfer still need testing.

### Online Reinforcement Learning and GRPO

Navigation also depends on the consequences of a sequence of actions. Online reinforcement learning runs the model in simulation and updates parameters from complete outcomes. A **rollout** is one sequence of observations, actions, and consequences from start to termination. **GRPO (Group Relative Policy Optimization)** compares returns across attempts under the same task conditions.

For rollout $g$ with return $R^{(g)}$:

$$
A^{(g)}=\frac{R^{(g)}-\mu_R}{\sigma_R+\epsilon}.
$$

$\mu_R$ and $\sigma_R$ are the group mean and standard deviation; $\epsilon$ avoids division by zero. Advantage $A^{(g)}$ measures performance relative to the group.

For teaching returns $[0,1,2]$, the mean is 1 and population standard deviation about 0.816. Ignoring tiny $\epsilon$, advantages are approximately $[-1.225,0,1.225]$. Updates favor better outputs and suppress worse ones. Identical returns give no within-group distinction. The actual objective additionally clips probability ratios and uses a KL-divergence constraint against a reference policy.

Action tokens have language-model probabilities just like pointing tokens, so they can participate in these updates. Discrete actions connect trajectory representation with the language-model training interface.

The three stages use different signals: spatial/action reasoning tasks, reference outputs, and actual rollout returns. Training-time privileged information and reward distances differ from deployment inputs. The current demo does not inject true refrigerator coordinates into the model.

## Stop Recognition and Local Handling

Record explicit action STOP, near-zero-path postprocessing, APOS STOP, and independent task scoring separately. First-level action code 6 identifies explicit stopping, and the manifest records `[6,122,174]`. Near-zero zeroing is a separate postprocessing route.

APOS STOP with nonzero motion may satisfy upstream token syntax, but the local integration adds a consistency rejection; see the [strict decoding adapter](../code/overlay/src/lightnav/serving/strict_decode.py). If a response omits its path, record “not returned.” An offline reconstruction is not the original response.

## Assets and Offline Checks

- [SlowFast materials](../code/teaching/slowfast/README.md): 240-frame selection data, temporal/spatial animations, and checks.
- [LightNav algorithm materials](../code/teaching/lightnav/README.md): five animations, source data, numerical measures, and checks.
- [Lab 2: Interpret a Real Prediction](../7.3_From_Navigation_Prediction_to_Closed_Loop_Control/LAB_GUIDE.md#lab-2-interpret-a-real-prediction): associate real inputs, tokens, and paths.

Teaching animations combine illustrations and deterministic calculations. Static versions and earlier illustrations remain available below.

| Topic | Static explanation and illustration |
| --- | --- |
| Architecture | [Decision](assets/lightnav/01-decision-loop.png), [model structure](assets/images/04-inside-lightnav.jpg) |
| SlowFast | [Selection](assets/slowfast/time-selection.png), [pooling](assets/slowfast/spatial-pooling.png), [combined example](assets/slowfast/combined-example.png) |
| APOS/OPOS | [Grid](assets/lightnav/02-pointing-grid.png), [dual pointing](assets/images/06-goal-affordance.jpg) |
| RVQ | [Refinement](assets/lightnav/03-rvq-residual.png), [RVQ illustration](assets/images/07-rvq.jpg) |
| Poses | [Increments and cumulative poses](assets/lightnav/04-se2-compose.png), [motion illustration](assets/images/09-motion-waypoints.jpg) |
| Training | [Stages](assets/lightnav/05-training-stages.png), [training illustration](assets/images/08-learning-navigate.jpg) |

Also see the [LightNav input/output overview](assets/images/lightnav-overview-reference.png).

[Back to Chapter 7.2](README_en_US.md)
