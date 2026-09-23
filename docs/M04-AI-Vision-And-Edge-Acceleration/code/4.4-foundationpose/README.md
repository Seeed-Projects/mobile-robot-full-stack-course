# M4.5 Native NVlabs FoundationPose 6D Object Pose

The physical directory name and old `run_m4_4_demo.sh` entry are retained for
compatibility. This is the unverified native/PyTorch scaffold, not the M4.4
Isaac ROS official quickstart. See `../4.4-isaac-ros-foundationpose/README.md`
for the Isaac ROS path and the canonical `../PROJECT_STATUS.md` for status.

> All relative paths below are relative to the module root,
> `modules/m04-ai-vision-and-edge-acceleration/`.

NVlabs FoundationPose over an Orbbec Gemini 2 RGB-D stream, registering and
tracking a known CAD object and publishing its 6D pose.

**Status: BLOCKED.** The package installs, but nothing it depends on at runtime
exists on this machine, and the scaffold's FoundationPose calls were **invented**
rather than written against the upstream API.

## What is actually wrong

### 1. The API is fabricated (CRITICAL)

Verified against the real upstream source, not against a summary of it. The
constructor in `estimater.py` is:

```python
class FoundationPose:
  def __init__(self, model_pts, model_normals, symmetry_tfs=None, mesh=None,
               scorer: ScorePredictor=None, refiner: PoseRefinePredictor=None,
               glctx=None, debug=0, debug_dir='...')
```

`bev_pose/foundationpose_engine.py` instead calls:

```python
FoundationPose(model_dir=..., scorer_module=..., refiner_module=...,
               mesh_file=..., device=...)
```

**Five parameters that do not exist**, and it omits both required ones
(`model_pts`, `model_normals`). The fallback path is worse: it imports
`PoseRefiner`, which is not a class in that module at all (the real name is
`PoseRefinePredictor`), constructs `ScorePredictor(weights=..., device=...)` when
the real signature is `ScorePredictor(amp=True)`, and then calls `FoundationPose`
without ever importing it.

`track_one` is wrong too. Upstream:

```python
def track_one(self, rgb, depth, K, iteration, extra={})
```

The scaffold passes `pose_init=self._last_pose` — no such parameter exists, and
the tracker holds its previous pose in `self.pose_last` internally. There is no
re-registration-per-frame path to build here; `track_one` is simply called again.

### 2. The canonical usage, for whoever fixes this

From upstream `run_demo.py`, which is the shape any rewrite must follow:

```python
mesh    = trimesh.load(mesh_file)
scorer  = ScorePredictor()
refiner = PoseRefinePredictor()
glctx   = dr.RasterizeCudaContext()          # nvdiffrast
est = FoundationPose(model_pts=mesh.vertices,
                     model_normals=mesh.vertex_normals,
                     mesh=mesh, scorer=scorer, refiner=refiner,
                     glctx=glctx, debug=debug, debug_dir=debug_dir)
pose = est.register(K=K, rgb=color, depth=depth, ob_mask=mask, iteration=n)
pose = est.track_one(rgb=color, depth=depth, K=K, iteration=n)
```

Note `model_pts`/`model_normals` are **arrays sampled from the mesh**, not a file
path — which is why the scaffold's `mesh_file=` could never have worked.

### 3. `phase0_foundationpose_verify.sh` cannot run

Line 285 is a hard Python syntax error:

```python
pose = est.track_one(rgbs[i], depths[i], K, pose, masks[i]=masks[i] if i%10==0 else None)
```

`masks[i]=masks[i]` uses a subscript as a keyword name. Independently, `pose` is
passed as the 4th positional argument, which upstream defines as `iteration`.
The script also runs under the system `python3`, which has no torch; FoundationPose
needs the conda `py310` environment.

`fetch_foundationpose_testdata.sh` also cannot work: it sparse-checks-out
`test_data/test/cup` from the NVlabs repository, and **that path does not exist
in the repository** — the tree has 67 tracked paths and no `test_data/` or
`demo_data/` at all.

## Dependency status

| Dependency | State |
| --- | --- |
| NVlabs/FoundationPose checkout | **present** at `/home/seeed/workspace/third_party/FoundationPose` (outside the course repo, per the brief) - revision `a1b694b83e633c2cb6115b9063d940a687759392` |
| Refiner weights `2023-10-28-18-33-37` | downloading — `model_best.pth` from `hf-mirror.com/gpue/foundationpose-weights` |
| Scorer weights `2024-01-11-20-02-45` | queued |
| Official demo data (`demo_data/`) | **UNOBTAINABLE** — Google Drive only, and Drive is unreachable from this Jetson |
| Orbbec SDK / driver | not installed (0 of 279 ROS packages) |
| Orbbec Gemini 2 hardware | **not attached** (`lsusb` shows no vendor `2bc5`) |
| `nvdiffrast`, `mycpp`, `trimesh`, `tf_transformations` | not installed in conda `py310` |

The weights are reachable only because `gpue/foundationpose-weights` mirrors the
two folder names the upstream README names. That mirror declares
`license: cc-by-nc-4.0`; treat that as the mirror's declaration, not as NVIDIA's
terms.

**Google Drive is blocked** from this Jetson — `drive.google.com`,
`drive.usercontent.google.com` and `docs.google.com` all time out, while GitHub,
`hf-mirror.com` and `pypi.org/pypi/*/json` all work.

## What exists now

A real CAD model for the target object, converted and shipped:

```
models/m4/pose/
  object.yaml                       tracked - measured metadata, source of truth
  model.obj                         gitignored - 36,504 triangles, metres
  original/gl_sft1200_opal.step     gitignored - the untouched source CAD
```

| Field | Value |
| --- | --- |
| Object | GL.iNet GL-SFT1200 "Opal" portable travel router |
| Source | `gl_sft1200_opal.step`, build123d 0.11.1, AP214 |
| Measured bbox (CAD units) | 111.671 x 80.000 x 75.000 |
| **Units** | **mm** — the STEP header declares `SI_UNIT(.MILLI.,.METRE.)` |
| `scale_to_meters` | **0.001**, derived from the file, not assumed |
| Resulting size | 0.1117 x 0.0800 x 0.0750 m, diameter 0.1455 m |
| `symmetry` | `none`, verified — the -Y port face and +Y antenna hinges break the 180 degree rotational ambiguity (a left/right mirror is a reflection, not a rotation) |

Note the CAD models the **antennas raised and splayed**: the widest axis reaches
111.67 mm against the 105 mm body. The physical unit must be posed the same way
or the mesh will not register.

## Hard gates that are still unmet

Per the brief, these are not optional:

1. **FoundationPose standalone has not run.** The repo is cloned but the weight
   download had not finished and nothing has been executed.
   The gate is a genuine gate — no ROS integration work should be done until the
   official path runs.
2. **Official demo data is unobtainable**, so the "test with official data first"
   step cannot be satisfied as written. The only data that could be used is the
   user's own object captured from a depth camera — which does not exist here yet.
3. **No Orbbec hardware and no driver.** The existing
   `launch/orbbec_gemini2.launch.py` names a package (`orbbec_camera`) and
   executable (`orbbec_camera_node`) that **do not exist**; it was written against
   an invented interface. Per the brief, the installed driver's real names must be
   inspected before that launch file is touched.
4. **Topic namespace collides.** `foundationpose_node.py` defaults its RGB topic
   to `/perception/cameras/front/image`, the same topic 4.1-4.3 consume. Running
   both would put two publishers on it.

## Other defects found in the scaffold (not yet fixed)

- `foundationpose_node.py` does not check `rgb.shape[:2] == depth.shape[:2]`,
  while `orbbec_gemini2.launch.py` requests 1280x720 colour against 640x576
  depth. A mismatch is silent.
- `object_mask_node.py` silently `cv2.resize`s depth to the RGB shape rather than
  failing — the brief forbids exactly that.
- The published quaternion is never normalised and NaN/Inf is never rejected.
- `bev_pose` has **zero** tests.

## Next steps, in order

1. Finish the clone and the weight download; record the exact upstream commit hash.
2. Install `trimesh`, `tf_transformations`, then build `mycpp` and `nvdiffrast`
   against `/usr/local/cuda-12.6/bin/nvcc` (present).
3. Rewrite `foundationpose_engine.py` against the API quoted above and fix
   `phase0_foundationpose_verify.sh` (line 285 plus the interpreter).
4. Run the official standalone path. **This is the gate.** If it does not pass,
   do not proceed.
5. Only then: attach the Gemini 2, install `OrbbecSDK_ROS2`, inspect the real
   driver names, and build the ROS pipeline.
