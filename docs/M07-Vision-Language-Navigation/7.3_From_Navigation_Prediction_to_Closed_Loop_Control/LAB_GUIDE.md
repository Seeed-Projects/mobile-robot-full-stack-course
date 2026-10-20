# M7 Lab Guide: Refrigerator Navigation, Jetson Deployment, and Custom Chassis

[English](LAB_GUIDE.md) | [中文](LAB_GUIDE_zh_CN.md)

The goal is to observe and explain camera capture → real model tokens → decoded path → controller tracking → new images after movement. Not every instruction will succeed. Preserve model stops, unsafe paths, stale results, and observation failures. These exercises use simulation, not a physical chassis.

Read [7.2](../7.2_Semantic_Grounding_and_Learned_VLN/README_en_US.md) and [7.3](README_en_US.md) first. This guide retains the complete setup commands. The refrigerator instruction is an unscored custom task; earlier doorframe records support verifiable token and scoring exercises.

## Routes and Preparation

- **Route A:** Use an instructor-prepared device. Check status and open the page; allow about 10–15 minutes for preparation.
- **Route B:** Prepare a new device, including source, environments, model, and CAD. Complete this before class; duration depends on network, caches, and hardware.
- **Without a device:** Use the [archived real records](../code/evidence/README.md) for Labs 2 and 5. Label the work as offline analysis, rather than claiming a new GPU or navigation run.

For each exercise, record actions, expected observations, diagnosis, and deliverables. Only one group should control a shared device; others observe to avoid cancelling each other's tasks.

### Identify Which Machine Holds Each File

The **Jetson project directory** runs the model and demo. The **course repository** supplies tools and archived examples. **Your computer** opens SSH tunnels and the browser. The repository may be on Jetson or your computer, but an analysis terminal must have access to both its tools and the relevant logs.

In a terminal that needs course tools, set the actual repository location on that machine:

```bash
export COURSE_ROOT=/absolute/path/to/mobile-robot-full-stack-course
export MATERIALS="$COURSE_ROOT/docs/M07-Vision-Language-Navigation/code"
test -f "$MATERIALS/tools/analyze_run.py"
```

Expect exit code 0. If the file is absent, locate the repository first. Environment variables apply only to that terminal and its children; set them again in a new terminal. Route A's service checks do not need the repository, but Labs 2 and 5 do.

## Route A: Use an Already-Deployed Device

### A1. Check Status on Jetson

Run in the **Jetson terminal**. These paths and username match the verified deployment:

```bash
cd /home/seeed/workspace/LightNav-0
bash scripts/jetson_lightnav.sh status
python3 scripts/jetson_sim_demo.py status
ss -ltnp | awk '$4 ~ /:(8050|8091)$/ {print}'
df -h .
free -h
```

`status` inspects services. An open port 8050 does not establish GPU inference correctness. The model has process-identity and ready records; the demo additionally needs live threads and fresh frames.

- Ready model: reuse it without loading a second copy.
- Stopped model: the device owner starts it with `bash scripts/jetson_lightnav.sh start` and waits for ready status.
- Running demo: check for an active task; do not restart another group's run.
- Stopped demo: choose **one** of the following asset configurations.

For the original deployment assets:

```bash
python3 scripts/jetson_sim_demo.py start --host 127.0.0.1 --port 8091
```

For course assets prepared through Route B:

```bash
python3 scripts/jetson_sim_demo.py start --host 127.0.0.1 --port 8091 \
  --chassis-config course-assets/dm-h65/config.json \
  --task-config course-assets/task.json
```

Display resolution defaults to 960×540. Use `--resolution 640x360` to reduce display load if needed; model images remain 480×270. Do not repeatedly start an existing instance.

```bash
curl -fsS http://127.0.0.1:8091/api/health
```

Expect `ok=true`, `model_ready=true`, `simulation.physics_alive=true`, `render_alive=true`, `frame_age_s<1`, `telemetry_alive=true`, and `telemetry_error=null`. Check again after a few seconds for increasing `sim_time` and frame counts. If `model_ready=false`, refreshing the browser does not make this implementation rediscover the model automatically. Once 8050 is ready, only the owner should restart the group's idle demo.

A read-only check on 2026-10-15 found the model running and the demo stopped. Historical statements that a service was left running do not replace a current check.

### A2. Open Access Tunnels from Your Computer

Run on **your computer** using your device's SSH address. `j50-robotics` in older records is a machine-specific alias, not a requirement.

```bash
export JETSON_HOST=seeed@YOUR_JETSON_IP
ssh -N -o ControlMaster=no -o ControlPath=none \
  -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18091:127.0.0.1:8091 "$JETSON_HOST"
```

Leave that terminal open, and run in a second local terminal:

```bash
export JETSON_HOST=seeed@YOUR_JETSON_IP
ssh -N -o ControlMaster=no -o ControlPath=none \
  -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18092:127.0.0.1:8091 "$JETSON_HOST"
```

Open `http://127.0.0.1:18091/`. At this address the page uses 18091 for video and 18092 for control. Independent SSH connections keep video congestion from blocking control. Keep both connections and the expected local ports. The Jetson backend connects to 8050, so the browser needs no model-port tunnel.

On a managed LAN, the owner can alternatively bind the demo explicitly to `0.0.0.0` and access `http://<Jetson-IP>:8091/`. The default course route uses loopback and tunnels; changing the bind address is a separate connection choice.

## Route B: Prepare a New Device

### B1. Source and Material Versions

Obtain the course repository, [course code package](../code/README.md), and separately supplied instructor CAD bundle. An upstream LightNav clone alone lacks the course launchers and chassis integration.

Run the following on a **new Jetson**. The example uses `/home/seeed/workspace/LightNav-0`, which must not already exist. Use Route A for an existing deployment.

```bash
export COURSE_ROOT=/path/to/mobile-robot-full-stack-course
export MATERIALS="$COURSE_ROOT/docs/M07-Vision-Language-Navigation/code"
python3 "$MATERIALS/tools/check_materials.py"
mkdir -p /home/seeed/workspace
cd /home/seeed/workspace
git clone https://github.com/lightorigins/LightNav-0.git
cd LightNav-0
git checkout --detach 3015508b70fb6e30bbd66701e94c9b916dcb8df8
python3 "$MATERIALS/tools/install_overlay.py" --project "$PWD"
python3 "$MATERIALS/tools/install_overlay.py" --project "$PWD" --apply
```

The first installer call is a preview; the second applies files only if the destination matches the expected upstream baseline or the course copy. Different local content is rejected. The overlay includes shared modules, so prepare a fresh clone rather than applying it to the classroom's running project.

### B2. Hardware, Space, and uv

The target is aarch64, JetPack 6.2.1, CUDA 12.6, and AGX Orin 32GB. Plan roughly 30 GiB or more for a fresh installation, with additional room for CAD, caches, and recordings. Model files alone occupy about 9.04 GiB. Actual requirements vary. Starting a task requires at least 3 GiB available memory and 3.5 GiB free disk; logs also consume space.

```bash
uname -m
cat /etc/nv_tegra_release
nvcc --version
df -h /home/seeed/workspace
free -h
command -v uv
```

Reuse an installed uv. If it is absent, use its official user-space installer:

```bash
curl -LsSf https://astral.sh/uv/install.sh -o /tmp/lightnav-uv-install.sh
sh /tmp/lightnav-uv-install.sh
export PATH="$HOME/.local/bin:$PATH"
uv python install 3.10.21 3.11.16
```

Keep system Python and CUDA unchanged. The frozen Seeed profile uses `/home/seeed/.local/bin/uv`, and two wheel URLs in the model lock contain the original deployment path. This guide directly supports that username and directory. For another location, edit the profile's uv path and both `file://` prefixes in a **new clone**, retaining wheel names, versions, and hashes. Save the migration diff; environment variables alone do not relocate that profile.

### B3. Model Environment and GPU Validation

Use the [Seeed JetPack 6.1/6.2 PyTorch reference](https://github.com/Seeed-Projects/reComputer-Jetson-for-Beginners/tree/main/3-Basic-Tools-and-Getting-Started/3.5-Pytorch) to obtain the cp310, aarch64, CUDA 12.6 wheels below. Place them in the wheel directory before running setup. They are external assets; x86_64, CPU, or other JetPack builds are not substitutes.

```bash
cd /home/seeed/workspace/LightNav-0
mkdir -p .jetson-lightnav/py310/wheels
cp "$MATERIALS/locks/requirements-lightnav-py310.lock" \
  .jetson-lightnav/py310/requirements.lock
sha256sum .jetson-lightnav/py310/wheels/torch-2.7.0-cp310-cp310-linux_aarch64.whl
sha256sum .jetson-lightnav/py310/wheels/torchvision-0.22.0-cp310-cp310-linux_aarch64.whl
bash scripts/jetson_lightnav.sh setup --profile seeed310
/home/seeed/.local/bin/uv pip check --python .venv-lightnav-py310/bin/python
```

Expected hashes are in [Environment Locks and External Binaries](../code/README.md#environment-locks-and-external-binaries). These are hashes measured for the project's downloads, not a claimed Seeed signature. The installer also unpacks NVIDIA cuSPARSELt 0.8.1.1 inside the project and selects it through process-level `LD_LIBRARY_PATH`, without copying libraries into system directories.

On the original device, ordinary-user CUPTI access was restricted, and complete validation included an independent profiling run. The following temporary privileged check is for execution after the instructor verifies the system; sudo applies only to this check process:

```bash
export P=/home/seeed/workspace/LightNav-0
sudo env LD_LIBRARY_PATH="$P/.jetson-lightnav/py310/native/libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive/lib" \
  "$P/.venv-lightnav-py310/bin/python" "$P/scripts/jetson_lightnav_gpu_check.py" \
  --output "$P/.jetson-lightnav/py310/evidence/gate1-privileged" \
  --python-version 3.10.21 --torch-version 2.7.0 --vision-version 0.22.0
bash scripts/jetson_lightnav.sh check --profile seeed310
```

Expect a combined PASS, finite numerical results, and real CUDA kernels. Archived GPU evidence cannot serve as a new device's PASS. Distinguish profiling permission errors from library or operator failures; do not silently fall back to CPU or reinstall system CUDA.

### B4. Fixed Model and Simulation Environment

```bash
bash scripts/jetson_lightnav.sh download --profile seeed310
cat .jetson-lightnav/model-manifest.json
```

Use `LightOriginsHQ/LightNav-0@826dc5fbfa37afa8293d2e336d329b6ffc0bfb64`. Its 18 files include weights, tokenizer, processor, evaluation settings, and RVQ bundle; no separate base Qwen download is needed. Expect a passing manifest with matching revision and file checksums. If Jetson lacks network access, transfer the complete same-revision files and official metadata from a connected computer, then use the installer's verify-only entry. A lone weights file or skipped hashes is insufficient.

Create the second environment in the simulation directory:

```bash
cd /home/seeed/workspace/LightNav-0/mujoco_demo
uv venv --python 3.11.16 .venv-cpu
uv pip sync --python .venv-cpu/bin/python "$MATERIALS/locks/requirements-demo-cpu.lock"
uv pip check --python .venv-cpu/bin/python
MUJOCO_GL=egl .venv-cpu/bin/python - <<'CHECK'
import mujoco
model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="plane" size="1 1 .1"/></worldbody></mujoco>')
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
with mujoco.Renderer(model, height=270, width=480) as renderer:
    renderer.update_scene(data)
    image = renderer.render()
    print(mujoco.__version__, image.shape, image.dtype)
CHECK
```

Expect MuJoCo 3.11.0, `(270,480,3)`, and `uint8`. The model lock contains package hashes. The simulation lock fixes versions but has no per-wheel hashes and includes `-e .`; resolve it from `mujoco_demo` without adding `--require-hashes`. The locks provide different guarantees.

### B5. CAD, Scene, and Configuration

The course retains upstream scene `val_2` rather than downloading an entire dataset. Check the scene XML and referenced resources. Resolve Git LFS pointers or missing assets before proceeding.

The instructor supplies `dm-h65-course-assets-20261015.tar.gz`, with 45 genuine simplified CAD meshes, about 4.1 MB compressed. Its checksum is in [asset-bundle metadata](../code/configs/asset-bundle.json). There is currently no public download URL: a student without that bundle cannot complete a fresh DM-H65 setup. This course rewrite does not establish redistribution rights or publish the meshes.

```bash
cd /home/seeed/workspace/LightNav-0
export ASSET_BUNDLE=/path/to/dm-h65-course-assets-20261015.tar.gz
sha256sum "$ASSET_BUNDLE"
python3 "$MATERIALS/tools/prepare_assets.py" --project "$PWD" --bundle "$ASSET_BUNDLE"
python3 "$MATERIALS/tools/prepare_assets.py" --project "$PWD" --bundle "$ASSET_BUNDLE" --apply
python3 "$MATERIALS/tools/preflight.py" --project "$PWD" \
  --chassis-config course-assets/dm-h65/config.json --task-config course-assets/task.json
```

The preparer checks every mesh SHA256 and relocates geometry-JSON paths in the course copy into the project. It needs no physical SDK directory and opens no device. Frozen wheel radius is approximately 0.085428 m and track width 0.370294 m; do not substitute old SDK examples such as 0.0825/0.42. Task configuration retains its original registration and migration marker.

Then start model port 8050 and demo port 8091 through Route A. The course overlay adds request association and diagnostics, so arbitrary third-party model services are not automatically protocol-compatible.

## Lab 1: Check the Device and Views

**Actions:** Read health twice, at least two seconds apart. Inspect first-person, third-person, prediction-capture, and top-down views. Try view switching and collapsing the inset.

**Expected:** Live frames keep updating, with a clear “no prediction” state before inference. The first-person view comes from the approximately 1 m virtual camera; the third-person view shows the CAD chassis. The latest model observation updates less often than display frames, so its still image is not evidence that rendering has stalled.

**Diagnosis:** Connection refusal on 8091 points to the demo; model readiness to 8050; video without status to the control channel or telemetry; `RENDER_ERROR` to EGL or resources. Read logs before reinstalling.

**Deliverables:** Device and environment versions, simulation-time/frame differences from two health responses, and a chassis-page screenshot labeled with time and task state.

## Lab 2: Interpret a Real Prediction

Without a device, run from the course repository root:

```bash
python3 docs/M07-Vision-Language-Navigation/code/tools/analyze_run.py docs/M07-Vision-Language-Navigation/code/evidence \
  --state docs/M07-Vision-Language-Navigation/code/evidence/actual-state.jsonl --task-id navigation-0001
```

**Actions:** Open `evidence/prediction-01/input.jpg` and `raw.json`. Check the JPEG hash, `seq`, `request_id`, capture pose, APOS/OPOS pixels, raw tokens, top-level `stop`, and all ten poses. `seq` counts session images, not predictions. Transform the first local point into the world frame by rotating and translating it with the capture-time root pose.

**Expected:** All five archived inputs pass their hashes. The first prediction has `seq=4`, action codes `[187,39,97]`, and `stop=false`. A second point near 0.300 m is already cumulative; do not add it to the first point again.

**Diagnosis:** A mismatched image hash suggests an association problem. Mark missing fields “not returned.” Inspect OPOS on the original image before deciding it is correct.

**Deliverables:** Annotated capture, full ten-pose table, one hand-calculated world point, and timing-boundary definitions.

## Lab 3: Observe a Continuous Closed Loop

Use only an idle demo assigned to your group. Record the starting state; RESET before the task if using the configured start. The frozen task registration is for the doorframe, so the refrigerator instruction is an unscored custom task. Record images, actual trajectory, and termination, and enter “not registered” for independent scoring.

```text
Locate the refrigerator in the room and stop beside it.
```

Start once and wait for termination or a clearly defined stopping point. Task limits are 300 seconds and 40 formal predictions. An instructor may preregister a shorter observation period, ending it with user STOP and recording manual termination.

**Observe:** Does the prediction count increase? Does the robot follow valid path A while waiting for prediction B? Does B use new images from a changed pose? Does it stop because of expiry, exhaustion, or a failed path connection? Keep the instruction and start fixed for one record; changed conditions require a new record.

**Expected:** Actual motion produces new observations, rather than replaying one old image. Unsafe paths, loops, or early model STOP remain possible. Record `MODEL_STOP_UNSCORED` as “model stopped; not independently scored,” not decoding failure or proven success. The video's 17 predictions do not guarantee the same result in another run.

**Deliverables:** At least two predictions and their new images if produced. Plot actual poses if motion occurred; otherwise identify the exact termination event. Do not alter the instruction, start, or thresholds to turn a failure into a reported success.

## Lab 4: Stop, Cancellation, and Path Invalidation

Record this separately from navigation scoring as a manual-cancellation check. Do not use fault-injection software on physical hardware.

1. Press STOP while a normal task is waiting for inference.
2. Wait beyond the expected response time and confirm a late result does not restart movement.
3. RESET while idle and check that prediction images and task counters clear.
4. If assigned by the instructor, close the control page during another short task and inspect logs for disconnection or browser-lease expiry. Distinguish background-tab throttling from model failure.

**Expected:** Kinematic stopping, reference clearing, and rejection of results from the cancelled generation. Cancellation does not imply a model STOP or arrival.

**Deliverables:** States before and after cancellation, its reason, and at least one later observation. Mark unavailable branches “not covered” rather than using an idle screenshot as proof.

## Lab 5: Explain a Run from Its Logs

On Jetson, use `python3 scripts/jetson_sim_demo.py status` and read `process.output` for the run directory. Task records are in `navigation-XXXX` subdirectories; actual poses are in their parent's `state.jsonl`.

Analyze in a terminal that can access **both course tools and run logs**, with `MATERIALS` set as above. For local analysis, first obtain the complete run directory, preserving original JPEGs, requests, responses, and poses. `RUN_ROOT` must refer to its location on the analysis machine.

```bash
export RUN_ROOT=/path/to/the/actual/sim-demo-run
export TASK_ID=navigation-0001  # Replace with the actual task directory name.
python3 "$MATERIALS/tools/analyze_run.py" "$RUN_ROOT/$TASK_ID" \
  --state "$RUN_ROOT/state.jsonl" --task-id "$TASK_ID"
```

The tool checks image associations and ten-point paths, summarizes request/response latency, and extracts actual poses by task ID. It reads existing records; it does not independently recertify success. `strict_success_reported` is the original outcome field.

Record:

- Original model path, execution reference, and actual trajectory separately. An unrepaired reference may match the original; a rejected path has no adopted reference.
- Exact termination type: model STOP, user STOP, collision risk, actual contact, timeout, or no progress.
- Inference latency, capture-to-adoption age, motion duty cycle, model Hz, simulation Hz, camera FPS, and browser FPS, each with its measurement definition.
- Version and sample count. A p99 from five samples does not establish long-run reliability.

## Lab 6: Inspect Your Own CAD and Preview It

Obtain authorized URDF and component meshes. Follow the [custom-chassis guide](../code/examples/custom-chassis/README.md) for a read-only inventory. Record part mapping, units, root frame, wheel radius, track width, four casters, and camera. A chassis with a different structure should reflect that structure rather than acquire invented casters.

Learn from the static MJCF preview template, then substitute your real meshes. Primitive preview geometry is only a starting point. The current DM-H65 loader expects 43 visual links; new CAD requires its own backend adaptation in a copy, not removal of the count check.

**Deliverables:** Inventory JSON, asset SHA256, part table, visual/collision comparison, and an independent camera preview. If room integration is incomplete, state “static preview complete; closed loop pending.” No physical motors, CAN, or vehicle calibration is required.

## Troubleshooting

| Symptom | Inspect first | Avoid |
| --- | --- | --- |
| Page opens but navigation cannot start | `model_ready`, threads, frame age, resources, ownership | Repeated model loading or disabling checks |
| Stops immediately after inference | Outcome, raw response, path check, TTL, task generation | Calling every event a GPU or token error |
| Chases old positions or oscillates | Capture pose, result age, `NEW_PATH_CAPTURE_MISMATCH`, raw yaw | Shifting the old path to the current root pose |
| Near target but not successful | Valid model STOP, two-second conditions, registered instruction | Declaring success from distance alone |
| Custom instruction is unscored | `task_context` and registered instruction | Scoring the refrigerator with doorframe coordinates |
| OPOS appears unchanged | Current code, capture, point age, reprojection | Permanently attaching an earlier point to live video |
| Black or slow video | EGL, render thread, FFmpeg, H.264 fallback, both channels | Confusing model Hz with browser FPS |
| Wheel installs but GPU operators fail | cp310/aarch64, sm_87, loaded libraries | Accepting only `cuda.is_available()` |
| Missing CAD or mismatched paths | 45-mesh manifest, geometry JSON, relocation records | Substituting an empty shell and reporting reproduction |

## Finish

Press STOP and confirm the task is no longer active. Stop only the matching demo started by your group:

```bash
cd /home/seeed/workspace/LightNav-0
python3 scripts/jetson_sim_demo.py stop
```

Close each local SSH tunnel with `Ctrl+C`. Leave shared model port 8050 under the instructor's control. Do not use `killall` or stop physical SDK services. Editing this guide and running offline checks do not automatically execute these experiments or constitute a fresh-device installation.

[Back to M7](../README.md)
