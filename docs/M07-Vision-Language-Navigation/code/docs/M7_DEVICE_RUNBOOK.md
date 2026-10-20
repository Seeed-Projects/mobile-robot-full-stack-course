# M7 device runbook (Jetson AGX Orin)

Engineering notes for running the M7 simulation demo on the reference device
(`seeed@<jetson>`, JetPack 6.2.1, ROS 2 Humble, aarch64, GPU sm_87). The Chinese
lab procedure for students is `docs/M07-Vision-Language-Navigation/7.3_From_Navigation_Prediction_to_Closed_Loop_Control/LAB_GUIDE.md`
in the course repository; this page is the operator view.

## 1. What runs where

| Part | Responsibility | Address / environment |
| --- | --- | --- |
| LightNav model service | vision history + instruction inference, tokens and paths | `127.0.0.1:8050`, `.venv-lightnav-py310` (py3.10) |
| MuJoCo + chassis model | body/joint state, scene geometry, camera rendering | `mujoco_demo/.venv-cpu` (py3.11) |
| Demo backend | tasks, model requests, path checks, tracking speeds | `127.0.0.1:8091` |
| SDK WebUI (hardware, unrelated to the demo) | DM-H65 hardware SDK web UI | `0.0.0.0:8765` |
| Browser | frames, predictions, trajectories, STOP/RESET | forwarded `18091` (page) and `18092` |

Two environments, one WebSocket: Python versions differ on purpose. Do not
"unify" them.

## 2. Status before touching anything

```bash
cd /home/seeed/workspace/LightNav-0
bash scripts/jetson_lightnav.sh status        # model service identity + ready flag
python3 scripts/jetson_sim_demo.py status     # demo process + /api/health
ss -ltnp | grep -E ':(8050|8091|8765)'
df -h .; free -h
```

`status` is read-only and is always the first command. The demo's health payload
should show `ok=true`, `model_ready=true`, `simulation.physics_alive` and
`render_alive` true, and `frame_age_s<1`; `physics_alive` is a legacy thread name
that now updates the kinematic state.

## 3. Prepare a fresh clone (never the running project)

```bash
cd <workspace>/mobile_robot_ws
docs/M07-Vision-Language-Navigation/code/scripts/m7/m7_materials_check.sh
docs/M07-Vision-Language-Navigation/code/scripts/m7/m7_prepare_lightnav.sh            # dry-run
docs/M07-Vision-Language-Navigation/code/scripts/m7/m7_prepare_lightnav.sh --apply \
  --upstream /home/seeed/workspace/LightNav-0 \
  --bundle /path/to/dm-h65-course-assets-20261015.tar.gz
```

The prepared clone lands in `shared/lightnav/LightNav-0` (gitignored). The script
pins upstream commit `3015508b…`, applies the 34-file overlay, relocates the CAD
assets into `course-assets/` and finishes with `preflight.py`. `preflight` reporting
missing `.venv-lightnav-py310`, `mujoco_demo/.venv-cpu` or a checkpoint on a fresh
clone is expected: those are the external, uncommitted parts.

## 4. Start the demo

```bash
docs/M07-Vision-Language-Navigation/code/scripts/m7/m7_sim_demo.sh status
# model service must be up first (device owner decides):
cd /home/seeed/workspace/LightNav-0 && bash scripts/jetson_lightnav.sh start
docs/M07-Vision-Language-Navigation/code/scripts/m7/m7_sim_demo.sh start
curl -fsS http://127.0.0.1:8091/api/health
```

`m7_sim_demo.sh start` binds `127.0.0.1:8091` and passes the prepared
`course-assets/dm-h65/config.json` + `course-assets/task.json` when both exist.
When a project keeps its original deployment directories instead, omit those two
options (`python3 scripts/jetson_sim_demo.py start --host 127.0.0.1 --port 8091`) —
do not mix the two layouts. `stop` terminates only the process recorded in the
project's own `.sim-demo-process.json`; there is no blind kill.

## 5. Browser access from a workstation

```bash
export JETSON_HOST=seeed@YOUR_JETSON_IP
ssh -N -o ControlMaster=no -o ControlPath=none -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18091:127.0.0.1:8091 "$JETSON_HOST"
ssh -N -o ControlMaster=no -o ControlPath=none -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18092:127.0.0.1:8092 "$JETSON_HOST"
```

Open `http://127.0.0.1:18091`. The page splits frames and control across the two
connections, so forward both.

## 6. Troubleshooting order

| Symptom | Check first |
| --- | --- |
| No frames, health `render_alive=false` | EGL/GPU access in the simulation environment; `render_alive` is the render thread, not the browser |
| `model_ready=false` or predictions stall | model service on 8050 (`jetson_lightnav.sh status`), checkpoint completeness (weights + tokenizer + processor + eval config + RVQ) |
| Sluggish page | separate inference latency, simulation stalls and the encode/transport path before blaming the model |
| GPU work fails to load | three layers: dependency load, real op execution (BF16 matmul, cuDNN conv, SDPA), model profiling — see 7.3 |
| Silent stop / odd tokens | strict decode consistency (`overlay/src/lightnav/serving/strict_decode.py`) and the archived evidence in `evidence/` |
| Prepared clone rejects the overlay | the clone has edits that match neither upstream nor the course copy; use a fresh clone |

## 7. Boundaries

- Simulation only. Simplified differential kinematics, geometric lookahead and a
  fast collision check — not MPC/Nav2, and not evidence of real tyre, caster or
  braking behaviour.
- Real model paths, limited local repair and actual motion are recorded separately;
  model STOP is not a scored success.
- No models, wheels or CAD bundles in git; no service restarts without the device
  owner.
