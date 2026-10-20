# 7.3 Running the LightNav Navigation Demo on Jetson

[English](README_en_US.md) | [中文](README_zh_CN.md)

The earlier chapters explained how LightNav generates paths from images and an instruction. Here you will run the chassis simulation on an already prepared AGX Orin, ask it to find a refrigerator, and observe predictions, movement, and new camera views updating in a loop.

![LightNav chassis navigation demo](assets/demo.gif)

*Figure 7.3-1. The chassis observes the room, predicts paths, and moves in response to a language instruction.*

This chapter assumes the project, model weights, two Python environments, scene, and chassis assets are ready. For first-time installation, complete [Lab Guide, Route B](LAB_GUIDE.md#route-b-prepare-a-new-device). For your own CAD, see the [custom-chassis materials](../code/examples/custom-chassis/README.md). The commands below use the existing project's default assets.

## 7.3.1 Check and Start Services on Jetson

Connect by SSH and run in the **Jetson terminal**:

```bash
cd /home/seeed/workspace/LightNav-0
bash scripts/jetson_lightnav.sh status
python3 scripts/jetson_sim_demo.py status
```

The model service on port 8050 performs GPU inference. The demo on port 8091 serves the simulation and web interface. Reuse a ready model. Before using an existing demo, confirm nobody else is controlling it.

**Start only services that are not running.** If the model is stopped:

```bash
bash scripts/jetson_lightnav.sh start
bash scripts/jetson_lightnav.sh status
```

Wait for model readiness. If the demo is stopped, start it and check health:

```bash
python3 scripts/jetson_sim_demo.py start --host 127.0.0.1 --port 8091
curl -fsS http://127.0.0.1:8091/api/health
```

Expect `ok=true`, `model_ready=true`, simulation fields `physics_alive=true` and `render_alive=true`, and `frame_age_s` below 1. Check again after a few seconds: simulation time and frame counts should advance. A returned process ID alone does not establish readiness.

These commands use the original deployment's asset paths. If you prepared the relocated course assets, supply the configuration arguments in [Lab Guide, Route A](LAB_GUIDE.md#route-a-use-an-already-deployed-device). If a file is missing, check its asset path before reinstalling anything.

## 7.3.2 Open the Web Demo from Your Computer

Run the following on **your own computer**. Replace `YOUR_JETSON_IP` and, if necessary, the SSH username. Use two terminals for two independent connections.

Terminal 1:

```bash
export JETSON_HOST=seeed@YOUR_JETSON_IP
ssh -N -o ControlMaster=no -o ControlPath=none \
  -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18091:127.0.0.1:8091 "$JETSON_HOST"
```

Terminal 2:

```bash
export JETSON_HOST=seeed@YOUR_JETSON_IP
ssh -N -o ControlMaster=no -o ControlPath=none \
  -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18092:127.0.0.1:8091 "$JETSON_HOST"
```

Keep both terminals open and visit **http://127.0.0.1:18091/**.

At this address, the page uses 18091 for video and 18092 for control, so both tunnels are needed. A tunnel waiting without further output is normal. If a port is occupied, check existing local tunnels. The Jetson backend connects to model port 8050; the browser needs no separate model tunnel.

## 7.3.3 Start Navigation with One Instruction

Confirm the first- and third-person views are updating and the model is ready. Enter:

> Locate the refrigerator in the room and stop beside it.

You can also try your own instruction. Click **Start navigation** once. The system keeps collecting images, requesting predictions, and tracking valid paths without another click for each prediction.

Watch three areas:

- **Live views:** what the robot sees now and whether the chassis is moving.
- **Latest prediction:** the captured view used for that decision, its target pointing, and its path.
- **Top-down trajectory:** the reference path and the robot's actual movement.

While a new prediction is being computed, the robot can continue tracking a still-valid path. An exhausted path, stale result, or failed check causes waiting or stopping, with a reason shown on the page. Read that reason before starting again.

**STOP** immediately cancels the task. **RESET** returns the stopped simulation to the configured start. Before changing the instruction or trying again, press STOP and confirm the robot has stopped. Record model stopping, user cancellation, and path-check stopping separately.

## 7.3.4 Finish the Demo

Press STOP on the page. To close your group's demo, run on Jetson:

```bash
cd /home/seeed/workspace/LightNav-0
python3 scripts/jetson_sim_demo.py stop
```

Press `Ctrl+C` in each local SSH terminal to close the tunnels. Leave the shared model service according to the instructor's arrangement, and use the launcher to manage your own demo process.

The startup output or `status` output field identifies the log directory. Save the instruction, termination reason, and observations. See the [Lab Guide](LAB_GUIDE.md) for detailed analysis.

## 7.3.5 Quick Troubleshooting

| Symptom | First check |
| --- | --- |
| Page does not open | Demo status on Jetson and the local 18091 tunnel |
| Video works but buttons do not | The 18092 control tunnel and competing control clients |
| Model not ready | Model status on 8050; once ready, ask the owner to restart only the idle demo |
| Robot waits or stops | The displayed reason: model STOP, reference expiry, or collision checking |

Check service readiness, browser connectivity, model responses, and actual movement in that order. First-time deployment, asset preparation, and further diagnostics are in the [complete Lab Guide](LAB_GUIDE.md).

[7.2 Understanding LightNav-0](../7.2_Semantic_Grounding_and_Learned_VLN/README_en_US.md) · [Back to M7](../README.md)
