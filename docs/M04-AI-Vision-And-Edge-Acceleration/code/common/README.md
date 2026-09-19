# M4 common infrastructure

Everything in this module that is **not** algorithm code.

| Path | What it is |
| --- | --- |
| `ros2/m4_demo_bringup/` | demo orchestrator: launch files, visualizers, web preview server |
| `ros2/bev_interfaces/` | `FrameSet` / `SyncStats` / `SystemStatus` messages (hard build dep of `bev_detection`) |
| `../scripts/m4/` | demo runners, shared process-lifecycle library, model scripts |
| `../scripts/regression/` | regression gates (`run_m4_*_regression.sh`) |
| `../models/m4/` | model artifacts + metadata (binaries are gitignored) |

## m4_demo_bringup

Executables: `detection_visualizer`, `segmentation_visualizer`, `pose_visualizer`,
`wait_for_message_node`, `m4_web_demo_server`.

It is deliberately limited to orchestration, visualization, readiness and process
ownership - no algorithm implementation.

### Web preview

A single aiohttp server on port **8080** served by `m4_web_demo_server`:

| Route | Purpose |
| --- | --- |
| `/healthz` | real readiness: `server_ready`, `ros_frame_ready`, `peer_ready`, per-topic state |
| `/stream` | MJPEG (always available fallback) |
| `/signaling` | WebSocket WebRTC signalling |
| `/api/demos` | module list + readiness + transport |
| `/m4/{1,2,3,4,hub}` | per-chapter pages |

Transport is chosen automatically `h264 -> mjpeg -> vp8`; hardware H.264
(`nvv4l2h264enc`) is the normal result, with `aiortc` VP8 as the software path.
Frames are held in a single-slot buffer that drops stale frames - there are no
unbounded queues.

### Process lifecycle (`scripts/m4/lib/m4_demo_lib.sh`)

The bash supervisor stays alive (no `exec`), spawns every child via `setsid` into
its own process group, records PIDs **and** `/proc/<pid>/stat` start-ticks in
`/tmp/m4_demo/*.json`, and traps `INT`/`TERM`/`EXIT`. Cleanup is SIGINT-first with a
5 s grace, then SIGKILL by PGID - and every signal is gated on the start-tick
matching, so a recycled PID is never signalled. There is no blanket `pkill` in this
path. `cleanup_demo_residual.sh` is the recovery tool for crashed runs and follows
the same contract.

`$REPO` is derived from the library's own location (`scripts/m4/lib/../../..`), so
the demo scripts work unchanged from this module.

## Known technical debt

- `csi_camera_publisher.py` lives in `4.1-.../ros2/bev_detection/test/` but is the
  de-facto production camera source for 4.1-4.3. It is referenced by ~10 callers;
  moving it is deferred rather than risk the only physically verified camera path.
- The GMSL `use_camera_sync:=true` branch of `bev_detection/launch/m4_detection.launch.py`
  needs the pre-M4 `bev_camera_sync` package, which is **not** part of this module.
  The V4L2 path (what the demos actually use) does not need it.
- `visualize_yolo_live.sh` uses `pkill -9 -P $$` (kills all descendants of its own
  shell). Standalone manual tool only, not on the demo path.
