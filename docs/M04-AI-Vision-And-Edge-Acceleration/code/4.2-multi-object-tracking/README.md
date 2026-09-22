# 4.2 Multi-Object Tracking (ByteTrack)

> All relative paths below are relative to the module root,
> `modules/m04-ai-vision-and-edge-acceleration/`.

Concepts: MOT association (ByteTrack / DeepSORT / Bot-SORT). Implemented here with
**ByteTrack** from the `supervision` library.

**Status: PASS - verified on physical hardware.**

## Pipeline

```
/perception/detections (Detection2DArray)
   -> bev_tracking/tracking_node  (supervision.ByteTrack, persistent instance)
   -> /perception/tracks (Detection2DArray, Detection2D.id == track id)
```

The tracking node never touches a camera and never runs a detector - it is a pure
detection-to-track transformer with no code-level dependency on `bev_detection`.

## Source

`ros2/bev_tracking/` - Python package.

| File | Role |
| --- | --- |
| `bev_tracking/tracking_node.py` | ROS node; builds the tracker once, updates per frame |
| `bev_tracking/adapter.py` | `Detection2DArray` <-> `sv.Detections` conversion |
| `bev_tracking/tracking_visualizer.py` | debug overlay publisher (separate node) |
| `config/bytetrack.yaml` | tracker tuning |
| `test/` | 4 pytest modules, 30 tests |

## The track-id contract (important)

The track id is written into **`Detection2D.id`** as a string:

```python
det.id = str(int(tracker_ids[i]))     # adapter.py
```

It is *not* written into `results[].hypothesis.id`. Verified live:

```
/perception/detections   id: ''                  <- producer leaves it empty
/perception/tracks       id: '96'  class_id: person
```

Empty input still updates the tracker (so `lost_track_buffer` advances) and still
publishes a `Detection2DArray`.

## Config (`config/bytetrack.yaml`)

```yaml
track_activation_threshold: 0.25
lost_track_buffer: 90          # ~3 s of occlusion tolerance at 30 Hz
minimum_matching_threshold: 0.8
frame_rate: 30                 # measured: camera 29.1 Hz -> tracks 30.0 Hz
minimum_consecutive_frames: 1
```

## Demo

```bash
./scripts/m4/run_m4_2_demo.sh
```

## Measured performance

| Metric | Value |
| --- | --- |
| `/perception/tracks` | 24.7-28.3 Hz |
| Per-frame latency | 1.6-3.6 ms |
| Example | `frame=4888 in_dets=1 out_tracks=1 latency_ms=1.60` |

## Tests

30 pytest tests (adapter round-trips, tracker id stability, occlusion, visualizer).
**They only run with plugin autoload disabled:**

```bash
cd ros2/bev_tracking
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q      # -> 30 passed
```

Root cause: two incompatible auto-loaded pytest plugins on this host -
`anyio` in `~/.local` is newer than pytest 6.2.5 (`ModuleNotFoundError:
_pytest.scope`), and ROS's `launch_testing_ros_pytest_entrypoint` declares a hook
pytest 6 does not know. Disabling autoload isolates both while keeping the
user-site `supervision` importable. `pytest.ini` already carries the launch_testing
opt-outs; the autoload flag is the missing piece.

## Known technical debt

- `mock_detection_publisher.py` (a synthetic detection source) ships inside the
  installable package rather than under `test/`.
- `package.xml` has a `schematyps` typo in the schema declaration.
