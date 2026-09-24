# M4.2: Object Tracking (ByteTrack)

**Owner:** bev_tracking Agent
**Branch:** `M4-2-tracking`
**Upstream:** bev_detection M4.1 (YOLO + TensorRT detector) – already merged
**Downstream consumers (future):** bev_perception (BEV fusion), FoundationPose, depth alignment

## Scope

bev_tracking consumes the `vision_msgs/Detection2DArray` published by bev_detection and
publishes a tracked `vision_msgs/Detection2DArray` (one `Detection2D` per active track,
with stable `id` field). Tracking is delegated to **supervision.ByteTrack**, a faithful
port of ByteTrack (Zhang et al., ECCV 2022). Kalman filter + Hungarian matching for
IoU association.

## What is in this package

| File                                          | Purpose                                      |
|-----------------------------------------------|----------------------------------------------|
| `bev_tracking/adapter.py`                     | Pure-Python ROS <-> supervision.Detections   |
| `bev_tracking/tracking_node.py`               | rclpy node, owns the ByteTrack instance      |
| `bev_tracking/mock_detection_publisher.py`    | Synthetic detector for dev / smoke tests     |
| `bev_tracking/tracking_visualizer.py`         | 2D overlay (bbox + class + score + track_id) |
| `launch/tracking.launch.py`                   | Standalone launch (mock detector)            |
| `launch/tracking_with_detection.launch.py`    | End-to-end launch (wraps bev_detection)      |
| `launch/tracking_demo.launch.py`              | Real-time demo launch (camera + viewer)      |
| `config/bytetrack.yaml`                       | Default ByteTrack hyperparameters            |
| `test/test_adapter.py`                        | Unit tests for adapter.py                    |
| `test/test_tracker.py`                        | Unit tests for ByteTrack behaviour           |
| `test/test_integration.py`                    | rclpy end-to-end tests for tracking_node     |
| `test/test_visualizer.py`                     | Unit tests for tracking_visualizer           |

## What is NOT in this package

- No model weights, calibration files, or GPU code
- No camera drivers, image preprocessing, or BEV transform
- No re-implementation of Kalman / Hungarian (ByteTrack handles it)
- No modification of bev_detection or any upstream package

## Build & test

```sh
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select bev_tracking
colcon test --packages-select bev_tracking --ctest-args -V
```

## Run

```sh
# Standalone (mock detector):
ros2 launch bev_tracking tracking.launch.py

# End-to-end (with bev_detection + camera):
ros2 launch bev_tracking tracking_with_detection.launch.py

# One-click real-time camera demo (camera + YOLO + tracker + overlay + viewer):
./scripts/m4/run_m4_2_demo.sh
./scripts/m4/run_m4_2_demo.sh --no-gui   # headless

# Subscribe to tracks and inspect IDs:
ros2 topic echo /perception/tracks vision_msgs/msg/Detection2DArray --once

# Subscribe to debug overlay (visualizer output):
ros2 topic echo /perception/tracking_debug_image sensor_msgs/msg/Image --once
```

## Topics

| Topic                          | Direction | Type                              | QoS              |
|--------------------------------|-----------|-----------------------------------|------------------|
| `/perception/detections`       | in        | `vision_msgs/Detection2DArray`    | BEST_EFFORT, d=10 |
| `/perception/tracks`           | out       | `vision_msgs/Detection2DArray`    | BEST_EFFORT, d=10 |
| `/perception/cameras/front/image` | in (visualizer) | `sensor_msgs/Image`        | BEST_EFFORT, d=10 |
| `/perception/tracking_debug_image` | out (visualizer) | `sensor_msgs/Image` (bgr8) | BEST_EFFORT, d=10 |

`BEST_EFFORT` matches bev_detection M4.1 and the image pipeline upstream.
Downstream consumers that need RELIABLE (e.g. services) must bridge.

## Demo layer

The student-facing demo is a thin visualization layer that draws the
existing M4.2 track output on top of the camera image:

```
/perception/cameras/front/image  ─┐
                                  ├─►  tracking_visualizer  ─►  /perception/tracking_debug_image
/perception/tracks              ─┘
```

`tracking_visualizer.py` is a **separate node** from `tracking_node.py`:
the tracker core stays camera- and image-independent. The visualizer:

- Subscribes (BEST_EFFORT) to the camera image and the tracks.
- Caches the last ~10 `Detection2DArray` messages keyed by their
  header stamp and matches each incoming image to the closest-in-time
  track within a 200 ms window.
- Draws a green rectangle and a label `class #track_id score` per
  detection (track_id is visually emphasised).
- Always publishes the resulting image (with a "no tracks (waiting)"
  banner when no recent track is available) so the viewer keeps
  streaming.

The full pipeline is launched with `scripts/m4/run_m4_2_demo.sh`,
which:

- Resolves paths, sources ROS + the workspace overlay.
- Pre-flights: verifies packages, engine, labels, executables.
- Starts `csi_camera_publisher.py` (auto-detects CSI / USB /
  videotestsrc) and `tracking_demo.launch.py`.
- Waits up to 30 s for the four required topics to be advertised.
- Optionally spawns `rqt_image_view` if `DISPLAY` is set.
- On Ctrl-C: SIGINT → 5 s grace → SIGKILL → `pkill -9 -P $$` →
  verifies `/dev/video*` is released. The script NEVER uses
  `pkill -9 ros`, `killall python`, or `killall ros2` — only the
  PIDs it explicitly spawned.
