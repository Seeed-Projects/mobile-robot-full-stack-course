# PHASE2_REPORT: ROS2 nuScenes Pipeline Validation

Gate: reproduce Phase-1 offline detection quality through the full ROS2
perception stack (bag playback → camera_sync → BEVDet node → detections).

## Result: ✅ PASSED (baseline parity)

| Metric | Phase 1 offline | Phase 2 ROS2 stack |
|---|---|---|
| Frames evaluated | 200 | 37 (of 60 played; first 23 consumed by node init) |
| GT match fraction (score ≥ 0.25) | 42.9 % | **40.6 %** |
| Center error p50 / p95 (m) | 0.37 / — | **0.37 / 1.37** |
| Mean yaw error (deg) | 9.6 | **14.4** |
| Weak classes (pedestrian/cones) | weak (documented) | same (documented BEVDet trait) |

Also verified: node-processed detections for the SAME sample (rank 26) are
position/score identical to the Phase-1 egobox file (labels differ only because
the node publishes autoware ObjectClassification labels, not model labels).

## Bugs found and fixed during this phase

1. **FrameSet CDR corruption** (root cause of "garbage detections"):
   `FrameSet.msg` used fixed-size arrays (`sensor_msgs/Image[6]`) of messages
   containing variable-length fields (`data`). The generated CDR
   serialization/deserialization disagreed on element offsets for inline
   fixed arrays → payload bytes destroyed on the wire. Reproduced with a pure
   rclpy round-trip. Fix: `[<=6]` bounded sequences (serialized per element);
   the "exactly 6" contract is enforced by the nodes. Adds a regression test
   (`tools/frameset_loopback.py`) and a `resize()` in the assembler.
2. **Bag writer camera-index bug**: `"CAM_BACK" in filename` substring match
   also matched `CAM_BACK_LEFT`/`CAM_BACK_RIGHT` → the /camera/back topic
   contained another camera's JPEGs. Fix: match `__CAM_BACK__` delimiters.
   Verified all 360 bag image payloads byte-identical to the source dataset.
3. **QoS serialization in the bag writer**: rosbag2 needs YAML lists of
   complete rmw-qos profiles (history/depth/reliability/durability + infinite
   deadline/lifespan …) captured from a real `ros2 bag record`, otherwise the
   player publishes RELIABLE while the stack subscribes SENSOR_DATA →
   silently disconnected.
4. **Bag timestamps**: absolute 2018-epoch stamps made bag *duration* decades
   long → `ros2 bag play --loop` scheduled a decade-long pause. Fix: normalized
   bag-local stamps t0 = 1.0 s, 2 Hz; original tokens in sidecar JSON.
5. **tf2 "TF_OLD_DATA" flood**: `/tf_static` stamped 0 is treated by tf2 as
   "latest for all time", which made every per-frame `/tf` (ego chain +
   camera statics) get rejected as "old data" and left `base_link` unknown →
   hundreds of startup lookup failures. Fix: stamp /tf_static at 0.5 s
   (just before first frame) and publish `/tf` before the sample's images.
6. **Sync assembler purge clock domain**: purge compared wall time with bag
   stamps → every pending frame purged → incomplete_framesets ≈ 1000s.
   Fix: purge relative to the NEWEST MESSAGE STAMP; detect stamp rollback
   (loop restart) and reset the assembler.
7. **Loop-restart ghosting**: rollback detection (above) resets both the
   assembler and per-scene state; single-pass `--once` mode added to the
   integration script for clean validation.
8. **Node metrics clock domain**: e2e latency compared system clock against
   bag stamps (reported ~48 years). Fix: latency = wall time from frameset
   receipt to publish, fps = frames / elapsed steady time.
9. **26 MB frameset over DDS**: frameset pub/sub switched to RELIABLE QoS
   (depth 5).
10. **OpenCV 4.14 runtime link break**: the robot environment injected a local
    CUDA OpenCV 4.14 whose libs weren't installed; bevdet_node debug dumps
    now use PPM (no OpenCV dependency).

## Environment quirks worth documenting (TROUBLESHOOTING.md)

- The Seeed j501_robot workspace is auto-sourced in this shell
  (`AMENT_PREFIX_PATH` includes it); it can inject conflicting assets
  (OpenCV 4.14 via CMAKE_PREFIX_PATH, /events topics). Always build the
  project's workspace with explicit PATH to CUDA and, for OpenCV-dependent
  packages, the system `/usr/lib/aarch64-linux-gnu/cmake/opencv4` first.
- `ros2 topic echo`/(`hz`) on best-effort topics requires
  `--qos-reliability best_effort`.

## How to re-run

```bash
# clean pass: fresh stack + single bag play + capture + analysis
BEV_DEBUG_DUMP=/tmp/node_dbg SYNC_DEBUG_DUMP=/tmp/sync_dbg \
  scripts/run_clean_pass.sh          # (driver logs to /tmp/clean_pass.log)
# GT evaluation
python3 tools/capture_objects.py /tmp/obs.json 40 &
scripts/run_clean_pass.sh            # writes /tmp/objects_clean.json
# then convert + evaluate (see run_clean_pass.sh / eval_phase1.py)
python3 tools/eval_phase1.py datasets/nuscenes <pred_dir> /tmp/eval.json 0.25
```

## What still limits the score (known, from Phase 1)

- BEVDet-r50-lt-depth weak on pedestrians, motorcycles, cones (nuScenes-mini)
- First 23 frames dropped during node init (engine load ~7 s + rank precompute).
  For offline eval, either start capture after init or replay.

## Files touched this phase

- `ros2_ws/src/bev_camera_sync`: assembler (stamp-relative purge, resize),
  decode safety copy, stamp-aware debug dumps, RELIABLE frameset QoS
- `ros2_ws/src/bev_perception/src/bevdet_node.cpp`: metrics (receipt-based
  e2e, fps via steady time), PPM debug dumps, RELIABLE frameset sub
- `ros2_ws/src/bev_interfaces/msg/FrameSet.msg`: fixed arrays → bounded seq
- `tools/dataset_converter/nuscenes_to_rosbag.py`: delimiter camera match,
  YAML-list QoS, normalized timestamps, tf-first + tf_static@0.5 s
- `tools/capture_objects.py` + `tools/frameset_loopback.py` (new)
- `scripts/run_clean_pass.sh` (new) + `run_phase2_integration.sh` (`--once`)