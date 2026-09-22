#!/usr/bin/env bash
# M4.1 Smoke Test — YOLO11n TensorRT detector ROS 2 end-to-end.
#
# Spins up yolo_trt_node + image_republisher, counts Detection2DArray
# messages published on /perception/detections.
#
# Usage: scripts/m4/test_yolo_node.sh
# Exit: 0 = smoke OK (≥ 1 detection frame), 1 = smoke FAIL
set -uo pipefail

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
ENGINE="$M4_ROOT/models/m4/detection/engines/yolo11n_fp16.engine"
TEST_IMAGE="$WS_ROOT/shared/datasets/nuscenes/samples/CAM_FRONT/$(ls $WS_ROOT/shared/datasets/nuscenes/samples/CAM_FRONT/ 2>/dev/null | head -1)"
OUT="$M4_ROOT/output/m4/4.1"
TIMEOUT="${TIMEOUT:-15}"  # seconds for capture

mkdir -p "$OUT"

if [ ! -f "$ENGINE" ]; then
  echo "ERROR: engine not found: $ENGINE" >&2
  exit 1
fi
if [ ! -f "$TEST_IMAGE" ]; then
  echo "ERROR: test image not found" >&2
  exit 1
fi

# Source ROS env (avoid set -u issues from /opt/ros/* setup.bash)
set +u
source /opt/ros/humble/setup.bash
cd "$WS_ROOT"
source install/setup.bash
set -u

# Required libs (opencv-cuda, CUDA, TensorRT)
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$WS_ROOT/install/bev_detection/lib:/home/seeed/src/opencv-4.14.0-cuda-build/lib:/usr/local/cuda-12.6/lib64:/usr/lib/aarch64-linux-gnu"

COUNT_FILE="$OUT/smoke_count.txt"
rm -f "$COUNT_FILE"

echo "== M4.1 smoke test =="
echo "Engine: $ENGINE"
echo "Image : $TEST_IMAGE"

# 1. YOLO node
ros2 run bev_detection yolo_trt_node \
  --ros-args \
  -p model_path:="$ENGINE" \
  -p publish_debug_image:=false \
  > "$OUT/smoke_yolo.log" 2>&1 &
YOLO_PID=$!
sleep 5

# 2. Publisher
python3 "$M4_ROOT/4.1-yolo-object-detection/ros2/bev_detection/test/image_republisher.py" "$TEST_IMAGE" \
  > "$OUT/smoke_pub.log" 2>&1 &
PUB_PID=$!
sleep 2

# 3. Counter (Python subscription)
python3 "$M4_ROOT/4.1-yolo-object-detection/ros2/bev_detection/test/count_detections.py" "$COUNT_FILE" "$TIMEOUT" \
  > "$OUT/smoke_count_stdout.txt" 2> "$OUT/smoke_count_err.txt" &
COUNT_PID=$!

# Wait for counter to finish
wait "$COUNT_PID" 2>/dev/null || true

# Cleanup
kill $YOLO_PID $PUB_PID 2>/dev/null
sleep 1

# Read count from stdout
COUNT=$(cat "$OUT/smoke_count_stdout.txt" 2>/dev/null | tr -d '[:space:]')
COUNT=${COUNT:-0}

if [ "$COUNT" -ge 1 ] 2>/dev/null; then
  echo "  [PASS] received $COUNT Detection2DArray messages"
  exit 0
else
  echo "  [FAIL] no Detection2DArray frames in ${TIMEOUT}s"
  echo "         (see $OUT/smoke_yolo.log)"
  exit 1
fi
