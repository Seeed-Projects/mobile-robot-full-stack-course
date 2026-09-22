#!/usr/bin/env bash
# M4.1 Empty-Frame Contract Test.
#
# Verifies the M4.1 → M4.2 contract:
#   - yolo_trt_node publishes ONE Detection2DArray per processed frame,
#     even when the input image produces zero detections.
#   - For empty frames, detections.size() == 0 but the message is published.
#
# Strategy: generate a near-blank 1920x1080 image (uniform grey, no
# features). YOLO11n's confidence_threshold=0.25 will filter all
# detections. The publisher keeps sending this image for TIMEOUT seconds.
# The subscriber records how many Detection2DArray messages it received
# and how many had detections.size() == 0. Pass = at least one empty
# message received.
#
# Usage: scripts/m4/test_empty_frame_contract.sh
# Exit: 0 = PASS, 1 = FAIL
set -uo pipefail

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
ENGINE="$M4_ROOT/models/m4/detection/engines/yolo11n_fp16.engine"
OUT="$M4_ROOT/output/m4/4.1"
TIMEOUT="${TIMEOUT:-12}"   # seconds for capture
MIN_EMPTY_MSGS="${MIN_EMPTY_MSGS:-1}"

mkdir -p "$OUT"

if [ ! -f "$ENGINE" ]; then
  echo "ERROR: engine not found: $ENGINE" >&2
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

COUNT_FILE="$OUT/empty_frame_count.txt"
rm -f "$COUNT_FILE"

echo "== M4.1 empty-frame contract test =="
echo "Engine: $ENGINE"
echo "Timeout: ${TIMEOUT}s"
echo "Strategy: feed uniform-grey 1920x1080 image; expect 0 detections but"
echo "          exactly one Detection2DArray per processed frame."

# 1. YOLO node
ros2 run bev_detection yolo_trt_node \
  --ros-args \
  -p model_path:="$ENGINE" \
  -p publish_debug_image:=false \
  -p class_names_path:="" \
  > "$OUT/empty_yolo.log" 2>&1 &
YOLO_PID=$!
sleep 5

# 2. Blank-image publisher (Python)
python3 "$M4_ROOT/4.1-yolo-object-detection/ros2/bev_detection/test/blank_image_publisher.py" \
  > "$OUT/empty_pub.log" 2>&1 &
PUB_PID=$!
sleep 2

# 3. Empty-frame checker
python3 "$M4_ROOT/4.1-yolo-object-detection/ros2/bev_detection/test/empty_frame_checker.py" "$COUNT_FILE" "$TIMEOUT" \
  > "$OUT/empty_check_stdout.txt" 2> "$OUT/empty_check_err.txt"
CHECK_RC=$?

# Cleanup
kill $YOLO_PID $PUB_PID 2>/dev/null
sleep 1

# Parse result
TOTAL=$(grep '^__total__' "$COUNT_FILE" 2>/dev/null | awk '{print $2}')
EMPTY=$(grep '^__empty__' "$COUNT_FILE" 2>/dev/null | awk '{print $2}')
TOTAL=${TOTAL:-0}
EMPTY=${EMPTY:-0}

echo ""
echo "Total Detection2DArray received: $TOTAL"
echo "Empty-frame messages: $EMPTY"

if [ "$CHECK_RC" -ne 0 ] || [ "$EMPTY" -lt "$MIN_EMPTY_MSGS" ]; then
  echo "  [FAIL] empty-frame contract violated:"
  echo "         expected >= $MIN_EMPTY_MSGS empty Detection2DArray messages,"
  echo "         got $EMPTY (out of $TOTAL total)."
  echo "         (see $OUT/empty_yolo.log)"
  exit 1
fi

echo "  [PASS] empty-frame contract holds: $EMPTY of $TOTAL frames were empty,"
echo "         proving yolo_trt_node still publishes Detection2DArray on"
echo "         frames with zero detections."
exit 0
