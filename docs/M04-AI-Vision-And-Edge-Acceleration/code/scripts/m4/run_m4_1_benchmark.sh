#!/usr/bin/env bash
# M4.1 Benchmark: YOLO11n TensorRT FPS / latency on real camera input.
#
# Usage: ./run_m4_1_benchmark.sh [duration_seconds]
set -uo pipefail

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
ENGINE="$M4_ROOT/models/m4/detection/engines/yolo11n_fp16.engine"
TEST_IMAGE="$WS_ROOT/shared/datasets/nuscenes/samples/CAM_FRONT/$(ls $WS_ROOT/shared/datasets/nuscenes/samples/CAM_FRONT/ 2>/dev/null | head -1)"
OUT="$M4_ROOT/output/m4/4.1"
DURATION="${1:-30}"

# Compute valid pub window (hz --window must be > total collect time)
DURATION_INT=$((DURATION + 0))
if [ $DURATION_INT -lt 12 ]; then
  DURATION_INT=12
fi
HZ_WIN=$((DURATION_INT - 7))

mkdir -p "$OUT"
echo "== M4.1 benchmark =="
echo "Engine: $ENGINE"
echo "Image: $TEST_IMAGE"
echo "Duration: ${DURATION_INT}s"

if [ ! -f "$ENGINE" ]; then
  echo "ERROR: engine file not found at $ENGINE" >&2
  exit 1
fi
if [ -z "$TEST_IMAGE" ] || [ ! -f "$TEST_IMAGE" ]; then
  echo "ERROR: no test image in $WS_ROOT/shared/datasets/nuscenes/samples/CAM_FRONT/" >&2
  exit 1
fi

# Source ROS environment (avoid set -u issues)
set +u
source /opt/ros/humble/setup.bash
cd "$WS_ROOT"
source install/setup.bash
set -u

# Extend LD_LIBRARY_PATH for opencv + CUDA + TensorRT
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$WS_ROOT/install/bev_detection/lib:${OPENCV_CUDA_LIB:-}:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu"

# Launch YOLO node
ros2 run bev_detection yolo_trt_node \
  --ros-args \
  -p model_path:="$ENGINE" \
  -p publish_debug_image:=false > "$OUT/yolo.log" 2>&1 &
YOLO_PID=$!
echo "yolo_pid=$YOLO_PID"

# Wait for engine load + warmup
sleep 6

# Launch image publisher at ~30 FPS
python3 "$M4_ROOT/4.1-yolo-object-detection/ros2/bev_detection/test/image_republisher.py" "$TEST_IMAGE" \
  > "$OUT/pub.log" 2>&1 &
PUB_PID=$!
echo "pub_pid=$PUB_PID"
sleep 2

# Run inference rate measurement
echo "Measuring inference rate for ${DURATION_INT}s (window=${HZ_WIN}s)..."
set +u
timeout "${DURATION_INT}" ros2 topic hz /perception/detections --window "${HZ_WIN}" \
  > "$OUT/hz.txt" 2>&1 || true
set -u

# Cleanup
kill $YOLO_PID $PUB_PID 2>/dev/null
sleep 2

# Extract stats
echo ""
echo "== YOLO stats =="
grep "YOLO:" "$OUT/yolo.log" | tail -10 || true

ENGINE_SIZE=$(stat -c%s "$ENGINE" 2>/dev/null || stat -f%z "$ENGINE")
AVERAGE_RATE=$(grep "average rate" "$OUT/hz.txt" 2>/dev/null | tail -1 | awk '{print $3}')
AVERAGE_RATE=${AVERAGE_RATE:-"0.0"}

# Compute mean inference / e2e latencies (force text mode for binary logs)
MEAN_INF=$(grep -ao "YOLO: fps=[0-9.]*, last_inf=[0-9.]*ms, last_e2e=[0-9.]*ms" "$OUT/yolo.log" 2>/dev/null \
  | tail -20 \
  | sed -nE 's/.*last_inf=([0-9.]+)ms.*/\1/p' \
  | awk '{s+=$1; n++} END {if(n>0) printf "%.2f", s/n; else print "0.00"}')
MEAN_E2E=$(grep -ao "YOLO: fps=[0-9.]*, last_inf=[0-9.]*ms, last_e2e=[0-9.]*ms" "$OUT/yolo.log" 2>/dev/null \
  | tail -20 \
  | sed -nE 's/.*last_e2e=([0-9.]+)ms.*/\1/p' \
  | awk '{s+=$1; n++} END {if(n>0) printf "%.2f", s/n; else print "0.00"}')

cat > "$OUT/benchmark.json" <<EOF
{
  "model": "yolo11n",
  "engine": "$ENGINE",
  "duration_seconds": ${DURATION_INT},
  "device": "Jetson AGX Orin",
  "jetpack": "6.2.1",
  "tensorrt": "10.3.0",
  "precision": "fp16",
  "input_shape": "1x3x640x640",
  "output_shape": "1x84x8400",
  "engine_size_bytes": ${ENGINE_SIZE},
  "test_image": "$TEST_IMAGE",
  "inference_rate_hz": ${AVERAGE_RATE},
  "mean_inference_ms": ${MEAN_INF},
  "mean_e2e_ms": ${MEAN_E2E},
  "timestamp": "$(date -Iseconds)"
}
EOF

cat "$OUT/benchmark.json"
echo ""
echo "Saved: $OUT/benchmark.json"
