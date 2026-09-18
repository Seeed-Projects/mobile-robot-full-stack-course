#!/usr/bin/env bash
# M4.1 Benchmark: YOLO11n TensorRT FPS / latency on real camera input.
#
# Usage: ./run_m4_1_benchmark.sh [duration_seconds]
set -uo pipefail

REPO="${REPO:-/home/seeed/mobile-robot-full-stack-course/modules/m04-ai-vision-and-edge-acceleration}"
ENGINE="$REPO/models/m4/detection/engines/yolo11n_fp16.engine"
TEST_IMAGE="$REPO/datasets/nuscenes/samples/CAM_FRONT/$(ls $REPO/datasets/nuscenes/samples/CAM_FRONT/ 2>/dev/null | head -1)"
OUT="$REPO/output/m4/4.1"
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
  echo "ERROR: no test image in $REPO/datasets/nuscenes/samples/CAM_FRONT/" >&2
  exit 1
fi

# Source ROS environment (avoid set -u issues)
set +u
source /opt/ros/humble/setup.bash
cd "$REPO/ros2_ws"
source install/setup.bash
set -u

# Extend LD_LIBRARY_PATH for opencv + CUDA + TensorRT
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$REPO/ros2_ws/install/bev_detection/lib:/home/seeed/src/opencv-4.14.0-cuda-build/lib:/usr/local/cuda-12.6/lib64:/usr/lib/aarch64-linux-gnu"

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
python3 src/bev_detection/test/image_republisher.py "$TEST_IMAGE" \
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
