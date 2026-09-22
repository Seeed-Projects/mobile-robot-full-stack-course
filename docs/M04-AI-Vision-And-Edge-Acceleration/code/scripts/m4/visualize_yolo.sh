#!/usr/bin/env bash
# M4.1 一键可视化脚本。
#
# 一个进程里同时跑：
#   1. yolo_trt_node（订阅图像，发布 Detection2DArray + debug 画框图）
#   2. image_loop_publisher（循环发布 nuScenes CAM_FRONT 样本）
#   3. screenshot_saver（订阅 debug 图，存一帧 PNG 就退）
#
# 跑完后 output/m4/4.1/snapshot.png 就是带 bbox 的可视化结果。
#
# 用法：
#   bash /home/seeed/workspace/ros2_bev/modules/m04-ai-vision-and-edge-acceleration/scripts/m4/visualize_yolo.sh
#
# 自定义：
#   SAMPLE_DIR=/path/to/images bash scripts/m4/visualize_yolo.sh
#   NUM_SAMPLES=10 bash scripts/m4/visualize_yolo.sh
#   SKIP_SCREENSHOT=1 bash scripts/m4/visualize_yolo.sh   # 只跑节点，不截图
set -uo pipefail

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
ENGINE="$M4_ROOT/models/m4/detection/engines/yolo11n_fp16.engine"
LABELS="$M4_ROOT/models/m4/detection/labels/coco.names"
SAMPLE_DIR="${SAMPLE_DIR:-$WS_ROOT/shared/datasets/nuscenes/samples/CAM_FRONT}"
OUT="$M4_ROOT/output/m4/4.1"
SNAPSHOT="$OUT/snapshot.png"
NUM_SAMPLES="${NUM_SAMPLES:-3}"
TIMEOUT_PUBLISH="${TIMEOUT_PUBLISH:-25}"   # seconds of "loop" mode runtime
SKIP_SCREENSHOT="${SKIP_SCREENSHOT:-0}"
PUB_MODE="${PUB_MODE:-loop}"                # loop | finite
LOG_DIR="$OUT/visualize"

mkdir -p "$OUT" "$LOG_DIR"

# Sanity
if [ ! -f "$ENGINE" ]; then
  echo "[ERROR] engine not found: $ENGINE" >&2
  exit 1
fi
if [ ! -d "$SAMPLE_DIR" ]; then
  echo "[ERROR] sample dir not found: $SAMPLE_DIR" >&2
  exit 1
fi
N_FILES=$(ls "$SAMPLE_DIR"/*.jpg 2>/dev/null | wc -l)
if [ "$N_FILES" -eq 0 ]; then
  echo "[ERROR] no .jpg files in $SAMPLE_DIR" >&2
  exit 1
fi

# Clean logs
rm -f "$LOG_DIR"/*.log "$LOG_DIR"/*.txt

# ---- 颜色（终端可读） ----
GREEN=$'\e[0;32m'
YELLOW=$'\e[1;33m'
RED=$'\e[0;31m'
NC=$'\e[0m'
section() { echo "${YELLOW}== $* ==${NC}"; }
ok()      { echo "${GREEN}  [OK] $*${NC}"; }
fail()    { echo "${RED}  [FAIL] $*${NC}"; }

# ---- 0. 环境 ----
section "Step 0/4 — ROS environment"
set +u
source /opt/ros/humble/setup.bash
cd "$WS_ROOT"
source install/setup.bash
set -u
ok "ros2 + bev_detection sourced"

# ---- LD_LIBRARY_PATH（CUDA + opencv 4.14 + TRT） ----
export LD_LIBRARY_PATH="$WS_ROOT/install/bev_detection/lib:/home/seeed/src/opencv-4.14.0-cuda-build/lib:/usr/local/cuda-12.6/lib64:/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
ok "LD_LIBRARY_PATH configured"

# ---- 1. 启动 yolo_trt_node（后台） ----
section "Step 1/4 — yolo_trt_node"
ros2 launch bev_detection yolo.launch.py \
  model_path:="$ENGINE" \
  class_names_path:="$LABELS" \
  publish_debug_image:=true \
  > "$LOG_DIR/yolo.log" 2>&1 &
YOLO_PID=$!
echo "  yolo_trt_node PID=$YOLO_PID  (logs: $LOG_DIR/yolo.log)"

# ---- 2. 启动 image loop publisher（后台） ----
section "Step 2/4 — image_loop_publisher"
if [ ! -f "$WS_ROOT/modules/m04-ai-vision-and-edge-acceleration/4.1-yolo-object-detection/ros2/bev_detection/test/image_loop_publisher.py" ]; then
  fail "image_loop_publisher.py not found"
  kill $YOLO_PID 2>/dev/null
  exit 1
fi
python3 "$WS_ROOT/modules/m04-ai-vision-and-edge-acceleration/4.1-yolo-object-detection/ros2/bev_detection/test/image_loop_publisher.py" \
  --dir "$SAMPLE_DIR" \
  --fps 2 \
  --num-frames "$NUM_SAMPLES" \
  --timeout "$TIMEOUT_PUBLISH" \
  --mode "$PUB_MODE" \
  > "$LOG_DIR/publisher.log" 2>&1 &
PUB_PID=$!
echo "  publisher PID=$PUB_PID  (logs: $LOG_DIR/publisher.log)"

# Wait for both
sleep 4

# ---- 3. 健康检查 ----
section "Step 3/4 — health check"
for i in 1 2 3 4 5 6 7 8 9 10; do
  if ros2 topic list 2>/dev/null | grep -q "/perception/detections"; then
    ok "topics up after ${i}s"
    break
  fi
  sleep 1
done

if ! ros2 topic list 2>/dev/null | grep -q "/perception/detections"; then
  fail "/perception/detections topic never appeared"
  fail "see $LOG_DIR/yolo.log for details"
  kill $YOLO_PID $PUB_PID 2>/dev/null
  exit 1
fi

echo "  --- live topics (subset) ---"
ros2 topic list 2>/dev/null | grep -E "perception" | sed 's/^/    /'
echo "  ----------------------------"

# Wait an extra moment so the loop publisher has emitted at least one frame
# AND the yolo_trt_node has had a chance to process it AND publish the
# debug_image with bbox drawn on it.
echo "  waiting 4s for first inference to round-trip..."
sleep 4

# ---- 4. 截图 / 实时打印 ----
section "Step 4/4 — visualization output"
if [ "$SKIP_SCREENSHOT" = "1" ]; then
  ok "skipped screenshot (SKIP_SCREENSHOT=1). nodes still running."
  echo "  monitor live:"
  echo "    ros2 topic hz /perception/detections"
  echo "    tail -f $LOG_DIR/yolo.log"
  echo ""
  echo "  stopping nodes in 8s..."
  sleep 8
else
  python3 "$WS_ROOT/modules/m04-ai-vision-and-edge-acceleration/4.1-yolo-object-detection/ros2/bev_detection/test/screenshot_saver.py" \
    --out "$SNAPSHOT" \
    --topic "/perception/debug/detection_image" \
    --timeout 25 \
    --min-frames 1 \
    > "$LOG_DIR/screenshot.log" 2>&1
  SHOT_RC=$?

  if [ "$SHOT_RC" -eq 0 ] && [ -f "$SNAPSHOT" ]; then
    ok "snapshot saved: $SNAPSHOT"
    SIZE=$(stat -c '%s' "$SNAPSHOT" 2>/dev/null || stat -f '%z' "$SNAPSHOT")
    echo "    file size: $SIZE bytes"
    echo "    open with:  xdg-open $SNAPSHOT  (or copy to host)"
  else
    fail "snapshot failed (rc=$SHOT_RC, see $LOG_DIR/screenshot.log)"
  fi

  # 也保存一份 det 文本输出
  echo "  --- live detection count ---"
  ros2 topic echo /perception/detections --once --field detections.size \
    2>/dev/null | head -3 | sed 's/^/    /' || true
fi

# ---- 5. Cleanup ----
section "Cleanup"
kill $YOLO_PID $PUB_PID 2>/dev/null
sleep 1
kill -9 $YOLO_PID $PUB_PID 2>/dev/null
wait $YOLO_PID 2>/dev/null
wait $PUB_PID 2>/dev/null
ok "all nodes stopped"
echo ""
ok "DONE. artifacts:"
echo "    snapshot:    $SNAPSHOT"
echo "    yolo.log:    $LOG_DIR/yolo.log"
echo "    pub.log:     $LOG_DIR/publisher.log"
echo "    screen.log:  $LOG_DIR/screenshot.log"
exit 0
