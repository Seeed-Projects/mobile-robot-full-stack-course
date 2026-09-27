#!/usr/bin/env bash
# scripts/m4/run_m4_3_benchmark.sh
#
# M4.3 benchmark: 推理 latency / FPS report.
# 使用真实 camera image stream, 跑 N 次推理, 报告 P50 / P95 latency.
#
# 依赖: trtexec (仅做 baseline), 真实 .engine, ROS 节点已启动并接受 image.

set -uo pipefail

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
PKG="bev_segmentation"
ENGINE="$M4_ROOT/models/m4/segmentation/engines/segformer_b0_fp16.engine"
OUT_DIR="$M4_ROOT/output/m4/4.3"
LOG_DIR="$OUT_DIR/logs"
N_FRAMES="${N_FRAMES:-200}"

mkdir -p "$LOG_DIR"

[[ -f "$ENGINE" ]] || { echo "[bench] FAIL: missing engine $ENGINE"; exit 2; }

# ROS's setup.bash reads variables that are unset here, and 'set -u'
# turns that into a fatal error: the script would die before its first
# echo. The other M4 scripts already guard their source this way.
set +u
source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null
source $WS_ROOT/install/setup.bash 2>/dev/null || true
set -u

echo "[bench] === M4.3 Semantic Segmentation Latency Benchmark ==="
echo "[bench] engine: $ENGINE"
echo "[bench] frames: $N_FRAMES"

# ---- 1) trtexec baseline (仅硬件层 throughput) ----
echo "[bench] [1/2] trtexec baseline (HW throughput, no ROS overhead)..."
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
"$TRTEXEC" --loadEngine="$ENGINE" --iterations="$N_FRAMES" \
    --useSpinWait 2>&1 | tee "$LOG_DIR/trtexec.log" | grep -E "Throughput|min:|max:|mean:|median:" | head -10

# ---- 2) end-to-end latency (从 camera image 输入到 mask 输出) ----
#    通过 ROS 时间戳对齐: 测量 msg header.stamp 在 topic 上的差值.
#    简化: 在 segmentation_node 内部已经打印一次推理耗时; 我们通过 ros2 topic hz + 自定义 timer
#    给出 end-to-end rate. 实际 per-frame latency 通过 node log 中 "infer ms" 抓取.
# ---- 2) end-to-end ROS topic rate (Hz) ----
# Two modes, decided by what is ALREADY running:
#   owned  -- nothing is up: start our own segmentation_node, feed it frames,
#             tear it down afterwards.
#   attach -- a segmentation_node is already running (the usual case: the web
#             hub owns the pipeline). Start NOTHING and measure it. A second
#             node would double-publish /perception/semantic_mask and
#             /perception/drivable_mask.
echo "[bench] [2/2] end-to-end ROS topic rate (Hz)..."

TEST_IMAGE="${TEST_IMAGE:-$M4_ROOT/output/m4/4.3/test_input.png}"
NODE_PID=""
cleanup_bench_node() { [ -n "$NODE_PID" ] && kill "$NODE_PID" 2>/dev/null || true; }
trap cleanup_bench_node EXIT

if ros2 node list 2>/dev/null | grep -q 'segmentation_node'; then
    MODE=attach
else
    MODE=owned
fi
echo "[bench] mode: $MODE"

SEG_PARAMS="$WS_ROOT/install/$PKG/share/$PKG/config/segmentation.yaml"
[ -f "$SEG_PARAMS" ] || SEG_PARAMS="$WS_ROOT/src/$PKG/config/segmentation.yaml"

if [ "$MODE" = owned ]; then
    ros2 run "$PKG" segmentation_node \
        --ros-args --params-file "$SEG_PARAMS" \
        > "$LOG_DIR/node.log" 2>&1 &
    NODE_PID=$!

    for _ in $(seq 1 30); do
        sleep 0.5
        ros2 node list 2>/dev/null | grep -q 'segmentation_node' && break
    done

    # Push N_FRAMES frames. A real frame is used when one is on disk; otherwise
    # a synthetic one is generated, so the benchmark never depends on a file
    # that may not exist.
    python3 - "$N_FRAMES" "$TEST_IMAGE" "$LOG_DIR/e2e_publish.log" << 'PY'
import os, sys, time
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

n = int(sys.argv[1])
img_path = sys.argv[2]

rclpy.init()
node = rclpy.create_node('bench_publisher')
pub = node.create_publisher(Image, '/perception/cameras/front/image', 10)
bridge = CvBridge()

img = cv2.imread(img_path) if img_path and os.path.exists(img_path) else None
if img is None:
    print('[bench_pub] no usable image at %r - publishing a synthetic frame' % img_path)
    img = np.full((1080, 1920, 3), 40, np.uint8)
    cv2.rectangle(img, (200, 200), (1000, 800), (90, 140, 90), -1)
    cv2.line(img, (0, 540), (1919, 540), (230, 230, 230), 12)

t0 = time.monotonic()
for _ in range(n):
    msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.01)
elapsed = time.monotonic() - t0
print('[bench_pub] %d frames in %.2fs = %.1f FPS pub-side' % (n, elapsed, n / elapsed))
node.destroy_node()
rclpy.shutdown()
PY
else
    echo "[bench] a segmentation_node is already running - measuring it, starting nothing."
    echo "[bench] (a second node would double-publish the mask topics)"
fi

sleep 2

echo "[bench] /perception/drivable_mask topic rate:"
timeout 5 ros2 topic hz /perception/drivable_mask 2>&1 | tee "$LOG_DIR/hz_drivable.log" | head -10
echo "[bench] /perception/semantic_mask topic rate:"
timeout 5 ros2 topic hz /perception/semantic_mask 2>&1 | tee "$LOG_DIR/hz_semantic.log" | head -10

DRIVABLE_HZ=$(grep -o 'average rate: [0-9.]*' "$LOG_DIR/hz_drivable.log" 2>/dev/null | tail -1 | awk '{print $3}')
SEMANTIC_HZ=$(grep -o 'average rate: [0-9.]*' "$LOG_DIR/hz_semantic.log" 2>/dev/null | tail -1 | awk '{print $3}')

echo "[bench] infer timings from node log (if printed):"
grep -Ei "infer|inference| ms" "$LOG_DIR/node.log" 2>/dev/null | tail -20 || true

ENGINE_SIZE=$(stat -c%s "$ENGINE" 2>/dev/null || echo 0)
cat > "$OUT_DIR/benchmark.json" <<JSON
{
  "model": "segformer_b0",
  "engine": "$ENGINE",
  "device": "Jetson AGX Orin",
  "jetpack": "6.2.1",
  "tensorrt": "10.3.0",
  "precision": "fp16",
  "mode": "${MODE}",
  "frames_requested": ${N_FRAMES},
  "engine_size_bytes": ${ENGINE_SIZE},
  "semantic_mask_hz": ${SEMANTIC_HZ:-0},
  "drivable_mask_hz": ${DRIVABLE_HZ:-0},
  "timestamp": "$(date -Iseconds)"
}
JSON

echo "[bench] === Done. Artifacts ==="
echo "  benchmark:     $OUT_DIR/benchmark.json"
echo "  trtexec:       $LOG_DIR/trtexec.log"
echo "  hz_drivable:   $LOG_DIR/hz_drivable.log"
echo "  hz_semantic:   $LOG_DIR/hz_semantic.log"
echo "  node log:      $LOG_DIR/node.log"
cat "$OUT_DIR/benchmark.json"
