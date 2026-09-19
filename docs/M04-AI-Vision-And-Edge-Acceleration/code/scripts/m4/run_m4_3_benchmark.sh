#!/usr/bin/env bash
# scripts/m4/run_m4_3_benchmark.sh
#
# M4.3 benchmark: 推理 latency / FPS report.
# 使用真实 camera image stream, 跑 N 次推理, 报告 P50 / P95 latency.
#
# 依赖: trtexec (仅做 baseline), 真实 .engine, ROS 节点已启动并接受 image.

set -uo pipefail

REPO_ROOT="${REPO_ROOT:-/home/seeed/mobile-robot-full-stack-course/modules/m04-ai-vision-and-edge-acceleration}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
PKG="bev_segmentation"
ENGINE="$REPO_ROOT/models/m4/segmentation/engines/segformer_b0_fp16.engine"
OUT_DIR="$REPO_ROOT/output/m4/4.3"
LOG_DIR="$OUT_DIR/logs"
N_FRAMES="${N_FRAMES:-200}"

mkdir -p "$LOG_DIR"

[[ -f "$ENGINE" ]] || { echo "[bench] FAIL: missing engine $ENGINE"; exit 2; }

source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null
source $REPO_ROOT/ros2_ws/install/setup.bash 2>/dev/null || true

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
echo "[bench] [2/2] end-to-end ROS topic rate (Hz)..."
echo "[bench] (启动 segmentation_node + image_republisher, 跑 ${N_FRAMES} 帧后停止)"

# start node
ros2 run $PKG segmentation_node \
    --ros-args \
    --params-file "$REPO_ROOT/ros2_ws/src/$PKG/config/segmentation.yaml" \
    > "$LOG_DIR/node.log" 2>&1 &
NODE_PID=$!
trap "kill $NODE_PID 2>/dev/null || true" EXIT

for i in $(seq 1 30); do sleep 0.5; ros2 node list 2>/dev/null | grep -q segmentation_node && break; done

# 推入 N_FRAMES 帧 (用 cv_bridge 生成 + publish)
python3 - "$N_FRAMES" "$REPO_ROOT/output/m4/4.3/test_input.png" "$LOG_DIR/e2e_publish.log" << 'PY'
import sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

n = int(sys.argv[1])
img_path = sys.argv[2]

rclpy.init()
node = rclpy.create_node('bench_publisher')
pub = node.create_publisher(Image, '/perception/cameras/front/image', 10)
bridge = CvBridge()
img = cv2.imread(img_path)
if img is None:
    raise SystemExit(f'no test image: {img_path}')

t0 = time.monotonic()
for i in range(n):
    msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.01)
elapsed = time.monotonic() - t0
print(f'[bench_pub] {n} frames in {elapsed:.2f}s = {n/elapsed:.1f} FPS pub-side')
node.destroy_node()
rclpy.shutdown()
PY

# wait for downstream
sleep 2

# 测量 topic rate
echo "[bench] /perception/drivable_mask topic rate:"
timeout 5 ros2 topic hz /perception/drivable_mask 2>&1 | tee "$LOG_DIR/hz_drivable.log" | head -10
echo "[bench] /perception/semantic_mask topic rate:"
timeout 5 ros2 topic hz /perception/semantic_mask 2>&1 | tee "$LOG_DIR/hz_semantic.log" | head -10

# 从 node log 抓 infer ms
echo "[bench] infer timings from node log (if printed):"
grep -E "infer|inference|ms" "$LOG_DIR/node.log" | tail -20 || echo "  (no per-frame infer log printed)"

# 报告
echo "[bench] === Done. Artifacts ==="
echo "  trtexec:       $LOG_DIR/trtexec.log"
echo "  hz_drivable:   $LOG_DIR/hz_drivable.log"
echo "  hz_semantic:   $LOG_DIR/hz_semantic.log"
echo "  node log:      $LOG_DIR/node.log"

kill $NODE_PID 2>/dev/null || true
