#!/usr/bin/env bash
# scripts/m4/test_segmentation_smoke.sh
#
# M4.3 smoke test:
#   1) launch segmentation_node
#   2) publish a single test image via image_republisher.py
#   3) verify semantic_mask + drivable_mask topics output a frame
#
# 退出码: 0 = pass, 非 0 = fail
# 期望: ONNX + .engine + labels.json 都已就位; 若任何缺失则 fail-fast.

set -uo pipefail

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
PKG="bev_segmentation"
TEST_IMAGE="${TEST_IMAGE:-$M4_ROOT/output/m4/4.3/test_input.png}"
OUT_DIR="$M4_ROOT/output/m4/4.3"
SEMANTIC_DIR="$OUT_DIR/semantic_masks"
DRIVABLE_DIR="$OUT_DIR/drivable_masks"
LOG_DIR="$OUT_DIR/logs"
ENGINE="$M4_ROOT/models/m4/segmentation/engines/segformer_b0_fp16.engine"
ONNX="$M4_ROOT/models/m4/segmentation/onnx/segformer_b0.onnx"
LABELS="$M4_ROOT/models/m4/segmentation/labels/labels.json"

mkdir -p "$SEMANTIC_DIR" "$DRIVABLE_DIR" "$LOG_DIR"

# ---- preflight ----
for f in "$ENGINE" "$ONNX" "$LABELS"; do
    [[ -f "$f" ]] || { echo "[smoke] FAIL: missing $f"; exit 2; }
done
[[ -f "$TEST_IMAGE" ]] || { echo "[smoke] FAIL: missing test image $TEST_IMAGE"; exit 2; }

# ROS's setup.bash reads variables that are unset here, and 'set -u'
# turns that into a fatal error: the script would die before its first
# echo. The other M4 scripts already guard their source this way.
set +u
source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null
source $WS_ROOT/install/setup.bash 2>/dev/null || true
set -u

# ---- launch ----
echo "[smoke] starting segmentation_node (logs -> $LOG_DIR/node.log)"
ros2 run $PKG segmentation_node \
    --ros-args \
    --params-file "$WS_ROOT/src/$PKG/config/segmentation.yaml" \
    > "$LOG_DIR/node.log" 2>&1 &
NODE_PID=$!
trap "kill $NODE_PID 2>/dev/null || true" EXIT

# wait for node to be ready
for i in $(seq 1 30); do
    sleep 0.5
    if ros2 node list 2>/dev/null | grep -q segmentation_node; then break; fi
done

if ! ros2 node list 2>/dev/null | grep -q segmentation_node; then
    echo "[smoke] FAIL: node did not come up. log tail:"; tail -50 "$LOG_DIR/node.log"
    exit 3
fi
echo "[smoke] node up."

# ---- publish test image once ----
echo "[smoke] publishing test image -> $TEST_IMAGE"
python3 "$WS_ROOT/src/$PKG/test/image_republisher.py" \
    --image "$TEST_IMAGE" \
    --once \
    > "$LOG_DIR/repub.log" 2>&1 &
REPUB_PID=$!

# ---- wait for output frames ----
echo "[smoke] waiting up to 10s for /perception/semantic_mask + /perception/drivable_mask frames..."
SEM_OK=0; DRV_OK=0
for i in $(seq 1 20); do
    sleep 0.5
    if [[ $SEM_OK -eq 0 ]] && ros2 topic echo /perception/semantic_mask sensor_msgs/msg/Image \
        --once --timeout 1 2>/dev/null | grep -q 'encoding:'; then
        SEM_OK=1
    fi
    if [[ $DRV_OK -eq 0 ]] && ros2 topic echo /perception/drivable_mask sensor_msgs/msg/Image \
        --once --timeout 1 2>/dev/null | grep -q 'encoding:'; then
        DRV_OK=1
    fi
    [[ $SEM_OK -eq 1 && $DRV_OK -eq 1 ]] && break
done

if [[ $SEM_OK -eq 0 ]]; then echo "[smoke] FAIL: semantic_mask topic empty"; tail -40 "$LOG_DIR/node.log"; exit 4; fi
if [[ $DRV_OK -eq 0 ]]; then echo "[smoke] FAIL: drivable_mask topic empty"; tail -40 "$LOG_DIR/node.log"; exit 5; fi

# ---- 落盘 mask 验证 (用 rqt_image_view 之外: 直接通过 ros2 bag record + 解析, 简化版只 echo) ----
echo "[smoke] recording 1 frame of each mask..."
timeout 3 ros2 bag record -o "$OUT_DIR/_smoke_bag" \
    /perception/semantic_mask /perception/drivable_mask \
    > "$LOG_DIR/record.log" 2>&1 || true

echo "[smoke] PASS — node published both masks"
echo "  semantic_mask:  /perception/semantic_mask"
echo "  drivable_mask:  /perception/drivable_mask"
echo "  test image:     $TEST_IMAGE"
echo "  logs:           $LOG_DIR/"
kill $REPUB_PID 2>/dev/null || true
exit 0
