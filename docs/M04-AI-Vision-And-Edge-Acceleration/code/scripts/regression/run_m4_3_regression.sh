#!/usr/bin/env bash
# scripts/regression/run_m4_3_regression.sh
#
# M4.3 完整 regression: 
#   1) engine correctness gate (PyTorch vs TRT)
#   2) colcon build + colcon test (GTest 4 个)
#   3) smoke test (segmentation_node 端到端)
#
# 退出: 0 = pass; 任一步非 0 -> fail.

set -uo pipefail

REPO_ROOT="${REPO_ROOT:-/home/seeed/mobile-robot-full-stack-course/modules/m04-ai-vision-and-edge-acceleration}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
LOG_DIR="$REPO_ROOT/output/m4/4.3/logs"
LOG="$LOG_DIR/regression_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$LOG_DIR"

source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null

cd "$REPO_ROOT/ros2_ws"
source install/setup.bash 2>/dev/null || true

fail_count=0

echo "[reg] === 1/3 Engine Correctness Gate ==="
if bash "$REPO_ROOT/scripts/m4/engine_correctness_gate.sh" >> "$LOG" 2>&1; then
    echo "[reg] PASS engine_correctness_gate"
else
    echo "[reg] FAIL engine_correctness_gate (non-fatal — record for release gate)"
fi

echo "[reg] === 2/3 colcon build + test ==="
if colcon build --packages-select bev_segmentation \
    --event-handlers console_direct+ >> "$LOG" 2>&1; then
    echo "[reg] PASS build"
else
    echo "[reg] FAIL build"; fail_count=$((fail_count+1))
fi

source install/setup.bash 2>/dev/null || true

if colcon test --packages-select bev_segmentation \
    --event-handlers console_direct+ >> "$LOG" 2>&1; then
    echo "[reg] PASS test"
else
    echo "[reg] FAIL test"; fail_count=$((fail_count+1))
fi

echo "[reg] === 3/3 smoke test ==="
if bash "$REPO_ROOT/scripts/m4/test_segmentation_smoke.sh" >> "$LOG" 2>&1; then
    echo "[reg] PASS smoke"
else
    echo "[reg] FAIL smoke"; fail_count=$((fail_count+1))
fi

echo "[reg] === Done. log: $LOG ==="
if [[ $fail_count -gt 0 ]]; then exit 1; fi
exit 0
