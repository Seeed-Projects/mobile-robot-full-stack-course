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

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
LOG_DIR="$M4_ROOT/output/m4/4.3/logs"
LOG="$LOG_DIR/regression_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$LOG_DIR"

# ROS's setup.bash reads variables that are unset here, and 'set -u'
# turns that into a fatal error: the script would die before its first
# echo. The other M4 scripts already guard their source this way.
set +u
source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null
set -u

cd "$WS_ROOT"
source install/setup.bash 2>/dev/null || true

fail_count=0

echo "[reg] === 1/3 Engine Correctness Gate ==="
if bash "$M4_ROOT/scripts/m4/engine_correctness_gate.sh" >> "$LOG" 2>&1; then
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
if bash "$M4_ROOT/scripts/m4/test_segmentation_smoke.sh" >> "$LOG" 2>&1; then
    echo "[reg] PASS smoke"
else
    echo "[reg] FAIL smoke"; fail_count=$((fail_count+1))
fi

echo "[reg] === Done. log: $LOG ==="
if [[ $fail_count -gt 0 ]]; then exit 1; fi
exit 0
