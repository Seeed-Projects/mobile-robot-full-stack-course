#!/usr/bin/env bash
# scripts/regression/run_m4_web_hub_regression.sh
#
# Headless regression for the UNIFIED M4 web hub (one server, three modules).
# Requires no physical GMSL: synthetic publishers feed the three overlay
# topics. Complements run_m4_web_regression.sh, which covers single-demo mode.
#
# Usage:
#   scripts/regression/run_m4_web_hub_regression.sh
#   CYCLES=5 scripts/regression/run_m4_web_hub_regression.sh
#
# Exits 0 only if all cycles pass.

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
export M4_ROOT WS_ROOT

set -u
# NOTE: do NOT enable 'set -e'; we want every cycle attempted. Also do not
# source ROS under 'set -u' — /opt/ros/humble/setup.bash touches unbound vars.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$WS_ROOT/"
CYCLES="${CYCLES:-3}"
PORT="${PORT:-8091}"
LOG_DIR="${LOG_DIR:-$M4_ROOT/output/regression/m4_web_hub_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "$LOG_DIR"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

TEST_PY="$M4_ROOT/common/ros2/m4_demo_bringup/test/m4_web_hub_regression.py"
if [ ! -f "$TEST_PY" ]; then
    echo "[FAIL] missing test: $TEST_PY"
    exit 2
fi

echo "== M4 web HUB regression =="
echo "  cycles=$CYCLES port=$PORT log_dir=$LOG_DIR"

passes=0
fails=0
for i in $(seq 1 "$CYCLES"); do
    echo
    echo "-- cycle $i --"
    # A pipe to tee masks the test's exit status, so read PIPESTATUS[0]
    # explicitly: a broken cycle must never report PASS.
    python3 "$TEST_PY" --port "$PORT" --log-dir "$LOG_DIR/cycle_$i" \
        2>&1 | tee "$LOG_DIR/cycle_$i.log"
    rc=${PIPESTATUS[0]}
    if [ "$rc" -eq 0 ]; then
        echo "[OK] cycle $i"
        passes=$((passes + 1))
    else
        echo "[FAIL] cycle $i (rc=$rc, see $LOG_DIR/cycle_$i.log)"
        fails=$((fails + 1))
    fi
done

echo
echo "== hub regression summary =="
echo "  passes=$passes fails=$fails cycles=$CYCLES"
echo "  log_dir=$LOG_DIR"
[ "$fails" = "0" ]
