#!/usr/bin/env bash
# scripts/regression/run_m4_web_regression.sh
#
# Headless regression for the M4 web preview server.
#
# Does NOT require physical GMSL. Uses a synthetic ROS image source
# publishing bgr8 frames on /perception/demo/m4_1. For the test only.
#
# Acceptance per cycle:
#   1. synthetic publisher alive on /perception/demo/m4_1 (10 Hz)
#   2. web server starts on a reserved port (default 8089)
#   3. GET /healthz returns server_ready=true within 5s
#   4. GET /healthz returns ros_frame_ready=true within 5s
#   5. aiortc client performs offer/answer, peer reaches "connected"
#      within 10s, first RTP frame within 2s of connection
#   6. SIGINT, port freed within 2s, no orphan
#   7. immediate restart succeeds (no manual kill needed)
#
# Runs three cycles by default.
#
# Usage:
#   scripts/regression/run_m4_web_regression.sh
#   CYCLES=5 scripts/regression/run_m4_web_regression.sh
#
# Exits 0 only if all cycles pass.

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
export M4_ROOT WS_ROOT

set -u
# NOTE: do NOT enable 'set -e'; we explicitly handle error codes from
# sub-processes and want the regression to attempt all cycles. Also do
# not source ROS under `set -u` because /opt/ros/humble/setup.bash
# touches unbound variables during its own initialization.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$WS_ROOT/"
CYCLES="${CYCLES:-3}"
PORT="${PORT:-8089}"
LOG_DIR="${LOG_DIR:-$M4_ROOT/output/regression/m4_web_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "$LOG_DIR"

# shellcheck disable=SC1091
# Disable `set -u` while sourcing ROS so /opt/ros/humble/setup.bash
# can use its own conventions. Re-enable below.
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

# Sanity: aiortc and aiohttp must be importable from the test process.
python3 -c "import aiohttp, aiortc" 2>/dev/null || {
    echo "[FAIL] aiohttp or aiortc not importable; cannot run regression"
    exit 2
}

echo "== M4 web preview regression =="
echo "  cycles=$CYCLES port=$PORT log_dir=$LOG_DIR"

# Helper: run a single cycle. Args: <cycle_index>
run_cycle() {
    local idx="$1"
    local cycle_log="$LOG_DIR/cycle_${idx}.log"
    echo
    echo "-- cycle $idx --"
    if python3 "$WS_ROOT/modules/m04-ai-vision-and-edge-acceleration/common/ros2/m4_demo_bringup/test/m4_web_regression_cycle.py" \
        --port "$PORT" \
        --cycles 1 \
        --log-dir "$LOG_DIR/cycle_${idx}" \
        2>&1 | tee "$cycle_log"; then
        echo "[OK] cycle $idx"
        return 0
    fi
    echo "[FAIL] cycle $idx (see $cycle_log)"
    return 1
}

# Ensure the test port is free at start.
python3 - "$PORT" <<'PY'
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    s.bind(("127.0.0.1", int(sys.argv[1])))
    s.close()
    sys.exit(0)
except OSError:
    sys.exit(1)
PY
if [ $? -ne 0 ]; then
    echo "[FAIL] port $PORT already in use; cannot start regression"
    exit 2
fi

passes=0
fails=0
for i in $(seq 1 "$CYCLES"); do
    if run_cycle "$i"; then
        passes=$((passes + 1))
    else
        fails=$((fails + 1))
    fi
done

echo
echo "== regression summary =="
echo "  passes=$passes fails=$fails cycles=$CYCLES"
echo "  log_dir=$LOG_DIR"
[ "$fails" = "0" ]
