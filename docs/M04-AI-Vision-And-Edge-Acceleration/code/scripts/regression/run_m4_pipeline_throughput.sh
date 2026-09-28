#!/usr/bin/env bash
# scripts/regression/run_m4_pipeline_throughput.sh
#
# Guards the single most damaging regression found in the M4 web pipeline.
#
# `sensor_msgs/Image.data` is a `uint8[]` that rclpy backs with an
# `array.array`. Assigning a **`bytes`** object to it makes rclpy convert
# element-by-element through Python at ~149 ns/byte — measured 927 ms for one
# 1920x1080 bgr8 frame. That one line capped csi_camera_publisher.py at
# 1.2 fps and dragged the whole chain (YOLO detections, tracking, every
# /perception/demo/m4_* overlay) down to ~1.16 Hz. Assigning an
# `array.array('B', ...)` instead costs ~0.6 ms (verified 1.2 -> 29.7 fps).
#
# This runs the REAL publisher straight from the source tree against the
# synthetic GStreamer source, so it needs no camera and cannot contend for
# /dev/video0 with a running demo.
#
# Usage:
#   scripts/regression/run_m4_pipeline_throughput.sh
#   MIN_FPS=25 scripts/regression/run_m4_pipeline_throughput.sh
#   WIDTH=1280 HEIGHT=720 scripts/regression/run_m4_pipeline_throughput.sh
#
# Exits 0 only when the measured rate is >= MIN_FPS.

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
export M4_ROOT WS_ROOT

set -u
# Do NOT enable 'set -e' (we want the log inspected on failure) and do not
# source ROS under 'set -u' — /opt/ros/humble/setup.bash touches unbound vars.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$WS_ROOT/"
PUBLISHER="$WS_ROOT/install/m4_demo_bringup/lib/m4_demo_bringup/csi_camera_publisher"

MIN_FPS="${MIN_FPS:-25}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1080}"
FPS="${FPS:-30}"
DURATION="${DURATION:-15}"
TOPIC="${TOPIC:-/probe/m4_throughput}"
LOG_DIR="${LOG_DIR:-$M4_ROOT/output/regression/m4_throughput_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/publisher.log"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

if [ ! -f "$PUBLISHER" ]; then
    echo "[FAIL] missing publisher: $PUBLISHER"
    exit 2
fi

echo "== M4 image pipeline throughput guard =="
echo "  publisher=$PUBLISHER"
echo "  ${WIDTH}x${HEIGHT}@${FPS}  min_fps=$MIN_FPS  duration=${DURATION}s"
echo "  log=$LOG"

timeout $((DURATION + 20)) python3 "$PUBLISHER" \
    --source test --width "$WIDTH" --height "$HEIGHT" --fps "$FPS" \
    --topic "$TOPIC" --timeout "$DURATION" > "$LOG" 2>&1
rc=$?

RATE=$(grep -o '([0-9.]* fps)' "$LOG" | tail -1 | tr -dc '0-9.')
if [ -z "$RATE" ]; then
    echo "[FAIL] publisher produced no 'fps' line (rc=$rc). Tail of $LOG:"
    tail -20 "$LOG"
    exit 1
fi

echo "  measured: ${RATE} fps"

if awk -v r="$RATE" -v m="$MIN_FPS" 'BEGIN { exit !(r >= m) }'; then
    echo "== THROUGHPUT GUARD PASSED (${RATE} >= ${MIN_FPS} fps) =="
    exit 0
fi

echo "== THROUGHPUT GUARD FAILED (${RATE} < ${MIN_FPS} fps) =="
echo "   Most likely cause: a 'bytes' object was assigned to"
echo "   sensor_msgs/Image.data again (element-wise, ~149 ns/byte), or a"
echo "   numpy -> Image path lost its bulk buffer-protocol copy."
echo "   See the PERFORMANCE CONTRACT in:"
echo "     $PUBLISHER"
exit 1
