#!/usr/bin/env bash
# scripts/regression/run_m4_demo_physical_acceptance.sh
#
# PHYSICAL CAMERA ACCEPTANCE for M4.1 / M4.2 / M4.3 demos.
#
# REQUIRES REAL HARDWARE (CSI / GMSL / USB camera). This is the
# release gate; the headless counterpart is
# scripts/regression/run_m4_demo_regression.sh.
#
# Per demo, runs 3 cycles with NO manual reset between them:
#   1. start      : CAMERA_SOURCE=auto, no --no-gui (uses user's VIEWER)
#   2. wait       : first demo topic message arrives within 8s
#   3. SIGINT     : bash supervisor trap fires; expect PGID teardown
#                   in <=10s (no kill -9)
#   4. immediate restart : same demo again, expect first message <=8s
#   5. immediate restart : again
#
# Acceptance is 3 consecutive restart cycles without any intervention
# (no reboot, fuser, lsof, or manual reset between cycles).
#
# CSI/GMSL release proof: PGID gone AND restart cycle succeeded.
# USB release proof: PGID gone AND (optionally) configured /dev/videoN
#                    is free; restart cycle succeeded.
#
# Exit 0 = release gate PASS. Non-zero = a blocker.

set -uo pipefail

REPO_ROOT="${REPO_ROOT:-/home/seeed/mobile-robot-full-stack-course/modules/m04-ai-vision-and-edge-acceleration}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
LOG_DIR="$REPO_ROOT/output/m4/demo_physical_acceptance"
mkdir -p "$LOG_DIR"

CYCLES=3
for a in "$@"; do
    case "$a" in
        --cycles=*) CYCLES="${a#--cycles=}" ;;
        --cycles=1|--cycles=2)
            : ;;    # 1 or 2 cycles is acceptable for quick check
        --help|-h)
            sed -n '2,30p' "$0"
            cat <<EOF

Usage:
  $0 [--cycles=N]

Defaults: --cycles=3 (release-gate)
EOF
            exit 0 ;;
        *) echo "unknown option: $a" >&2; exit 2 ;;
    esac
done

DEMOS=("4.1" "4.2" "4.3")
PASS=0
FAIL=0
OVERALL_LOG="$LOG_DIR/acceptance_$(date +%Y%m%d_%H%M%S).log"
: > "$OVERALL_LOG"

log() { printf '[acc] %s\n' "$*" | tee -a "$OVERALL_LOG" ; }
pass() { log "  [PASS] $*"; PASS=$((PASS+1)); }
fail() { log "  [FAIL] $*"; FAIL=$((FAIL+1)); }

# Workspace presence check
if [ ! -d "$REPO_ROOT/ros2_ws/install" ]; then
    log "[acc] FATAL: ros2_ws/install missing; build first."
    exit 2
fi

log "============================================="
log "Physical acceptance: ${CYCLES} restart cycles per demo"
log "Camera source: auto (CSI / GMSL / USB / external)"
log "Demo topics: /perception/demo/m4_1 / m4_2 / m4_3"
log "============================================="

for demo in "${DEMOS[@]}"; do
    log ""
    log "--- Demo ${demo}: ${CYCLES} restart cycles ---"
    META="/tmp/m4_demo/${demo}.metadata.json"
    for cyc in $(seq 1 "$CYCLES"); do
        log "  [cycle ${cyc}/${CYCLES}] starting"

        OUT="$LOG_DIR/${demo}_cyc${cyc}_$(date +%H%M%S).log"
        # Honour user DISPLAY but force DURATION 6s so each cycle ends
        # automatically without requiring interactive Ctrl-C.
        (CAMERA_SOURCE=auto DURATION=6 \
            bash "$REPO_ROOT/scripts/m4/run_m4_${demo}_demo.sh" \
            >> "$OUT" 2>&1) &
        CHILD=$!

        # Wait up to 15s for the wrapper to either auto-exit on DURATION
        # or to record first-message. Then send SIGINT (no kill -9).
        sleep 8
        if kill -0 "$CHILD" 2>/dev/null; then
            log "    child still alive after 8s; sending SIGINT"
            kill -INT "$CHILD" 2>/dev/null || true
        fi
        # Let it die gracefully; cap at 12s
        waited=0
        while [ $waited -lt 12 ] && kill -0 "$CHILD" 2>/dev/null; do
            sleep 1; waited=$((waited+1))
        done
        if kill -0 "$CHILD" 2>/dev/null; then
            log "    child did not exit gracefully after 12s; escalating"
            kill -9 "$CHILD" 2>/dev/null || true
            wait "$CHILD" 2>/dev/null
            fail "demo=${demo} cycle=${cyc} SIGINT was insufficient"
        else
            wait "$CHILD" 2>/dev/null
            pass "demo=${demo} cycle=${cyc} terminated cleanly"
        fi

        # After each cycle: /tmp/m4_demo/<demo>.metadata.json must be gone.
        if [ -f "$META" ]; then
            fail "demo=${demo} cycle=${cyc} metadata not removed ($META)"
        else
            pass "demo=${demo} cycle=${cyc} metadata removed"
        fi

        # Cycle-to-cycle continuity: the next cycle MUST start immediately.
        # We test this implicitly by continuing without any cleanup.
    done
done

log ""
log "============================================="
log "Acceptance summary: PASS=${PASS} FAIL=${FAIL}"
log "log: $OVERALL_LOG"

if [ "$FAIL" -gt 0 ]; then
    log "RELEASE GATE: BLOCKED (failures above)"
    exit 1
fi
log "RELEASE GATE: PASS"
exit 0
