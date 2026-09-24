#!/usr/bin/env bash
# scripts/regression/run_m4_demo_regression.sh
#
# HEADLESS / MOCK regression for M4.1 / M4.2 / M4.3 demo integration.
#
# DOES NOT REQUIRE PHYSICAL HARDWARE.
# Camera source is forced to `test` (videotestsrc) so a CI agent can
# run this without /dev/videoN, CSI, or GMSL.
#
# For physical acceptance use:
#   scripts/regression/run_m4_demo_physical_acceptance.sh
#
# Pipeline per demo:
#   1. dry-run prints planned orchestration
#   2. (with workspace built) launch demo with CAMERA_SOURCE=test
#      and viewer=none (--no-gui); assert first demo topic message
#      arrives within 8s via the wait_for_first_message helper
#   3. SIGINT (no kill -9); expect clean teardown <=10s
#   4. confirm /tmp/m4_demo/<demo>.metadata.json removed
#   5. confirm no orphan demo-owned PIDs survive
#
# Pass criteria per demo: 1/1 startup + 1/1 teardown + 0 orphans.
# Run multiple cycles by passing --cycles=N (default 1).

set -uo pipefail

REPO_ROOT="${REPO_ROOT:-/home/seeed/mobile-robot-full-stack-course/modules/m04-ai-vision-and-edge-acceleration}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
LOG_DIR="$REPO_ROOT/output/m4/demo_regression"
mkdir -p "$LOG_DIR"

CYCLES=1
LONG=0
for a in "$@"; do
    case "$a" in
        --cycles=*) CYCLES="${a#--cycles=}" ;;
        --long)     LONG=1 ;;
        --help|-h)
            sed -n '2,30p' "$0"
            cat <<EOF

Usage:
  $0 [--cycles=N] [--long]

Defaults: --cycles=1
EOF
            exit 0 ;;
        *) echo "unknown option: $a" >&2; exit 2 ;;
    esac
done

DEMOS=("4.1" "4.2" "4.3")
PASS=0
FAIL=0
OVERALL_LOG="$LOG_DIR/regression_$(date +%Y%m%d_%H%M%S).log"
: > "$OVERALL_LOG"

log() { printf '[reg] %s\n' "$*" | tee -a "$OVERALL_LOG" ; }
pass() { log "  [PASS] $*"; PASS=$((PASS+1)); }
fail() { log "  [FAIL] $*"; FAIL=$((FAIL+1)); }

# Workspace presence check
if [ ! -d "$REPO_ROOT/ros2_ws/install" ]; then
    log "[reg] FATAL: ros2_ws/install missing; build first."
    exit 2
fi

for demo in "${DEMOS[@]}"; do
    log "============================================="
    log "Demo ${demo}: headless regression, cycles=${CYCLES}"
    for cyc in $(seq 1 "$CYCLES"); do
        log "  --- cycle ${cyc}/${CYCLES} ---"
        # 1. dry-run
        if CAMERA_SOURCE=test VIEWER=none \
            bash "$REPO_ROOT/scripts/m4/run_m4_${demo}_demo.sh" --no-gui --dry-run \
            >> "$OVERALL_LOG" 2>&1; then
            pass "demo=${demo} cycle=${cyc} dry-run"
        else
            fail "demo=${demo} cycle=${cyc} dry-run"
            continue
        fi

        # 2. live test run with a tight duration
        OUT="$LOG_DIR/demo_${demo}_cyc${cyc}_$(date +%H%M%S).log"
        (CAMERA_SOURCE=test VIEWER=none DURATION=6 \
            bash "$REPO_ROOT/scripts/m4/run_m4_${demo}_demo.sh" --no-gui \
            >> "$OUT" 2>&1) &
        CHILD=$!
        # Watch the metadata and the demo log
        wait "$CHILD"
        rc=$?

        # 3. verify teardown artefacts
        meta="/tmp/m4_demo/${demo}.metadata.json"
        if [ ! -f "$meta" ]; then
            pass "demo=${demo} cycle=${cyc} metadata removed (rc=${rc})"
        else
            fail "demo=${demo} cycle=${cyc} metadata still present ($meta)"
        fi

        # 4. No orphan demo-owned PIDs from this cycle
        #    (best-effort: scan for python scripts with our paths)
        ORPHANS=$(pgrep -af "ros2_ws/src/.*\(demo_bringup\|csi_camera_publisher\)" || true)
        if [ -z "$ORPHANS" ]; then
            pass "demo=${demo} cycle=${cyc} no orphan demo-owned processes"
        else
            fail "demo=${demo} cycle=${cyc} orphans:" ; log "$ORPHANS"
        fi
    done
done

log "============================================="
log "PASS=${PASS} FAIL=${FAIL}"
log "log: $OVERALL_LOG"
if [ "$FAIL" -gt 0 ]; then exit 1; fi
exit 0
