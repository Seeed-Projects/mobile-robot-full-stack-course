#!/usr/bin/env bash
# scripts/m4/run_m4_web_hub.sh — ONE-CLICK unified M4 web preview.
#
# What it does differently from run_m4_1/2/3_demo.sh
# --------------------------------------------------
# The three per-lesson demos each bring up their own pipeline and their own
# web server on port 8080. This Hub owns one camera and one web server. The
# server starts exactly ONE selected inference lesson at a time:
#
#   ONE camera publisher  → /perception/cameras/front/image
#   selected 4.1: yolo_trt_node → /perception/demo/m4_1
#   selected 4.2: yolo_trt_node + tracking_node → /perception/demo/m4_2
#   selected 4.3: segmentation_node → /perception/demo/m4_3
#   ONE m4_web_demo_server --demo hub  →  http://<jetson>:8080/m4/1
#
# A module whose engine is not built yet (M4.3 before segformer_b0_fp16.engine
# exists) is reported as not-ready on the page instead of failing the whole
# bring-up.
#
# Usage:
#   scripts/m4/run_m4_web_hub.sh                 # native camera, 720p15 preview
#   scripts/m4/run_m4_web_hub.sh --check         # read-only preflight, no side effects
#   scripts/m4/run_m4_web_hub.sh --dry-run       # print the plan, launch nothing
#   scripts/m4/run_m4_web_hub.sh --duration=120  # auto-stop after 120s
#   CAMERA_SOURCE=test scripts/m4/run_m4_web_hub.sh   # synthetic source (no camera)
#
# Environment overrides:
#   CAMERA_SOURCE={auto|csi|v4l2|existing|gmsl|usb|test}   default: auto
#   CAMERA_DEVICE=/dev/videoN                              default: /dev/video0
#   CAMERA_WIDTH/HEIGHT/FPS                                1920/1080/30 (published)
#   CAMERA_CAPTURE_WIDTH/HEIGHT                            1920/1536 (native calibration input)
#   WEB_PORT (8080) · WEB_HOST (0.0.0.0) · WEB_BACKEND (h264|auto|mjpeg|vp8) · WEB_ACTIVE (m4_1)
#   WEB_MJPEG_SIDE_CHANNEL=1 enables the crisp fallback beside H.264 (extra CPU)
#   ENABLE_TRACKING=1 · ENABLE_SEGMENTATION=auto|0|1

set -u

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib"
# shellcheck disable=SC1091
source "$LIB_DIR/m4_demo_lib.sh"

DEMO_NAME="hub"
m4_lib_init "$DEMO_NAME"

# ---- native camera geometry ---------------------------------------------
# /dev/video0 enumerates 1920x1536, 1920x1080 and 3840x2160, all at 30 fps.
# The M4 course material specifies 1080p, so 1080p is the shared default for
# every module (previously 4.1 asked for 1536 while 4.2/4.3 asked for 720).
: "${CAMERA_SOURCE:=auto}"
: "${CAMERA_DEVICE:=/dev/video0}"
: "${CAMERA_WIDTH:=1920}"
: "${CAMERA_HEIGHT:=1080}"
: "${CAMERA_CAPTURE_WIDTH:=1920}"
: "${CAMERA_CAPTURE_HEIGHT:=1536}"
: "${CAMERA_FPS:=30}"
: "${VIEWER:=none}"          # the web hub replaces local GUI viewers
: "${DURATION:=0}"
: "${ENABLE_TRACKING:=1}"
: "${ENABLE_SEGMENTATION:=auto}"
: "${SEGMENTATION_MAX_FPS:=10.0}"
: "${M4_WEB_ACTIVE:=m4_1}"
: "${M4_RUNTIME_SETTINGS_PATH:=/home/seeed/.config/m4-perception/runtime_settings.json}"
: "${M4_IMAGE_TOPIC:=/perception/inputs/camera}"

export CAMERA_SOURCE CAMERA_DEVICE CAMERA_WIDTH CAMERA_HEIGHT CAMERA_CAPTURE_WIDTH CAMERA_CAPTURE_HEIGHT CAMERA_FPS VIEWER DURATION \
  M4_RUNTIME_SETTINGS_PATH SEGMENTATION_MAX_FPS
export M4_WEB_MANAGED_MODULES=1
export M4_IMAGE_TOPIC
export M4_SEGMENTATION_MAX_FPS="$SEGMENTATION_MAX_FPS"

awk -v fps="$SEGMENTATION_MAX_FPS" 'BEGIN { exit !(fps >= 1 && fps <= 30) }' || \
  m4_die "SEGMENTATION_MAX_FPS must be within 1..30 (got: $SEGMENTATION_MAX_FPS)"

# ---- CLI -----------------------------------------------------------------
CHECK_ONLY=0
NO_WEB=0
REAP_UNTRACKED=1
# --check / --no-web / --no-reap belong to THIS script; m4_parse_cli rejects
# unknown options, so they are stripped before delegating. Without this,
# `--check` aborts with "unknown option: --check".
PASSTHRU=()
for a in "$@"; do
    case "$a" in
        --check)      CHECK_ONLY=1 ;;
        --no-web)     NO_WEB=1 ;;
        --no-reap)    REAP_UNTRACKED=0 ;;
        *)            PASSTHRU+=("$a") ;;
    esac
done

m4_parse_cli "$DEMO_NAME" ${PASSTHRU[@]+"${PASSTHRU[@]}"}
m4_setup_env

SEG_ENGINE="$M4_CODE_ROOT/models/m4/segmentation/engines/segformer_b0_fp16.engine"
SEG_LABELS="$M4_CODE_ROOT/models/m4/segmentation/labels/labels.json"
YOLO_ENGINE="$M4_CODE_ROOT/models/m4/detection/engines/yolo11n_fp16.engine"

# Resolve segmentation enablement: auto = only when the engine AND labels
# exist. TensorRT cannot be asked to "partially" load, so a missing engine
# must disable the module rather than abort the whole bring-up.
SEG_REASON=""
case "$ENABLE_SEGMENTATION" in
    auto)
        if [ -f "$SEG_ENGINE" ] && [ -f "$SEG_LABELS" ]; then
            ENABLE_SEGMENTATION=1
        else
            ENABLE_SEGMENTATION=0
            [ -f "$SEG_ENGINE" ] || SEG_REASON="engine missing: $SEG_ENGINE"
            [ -f "$SEG_LABELS" ] || SEG_REASON="${SEG_REASON:+$SEG_REASON; }labels missing: $SEG_LABELS"
        fi
        ;;
    0|1) ;;
    *) m4_die "ENABLE_SEGMENTATION must be auto|0|1 (got: $ENABLE_SEGMENTATION)" ;;
esac

# ---- preflight -----------------------------------------------------------
m4_section "Preflight"

CAMERA_PUBLISHER="$M4_WS_ROOT/install/m4_demo_bringup/lib/m4_demo_bringup/csi_camera_publisher"
M4_PREFLIGHT_PATHS="$YOLO_ENGINE \
  $M4_WS_ROOT/install/bev_detection/lib/bev_detection/yolo_trt_node \
  $CAMERA_PUBLISHER \
  $M4_WS_ROOT/install/m4_demo_bringup/lib/m4_demo_bringup/m4_web_demo_server"
m4_preflight

# The hub launch file must be installed; a stale build is a common footgun.
HUB_LAUNCH="$M4_WS_ROOT/install/m4_demo_bringup/share/m4_demo_bringup/launch/m4_all_demo.launch.py"
if [ ! -f "$HUB_LAUNCH" ]; then
    m4_fail "missing installed launch: $HUB_LAUNCH"
    m4_fail "  rebuild first:  cd $M4_WS_ROOT && colcon build --packages-select m4_demo_bringup --symlink-install"
    m4_die "preflight failed"
fi
m4_ok "hub launch present"

if [ "$ENABLE_TRACKING" = "1" ]; then
    for p in "$M4_WS_ROOT/install/bev_tracking/lib/bev_tracking/tracking_node" \
             "$M4_WS_ROOT/install/bev_tracking/lib/bev_tracking/tracking_visualizer"; do
        [ -e "$p" ] || m4_die "ENABLE_TRACKING=1 but missing: $p"
    done
    m4_ok "tracking artifacts present"
fi

if [ "$ENABLE_SEGMENTATION" = "1" ]; then
    for p in "$SEG_ENGINE" \
             "$M4_WS_ROOT/install/bev_segmentation/lib/bev_segmentation/segmentation_node"; do
        [ -e "$p" ] || m4_die "ENABLE_SEGMENTATION=1 but missing: $p"
    done
    m4_ok "segmentation artifacts present"
else
    m4_warn "M4.3 segmentation DISABLED for this run"
    [ -n "$SEG_REASON" ] && m4_warn "  reason: $SEG_REASON"
    m4_warn "  build it with: scripts/m4/export_segformer.sh && scripts/m4/build_segformer_engine.sh && scripts/m4/generate_labels_json.sh"
    m4_warn "  the 4.3 tab will show as not-ready; 4.1/4.2 are unaffected"
fi

# Read-only camera/environment report (also the whole of --check).
m4_section "Environment"
m4_note "camera source=$CAMERA_SOURCE device=$CAMERA_DEVICE native=${CAMERA_CAPTURE_WIDTH}x${CAMERA_CAPTURE_HEIGHT} published=${CAMERA_WIDTH}x${CAMERA_HEIGHT}@${CAMERA_FPS}"
if [ -e "$CAMERA_DEVICE" ]; then
    local_fmt=$(v4l2-ctl -d "$CAMERA_DEVICE" --get-fmt-video 2>/dev/null | tr -d '\n' || true)
    m4_note "current v4l2 fmt: ${local_fmt:-unavailable}"
else
    m4_warn "$CAMERA_DEVICE not present"
fi
holders=$(lsof -t "$CAMERA_DEVICE" 2>/dev/null || true)
if [ -n "$holders" ]; then
    m4_warn "$CAMERA_DEVICE currently held by PIDs: $holders"
    for hp in $holders; do
        hcmd=$(tr '\0' ' ' < "/proc/$hp/cmdline" 2>/dev/null || echo "?")
        m4_warn "    pid $hp: ${hcmd:0:120}"
    done
else
    m4_ok "$CAMERA_DEVICE is free"
fi
if pgrep -x camera_driver >/dev/null 2>&1; then
    m4_warn "camera_driver is running and will own the cameras; use CAMERA_SOURCE=existing"
fi
m4_note "web: port=$M4_WEB_PORT host=$M4_WEB_HOST backend=$M4_WEB_BACKEND"

if [ "$CHECK_ONLY" = "1" ]; then
    m4_section "--check complete (no side effects)"
    m4_note "tracking=$( [ "$ENABLE_TRACKING" = 1 ] && echo enabled || echo disabled ) segmentation=$( [ "$ENABLE_SEGMENTATION" = 1 ] && echo enabled || echo disabled )"
    exit 0
fi

# ---- plan ----------------------------------------------------------------
TOPICS_SPEC="m4_1=/perception/demo/m4_1"
UNAVAILABLE_SPEC=""
[ "$ENABLE_TRACKING" = "1" ] && TOPICS_SPEC="$TOPICS_SPEC,m4_2=/perception/demo/m4_2"
if [ "$ENABLE_SEGMENTATION" = "1" ]; then
    TOPICS_SPEC="$TOPICS_SPEC,m4_3=/perception/demo/m4_3"
else
    UNAVAILABLE_SPEC="m4_3=${SEG_REASON:-segmentation disabled by ENABLE_SEGMENTATION=0}"
fi

if [ "$M4_DRY_RUN" = "1" ]; then
    cat <<EOF
[DRY-RUN] $DEMO_NAME would:
  1. reap a leaked camera publisher from a previous run (ownership-checked)
  2. resolve camera source: $CAMERA_SOURCE
  3. spawn ONE camera publisher ($CAMERA_WIDTH x $CAMERA_HEIGHT @ $CAMERA_FPS) if unresolved
  4. start ONE hub web server with a single-module runtime manager
  5. initially launch $M4_WEB_ACTIVE only; clicking another chapter stops it
       and starts only that lesson (4.2 includes its YOLO dependency)
  6. print URL http://<jetson-ip>:$M4_WEB_PORT/m4/1
  7. Ctrl-C → SIGINT camera/web/module PGIDs, 5s grace, SIGKILL survivors

[DRY-RUN] topics: $TOPICS_SPEC
[DRY-RUN] tracking=$ENABLE_TRACKING segmentation=$ENABLE_SEGMENTATION
EOF
    exit 0
fi

m4_log_open
m4_install_traps

# ---- reap any leaked camera publisher from a previous run ----------------
m4_reap_leaked_camera_publisher

# ---- clear untracked camera publishers from THIS repo --------------------
# A publisher leaked by an older run (before ownership was recorded) keeps
# /dev/video0 busy, so every later run fails with "device busy". The match is
# deliberately narrow: the executable name AND this repo path must both appear
# in /proc/<pid>/cmdline. Never a blanket pkill; disable with --no-reap.
reap_untracked_publishers() {
    [ "$REAP_UNTRACKED" = "1" ] || { m4_note "untracked-publisher reaping disabled (--no-reap)"; return 0; }
    local pids p cmd up
    pids=$(pgrep -f "csi_camera_publisher" 2>/dev/null || true)
    for p in $pids; do
        [ "$p" = "${M4_CAMERA_PID:-}" ] && continue
        grep -qa "$REPO" "/proc/$p/cmdline" 2>/dev/null || continue
        cmd=$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null || echo "?")
        up=$(ps -o etime= -p "$p" 2>/dev/null | tr -d ' ')
        m4_warn "untracked camera publisher left by an earlier run: pid=$p uptime=${up:-?}"
        m4_warn "    ${cmd:0:140}"
        kill -TERM "$p" 2>/dev/null || true
    done
    sleep 2
    for p in $(pgrep -f "csi_camera_publisher" 2>/dev/null || true); do
        [ "$p" = "${M4_CAMERA_PID:-}" ] && continue
        grep -qa "$REPO" "/proc/$p/cmdline" 2>/dev/null || continue
        m4_warn "SIGKILL stubborn untracked publisher pid=$p"
        kill -9 "$p" 2>/dev/null || true
    done
    m4_ok "untracked publisher sweep done"
}

reap_untracked_publishers
# ---- camera resolution ---------------------------------------------------
m4_camera_probe_and_choose

# Guard: never let both an external publisher and our own publisher run.
if [ "$M4_CAMERA_OWNED" = "true" ] && [ "$M4_CAMERA_MODE" != "gmsl" ] \
   && [ "$M4_CAMERA_MODE" != "test" ]; then
    holders=$(lsof -t "$M4_CAMERA_DEVICE" 2>/dev/null || true)
    if [ -n "$holders" ]; then
        m4_fail "$M4_CAMERA_DEVICE is held by PIDs: $holders"
        for hp in $holders; do
            m4_fail "    pid $hp: $(tr '\0' ' ' < "/proc/$hp/cmdline" 2>/dev/null | cut -c1-140)"
        done
        m4_fail "  options:"
        m4_fail "    * stop it manually:            kill -INT <pid>"
        m4_fail "    * consume its topic instead:   CAMERA_SOURCE=existing"
        m4_fail "    * publishers leaked by older runs of this repo are reaped"
        m4_fail "      automatically unless --no-reap was given"
        m4_die "camera is busy"
    fi
fi

# ---- algorithm duplicate probe ------------------------------------------
m4_algorithm_duplicate_probe "yolo_trt_node,tracking_node,segmentation_node" \
    /perception/detections /perception/tracks /perception/semantic_mask

# ---- start exactly ONE camera publisher ----------------------------------
if [ "$M4_CAMERA_OWNED" = "true" ] && [ "$M4_CAMERA_MODE" != "gmsl" ]; then
    m4_section "Camera ($M4_CAMERA_MODE) starting"
    case "$M4_CAMERA_MODE" in
        csi|v4l2) m4_spawn_camera_publisher "csi" ;;
        usb)      m4_spawn_camera_publisher "usb" ;;
        test)     m4_spawn_camera_publisher "test" ;;
        *)        m4_die "unexpected camera mode: $M4_CAMERA_MODE" ;;
    esac
    sleep 1
fi

# ---- the single unified web server --------------------------------------
if [ "$NO_WEB" = "1" ]; then
    m4_note "web hub disabled by --no-web"
else
    m4_section "Web hub (one server, one selected inference module)"
    if m4_launch_web_hub "$TOPICS_SPEC" "$UNAVAILABLE_SPEC"; then
        if [ "$M4_WEB_PID" != "0" ]; then
            if ! m4_report_hub_url; then
                m4_die "web hub health gate failed; refusing to continue with an unavailable preview"
            fi
        fi
    else
        m4_die "web hub failed to start"
    fi
fi

# ---- staged readiness gate ----------------------------------------------
m4_section "Readiness gate"

m4_wait_for_stage "/perception/cameras/front/image" 10.0 "camera front image" || true
case "$M4_WEB_ACTIVE" in
    m4_1) m4_wait_for_stage "/perception/demo/m4_1" 15.0 "initial 4.1 overlay" || true ;;
    m4_2) m4_wait_for_stage "/perception/demo/m4_2" 15.0 "initial 4.2 overlay" || true ;;
    m4_3) m4_wait_for_stage "/perception/demo/m4_3" 20.0 "initial 4.3 overlay" || true ;;
    *) m4_die "M4_WEB_ACTIVE must be m4_1, m4_2 or m4_3 (got: $M4_WEB_ACTIVE)" ;;
esac

# ---- status + wait -------------------------------------------------------
m4_start_status_reporter
m4_note "M4 unified preview is live. Press Ctrl-C to stop."
wait "$M4_WEB_PID" 2>/dev/null
rc=$?
[ $rc -ne 0 ] && m4_warn "web hub process exited (rc=$rc)"
m4_note "web hub process exited (rc=$rc); cleanup follows via EXIT trap"
exit 0
