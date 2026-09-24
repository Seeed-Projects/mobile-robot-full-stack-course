#!/usr/bin/env bash
# scripts/m4/run_m4_3_demo.sh — one-click student entry for M4.3 (semantic seg).
#
# Pipeline (CAMERA_SOURCE=gmsl — course hardware):
#   nvargus-daemon (root, external)
#     → /camera/front/image_raw/compressed
#   camera_sync_node (may already be running externally)
#     → /bev/frameset
#   camera_adapter_node  ← started by this demo when CAMERA_SOURCE=gmsl
#     → /perception/cameras/front/image
#   segmentation_node
#     → /perception/semantic_mask
#     → /perception/drivable_mask
#   segmentation_visualizer
#     → /perception/demo/m4_3  (side-by-side bgr8)

set -u

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib"
# shellcheck disable=SC1091
source "$LIB_DIR/m4_demo_lib.sh"

DEMO_NAME="4.3"
m4_lib_init "$DEMO_NAME"

# ---- CLI / env overrides ----
: "${CAMERA_SOURCE:=auto}"
: "${CAMERA_DEVICE:=/dev/video0}"
: "${CAMERA_WIDTH:=1280}"
: "${CAMERA_HEIGHT:=720}"
: "${CAMERA_FPS:=15}"
: "${VIEWER:=auto}"
: "${DURATION:=0}"

export CAMERA_SOURCE CAMERA_DEVICE CAMERA_WIDTH CAMERA_HEIGHT CAMERA_FPS VIEWER DURATION
m4_parse_cli "$DEMO_NAME" "$@"
m4_setup_env

m4_log_open

# ---- preflight ----
M4_PREFLIGHT_PATHS="$REPO/models/m4/segmentation/engines/segformer_b0_fp16.engine \
  $REPO/ros2_ws/install/bev_segmentation/lib/bev_segmentation/segmentation_node \
  $REPO/ros2_ws/install/bev_detection/lib/bev_detection/camera_adapter_node"
m4_preflight
m4_section "Preflight OK"

# Resolve camera source early so dry-run can report the resolved mode.
m4_camera_probe_and_choose

if [ "$M4_DRY_RUN" = "1" ]; then
    cat <<EOF
[DRY-RUN] $DEMO_NAME would:
  1. camera source: ${M4_CAMERA_SOURCE:-auto} → owned=$M4_CAMERA_OWNED mode=$M4_CAMERA_MODE
  2. (if owned && mode=csi/usb/test) spawn csi_camera_publisher
  3. (if owned && mode=gmsl) launch includes camera_adapter_node
  4. duplicate-probe: /perception/semantic_mask, /perception/drivable_mask
  5. spawn via setsid: ros2 launch m4_demo_bringup m4_3_demo.launch.py
  6. staged readiness gate (gmsl: FrameSet→image→semantic→drivable→overlay)
  7. spawn viewer ($VIEWER) on /perception/demo/m4_3
  8. start status reporter; wait for Ctrl-C
  9. on EXIT/INT/TERM: SIGINT to launch PGID, wait 5s, SIGKILL

[DRY-RUN] Camera source: $M4_CAMERA_SOURCE  resolved: $M4_CAMERA_MODE
[DRY-RUN] Camera owned: $M4_CAMERA_OWNED
[DRY-RUN] Viewer: $VIEWER  DURATION=$DURATION
EOF
    exit 0
fi

m4_install_traps
m4_algorithm_duplicate_probe "segmentation_node" \
    /perception/semantic_mask /perception/drivable_mask

# ---- camera bringup (non-gmsl owned modes only) ----
if [ "$M4_CAMERA_OWNED" = "true" ] && [ "$M4_CAMERA_MODE" != "gmsl" ]; then
    m4_section "Camera ($M4_CAMERA_MODE) starting"
    case "$M4_CAMERA_MODE" in
        csi|usb)
            setsid python3 "$REPO/ros2_ws/src/bev_detection/test/csi_camera_publisher.py" \
                --source "$M4_CAMERA_MODE" \
                --device "$M4_CAMERA_DEVICE" \
                --width  "$M4_CAMERA_WIDTH" \
                --height "$M4_CAMERA_HEIGHT" \
                --fps    "$M4_CAMERA_FPS" \
                --timeout 0 \
                --topic /perception/cameras/front/image \
                > "$M4_LOG_DIR/camera.log" 2>&1 &
            ;;
        test)
            setsid python3 "$REPO/ros2_ws/src/bev_detection/test/csi_camera_publisher.py" \
                --source test \
                --width  "$M4_CAMERA_WIDTH" \
                --height "$M4_CAMERA_HEIGHT" \
                --fps    "$M4_CAMERA_FPS" \
                --timeout 0 \
                --topic /perception/cameras/front/image \
                > "$M4_LOG_DIR/camera.log" 2>&1 &
            ;;
    esac
    sleep 1
    m4_note "camera publisher spawned (see $M4_LOG_DIR/camera.log)"
fi

# ---- launch the demo ----
m4_section "Launching m4_3_demo.launch.py (camera_mode=$M4_CAMERA_MODE)"
m4_launch_ros ros2 launch m4_demo_bringup m4_3_demo.launch.py \
    camera_source:="$M4_CAMERA_MODE"
m4_record_meta

# ---- staged readiness gate ----
m4_section "Readiness gate"

# Stage 1: GMSL→FrameSet (only for gmsl mode)
if [ "$M4_CAMERA_MODE" = "gmsl" ]; then
    m4_wait_for_stage "/bev/frameset"              5.0 "GMSL→FrameSet"      || true
fi

# Stage 2: camera front image
m4_wait_for_stage "/perception/cameras/front/image"  5.0 "camera front image" || true

# Stage 3: semantic_mask
m4_wait_for_stage "/perception/semantic_mask"       8.0 "semantic_mask"      || true

# Stage 4: drivable_mask
m4_wait_for_stage "/perception/drivable_mask"       5.0 "drivable_mask"      || true

# Stage 5: demo overlay
m4_wait_for_stage "/perception/demo/m4_3"          5.0 "demo overlay"       || true

# ---- optional viewer ----
# M4 web preview replaces the legacy local GUI. The bash supervisor
# owns the web server in a separate PGID via m4_launch_web_server.
# --no-gui / --viewer=none still suppress it (matches old behaviour).
if [ "$M4_NO_GUI" = "1" ] || [ "$M4_VIEWER" = "none" ]; then
    m4_note "viewer (local GUI): disabled (--no-gui or --viewer=none)"
    m4_note "web server: disabled (same flag)"
else
    m4_section "Web server"
    m4_launch_web_server "m4_3" "3" "/perception/demo/m4_3" || \
        m4_warn "web server failed to start; continuing without preview"
    if [ "$M4_WEB_PID" != "0" ]; then
        m4_report_web_url "3" || m4_warn "web server health gate not satisfied"
    fi
fi

# ---- status reporter ----
m4_start_status_reporter

m4_note "demo is live. Press Ctrl-C to stop."
wait "$M4_LAUNCH_PID" 2>/dev/null
rc=$?
if [ $rc -ne 0 ]; then
    m4_report_launch_failure
fi
m4_note "launch process exited (rc=$rc); cleanup follows via EXIT trap"
exit 0
