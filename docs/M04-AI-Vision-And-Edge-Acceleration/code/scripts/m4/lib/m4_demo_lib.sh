#!/usr/bin/env bash
# scripts/m4/lib/m4_demo_lib.sh
#
# M4 Demo Integration shared bash helper.
#
# PURPOSE
#   Orchestrate camera ownership, algorithm-duplicate probing, PID/PGID
#   ownership metadata, viewer lifecycle, and graceful shutdown for the
#   three M4 demos (m4_1, m4_2, m4_3). This file is shared between
#   scripts/m4/run_m4_*_demo.sh. It MUST stay orchestration-only:
#   no YOLO / ByteTrack / SegFormer logic, no TensorRT calls.
#
# CONVENTIONS
#   * Source-only library; never executed directly.
#   * Public entry points are prefixed `m4_`.
#   * State is held in globals; the only writes that survive are the
#     files under /tmp/m4_demo/ (one per demo).
#
# PUBLIC ENTRY POINTS
#   m4_lib_init <demo_name>     -- source-time bookkeeping, sets globals
#   m4_setup_env                -- source ROS + add LD_LIBRARY_PATH
#   m4_parse_cli <demo_name>    -- populate NO_GUI / VIEWER / DRY_RUN / DURATION
#   m4_log_open <demo_name>     -- open teed log files; writes /tmp/m4_demo/init
#   m4_preflight                -- check artifacts exist + workspace built
#   m4_camera_probe_and_choose  -- decide camera_owned_by_demo + log it
#   m4_algorithm_duplicate_probe -- fail-fast if pre-existing inference
#   m4_record_meta              -- write /tmp/m4_demo/<demo>.metadata.json
#   m4_launch_ros <args...>     -- spawn ros2 launch via setsid, record PID/PGID/SID
#   m4_wait_for_first_message <topic> <timeout_s> -- readiness helper
#   m4_launch_viewer <topic>    -- spawn rqt or cv viewer
#   m4_start_status_reporter    -- 5s periodic status loop
#   m4_cleanup <reason>         -- graceful shutdown, removes metadata file
#
# See scripts/m4/README for usage.
#
# DO NOT add algorithm logic here. If you need to, you are in the wrong
# file: it belongs in m4_demo_bringup/.

# Guard against double-sourcing.
if [ -n "${__M4_DEMO_LIB_SOURCED:-}" ]; then
    return 0 2>/dev/null || true
fi
__M4_DEMO_LIB_SOURCED=1

set -u
# We deliberately do NOT enable 'set -e' or 'pipefail'.
#   - signal traps must run regardless of intermediate failures
#   - grep finding 0 lines is normal for empty logs

# ---- Paths --------------------------------------------------------------
M4_LIB_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
M4_LIB_REPO_DEFAULT="$(cd "$M4_LIB_SCRIPT_DIR/../../.." && pwd)"
# When sourced via scripts/m4/lib/m4_demo_lib.sh the depth is 3 (lib/m4/scripts)
# When sourced directly for testing (./m4_demo_lib.sh) BASH_SOURCE collapses to ".".
if [ "$M4_LIB_REPO_DEFAULT" = "$M4_LIB_SCRIPT_DIR" ] || [ "$M4_LIB_REPO_DEFAULT" = "." ]; then
    M4_LIB_REPO_DEFAULT="${REPO:-$(pwd)}"
fi
: "${REPO:=$M4_LIB_REPO_DEFAULT}"

M4_DEMO_STATE_DIR="${M4_DEMO_STATE_DIR:-/tmp/m4_demo}"
mkdir -p "$M4_DEMO_STATE_DIR"

# ---- Colors -------------------------------------------------------------
if [ -t 1 ]; then
    GREEN=$'\e[0;32m'; YELLOW=$'\e[1;33m'; RED=$'\e[0;31m'
    CYAN=$'\e[0;36m'; DIM=$'\e[2m'; NC=$'\e[0m'
else
    GREEN=''; YELLOW=''; RED=''; CYAN=''; DIM=''; NC=''
fi
m4_section() { echo "${YELLOW}== $* ==${NC}"; }
m4_ok()      { echo "${GREEN}  [OK] $*${NC}"; }
m4_fail()    { echo "${RED}  [FAIL] $*${NC}"; }
m4_note()    { echo "${CYAN}  [INFO] $*${NC}"; }
m4_warn()    { echo "${YELLOW}  [WARN] $*${NC}"; }
m4_stage()   { echo "${CYAN}  [STAGE] $*${NC}"; }

m4_die() {
    # Print and exit non-zero from the running script.
    m4_fail "$*"
    exit "${M4_DIE_RC:-1}"
}

# ---- Library state (per-source) ----------------------------------------
M4_DEMO_NAME=""
M4_NO_GUI=0
M4_VIEWER="auto"            # auto | rqt | cv | none
M4_DRY_RUN=0
M4_DURATION=0               # 0 = forever
M4_CAMERA_SOURCE="auto"
M4_CAMERA_DEVICE="/dev/video0"
M4_CAMERA_WIDTH="1280"
M4_CAMERA_HEIGHT="720"
M4_CAMERA_FPS="15"
M4_CAMERA_OWNED=false
M4_CAMERA_MODE=""           # resolved: existing | csi | gmsl | usb | test
M4_LOG_DIR=""
M4_LAUNCH_PID=""
M4_LAUNCH_PGID=""
M4_LAUNCH_SID=""
M4_LAUNCH_START_TICK=""
M4_VIEWER_PID=""
M4_VIEWER_PGID=""
M4_WEB_PID=""
M4_WEB_PGID=""
M4_WEB_META=""
# Camera publisher ownership. The publisher is spawned with `setsid` so it
# survives a naive shell teardown; without its own recorded PID/PGID it was
# never reaped, and a leaked publisher kept /dev/video0 busy for 16h+ (which
# then made every later demo fail with "device busy").
M4_CAMERA_PID=""
M4_CAMERA_PGID=""
M4_CAMERA_TICK=""
M4_CAMERA_META=""
M4_STATUS_PID=""
M4_CLEANED_UP=0
M4_ALGO_PROBE_DONE=0
M4_FIRST_READY=0
M4_TOPICS_AT_READY=""

m4_lib_init() {
    M4_DEMO_NAME="$1"
    [ -n "$M4_DEMO_NAME" ] || m4_die "m4_lib_init: demo name required"

    # Cleanup any stale metadata for THIS demo (previously crashed run).
    local stale="$M4_DEMO_STATE_DIR/$M4_DEMO_NAME.metadata.json"
    if [ -f "$stale" ]; then
        local pid tick
        pid=$(python3 -c "import json,sys; print(json.load(open('$stale'))['launcher_pid'])" 2>/dev/null || echo "")
        if [ -n "$pid" ] && [ -d "/proc/$pid" ]; then
            tick=$(awk '{print $22}' "/proc/$pid/stat" 2>/dev/null || echo "")
            local stored
            stored=$(python3 -c "import json,sys; print(json.load(open('$stale'))['started_at_tick'])" 2>/dev/null || echo "")
            if [ -n "$tick" ] && [ "$tick" = "$stored" ]; then
                m4_warn "previous $M4_DEMO_NAME run (pid=$pid, tick=$tick) is still alive; cleaning it up first"
                m4_cleanup "stale-prev-run"
            else
                m4_warn "stale metadata for $M4_DEMO_NAME (pid=$pid reused or dead); ignoring"
            fi
        fi
        rm -f "$stale" "$M4_DEMO_STATE_DIR/$M4_DEMO_NAME.init"
    fi
}

m4_setup_env() {
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash 2>/dev/null || m4_die "ROS humble not found"
    cd "$REPO/ros2_ws"
    # shellcheck disable=SC1091
    source install/setup.bash 2>/dev/null || m4_die "ros2_ws not built; run colcon build first"
    set -u
    m4_ok "ros2 + workspace sourced"

    # If the caller exported CAMERA_SOURCE, propagate into our global state
    # BEFORE m4_camera_probe_and_choose runs.
    M4_CAMERA_SOURCE="${CAMERA_SOURCE:-$M4_CAMERA_SOURCE}"
    M4_CAMERA_DEVICE="${CAMERA_DEVICE:-$M4_CAMERA_DEVICE}"
    M4_CAMERA_WIDTH="${CAMERA_WIDTH:-$M4_CAMERA_WIDTH}"
    M4_CAMERA_HEIGHT="${CAMERA_HEIGHT:-$M4_CAMERA_HEIGHT}"
    M4_CAMERA_FPS="${CAMERA_FPS:-$M4_CAMERA_FPS}"

    # Ensure m4_demo_bringup module is importable. ament_python install
    # only registers compiled packages; for our source-only package we
    # prepend the source dir to PYTHONPATH (re-source install/setup.bash
    # last so its own path takes precedence for other packages).
    set +u
    export PYTHONPATH="$REPO/ros2_ws/src/m4_demo_bringup:${PYTHONPATH:-}"
    cd "$REPO/ros2_ws"
    # shellcheck disable=SC1091
    source install/setup.bash 2>/dev/null || m4_die "ros2_ws not built; run colcon build first"
    # Re-prepend in case the workspace's setup.bash overwrote it.
    export PYTHONPATH="$REPO/ros2_ws/src/m4_demo_bringup:${PYTHONPATH:-}"
    set -u

    # Same LD_LIBRARY_PATH as the upstream benchmark / visualize scripts.
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$REPO/ros2_ws/install/bev_detection/lib:/home/seeed/src/opencv-4.14.0-cuda-build/lib:/usr/local/cuda-12.6/lib64:/usr/lib/aarch64-linux-gnu"
    m4_ok "LD_LIBRARY_PATH configured"

    # Honour DISPLAY if the user has set it; never force a default.
    if [ -n "${DISPLAY:-}" ]; then
        m4_ok "DISPLAY=${DISPLAY} (forwarded from caller)"
    else
        m4_note "DISPLAY not set — viewer will run headless; can still receive data"
    fi
}

m4_parse_cli() {
    local demo="$1"
    shift
    for a in "$@"; do
        case "$a" in
            --no-gui)          M4_NO_GUI=1 ;;
            --viewer=*)        M4_VIEWER="${a#--viewer=}" ;;
            --viewer)          shift; M4_VIEWER="${1:-auto}" ;;
            --duration=*)      M4_DURATION="${a#--duration=}" ;;
            --duration)        shift; M4_DURATION="${1:-0}" ;;
            --dry-run)         M4_DRY_RUN=1 ;;
            --help|-h)
                sed -n '2,30p' "${BASH_SOURCE[0]}" 2>/dev/null
                m4_print_usage "$demo"
                exit 0
                ;;
            *) m4_die "unknown option: $a (try --help)" ;;
        esac
    done

    case "$M4_VIEWER" in
        auto|rqt|cv|none) ;;
        *) m4_die "--viewer must be one of: auto|rqt|cv|none (got: $M4_VIEWER)" ;;
    esac

    case "$M4_CAMERA_SOURCE" in
        existing|csi|v4l2|gmsl|usb|test|auto) ;;
        *) m4_die "CAMERA_SOURCE must be one of: existing|csi|v4l2|gmsl|usb|test|auto (got: $M4_CAMERA_SOURCE)" ;;
    esac

    [ "$M4_NO_GUI" = "1" ] && M4_VIEWER="none"
}

m4_print_usage() {
    cat <<EOF

Usage:
    scripts/m4/run_$1_demo.sh [options]

Options:
    --no-gui                force headless (overrides --viewer)
    --viewer=auto|rqt|cv|none   viewer selection (default: auto)
    --duration=SECS         auto-stop after SECS seconds (0=forever, default: 0)
    --dry-run               print what would be done, exit before launching
    --help|-h               this help

Environment overrides:
    CAMERA_SOURCE={existing|csi|v4l2|gmsl|usb|test|auto}   default: csi
    CAMERA_DEVICE=/dev/videoN                         default: /dev/video0 (only used when CAMERA_SOURCE=csi/usb)
    CAMERA_WIDTH=int, CAMERA_HEIGHT=int                default: 1920x1536 (GMSL native)
    CAMERA_FPS=int                                    default: 30
    VIEWER=auto|rqt|cv|none                           default: auto
    DURATION=int                                      default: 0
    DISPLAY                                           forwarded, never overwritten

EOF
}

m4_log_open() {
    M4_LOG_DIR="$REPO/output/m4/$M4_DEMO_NAME/demo_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$M4_LOG_DIR"
    m4_ok "log dir: $M4_LOG_DIR"

    # Remember we initiated; subsequent runs of same demo can detect leftovers.
    date -Iseconds > "$M4_DEMO_STATE_DIR/$M4_DEMO_NAME.init"
    echo "$M4_LOG_DIR" >> "$M4_DEMO_STATE_DIR/$M4_DEMO_NAME.init"
}

m4_preflight() {
    # Each demo provides a list of "engine/bin" path pairs in M4_PREFLIGHT_PATHS (set by caller).
    local miss=0
    if [ -z "${M4_PREFLIGHT_PATHS:-}" ]; then
        return 0
    fi
    for path in $M4_PREFLIGHT_PATHS; do
        if [ ! -e "$path" ]; then
            m4_fail "missing: $path"
            miss=1
        fi
    done
    [ "$miss" = "1" ] && m4_die "preflight failed; aborting"
}

# ---- camera_probe_and_choose -------------------------------------------
#
# Jetson GMSL topology (confirmed by runtime audit):
#
#   nvargus-daemon (root, PID 1265)  ← external GMSL driver (NOT owned by demo)
#     → publishes /camera/front/image_raw/compressed  (CompressedImage, hw-synced)
#   camera_sync_node (PID 71829)       ← may already be running externally
#     → subscribes /camera/front/image_raw/compressed + 5 others
#     → publishes /bev/frameset  (bev_interfaces/FrameSet, Reliable QoS)
#   camera_adapter_node                ← NOT running by default
#     → subscribes /bev/frameset
#     → publishes /perception/cameras/front/image  (sensor_msgs/Image)
#
# Decision matrix:
#   CAMERA_SOURCE=existing  → must see /perception/cameras/front/image alive
#   CAMERA_SOURCE=gmsl     → /perception/cameras/front/image is dead but the
#                             full GMSL chain exists; the demo starts
#                             camera_adapter_node to bridge /bev/frameset → /image.
#                             The external nvargus-daemon + camera_sync_node are
#                             NOT owned by the demo.
#   CAMERA_SOURCE=csi       → spawn csi_camera_publisher.py via v4l2src
#   CAMERA_SOURCE=usb       → spawn csi_camera_publisher.py on configured device
#   CAMERA_SOURCE=test      → spawn csi_camera_publisher.py --source test
#   CAMERA_SOURCE=auto      → Step1: probe /image; Step2: probe /bev/frameset;
#                             Step3: if neither → FAIL FAST (no silent CSI fallback)
#
m4_camera_probe_and_choose() {
    local image_topic="${M4_IMAGE_TOPIC:-/perception/cameras/front/image}"
    local frameset_topic="/bev/frameset"
    local source="$M4_CAMERA_SOURCE"

    if [ "$source" = "auto" ]; then
        # Step 1: /perception/cameras/front/image is alive → external publisher
        if m4_wait_for_first_message "$image_topic" 2.0 >/dev/null 2>&1; then
            source="existing"
            m4_note "auto: /perception/cameras/front/image is live → existing"
        # Step 2: /bev/frameset is alive → GMSL chain running, need adapter
        elif m4_wait_for_first_message "$frameset_topic" 2.0 >/dev/null 2>&1; then
            source="gmsl"
            m4_note "auto: /bev/frameset is live → gmsl (will start camera_adapter_node)"
        # Step 3: V4L2 device exists and produces frames → use csi mode
        elif [ -e "$M4_CAMERA_DEVICE" ]; then
            source="csi"
            m4_note "auto: V4L2 device $M4_CAMERA_DEVICE found → csi (GMSL via v4l2src)"
        # Step 4: neither → FAIL FAST
        else
            m4_fail "auto: cannot determine camera source."
            m4_fail "  /perception/cameras/front/image  — no live publisher"
            m4_fail "  /bev/frameset                  — no live publisher"
            m4_fail "  $M4_CAMERA_DEVICE            — device not found"
            m4_fail "  Available sources: CAMERA_SOURCE={csi|existing|gmsl|test}"
            m4_fail "  If running on the Jetson with GMSL cameras connected,"
            m4_fail "  use:  CAMERA_SOURCE=csi (or v4l2)"
            m4_die "auto camera resolution failed"
        fi
    fi

    case "$source" in
        existing)
            M4_CAMERA_OWNED=false
            M4_CAMERA_MODE="existing"
            m4_ok "[M4 Demo] Camera source: existing ROS publisher"
            m4_ok "[M4 Demo] Camera ownership: external"
            ;;
        gmsl)
            # The demo owns camera_adapter_node (bridge /bev/frameset → /image).
            # nvargus-daemon + camera_sync_node are external; we do NOT touch them.
            M4_CAMERA_OWNED=true          # we own camera_adapter_node
            M4_CAMERA_MODE="gmsl"
            m4_ok "[M4 Demo] Camera source: gmsl"
            m4_ok "[M4 Demo]   GMSL driver (nvargus-daemon): external, NOT owned by demo"
            m4_ok "[M4 Demo]   camera_sync_node: external (may already be running)"
            m4_ok "[M4 Demo]   camera_adapter_node: owned by demo"
            ;;
        csi|v4l2)
            [ -e "$M4_CAMERA_DEVICE" ] || m4_die "CAMERA_SOURCE=csi/v4l2 but $M4_CAMERA_DEVICE does not exist"
            M4_CAMERA_OWNED=true
            M4_CAMERA_MODE="csi"
            m4_ok "[M4 Demo] Camera source: csi/v4l2 (device=$M4_CAMERA_DEVICE)"
            m4_ok "[M4 Demo] Camera ownership: demo"
            ;;
        usb)
            [ -e "$M4_CAMERA_DEVICE" ] || m4_die "CAMERA_SOURCE=usb but $M4_CAMERA_DEVICE does not exist"
            M4_CAMERA_OWNED=true
            M4_CAMERA_MODE="usb"
            m4_ok "[M4 Demo] Camera source: usb (device=$M4_CAMERA_DEVICE)"
            m4_ok "[M4 Demo] Camera ownership: demo"
            ;;
        test)
            M4_CAMERA_OWNED=true
            M4_CAMERA_MODE="test"
            m4_ok "[M4 Demo] Camera source: test (videotestsrc synthetic)"
            m4_ok "[M4 Demo] Camera ownership: demo"
            ;;
    esac
}

# ---- algorithm duplicate probe -----------------------------------------
#
# m4_algorithm_duplicate_probe <topic1> [topic2] [topic3] ...
#
# For each topic: subscribe via m4_wait_for_first_message (bounded). If
# message arrives, resolve the publisher with `ros2 node info <topic>`
# and check the executable name. Fail fast if it matches an algorithm
# executable that THIS demo owns.
m4_algorithm_duplicate_probe() {
    local owned_executables_csv="$1"   # comma-separated, e.g. "yolo_trt_node,tracking_node"
    shift
    local topics=("$@")

    if [ "${M4_ALGO_PROBE_DONE}" = "1" ]; then
        return 0
    fi
    M4_ALGO_PROBE_DONE=1

    local owned_csv="${owned_executables_csv//./_}"  # sanitise
    for topic in "${topics[@]}"; do
        local msg_file
        msg_file="$(mktemp)"
        if m4_wait_for_first_message "$topic" 1.5 >/dev/null 2>"$msg_file"; then
            # Topic is alive. Get the publisher node(s).
            local pubs
            pubs=$(ros2 topic info "$topic" 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')
            if [ -z "$pubs" ] || [ "$pubs" = "0" ]; then
                m4_note "duplicate probe: $topic has 0 publishers (stale message?) — skipping"
                rm -f "$msg_file"; continue
            fi
            local nodes
            nodes=$(ros2 node list 2>/dev/null)
            local matched_node=""
            for n in $nodes; do
                local exec_name
                exec_name=$(ros2 node info "$n" 2>/dev/null | awk '/executable name:/{print $NF; exit}')
                if [ -z "$exec_name" ]; then
                    # rclpy nodes do not always expose executable name; try name match
                    exec_name="$n"
                fi
                local csv=",$owned_csv,"
                if [[ ",$exec_name," =~ ,.*$csv.*, ]]; then
                    matched_node="$n"; break
                fi
            done
            if [ -n "$matched_node" ]; then
                m4_die "duplicate inference detected: node '$matched_node' (one of: $owned_csv) is already publishing '$topic'. Stop it first or use ROS_DOMAIN_ID to isolate."
            else
                m4_warn "$topic is alive but publisher is not in owned set ($owned_csv) — likely an external recorder; proceeding"
            fi
        else
            m4_ok "duplicate probe: $topic has no live publisher — safe to start"
        fi
        rm -f "$msg_file"
    done
}

# ---- wait_for_first_message -------------------------------------------
#
# Spawns the m4_demo_bringup/wait_for_message_node.py helper into an
# isolated PGID, captures its stdout (the helper prints
# `WAIT_FOR_FIRST_MESSAGE:UP <topic> after <Ns>` on success or times
# out non-zero). This is used for both readiness gates and algorithm
# duplicate probes. Never calls ros2 topic hz.
#
# Args: topic timeout_s [msg_type]
m4_wait_for_first_message() {
    local topic="$1"
    local timeout_s="${2:-5}"
    local msg_type="${3:-}"
    local out
    out="$(mktemp)"
    # shellcheck disable=SC2086
    setsid python3 -m m4_demo_bringup.wait_for_message_node \
        --topic "$topic" --timeout "$timeout_s" $msg_type \
        > "$out" 2>&1 &
    local pid=$!
    local pgid
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    wait "$pid"
    local rc=$?
    if [ $rc -eq 0 ] && grep -q "WAIT_FOR_FIRST_MESSAGE:UP" "$out"; then
        rm -f "$out"; return 0
    fi
    rm -f "$out"; return 1
}

# ---- record_meta + launch_ros ------------------------------------------
m4_record_meta() {
    local meta="$M4_DEMO_STATE_DIR/$M4_DEMO_NAME.metadata.json"
    cat > "$meta" <<EOF
{
  "demo": "$M4_DEMO_NAME",
  "launcher_pid": $$,
  "pgid": $M4_LAUNCH_PGID,
  "sid": $M4_LAUNCH_SID,
  "launch_pid": $M4_LAUNCH_PID,
  "camera_owned_by_demo": $M4_CAMERA_OWNED,
  "camera_mode": "$M4_CAMERA_MODE",
  "camera_device": "$M4_CAMERA_DEVICE",
  "started_at_iso": "$(date -Iseconds)",
  "started_at_tick": $(awk '{print $22}' "/proc/$$/stat" 2>/dev/null || echo 0),
  "launch_args": {"viewer": "$M4_VIEWER", "no_gui": $M4_NO_GUI, "duration": $M4_DURATION},
  "log_dir": "$M4_LOG_DIR"
}
EOF
    m4_ok "metadata: $meta"
}

# Spawn `ros2 launch ...` via setsid into an isolated process group + session.
# Record PID/PGID/SID into globals. Capture stdout to LOG_DIR/launch.log.
m4_launch_ros() {
    local logfile="$M4_LOG_DIR/launch.log"
    # shellcheck disable=SC2068
    setsid $@ > "$logfile" 2>&1 < /dev/null &
    M4_LAUNCH_PID=$!
    sleep 0.2
    M4_LAUNCH_PGID=$(ps -o pgid= -p "$M4_LAUNCH_PID" 2>/dev/null | tr -d ' ')
    M4_LAUNCH_SID=$(ps -o sid= -p "$M4_LAUNCH_PID" 2>/dev/null | tr -d ' ')
    M4_LAUNCH_START_TICK=$(awk '{print $22}' "/proc/$M4_LAUNCH_PID/stat" 2>/dev/null || echo 0)
    m4_ok "launch PID=$M4_LAUNCH_PID PGID=$M4_LAUNCH_PGID SID=$M4_LAUNCH_SID  (log: $logfile)"
}

# ---- camera publisher ownership ----------------------------------------
#
# Spawns csi_camera_publisher.py in its own PGID and PERSISTS its ownership
# (/tmp/m4_demo/<demo>.camera.json) so cleanup - and the next run - can reap
# it. Without this the publisher outlived the demo and held the V4L2 device
# forever.
#
# Args: <mode>   where mode is csi|usb|test (the publisher's --source)
m4_spawn_camera_publisher() {
    local mode="$1"
    local logfile="$M4_LOG_DIR/camera.log"
    local args=(--source "$mode" --timeout 0 --topic /perception/cameras/front/image)
    if [ "$mode" != "test" ]; then
        args+=(--device "$M4_CAMERA_DEVICE")
    fi
    args+=(--width "$M4_CAMERA_WIDTH" --height "$M4_CAMERA_HEIGHT" --fps "$M4_CAMERA_FPS")

    setsid python3 "$REPO/ros2_ws/src/bev_detection/test/csi_camera_publisher.py" \
        "${args[@]}" > "$logfile" 2>&1 < /dev/null &
    M4_CAMERA_PID=$!
    sleep 0.3
    M4_CAMERA_PGID=$(ps -o pgid= -p "$M4_CAMERA_PID" 2>/dev/null | tr -d ' ')
    M4_CAMERA_TICK=$(awk '{print $22}' "/proc/$M4_CAMERA_PID/stat" 2>/dev/null || echo 0)
    M4_CAMERA_PGID=${M4_CAMERA_PGID:-0}

    M4_CAMERA_META="$M4_DEMO_STATE_DIR/$M4_DEMO_NAME.camera.json"
    cat > "$M4_CAMERA_META" <<EOF
{
  "demo": "$M4_DEMO_NAME",
  "kind": "camera_publisher",
  "pid": $M4_CAMERA_PID,
  "pgid": $M4_CAMERA_PGID,
  "started_at_tick": $M4_CAMERA_TICK,
  "source": "$mode",
  "device": "$M4_CAMERA_DEVICE",
  "width": $M4_CAMERA_WIDTH,
  "height": $M4_CAMERA_HEIGHT,
  "fps": $M4_CAMERA_FPS,
  "started_at_iso": "$(date -Iseconds)"
}
EOF
    m4_ok "camera publisher PID=$M4_CAMERA_PID PGID=$M4_CAMERA_PGID mode=$mode  (log: $logfile)"
}

# Reap a camera publisher leaked by an earlier run of THIS demo.
# Ownership is proven by the recorded start-time tick (PID-reuse safe) and
# by the executable path in /proc/<pid>/cmdline; never a blanket pkill.
m4_reap_leaked_camera_publisher() {
    local meta="$M4_DEMO_STATE_DIR/$M4_DEMO_NAME.camera.json"
    [ -f "$meta" ] || return 0

    local pid pgid tick
    pid=$(python3 -c "import json;print(json.load(open('$meta')).get('pid',0))" 2>/dev/null || echo 0)
    pgid=$(python3 -c "import json;print(json.load(open('$meta')).get('pgid',0))" 2>/dev/null || echo 0)
    tick=$(python3 -c "import json;print(json.load(open('$meta')).get('started_at_tick',0))" 2>/dev/null || echo 0)

    if [ "${pid:-0}" = "0" ] || [ ! -d "/proc/$pid" ]; then
        rm -f "$meta"
        return 0
    fi
    local live_tick
    live_tick=$(awk '{print $22}' "/proc/$pid/stat" 2>/dev/null || echo "")
    if [ "$live_tick" != "$tick" ]; then
        m4_warn "camera.json pid=$pid tick mismatch ($live_tick vs $tick); refusing to signal (PID reuse)"
        rm -f "$meta"
        return 0
    fi
    if ! tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q "csi_camera_publisher.py"; then
        m4_warn "camera.json pid=$pid is not a csi_camera_publisher; refusing to signal"
        rm -f "$meta"
        return 0
    fi

    m4_warn "reaping leaked camera publisher from previous $M4_DEMO_NAME run (pid=$pid pgid=$pgid)"
    kill -INT -"$pgid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    local waited=0
    while [ $waited -lt 5 ] && kill -0 "$pid" 2>/dev/null; do sleep 1; waited=$((waited + 1)); done
    if kill -0 "$pid" 2>/dev/null; then
        kill -9 -"$pgid" 2>/dev/null || kill -9 "$pid" 2>/dev/null || true
    fi
    rm -f "$meta"
    m4_ok "leaked camera publisher reaped; $M4_CAMERA_DEVICE should be free again"
}

m4_launch_viewer() {
    local topic="$1"
    if [ "$M4_NO_GUI" = "1" ] || [ "$M4_VIEWER" = "none" ]; then
        m4_note "viewer: disabled (--no-gui or --viewer=none)"
        return 0
    fi
    if [ "$M4_VIEWER" = "auto" ]; then
        if [ -n "${DISPLAY:-}" ] && command -v rqt_image_view >/dev/null 2>&1; then
            M4_VIEWER="rqt"
        elif [ -n "${DISPLAY:-}" ] && python3 -c 'import cv2' 2>/dev/null; then
            M4_VIEWER="cv"
        else
            m4_note "viewer: no DISPLAY / no rqt / no cv2 — headless"
            return 0
        fi
    fi
    local logfile="$M4_LOG_DIR/viewer.log"
    case "$M4_VIEWER" in
        rqt)
            setsid ros2 run rqt_image_view rqt_image_view "$topic" \
                > "$logfile" 2>&1 < /dev/null &
            M4_VIEWER_PID=$!
            ;;
        cv)
            setsid python3 "$REPO/ros2_ws/src/bev_detection/test/cv_viewer.py" \
                --topic "$topic" --window "M4 demo ($M4_DEMO_NAME)" \
                > "$logfile" 2>&1 < /dev/null &
            M4_VIEWER_PID=$!
            ;;
    esac
    sleep 0.2
    M4_VIEWER_PGID=$(ps -o pgid= -p "$M4_VIEWER_PID" 2>/dev/null | tr -d ' ')
    m4_ok "viewer ($M4_VIEWER) PID=$M4_VIEWER_PID PGID=$M4_VIEWER_PGID  (log: $logfile)"
}

# Status reporter: writes a single line every 5s. Killed by cleanup.
m4_start_status_reporter() {
    (
        local parent=$$
        local start_ts
        start_ts=$(date +%s)
        while true; do
            sleep 5
            local now elapsed
            now=$(date +%s); elapsed=$((now - start_ts))
            if [ "$M4_DURATION" != "0" ] && [ $elapsed -ge $M4_DURATION ]; then
                m4_note "DURATION=${M4_DURATION}s reached"
                kill -INT $parent 2>/dev/null || true
                exit 0
            fi
            if [ -n "$M4_LAUNCH_PID" ] && ! kill -0 "$M4_LAUNCH_PID" 2>/dev/null; then
                m4_note "launch process died, asking parent to clean up"
                kill -INT $parent 2>/dev/null || true
                exit 0
            fi
            local ready="$M4_FIRST_READY"
            local topic_count
            topic_count=$(ros2 topic list 2>/dev/null | wc -l | tr -d ' ')
            echo "  [${elapsed}s] $M4_DEMO_NAME viewer=$M4_VIEWER camera=$M4_CAMERA_MODE topics=$topic_count first_ready=$ready"
        done
    ) &
    M4_STATUS_PID=$!
    m4_ok "status reporter PID=$M4_STATUS_PID"
}

# Wait for the first readiness gate to pass, used by callers.
# Args: <topic> <timeout_s> <description>
m4_wait_until_first_msg() {
    local topic="$1"
    local timeout_s="$2"
    local desc="$3"
    if m4_wait_for_first_message "$topic" "$timeout_s"; then
        m4_ok "$desc UP after <=${timeout_s}s"
        M4_FIRST_READY=1
        M4_TOPICS_AT_READY="$(ros2 topic list 2>/dev/null | grep '^/perception' | sort | tr '\n' ' ')"
        return 0
    fi
    m4_warn "$desc not UP after ${timeout_s}s; pipeline may still be starting"
    return 1
}

# m4_wait_for_stage <topic> <timeout_s> <stage_name>
# Prints [OK] or [FAIL] with exact stage name, returns exit code.
# This is the per-stage readiness gate used during startup diagnostics.
m4_wait_for_stage() {
    local topic="$1"
    local timeout_s="${2:-5}"
    local name="$3"
    # Optional 4th arg: the topic's real message type. wait_for_message_node
    # defaults to sensor_msgs/msg/Image, so probing a non-Image topic (e.g.
    # vision_msgs/Detection2DArray on /perception/detections) subscribed with
    # the WRONG type and could never match => false "timeout" every run.
    # m4_wait_for_first_message word-splits its 3rd arg, so the flag and value
    # must be passed together as one string.
    local msg_type="${4:-sensor_msgs.msg.Image}"
    if m4_wait_for_first_message "$topic" "$timeout_s" "--msg-type $msg_type"; then
        m4_ok "[$name] UP"
        return 0
    else
        # topic is already absolute (starts with "/"); do not prefix again.
        m4_fail "[$name] ${topic} timeout after ${timeout_s}s"
        return 1
    fi
}

# m4_report_launch_failure — call after wait "$M4_LAUNCH_PID" returns non-zero
# to surface the actual root cause from launch.log.
m4_report_launch_failure() {
    local logfile="$M4_LOG_DIR/launch.log"
    if [ -n "$logfile" ] && [ -f "$logfile" ]; then
        m4_fail "ROS launch exited during startup"
        m4_fail "Last 30 lines of launch.log:"
        tail -30 "$logfile" | while read -r line; do
            m4_fail "  $line"
        done
        m4_fail "Full log: $logfile"
    fi
}

m4_cleanup() {
    if [ "$M4_CLEANED_UP" = "1" ]; then
        return 0
    fi
    M4_CLEANED_UP=1
    local reason="${1:-exit}"
    m4_section "Cleanup (reason: $reason)"

    # Status first.
    [ -n "$M4_STATUS_PID" ] && kill "$M4_STATUS_PID" 2>/dev/null
    wait $M4_STATUS_PID 2>/dev/null

    # Viewer.
    if [ -n "$M4_VIEWER_PGID" ] && [ "$M4_VIEWER_PGID" != "0" ]; then
        m4_note "SIGINT viewer PGID $M4_VIEWER_PGID"
        kill -INT -"$M4_VIEWER_PGID" 2>/dev/null || true
    fi
    [ -n "$M4_VIEWER_PID" ] && kill -INT "$M4_VIEWER_PID" 2>/dev/null || true

    # Web server (separate PGID).
    if [ -n "$M4_WEB_PGID" ] && [ "$M4_WEB_PGID" != "0" ]; then
        m4_note "SIGINT web PGID $M4_WEB_PGID (PID $M4_WEB_PID)"
        kill -INT -"$M4_WEB_PGID" 2>/dev/null || true
    fi

    # Camera publisher (separate PGID; MUST be reaped or it holds the device).
    if [ -n "$M4_CAMERA_PGID" ] && [ "$M4_CAMERA_PGID" != "0" ]; then
        m4_note "SIGINT camera publisher PGID $M4_CAMERA_PGID (PID $M4_CAMERA_PID)"
        kill -INT -"$M4_CAMERA_PGID" 2>/dev/null || true
    fi
    [ -n "$M4_CAMERA_PID" ] && kill -INT "$M4_CAMERA_PID" 2>/dev/null || true

    # Launch + its descendants (PGID).
    if [ -n "$M4_LAUNCH_PGID" ] && [ "$M4_LAUNCH_PGID" != "0" ]; then
        m4_note "SIGINT launch PGID $M4_LAUNCH_PGID (PID $M4_LAUNCH_PID)"
        kill -INT -"$M4_LAUNCH_PGID" 2>/dev/null || true
    fi

    # Wait up to 5s for graceful exit.
    local waited=0
    while [ $waited -lt 5 ]; do
        local alive=0
        if [ -n "$M4_LAUNCH_PID" ] && kill -0 "$M4_LAUNCH_PID" 2>/dev/null; then alive=1; fi
        if [ -n "$M4_VIEWER_PID" ] && kill -0 "$M4_VIEWER_PID" 2>/dev/null; then alive=1; fi
        if [ -n "$M4_WEB_PID" ] && kill -0 "$M4_WEB_PID" 2>/dev/null; then alive=1; fi
        if [ -n "$M4_CAMERA_PID" ] && kill -0 "$M4_CAMERA_PID" 2>/dev/null; then alive=1; fi
        [ $alive -eq 0 ] && break
        sleep 1; waited=$((waited + 1))
    done

    # SIGKILL survivors (whole PGID).
    if [ -n "$M4_LAUNCH_PGID" ] && [ "$M4_LAUNCH_PGID" != "0" ] && \
       ( [ -n "$M4_LAUNCH_PID" ] && kill -0 "$M4_LAUNCH_PID" 2>/dev/null ); then
        m4_note "SIGKILL launch PGID $M4_LAUNCH_PGID (unresponsive)"
        kill -9 -"$M4_LAUNCH_PGID" 2>/dev/null || true
    fi
    if [ -n "$M4_WEB_PGID" ] && [ "$M4_WEB_PGID" != "0" ] && \
       ( [ -n "$M4_WEB_PID" ] && kill -0 "$M4_WEB_PID" 2>/dev/null ); then
        m4_note "SIGKILL web PGID $M4_WEB_PGID (unresponsive)"
        kill -9 -"$M4_WEB_PGID" 2>/dev/null || true
    fi
    if [ -n "$M4_VIEWER_PID" ] && kill -0 "$M4_VIEWER_PID" 2>/dev/null; then
        m4_note "SIGKILL viewer PID $M4_VIEWER_PID"
        kill -9 "$M4_VIEWER_PID" 2>/dev/null || true
    fi
    if [ -n "$M4_CAMERA_PID" ] && kill -0 "$M4_CAMERA_PID" 2>/dev/null; then
        m4_note "SIGKILL camera publisher PGID $M4_CAMERA_PGID (unresponsive)"
        kill -9 -"$M4_CAMERA_PGID" 2>/dev/null || kill -9 "$M4_CAMERA_PID" 2>/dev/null || true
    fi

    # Verify the device was actually released when the demo owned it.
    if [ "$M4_CAMERA_OWNED" = "true" ] && [ "$M4_CAMERA_MODE" != "test" ] \
       && [ "$M4_CAMERA_MODE" != "gmsl" ] && [ "$M4_CAMERA_MODE" != "existing" ] \
       && [ -e "$M4_CAMERA_DEVICE" ]; then
        local holders
        holders=$(lsof -t "$M4_CAMERA_DEVICE" 2>/dev/null || true)
        if [ -n "$holders" ]; then
            m4_warn "post-cleanup $M4_CAMERA_DEVICE still held by PIDs: $holders (external, not killing)"
        else
            m4_ok "$M4_CAMERA_DEVICE free"
        fi
    fi

    # Camera release:
    #   usb  : verify /dev/videoN is free (via lsof) — held by demo-owned v4l2src
    #   gmsl : nvargus-daemon (root, PID 1265) + camera_sync_node are external;
    #           we only own camera_adapter_node (terminated by PGID SIGINT above).
    #           GMSL release proof = PGID gone + immediate-restart regression.
    #   csi  : PGID SIGINT handles it; no lsof needed.
    #   test : PGID SIGINT handles it; no device.
    #   existing: nothing to release.
    if [ "$M4_CAMERA_OWNED" = "true" ] && [ "$M4_CAMERA_MODE" = "usb" ] \
       && [ -e "$M4_CAMERA_DEVICE" ]; then
        local holders
        holders=$(lsof -t "$M4_CAMERA_DEVICE" 2>/dev/null || true)
        if [ -n "$holders" ]; then
            m4_warn "post-cleanup $M4_CAMERA_DEVICE still held by PIDs: $holders"
            m4_warn "  these are external processes, not demo-owned; not killing"
        else
            m4_ok "$M4_CAMERA_DEVICE free"
        fi
    fi

    # Remove ownership metadata so the next demo run starts clean.
    local meta="$M4_DEMO_STATE_DIR/$M4_DEMO_NAME.metadata.json"
    [ -f "$meta" ] && rm -f "$meta"
    local webmeta="$M4_DEMO_STATE_DIR/$M4_DEMO_NAME.webmeta.json"
    [ -f "$webmeta" ] && rm -f "$webmeta"
    # Only clear the camera record when we actually owned and stopped it;
    # otherwise a failed reap would be forgotten instead of retried.
    if [ -n "$M4_CAMERA_PID" ] && ! kill -0 "$M4_CAMERA_PID" 2>/dev/null; then
        [ -f "${M4_CAMERA_META:-}" ] && rm -f "$M4_CAMERA_META"
    fi
    m4_ok "DONE"
}

# ---- web demo server helpers ------------------------------------------
#
# The M4 demos expose a low-latency browser preview via an aiohttp +
# aiortc process owned by THIS bash supervisor (second PGID).
# These helpers:
#   - pick a free port (or fail-fast if a non-demo listener holds it)
#   - poll /healthz for the three readiness booleans
#   - record PGID in /tmp/m4_demo/<demo>.webmeta.json so cleanup can
#     verify ownership before killing
#
# Per the plan, the bash supervisor owns the web server. There is no
# launch file for the web server. The server is installed as a
# console_scripts entry_point and started via `ros2 run`.

# Default for the LAN-facing host. Use 0.0.0.0 so laptops on the same
# LAN can connect. Override with WEB_HOST=127.0.0.1 for local-only.
M4_WEB_HOST="${WEB_HOST:-0.0.0.0}"
M4_WEB_PORT="${WEB_PORT:-8080}"
M4_WEB_BACKEND="${WEB_BACKEND:-auto}"     # auto | h264 | mjpeg | vp8
# Encode target for the preview stream. Frames are fitted inside this box;
# 1280x720 keeps both the software fallback and the LAN bandwidth sane while
# still looking sharp in a browser.
M4_WEB_ENCODE_WIDTH="${WEB_ENCODE_WIDTH:-1280}"
M4_WEB_ENCODE_HEIGHT="${WEB_ENCODE_HEIGHT:-720}"
M4_WEB_ENCODE_FPS="${WEB_ENCODE_FPS:-30}"
# Only meaningful for the hardware H.264 path (nvv4l2h264enc), which is NOT
# subject to aiortc's 1.5/3 Mbps encoder clamps.
M4_WEB_H264_BITRATE="${WEB_H264_BITRATE:-6000000}"
M4_WEB_JPEG_QUALITY="${WEB_JPEG_QUALITY:-85}"
M4_WEB_DRY_RUN="${WEB_DRY_RUN:-0}"

m4_jetson_ip() {
    # Best-effort LAN IPv4. Skip loopback. Used only for printing the
    # URL the user should type in their browser.
    local ip
    ip=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^127\.' | grep -v '^$' | head -1)
    if [ -z "$ip" ]; then
        ip=$(hostname -I 2>/dev/null | awk '{print $1}')
    fi
    echo "$ip"
}

# m4_reserve_port <port>
# Bind 127.0.0.1:<port> temporarily; on EADDRINUSE inspect who holds
# it and refuse to start if it is not a previous demo-owned listener.
# Returns 0 if free / cleanable, 1 if held by someone else.
m4_reserve_port() {
    local port="$1"
    if python3 - "$port" <<'PY' 2>/dev/null
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
    then
        m4_ok "port $port is free"
        return 0
    fi
    # Find holder via lsof/ss.
    local holder=""
    if command -v ss >/dev/null 2>&1; then
        holder=$(ss -ltnp "sport = :$port" 2>/dev/null | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2)
    fi
    if [ -z "$holder" ] && command -v lsof >/dev/null 2>&1; then
        holder=$(lsof -ti :"$port" 2>/dev/null | head -1)
    fi
    if [ -n "$holder" ] && [ -d "/proc/$holder" ]; then
        local comm
        comm=$(cat /proc/"$holder"/comm 2>/dev/null || echo "?")
        if [[ "$comm" == *"m4_web_demo_server"* ]] || [[ "$comm" == *"ros2"* ]] && grep -q "m4_web_demo_server" "/proc/$holder/cmdline" 2>/dev/null; then
            m4_warn "port $port held by previous demo server (pid=$holder, comm=$comm); killing"
            kill -9 "$holder" 2>/dev/null || true
            sleep 0.5
            return 0
        fi
        m4_fail "port $port held by unrelated process (pid=$holder, comm=$comm); refusing to bind"
        m4_fail "  hint: pick another port with WEB_PORT=<n>"
        return 1
    fi
    m4_fail "port $port is in use and the holder could not be identified; refusing to bind"
    return 1
}

# m4_wait_for_http_url <url> <timeout_s> <name>
# GET <url> until 200 or timeout. Prints [OK] / [FAIL].
m4_wait_for_http_url() {
    local url="$1"
    local timeout_s="${2:-5}"
    local name="$3"
    local t0
    t0=$(date +%s.%N)
    local deadline
    deadline=$(awk -v t="$t0" -v to="$timeout_s" 'BEGIN{print t+to}')
    while :; do
        local code
        code=$(curl -s -o /dev/null -w '%{http_code}' --max-time 1 "$url" 2>/dev/null || echo 000)
        if [ "$code" = "200" ]; then
            m4_ok "[$name] UP (200)"
            return 0
        fi
        local now
        now=$(date +%s.%N)
        awk -v n="$now" -v d="$deadline" 'BEGIN{exit !(n>=d)}' && break
        sleep 0.2
    done
    m4_fail "[$name] ${url} timeout after ${timeout_s}s"
    return 1
}

# m4_wait_for_health_field <host> <port> <field> <expected> <timeout_s> <name>
# GET http://<host>:<port>/healthz, parse JSON, assert field == expected.
m4_wait_for_health_field() {
    local host="$1"
    local port="$2"
    local field="$3"
    local expected="$4"
    local timeout_s="${5:-5}"
    local name="$6"
    local t0 deadline
    t0=$(date +%s.%N)
    deadline=$(awk -v t="$t0" -v to="$timeout_s" 'BEGIN{print t+to}')
    while :; do
        local body
        body=$(curl -s --max-time 1 "http://$host:$port/healthz" 2>/dev/null || true)
        if [ -n "$body" ]; then
            local val
            # Normalise booleans: Python renders True/False while callers pass
            # the JSON spelling "true"/"false". Without this the gate could
            # never pass for ANY boolean field, so a perfectly healthy server
            # still warned "health gate not satisfied" (pre-existing bug that
            # affected the per-lesson demos too).
            val=$(echo "$body" | python3 -c "
import sys, json
d = json.load(sys.stdin)
v = d.get('$field', '?')
print(str(v).lower() if isinstance(v, bool) else v)
" 2>/dev/null || echo "?")
            if [ "$val" = "$expected" ]; then
                m4_ok "[$name] UP ($field=$val)"
                return 0
            fi
        fi
        local now
        now=$(date +%s.%N)
        awk -v n="$now" -v d="$deadline" 'BEGIN{exit !(n>=d)}' && break
        sleep 0.2
    done
    m4_fail "[$name] field $field (expected $expected) not seen within ${timeout_s}s"
    return 1
}

# m4_web_server_cmd -- resolve how to start the web server.
#
# WHY NOT `ros2 run`: measured on the Jetson, `ros2 run` spawns the node as a
# CHILD in a DIFFERENT process group, so `kill -INT -<wrapper_pgid>` from
# m4_cleanup never reached the node. The node ignored the signal, kept the port
# bound and became an orphan that broke the next run (cycles 2 and 3 failed
# with "port already in use").
#
# Invoking the installed console script directly under setsid makes the server
# itself the session/group leader, so the recorded PGID is exactly the group
# that must receive SIGINT. The workspace is already sourced by m4_setup_env
# (AMENT_PREFIX_PATH/PYTHONPATH/LD_LIBRARY_PATH), so this is equivalent to
# `ros2 run` -- `ros2 run` only execs this same script. Falls back to
# `ros2 run` if the entry point is missing.
M4_WEB_ENTRYPOINT=""
m4_web_server_cmd() {
    local entry="$REPO/ros2_ws/install/m4_demo_bringup/lib/m4_demo_bringup/m4_web_demo_server"
    if [ -f "$entry" ]; then
        M4_WEB_ENTRYPOINT="$entry"
        echo "python3 $entry"
        return 0
    fi
    M4_WEB_ENTRYPOINT=""
    echo "ros2 run m4_demo_bringup m4_web_demo_server"
}

# m4_launch_web_server <demo_key> <demo_num> <video_topic>
# Starts m4_web_demo_server in its own PGID, records PGID in
# M4_DEMO_STATE_DIR/<demo>.webmeta.json.
# Sets globals: M4_WEB_PID, M4_WEB_PGID, M4_WEB_META.
m4_launch_web_server() {
    local demo_key="$1"   # m4_1 | m4_2 | m4_3
    local demo_num="$2"   # 1 | 2 | 3 (for URL)
    local video_topic="$3"

    m4_reserve_port "$M4_WEB_PORT" || return 1

    M4_WEB_META="$M4_DEMO_STATE_DIR/$M4_DEMO_NAME.webmeta.json"
    rm -f "$M4_WEB_META"

    if [ "$M4_WEB_DRY_RUN" = "1" ]; then
        m4_note "[dry-run] would start m4_web_demo_server port=$M4_WEB_PORT backend=$M4_WEB_BACKEND"
        M4_WEB_PID=0
        M4_WEB_PGID=0
        return 0
    fi

    local web_log="$M4_LOG_DIR/web.log"
    # NOTE: the server reads M4_WEB_META_PATH (not M4_WEB_META). Passing the
    # wrong name silently skipped writing webmeta.json, so orphan web servers
    # were invisible to cleanup_demo_residual.sh.
    # shellcheck disable=SC2046
    M4_WEB_META_PATH="$M4_WEB_META" \
    setsid $(m4_web_server_cmd) \
        --demo "$demo_key" \
        --video-topic "$video_topic" \
        --host "$M4_WEB_HOST" \
        --port "$M4_WEB_PORT" \
        --backend "$M4_WEB_BACKEND" \
        --encode-width "$M4_WEB_ENCODE_WIDTH" \
        --encode-height "$M4_WEB_ENCODE_HEIGHT" \
        --encode-fps "$M4_WEB_ENCODE_FPS" \
        --h264-bitrate "$M4_WEB_H264_BITRATE" \
        --jpeg-quality "$M4_WEB_JPEG_QUALITY" \
        > "$web_log" 2>&1 < /dev/null &
    M4_WEB_PID=$!
    sleep 0.4
    M4_WEB_PGID=$(ps -o pgid= -p "$M4_WEB_PID" 2>/dev/null | tr -d ' ')
    M4_WEB_PID=${M4_WEB_PID:-0}
    M4_WEB_PGID=${M4_WEB_PGID:-0}
    m4_ok "web server PID=$M4_WEB_PID PGID=$M4_WEB_PGID port=$M4_WEB_PORT backend=$M4_WEB_BACKEND  (log: $web_log)"
}

# m4_report_web_url <demo_num> -- prints the LAN URL and waits briefly
# for server_ready + ros_frame_ready, NOT peer_ready (per plan).
m4_report_web_url() {
    local demo_num="$1"
    local ip
    ip=$(m4_jetson_ip)
    m4_wait_for_health_field 127.0.0.1 "$M4_WEB_PORT" server_ready true 5.0 \
        "web server (port=$M4_WEB_PORT)" || return 1
    m4_wait_for_health_field 127.0.0.1 "$M4_WEB_PORT" ros_frame_ready true 5.0 \
        "web server ROS frame" || return 1

    m4_ok "Open in browser:"
    m4_ok "  http://$ip:$M4_WEB_PORT/m4/$demo_num"
    m4_note "WebRTC peer_ready will report true once a client connects."
}

# m4_launch_web_hub <topics_spec>
#
# ONE web server for the whole M4 module set (hub mode). Unlike
# m4_launch_web_server this binds a single port and serves a tabbed page
# whose tabs switch the live module over the existing WebRTC connection,
# so 4.1/4.2/4.3 no longer need three servers fighting over port 8080.
#
# Args: <topics_spec>  e.g. "m4_1=/perception/demo/m4_1,m4_2=..."
m4_launch_web_hub() {
    local topics_spec="$1"

    m4_reserve_port "$M4_WEB_PORT" || return 1

    M4_WEB_META="$M4_DEMO_STATE_DIR/hub.webmeta.json"
    rm -f "$M4_WEB_META"

    if [ "$M4_WEB_DRY_RUN" = "1" ]; then
        m4_note "[dry-run] would start hub web server port=$M4_WEB_PORT backend=$M4_WEB_BACKEND topics=$topics_spec"
        M4_WEB_PID=0
        M4_WEB_PGID=0
        return 0
    fi

    local web_log="$M4_LOG_DIR/web.log"
    # shellcheck disable=SC2046
    M4_WEB_META_PATH="$M4_WEB_META" \
    setsid $(m4_web_server_cmd) \
        --demo hub \
        --topics "$topics_spec" \
        --active "${M4_WEB_ACTIVE:-m4_1}" \
        --host "$M4_WEB_HOST" \
        --port "$M4_WEB_PORT" \
        --backend "$M4_WEB_BACKEND" \
        --encode-width "$M4_WEB_ENCODE_WIDTH" \
        --encode-height "$M4_WEB_ENCODE_HEIGHT" \
        --encode-fps "$M4_WEB_ENCODE_FPS" \
        --h264-bitrate "$M4_WEB_H264_BITRATE" \
        --jpeg-quality "$M4_WEB_JPEG_QUALITY" \
        > "$web_log" 2>&1 < /dev/null &
    M4_WEB_PID=$!
    sleep 0.4
    M4_WEB_PGID=$(ps -o pgid= -p "$M4_WEB_PID" 2>/dev/null | tr -d ' ')
    M4_WEB_PID=${M4_WEB_PID:-0}
    M4_WEB_PGID=${M4_WEB_PGID:-0}
    m4_ok "hub web server PID=$M4_WEB_PID PGID=$M4_WEB_PGID port=$M4_WEB_PORT backend=$M4_WEB_BACKEND  (log: $web_log)"
}

# m4_report_hub_url -- readiness gate + printed URL for hub mode.
# Waits for server_ready and (any) ros_frame_ready only; peer_ready is
# never expected before a browser connects.
m4_report_hub_url() {
    local ip
    ip=$(m4_jetson_ip)
    m4_wait_for_health_field 127.0.0.1 "$M4_WEB_PORT" server_ready true 6.0 \
        "hub web server (port=$M4_WEB_PORT)" || return 1
    m4_wait_for_health_field 127.0.0.1 "$M4_WEB_PORT" ros_frame_ready true 15.0 \
        "hub ROS frame (any module)" || return 1

    m4_ok "Open in browser (LAN):"
    m4_ok "  http://$ip:$M4_WEB_PORT/m4/1      (hub page, tabs for 4.1/4.2/4.3)"
    m4_note "Per-module readiness: curl -s http://127.0.0.1:$M4_WEB_PORT/healthz"
    m4_note "Over SSH you can instead forward the port and open http://127.0.0.1:$M4_WEB_PORT/m4/1"
    m4_note "  ssh -N -L $M4_WEB_PORT:127.0.0.1:$M4_WEB_PORT \$(whoami)@<jetson-ip>"
}

# Trap installer — call after init.
m4_install_traps() {
    trap 'm4_cleanup "SIGINT (Ctrl-C)"; exit 130' INT
    trap 'm4_cleanup "SIGTERM"; exit 143' TERM
    trap 'm4_cleanup "EXIT"' EXIT
}

# ---- End of library ----------------------------------------------------
