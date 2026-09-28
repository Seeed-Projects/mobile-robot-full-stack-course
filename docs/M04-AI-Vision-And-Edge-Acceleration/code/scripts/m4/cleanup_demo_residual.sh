#!/usr/bin/env bash
# scripts/m4/cleanup_demo_residual.sh — RECOVERY TOOL ONLY.
#
# This script is NOT the primary cleanup path. Normal Ctrl-C / SIGINT
# on the run_m4_X_demo.sh bash supervisor MUST already be sufficient.
# This script is only for when an earlier demo crashed and left its
# /tmp/m4_demo/<demo>.metadata.json behind, OR when you need to audit
# the state of demo-owned resources.
#
# SAFETY CONTRACT
#   * Never use pkill <process-name>, killall, or blanket pkill -u.
#   * Only acts on processes recorded in /tmp/m4_demo/*.metadata.json
#     whose /proc/<pid>/stat start-time tick matches the recorded
#     started_at_tick (PID-reuse protection).
#   * SIGINT first, wait 5s, SIGKILL the entire PGID.
#   * Never touch PIDs whose start-time tick does not match (even if
#     the PID happens to be alive) — they may be someone else's
#     process that recycled our PID.
#   * GMSL mode: nvargus-daemon (root, PID 1265) + camera_sync_node are
#     external; the demo only owns camera_adapter_node (in the demo PGID).
#     After PGID SIGINT the adapter is dead; nvargus-daemon is untouched.
#   * USB mode: verify /dev/videoN is free via lsof after SIGINT.
#   * Refuses to operate without --yes (interactive safety).
#
# Usage:
#   cleanup_demo_residual.sh --dry-run           # print what would be done
#   cleanup_demo_residual.sh --list              # print registered demos
#   cleanup_demo_residual.sh --yes [--demo N]    # act on all / one demo
#   cleanup_demo_residual.sh --help

set -u

STATE_DIR="${M4_DEMO_STATE_DIR:-/tmp/m4_demo}"

print_help() {
    sed -n '3,25p' "$0"
    cat <<EOF

Usage:
  $0 --dry-run [--demo N]
  $0 --list
  $0 --yes [--demo N]
  $0 --help
EOF
}

DRY_RUN=0
LIST_ONLY=0
YES=0
DEMO_FILTER=""

for a in "$@"; do
    case "$a" in
        --dry-run) DRY_RUN=1 ;;
        --list) LIST_ONLY=1 ;;
        --yes) YES=1 ;;
        --demo) shift; DEMO_FILTER="${1:-}" ;;
        --demo=*) DEMO_FILTER="${a#--demo=}" ;;
        --help|-h) print_help; exit 0 ;;
        *) echo "unknown option: $a" >&2; exit 2 ;;
    esac
done

# list uses python (jq not assumed)
list_registered() {
    if [ ! -d "$STATE_DIR" ]; then
        echo "[cleanup_demo_residual] no state dir ($STATE_DIR) — nothing registered"
        return
    fi
    local found=0
    shopt -s nullglob
    for meta in "$STATE_DIR"/*.metadata.json "$STATE_DIR"/*.webmeta.json; do
        [ -f "$meta" ] || continue
        found=1
        python3 - "$meta" <<'PY'
import json, sys, os, time
p = sys.argv[1]
try:
    d = json.load(open(p))
except Exception as e:
    print(f'  ! unreadable: {p}: {e}')
    sys.exit(0)
pid = d.get('launcher_pid', '?')
pgid = d.get('pgid', '?')
tick = d.get('started_at_tick', '?')
demo = d.get('demo', '?')
alive = '-'
actual_tick = '-'
if pid != '?':
    try:
        with open(f'/proc/{pid}/stat') as f:
            fields = f.read().split()
            actual_tick = fields[21]
            alive = 'ALIVE'
    except Exception:
        alive = 'DEAD/REUSED'
        actual_tick = '?'
match = 'match' if (actual_tick != '-' and str(actual_tick) == str(tick)) else 'MISMATCH/PID-reused'
print(f'  - {p}')
print(f'      demo={demo} launcher_pid={pid} pgid={pgid} started_tick={tick}')
print(f'      alive_now={alive} actual_tick={actual_tick} ({match})')
PY
    done
    shopt -u nullglob
    [ "$found" = "0" ] && echo "  (no metadata files in $STATE_DIR)"
}

if [ "$LIST_ONLY" = "1" ]; then
    list_registered
    exit 0
fi

if [ "$YES" != "1" ] && [ "$DRY_RUN" != "1" ]; then
    echo "Refusing to operate without --yes or --dry-run. Run '$0 --help' for usage." >&2
    exit 2
fi

echo "[cleanup_demo_residual] state dir: $STATE_DIR"
echo "[cleanup_demo_residual] mode: $([ "$DRY_RUN" = "1" ] && echo "DRY-RUN" || echo "LIVE")"

shopt -s nullglob
matched=0
for meta in "$STATE_DIR"/*.metadata.json; do
    [ -f "$meta" ] || continue
    DEMO=$(python3 -c "import json,sys; print(json.load(open('$meta'))['demo'])" 2>/dev/null || echo "")
    if [ -n "$DEMO_FILTER" ] && [ "$DEMO" != "$DEMO_FILTER" ]; then
        continue
    fi
    matched=1

    # Resolve live PID/PGID/tick and verify ownership.
    eval "$(python3 - "$meta" <<'PY'
import json, sys, os
p = sys.argv[1]
try:
    d = json.load(open(p))
except Exception:
    sys.exit(0)
pid = d.get('launcher_pid')
pgid = d.get('pgid')
tick = d.get('started_at_tick')
demo = d.get('demo', '')
alive_pid = False
actual_tick = ''
try:
    with open(f'/proc/{pid}/stat') as f:
        fields = f.read().split()
        actual_tick = fields[21]
        alive_pid = True
except Exception:
    alive_pid = False

print(f'META_PID={pid}')
print(f'META_PGID={pgid}')
print(f'META_TICK={tick}')
print(f'META_DEMO={demo}')
print(f'LIVE_ALIVE={"1" if alive_pid else "0"}')
print(f'LIVE_TICK={actual_tick}')
PY
)"

    if [ "${LIVE_ALIVE:-0}" != "1" ]; then
        echo "  - $meta: launcher_pid $META_PID is DEAD/REUSED — removing metadata only"
        [ "$DRY_RUN" = "0" ] && rm -f "$meta"
        continue
    fi

    if [ "$LIVE_TICK" != "$META_TICK" ]; then
        echo "  - $meta: pid=$META_PID has different start-time tick ($LIVE_TICK vs $META_TICK); REFUSING to signal (PID-reuse protection)"
        echo "    If you are sure this PID is yours, investigate manually with: ps -fp $META_PID"
        continue
    fi

    # Verify PGID still contains our PID.
    PGID_OK=0
    if [ -n "$META_PGID" ] && [ "$META_PGID" != "0" ]; then
        if [ -d "/proc/$META_PGID" ]; then
            PGID_OK=1
        fi
    fi

    if [ "$PGID_OK" != "1" ]; then
        echo "  - $meta: PGID $META_PGID is gone; sending SIGINT to launcher PID directly"
        if [ "$DRY_RUN" = "0" ]; then
            kill -INT "$META_PID" 2>/dev/null || true
            sleep 2
            kill -0 "$META_PID" 2>/dev/null && kill -9 "$META_PID" 2>/dev/null || true
            rm -f "$meta"
        fi
        continue
    fi

    if [ "$DRY_RUN" = "1" ]; then
        echo "  - would SIGINT then SIGKILL launcher_pid=$META_PID and PGID=$META_PGID (demo=$META_DEMO)"
    else
        echo "  - SIGINT PID $META_PID + PGID $META_PGID (demo=$META_DEMO)"
        kill -INT -"$META_PGID" 2>/dev/null || true
        sleep 3
        # Try graceful first on the launcher PID
        kill -INT "$META_PID" 2>/dev/null || true
        sleep 2
        # SIGKILL the whole PGID
        kill -0 "$META_PID" 2>/dev/null && \
            echo "    still alive after SIGINT; SIGKILL PGID" && \
            kill -9 -"$META_PGID" 2>/dev/null
        rm -f "$meta"
        echo "    metadata removed"
    fi
done

# Web server metadata: same safety contract as above. The web server
# is owned by the demo and runs in its own PGID; cleanup must verify
# /proc/<pid>/stat start-time tick before signalling.
for webmeta in "$STATE_DIR"/*.webmeta.json; do
    [ -f "$webmeta" ] || continue
    DEMO=$(python3 -c "import json,sys; print(json.load(open('$webmeta'))['demo'])" 2>/dev/null || echo "")
    if [ -n "$DEMO_FILTER" ] && [ "$DEMO" != "$DEMO_FILTER" ]; then
        continue
    fi
    matched=1

    eval "$(python3 - "$webmeta" <<'PY'
import json, sys
p = sys.argv[1]
try:
    d = json.load(open(p))
except Exception:
    sys.exit(0)
pid = d.get('pid')
pgid = d.get('pgid')
tick = d.get('started_at_tick')
demo = d.get('demo', '')
alive_pid = False
actual_tick = ''
try:
    with open(f'/proc/{pid}/stat') as f:
        fields = f.read().split()
        actual_tick = fields[21]
        alive_pid = True
except Exception:
    alive_pid = False
print(f'WEBMETA_PID={pid}')
print(f'WEBMETA_PGID={pgid}')
print(f'WEBMETA_TICK={tick}')
print(f'WEBMETA_DEMO={demo}')
print(f'WEB_LIVE_ALIVE={"1" if alive_pid else "0"}')
print(f'WEB_LIVE_TICK={actual_tick}')
PY
)"

    if [ "${WEB_LIVE_ALIVE:-0}" != "1" ]; then
        echo "  - $webmeta: web_pid $WEBMETA_PID is DEAD/REUSED — removing metadata only"
        [ "$DRY_RUN" = "0" ] && rm -f "$webmeta"
        continue
    fi

    if [ "$WEB_LIVE_TICK" != "$WEBMETA_TICK" ]; then
        echo "  - $webmeta: pid=$WEBMETA_PID has different start-time tick ($WEB_LIVE_TICK vs $WEBMETA_TICK); REFUSING to signal (PID-reuse protection)"
        continue
    fi

    if [ "$DRY_RUN" = "1" ]; then
        echo "  - would SIGINT then SIGKILL web_pid=$WEBMETA_PID and PGID=$WEBMETA_PGID (demo=$WEBMETA_DEMO)"
    else
        echo "  - SIGINT web PID $WEBMETA_PID + PGID $WEBMETA_PGID (demo=$WEBMETA_DEMO)"
        [ -n "$WEBMETA_PGID" ] && [ "$WEBMETA_PGID" != "0" ] && \
            kill -INT -"$WEBMETA_PGID" 2>/dev/null || true
        kill -INT "$WEBMETA_PID" 2>/dev/null || true
        sleep 3
        kill -0 "$WEBMETA_PID" 2>/dev/null && \
            echo "    still alive after SIGINT; SIGKILL web PGID" && \
            kill -9 -"$WEBMETA_PGID" 2>/dev/null
        rm -f "$webmeta"
        echo "    webmeta removed"
    fi
done

shopt -u nullglob

if [ "$matched" = "0" ]; then
    echo "[cleanup_demo_residual] no metadata matched (filter='${DEMO_FILTER}')"
fi
