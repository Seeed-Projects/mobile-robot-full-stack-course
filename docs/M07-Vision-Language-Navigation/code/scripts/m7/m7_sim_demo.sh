#!/usr/bin/env bash
# Start, stop or query the M7 MuJoCo simulation demo through a prepared project.
#
#   m7_sim_demo.sh {start|stop|status} [--project DIR] [--host ADDR] [--port PORT]
#
# status is read-only. start refuses to run without both uv environments and writes
# its log under the module output/ tree. The demo binds 127.0.0.1:8091 by default
# and talks to the model service on 127.0.0.1:8050.
set -euo pipefail

M7_ROOT="${M7_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M7_ROOT/../../.." && pwd)}"

RUNTIME_ROOT="${LIGHTNAV_RUNTIME_ROOT:-$WS_ROOT/shared/lightnav}"
PROJECT="${LIGHTNAV_PROJECT:-$RUNTIME_ROOT/LightNav-0}"
HOST="127.0.0.1"
PORT="8091"
PYTHON="${PYTHON:-python3}"

die() { printf '[m7] %s\n' "$*" >&2; exit 1; }

ACTION="${1:-status}"
case "$ACTION" in
  start|stop|status) shift ;;
  -h|--help) sed -n '2,8p' "${BASH_SOURCE[0]}"; exit 0 ;;
  *) die "usage: m7_sim_demo.sh {start|stop|status} [--project DIR] [--host ADDR] [--port PORT]" ;;
esac

while [ $# -gt 0 ]; do
  case "$1" in
    --project) PROJECT="${2:?--project needs a directory}"; shift ;;
    --host) HOST="${2:?--host needs an address}"; shift ;;
    --port) PORT="${2:?--port needs a number}"; shift ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done

ENTRY="$PROJECT/scripts/jetson_sim_demo.py"
[ -f "$ENTRY" ] || die "not a prepared project (no scripts/jetson_sim_demo.py): $PROJECT - run m7_prepare_lightnav.sh first"

case "$ACTION" in
  status)
    "$PYTHON" "$ENTRY" status
    ;;
  stop)
    "$PYTHON" "$ENTRY" stop
    ;;
  start)
    [ -x "$PROJECT/mujoco_demo/.venv-cpu/bin/python" ] \
      || die "missing simulation environment: $PROJECT/mujoco_demo/.venv-cpu (see docs/M7_DEVICE_RUNBOOK.md)"
    [ -x "$PROJECT/.venv-lightnav-py310/bin/python" ] \
      || die "missing model environment: $PROJECT/.venv-lightnav-py310 (see docs/M7_DEVICE_RUNBOOK.md)"
    if ! "$PYTHON" -c "import socket,sys; s=socket.socket(); sys.exit(0 if s.connect_ex(('127.0.0.1', 8050)) == 0 else 1)"; then
      printf '[m7] warning: nothing is listening on 127.0.0.1:8050 (model service); the demo will not get predictions\n'
    fi
    mkdir -p "$M7_ROOT/output"
    log="$M7_ROOT/output/sim_demo_start_$(date +%Y%m%d-%H%M%S).log"
    args=(start --host "$HOST" --port "$PORT")
    if [ -f "$PROJECT/course-assets/dm-h65/config.json" ] && [ -f "$PROJECT/course-assets/task.json" ]; then
      args+=(--chassis-config "$PROJECT/course-assets/dm-h65/config.json" --task-config "$PROJECT/course-assets/task.json")
    fi
    printf '[m7] log: %s\n' "$log"
    "$PYTHON" "$ENTRY" "${args[@]}" 2>&1 | tee "$log"
    ;;
esac