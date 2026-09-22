#!/usr/bin/env bash
# scripts/m4/preprocess_mesh.sh — wrap mesh_preprocessor for offline use.
#
# Pre-compute geometry (vertices, faces, bbox corners, diameter) for an
# OBJ → /models/m4/pose/processed/<name>.npz. This is a one-time step
# per object.
#
# Usage:
#   bash scripts/m4/preprocess_mesh.sh \
#       --obj models/m4/pose/obj_models/cup.obj \
#       --name cup
#
# Adds the bev_pose source path so the package is importable without a
# full colcon install. If the package was already installed under
# install, that one is preferred (m4_setup_env-like ordering).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"

# ---- CLI ---------------------------------------------------------------
OBJ_PATH=""
OBJ_NAME=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --obj)         OBJ_PATH="$2"; shift 2 ;;
        --name)        OBJ_NAME="$2"; shift 2 ;;
        --out)         OUT_PATH="$2"; shift 2 ;;
        --frame-id)    FRAME_ID="$2"; shift 2 ;;
        *) echo "unknown arg: $1"; exit 2 ;;
    esac
done

if [ -z "$OBJ_PATH" ]; then
    echo "usage: $0 --obj <path.obj> [--name <out_base>] [--out <path.npz>] [--frame-id <id>]"
    exit 2
fi

# Resolve output
[ -z "$OBJ_NAME" ] && OBJ_NAME="$(basename "$OBJ_PATH" .obj)"
[ -z "${OUT_PATH:-}" ] && OUT_PATH="$M4_ROOT/models/m4/pose/processed/${OBJ_NAME}.npz"
[ -z "${FRAME_ID:-}" ] && FRAME_ID="$OBJ_NAME"

mkdir -p "$(dirname "$OUT_PATH")"

# ---- Run ----------------------------------------------------------------
if [ -d "$WS_ROOT/install/bev_pose" ]; then
    echo "[preprocess_mesh] using installed bev_pose"
    set +u; source "$WS_ROOT/install/setup.bash"; set -u
else
    echo "[preprocess_mesh] using source-tree bev_pose"
    export PYTHONPATH="$WS_ROOT/modules/m04-ai-vision-and-edge-acceleration/4.4-foundationpose/ros2/bev_pose:${PYTHONPATH:-}"
fi

python3 -m bev_pose.mesh_preprocessor \
    --obj "$OBJ_PATH" \
    --out "$OUT_PATH" \
    --frame-id "$FRAME_ID"
