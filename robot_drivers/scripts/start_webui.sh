#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"

if [[ ! -x "${PYTHON_BIN}" ]]; then
    echo "Virtual environment not found. Run: python3 -m venv .venv && .venv/bin/pip install -e ." >&2
    exit 1
fi

exec "${PYTHON_BIN}" -m dm_h65.webui \
    --host "${DM_H65_WEBUI_HOST:-0.0.0.0}" \
    --port "${DM_H65_WEBUI_PORT:-8765}" \
    --interface "${DM_H65_CAN_INTERFACE:-can0}" \
    "$@"

