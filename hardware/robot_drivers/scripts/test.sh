#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"

if [[ ! -x "${PYTHON_BIN}" ]]; then
    echo "Virtual environment not found: ${PROJECT_DIR}/.venv" >&2
    exit 1
fi

# ROS 2 installs global pytest plugins on Jetson. Keep SDK tests isolated from
# unrelated plugins and their optional dependencies.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
exec "${PYTHON_BIN}" -m pytest -q "$@"

