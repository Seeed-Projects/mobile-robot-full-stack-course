#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
M4_ROOT="${M4_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
PYTHON="${PYTHON:-python3}"
FOUNDATIONPOSE_DIR="${FOUNDATIONPOSE_DIR:-}"
OUTPUT="${M45_OUTPUT:-$M4_ROOT/output/m4/m45_native_mvp}"

# Keep binary extensions on the tested conda NumPy/PyTorch stack instead of
# loading incompatible packages from ~/.local/lib/python3.10/site-packages.
export PYTHONNOUSERSITE=1

if [[ -z "$FOUNDATIONPOSE_DIR" ]]; then
  echo 'FoundationPose checkout not configured; export FOUNDATIONPOSE_DIR first.' >&2
  exit 2
fi
if ! command -v "$PYTHON" >/dev/null 2>&1 && [[ ! -x "$PYTHON" ]]; then
  echo "Python interpreter not found: $PYTHON" >&2
  exit 2
fi

exec "$PYTHON" "$SCRIPT_DIR/run_m4_5_native_mvp.py" \
  --foundationpose-dir "$FOUNDATIONPOSE_DIR" \
  --output "$OUTPUT" \
  "$@"
