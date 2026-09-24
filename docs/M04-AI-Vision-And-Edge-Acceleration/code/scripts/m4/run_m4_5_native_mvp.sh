#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
M4_ROOT="${M4_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
PYTHON="${PYTHON:-/home/seeed/miniconda3/envs/py310/bin/python}"
FOUNDATIONPOSE_DIR="${FOUNDATIONPOSE_DIR:-/home/seeed/workspace/third_party/FoundationPose}"
OUTPUT="${M45_OUTPUT:-$M4_ROOT/output/m4/m45_native_mvp}"

# Keep binary extensions on the tested conda NumPy/PyTorch stack instead of
# loading incompatible packages from ~/.local/lib/python3.10/site-packages.
export PYTHONNOUSERSITE=1

if [[ ! -x "$PYTHON" ]]; then
  echo "Python interpreter not found: $PYTHON" >&2
  exit 2
fi

exec "$PYTHON" "$SCRIPT_DIR/run_m4_5_native_mvp.py" \
  --foundationpose-dir "$FOUNDATIONPOSE_DIR" \
  --output "$OUTPUT" \
  "$@"
