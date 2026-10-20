#!/usr/bin/env bash
# M7 material check: verify the frozen course package against its hash manifest and
# run its unit tests. Read-only - no model is imported, no network is used and no
# LightNav project is touched.
#
#   docs/M07-Vision-Language-Navigation/code/scripts/m7/m7_materials_check.sh
set -euo pipefail

M7_ROOT="${M7_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M7_ROOT/../../.." && pwd)}"
PYTHON="${PYTHON:-python3}"

printf '[m7] module:    %s\n' "$M7_ROOT"
printf '[m7] workspace: %s\n' "$WS_ROOT"

"$PYTHON" "$M7_ROOT/tools/check_materials.py"
"$PYTHON" -m unittest discover -s "$M7_ROOT/tests" -v