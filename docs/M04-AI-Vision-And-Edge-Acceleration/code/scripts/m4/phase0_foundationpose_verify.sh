#!/usr/bin/env bash
set -euo pipefail

# Compatibility entry point retained for existing M4 notes. The MVP runner is
# the source of truth for the native FoundationPose smoke test.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/run_m4_5_native_mvp.sh" "$@"
