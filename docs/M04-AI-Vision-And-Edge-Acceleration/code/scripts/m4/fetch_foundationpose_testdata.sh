#!/usr/bin/env bash
# scripts/m4/fetch_foundationpose_testdata.sh
#
# Fetch NVlabs/FoundationPose official test dataset (test/cup/) for
# Phase 0 standalone verification. This is a one-time download.
#
# Output: models/m4/pose/test/cup/{rgb,depth,mask,texture.png,textured.obj,cam_K.txt}
#
# Source data is committed in NVlabs/FoundationPose repo (test_data/test/cup/).
# Since we don't want to clone the upstream repo (large), we pull just the
# assets via git sparse-checkout.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
DEST="$M4_ROOT/models/m4/pose/test/cup"
TMP="$(mktemp -d)"

mkdir -p "$DEST"

echo "=== Fetching NVlabs/FoundationPose test/cup assets via sparse git ==="
cd "$TMP"
git clone --no-checkout --depth 1 --filter=blob:none https://github.com/NVlabs/FoundationPose.git foundationpose 2>&1 | tail -3
cd foundationpose
git sparse-checkout init --cone
git sparse-checkout set test_data/test/cup
git checkout

if [ -d test_data/test/cup ]; then
    cp -r test_data/test/cup/. "$DEST/"
    echo ""
    echo "  [OK] test data at $DEST"
    ls -la "$DEST"
    exit 0
else
    echo "  [FAIL] sparse-checkout did not produce test_data/test/cup"
    echo "  The repo layout may have changed; fetch full repo manually:"
    echo "    git clone https://github.com/NVlabs/FoundationPose.git /tmp/FoundationPose"
    echo "    cp -r /tmp/FoundationPose/test_data/test/cup/* $DEST/"
    exit 2
fi
