#!/usr/bin/env bash
# nuScenes v1.0-mini download + integrity record (Phase 1 data gate)
# Source: https://www.nuscenes.org/nuscenes (public, CC BY-NC-SA 4.0 — non-commercial)
set -euo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS_DIR="$REPO_DIR/datasets/nuscenes"
mkdir -p "$NS_DIR"
cd "$NS_DIR"

URL="https://www.nuscenes.org/data/v1.0-mini.tgz"
TARGET="v1.0-mini.tgz"

if [ -f "$TARGET" ]; then
  SIZE=$(stat -c%s "$TARGET")
  if [ "$SIZE" -gt 4000000000 ]; then
    echo "already downloaded: $(du -h "$TARGET" | cut -f1)"
  fi
else
  echo "downloading $URL ..."
  curl -L --retry 3 -o "$TARGET" "$URL"
fi

echo "sha256:"
sha256sum "$TARGET" | tee "$TARGET.sha256"

echo "extracting ..."
tar -xzf "$TARGET"
ls "$NS_DIR/v1.0-mini" | head
echo "OK — meta + samples data ready for dataset_converter"