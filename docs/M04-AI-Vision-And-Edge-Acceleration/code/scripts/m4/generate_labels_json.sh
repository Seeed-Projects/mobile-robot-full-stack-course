#!/usr/bin/env bash
# scripts/m4/generate_labels_json.sh
#
# Auto-extract id2label mapping from the HuggingFace checkpoint
# and write to models/m4/segmentation/labels/labels.json.
#
# MUST run in EXPORT-ONLY env (conda py310 with transformers).

set -euo pipefail

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
OUT_DIR="$M4_ROOT/models/m4/segmentation/labels"
OUT_FILE="$OUT_DIR/labels.json"
CHECKPOINT="nvidia/segformer-b0-finetuned-cityscapes-512-1024"

# huggingface.co itself has no route from this Jetson; hf-mirror.com does.
# An explicit HF_ENDPOINT from the caller always wins.
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"

PYTHON="${PYTHON:-python3}"

mkdir -p "$OUT_DIR"

"$PYTHON" - "$CHECKPOINT" "$OUT_FILE" << 'PYEOF'
import sys, json
from transformers import AutoModelForSemanticSegmentation

checkpoint = sys.argv[1]
out_file = sys.argv[2]

model = AutoModelForSemanticSegmentation.from_pretrained(checkpoint)
id2label = model.config.id2label  # {0: "road", 1: "sidewalk", ...}

# Cityscapes standard 19-class
labels = {str(int(k)): str(v) for k, v in id2label.items()}
n = len(labels)
assert n == 19, f"Expected 19 classes, got {n}"

with open(out_file, "w") as f:
    json.dump(labels, f, indent=2)

print(f"[labels] {n} classes written to {out_file}")
for k in sorted(labels.keys(), key=int):
    print(f"  {k:>2}: {labels[k]}")
PYEOF

echo "[generate_labels_json] Done."
