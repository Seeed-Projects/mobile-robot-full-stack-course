#!/usr/bin/env bash
# scripts/m4/build_segformer_engine.sh
#
# Build TensorRT FP16 engine for the static ONNX model.
#
# ⚠️ TensorRT 10.3+ removed --workspace flag.
#    Use --memPoolSize=workspace:<MB> instead (see Plan §C Step 3).
#
# ⚠️ .engine is a TARGET-SPECIFIC artifact (Jetson/AGX-Orin/CUDA12.6/TensorRT10.3).
#    It MUST be reproducible from checkpoint → ONNX → this command on the SAME
#    target hardware. Do NOT treat .engine as a portable source artifact.
#    See docs/M4.3_SEMANTIC_SEGMENTATION.md reproduction guide.

set -euo pipefail

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
ONNX="$M4_ROOT/models/m4/segmentation/onnx/segformer_b0.onnx"
ENGINE_DIR="$M4_ROOT/models/m4/segmentation/engines"
ENGINE="$ENGINE_DIR/segformer_b0_fp16.engine"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"

if [[ ! -f "$ONNX" ]]; then
    echo "[build_engine] ERROR: ONNX not found: $ONNX"
    echo "[build_engine] Run scripts/m4/export_segformer.sh first."
    exit 1
fi

if [[ ! -x "$TRTEXEC" ]]; then
    echo "[build_engine] ERROR: trtexec not executable: $TRTEXEC"
    exit 1
fi

mkdir -p "$ENGINE_DIR"

echo "[build_engine] ONNX:   $ONNX"
echo "[build_engine] Output: $ENGINE"
echo "[build_engine] TRT version:"
# `--version` is not a real flag in TensorRT 10's trtexec: it prints the whole
# usage text (~261 lines) and exits 1. That broke this script in two ways at
# once under `set -euo pipefail` — `head -3` closed the pipe early (SIGPIPE,
# exit 141) and even without it the probe's own exit 1 propagated. The build
# never started. Capture the banner line and never let the probe fail.
trt_banner="$("$TRTEXEC" --version 2>&1 || true)"
echo "[build_engine] $(printf '%s\n' "$trt_banner" | sed -n '1p')"
echo "[build_engine] flags:  --fp16 --memPoolSize=workspace:4096M (TRT10.x)"
echo "[build_engine]         static shape [1,3,512,1024] — no --minShapes/--optShapes/--maxShapes"

# NOTE: NO --minShapes/--optShapes/--maxShapes — ONNX is STATIC.
#       Just build with FP16 + workspace pool size.
"$TRTEXEC" \
    --onnx="$ONNX" \
    --fp16 \
    --memPoolSize=workspace:4096M \
    --saveEngine="$ENGINE" \
    --verbose 2>&1 | tail -80

echo "[build_engine] Done."
ls -la "$ENGINE"
echo "[build_engine] NOTE: this .engine is bound to Jetson AGX Orin / CUDA 12.6 / TensorRT 10.3."
echo "[build_engine] NOTE: to reproduce, re-run from checkpoint → ONNX → this script on the same target."
