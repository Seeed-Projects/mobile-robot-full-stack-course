#!/usr/bin/env bash
# scripts/m4/export_segformer.sh
#
# Export nvidia/segformer-b0-finetuned-cityscapes-512-1024
# to ONNX with STATIC fixed shape [1, 3, 512, 1024].
#
# ⚠️ This script MUST run in an EXPORT-ONLY environment
#    (e.g. conda env py310 with torch+transformers), NOT in Jetson runtime env.
#
# ⚠️ STATIC SHAPE EXPORT — matches fixed TensorRT engine shape.
#    dynamic_axes is INTENTIONALLY omitted to keep ONNX static and prevent
#    mismatch with TRT fixed shape.

set -euo pipefail

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
OUT_DIR="$M4_ROOT/models/m4/segmentation/onnx"
OUT_FILE="$OUT_DIR/segformer_b0.onnx"
CHECKPOINT="nvidia/segformer-b0-finetuned-cityscapes-512-1024"

# huggingface.co itself has no route from this Jetson; hf-mirror.com does.
# An explicit HF_ENDPOINT from the caller always wins.
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"

# Use the caller-selected export environment, or system python3.
PYTHON="${PYTHON:-python3}"

echo "[export_segformer] python:    $($PYTHON --version 2>&1)"
echo "[export_segformer] endpoint:  $HF_ENDPOINT"
echo "[export_segformer] checkpoint: $CHECKPOINT"
echo "[export_segformer] output:     $OUT_FILE"
echo "[export_segformer] shape:      [1, 3, 512, 1024] (STATIC, no dynamic_axes)"

mkdir -p "$OUT_DIR"

"$PYTHON" - "$CHECKPOINT" "$OUT_FILE" << 'PYEOF'
import sys
import torch
from transformers import AutoModelForSemanticSegmentation

checkpoint = sys.argv[1]
out_file = sys.argv[2]

print(f"[export] Loading checkpoint: {checkpoint}")
model = AutoModelForSemanticSegmentation.from_pretrained(checkpoint)
model.eval()
assert model.config.num_labels == 19, \
    f"expected a 19-class Cityscapes checkpoint, got num_labels={model.config.num_labels}"

class LogitsOnly(torch.nn.Module):
    """SegFormer returns a ModelOutput dataclass; the graph must emit a tensor.

    TensorRT binds outputs by shape, so exporting the bare `.logits` tensor
    keeps the graph surface exactly [1,3,512,1024] -> [1,19,128,256].
    """

    def __init__(self, inner):
        super().__init__()
        self.inner = inner

    def forward(self, pixel_values):
        return self.inner(pixel_values=pixel_values).logits

wrapped = LogitsOnly(model).eval()

# STATIC fixed shape — matches TensorRT fixed engine.
# 512 = H, 1024 = W (training contract of this checkpoint)
dummy = torch.randn(1, 3, 512, 1024)

print("[export] Exporting ONNX (opset=18, static shape)...")
export_kwargs = dict(
    input_names=["input"],
    output_names=["logits"],
    # 18, not 13: the torch.export-based exporter has no Resize implementation
    # below 17, and the down-conversion to 13 fails ("No Adapter To Version 17
    # for Resize"). Asking for 13 would leave the file at 18 anyway, with a
    # misleading failure in the log. TensorRT 10.3 parses opset 18 natively.
    opset_version=18,
    do_constant_folding=True,
    # NO dynamic_axes — the TensorRT engine is built for a fixed shape.
)
try:
    # Preferred: the torch.export-based exporter (torch >= 2.9 default).
    torch.onnx.export(wrapped, dummy, out_file, dynamo=True, **export_kwargs)
except (ImportError, ModuleNotFoundError) as exc:
    # Falls back to the legacy TorchScript exporter on environments without
    # onnxscript. Slated for removal upstream, hence the try/except ordering.
    print(f"[export] modern exporter unavailable ({exc}); using legacy exporter")
    torch.onnx.export(wrapped, dummy, out_file, dynamo=False, **export_kwargs)

# Sanity check ONNX structure
import onnx
onnx_model = onnx.load(out_file)
onnx.checker.check_model(onnx_model)

inp = onnx_model.graph.input[0]
out = onnx_model.graph.output[0]
print(f"[export] ONNX input:  name={inp.name} shape={[d.dim_value for d in inp.type.tensor_type.shape.dim]}")
print(f"[export] ONNX output: name={out.name} shape={[d.dim_value for d in out.type.tensor_type.shape.dim]}")

inp_shape = [d.dim_value for d in inp.type.tensor_type.shape.dim]
out_shape = [d.dim_value for d in out.type.tensor_type.shape.dim]
assert inp_shape == [1, 3, 512, 1024], \
    f"Input shape mismatch! expected [1,3,512,1024], got {inp_shape}"
assert out_shape == [1, 19, 128, 256], \
    f"Output shape mismatch! expected [1,19,128,256], got {out_shape}"

actual_opset = next((o.version for o in onnx_model.opset_import
                     if o.domain in ("", "ai.onnx")), None)
assert actual_opset == 18, \
    f"opset mismatch! expected 18, got {actual_opset}"
print(f"[export] opset: {actual_opset}, nodes: {len(onnx_model.graph.node)}")

print(f"[export] OK — {out_file}")
PYEOF

echo "[export_segformer] Done."
ls -la "$OUT_FILE"
