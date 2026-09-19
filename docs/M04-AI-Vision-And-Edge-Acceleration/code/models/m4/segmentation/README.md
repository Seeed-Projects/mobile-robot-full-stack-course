# SegFormer-B0 model artifacts

**Status: PRESENT (2026-09-18).** Built and verified on the target Jetson, AGX
Orin 32GB / CUDA 12.6 / TensorRT 10.3.0.

| Artifact | Expected path | State |
| --- | --- | --- |
| ONNX export | `onnx/segformer_b0.onnx` | present - 1.3 MB, opset 18 |
| TensorRT engine | `engines/segformer_b0_fp16.engine` | present - 10.8 MB, FP16 |
| Class labels | `labels/labels.json` | present - 19 classes (tracked in git) |

The `.onnx`, the `.onnx.data` sidecar and the `.engine` are **gitignored** - they
are target-specific binaries and the checkpoint's licence does not permit
redistribution. Reproduce them on the target rather than copying them.

## Model

| Field | Value |
| --- | --- |
| Model | SegFormer-B0, Cityscapes fine-tune |
| Checkpoint | `nvidia/segformer-b0-finetuned-cityscapes-512-1024` |
| Task | semantic segmentation, 19 Cityscapes classes |
| Input shape | `1x3x512x1024` (letterboxed, ImageNet normalised) |
| Output shape | `1x19x128x256` logits (stride 4) |
| Precision | FP16 |
| Licence | **UNSPECIFIED** - `license: other` with no terms; see `LICENSE.md` |

## How to produce

```bash
cd modules/m04-ai-vision-and-edge-acceleration
export HF_ENDPOINT=https://hf-mirror.com   # huggingface.co is unreachable from this Jetson
./scripts/m4/export_segformer.sh           # checkpoint -> ONNX
./scripts/m4/build_segformer_engine.sh     # ONNX -> FP16 engine
./scripts/m4/generate_labels_json.sh       # labels/labels.json
```

Then check the result rather than assuming it:

```bash
./scripts/m4/verify_segformer_parity.py \
    --image  output/m4/4.3/camera_frame.png \
    --engine models/m4/segmentation/engines/segformer_b0_fp16.engine \
    --runner ros2_ws/install/bev_segmentation/lib/bev_segmentation/parity_runner
```

## Notes worth knowing before you rebuild

`export_segformer.sh` writes **opset 18**, not 13. The `torch.export`-based ONNX
exporter (torch >= 2.9 default) has no `Resize` implementation below opset 17, so
asking for 13 left the file at 18 anyway while printing a conversion failure.
TensorRT 10.3 parses 18 natively. It also needs `onnxscript`; the script falls back
to the legacy TorchScript exporter if that package is absent.

The exporter spills weights into a sidecar `segformer_b0.onnx.data`. Keep the two
files together or TensorRT will not find the weights.

`build_segformer_engine.sh` used to abort before reaching `trtexec` - `trtexec
--version` is not a flag in TensorRT 10, it prints the usage text and exits 1,
which under `set -euo pipefail` killed the script. That is fixed here.