#!/usr/bin/env python3
"""PyTorch <-> TensorRT parity gate for M4.3 semantic segmentation.

Run in the EXPORT-ONLY conda environment (py310: torch + transformers). See
test/parity_runner.cpp for why this cannot be a single process: torch lives
only in conda, the `tensorrt` bindings only in the system interpreter.

What it does, in order:

  1. preprocess the image with a byte-faithful reimplementation of the C++
     letterbox_to_chw_float() in src/preprocess.cpp
  2. run the PyTorch checkpoint on that input  -> pytorch_logits.bin
  3. shell out to parity_runner, which replays the SAME input through the real
     production SegmentationEngine        -> trt_logits.bin
  4. compare argmax masks: pixel agreement, per-class IoU, mean IoU

Comparing argmax rather than raw logits is deliberate: FP16 rounding moves
near-zero logits around harmlessly, and what the node actually publishes is the
argmax. Raw-logit deltas are reported too, as diagnostics only.

Usage
-----
  python3 verify_segformer_parity.py \
      --image    output/m4/4.3/semantic_snapshot.png \
      --engine   models/m4/segmentation/engines/segformer_b0_fp16.engine \
      --runner   ros2_ws/install/bev_segmentation/lib/bev_segmentation/parity_runner \
      --out-dir  output/m4/4.3/parity \
      --json     output/m4/4.3/parity.json

  # reference only, without the TensorRT half (debugging)
  python3 verify_segformer_parity.py --image x.png --reference-only
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

# Must match include/bev_segmentation/types.hpp exactly.
MODEL_H = 512
MODEL_W = 1024
LOGIT_H = MODEL_H // 4          # 128
LOGIT_W = MODEL_W // 4          # 256
NUM_CLASSES = 19
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

# The letterbox pad colour in src/preprocess.cpp is ImageNet-normalised mid-grey.
PAD_PIXEL = 114.0 / 255.0

# Gate thresholds. FP16 vs FP32 on SegFormer-B0 moves only pixels already near
# a class boundary, so agreement is expected to be very high; the slack below
# is what we are willing to accept rather than an expectation.
MIN_AGREEMENT = 0.95
MIN_MIOU = 0.85

CHECKPOINT = "nvidia/segformer-b0-finetuned-cityscapes-512-1024"


# --------------------------------------------------------------------------
# preprocessing: byte-faithful port of src/preprocess.cpp
# --------------------------------------------------------------------------
def compute_letterbox(orig_h: int, orig_w: int, dst_h: int, dst_w: int):
    """Mirror of compute_letterbox() in src/preprocess.cpp.

    Rounding matters: the C++ uses static_cast<int>(v + 0.5f), i.e. round-half-up
    for positive values. Any other rounding here desynchronises the geometry the
    parity test is meant to validate.
    """
    scale = min(dst_h / float(orig_h), dst_w / float(orig_w))
    new_w = int(orig_w * scale + 0.5)
    new_h = int(orig_h * scale + 0.5)
    pad_x = (dst_w - new_w) // 2
    pad_y = (dst_h - new_h) // 2
    return scale, new_w, new_h, pad_x, pad_y


def letterbox_to_chw_float(rgb: np.ndarray, dst_h: int = MODEL_H, dst_w: int = MODEL_W):
    """Mirror of letterbox_to_chw_float() with encoding="rgb8".

    rgb: uint8 (H, W, 3) in RGB order, as the C++ sees an rgb8 image.
    Returns (chw float32 [3,dst_h,dst_w], meta dict).
    """
    orig_h, orig_w = rgb.shape[:2]
    scale, new_w, new_h, pad_x, pad_y = compute_letterbox(orig_h, orig_w, dst_h, dst_w)

    out = np.empty((3, dst_h, dst_w), dtype=np.float32)
    for c in range(3):
        out[c].fill((PAD_PIXEL - IMAGENET_MEAN[c]) / IMAGENET_STD[c])

    inv_scale = (1.0 / scale) if scale > 0 else 0.0

    dy = np.arange(new_h, dtype=np.float32)
    oy = np.minimum(orig_h - 1, (dy * inv_scale + 0.5).astype(np.int32))
    dx = np.arange(new_w, dtype=np.float32)
    ox = np.minimum(orig_w - 1, (dx * inv_scale + 0.5).astype(np.int32))

    sampled = rgb[np.ix_(oy, ox)].astype(np.float32) / 255.0        # (new_h, new_w, 3)
    normed = (sampled - IMAGENET_MEAN) / IMAGENET_STD

    # The C++ skips samples that would fall outside the canvas; with
    # scale = min(...) that only matters at a rounding boundary, so clamp the
    # destination window rather than assuming it always fits.
    copy_h = min(new_h, dst_h - pad_y)
    copy_w = min(new_w, dst_w - pad_x)
    if copy_h > 0 and copy_w > 0:
        out[:, pad_y:pad_y + copy_h, pad_x:pad_x + copy_w] = \
            normed[:copy_h, :copy_w].transpose(2, 0, 1)

    meta = {
        "orig_h": orig_h, "orig_w": orig_w,
        "dst_h": dst_h, "dst_w": dst_w,
        "scale": scale, "new_h": new_h, "new_w": new_w,
        "pad_x": pad_x, "pad_y": pad_y,
    }
    return out, meta


# --------------------------------------------------------------------------
# metrics
# --------------------------------------------------------------------------
def argmax_mask(logits: np.ndarray) -> np.ndarray:
    return logits.reshape(NUM_CLASSES, LOGIT_H, LOGIT_W).argmax(axis=0).astype(np.uint8)


def compare(ref_logits: np.ndarray, trt_logits: np.ndarray) -> dict:
    ref_arg = argmax_mask(ref_logits)
    trt_arg = argmax_mask(trt_logits)

    agreement = float((ref_arg == trt_arg).mean())

    per_class = {}
    for c in range(NUM_CLASSES):
        r = ref_arg == c
        t = trt_arg == c
        union = int(np.count_nonzero(r | t))
        if union == 0:
            continue                     # class absent from both: undefined, not zero
        inter = int(np.count_nonzero(r & t))
        per_class[c] = {"iou": inter / union, "ref_px": int(r.sum()), "trt_px": int(t.sum())}

    miou = float(np.mean([v["iou"] for v in per_class.values()])) if per_class else 0.0

    diff = np.abs(ref_logits - trt_logits)
    return {
        "pixel_agreement": agreement,
        "mean_iou": miou,
        "classes_present": len(per_class),
        "per_class_iou": per_class,
        "logit_abs_diff_max": float(diff.max()),
        "logit_abs_diff_mean": float(diff.mean()),
        "disagreeing_pixels": int(np.count_nonzero(ref_arg != trt_arg)),
        "total_pixels": int(ref_arg.size),
    }


# --------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image", required=True, help="source RGB image (any aspect ratio)")
    ap.add_argument("--engine", help="TensorRT engine path")
    ap.add_argument("--runner", help="parity_runner executable")
    ap.add_argument("--out-dir", default=None, help="where to write the .bin artifacts")
    ap.add_argument("--json", dest="json_out", default=None, help="write the report here")
    ap.add_argument("--reference-only", action="store_true",
                    help="only run the PyTorch half (no TensorRT)")
    ap.add_argument("--threshold-agreement", type=float, default=MIN_AGREEMENT)
    ap.add_argument("--threshold-miou", type=float, default=MIN_MIOU)
    args = ap.parse_args()

    out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(args.image)) or ".",
                                           "parity")
    os.makedirs(out_dir, exist_ok=True)

    from PIL import Image
    rgb = np.asarray(Image.open(args.image).convert("RGB"), dtype=np.uint8)
    print(f"[parity] image        : {args.image}  {rgb.shape[1]}x{rgb.shape[0]}")

    chw, meta = letterbox_to_chw_float(rgb)
    print(f"[parity] letterbox    : scale={meta['scale']:.6f} new={meta['new_w']}x{meta['new_h']} "
          f"pad=({meta['pad_x']},{meta['pad_y']})")

    input_path = os.path.join(out_dir, "input.bin")
    chw.tofile(input_path)
    print(f"[parity] wrote input  : {input_path}  ({chw.size} float32)")

    import torch
    from transformers import AutoModelForSemanticSegmentation

    t0 = time.time()
    model = AutoModelForSemanticSegmentation.from_pretrained(CHECKPOINT).eval()
    load_s = time.time() - t0
    with torch.no_grad():
        out = model(pixel_values=torch.from_numpy(chw).unsqueeze(0).float())
    ref_logits = out.logits.squeeze(0).cpu().numpy().astype(np.float32)
    print(f"[parity] pytorch      : logits {ref_logits.shape}  (load {load_s:.1f}s)")

    ref_path = os.path.join(out_dir, "pytorch_logits.bin")
    ref_logits.tofile(ref_path)

    report = {
        "image": os.path.abspath(args.image),
        "image_size": {"width": int(rgb.shape[1]), "height": int(rgb.shape[0])},
        "letterbox": meta,
        "checkpoint": CHECKPOINT,
        "pytorch_logits_shape": list(ref_logits.shape),
        "pytorch_load_seconds": round(load_s, 3),
    }

    if args.reference_only:
        report["tensorrt"] = "skipped (--reference-only)"
    else:
        for name, val in (("--engine", args.engine), ("--runner", args.runner)):
            if not val:
                print(f"[parity] ERROR: {name} is required unless --reference-only", file=sys.stderr)
                return 2
            if not os.path.exists(val):
                print(f"[parity] ERROR: {name} not found: {val}", file=sys.stderr)
                return 2

        trt_path = os.path.join(out_dir, "trt_logits.bin")
        cmd = [args.runner, "--engine", args.engine, "--input", input_path, "--output", trt_path]
        print(f"[parity] runner       : {' '.join(cmd)}")
        proc = subprocess.run(cmd, capture_output=True, text=True)
        sys.stdout.write(proc.stdout)
        if proc.returncode != 0:
            sys.stderr.write(proc.stderr)
            print(f"[parity] ERROR: parity_runner exited {proc.returncode}", file=sys.stderr)
            return 1

        trt_logits = np.fromfile(trt_path, dtype=np.float32)
        expected = NUM_CLASSES * LOGIT_H * LOGIT_W
        if trt_logits.size != expected:
            print(f"[parity] ERROR: trt logits size {trt_logits.size} != {expected}", file=sys.stderr)
            return 1

        cmp_result = compare(ref_logits.ravel(), trt_logits)
        report["tensorrt"] = {
            "engine": os.path.abspath(args.engine),
            "logits_shape": [NUM_CLASSES, LOGIT_H, LOGIT_W],
            **cmp_result,
        }

        ok_agree = cmp_result["pixel_agreement"] >= args.threshold_agreement
        ok_miou = cmp_result["mean_iou"] >= args.threshold_miou
        report["thresholds"] = {
            "min_pixel_agreement": args.threshold_agreement,
            "min_mean_iou": args.threshold_miou,
        }
        report["passed"] = bool(ok_agree and ok_miou)

        print()
        print(f"[parity] pixel agreement : {cmp_result['pixel_agreement']:.6f} "
              f"(>= {args.threshold_agreement}: {'OK' if ok_agree else 'FAIL'})")
        print(f"[parity] mean IoU        : {cmp_result['mean_iou']:.6f} "
              f"(>= {args.threshold_miou}: {'OK' if ok_miou else 'FAIL'})")
        print(f"[parity] classes present : {cmp_result['classes_present']}/{NUM_CLASSES}")
        print(f"[parity] disagreeing px   : {cmp_result['disagreeing_pixels']} "
              f"/ {cmp_result['total_pixels']}")
        print(f"[parity] logit |diff|     : max={cmp_result['logit_abs_diff_max']:.4f} "
              f"mean={cmp_result['logit_abs_diff_mean']:.6f}")

    if args.json_out:
        os.makedirs(os.path.dirname(os.path.abspath(args.json_out)), exist_ok=True)
        with open(args.json_out, "w") as fh:
            json.dump(report, fh, indent=2)
        print(f"[parity] report       : {args.json_out}")

    if report.get("passed") is False:
        print("\n[parity] RESULT: FAIL — do NOT proceed to camera acceptance (see Plan §B6)")
        return 1
    if report.get("passed"):
        print("\n[parity] RESULT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())