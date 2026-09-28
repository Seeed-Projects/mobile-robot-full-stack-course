#!/usr/bin/env python3
"""End-to-end geometric acceptance for M4.3 on live camera data.

The unit tests prove unletterbox_mask() is correct in isolation. This proves the
running node is correct as a system: it replays the SAME camera frame through
the real TensorRT engine and compares the result, restored two different ways,
against the mask the node actually published.

  fixed  = argmax -> unletterbox_mask  (what the node now does)
  buggy  = argmax -> nearest_neighbor_resize 128x256 -> 1920x1080 (what it did before)

If the node is doing the right thing, `fixed` agrees with the published mask and
`buggy` visibly does not -- most sharply in the left/right bands that the
letterbox padding used to occupy.

Runs under the SYSTEM python3 (it needs rclpy from the ROS install). No torch is
required: preprocessing here is pure numpy and the inference is done by the
C++ parity_runner, which links the same engine the node loads.

  source /opt/ros/humble/setup.bash
  python3 verify_m4_3_geometry.py --out-dir <dir> [--json <file>]
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

MODEL_H, MODEL_W = 512, 1024
LOGIT_H, LOGIT_W = MODEL_H // 4, MODEL_W // 4
NUM_CLASSES = 19
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
PAD_PIXEL = 114.0 / 255.0


# --------------------------------------------------------------------------
# mirrors of the C++ (same rounding, on purpose)
# --------------------------------------------------------------------------
def compute_letterbox(orig_h, orig_w, dst_h=MODEL_H, dst_w=MODEL_W):
    scale = min(dst_h / float(orig_h), dst_w / float(orig_w))
    new_w = int(orig_w * scale + 0.5)
    new_h = int(orig_h * scale + 0.5)
    return scale, new_w, new_h, (dst_w - new_w) // 2, (dst_h - new_h) // 2


def preprocess(rgb):
    """Mirror of letterbox_to_chw_float() with encoding=rgb8."""
    oh, ow = rgb.shape[:2]
    scale, new_w, new_h, pad_x, pad_y = compute_letterbox(oh, ow)
    out = np.empty((3, MODEL_H, MODEL_W), dtype=np.float32)
    for c in range(3):
        out[c].fill((PAD_PIXEL - MEAN[c]) / STD[c])
    inv = 1.0 / scale if scale > 0 else 0.0
    oy = np.minimum(oh - 1, (np.arange(new_h, dtype=np.float32) * inv + 0.5).astype(np.int32))
    ox = np.minimum(ow - 1, (np.arange(new_w, dtype=np.float32) * inv + 0.5).astype(np.int32))
    normed = (rgb[np.ix_(oy, ox)].astype(np.float32) / 255.0 - MEAN) / STD
    ch, cw = min(new_h, MODEL_H - pad_y), min(new_w, MODEL_W - pad_x)
    out[:, pad_y:pad_y + ch, pad_x:pad_x + cw] = normed[:ch, :cw].transpose(2, 0, 1)
    return out, {"scale": scale, "new_w": new_w, "new_h": new_h,
                 "pad_x": pad_x, "pad_y": pad_y, "orig_h": oh, "orig_w": ow}


def unletterbox(mask, meta, orig_h, orig_w):
    """Mirror of unletterbox_mask()."""
    stride_h, stride_w = MODEL_H // LOGIT_H, MODEL_W // LOGIT_W
    out = np.empty((orig_h, orig_w), dtype=np.uint8)
    ys = np.minimum(MODEL_H - 1,
                    meta["pad_y"] + (np.arange(orig_h, dtype=np.float32) * meta["scale"] + 0.5).astype(np.int32))
    xs = np.minimum(MODEL_W - 1,
                    meta["pad_x"] + (np.arange(orig_w, dtype=np.float32) * meta["scale"] + 0.5).astype(np.int32))
    return mask[(ys // stride_h)[:, None], (xs // stride_w)[None, :]]


def nearest_neighbor_resize(mask, dst_h, dst_w):
    """Mirror of the OLD path's nearest_neighbor_resize()."""
    sy = np.minimum(LOGIT_H - 1, (np.arange(dst_h) * LOGIT_H) // dst_h)
    sx = np.minimum(LOGIT_W - 1, (np.arange(dst_w) * LOGIT_W) // dst_w)
    return mask[sy[:, None], sx[None, :]]


# --------------------------------------------------------------------------
def capture(out_dir):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from sensor_msgs.msg import Image
    from PIL import Image as PILImage

    class Cap(Node):
        def __init__(self):
            super().__init__("m4_3_geometry_cap")
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=10)
            self.img = None
            self.sem = None
            self.create_subscription(Image, "/perception/cameras/front/image", self._i, qos)
            self.create_subscription(Image, "/perception/semantic_mask", self._s, qos)

        def _i(self, m):
            self.img = m

        def _s(self, m):
            self.sem = m

    def ns(m):
        return m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec

    rclpy.init()
    n = Cap()
    best = None            # tightest image/mask stamp pair we can find
    deadline = time.time() + 30
    while time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.img is None or n.sem is None:
            continue
        delta = abs(ns(n.sem) - ns(n.img))
        if best is None or delta < best[0]:
            best = (delta, np.frombuffer(n.img.data, np.uint8).reshape(n.img.height, n.img.width, 3).copy(),
                    np.frombuffer(n.sem.data, np.uint8).reshape(n.sem.height, n.sem.width).copy(),
                    n.img.encoding)
        if delta <= 1_000_000:      # within 1 ms: same frame
            break
    rclpy.shutdown()

    if best is None:
        raise SystemExit("no camera/mask pair received within 30s")

    delta, raw, sem, enc = best
    if enc == "bgr8":
        rgb = raw[:, :, ::-1]
    elif enc == "rgb8":
        rgb = raw
    else:
        raise SystemExit("unexpected source encoding: " + enc)

    PILImage.fromarray(rgb).save(os.path.join(out_dir, "geom_source.png"))
    np.save(os.path.join(out_dir, "geom_node_mask.npy"), sem)
    print(f"[geom] captured pair, stamp delta {delta/1e6:.3f} ms, source {enc} "
          f"{rgb.shape[1]}x{rgb.shape[0]}")
    return rgb, sem


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--runner", required=True)
    ap.add_argument("--engine", required=True)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    rgb, node_mask = capture(args.out_dir)
    orig_h, orig_w = rgb.shape[:2]

    chw, meta = preprocess(rgb)
    print(f"[geom] letterbox scale={meta['scale']:.6f} pad=({meta['pad_x']},{meta['pad_y']}) "
          f"new={meta['new_w']}x{meta['new_h']}")

    input_path = os.path.join(args.out_dir, "geom_input.bin")
    chw.tofile(input_path)
    logits_path = os.path.join(args.out_dir, "geom_trt_logits.bin")
    proc = subprocess.run([args.runner, "--engine", args.engine,
                           "--input", input_path, "--output", logits_path],
                          capture_output=True, text=True)
    sys.stdout.write(proc.stdout)
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr)
        raise SystemExit(f"parity_runner exited {proc.returncode}")

    logits = np.fromfile(logits_path, dtype=np.float32)
    if logits.size != NUM_CLASSES * LOGIT_H * LOGIT_W:
        raise SystemExit("unexpected logits size from runner")
    argmax = logits.reshape(NUM_CLASSES, LOGIT_H, LOGIT_W).argmax(axis=0).astype(np.uint8)

    fixed = unletterbox(argmax, meta, orig_h, orig_w)
    buggy = nearest_neighbor_resize(argmax, orig_h, orig_w)

    def agree(a, b):
        return float((a == b).mean())

    band = int(np.ceil((meta["pad_x"] // 4) * orig_w / LOGIT_W))

    report = {
        "source": {"width": orig_w, "height": orig_h},
        "letterbox": meta,
        "padding_band_columns": band,
        "agreement_with_published_mask": {
            "fixed_overall": agree(fixed, node_mask),
            "buggy_overall": agree(buggy, node_mask),
            "fixed_left_band": agree(fixed[:, :band], node_mask[:, :band]),
            "buggy_left_band": agree(buggy[:, :band], node_mask[:, :band]),
            "fixed_right_band": agree(fixed[:, orig_w - band:], node_mask[:, orig_w - band:]),
            "buggy_right_band": agree(buggy[:, orig_w - band:], node_mask[:, orig_w - band:]),
            "fixed_vs_buggy_differ": float((fixed != buggy).mean()),
        },
    }

    print()
    print("agreement with the mask the node actually published")
    print(f"  fixed, whole frame : {report['agreement_with_published_mask']['fixed_overall']:.4f}")
    print(f"  buggy, whole frame : {report['agreement_with_published_mask']['buggy_overall']:.4f}")
    print(f"  fixed, left  band  : {report['agreement_with_published_mask']['fixed_left_band']:.4f}"
          f"   (first {band} cols)")
    print(f"  buggy, left  band  : {report['agreement_with_published_mask']['buggy_left_band']:.4f}")
    print(f"  fixed, right band  : {report['agreement_with_published_mask']['fixed_right_band']:.4f}")
    print(f"  buggy, right band  : {report['agreement_with_published_mask']['buggy_right_band']:.4f}")
    print(f"  fixed vs buggy differ on {100*report['agreement_with_published_mask']['fixed_vs_buggy_differ']:.1f}% of pixels")

    a = report["agreement_with_published_mask"]
    verdict = (a["fixed_overall"] > a["buggy_overall"]
               and a["fixed_left_band"] > a["buggy_left_band"])
    report["passed"] = bool(verdict)
    print()
    print("[geom] RESULT:", "PASS - the node matches the restored geometry, not the smeared one"
          if verdict else "FAIL - the node does not match the corrected geometry")

    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w") as fh:
            json.dump(report, fh, indent=2)
        print(f"[geom] report: {args.json}")
    return 0 if verdict else 1


if __name__ == "__main__":
    sys.exit(main())