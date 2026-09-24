#!/usr/bin/env python3
"""Run the smallest useful native FoundationPose demonstration.

The MVP uses NVlabs' recorded Mustard RGB-D sequence. It keeps the lifecycle
visible: register on frame zero, then track the following frames. The output
directory contains poses, annotated images, and a JSON report.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch


def _patch_cuda_3x3_inverse() -> None:
    """Avoid the missing cuSOLVER 3x3 inverse symbol on JetPack 6.2.1.

    The tested Jetson PyTorch wheel can otherwise load CUDA kernels, but its
    ``Tensor.inverse`` dispatch asks for a newer cuSOLVER symbol than the
    system CUDA 12.6 library exports. FoundationPose only uses this operation
    for 3x3 camera/crop matrices, so an adjugate formula keeps the MVP local
    and does not alter the upstream FoundationPose checkout.
    """
    original_inverse = torch.Tensor.inverse
    original_linalg_inverse = torch.linalg.inv

    def inverse_3x3(matrix: torch.Tensor) -> torch.Tensor:
        a, b, c = matrix[..., 0, 0], matrix[..., 0, 1], matrix[..., 0, 2]
        d, e, f = matrix[..., 1, 0], matrix[..., 1, 1], matrix[..., 1, 2]
        g, h, i = matrix[..., 2, 0], matrix[..., 2, 1], matrix[..., 2, 2]
        det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
        cof = torch.stack(
            (
                e * i - f * h,
                c * h - b * i,
                b * f - c * e,
                f * g - d * i,
                a * i - c * g,
                c * d - a * f,
                d * h - e * g,
                b * g - a * h,
                a * e - b * d,
            ),
            dim=-1,
        ).reshape(matrix.shape)
        return cof / det.unsqueeze(-1).unsqueeze(-1)

    def inverse(self: torch.Tensor) -> torch.Tensor:
        if self.is_cuda and self.shape[-2:] == (3, 3):
            return inverse_3x3(self)
        return original_inverse(self)

    def linalg_inverse(matrix: torch.Tensor, *, out=None) -> torch.Tensor:
        if matrix.is_cuda and matrix.shape[-2:] == (3, 3):
            result = inverse_3x3(matrix)
        else:
            result = original_linalg_inverse(matrix)
        if out is not None:
            out.copy_(result)
            return out
        return result

    torch.Tensor.inverse = inverse
    torch.linalg.inv = linalg_inverse


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--foundationpose-dir",
        default=os.environ.get("FOUNDATIONPOSE_DIR", ""),
    )
    parser.add_argument("--scene", default="")
    parser.add_argument("--mesh", default="")
    parser.add_argument("--output", default="output/m4/m45_native_mvp")
    parser.add_argument("--frames", type=int, default=8)
    parser.add_argument("--register-iterations", type=int, default=5)
    parser.add_argument("--track-iterations", type=int, default=2)
    args = parser.parse_args()

    if not args.foundationpose_dir:
        raise SystemExit("FOUNDATIONPOSE_DIR is required; export it or pass --foundationpose-dir")
    fp_dir = Path(args.foundationpose_dir).expanduser().resolve()
    scene_dir = Path(args.scene or fp_dir / "demo_data/mustard0").expanduser().resolve()
    mesh_file = Path(
        args.mesh or scene_dir / "mesh/textured_simple.obj"
    ).expanduser().resolve()
    output_dir = Path(args.output).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if not fp_dir.is_dir():
        raise SystemExit(f"FoundationPose checkout not found: {fp_dir}")
    if not scene_dir.is_dir() or not mesh_file.is_file():
        raise SystemExit(
            f"Mustard scene or mesh not found: {scene_dir} / {mesh_file}"
        )

    sys.path.insert(0, str(fp_dir))
    _patch_cuda_3x3_inverse()
    from datareader import YcbineoatReader
    from estimater import FoundationPose, PoseRefinePredictor, ScorePredictor
    import nvdiffrast.torch as dr
    import trimesh
    from Utils import draw_posed_3d_box, draw_xyz_axis

    mesh = trimesh.load(str(mesh_file), force="mesh", process=False)
    to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
    bbox = np.stack([-extents / 2, extents / 2], axis=0).reshape(2, 3)
    reader = YcbineoatReader(
        video_dir=str(scene_dir), shorter_side=None, zfar=np.inf
    )
    frame_count = min(max(args.frames, 1), len(reader.color_files))
    estimator = FoundationPose(
        model_pts=mesh.vertices,
        model_normals=mesh.vertex_normals,
        mesh=mesh,
        scorer=ScorePredictor(),
        refiner=PoseRefinePredictor(),
        glctx=dr.RasterizeCudaContext(),
        debug=1,
        debug_dir=str(output_dir / "debug"),
    )

    register_ms = None
    track_ms: list[float] = []
    poses: list[np.ndarray] = []
    for index in range(frame_count):
        color = reader.get_color(index)
        depth = reader.get_depth(index)
        started = time.perf_counter()
        if index == 0:
            pose = estimator.register(
                K=reader.K,
                rgb=color,
                depth=depth,
                ob_mask=reader.get_mask(index),
                iteration=args.register_iterations,
            )
            register_ms = (time.perf_counter() - started) * 1000.0
        else:
            pose = estimator.track_one(
                rgb=color,
                depth=depth,
                K=reader.K,
                iteration=args.track_iterations,
            )
            track_ms.append((time.perf_counter() - started) * 1000.0)

        pose = np.asarray(pose, dtype=np.float32).reshape(4, 4)
        if not np.isfinite(pose).all():
            raise RuntimeError(f"frame {index} produced a non-finite pose")
        poses.append(pose)
        np.savetxt(output_dir / f"pose_{index:04d}.txt", pose, fmt="%.8f")

        if index in (0, frame_count - 1):
            centered_pose = pose @ np.linalg.inv(to_origin)
            visualization = draw_posed_3d_box(
                reader.K, img=color, ob_in_cam=centered_pose, bbox=bbox
            )
            visualization = draw_xyz_axis(
                visualization,
                ob_in_cam=centered_pose,
                scale=0.1,
                K=reader.K,
                thickness=3,
                transparency=0,
                is_input_rgb=True,
            )
            cv2.imwrite(
                str(output_dir / f"frame_{index:04d}_pose.png"),
                visualization[..., ::-1],
            )

    report = {
        "foundationpose_dir": str(fp_dir),
        "scene": str(scene_dir),
        "mesh": str(mesh_file),
        "frames": frame_count,
        "register_ms": register_ms,
        "track_ms": track_ms,
        "track_fps": (
            1000.0 / float(np.mean(track_ms)) if track_ms else None
        ),
        "first_pose": poses[0].tolist(),
        "last_pose": poses[-1].tolist(),
    }
    (output_dir / "report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    print(f"M45_NATIVE_MVP_OK output={output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
