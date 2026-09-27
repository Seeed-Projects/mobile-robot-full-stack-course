"""bev_pose.mesh_preprocessor — offline OBJ → .npz preprocessor.

This is invoked ONCE per object, BEFORE the FoundationPose node is started.
Output goes to models/m4/pose/processed/<obj_name>.npz and contains:

    vertices, faces, normals, bbox_corners,
    centroid, bounds_min, bounds_max, diameter,
    source_obj, frame_id

Running this offline keeps OBJ parsing OUT of the runtime hot path:
the pose visualizer and the engine both load the .npz file at startup,
not the .obj. This avoids a 200ms+ Python-side parse on every track call.

Usage:
    python3 -m bev_pose.mesh_preprocessor \\
        --obj models/m4/pose/obj_models/cup.obj \\
        --out models/m4/pose/processed/cup.npz \\
        --frame-id cup
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Optional

import numpy as np

from .types import PreprocessedMesh


def preprocess(obj_path: str, frame_id: str = 'object') -> PreprocessedMesh:
    """Parse OBJ → PreprocessedMesh.

    Uses trimesh for OBJ parsing. trimesh is allowed as an *offline-only*
    dependency: it is NOT used in the runtime FoundationPose node.
    """
    if not os.path.isfile(obj_path):
        raise FileNotFoundError(f"OBJ file not found: {obj_path}")

    mesh = PreprocessedMesh.from_obj(obj_path, frame_id=frame_id)
    return mesh


def save(mesh: PreprocessedMesh, out_path: str) -> None:
    """Persist PreprocessedMesh to a single .npz."""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    np.savez(out_path, **mesh.as_dict())
    print(f"[mesh_preprocessor] wrote {out_path}")
    print(f"  vertices       : {mesh.vertices.shape}")
    print(f"  faces          : {mesh.faces.shape}")
    print(f"  bbox_corners   : {mesh.bbox_corners.shape}")
    print(f"  diameter (m)   : {mesh.diameter:.4f}")
    print(f"  bounds_min     : {mesh.bounds_min}")
    print(f"  bounds_max     : {mesh.bounds_max}")


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description='Offline OBJ → .npz preprocessor for NVlabs/FoundationPose.')
    parser.add_argument('--obj', required=True,
                        help='path to input .obj file')
    parser.add_argument('--out', required=True,
                        help='path to output .npz file')
    parser.add_argument('--frame-id', default='object',
                        help='suggested tf2 child frame id (default: object)')
    args = parser.parse_args(argv)

    try:
        mesh = preprocess(args.obj, frame_id=args.frame_id)
        save(mesh, args.out)
        return 0
    except Exception as e:
        print(f"[mesh_preprocessor] FAIL: {e}", file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
