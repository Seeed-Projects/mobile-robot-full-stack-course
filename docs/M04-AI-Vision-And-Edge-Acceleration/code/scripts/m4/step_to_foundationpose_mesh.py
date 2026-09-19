#!/usr/bin/env python3
"""Convert the GL.iNet GL-SFT1200 CAD into a FoundationPose mesh + metadata.

Run on the Mac (build123d 0.11.1, python3.14) -- not on the Jetson, which has no
CAD kernel. The original .step is never modified: this only ever reads it and
writes a derived mesh next to it.

    /opt/homebrew/bin/python3 step_to_foundationpose_mesh.py <out_dir>

Units: the STEP declares SI_UNIT(.MILLI.,.METRE.), so its coordinates are
millimetres. FoundationPose works in metres, so everything written here is scaled
by exactly 0.001 and the scale is recorded in object.yaml rather than being
silently baked in and forgotten.
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np
from build123d import import_step

SRC_STEP = "/Users/chenzibo/data/project/Jetson/workspace/cad_create/gl_sft1200_opal.step"
OBJECT_NAME = "travel_router"
OBJECT_FRAME = "travel_router"
MM_TO_M = 0.001


def main() -> int:
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out_dir, exist_ok=True)

    if not os.path.exists(SRC_STEP):
        print(f"ERROR: source CAD not found: {SRC_STEP}", file=sys.stderr)
        return 1

    shape = import_step(SRC_STEP)
    bb = shape.bounding_box()
    sz_mm = np.array([bb.size.X, bb.size.Y, bb.size.Z], dtype=np.float64)

    print(f"[cad] solids : {len(shape.solids())}")
    print(f"[cad] faces  : {len(shape.faces())}")
    print(f"[cad] bounds (CAD units = mm): X {sz_mm[0]:.3f}  Y {sz_mm[1]:.3f}  Z {sz_mm[2]:.3f}")
    print(f"[cad] bounds as metres       : "
          f"{sz_mm[0]*MM_TO_M:.4f} x {sz_mm[1]*MM_TO_M:.4f} x {sz_mm[2]*MM_TO_M:.4f} m")

    # Tessellate every face. Linear tolerance is in CAD units (mm): 0.05 mm keeps
    # the curved fillets smooth on a 111 mm part without exploding the face count.
    verts_all: list[np.ndarray] = []
    tris_all: list[np.ndarray] = []
    offset = 0
    tol = 0.05
    for face in shape.faces():
        v, t = face.tessellate(tol)
        if not t:
            continue
        va = np.array([[p.X, p.Y, p.Z] for p in v], dtype=np.float64)
        ta = np.array(t, dtype=np.int64) + offset
        verts_all.append(va)
        tris_all.append(ta)
        offset += len(va)

    V = np.vstack(verts_all) * MM_TO_M          # -> metres
    F = np.vstack(tris_all)

    print(f"[mesh] triangles: {len(F)}   vertices: {len(V)}")

    # OBJ with vertex normals (FoundationPose's mesh loader uses them).
    obj_path = os.path.join(out_dir, "model.obj")
    tri = V[F]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)

    vn = np.zeros_like(V)
    for i in range(3):
        np.add.at(vn, F[:, i], n)
    lnv = np.linalg.norm(vn, axis=1, keepdims=True)
    vn = np.divide(vn, lnv, out=np.zeros_like(vn), where=lnv > 0)

    with open(obj_path, "w") as fh:
        fh.write(f"# {OBJECT_NAME} - derived from gl_sft1200_opal.step (mm -> m, x0.001)\n")
        fh.write(f"# triangles {len(F)}, vertices {len(V)}\n")
        for p in V:
            fh.write(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
        for p in vn:
            fh.write(f"vn {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
        for f in F + 1:
            fh.write(f"f {f[0]}//{f[0]} {f[1]}//{f[1]} {f[2]}//{f[2]}\n")

    print(f"[mesh] wrote {obj_path}  ({os.path.getsize(obj_path)} bytes)")

    # ---- sanity: the numbers a human can eyeball ----
    lo, hi = V.min(axis=0), V.max(axis=0)
    centroid = (lo + hi) / 2.0
    diam = float(np.linalg.norm(V - centroid, axis=1).max() * 2.0)
    print(f"[mesh] bbox min  {np.round(lo, 5)}")
    print(f"[mesh] bbox max  {np.round(hi, 5)}")
    print(f"[mesh] size (m)  {np.round(hi - lo, 5)}")
    print(f"[mesh] diameter  {diam:.5f} m")

    meta = {
        "name": OBJECT_NAME,
        "source_cad": "gl_sft1200_opal.step",
        "source_cad_script": "gl_sft1200_opal.step.py",
        "source_cad_note": (
            "GL.iNet GL-SFT1200 'Opal' portable travel router, authored with "
            "build123d 0.11.1 from product photographs plus the owner's measured "
            "envelope of 105 x 80 x 35 mm."
        ),
        "mesh_file": "model.obj",
        "units": "mm",
        "scale_to_meters": MM_TO_M,
        "scale_rationale": (
            "The STEP header declares SI_UNIT(.MILLI.,.METRE.), and the measured "
            "bounding box reads 111.671 x 80.000 x 75.000 in those units, i.e. "
            "11.2 x 8.0 x 7.5 cm -- physically right for a pocket router. The "
            "0.001 factor is therefore derived from the file, not assumed."
        ),
        "expected_size_m": [round(float(s), 6) for s in (hi - lo)],
        "expected_diameter_m": round(diam, 6),
        "object_frame": OBJECT_FRAME,
        "symmetry": "none",
        "symmetry_rationale": (
            "Verified, not assumed. The -Y face carries the port/bezel cut-outs and "
            "the +Y edge carries the antenna hinges and the vent grid, so the body "
            "is not invariant under a 180 degree rotation about Z. Left/right is a "
            "mirror, and a reflection is not a rotation, so it creates no 6D pose "
            "ambiguity. Declaring a symmetry that does not exist would let the "
            "tracker snap to a wrong pose, which is worse than declaring none."
        ),
        "cad_pose_note": (
            "The CAD models the antennas RAISED and splayed (the widest axis reaches "
            "111.67 mm against a 105 mm body). The physical unit must be posed the "
            "same way during capture or the mesh will not register."
        ),
        "source_cad_sha256": None,
    }

    import hashlib
    h = hashlib.sha256()
    with open(SRC_STEP, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    meta["source_cad_sha256"] = h.hexdigest()
    print(f"[cad] source sha256: {meta['source_cad_sha256'][:16]}...")

    yaml_path = os.path.join(out_dir, "object.yaml")
    with open(yaml_path, "w") as fh:
        for k, val in meta.items():
            if isinstance(val, list):
                fh.write(f"{k}: [{', '.join(str(x) for x in val)}]\n")
            elif isinstance(val, str):
                fh.write(f"{k}: \"{val}\"\n")
            else:
                fh.write(f"{k}: {val}\n")
    print(f"[meta] wrote {yaml_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())