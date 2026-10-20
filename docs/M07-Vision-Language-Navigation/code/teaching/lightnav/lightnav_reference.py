"""Dual-channel grid pointing + SE(2) composition.

Verbatim copy of the released functions this chapter explains, kept here so the teaching
scripts can be re-run offline without installing the model package. Only the imports and
the module docstring differ from the upstream files; every function body is unchanged.

Sources (commit ``3015508b70fb6e30bbd66701e94c9b916dcb8df8``):

* ``src/lightnav/vln_utils.py`` -- ``POINT_GRID_W`` .. ``OPOS_VOCAB_SIZE`` (lines 178-185)
  and ``encode_point_pixel`` / ``decode_point_pixel_center`` (lines 198-212).
* ``src/lightnav/traj_vocab.py`` -- ``wrap_to_pi`` (line 33) and ``compose_to_abs``
  (line 38).

Both source files are recorded with SHA256 in ``example-config.json``. If you edit a
function here, the animation no longer demonstrates the released contract.

Why the grid contract matters: ``encode_point_pixel`` clamps ``u``/``v`` into range
BEFORE quantising, so a point outside the frame collapses onto an edge cell and the id
alone cannot say which happened. Sentinel ids 1297-1299 exist only for APOS; OPOS uses 0
for "not visible".
"""

from __future__ import annotations

import numpy as np

# ── Pointing grid (vln_utils.py, lines 178-185, verbatim) ─────────────────────
POINT_GRID_W: int = 48
POINT_GRID_H: int = 27
POINT_CELLS: int = POINT_GRID_W * POINT_GRID_H  # 1296
APOS_ROTL_ID: int = 1 + POINT_CELLS  # 1297 -- rotate left
APOS_ROTR_ID: int = 2 + POINT_CELLS  # 1298 -- rotate right
APOS_STOP_ID: int = 3 + POINT_CELLS  # 1299 -- path exhausted: stop here
APOS_VOCAB_SIZE: int = 4 + POINT_CELLS  # 1300 (0=none, 1..1296 grid, rot-L/rot-R/stop)
OPOS_VOCAB_SIZE: int = 1 + POINT_CELLS  # 1297 (0=not visible, 1..1296 grid)


def encode_point_pixel(u: float, v: float, width: float, height: float) -> int:
    """(u, v) pixel -> fine-grid point id (1..1296); clamps to the edge cell."""
    col = int(min(max(float(u) / max(float(width), 1.0), 0.0), 0.999999) * POINT_GRID_W)
    row = int(min(max(float(v) / max(float(height), 1.0), 0.0), 0.999999) * POINT_GRID_H)
    return 1 + row * POINT_GRID_W + col


def decode_point_pixel_center(token_id: int, width: float, height: float) -> "tuple[float, float] | None":
    """Fine-grid point id -> (u, v) cell-center pixel, or None for 0/rot/out-of-range."""
    tid = int(token_id)
    if not (1 <= tid <= POINT_CELLS):
        return None
    k = tid - 1
    col, row = k % POINT_GRID_W, k // POINT_GRID_W
    return ((col + 0.5) / POINT_GRID_W * float(width), (row + 0.5) / POINT_GRID_H * float(height))


# ── SE(2) integration (traj_vocab.py, lines 33-104, verbatim) ─────────────────
def wrap_to_pi(theta: np.ndarray) -> np.ndarray:
    """Wrap angles to (-pi, pi]."""
    return (theta + np.pi) % (2.0 * np.pi) - np.pi


def compose_to_abs(deltas: np.ndarray) -> np.ndarray:
    """SE(2) step differentials -> absolute ego waypoints.

    A trajectory chunk is H future waypoints in the chunk-start ego frame, each a
    planar pose ``(x, y, theta)`` (x=forward, y=lateral, theta=heading [rad]). The
    chunk-start pose is the identity and is not stored; the deltas are the per-step
    relative transforms ``delta_j = T_{j-1}^{-1} . T_j`` in the previous step's local
    frame, so the first delta equals wp1's ego pose directly.

    Args:
        deltas: ``(..., H, 3)`` ``(dx_rel, dy_rel, dtheta_rel)``.

    Returns:
        ``(..., H, 3)`` absolute poses ``(x, y, theta)`` in the chunk-start frame
        (float32; accumulation in float64).
    """
    deltas = np.asarray(deltas, dtype=np.float64)
    *batch, H, C = deltas.shape
    if C != 3:
        raise ValueError(f"expected last dim 3, got {C}")

    out = np.empty((*batch, H, 3), dtype=np.float64)
    x = np.zeros(batch, dtype=np.float64)
    y = np.zeros(batch, dtype=np.float64)
    th = np.zeros(batch, dtype=np.float64)
    for j in range(H):
        dx, dy, dth = deltas[..., j, 0], deltas[..., j, 1], deltas[..., j, 2]
        cos_t, sin_t = np.cos(th), np.sin(th)
        x = x + cos_t * dx - sin_t * dy
        y = y + sin_t * dx + cos_t * dy
        th = wrap_to_pi(th + dth)
        out[..., j, 0] = x
        out[..., j, 1] = y
        out[..., j, 2] = th
    return out.astype(np.float32)
