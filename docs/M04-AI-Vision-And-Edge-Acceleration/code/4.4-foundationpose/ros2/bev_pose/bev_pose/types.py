"""bev_pose.types — typed structures shared by engine + node + visualizer.

The preprocessed mesh (.npz) is intentionally self-contained so that the
visualizer does not need to re-parse the OBJ at every frame:

    vertices          : (N, 3) float32  — model space
    faces             : (F, 3) int32    — triangle indices
    normals           : (N, 3) float32  — vertex normals
    bbox_corners      : (8, 3) float32  — axis-aligned bounding-box corners
    centroid          : (3,)   float32  — model centroid
    bounds_min        : (3,)   float32
    bounds_max        : (3,)   float32
    diameter          : float           — bounding-sphere diameter (meters)
    source_obj        : str             — original OBJ path (for reference only)
    frame_id          : str             — suggested TF child frame name
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class PreprocessedMesh:
    """A mesh, precomputed offline so the runtime is geometry-free.

    The data is intentionally stored as a flat dict-of-arrays so that
    `np.load(...)` can reconstruct it without losing dtype information.
    """

    vertices: np.ndarray            # (N, 3) float32
    faces: np.ndarray               # (F, 3) int32
    normals: np.ndarray             # (N, 3) float32
    bbox_corners: np.ndarray        # (8, 3) float32
    centroid: np.ndarray            # (3,) float32
    bounds_min: np.ndarray          # (3,) float32
    bounds_max: np.ndarray          # (3,) float32
    diameter: float
    source_obj: str = ''
    frame_id: str = 'object'

    def as_dict(self) -> dict:
        """Convert to a dict-of-arrays suitable for np.savez."""
        return {
            'vertices': self.vertices,
            'faces': self.faces,
            'normals': self.normals,
            'bbox_corners': self.bbox_corners,
            'centroid': self.centroid,
            'bounds_min': self.bounds_min,
            'bounds_max': self.bounds_max,
            'diameter': np.asarray(self.diameter, dtype=np.float32),
            'source_obj': np.asarray(self.source_obj),
            'frame_id': np.asarray(self.frame_id),
        }

    @classmethod
    def from_npz(cls, path: str) -> 'PreprocessedMesh':
        """Load from a .npz file produced by `mesh_preprocessor.main()`."""
        data = np.load(path, allow_pickle=True)
        return cls(
            vertices=data['vertices'].astype(np.float32),
            faces=data['faces'].astype(np.int32),
            normals=data['normals'].astype(np.float32),
            bbox_corners=data['bbox_corners'].astype(np.float32),
            centroid=data['centroid'].astype(np.float32),
            bounds_min=data['bounds_min'].astype(np.float32),
            bounds_max=data['bounds_max'].astype(np.float32),
            diameter=float(data['diameter']),
            source_obj=str(data['source_obj']) if 'source_obj' in data.files else '',
            frame_id=str(data['frame_id']) if 'frame_id' in data.files else 'object',
        )

    @classmethod
    def from_obj(cls, obj_path: str, frame_id: str = 'object') -> 'PreprocessedMesh':
        """Build a PreprocessedMesh from an OBJ file using trimesh.

        This runs OFFLINE only; the runtime pipeline never calls this.
        """
        import trimesh
        mesh = trimesh.load(obj_path, force='mesh', process=False)
        if not isinstance(mesh, trimesh.Trimesh):
            # Some loaders return a Scene with one geometry
            mesh = next(iter(mesh.geometry.values()))

        v = np.asarray(mesh.vertices, dtype=np.float32)
        f = np.asarray(mesh.faces, dtype=np.int32)
        n = np.asarray(mesh.vertex_normals, dtype=np.float32) if hasattr(mesh, 'vertex_normals') \
            else np.zeros_like(v, dtype=np.float32)
        bmin = v.min(axis=0)
        bmax = v.max(axis=0)
        # 8 axis-aligned bounding-box corners, centred at centroid
        cx, cy, cz = (bmax + bmin) / 2.0
        ex, ey, ez = (bmax - bmin) / 2.0
        bbox = np.asarray([
            [cx - ex, cy - ey, cz - ez],
            [cx + ex, cy - ey, cz - ez],
            [cx + ex, cy + ey, cz - ez],
            [cx - ex, cy + ey, cz - ez],
            [cx - ex, cy - ey, cz + ez],
            [cx + ex, cy - ey, cz + ez],
            [cx + ex, cy + ey, cz + ez],
            [cx - ex, cy + ey, cz + ez],
        ], dtype=np.float32)

        centroid = np.asarray([cx, cy, cz], dtype=np.float32)
        # Bounding-sphere diameter (max distance from centroid to a vertex)
        span = float(np.linalg.norm(v - centroid, axis=1).max() * 2.0)

        return cls(
            vertices=v,
            faces=f,
            normals=n,
            bbox_corners=bbox,
            centroid=centroid,
            bounds_min=bmin.astype(np.float32),
            bounds_max=bmax.astype(np.float32),
            diameter=span,
            source_obj=obj_path,
            frame_id=frame_id,
        )


@dataclass
class PoseResult:
    """Single-object pose estimate with the metadata the node logs."""

    pose: np.ndarray                # (4, 4) float32 — camera_T_object
    score: float = 0.0
    latency_ms: float = 0.0
    is_registered: bool = False
    is_tracked: bool = False
    frame_index: int = 0            # since registration

    def as_pose_stamped(self, header_stamp, header_frame: str, child_frame: str) -> 'PoseStamped':
        """Convert to a geometry_msgs/PoseStamped.

        Import is local to keep this module import-cheap when FoundationPose
        is not available yet.
        """
        from geometry_msgs.msg import PoseStamped
        from geometry_msgs.msg import Pose, Point, Quaternion
        from tf_transformations import quaternion_from_matrix

        qx, qy, qz, qw = quaternion_from_matrix(self.pose)
        msg = PoseStamped()
        msg.header.stamp = header_stamp
        msg.header.frame_id = header_frame
        msg.pose = Pose(
            position=Point(
                x=float(self.pose[0, 3]),
                y=float(self.pose[1, 3]),
                z=float(self.pose[2, 3]),
            ),
            orientation=Quaternion(x=float(qx), y=float(qy),
                                   z=float(qz), w=float(qw)),
        )
        return msg


@dataclass
class EngineConfig:
    """Runtime configuration for FoundationPoseEngine."""

    model_dir: str
    mesh_npz_path: str
    scorer_threshold: float = 0.3
    refiner_iterations: int = 5
    device: str = 'cuda:0'
    enforce_register_first: bool = True
