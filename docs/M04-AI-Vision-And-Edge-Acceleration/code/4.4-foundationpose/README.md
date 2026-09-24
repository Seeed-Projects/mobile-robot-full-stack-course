# M4.5 Native NVlabs FoundationPose

The physical directory keeps its historical `4.4-foundationpose` name, but it
implements Chapter 4.5: the native NVlabs/PyTorch FoundationPose route. Chapter
4.4 uses the separate `4.4-isaac-ros-foundationpose` directory and Isaac ROS.

## Minimum MVP

The release MVP runs the official recorded Mustard RGB-D sequence without ROS:

```bash
cd /home/seeed/workspace/ros2_bev/modules/m04-ai-vision-and-edge-acceleration
bash scripts/m4/run_m4_5_native_mvp.sh --frames 8
```

The script performs one global `register` call and then calls `track_one` for
the remaining frames. It writes:

```text
output/m4/m45_native_mvp/
├── report.json
├── pose_0000.txt ...
├── frame_0000_pose.png
└── frame_0007_pose.png
```

The compatibility command below invokes the same runner:

```bash
bash scripts/m4/phase0_foundationpose_verify.sh --frames 8
```

The verified 8-frame J501 / AGX Orin run completed with
`M45_NATIVE_MVP_OK`: registration took 11050.90 ms, and seven tracking calls
took 89.37–237.11 ms each (about 120.43 ms average, 8.30 FPS tracking-only
reference). All exported 4x4 matrices contained finite values.

## Runtime Layout

The verified checkout layout is:

```text
/home/seeed/workspace/third_party/FoundationPose/
├── demo_data/mustard0/
├── weights/2023-10-28-18-33-37/{config.yml,model_best.pth}
├── weights/2024-01-11-20-02-45/{config.yml,model_best.pth}
└── mycpp/build/mycpp*.so
```

The course targets commit
`a1b694b83e633c2cb6115b9063d940a687759392`. The Python entry point defaults
to `/home/seeed/miniconda3/envs/py310/bin/python` and can be overridden with
`PYTHON=/path/to/python`. The shell entry point sets `PYTHONNOUSERSITE=1` to
keep binary dependencies on the tested conda NumPy stack. The Python runner
contains a narrow analytic 3x3 inverse compatibility path for JetPack 6.2.1's
CUDA 12.6 cuSOLVER; it does not modify the upstream checkout.

## ROS 2 Package

`ros2/bev_pose` wraps the same lifecycle for a robot perception graph:

```text
RGB + aligned depth + CameraInfo + object mask
  -> FoundationPoseEngine.register / track
  -> /perception/object_pose          geometry_msgs/PoseStamped
  -> /perception/object_poses_3d      vision_msgs/Detection3DArray
  -> /tf                              camera_front -> object
```

Important files:

- `bev_pose/foundationpose_engine.py`: upstream API adapter and lifecycle.
- `bev_pose/foundationpose_node.py`: ROS image conversion and pose publishing.
- `bev_pose/object_mask_node.py`: simple ROI-plus-depth mask helper.
- `bev_pose/mesh_preprocessor.py`: OBJ to self-contained `.npz` geometry.
- `config/pose_estimation.yaml`: topic, mesh, frame, and iteration parameters.

The wrapper uses the real upstream constructor:

```python
FoundationPose(
    model_pts=verts,
    model_normals=normals,
    mesh=mesh,
    scorer=ScorePredictor(),
    refiner=PoseRefinePredictor(),
    glctx=dr.RasterizeCudaContext(),
)
```

Initial registration uses `register(K, rgb, depth, ob_mask, iteration)`. Later
frames use `track_one(rgb, depth, K, iteration)` and the estimator's internal
previous pose.

## Object Mesh

The supplied GL.iNet GL-SFT1200 Opal CAD path converts STEP millimeters to
meters before runtime. Generate and preprocess it with:

```bash
python3 scripts/m4/step_to_foundationpose_mesh.py models/m4/pose
bash scripts/m4/preprocess_mesh.sh
```

The generated metadata records scale, dimensions, source CAD checksum, object
frame, and the required antenna configuration.

## Course Page

See the bilingual Chapter 4.5 pages for the 6D-pose fundamentals, hypothesis,
refine and score stages, register/track distinction, ROS topic contract, and
application examples.
