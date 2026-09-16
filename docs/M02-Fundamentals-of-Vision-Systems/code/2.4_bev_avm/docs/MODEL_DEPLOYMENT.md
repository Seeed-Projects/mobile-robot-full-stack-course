# Model Deployment (BEVDet → TensorRT 10.3 on Jetson AGX Orin)

## Model lineage

| | |
|---|---|
| Model | `bevdet_one_lt_d` — BEVDet with Depth head + long-term temporal fusion (8 frames), R50 backbone |
| Input | 6 cameras, 256×704 (from 1600×900 source, crop/resize inside `Preprocess` plugin) |
| BEV grid | 128×128 @ 0.8 m ⇒ ±51.2 m, z −5…3 m |
| Classes | 10 nuScenes classes (car, truck, construction_vehicle, bus, trailer, barrier, motorcycle, bicycle, pedestrian, traffic_cone) |
| Training | nuScenes, 20 epochs (Autoware Foundation) |
| License | Apache-2.0 (model); nuScenes data terms apply for the training data |

## Artifacts

| Artifact | Path / URL | Checksum (md5) |
|---|---|---|
| ONNX | `models/onnx/bevdet_one_lt_d.onnx` ← <https://huggingface.co/AutowareFoundation/tensorrt_bevdet> (tag v1.0, sha `391466349a1b39c93b9865364a8088110ca8d97b`) | `c6acd089ad8a1c65da5b95676dbd6757` |
| Engine (on-device built) | `models/engines/bevdet_one_lt_d_r50_256x704_fp16_trt10.3_sm87.engine` | recorded after final rebuild (see `output/inference_result.json`) |
| Model config | `ros2_ws/src/bevdet_vendor/cfgs/bevdet_lt_depth.yaml` | — |
| Run config | `models/configs/configure_phase0.yaml` | — |

Engine naming: `<model>_<backbone>_<WxH>_<precision>_trt<ver>_sm<arch>.engine`

## Hard rules

1. Engines are built **on the target device only** — never copy an x86/different-arch engine.
2. Startup validation (implemented in Phase 2 node; enforced manually in Phase 0):
   - TensorRT version matches build version;
   - GPU arch is sm_87;
   - model config (camera count 6, input 256×704, precision FP16) matches;
   - incompatible ⇒ **hard error**, no silent fallback.
3. Runtime never touches PyTorch. Python is used only for artifact inspection/viz.
4. FP16 by default; FP32 available by changing `Precision` (vendor supports both).

## Rebuild procedure (reproducible)

```bash
# 1. environment gate
scripts/check_environment.sh

# 2. standalone build
cd tools && cmake -S . -B build \
  -DCUDAToolkit_ROOT=/usr/local/cuda-12.6 \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.6/bin/nvcc \
  -DCMAKE_BUILD_TYPE=Release && cmake --build build -j8

# 3. engine build + single-scene inference (engine is auto-built if missing)
cd tools/build && ./bevdemo /home/seeed/workspace/ros2_bev/models/configs/configure_phase0.yaml

# 4. CUDA validation (requires root on this device)
sudo /usr/local/cuda/bin/compute-sanitizer --tool memcheck ./bevdemo \
  /home/seeed/workspace/ros2_bev/models/configs/configure_phase0.yaml
sudo /usr/local/cuda/bin/compute-sanitizer --tool racecheck ./bevdemo \
  /home/seeed/workspace/ros2_bev/models/configs/configure_phase0.yaml

# 5. benchmark
cd tools/build && ./bevdet_bench \
  /home/seeed/workspace/ros2_bev/models/configs/configure_phase0.yaml \
  /home/seeed/workspace/ros2_bev/output/benchmark_phase0.json 50 5
```

## Known quirk (must survive refactors)

The ONNX declares the image input as `int32 [6,3,900,400]`; the plugin
reinterprets that buffer as `uint8 [6,3,900,1600]` (400×4B = 1600 bytes/row,
same bytes). Do NOT "fix" the 400-wide tensor without changing the plugin and
the image-upload path together.