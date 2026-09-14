# Phase 0 Report — Environment + TensorRT 10.3 Compatibility Gate

> Date: 2026-09-03 · Device: Jetson AGX Orin 32GB (Seeed reComputer J501)
>
> **Verdict: PHASE 0 GATE — PASS** ✅ (all acceptance items verified on-device)

---

## 1. 当前 hardware/software 环境

Full fingerprint: `diagnostics/environment.json` (regenerate: `scripts/check_environment.sh`).

| Item | Measured |
|---|---|
| Board | NVIDIA Jetson AGX Orin **32GB** (Seeed reComputer J501); device-tree label "Jetson AGX Orin Developer Kit" |
| GPU | Orin (nvgpu), **sm_87** (capability 8.7), 30697 MB |
| CPU / RAM | 8× Cortex-A78AE / ~30 GB usable |
| L4T / JetPack | **R36.4.4 / 6.2.1+b38** (Ubuntu 22.04, kernel 5.15.148-tegra) |
| CUDA | **12.6.11** (nvcc V12.6.68 @ `/usr/local/cuda-12.6/bin`) |
| TensorRT | **10.3.0** (`libnvinfer.so.10.3.0`, headers `/usr/include/aarch64-linux-gnu`) |
| cuDNN | 9.3.0 (headers `/usr/include/cudnn_version.h`) — not needed by this model |
| ROS2 | Humble (`/opt/ros/humble`) — not required for Phase 0 |
| Toolchain | GCC 11.4, CMake 3.22, OpenCV 4.8, Eigen 3.4, yaml-cpp 0.7 |
| Power | MODE_40W (`nvpmodel=3`) |
| Storage | 116 GB nvme, ~9.1 GB free (nuScenes-mini not yet downloaded) |
| Network | GitHub + HuggingFace reachable from device |

**Deviation from master spec:** device is the 32GB SKU, not 64GB (spec assumed 64GB).
All Phase 0 results and perf targets still hold; memory margins documented in
`docs/ENVIRONMENT.md`.

## 2. 当前仓库状态

- Repository `ros2_bev` initialized this phase: skeleton (docs/, scripts/, models/,
  calibration/, datasets/, ros2_ws/, tools/, diagnostics/), README, LICENSE, CI-free.
- Commits:
  - `9d88926` feat(phase0): initialize repository skeleton
  - `06c602f` feat(phase0): add environment fingerprint script and docs
  - `8694297` feat(phase0): vendor bevdet_vendor … + standalone build/benchmark/demo config
- Vendored: `ros2_ws/src/bevdet_vendor` @ `143465b` (autowarefoundation, branch
  `bevdet_vendor-ros2`), with local patches documented in its `VENDOR_ORIGIN.md`.
- Artifacts (gitignored, on device): `models/onnx/bevdet_one_lt_d.onnx` (305 MB),
  `models/engines/bevdet_one_lt_d_r50_256x704_fp16_trt10.3_sm87.engine` (153 MB),
  outputs in `output/`.

## 3. BEVDet 候选实现比较

| Candidate | Role | Assessment |
|---|---|---|
| **autowarefoundation/bevdet_vendor** (`bevdet_vendor-ros2` @143465b) | **CHOSEN base** | BEVDet TensorRT C++ port maintained by Autoware Foundation. Already carries TRT-10.x API migration (PR #1), FP16 precision, no cuDNN dependency, no OpenCV dependency, no x86-only code. Ships the 4 plugins we need (Preprocess, BEVPool, AlignBEV, GatherBEV) and a nuScenes dataloader. Compiled on TRT 10.3 **unmodified (warning-only)**. |
| LCH1238/bevdet-tensorrt-cpp (`one` branch) | Reference/upstream | The parent implementation of bevdet_vendor (same plugin code in older form, `Dims32`, `getNbBindings`). Not maintained for TRT 10. Used only to verify that the [6,3,900,400]-int32 image quirk is original behavior, not a vendor bug. |
| HuangJunJie2017/BEVDet (+ LCH1238/BEVDet `export` fork) | Training/export reference | PyTorch training repo; ONNX export script inspected to confirm input specs (`images int32 [6,3,900,400]`, dynamic `M/N` ranks). Not built on device. |
| Autoware `autoware_tensorrt_bevdet` node | Phase 2 reference | ROS2 node consuming bevdet_vendor; message conventions (DetectedObjects, MarkerArray). Will guide Phase 2, not installed now. |

## 4. TensorRT 10.3 compatibility analysis

- `IPluginV2DynamicExt` and `IPluginCreator` **remain available** in TRT 10.3
  (`NvInferRuntime.h`), flagged deprecated but functional — this is the same
  path Autoware ships in production.
- `REGISTER_TENSORRT_PLUGIN`, `executeV2` pointer-binding bridge, and plugin
  (de)serialization all work unchanged in this release.
- ONNX parser handles the opset-13 IR7 model natively; all 4 `bevdet::`-domain
  plugins resolve by name with no registration errors.
- Full engine round-trip (parse → optimize → FP16 build → serialize →
  deserialize → infer) verified on-device.
- Verified quirks: images tensor is `int32 [6,3,900,400]` (byte-equivalent to
  `uint8 [6,3,900,1600]`); documented as a must-preserve invariant.
- Sanitizer note: compute-sanitizer reports "GPU debugging features are
  disabled" for non-root; runs clean under `sudo` (memcheck 0 errors, racecheck
  0 hazards).

## 5. 实际修改清单 (plugin/API patches)

**Required patches: NONE** — the vendored commit was already TRT-10 ready.

Local changes made (non-plugin, documented in `VENDOR_ORIGIN.md` + commits):
1. `demo_bevdet.cpp`: stale 6-arg `BEVDet` ctor call → 9-arg signature; added
   optional `OnnxFile`/`Precision` config keys.
2. Excluded upstream's stale `tools/export_engine.cu` (uses removed `Dims32`);
   engine export runs through `BEVDet::exportEngine()` (already TRT-10 API).
3. Standalone CMake build (`tools/CMakeLists.txt`) for Phase 0 (no ament/ROS).

Full audit table: `docs/TRT10_PORTING.md`.

## 6. Phase 0 implementation plan (executed)

See plan record in repo history; steps executed:
0.1 repo skeleton → 0.2 environment fingerprint → 0.3 vendor audit +
standalone build (one-shot success) → 0.4 official HF ONNX (305MB,
md5 c6acd089) → 0.5 engine build FP16 (2.5 min) → 0.7 real 6-image inference +
sanitizers + benchmark + evidence files. Step 0.6 (nuScenes-mini download) is
deferred to Phase 1 by design — the vendor's bundled scene (`sample0`, real
nuScenes sample n015) already provides real 6-camera data + calibration for the
gate.

## 7. 风险

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| V2 plugin API removed in future TRT | low | engine rebuild blocked on new JetPack | migration plan to IPluginV3 documented; engine cache is valid per-JetPack |
| ~9 GB free disk | high | nuScenes-mini (4 GB) tight | cleanup discipline; dataset on external disk if needed |
| 32GB vs 64GB spec mismatch | fact | memory sizing | documented; engine workspace measured OK |
| Non-root sanitizer limitation | fact | CI must sudo | documented procedure (sudo) |
| Model domain gap on real vehicle cameras | expected | Phase 4 caution | §18 checklist enforced; retraining phased after P4 |
| Stale upstream README versions | medium | confusion | VENDOR_ORIGIN.md + this report pin verified env |

## 8. Phase 0 PASS criteria (gate)

| # | Criterion | Evidence | Status |
|---|---|---|---|
| 1 | TensorRT engine build PASS | engine built on-device, 153 MB, `models/engines/*.engine` | ✅ |
| 2 | TensorRT deserialize PASS | engine deserialized across multiple runs | ✅ |
| 3 | CUDA plugins PASS | all 4 plugins registered/executed, no registration errors | ✅ |
| 4 | 6-image inference PASS | 105 detections; ~match vendor reference (104 boxes, same score distribution) | ✅ |
| 5 | postprocess PASS | boxes decoded in lidar/ego frames; scores/geometry sane vs reference | ✅ |
| 6 | no CUDA illegal memory access | compute-sanitizer memcheck **0 errors** | ✅ |
| 7 | no plugin registration error | builder/runtime logs clean | ✅ |
| 8 | no segmentation fault | every run clean; racecheck 0 hazards | ✅ |
| 9 | Artifacts saved | `inference_result.json`, `inference_debug.png`, `benchmark_phase0.json` in `output/` | ✅ |
| 10 | Benchmark protocol honored | 5 warmup + 50 samples; mean 42.08 / median 42.06 / p95 42.24 / p99 42.24 ms ≈ **23.8 FPS** (MODE_40W) | ✅ |

## Conclusion

**Phase 0 gate PASSED** on JetPack 6.2.1 native environment with TensorRT 10.3:
BEVDet (R50, depth + long-term, 256×704) FP16 engine built and validated on
the Jetson itself, real 6-image inference, clean CUDA sanitizers, documented
benchmark. Per the master discipline, **Phase 1 (nuScenes offline validation)
may now begin**; no ROS development was performed in Phase 0.