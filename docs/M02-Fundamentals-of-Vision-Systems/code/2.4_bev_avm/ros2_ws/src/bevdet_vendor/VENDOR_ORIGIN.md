# bevdet_vendor — vendored copy

## Origin

| Field | Value |
|---|---|
| URL | https://github.com/autowarefoundation/bevdet_vendor |
| Branch | `bevdet_vendor-ros2` |
| Commit | `143465b355ef3e885174f6ea0bf4eb273ca6bb83` — "chore: remove cudnn dependency (#3)" |
| Cloned | 2026-09-03, shallow (depth 1) |

Note: the repo's branches target consumers on different TensorRT versions. This
commit already includes PR #1 (fp16 + TRT 10.x API migration). We validated it
against TensorRT 10.3.0 / CUDA 12.6 / JetPack 6.2.1 (sm_87).

## Local modifications (this copy)

1. `demo_bevdet.cpp` — constructor calls updated to the 9-arg signature
   (`model_config, N, intrin, rot, trans, onnx_file, engine_file, precision`);
   reads optional `OnnxFile` / `Precision` keys from the configure YAML.
   (Upstream demo was stale, calling a 6-arg ctor that no longer exists.)

No plugin or inference-core changes were required:
- `IPluginV2DynamicExt` + `REGISTER_TENSORRT_PLUGIN` compile and run on TRT 10.3
  (deprecation warnings only).
- Plugins: Preprocess / BEVPool / AlignBEV / GatherBEV — FP16 supported,
  sm_87 verified, no x86-only code found.
- `executeV2` bridge works on TRT 10.3 with the legacy pointer-binding path.

## Build (Phase 0, standalone)

See `../../tools/CMakeLists.txt` — builds `libbevdet_phase0.so`,
`bevdemo`, `bevdet_bench` (no ROS2/ament). ROS2 ament packaging of this
package comes in Phase 2.

## Upstream README caveats

- Its documented desktop/CUDA/TRT versions (11.8 / 8.5.2.2) are stale; this
  commit is TRT-10-ready (see PR #1). All claims re-verified on-device,
  see `docs/PHASE0_REPORT.md`.
- ONNX download links in upstream README (Google Drive/Baidu) are obsolete;
  the official Autoware artifact is hosted on Hugging Face:
  `AutowareFoundation/tensorrt_bevdet` (`bevdet_one_lt_d.onnx`).