# TensorRT 10.3 Porting Notes

Status: **VERIFIED ON DEVICE** — no patch required beyond the upstream PR #1 work
already present in `bevdet_vendor@143465b` (`bevdet_vendor-ros2` branch).

## Verification environment

| | |
|---|---|
| Device | Jetson AGX Orin 32GB (sm_87) |
| JetPack / L4T | 6.2.1+b38 / R36.4.4 |
| CUDA | 12.6.11 (nvcc V12.6.68) |
| TensorRT | 10.3.0 (`libnvinfer_plugin.so.10`) |
| Headers | `/usr/include/aarch64-linux-gnu/{NvInfer.h,NvInferRuntime.h,NvOnnxParser.h}` |

## API audit table

| Concern | Upstream (TRT 8.x era) | TRT 10.3 state | Action taken |
|---|---|---|---|
| Plugin base class | `IPluginV2DynamicExt` | Still present (`NvInferRuntime.h`), marked `TRT_DEPRECATED` but fully functional; `IPluginV3` also available | **None** — deprecated-V2 path validated end-to-end |
| Plugin creator registration | `REGISTER_TENSORRT_PLUGIN(Creator)` | Macro still in `NvInferRuntime.h` (deprecated) | **None** |
| Plugin enqueue | `enqueueV2` / renames | Code already on `int32_t enqueue(const PluginTensorDesc*, ...)` + `using nvinfer1::IPluginV2::enqueue` | **None** (was fixed upstream in PR #1) |
| Engine tensor enumeration | `getNbBindings` / `getBindingDimensions(i)` | `getNbIOTensors()` / `getTensorShape(name)` | **None** (fixed upstream in PR #1) |
| Dims type | `Dims32` | `Dims` (int64 payload) | **None** in core; `tools/export_engine.cu` (upstream helper, not built here) still uses `Dims32` and is excluded from our build |
| BuilderConfig workspace | `maxWorkspaceSize` | `setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 12GB)` in vendor `exportEngine()` | **None** — already new API |
| Context execution | `executeV2(void**)` w/ legacy bindings | Legacy binding array still works (deprecated); `executeV3`/`setTensorAddress` are the modern path | **None for Phase 0**; migration to `executeV3`+`setTensorAddress` planned with ROS integration (Phase 2) |
| Plugin serialization | `IPluginV2DynamicExt` serialize | Unchanged | **None** — engine round-trips serialize→build→deserialize verified |
| ONNX parser | `nvonnxparser::createParser` | `libnvonnxparser.so.10` present, opset-13 model parses | **None** |
| sm_87 codegen | gencode flags | CUDA 12.6 arch 87 | CMake `CMAKE_CUDA_ARCHITECTURES=87` |

Deprecation warnings observed: `IPluginV2DynamicExt/ICommand` class-level deprecations;
suppressed via `-Wno-deprecated-declarations` (same approach autoware uses).

## Plugin list (shipped in ONNX, linked into libbevdet_phase0.so)

| Plugin | Domain | FP16 | sm_87 | x86-only code | Status |
|---|---|---|---|---|---|
| Preprocess | `bevdet::` | yes | yes | none | PASS |
| BEVPool (V2 kernel) | `bevdet::` | yes | yes | none | PASS |
| AlignBEV | `bevdet::` | yes | yes | none | PASS |
| GatherBEV | `bevdet::` | yes | yes | none | PASS |

## Test results (on-device, all real runs)

- [x] ONNX parse (`bevdet_one_lt_d.onnx`, opset 13): **PASS**
- [x] Engine build FP16 batch 1: **PASS** (~2.5 min on Orin)
- [x] Engine deserialize: **PASS** (repeated across runs)
- [x] 6-image inference: **PASS** — 105 boxes vs upstream reference 104 (score histograms match)
- [x] compute-sanitizer memcheck: **0 errors** (must run as root; as non-root the device
      reports "GPU debugging features are disabled" — permission behavior, not app fault)
- [x] compute-sanitizer racecheck: **0 hazards**
- [x] Benchmark: mean/median 42.1 ms, p95 42.2 ms, p99 42.2 ms (50 samples, 5 warmup,
      MODE_40W) ⇒ ~23.8 FPS inference

## Risks archived (not hit)

1. *"IPluginV2DynamicExt may not exist in TRT 10.3"* — it does (deprecated but
   functional). Adding an IPluginV3 rewrite is therefore NOT a Phase 0 requirement.
2. *"BuilderConfig workspaceSize breaks"* — vendor already on `setMemoryPoolLimit`.
3. *"enqueueV2 removed"* — already migrated upstream.
4. *Dims32 in `tools/export_engine.cu`* — stale helper; not part of our build.
   If we ever re-enable it, replace `Dims32` → `Dims`.

## Decision log

- Keep V2 plugin API for Phase 0–2 (matches autoware's production path).
- Revisit IPluginV3 migration only if deprecation blocks engine rebuilds on a
  future TRT release (documented in MODEL_DEPLOYMENT.md).
- `executeV3` + `setTensorAddress` migration is scheduled with Phase 2
  (intra-process / minimal-copy input path) — not required for correctness today.