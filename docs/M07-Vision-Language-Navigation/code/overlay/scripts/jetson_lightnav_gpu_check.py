#!/usr/bin/env python3
"""Gate 1: actual CUDA operations, numerical checks, and runtime provenance."""
import argparse
import contextlib
import importlib.metadata as metadata
import json
import os
import platform
import subprocess
import sys
import time
import traceback
from pathlib import Path


def run(output, python_version="3.11.16", torch_version="2.10.0+cu126", vision_version="0.25.0+cu126", operations_only=False):
    import numpy as np
    import torch
    import torch.nn.functional as functional
    import torchvision
    from torchvision.transforms import v2

    assert sys.version_info[:3] == tuple(map(int, python_version.split("."))), sys.version
    assert platform.machine() == "aarch64"
    assert torch.__version__ == torch_version, torch.__version__
    assert torchvision.__version__ == vision_version, torchvision.__version__
    assert metadata.version("transformers") == "5.8.0"
    assert torch.cuda.is_available(), "CUDA not available; CPU fallback forbidden"
    assert torch.cuda.get_device_capability(0) == (8, 7)
    (output / "runtime-architecture.json").write_text(json.dumps({
        "python": sys.version, "torch": torch.__version__, "vision": torchvision.__version__,
        "cuda": torch.version.cuda, "gpu": torch.cuda.get_device_name(),
        "capability": list(torch.cuda.get_device_capability()), "arch_list": torch.cuda.get_arch_list()}, indent=2))
    torch.set_num_threads(1)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.manual_seed(42)
    rgb = np.arange(3 * 256 * 448, dtype=np.uint8).reshape(256, 448, 3)
    np.testing.assert_array_equal(torch.from_numpy(rgb).numpy(), rgb)
    cpu_a = torch.randn(256, 256)
    cpu_b = torch.randn(256, 256)
    reference = cpu_a @ cpu_b
    assert torch.isfinite(reference).all()
    image = torch.zeros(3, 256, 448, dtype=torch.uint8)
    transformed = v2.Compose([v2.ToDtype(torch.float32, scale=True),
                              v2.Normalize([0.5] * 3, [0.5] * 3)])(image)
    assert transformed.shape == (3, 256, 448)
    checks = {}
    profiler = contextlib.nullcontext() if operations_only else torch.profiler.profile(
        activities=[torch.profiler.ProfilerActivity.CPU, torch.profiler.ProfilerActivity.CUDA])
    with profiler as prof:
        with torch.profiler.record_function("gate1_bf16_matmul"):
            result = cpu_a.cuda().bfloat16() @ cpu_b.cuda().bfloat16()
            torch.cuda.synchronize()
            torch.testing.assert_close(result.float().cpu(), reference, rtol=0.05, atol=0.5)
            checks["bf16_matmul"] = {"finite": bool(torch.isfinite(result).all()),
                                      "max_absolute_error": float((result.float().cpu() - reference).abs().max())}
        with torch.profiler.record_function("gate1_cudnn_patch_conv3d"):
            video = torch.randn(1, 3, 2, 256, 448, device="cuda", dtype=torch.bfloat16)
            weight = torch.randn(8, 3, 2, 16, 16, device="cuda", dtype=torch.bfloat16)
            result = functional.conv3d(video, weight, stride=(2, 16, 16))
            torch.cuda.synchronize()
            assert torch.isfinite(result).all()
            checks["conv3d"] = {"shape": list(result.shape), "finite": True}
        with torch.profiler.record_function("gate1_vision_sdpa"):
            query = torch.randn(1, 16, 448, 64, device="cuda", dtype=torch.bfloat16)
            result = functional.scaled_dot_product_attention(query, query, query)
            torch.cuda.synchronize()
            assert torch.isfinite(result).all()
            checks["vision_sdpa"] = {"shape": list(result.shape), "finite": True}
        with torch.profiler.record_function("gate1_text_gqa_sdpa"):
            query = torch.randn(1, 32, 256, 128, device="cuda", dtype=torch.bfloat16)
            key = torch.randn(1, 8, 256, 128, device="cuda", dtype=torch.bfloat16)
            result = functional.scaled_dot_product_attention(query, key, key, enable_gqa=True)
            torch.cuda.synchronize()
            assert torch.isfinite(result).all()
            checks["text_gqa_sdpa"] = {"shape": list(result.shape), "finite": True}
    (output / "operation-checks.json").write_text(json.dumps(checks, indent=2))
    kernels = []
    if prof is not None:
        trace = output / "cuda-operations.json"
        prof.export_chrome_trace(str(trace))
        events = json.loads(trace.read_text())["traceEvents"]
        kernels = [event for event in events if event.get("cat") == "kernel"]
        assert kernels, "Profiler contains no actual CUDA kernels"
    (output / "loaded-libraries.txt").write_text(Path("/proc/self/maps").read_text())
    torch_dir = Path(torch.__file__).parent
    for library in [torch_dir / "lib/libtorch_cuda.so", torch_dir / "lib/libtorch_cpu.so"]:
        (output / (library.name + ".ldd.txt")).write_text(
            subprocess.run(["ldd", str(library)], text=True, capture_output=True).stdout)
        (output / (library.name + ".file.txt")).write_text(
            subprocess.run(["file", str(library)], text=True, capture_output=True).stdout)
    return {"status": "PASS", "python": sys.version, "torch": torch.__version__,
            "torchvision": torchvision.__version__, "cuda": torch.version.cuda,
            "numpy": np.__version__, "numpy_roundtrip": True,
            "operations_only": operations_only,
            "cudnn": torch.backends.cudnn.version(), "gpu": torch.cuda.get_device_name(),
            "capability": list(torch.cuda.get_device_capability()), "arch_list": torch.cuda.get_arch_list(),
            "checks": checks, "cuda_kernel_count": len(kernels),
            "cuda_kernel_names": sorted({event["name"] for event in kernels}),
            "memory_allocated": torch.cuda.memory_allocated(), "memory_reserved": torch.cuda.memory_reserved()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--python-version", default="3.11.16")
    parser.add_argument("--torch-version", default="2.10.0+cu126")
    parser.add_argument("--vision-version", default="0.25.0+cu126")
    parser.add_argument("--operations-only", action="store_true", help="Numerical checks only; separate profiling proof is required for deployment")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    started = time.time()
    result = {"status": "FAIL"}
    try:
        result = run(args.output, args.python_version, args.torch_version, args.vision_version, args.operations_only)
    except BaseException as error:
        result.update(error=repr(error), traceback=traceback.format_exc())
        print(result["traceback"], flush=True)
    result.update(started=started, ended=time.time(), pid=os.getpid())
    (args.output / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    raise SystemExit(0 if result["status"] == "PASS" else 1)


if __name__ == "__main__":
    main()
