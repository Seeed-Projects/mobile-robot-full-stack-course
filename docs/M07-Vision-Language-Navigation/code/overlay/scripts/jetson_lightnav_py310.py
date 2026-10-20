#!/usr/bin/env python3
"""Isolated Seeed JetPack 6.2 / cp310 profile, with user-space NVIDIA runtime."""
import argparse
import hashlib
import json
import os
import subprocess
import tarfile
import time
import urllib.request
from pathlib import Path

import jetson_lightnav as deployment

ROOT = deployment.ROOT
DATA = ROOT / ".jetson-lightnav/py310"
ENV = ROOT / ".venv-lightnav-py310"
PYTHON = "3.10.21"
WHEELS = {
    "torch-2.7.0-cp310-cp310-linux_aarch64.whl": "6eff643c0a7acda92734cc798338f733ff35c7df1a4434576f5ff7c66fc97319",
    "torchvision-0.22.0-cp310-cp310-linux_aarch64.whl": "daabff3a0725996886b92e4b5dd143f5750ef4b181b5c7d01371a9185e8f0402",
}
LIB_NAME = "libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive"
LIB_URL = "https://developer.download.nvidia.com/compute/cusparselt/redist/libcusparse_lt/linux-aarch64/" + LIB_NAME + ".tar.xz"
LIB_SHA = "5426a897c73a9b98a83c4e132d15abc63dc4a00f7e38266e7b82c42cd58a01e1"
LIB_DIR = DATA / "native" / LIB_NAME / "lib"


def sha(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            result.update(block)
    return result.hexdigest()


def native_runtime():
    archive = DATA / "wheels" / (LIB_NAME + ".tar.xz")
    if not archive.exists():
        temporary = archive.with_suffix(".partial")
        last = 0
        with urllib.request.urlopen(LIB_URL, timeout=60) as response, temporary.open("wb") as stream:
            while block := response.read(1024 * 1024):
                stream.write(block)
                if time.monotonic() - last >= 15:
                    remaining = deployment.free_bytes()
                    print(json.dumps({"downloaded": stream.tell(), "free_bytes": remaining}), flush=True)
                    if remaining < deployment.MIN_FREE:
                        raise RuntimeError("Disk reserve below 3 GiB")
                    last = time.monotonic()
        temporary.replace(archive)
    if sha(archive) != LIB_SHA:
        raise RuntimeError("Native Jetson cuSPARSELt archive hash mismatch")
    destination = DATA / "native"
    destination.mkdir(exist_ok=True)
    with tarfile.open(archive) as source:
        for member in source.getmembers():
            target = (destination / member.name).resolve()
            if not target.is_relative_to(destination.resolve()):
                raise RuntimeError("Unsafe native library archive path")
            if not (member.isfile() or member.isdir() or member.issym()):
                raise RuntimeError("Unsupported native archive member")
            if member.issym() and not (target.parent / member.linkname).resolve().is_relative_to(destination.resolve()):
                raise RuntimeError("Unsafe native archive symlink")
        source.extractall(destination)
    library = LIB_DIR / "libcusparseLt.so.0"
    if not library.exists():
        raise RuntimeError("Expected native library missing")
    (DATA / "evidence/native-runtime.json").write_text(json.dumps({
        "source_url": LIB_URL, "archive_sha256": LIB_SHA,
        "library_sha256": sha(library), "library": str(library),
        "system_installation": False}, indent=2))


def runtime_environment():
    return dict(os.environ, LD_LIBRARY_PATH=str(LIB_DIR) + (
        ":" + os.environ["LD_LIBRARY_PATH"] if os.environ.get("LD_LIBRARY_PATH") else ""))


def setup():
    uv = Path("/home/seeed/.local/bin/uv")
    for name, expected in WHEELS.items():
        file = DATA / "wheels" / name
        if not file.exists() or sha(file) != expected:
            raise RuntimeError("Missing or unverified Seeed wheel: " + name)
    (DATA / "evidence/wheel-hashes.json").write_text(json.dumps(WHEELS, indent=2))
    native_runtime()
    if not (ENV / "bin/python").exists():
        deployment.guarded([uv, "venv", "--python", PYTHON, ENV], "setup.log")
    requirements = DATA / "requirements.in"
    requirements.write_text("torch @ " + (DATA / "wheels" / next(iter(WHEELS))).as_uri() + "\n"
                            "torchvision @ " + (DATA / "wheels" / list(WHEELS)[1]).as_uri() + "\n"
                            "transformers==5.8.0\nruff==0.15.0\n"
                            "numpy==1.26.4\nopencv-python-headless==4.11.0.86\n")
    lock = DATA / "requirements.lock"
    if lock.exists() and "numpy==1.26.4" not in lock.read_text():
        lock.replace(DATA / "requirements-initial-numpy2.lock")
    if not lock.exists():
        deployment.guarded([uv, "pip", "compile", "--python", ENV / "bin/python", "--generate-hashes",
                            ROOT / "pyproject.toml", "--extra", "test", "--extra", "video",
                            requirements, "--output-file", lock], "resolve.log")
    for expected in WHEELS.values():
        if expected not in lock.read_text():
            raise RuntimeError("Seeed wheel hash absent from lock")
    deployment.guarded([uv, "pip", "sync", "--python", ENV / "bin/python", "--require-hashes", lock], "setup.log")
    deployment.guarded([uv, "pip", "install", "--python", ENV / "bin/python", "--constraint", lock,
                        "--editable", ROOT], "setup.log")
    deployment.guarded([uv, "pip", "check", "--python", ENV / "bin/python"], "dependencies.log")
    (DATA / "environment.txt").write_text(subprocess.check_output(
        [str(uv), "pip", "freeze", "--python", str(ENV / "bin/python")], text=True))


def check():
    (DATA / "evidence/gate1/result.json").unlink(missing_ok=True)
    output = DATA / "evidence/gate1-numerical"
    output.mkdir(exist_ok=True)
    (output / "result.json").unlink(missing_ok=True)
    os.environ.update(runtime_environment())
    try:
        deployment.guarded([ENV / "bin/python", ROOT / "scripts/jetson_lightnav_gpu_check.py",
                            "--output", output, "--python-version", PYTHON,
                            "--torch-version", "2.7.0", "--vision-version", "0.22.0",
                            "--operations-only"], "gate1.log")
    except BaseException as error:
        if not (output / "result.json").exists():
            (output / "result.json").write_text(json.dumps({"status": "FAIL", "error": repr(error)}))
        raise
    numerical = json.loads((output / "result.json").read_text())
    profiling_file = DATA / "evidence/gate1-privileged/result.json"
    profiling = json.loads(profiling_file.read_text())
    assert numerical["status"] == profiling["status"] == "PASS"
    assert profiling["cuda_kernel_count"] > 0
    for key in ("torch", "torchvision", "cuda", "numpy", "cudnn", "gpu", "capability", "arch_list"):
        assert numerical[key] == profiling[key], "Profiling runtime mismatch: " + key
    for name, expected in WHEELS.items():
        assert sha(DATA / "wheels" / name) == expected
    result = {"status": "PASS", "numerical_record": str(output / "result.json"),
              "profiling_record": str(profiling_file), "profiling_sha256": sha(profiling_file),
              "trace_sha256": sha(profiling_file.parent / "cuda-operations.json"),
              "cuda_kernel_count": profiling["cuda_kernel_count"], "wheel_sha256": WHEELS,
              "profiling_used_temporary_root_process": True}
    gate = DATA / "evidence/gate1"
    gate.mkdir(exist_ok=True)
    (gate / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["setup", "check"])
    args = parser.parse_args()
    for directory in [DATA / "wheels", DATA / "logs", DATA / "evidence"]:
        directory.mkdir(parents=True, exist_ok=True)
    deployment.DATA = DATA
    deployment.ENV = ENV
    globals()[args.action]()


if __name__ == "__main__":
    main()
