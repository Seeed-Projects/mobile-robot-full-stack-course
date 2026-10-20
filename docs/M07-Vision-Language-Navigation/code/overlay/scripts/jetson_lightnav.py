#!/usr/bin/env python3
"""Isolated, disk-guarded LightNav deployment. Never synchronizes gait environments."""
import argparse
import json
import os
import re
import signal
import socket
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / ".jetson-lightnav"
ENV = ROOT / ".venv-lightnav"
PROFILE = "official311"
MODEL = ROOT / "checkpoints" / "LightNav-0"
REVISION = "826dc5fbfa37afa8293d2e336d329b6ffc0bfb64"
MIN_FREE = 3 * 2**30
TORCH_URL = "https://download-r2.pytorch.org/whl/cu126/torch-2.10.0%2Bcu126-cp311-cp311-manylinux_2_28_aarch64.whl"
VISION_URL = "https://download-r2.pytorch.org/whl/cu126/torchvision-0.25.0%2Bcu126-cp311-cp311-manylinux_2_28_aarch64.whl"


def free_bytes():
    stat = os.statvfs(ROOT)
    return stat.f_bavail * stat.f_frsize


def guarded(args, filename):
    """Terminate only our child process group when disk space falls below reserve."""
    env = dict(os.environ, UV_CACHE_DIR=str(DATA / "uv-cache"), PYTHONUNBUFFERED="1")
    with (DATA / "logs" / filename).open("a", buffering=1) as log:
        log.write(json.dumps({"started": time.time(), "argv": list(map(str, args))}) + "\n")
        child = subprocess.Popen(list(map(str, args)), cwd=ROOT, env=env,
                                 stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while child.poll() is None:
                space = free_bytes()
                with (DATA / "space.jsonl").open("a") as space_log:
                    space_log.write(json.dumps({"time": time.time(), "free_bytes": space,
                                                "child_pid": child.pid}) + "\n")
                if space < MIN_FREE:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                    raise RuntimeError("Disk reserve below 3 GiB; our operation stopped")
                try:
                    child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    pass
        except BaseException:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
            raise
        if child.returncode:
            raise RuntimeError(f"Command exited {child.returncode}; see {log.name}")


def setup():
    uv = Path(os.environ.get("UV_BIN", "/home/seeed/.local/bin/uv"))
    if not (ENV / "bin/python").exists():
        if free_bytes() < 24 * 2**30:
            raise RuntimeError("Fresh deployment requires at least 24 GiB free")
        guarded([uv, "venv", "--python", "3.11.16", ENV], "setup.log")
    requirements = DATA / "requirements.in"
    guarded(["python3", ROOT / "scripts/jetson_cusparselt_wheel.py"], "cusparselt-repair.log")
    repair = json.loads((DATA / "evidence/cusparselt-wheel-repair.json").read_text())
    wheel = DATA / "wheels/repaired/nvidia_cusparselt_cu12-0.7.1-py3-none-manylinux2014_aarch64.whl"
    wheel_requirement = "nvidia-cusparselt-cu12 @ " + wheel.as_uri()
    requirements.write_text(f"torch @ {TORCH_URL}\ntorchvision @ {VISION_URL}\n"
                            "transformers==5.8.0\nruff==0.15.0\n" + wheel_requirement + "\n")
    lock = DATA / "requirements.lock"
    if not lock.exists():
        guarded([uv, "pip", "compile", "--python", ENV / "bin/python", "--generate-hashes",
                 "pyproject.toml", "--extra", "test", "--extra", "video",
                 requirements, "--output-file", lock], "resolve.log")
    else:
        # Preserve the existing resolution; replace only this verified metadata repair.
        content = lock.read_text()
        content, count = re.subn(r"^nvidia-cusparselt-cu12(?:==0\.7\.1| @ [^\n]+) \\\n(?:    --hash=sha256:[0-9a-f]+(?: \\)?\n)+",
                                 lambda _: wheel_requirement + " \\\n    --hash=sha256:" + repair["repaired_sha256"] + "\n",
                                 content, flags=re.MULTILINE)
        if count != 1:
            raise RuntimeError("Cannot uniquely replace cuSPARSELt lock entry")
        lock.write_text(content)
    for expected in ["3a5fb967ffb53ffa0d2579c9819491cfc36c557040de6fdeabcfcfb45df019bc",
                     "1bacf3a2c4c5d77615be7fa77b39976aace0dfeeb580a549776cae192790401d"]:
        if expected not in lock.read_text():
            raise RuntimeError("Official torch/vision SHA256 absent from resolved lock")
    guarded([uv, "pip", "sync", "--python", ENV / "bin/python", "--require-hashes", lock], "setup.log")
    guarded([uv, "pip", "install", "--python", ENV / "bin/python", "--constraint", lock,
             "--editable", "."], "setup.log")
    guarded([uv, "pip", "check", "--python", ENV / "bin/python"], "dependencies.log")
    result = subprocess.check_output([str(uv), "pip", "freeze", "--python", str(ENV / "bin/python")], text=True)
    (DATA / "environment.txt").write_text(result)


def check():
    result_file = DATA / "evidence/gate1/result.json"
    result_file.parent.mkdir(parents=True, exist_ok=True)
    result_file.unlink(missing_ok=True)
    try:
        guarded([ENV / "bin/python", ROOT / "scripts/jetson_lightnav_gpu_check.py",
                 "--output", DATA / "evidence/gate1"], "gate1.log")
    except BaseException as error:
        if not result_file.exists():
            result_file.write_text(json.dumps({"status": "FAIL", "error": repr(error),
                                               "time": time.time()}))
        raise
    print((DATA / "evidence/gate1/result.json").read_text())


def download():
    result = gate_file()
    if not result.exists() or json.loads(result.read_text()).get("status") != "PASS":
        raise RuntimeError("Gate 1 must PASS before model download")
    if not (MODEL / "model-00001-of-00001.safetensors").exists() and free_bytes() < 13 * 2**30:
        raise RuntimeError("Need 10 GiB model budget plus 3 GiB reserve")
    guarded([ENV / "bin/python", ROOT / "scripts/jetson_lightnav_download.py"], "download.log")


def identity(kind="server"):
    path = DATA / f"{kind}.identity.json"
    if not path.exists():
        return None
    value = json.loads(path.read_text())
    proc = Path(f"/proc/{value['pid']}")
    try:
        fields = (proc / "stat").read_text().rsplit(")", 1)[1].split()
        argv = (proc / "cmdline").read_bytes().split(b"\0")
    except FileNotFoundError:
        return None
    if fields[0] == "Z" or not any(argv):
        # Linux may clear cmdline during process exit before publishing zombie state.
        # No executable remains to signal; never match a different nonempty argv.
        return None
    expected = Path(value["argv"][0]) if value.get("argv") else (
        ENV / "bin/lightnav-serve" if kind == "server" else ROOT / "mujoco_demo/.venv-cpu/bin/vln-mujoco")
    allowed = {ROOT / ".venv-lightnav/bin/lightnav-serve", ROOT / ".venv-lightnav-py310/bin/lightnav-serve"} if kind == "server" else {ROOT / "mujoco_demo/.venv-cpu/bin/vln-mujoco"}
    if expected not in allowed:
        raise RuntimeError("Unexpected managed executable; refusing to signal process")
    if fields[19] != value["starttime"]:
        # A stale PID from a previous boot must never be signalled.
        return None
    if str(expected).encode() not in argv:
        raise RuntimeError("PID identity mismatch; refusing to signal process")
    return value


def start():
    if identity():
        raise RuntimeError("Managed LightNav already running")
    gate = gate_file()
    if not gate.exists() or json.loads(gate.read_text()).get("status") != "PASS":
        raise RuntimeError("Gate 1 must PASS before CUDA model startup")
    if not (DATA / "model-manifest.json").exists():
        raise RuntimeError("Model must be downloaded and verified first")
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("127.0.0.1", 8050))
    ready = DATA / "server.ready"
    ready.unlink(missing_ok=True)
    log_path = DATA / "logs" / f"server-{time.strftime('%Y%m%d-%H%M%S')}.log"
    args = [str(ENV / "bin/lightnav-serve"), "--task", "vln", "--backend", "hf",
            "--device", "cuda", "--model_path", str(MODEL), "--host", "127.0.0.1",
            "--port", "8050", "--ready_file", str(ready), "--record_dir", str(DATA / "episodes")]
    with log_path.open("w") as log:
        proc = subprocess.Popen(args, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, start_new_session=True,
                                env=dict(os.environ, PYTHONUNBUFFERED="1", HF_HUB_OFFLINE="1",
                                         HF_HOME=str(DATA / "hf-cache"), TORCH_HOME=str(DATA / "torch-cache"),
                                         CUDA_CACHE_PATH=str(DATA / "cuda-cache"),
                                         OMP_NUM_THREADS="1", MKL_NUM_THREADS="1", LIGHTNAV_STRICT_RVQ="1",
                                         LIGHTNAV_REQUEST_LEDGER=str(DATA / "requests.jsonl")))
    fields = Path(f"/proc/{proc.pid}/stat").read_text().rsplit(")", 1)[1].split()
    (DATA / "server.identity.json").write_text(json.dumps({"pid": proc.pid, "starttime": fields[19],
                                                         "argv": args, "log": str(log_path)}))
    print(json.dumps({"pid": proc.pid, "log": str(log_path), "ready_file": str(ready)}))


def stop(kind="server"):
    value = identity(kind)
    if value:
        os.kill(value["pid"], signal.SIGTERM)
        print(f"Termination requested for managed PID {value['pid']}")
        deadline = time.monotonic() + 30
        while identity(kind):
            if time.monotonic() > deadline:
                raise RuntimeError("Owned process did not terminate in 30 seconds")
            time.sleep(0.1)
        (DATA / f"{kind}.identity.json").unlink(missing_ok=True)
        if kind == "server":
            (DATA / "server.ready").unlink(missing_ok=True)
    else:
        print("No managed server running")


def start_demo(robot, record_dir=None):
    if identity("demo"):
        raise RuntimeError("Managed demo already running")
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("127.0.0.1", 8088))
    demo_root = ROOT / "mujoco_demo"
    calibration = DATA / "evidence/performance/timeout.json"
    measurement = json.loads(calibration.read_text()) if calibration.exists() else None
    if measurement is not None and measurement.get("status") != "PASS":
        raise RuntimeError("Full-history WebSocket performance is blocked; refusing calibrated navigation startup")
    timeout = measurement["timeout_s"] if measurement else 60
    if not 5 <= timeout <= 60:
        raise RuntimeError("Measured model latency exceeds 60-second cap")
    args = [str(demo_root / ".venv-cpu/bin/vln-mujoco"), "--host", "127.0.0.1", "--port", "8088",
            "--robot", robot, "--policy-device", "cpu", "--vln-server", "ws://127.0.0.1:8050",
            "--vln-timeout", str(timeout)]
    if robot == "microduck":
        assets = demo_root / ".jetson/assets"
        args += ["--robot-model", str(assets / "src/mjlab_microduck/robot/microduck/robot_allcollisions.xml"),
                 "--walking-policy", str(assets / "alpha_walking.onnx")]
    if record_dir:
        args += ["--record-dir", str(record_dir.resolve())]
    path = DATA / "logs" / f"demo-{robot}-{time.strftime('%Y%m%d-%H%M%S')}.log"
    with path.open("w") as log:
        proc = subprocess.Popen(args, cwd=demo_root, stdout=log, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, start_new_session=True,
                                env=dict(os.environ, MUJOCO_GL="egl", PYTHONUNBUFFERED="1",
                                         LIGHTNAV_CAPTURE_DIR=str(DATA / "captures" / f"{robot}-{time.time_ns()}")))
    fields = Path(f"/proc/{proc.pid}/stat").read_text().rsplit(")", 1)[1].split()
    (DATA / "demo.identity.json").write_text(json.dumps({"pid": proc.pid, "starttime": fields[19],
                                                       "argv": args, "log": str(path)}))
    print(json.dumps({"pid": proc.pid, "robot": robot, "log": str(path)}))


def gate_file():
    return DATA / ("py310/evidence/gate1/result.json" if PROFILE == "seeed310" else "evidence/gate1/result.json")


def main():
    global ENV, PROFILE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["setup", "check", "download", "start", "stop", "status",
                                          "start-demo", "stop-demo", "status-demo", "validate"])
    parser.add_argument("robot", nargs="?", choices=["turtlebot", "microduck"], default="microduck")
    parser.add_argument("--record-dir", type=Path)
    parser.add_argument("--profile", choices=["seeed310", "official311"], default="seeed310",
                        help="Isolated runtime; seeed310 supports JetPack 6.2 Orin")
    parser.add_argument("--validation", choices=["capture", "model", "performance", "performance_ws", "smoke", "trials", "video"], default="model")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--capture", type=Path)
    parser.add_argument("--frames", type=int, default=1200)
    args = parser.parse_args()
    PROFILE = args.profile
    if PROFILE == "seeed310":
        import jetson_lightnav_py310 as profile
        ENV = profile.ENV
        os.environ.update(profile.runtime_environment())
        if args.action in {"setup", "check"}:
            profile.deployment.DATA = profile.DATA
            profile.deployment.ENV = profile.ENV
            for directory in [profile.DATA / "wheels", profile.DATA / "logs", profile.DATA / "evidence"]:
                directory.mkdir(parents=True, exist_ok=True)
            getattr(profile, args.action)()
            return
    for directory in [DATA / "logs", DATA / "evidence"]:
        directory.mkdir(parents=True, exist_ok=True)
    if args.action == "validate":
        if args.output is None:
            parser.error("validate requires --output")
        command = [ENV / "bin/python", ROOT / "scripts/jetson_lightnav_validate.py", args.validation,
                   "--output", args.output, "--frames", str(args.frames)]
        if args.capture:
            command += ["--capture", args.capture]
        if args.validation in {"video", "smoke"}:
            command += ["--robot", args.robot]
        guarded(command, "validation-" + args.validation + ".log")
    elif args.action in {"status", "status-demo"}:
        active = identity("server" if args.action == "status" else "demo")
        ready = bool(active) and (DATA / "server.ready").exists()
        if args.action == "status-demo":
            ready = False
            if active:
                import urllib.request
                try:
                    health = json.loads(urllib.request.urlopen("http://127.0.0.1:8088/api/health", timeout=2).read())
                    ready = health.get("ok", False) and health["simulation"]["thread"]["alive"]
                except (OSError, ValueError, KeyError):
                    pass
        print(json.dumps({"identity": active, "ready": ready,
                          "free_GiB": free_bytes() / 2**30}))
    elif args.action == "start-demo":
        start_demo(args.robot, args.record_dir)
    elif args.action == "stop-demo":
        stop("demo")
    else:
        globals()[args.action]()


if __name__ == "__main__":
    main()
