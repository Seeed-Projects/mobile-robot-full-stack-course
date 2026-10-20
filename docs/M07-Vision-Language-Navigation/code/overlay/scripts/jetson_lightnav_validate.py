#!/usr/bin/env python3
"""Real camera capture, checkpoint/GPU proof, and full-history HF measurements."""
import argparse
import asyncio
import base64
import hashlib
import io
import json
import math
import os
import shutil
import subprocess
import time
import traceback
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False))


def capture(args):
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / "frames.jsonl").exists():
        raise RuntimeError("Refusing to overwrite existing capture")
    start = time.monotonic()
    stamps = set()
    epoch = None
    with (args.output / "frames.jsonl").open("x", buffering=1) as ledger:
        for index in range(args.frames):
            target = start + index / 4
            time.sleep(max(0, target - time.monotonic()))
            jitter = time.monotonic() - target
            if jitter > 0.075:
                raise RuntimeError(f"Missed 4Hz sampling slot {index}: {jitter:.3f}s")
            with urllib.request.urlopen(args.demo + "/api/capture", timeout=2) as response:
                frame = json.load(response)
            if frame["stamp_ns"] in stamps:
                raise RuntimeError("Repeated camera frame")
            stamps.add(frame["stamp_ns"])
            epoch = frame["scene_epoch"] if epoch is None else epoch
            if frame["scene_epoch"] != epoch:
                raise RuntimeError("Scene reset during capture")
            age = (time.monotonic_ns() - int(frame["monotonic_ns"])) / 1e9
            if not 0 <= age <= 0.15:
                raise RuntimeError(f"Old camera frame: {age}")
            jpeg = base64.b64decode(frame.pop("jpeg_base64"), validate=True)
            file = f"frame-{index:04d}.jpg"
            (args.output / file).write_bytes(jpeg)
            frame.update(index=index, file=file, jpeg_sha256=hashlib.sha256(jpeg).hexdigest(),
                         sample_monotonic_ns=str(time.monotonic_ns()), sampling_jitter_s=jitter)
            ledger.write(json.dumps(frame, allow_nan=False) + "\n")
            if index % 60 == 0:
                remaining = os.statvfs(ROOT)
                if remaining.f_bavail * remaining.f_frsize < 3 * 2**30:
                    raise RuntimeError("Disk reserve below 3 GiB")
                print(json.dumps({"captured": index + 1, "target": args.frames}), flush=True)
    write(args.output / "result.json", {"status": "PASS", "frames": args.frames, "fps": 4,
                                       "elapsed_s": time.monotonic() - start})


def load_frames(args, count):
    import numpy as np
    from PIL import Image

    rows = [json.loads(row) for row in (args.capture / "frames.jsonl").read_text().splitlines()]
    if len(rows) < count:
        raise RuntimeError(f"Need {count} authentic frames, have {len(rows)}")
    images = []
    for row in rows[:count]:
        jpeg = (args.capture / row["file"]).read_bytes()
        assert hashlib.sha256(jpeg).hexdigest() == row["jpeg_sha256"]
        images.append(np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB")))
    return images


def create_agent():
    import torch

    from lightnav.tracking import build_tracking_agent

    os.environ["LIGHTNAV_STRICT_RVQ"] = "1"
    os.environ["HF_HUB_OFFLINE"] = "1"
    torch.set_num_threads(1)
    started = time.monotonic()
    agent = build_tracking_agent(str(ROOT / "checkpoints/LightNav-0"), backend="hf",
                                 device="cuda", task_key="vlnce")
    assert agent.H == 10 and agent.slowfast
    assert tuple(agent.engine.bundle.video_size) == (256, 448)
    assert agent.engine.bundle.pool_stage == "post_vit"
    model = agent.engine.bundle.model
    assert all(parameter.device.type == "cuda" and parameter.dtype == torch.bfloat16
               for parameter in model.parameters())
    return agent, time.monotonic() - started


def predict(agent, instruction):
    import numpy as np
    import torch

    torch.cuda.synchronize()
    started = time.perf_counter()
    path, raw, reported_ms = agent.predict_waypoints(instruction, task_type="vlnce_traj")
    torch.cuda.synchronize()
    elapsed = time.perf_counter() - started
    assert path.shape == (10, 3) and path.dtype == np.float32 and np.isfinite(path).all()
    assert agent.last_decode_info["model_output_valid"] is True
    return {"raw_text": raw, "waypoints": path.tolist(), "decode_info": agent.last_decode_info,
            "elapsed_s": elapsed, "reported_latency_ms": reported_ms}


def model(args):
    import torch

    agent, cold_load = create_agent()
    for image in load_frames(args, 16):
        agent.observe(image)
    scopes = {}
    hooks = []
    network = agent.engine.bundle.model
    for name, module in network.named_modules():
        if name.endswith(".visual") or name.endswith(".language_model"):
            label = "gpu_module:" + name

            def before(_module, _inputs, label=label):
                scopes[label] = torch.profiler.record_function(label)
                scopes[label].__enter__()

            def after(_module, _inputs, _output, label=label):
                torch.cuda.synchronize()
                scopes.pop(label).__exit__(None, None, None)

            hooks.extend([module.register_forward_pre_hook(before), module.register_forward_hook(after)])
    failure = None
    result = None
    with torch.profiler.profile(activities=[torch.profiler.ProfilerActivity.CPU,
                                           torch.profiler.ProfilerActivity.CUDA]) as profile:
        try:
            result = predict(agent, args.instruction)
        except Exception as error:
            failure = error
    for hook in hooks:
        hook.remove()
    trace = args.output / "model-gpu-profile.json"
    profile.export_chrome_trace(str(trace))
    events = json.loads(trace.read_text())["traceEvents"]
    kernels = [event for event in events if event.get("cat") == "kernel"]
    evidence = {}
    for event in events:
        if event.get("name", "").startswith("gpu_module:"):
            within = [kernel for kernel in kernels if event["ts"] <= kernel["ts"] <= event["ts"] + event.get("dur", 0)]
            evidence.setdefault(event["name"], []).append({"kernel_count": len(within),
                "kernel_names": sorted({kernel["name"] for kernel in within})})
    write(args.output / "gpu-evidence.json", {"gpu_module_evidence": evidence,
        "cuda_kernel_count": len(kernels), "prediction_error": repr(failure) if failure else None,
        "raw_text_on_error": getattr(failure, "raw_text", None)})
    if failure is not None:
        raise failure
    assert any("visual" in name and any(row["kernel_count"] for row in rows) for name, rows in evidence.items())
    assert any("language_model" in name and any(row["kernel_count"] for row in rows) for name, rows in evidence.items())
    write(args.output / "result.json", {"status": "PASS", "cold_load_s": cold_load,
        "gpu_module_evidence": evidence, "torch": torch.__version__, "python": __import__("sys").version,
        "profiling_uid": os.geteuid(), "result": result})
    print(json.dumps({"status": "PASS", "cold_load_s": cold_load, "prediction_s": result["elapsed_s"]}), flush=True)


def performance(args):
    import numpy as np
    import torch

    agent, cold_load = create_agent()
    results = []
    with (args.output / "calls.jsonl").open("x", buffering=1) as ledger:
        for count in (16, 64, 256, 1200):
            agent.reset(args.instruction)
            prep_start = time.monotonic()
            for image in load_frames(args, count):
                agent.observe(image)
            preparation = time.monotonic() - prep_start
            assert agent._buffer_len == count and len(agent._history_frame_ids) == count
            times = []
            for index in range(35):
                record = predict(agent, args.instruction)
                record.update(frames=count, iteration=index, warmup=index < 5,
                              memory_allocated=torch.cuda.memory_allocated(), memory_reserved=torch.cuda.memory_reserved())
                ledger.write(json.dumps(record, allow_nan=False) + "\n")
                if index >= 5:
                    times.append(record["elapsed_s"])
                print(json.dumps({"frames": count, "iteration": index, "elapsed_s": record["elapsed_s"]}), flush=True)
            results.append({"frames": count, "samples": len(times), "preparation_s": preparation,
                "mean_s": float(np.mean(times)), **{f"p{p}_s": float(np.percentile(times, p)) for p in (50, 95, 99)}})
            write(args.output / "results.json", {"cold_load_s": cold_load, "history_results": results,
                "timing_scope": "CPU full-history model video through preprocessing, H2D, GPU generate, strict CPU decode and synchronization; JPEG/history ingestion reported separately"})
    timeout = max(5, math.ceil(2 * max(row["p99_s"] for row in results) + 1))
    write(args.output / "backend-timeout.json", {"status": "PASS" if timeout <= 60 else "BLOCKED",
        "required_timeout_s": timeout, "timeout_s": timeout if timeout <= 60 else None})
    if timeout > 60:
        raise RuntimeError(f"Required full-history timeout {timeout}s exceeds 60s cap")


def response_path(reply, count):
    """Read the official actions:{step,actions} payload without changing the protocol."""
    import numpy as np

    wrapper = reply.get("actions")
    if not isinstance(wrapper, dict) or wrapper.get("step") != count:
        raise RuntimeError("Missing or mismatched action history step")
    path = np.asarray(wrapper.get("actions"), dtype=np.float32)
    if path.shape != (10, 3) or not np.isfinite(path).all():
        raise RuntimeError("Expected a finite float32 (10,3) action path")
    return path


async def measure_websocket(args):
    import numpy as np
    import websockets

    rows = [json.loads(row) for row in (args.capture / "frames.jsonl").read_text().splitlines()]
    if len(rows) < 1200:
        raise RuntimeError("Need full 1200 authentic frames")
    images = []
    for row in rows[:1200]:
        jpeg = (args.capture / row["file"]).read_bytes()
        assert hashlib.sha256(jpeg).hexdigest() == row["jpeg_sha256"]
        images.append(base64.b64encode(jpeg).decode("ascii"))
    summaries = []
    with (args.output / "websocket-calls.jsonl").open("x", buffering=1) as ledger:
        async with websockets.connect("ws://127.0.0.1:8050", max_size=4 * 1024 * 1024) as socket:
            async def request(action, data, budget=15):
                started = time.perf_counter()
                await socket.send(json.dumps({"action": action, "data": data}))
                raw = await asyncio.wait_for(socket.recv(), budget)
                reply = json.loads(raw)
                elapsed = time.perf_counter() - started
                if reply.get("action") != action or reply.get("data", {}).get("rc") != 0:
                    raise RuntimeError("Model WebSocket error: " + raw)
                return reply["data"], elapsed

            await request("login", {"clientId": "jetson_performance"})
            for count in (16, 64, 256, 1200):
                durations = []
                for iteration in range(35):
                    await request("reset", {})
                    preload_start = time.monotonic()
                    for frame_index in range(count - 1):
                        reply, _ = await request("next", {"seq": frame_index + 1,
                            "image": images[frame_index], "instruction": ""})
                        if reply.get("seq") != frame_index + 1 or reply.get("msg") != "image received":
                            raise RuntimeError("Bad ordered buffer-only acknowledgement")
                    preload_s = time.monotonic() - preload_start
                    request_id = f"performance:{count}:{iteration}:{rows[count - 1]['stamp_ns']}"
                    reply, elapsed = await request("next", {"seq": count, "image": images[count - 1],
                        "instruction": args.instruction, "request_id": request_id, "diagnostics": True}, 120)
                    assert reply["seq"] == count and reply["request_id"] == request_id
                    response_path(reply, count)
                    assert reply["decode_info"]["model_output_valid"] is True
                    assert reply["runtime_metrics"]["history_frames"] == count
                    ledger.write(json.dumps({"frames": count, "iteration": iteration,
                        "warmup": iteration < 5, "elapsed_s": elapsed, "history_ingestion_s": preload_s,
                        "response": reply}, allow_nan=False) + "\n")
                    if iteration >= 5:
                        durations.append(elapsed)
                    print(json.dumps({"transport": "real_websocket", "frames": count,
                        "iteration": iteration, "elapsed_s": elapsed}), flush=True)
                summaries.append({"frames": count, "samples": len(durations),
                    "mean_s": float(np.mean(durations)),
                    **{f"p{p}_s": float(np.percentile(durations, p)) for p in (50, 95, 99)}})
                write(args.output / "websocket-results.json", {"history_results": summaries,
                    "profiling": False, "video_recording": False,
                    "timing_scope": "Latest JPEG serialization, send, decode/preprocess, CUDA prediction, strict CPU decode, receive/parse; earlier history ingestion reported separately"})
    timeout = max(5, math.ceil(2 * max(row["p99_s"] for row in summaries) + 1))
    write(args.output / "timeout.json", {"status": "PASS" if timeout <= 60 else "BLOCKED",
        "required_timeout_s": timeout, "timeout_s": timeout if timeout <= 60 else None,
        "timing_source": "real WebSocket", "max_history_frames": 1200})
    if timeout > 60:
        raise RuntimeError(f"Real WebSocket requires {timeout}s, above 60s cap")


def performance_ws(args):
    asyncio.run(measure_websocket(args))


TV_TARGETS = [(5.04414, 9.82001), (6.52205, 9.66586)]


def score_state(state):
    simulation = state["simulation"]
    pose = simulation["pose"]
    velocity = simulation["velocity"]
    distance = min(math.hypot(pose["x"] - x, pose["y"] - y) for x, y in TV_TARGETS)
    backend = simulation.get("backend", {})
    valid = (not simulation.get("collision", {}).get("detected")
             and backend.get("finite", True) and backend.get("trunk_height_m", 1) >= 0.065
             and backend.get("tilt_rad", 0) <= 0.9
             and simulation.get("physics_safety", {}).get("valid", True)
             and simulation["thread"]["alive"] and not simulation["thread"]["error"])
    stopped_at_goal = valid and distance <= 0.75 and velocity["planar"] <= 0.05 and abs(velocity["angular"]) <= 0.1
    return {"distance_m": distance, "valid": valid, "stopped_at_goal": stopped_at_goal}


def score_physics_window(window, *, epoch, stop_ns, now_ns):
    """Require every 5ms physics step over 2s after the accepted model stop."""
    samples = window.get("samples", [])
    if window.get("scene_epoch") != epoch or window.get("error") or not samples:
        return {"passed": False, "reason": "Invalid or empty physics window"}
    cutoff = now_ns - 2_000_000_000
    before = [row for row in samples if int(row["monotonic_ns"]) <= cutoff]
    after = [row for row in samples if int(row["monotonic_ns"]) > cutoff]
    if not before or not after:
        return {"passed": False, "reason": "Insufficient physics history"}
    rows = [before[-1], *after]
    stamps = [int(row["monotonic_ns"]) for row in rows]
    steps = [row["step"] for row in rows]
    if (stamps[0] < stop_ns or cutoff - stamps[0] > 100_000_000
            or now_ns - stamps[-1] > 100_000_000 or now_ns < stamps[-1]
            or any(b < a or b - a > 100_000_000 for a, b in zip(stamps, stamps[1:]))
            or any(b != a + 1 for a, b in zip(steps, steps[1:]))):
        return {"passed": False, "reason": "Incomplete, stale, or pre-stop physics history"}
    for row in rows:
        if not row["valid"]:
            return {"passed": False, "reason": "Unsafe physics step", "step": row["step"]}
        x, y, _, _ = row["pose"]
        values = (x, y, row["planar_speed"], row["angular_speed"])
        if not all(math.isfinite(value) for value in values):
            return {"passed": False, "reason": "Non-finite physics step", "step": row["step"]}
        distance = min(math.hypot(x - tx, y - ty) for tx, ty in TV_TARGETS)
        if distance > 0.75 or row["planar_speed"] > 0.05 or abs(row["angular_speed"]) > 0.1:
            return {"passed": False, "reason": "Goal or speed criterion violated", "step": row["step"]}
    return {"passed": True, "samples": len(rows), "span_s": (stamps[-1] - stamps[0]) / 1e9,
            "reason": "Every physics step meets ANY-TV stop criteria"}


def runtime_metrics(rows):
    """Summarize observed wall-time rates; retain all underlying samples."""
    import numpy as np

    if len(rows) < 2:
        return {"samples": len(rows)}
    first, last = rows[0], rows[-1]
    elapsed = last["observed_monotonic"] - first["observed_monotonic"]
    a, b = first["data"]["simulation"], last["data"]["simulation"]
    result = {"samples": len(rows), "elapsed_s": elapsed,
        "real_time_factor": (b["sim_time"] - a["sim_time"]) / elapsed,
        "camera_update_hz": (b["camera"]["frame_count"] - a["camera"]["frame_count"]) / elapsed,
        "physics_hz": (b["timing"]["physics_step_count"] - a["timing"]["physics_step_count"]) / elapsed,
        "physics_max_lag_ms": b["timing"]["physics_max_lag_ms"],
        "watchdog_timeouts": b["timing"]["watchdog_timeout_count"] - a["timing"]["watchdog_timeout_count"]}
    policies = [row["data"]["simulation"].get("backend", {}).get("policy", {}) for row in rows]
    if policies[-1]:
        result["policy_hz"] = (policies[-1]["call_count"] - policies[0]["call_count"]) / elapsed
        result["policy_20ms_overruns"] = policies[-1]["overrun_count"] - policies[0]["overrun_count"]
        distinct = {value["last_start_ns"]: value for value in policies if value.get("last_start_ns")}
        # Runtime messages are slower than gait calls. Exact intervals remain in the policy NPZ archive.
        intervals = [value["last_interval_ms"] for value in distinct.values() if value.get("last_interval_ms") is not None]
        result["runtime_sampled_policy_intervals_ms"] = {f"p{p}": float(np.percentile(intervals, p)) for p in (50, 95, 99)} if intervals else {}
    return result


async def record_video(args, directory, stopped):
    """Capture both cameras; encode afterwards using measured wall-time durations."""
    from PIL import Image

    directory.mkdir()
    rows = []
    deadline = time.monotonic()
    def fetch_pair(index):
        images = []
        source = []
        for name, endpoint in [("first", "/api/camera.jpg"), ("third", "/api/third-person.jpg")]:
            with urllib.request.urlopen(args.demo + endpoint, timeout=2) as response:
                jpeg = response.read()
                source.append({"view": name, "stamp_ns": response.headers.get("X-Frame-Stamp-Ns"),
                    "monotonic_ns": response.headers.get("X-Frame-Monotonic-Ns"), "sha256": hashlib.sha256(jpeg).hexdigest()})
            (directory / f"{name}-{index:06d}.jpg").write_bytes(jpeg)
            images.append(Image.open(io.BytesIO(jpeg)).convert("RGB"))
        combined = Image.new("RGB", (sum(image.width for image in images), max(image.height for image in images)))
        x = 0
        for image in images:
            combined.paste(image, (x, 0))
            x += image.width
        file = f"combined-{index:06d}.jpg"
        combined.save(directory / file, quality=90)
        return {"file": file, "sources": source, "observed_monotonic": time.monotonic()}
    with (directory / "frames.jsonl").open("x", buffering=1) as ledger:
        while not stopped.is_set():
            row = await asyncio.to_thread(fetch_pair, len(rows))
            rows.append(row)
            ledger.write(json.dumps(row) + "\n")
            if os.statvfs(ROOT).f_bavail * os.statvfs(ROOT).f_frsize < 3 * 2**30:
                raise RuntimeError("Video stopped at disk reserve")
            deadline += 0.1
            await asyncio.sleep(max(0, deadline - time.monotonic()))
    ended = time.monotonic()
    if not rows:
        raise RuntimeError("Video has no frames")
    timeline = []
    for index, row in enumerate(rows):
        next_time = rows[index + 1]["observed_monotonic"] if index + 1 < len(rows) else ended
        timeline.extend([f"file '{row['file']}'", f"duration {max(0.001, next_time - row['observed_monotonic']):.9f}"])
    timeline.append(f"file '{rows[-1]['file']}'")
    (directory / "timeline.txt").write_text("\n".join(timeline) + "\n")
    with (directory / "encoding.log").open("w") as log:
        await asyncio.to_thread(subprocess.run, ["ffmpeg", "-nostdin", "-f", "concat", "-safe", "1",
            "-i", "timeline.txt", "-vf", "fps=20", "-c:v", "libx264", "-pix_fmt", "yuv420p",
            "navigation.mp4"], cwd=directory, stdout=log, stderr=log, check=True)
    write(directory / "result.json", {"status": "PASS", "speed": "original wall-time, including inference waits",
        "frames": len(rows), "wall_duration_s": ended - rows[0]["observed_monotonic"],
        "output_fps": 20, "profiling": False, "separate_from_performance": True})


async def run_trial(args, robot, instruction, yaw, index):
    import websockets

    directory = args.output / f"{robot}-yaw-{yaw:+.3f}-trial-{index}"
    directory.mkdir()
    summary = {"status": "FAIL", "robot": robot, "instruction": instruction,
               "initial_yaw": yaw, "attempt": index, "targets_for_scoring_only": TV_TARGETS,
               "start_time_ns": str(time.time_ns()), "start_monotonic_ns": str(time.monotonic_ns()),
               "manual_intervention": False}
    runtime_rows = []
    video_stopped = asyncio.Event()
    video_task = None
    async with websockets.connect(args.demo.replace("http", "ws", 1) + "/ws", max_size=4 * 1024 * 1024) as socket:
        resource_log = (directory / "resources.log").open("x")
        resource_process = subprocess.Popen(["tegrastats", "--interval", "1000"], stdout=resource_log,
            stderr=subprocess.STDOUT) if shutil.which("tegrastats") else None
        async def command(value):
            await socket.send(json.dumps(value))
            while True:
                message = json.loads(await asyncio.wait_for(socket.recv(), 5))
                if message.get("type") == "command_result":
                    if not message.get("ok"):
                        raise RuntimeError(str(message))
                    return

        try:
            await command({"type": "reset", "initial_yaw": yaw})
            # Initialization only; all subsequent robot movement is model/MPC controlled.
            await asyncio.sleep(1)
            if getattr(args, "record_video", False):
                video_task = asyncio.create_task(record_video(args, directory / "video", video_stopped))
            await command({"type": "set_vln", "enabled": True, "instruction": instruction,
                           "server_url": "ws://127.0.0.1:8050"})
            started = time.monotonic()
            summary["start_monotonic_ns"] = str(int(started * 1e9))
            stable_since = None
            failed = False
            phases = []
            last_phase = None
            first_pose = None
            moved = False
            with (directory / "runtime.jsonl").open("x", buffering=1) as ledger:
                while time.monotonic() - started <= 302:
                    message = json.loads(await asyncio.wait_for(socket.recv(), 5))
                    if message.get("type") not in {"runtime", "snapshot"}:
                        continue
                    state = message["data"]
                    now = time.monotonic()
                    row = {"observed_monotonic": now, "data": state}
                    runtime_rows.append(row)
                    ledger.write(json.dumps(row, allow_nan=False) + "\n")
                    score = score_state(state)
                    phase = state["navigation"]["phase"]
                    if phase != last_phase:
                        phases.append({"phase": phase, "elapsed_s": now - started})
                        print(json.dumps({"robot": robot, "trial": index, "yaw": yaw, **phases[-1]}), flush=True)
                        last_phase = phase
                    pose = state["simulation"]["pose"]
                    if first_pose is None:
                        first_pose = pose
                        summary["actual_start_pose"] = pose
                    moved = moved or math.hypot(pose["x"] - first_pose["x"], pose["y"] - first_pose["y"]) >= 0.005
                    failed = failed or not score["valid"]
                    summary.update(final_score=score, final_phase=phase, final_pose=pose,
                                   predictions=state["navigation"]["predictions"], elapsed_s=now - started,
                                   navigation_error=state["navigation"]["error"])
                    if (getattr(args, "smoke", False) and not failed and moved
                            and summary["predictions"] >= 2 and phase == "EXECUTING"):
                        summary["reason"] = "Separate chain check completed; stopped before task scoring"
                        summary["chain_check_passed"] = True
                        break
                    if phase == "MODEL_STOP":
                        if score["stopped_at_goal"] and not failed:
                            stable_since = now if stable_since is None else stable_since
                            if now - stable_since >= 2.1:
                                with urllib.request.urlopen(args.demo + "/api/physics-window", timeout=2) as response:
                                    window = json.load(response)
                                proof = score_physics_window(window, epoch=state["simulation"]["scene_epoch"],
                                    stop_ns=int(stable_since * 1e9), now_ns=time.monotonic_ns())
                                write(directory / "stop-physics-window.json", window)
                                summary["physics_stop_proof"] = proof
                                if proof["passed"]:
                                    summary["status"] = "PASS"
                                    summary["reason"] = "Valid model stop and 2s continuously stable within ANY-TV target"
                                    break
                                if now - stable_since > 5:
                                    summary["reason"] = proof["reason"]
                                    break
                        else:
                            summary["reason"] = "Model stopped outside goal or speed/collision/fall criteria"
                            break
                    elif phase in {"FAILED", "STOPPED", "IDLE"}:
                        summary["reason"] = state["navigation"]["error"] or "Navigation ended without valid model stop"
                        break
                else:
                    summary["reason"] = "300-second trial budget exceeded"
            summary.update(phases=phases, observed_motion=moved,
                           closed_loop_motion_and_new_observation=moved and summary.get("predictions", 0) >= 2)
        except Exception as error:
            summary.update(reason=repr(error), traceback=traceback.format_exc())
        finally:
            try:
                await socket.send(json.dumps({"type": "stop"}))
            except Exception as error:
                summary["cleanup_stop_error"] = repr(error)
            finally:
                video_stopped.set()
                if video_task:
                    try:
                        await video_task
                    except Exception as error:
                        summary["video_error"] = repr(error)
                if resource_process is not None and resource_process.poll() is None:
                    resource_process.terminate()
                    resource_process.wait(timeout=5)
                resource_log.close()
            summary["metrics"] = runtime_metrics(runtime_rows)
            summary["end_monotonic_ns"] = str(int(runtime_rows[-1]["observed_monotonic"] * 1e9)) if runtime_rows else str(time.monotonic_ns())
            write(directory / "score.json", summary)
    return summary


def video(args):
    """A separate recorded trial, never used as a performance measurement."""
    timeout = json.loads((ROOT / ".jetson-lightnav/evidence/performance/timeout.json").read_text())
    if timeout.get("status") != "PASS":
        raise RuntimeError("Full-history performance must pass before recorded navigation")
    subprocess.run(["python3", str(ROOT / "scripts/jetson_lightnav.py"), "stop-demo"], check=True)
    subprocess.run(["python3", str(ROOT / "scripts/jetson_lightnav.py"), "start-demo", args.robot], check=True)
    deadline = time.monotonic() + 30
    while True:
        try:
            with urllib.request.urlopen(args.demo + "/api/health", timeout=2) as response:
                if json.load(response)["ok"]:
                    break
        except OSError:
            pass
        if time.monotonic() >= deadline:
            raise RuntimeError("Demo did not become healthy")
        time.sleep(0.2)
    args.record_video = True
    write(args.output / "score.json", asyncio.run(run_trial(args, args.robot, args.instruction, -math.pi / 2, 0)))


def smoke(args):
    """Verify two real predictions separated by MPC motion before task scoring."""
    timeout = json.loads((ROOT / ".jetson-lightnav/evidence/performance/timeout.json").read_text())
    if timeout.get("status") != "PASS":
        raise RuntimeError("Full-history performance must pass before closed-loop chain checks")
    subprocess.run(["python3", str(ROOT / "scripts/jetson_lightnav.py"), "stop-demo"], check=True)
    subprocess.run(["python3", str(ROOT / "scripts/jetson_lightnav.py"), "start-demo", args.robot,
                    "--record-dir", str(args.output / "simulation")], check=True)
    deadline = time.monotonic() + 30
    while True:
        try:
            with urllib.request.urlopen(args.demo + "/api/health", timeout=2) as response:
                if json.load(response)["ok"]:
                    break
        except OSError:
            pass
        if time.monotonic() >= deadline:
            raise RuntimeError("Demo did not become healthy")
        time.sleep(0.2)
    args.smoke = True
    score = asyncio.run(run_trial(args, args.robot, args.instruction, -math.pi / 2, 0))
    write(args.output / "result.json", {"status": "PASS" if score.get("chain_check_passed") else "FAIL",
        "robot": args.robot, "not_a_scored_task": True, "trial_evidence": score})
    subprocess.run(["python3", str(ROOT / "scripts/jetson_lightnav.py"), "stop-demo"], check=True)
    if not score.get("chain_check_passed"):
        raise RuntimeError("Real closed-loop chain did not pass: " + score.get("reason", ""))


def trials(args):
    timeout = json.loads((ROOT / ".jetson-lightnav/evidence/performance/timeout.json").read_text())
    if timeout.get("status") != "PASS":
        raise RuntimeError("Measured full-history performance must pass before formal trials")
    scores = []
    for robot in ("turtlebot", "microduck"):
        subprocess.run(["python3", str(ROOT / "scripts/jetson_lightnav.py"), "stop-demo"], check=True)
        subprocess.run(["python3", str(ROOT / "scripts/jetson_lightnav.py"), "start-demo", robot,
                        "--record-dir", str(args.output / (robot + "-simulation"))], check=True)
        deadline = time.monotonic() + 30
        while True:
            try:
                with urllib.request.urlopen(args.demo + "/api/health", timeout=2) as response:
                    if json.load(response)["ok"]:
                        break
            except OSError:
                pass
            if time.monotonic() >= deadline:
                raise RuntimeError("Demo did not become healthy")
            time.sleep(0.2)
        for yaw, instruction in [(-math.pi / 2, "Go to the TV and stop."),
                                 (math.pi / 2, "Turn around, go to the TV and stop.")]:
            for index in range(1, 4):
                scores.append(asyncio.run(run_trial(args, robot, instruction, yaw, index)))
                write(args.output / "scores.json", {"status": "PASS" if all(s["status"] == "PASS" for s in scores) else "FAIL",
                    "completed": len(scores), "planned": 12, "successful": sum(s["status"] == "PASS" for s in scores), "trials": scores})
        subprocess.run(["python3", str(ROOT / "scripts/jetson_lightnav.py"), "stop-demo"], check=True)
        if robot == "microduck":
            import numpy as np

            batches = []
            for file in sorted((args.output / (robot + "-simulation")).glob("policy-*.npz")):
                with np.load(file, allow_pickle=False) as batch:
                    batches.append({name: batch[name] for name in batch.files})
            if not batches:
                raise RuntimeError("MicroDuck policy recorder produced no samples")
            all_samples = {name: np.concatenate([batch[name] for batch in batches]) for name in batches[0]}
            for score in (value for value in scores if value["robot"] == robot):
                mask = ((all_samples["monotonic_start_ns"] >= int(score["start_monotonic_ns"]))
                        & (all_samples["monotonic_start_ns"] <= int(score["end_monotonic_ns"])))
                starts = all_samples["monotonic_start_ns"][mask]
                intervals = np.diff(starts) / 1e6
                latency = all_samples["latency_ms"][mask]
                score["exact_policy_metrics"] = {"calls": len(starts),
                    "finite_observations_and_actions": bool(np.isfinite(all_samples["observations"][mask]).all()
                        and np.isfinite(all_samples["actions"][mask]).all()),
                    "over_20ms": int(np.count_nonzero(latency > 20)),
                    "latency_ms": {f"p{p}": float(np.percentile(latency, p)) for p in (50, 95, 99)} if len(latency) else {},
                    "interval_ms": {f"p{p}": float(np.percentile(intervals, p)) for p in (50, 95, 99)} if len(intervals) else {},
                    "interval_jitter_std_ms": float(np.std(intervals)) if len(intervals) else None}
                directory = args.output / f"{robot}-yaw-{score['initial_yaw']:+.3f}-trial-{score['attempt']}"
                write(directory / "score.json", score)
    write(args.output / "scores.json", {"status": "PASS" if all(s["status"] == "PASS" for s in scores) else "FAIL",
        "completed": len(scores), "planned": 12, "successful": sum(s["status"] == "PASS" for s in scores), "trials": scores})
    subprocess.run(["python3", str(ROOT / "scripts/jetson_lightnav.py"), "start-demo", "microduck"], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["capture", "model", "performance", "performance_ws", "smoke", "trials", "video"])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--capture", type=Path)
    parser.add_argument("--frames", type=int, default=1200)
    parser.add_argument("--demo", default="http://127.0.0.1:8088")
    parser.add_argument("--instruction", default="Go to the TV and stop.")
    parser.add_argument("--robot", choices=["turtlebot", "microduck"], default="microduck")
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        globals()[args.action](args)
    except BaseException as error:
        write(args.output / "failure.json", {"status": "FAIL", "error": repr(error),
              "raw_text": getattr(error, "raw_text", None),
              "traceback": traceback.format_exc(), "time_ns": str(time.time_ns())})
        raise


if __name__ == "__main__":
    main()
