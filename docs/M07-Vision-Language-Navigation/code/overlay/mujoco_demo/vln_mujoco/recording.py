from __future__ import annotations

import json
import queue
import threading
import time
from pathlib import Path

import numpy as np


class RunRecorder:
    """Write bounded batches off the simulation thread, without silent sample loss."""

    def __init__(self, directory: Path, metadata: dict[str, object] | None = None) -> None:
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        if (
            (self.directory / "manifest.json").exists()
            or (self.directory / "runtime.jsonl").exists()
            or any(self.directory.glob("policy-*.npz"))
        ):
            raise ValueError(f"record directory already contains a run: {self.directory}")
        self._queue: queue.Queue = queue.Queue(maxsize=4096)
        self._lock = threading.Lock()
        self._closed = False
        self._error = ""
        self._policy_written = 0
        self._runtime_written = 0
        self._batch_index = 0
        self._manifest = {
            "format_version": 1,
            "created_time_ns": time.time_ns(),
            "monotonic_origin_ns": time.monotonic_ns(),
            "policy_pattern": "policy-*.npz",
            "runtime_file": "runtime.jsonl",
            "metadata": metadata or {},
        }
        self._write_manifest()
        self._thread = threading.Thread(target=self._write_loop, name="run-recorder", daemon=True)
        self._thread.start()

    def update_metadata(self, metadata: dict[str, object]) -> None:
        # Used during startup, before the simulation thread begins.
        self._manifest["metadata"].update(metadata)
        self._write_manifest()

    def _write_manifest(self) -> None:
        temporary = self.directory / "manifest.json.tmp"
        temporary.write_text(json.dumps(self._manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        temporary.replace(self.directory / "manifest.json")

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            return {
                "directory": str(self.directory),
                "error": self._error,
                "queued": self._queue.qsize(),
                "policy_samples_written": self._policy_written,
                "runtime_records_written": self._runtime_written,
                "closed": self._closed,
            }

    def _submit(self, kind: str, value: dict[str, object]) -> None:
        with self._lock:
            if self._error:
                raise RuntimeError(f"run recording failed: {self._error}")
            if self._closed:
                raise RuntimeError("run recorder is closed")
        try:
            self._queue.put_nowait((kind, value))
        except queue.Full as exc:
            with self._lock:
                self._error = "record queue full; recording cannot keep up"
            raise RuntimeError(self._error) from exc

    def record_policy(
        self,
        *,
        observation: np.ndarray,
        action: np.ndarray,
        monotonic_start_ns: int,
        monotonic_end_ns: int,
        interval_ns: int,
        sim_time: float,
        command: tuple[float, float],
        sample_index: int,
        reset_count: int,
    ) -> None:
        self._submit("policy", {
            "observations": np.asarray(observation, dtype=np.float32).reshape(61).copy(),
            "actions": np.asarray(action, dtype=np.float32).reshape(14).copy(),
            "monotonic_start_ns": monotonic_start_ns,
            "monotonic_end_ns": monotonic_end_ns,
            "interval_ns": interval_ns,
            "latency_ms": (monotonic_end_ns - monotonic_start_ns) / 1e6,
            "sim_time": sim_time,
            "command": np.asarray(command, dtype=np.float32),
            "sample_index": sample_index,
            "reset_count": reset_count,
        })

    def record_runtime(self, snapshot: dict[str, object], *, event: str = "frame") -> None:
        self._submit("runtime", {**snapshot, "event": event, "monotonic_ns": time.monotonic_ns()})

    def _flush_policies(self, records: list[dict[str, object]]) -> None:
        if not records:
            return
        arrays = {key: np.asarray([row[key] for row in records]) for key in records[0]}
        destination = self.directory / f"policy-{self._batch_index:06d}.npz"
        temporary = destination.with_suffix(".npz.tmp")
        with temporary.open("wb") as output:
            np.savez(output, **arrays)
        temporary.replace(destination)
        with self._lock:
            self._policy_written += len(records)
            self._batch_index += 1
        records.clear()

    def _write_loop(self) -> None:
        policies: list[dict[str, object]] = []
        last_flush = time.monotonic()
        try:
            with (self.directory / "runtime.jsonl").open("x", encoding="utf-8") as runtime:
                while True:
                    try:
                        item = self._queue.get(timeout=0.5)
                    except queue.Empty:
                        item = "flush"
                    if item is None:
                        self._flush_policies(policies)
                        runtime.flush()
                        break
                    if item != "flush":
                        kind, value = item
                        if kind == "policy":
                            policies.append(value)
                        else:
                            runtime.write(json.dumps(value, allow_nan=False) + "\n")
                            with self._lock:
                                self._runtime_written += 1
                    now = time.monotonic()
                    if len(policies) >= 256 or now - last_flush >= 2.0:
                        self._flush_policies(policies)
                        runtime.flush()
                        last_flush = now
        except Exception as exc:
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        try:
            if self._thread.is_alive():
                self._queue.put(None, timeout=5.0)
                self._thread.join(timeout=10.0)
            if self._thread.is_alive():
                with self._lock:
                    self._error = "record writer did not stop within 10 seconds"
            self._manifest["finished_time_ns"] = time.time_ns()
            self._manifest["recording"] = self.snapshot()
            self._write_manifest()
        except Exception as exc:
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
        if self._error:
            raise RuntimeError(f"run recording failed: {self._error}")
