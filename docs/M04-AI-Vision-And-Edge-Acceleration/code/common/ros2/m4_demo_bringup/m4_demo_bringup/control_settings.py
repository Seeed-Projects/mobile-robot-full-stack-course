"""Shared schema and durable storage for the M4 live-control surface.

The web server is the only writer.  Launch files and the camera publisher read
the same file so a restart begins with the last confirmed settings instead of
briefly processing frames with stale defaults.
"""
from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any


SETTINGS_PATH_ENV = "M4_RUNTIME_SETTINGS_PATH"
DEFAULT_SETTINGS_PATH = Path("/home/seeed/.config/m4-perception/runtime_settings.json")
VERSION = 1

DEFAULTS: dict[str, Any] = {
    "undistort_enabled": False,
    "confidence_threshold": 0.25,
    "nms_threshold": 0.45,
    "track_activation_threshold": 0.25,
    "minimum_matching_threshold": 0.80,
    "lost_track_buffer": 90,
    "minimum_consecutive_frames": 1,
}

LIMITS = {
    "confidence_threshold": (0.05, 0.95, 0.01),
    "nms_threshold": (0.05, 0.95, 0.01),
    "track_activation_threshold": (0.10, 0.90, 0.01),
    "minimum_matching_threshold": (0.10, 0.99, 0.01),
    "lost_track_buffer": (1, 300, 1),
    "minimum_consecutive_frames": (1, 30, 1),
}
INTEGER_KEYS = {"lost_track_buffer", "minimum_consecutive_frames"}


def settings_path() -> Path:
    return Path(os.environ.get(SETTINGS_PATH_ENV, str(DEFAULT_SETTINGS_PATH))).expanduser()


def validate_patch(patch: dict[str, Any]) -> tuple[dict[str, Any] | None, str | None]:
    if not isinstance(patch, dict):
        return None, "settings must be an object"
    result: dict[str, Any] = {}
    for key, value in patch.items():
        if key == "undistort_enabled":
            if not isinstance(value, bool):
                return None, "undistort_enabled must be a boolean"
            result[key] = value
            continue
        if key not in LIMITS:
            return None, f"unknown setting: {key}"
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None, f"{key} must be numeric"
        low, high, _step = LIMITS[key]
        value = int(value) if key in INTEGER_KEYS else float(value)
        if value < low or value > high:
            return None, f"{key} must be between {low} and {high}"
        result[key] = value
    return result, None


def load_settings() -> tuple[dict[str, Any], str | None]:
    """Return validated values and an optional non-fatal load warning."""
    path = settings_path()
    if not path.is_file():
        return dict(DEFAULTS), None
    try:
        with path.open("r", encoding="utf-8") as f:
            raw = json.load(f)
        if not isinstance(raw, dict) or raw.get("version") != VERSION:
            raise ValueError("unsupported settings file version")
        patch, error = validate_patch(raw.get("values", {}))
        if error:
            raise ValueError(error)
        values = dict(DEFAULTS)
        values.update(patch or {})
        return values, None
    except Exception as exc:
        return dict(DEFAULTS), f"runtime settings ignored: {exc}"


def write_settings(values: dict[str, Any]) -> None:
    patch, error = validate_patch(values)
    if error or patch is None or set(patch) != set(DEFAULTS):
        raise ValueError(error or "incomplete settings")
    path = settings_path()
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    payload = {"version": VERSION, "values": patch}
    fd, temporary = tempfile.mkstemp(prefix="runtime_settings.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, sort_keys=True, indent=2)
            f.write("\n")
            f.flush()
            os.fsync(f.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
