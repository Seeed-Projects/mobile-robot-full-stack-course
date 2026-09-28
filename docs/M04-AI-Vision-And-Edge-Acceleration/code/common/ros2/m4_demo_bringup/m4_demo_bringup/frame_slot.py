"""Thread-safe single-slot latest-frame buffer.

The M4 web demo server receives ROS Image messages on a dedicated ROS
executor thread. The aiortc video track runs on the asyncio main loop
and reads frames for encoding. To avoid unbounded queues and to keep
freshness over completeness (the primary robotics-demo requirement),
this module replaces any frame already in the slot with the newer one.

Invariants:
  * At most one Image is stored at any moment.
  * A newer frame ALWAYS replaces an older one (timestamp comparison).
  * `take_for_render()` empties the slot; the next call returns None
    until a new ROS callback refills it.
  * No thread ever blocks waiting for a frame; the caller decides what
    to do when the slot is empty.

This is intentionally simpler than a queue. It is the core of the
"latest-frame-only" semantics required by the M4 web preview.
"""
from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

# Avoid a hard dependency on sensor_msgs in this module's import path.
# The slot stores the raw msg; type checking is intentionally loose.


class LockFreeLatestFrameSlot:
    """Single-slot latest-frame buffer guarded by a Lock.

    Public surface:
        try_replace(msg, stamp_ns) -> bool
            Replace the stored frame iff `stamp_ns` is strictly newer than
            the existing one. Returns True if the slot now holds `msg`.
        take_for_render() -> Optional[Tuple[object, int]]
            Atomically drain and return the current (msg, stamp_ns) or None.

    Notes:
        A short lock is fine: contention is at most one writer (ROS thread)
        and one reader (asyncio track.recv). The Lock is never held across
        any blocking I/O.
    """

    __slots__ = ("_lock", "_msg", "_stamp_ns", "_max_age_ms", "_dropped_total")

    def __init__(self, max_age_ms: int = 1000) -> None:
        self._lock = threading.Lock()
        self._msg: Optional[object] = None
        self._stamp_ns: int = 0
        # Maximum age for a frame before we refuse to replace. 0 = keep all.
        # Default 1000 ms protects against pathological clock jumps
        # (e.g. ROS clock resets) which could otherwise pin a stale frame.
        self._max_age_ms: int = max_age_ms
        self._dropped_total: int = 0

    def try_replace(self, msg: object, stamp_ns: int) -> bool:
        """Replace the stored frame iff `stamp_ns` is strictly newer.

        Monotonic monotonic_ns() time is used; if it is older than the
        current slot's stamp by more than max_age_ms, the slot is reset
        and replaced (so we don't permanently lose a frame to a wall-clock
        hiccup). Returns True if stored.
        """
        with self._lock:
            if self._stamp_ns == 0:
                self._msg = msg
                self._stamp_ns = stamp_ns
                return True
            # Same-age or older — drop (latest-frame-only semantics).
            if stamp_ns <= self._stamp_ns:
                # If it is much older, treat as clock-jump and reset.
                age_ms = (self._stamp_ns - stamp_ns) / 1_000_000.0
                if age_ms > self._max_age_ms:
                    self._msg = msg
                    self._stamp_ns = stamp_ns
                    return True
                self._dropped_total += 1
                return False
            # Strictly newer: replace.
            self._msg = msg
            self._stamp_ns = stamp_ns
            return True

    def take_for_render(self) -> Optional[Tuple[object, int]]:
        """Atomically drain and return the current frame (msg, stamp_ns)."""
        with self._lock:
            if self._msg is None:
                return None
            m = self._msg
            s = self._stamp_ns
            self._msg = None
            self._stamp_ns = 0
            return (m, s)

    def peek_age_ms(self) -> Optional[float]:
        """How stale the current frame is (None if slot is empty)."""
        with self._lock:
            if self._msg is None:
                return None
            return (time.monotonic_ns() - self._stamp_ns) / 1_000_000.0

    @property
    def dropped_total(self) -> int:
        return self._dropped_total


class SwitchableFrameSlot:
    """A slot that proxies to whichever named source is currently active.

    Added for the unified M4 web hub: ONE web server subscribes to all
    three demo overlay topics (`/perception/demo/m4_1|m4_2|m4_3`), each
    into its own LockFreeLatestFrameSlot. The browser switches the
    visible module at runtime; the video backend keeps calling
    `take_for_render()` on a single stable object and therefore never
    needs to be restarted on a switch.

    Invariants:
      * Holds no frame of its own — only a source map plus the active key.
      * `take_for_render()` / `peek_age_ms()` are forwarded verbatim to
        the active source, preserving latest-frame-only semantics.
      * Switching never blocks and never raises for an unknown or
        not-yet-ready key: the forwarded call simply finds an empty slot
        and the backend's normal empty-slot path applies.
    """

    __slots__ = ("_lock", "_sources", "_active")

    def __init__(self, sources: dict, active: str) -> None:
        if not sources:
            raise ValueError("SwitchableFrameSlot requires at least one source")
        if active not in sources:
            # Fall back to the first declared source rather than failing:
            # the mapping order is the module order and is deterministic.
            active = next(iter(sources))
        self._lock = threading.Lock()
        self._sources = dict(sources)
        self._active = active

    @property
    def keys(self):
        return tuple(self._sources.keys())

    @property
    def active(self) -> str:
        with self._lock:
            return self._active

    def set_active(self, key: str) -> bool:
        """Switch the active source. Returns False for an unknown key."""
        with self._lock:
            if key not in self._sources:
                return False
            self._active = key
            return True

    def slot_for(self, key: str):
        """The concrete slot for a module key (None when unknown)."""
        return self._sources.get(key)

    def _active_slot(self) -> LockFreeLatestFrameSlot:
        with self._lock:
            return self._sources[self._active]

    def take_for_render(self):
        return self._active_slot().take_for_render()

    def peek_age_ms(self):
        return self._active_slot().peek_age_ms()
