#!/usr/bin/env python3
"""Regression tests for uploaded-video input session state."""
from __future__ import annotations

import os
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path

from m4_demo_bringup.web_demo_server import (
    ModuleRuntimeManager,
    RosImageSubscriber,
    VideoInputManager,
)


class FakeRosInput:
    def __init__(self) -> None:
        self.source = "camera"
        self.last_seen = {"video": time.monotonic() - 60.0}
        self.loop_count = 7
        self.session_starts = 0

    def begin_input_session(self, source: str) -> None:
        self.session_starts += 1
        self.last_seen.pop(source, None)
        if source == "video":
            self.loop_count = 0

    def set_input_source(self, source: str) -> None:
        self.source = source
        if source != "video":
            self.loop_count = 0

    def input_snapshot(self) -> dict:
        now = time.monotonic()
        return {
            "source": self.source,
            "last_frame_age_ms": {
                source: (now - seen) * 1000.0
                for source, seen in self.last_seen.items()
            },
            "loop_count": self.loop_count,
        }

    def reset_tracker(self) -> bool:
        return True


class SleepingVideoInputManager(VideoInputManager):
    def _publisher_command(self) -> list:
        return ["/bin/sleep", "60"]


class InputTelemetryTests(unittest.TestCase):
    def test_new_session_clears_stale_frame_and_loop_telemetry(self) -> None:
        subscriber = RosImageSubscriber.__new__(RosImageSubscriber)
        subscriber._input_state_lock = threading.Lock()
        subscriber._input_last_seen = {"video": time.monotonic() - 60.0}
        subscriber._video_loop_count = 9

        subscriber.begin_input_session("video")

        self.assertNotIn("video", subscriber._input_last_seen)
        self.assertEqual(subscriber._video_loop_count, 0)


class RuntimeOwnershipTests(unittest.IsolatedAsyncioTestCase):
    async def test_shutdown_terminates_detached_child_that_ignores_sigint(self) -> None:
        manager = ModuleRuntimeManager(object(), {}, 10.0)
        proc = subprocess.Popen(
            ["/bin/sh", "-c", "trap '' INT; exec sleep 60"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        manager._proc = proc
        manager._active = "m4_1"

        await manager.stop()

        self.assertIsNotNone(proc.poll())
        self.assertIsNone(manager.active)


class VideoInputManagerTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.old_log = os.environ.get("M4_VIDEO_INPUT_LOG")
        os.environ["M4_VIDEO_INPUT_LOG"] = str(
            Path(self.temp_dir.name) / "video-input.log")
        self.ros_input = FakeRosInput()
        self.manager = SleepingVideoInputManager(
            self.ros_input,
            str(Path(self.temp_dir.name) / "input"),
            str(Path(self.temp_dir.name) / "state.json"),
        )
        self.manager._video = {
            "path": str(Path(self.temp_dir.name) / "current.mp4"),
            "filename": "test.mp4",
            "codec": "h264",
            "size_bytes": 1,
            "duration_ms": 1000,
            "width": 1280,
            "height": 720,
            "fps": 25.0,
            "container": "mp4",
        }

    async def asyncTearDown(self) -> None:
        await self.manager.close()
        if self.old_log is None:
            os.environ.pop("M4_VIDEO_INPUT_LOG", None)
        else:
            os.environ["M4_VIDEO_INPUT_LOG"] = self.old_log

    async def _start_video(self) -> dict:
        self.manager._source = "video"
        await self.manager._start_video_locked()
        self.ros_input.set_input_source("video")
        return await self.manager._snapshot_locked()

    async def test_stale_age_does_not_kill_new_publisher(self) -> None:
        state = await self._start_video()

        self.assertEqual(state["source"], "video")
        self.assertEqual(state["video"]["status"], "starting")
        self.assertIsNone(state["error"])
        self.assertEqual(self.ros_input.loop_count, 0)
        self.assertIsNone(self.manager._proc.poll())

        self.ros_input.last_seen["video"] = time.monotonic()
        state = await self.manager._snapshot_locked()
        self.assertEqual(state["video"]["status"], "playing")

    async def test_second_camera_to_video_cycle_starts_cleanly(self) -> None:
        await self._start_video()
        self.ros_input.last_seen["video"] = time.monotonic()
        self.assertEqual(
            (await self.manager._snapshot_locked())["video"]["status"], "playing")

        camera = await self.manager.select("camera")
        self.assertEqual(camera["source"], "camera")
        self.ros_input.last_seen["video"] = time.monotonic() - 60.0
        self.ros_input.loop_count = 12

        second = await self.manager.select("video")
        self.assertEqual(second["source"], "video")
        self.assertEqual(second["video"]["status"], "starting")
        self.assertEqual(second["video"]["loop_count"], 0)
        self.assertEqual(self.ros_input.session_starts, 2)

        self.ros_input.last_seen["video"] = time.monotonic()
        self.assertEqual(
            (await self.manager._snapshot_locked())["video"]["status"], "playing")

    async def test_first_frame_timeout_falls_back_and_persists_error(self) -> None:
        await self._start_video()
        self.manager._started_at = time.monotonic() - 21.0

        failed = await self.manager._snapshot_locked()
        self.assertEqual(failed["source"], "camera")
        self.assertEqual(failed["video"]["status"], "error")
        self.assertIn("produced no frames", failed["error"])

        persisted = await self.manager._snapshot_locked()
        self.assertEqual(persisted["error"], failed["error"])
        self.assertEqual(persisted["video"]["error"], failed["error"])

    async def test_current_session_stall_falls_back_and_persists_error(self) -> None:
        await self._start_video()
        self.ros_input.last_seen["video"] = time.monotonic() - 4.0

        failed = await self.manager._snapshot_locked()
        self.assertEqual(failed["source"], "camera")
        self.assertEqual(failed["video"]["status"], "error")
        self.assertIn("stream stalled", failed["error"])
        self.assertEqual((await self.manager._snapshot_locked())["error"], failed["error"])


if __name__ == "__main__":
    unittest.main()
