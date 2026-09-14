#!/usr/bin/env python3
"""Focused regressions for the calibration web UI and state transitions."""
from __future__ import annotations

import re
import json
import os
import subprocess
import tempfile
import threading
import unittest
from unittest import mock
from pathlib import Path
from html.parser import HTMLParser

import cv2
import numpy as np

import calib_web


FIXTURES = Path(__file__).with_name("fixtures")
RIGHT_SAMPLES_FIXTURE = FIXTURES / "right.intr_samples.json"


class UntranslatedTextParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.stack = []
        self.untranslated = []

    def handle_starttag(self, tag, attrs):
        self.stack.append((tag, dict(attrs)))

    def handle_startendtag(self, tag, attrs):
        pass

    def handle_endtag(self, tag):
        for index in range(len(self.stack) - 1, -1, -1):
            if self.stack[index][0] == tag:
                self.stack = self.stack[:index]
                break

    def handle_data(self, data):
        if not re.search(r"[\u3400-\u9fff]", data):
            return
        if any(tag in ("script", "style") for tag, _ in self.stack):
            return
        if any("data-i18n-en" in attrs for _, attrs in self.stack):
            return
        self.untranslated.append(data.strip())


class FakeGrabber:
    device = "/dev/video-test"

    def latest(self):
        return np.zeros((8, 8, 3), np.uint8), 1.0


def bare_state() -> calib_web.CalibState:
    state = calib_web.CalibState.__new__(calib_web.CalibState)
    state._lock = threading.RLock()
    state.intr_task = None
    state.intr_collecting = {d: True for d in calib_web.DIRECTIONS}
    state.mode = "idle"
    state.active_dir = None
    state.intr_task_msg = ""
    state.intr_task_ui = None
    state.intr_rejected = {d: [] for d in calib_web.DIRECTIONS}
    state.intr_samples = {d: [] for d in calib_web.DIRECTIONS}
    state.intr_candidates = {}
    state.intr_preview_maps = {}
    state.intr_preview_revision = {d: 0 for d in calib_web.DIRECTIONS}
    state.cols = 8
    state.rows = 6
    state.width = 1920
    state.height = 1080
    state.grabbers = {d: FakeGrabber() for d in calib_web.DIRECTIONS}
    state.cfg = {
        "capture": {"cameras": {d: {"device": i}
                                  for i, d in enumerate(calib_web.DIRECTIONS)}}
    }
    return state


class WebRegressionTests(unittest.TestCase):
    @staticmethod
    def grid():
        x, y = np.meshgrid(np.arange(8), np.arange(6))
        return np.stack((180 + x * 42 + y * 2,
                         120 + y * 40 + x * 1.5), axis=-1).reshape(-1, 1, 2)

    def test_every_inline_script_parses(self):
        pages = {
            name: calib_web._inject_common(getattr(calib_web, name))
            for name in (
                "DASHBOARD_HTML", "INTRINSICS_HTML", "EXTRINSICS_HTML",
                "MULTI_EXTRINSICS_HTML", "SEAM_HTML", "BEV_HTML",
            )
        }
        with tempfile.TemporaryDirectory() as tmp:
            for page_name, html in pages.items():
                scripts = re.findall(r"<script(?:\s[^>]*)?>(.*?)</script>", html,
                                     flags=re.DOTALL | re.IGNORECASE)
                self.assertTrue(scripts, page_name)
                for index, script in enumerate(scripts):
                    path = Path(tmp) / f"{page_name}-{index}.js"
                    path.write_text(script, encoding="utf-8")
                    result = subprocess.run(
                        ["node", "--check", str(path)], capture_output=True,
                        text=True, check=False)
                    self.assertEqual(result.returncode, 0,
                                     f"{page_name} script {index}: {result.stderr}")

    def test_all_static_chinese_text_has_an_english_translation(self):
        for page_name in (
                "DASHBOARD_HTML", "INTRINSICS_HTML", "EXTRINSICS_HTML",
                "MULTI_EXTRINSICS_HTML", "SEAM_HTML", "BEV_HTML"):
            parser = UntranslatedTextParser()
            parser.feed(getattr(calib_web, page_name))
            self.assertEqual(parser.untranslated, [], page_name)

    def test_every_page_uses_the_device_viewport(self):
        viewport = '<meta name="viewport" content="width=device-width,initial-scale=1">'
        for page_name in (
                "DASHBOARD_HTML", "INTRINSICS_HTML", "EXTRINSICS_HTML",
                "MULTI_EXTRINSICS_HTML", "SEAM_HTML", "BEV_HTML"):
            self.assertIn(viewport, getattr(calib_web, page_name), page_name)

    def test_intrinsics_select_pauses_every_camera(self):
        state = bare_state()
        ok, detail = state.intr_select("left")
        self.assertTrue(ok)
        self.assertEqual(state.active_dir, "left")
        self.assertFalse(any(state.intr_collecting.values()))
        self.assertTrue(detail["available"])
        self.assertEqual(detail["device"], "/dev/video-test")

    def test_intrinsics_collect_is_exclusive(self):
        state = bare_state()
        ok, _ = state.intr_collect("right", True)
        self.assertTrue(ok)
        self.assertEqual(
            state.intr_collecting,
            {"front": False, "back": False, "left": False, "right": True},
        )

    def test_the_intrinsic_grid_validator_rejects_malformed_views(self):
        state = bare_state()
        valid = self.grid()
        self.assertEqual(state.intr_sample_quality(valid)[0:2], (True, "ok"))

        nonfinite = valid.copy()
        nonfinite[0, 0, 0] = np.nan
        self.assertEqual(state.intr_sample_quality(nonfinite)[1], "nonfinite")
        self.assertEqual(state.intr_sample_quality(valid[:-1])[1], "invalid_shape")

        collapsed_edge = valid.copy()
        collapsed_edge[1] = collapsed_edge[0]
        self.assertEqual(state.intr_sample_quality(collapsed_edge)[1],
                         "collapsed_edge")

        collapsed_cell = valid.copy()
        collapsed_cell[9, 0] = (collapsed_cell[0, 0] + collapsed_cell[1, 0]) / 2
        self.assertIn(state.intr_sample_quality(collapsed_cell)[1],
                      {"collapsed_cell", "folded_grid"})

        folded = valid.copy().reshape(6, 8, 1, 2)
        folded[3, [2, 3]] = folded[3, [3, 2]]
        self.assertIn(state.intr_sample_quality(folded)[1],
                      {"collapsed_cell", "folded_grid"})

    def test_calibration_start_pauses_all_collectors(self):
        state = bare_state()
        state.square = 0.03
        grid = self.grid()
        state.intr_samples["right"] = [([i / 30, 0.5, 0.3, 0.1], grid.copy())
                                        for i in range(calib_web.MIN_INTR_SAMPLES)]
        fake_thread = mock.Mock()
        with mock.patch.object(calib_web.threading, "Thread", return_value=fake_thread):
            ok, _ = state.intr_start_calibrate("right")
        self.assertTrue(ok)
        self.assertFalse(any(state.intr_collecting.values()))
        fake_thread.start.assert_called_once()

    def test_quarantine_is_written_without_deleting_audit_data(self):
        state = bare_state()
        state.intr_samples["right"] = [([0.1, 0.2, 0.3, 0.4], self.grid())]
        state.intr_rejected["right"] = [{
            "source_index": 2, "reason": "collapsed_edge", "metrics": {},
            "params": [0.2, 0.3, 0.4, 0.5],
            "corners": self.grid().reshape(-1, 2).tolist(),
            "rejected_at": "2026-09-13T10:00:00",
        }]
        with tempfile.TemporaryDirectory() as tmp, \
                mock.patch.object(calib_web, "results_dir", return_value=Path(tmp)):
            state._save_intr_samples("right")
            payload = json.loads((Path(tmp) / "right.intr_samples.json").read_text())
        self.assertEqual(len(payload["samples"]), 1)
        self.assertEqual(payload["rejected_samples"][0]["reason"], "collapsed_edge")

    def test_intrinsics_markup_keeps_preview_slider_inside_label(self):
        self.assertIn('<input id="previewBalance"', calib_web.INTRINSICS_HTML)
        self.assertNotRegex(
            calib_web.INTRINSICS_HTML,
            r'<label[^>]+data-i18n-(?:zh|en)=[^>]*>[^<]*<input id="previewBalance"')

    def test_right_sample_fixture_quarantines_the_two_collapsed_grids(self):
        path = Path(os.environ.get(
            "CALIB_TEST_RIGHT_SAMPLES",
            str(RIGHT_SAMPLES_FIXTURE)))
        if not path.is_file():
            self.skipTest("right sample fixture is available on the Jetson")
        payload = json.loads(path.read_text(encoding="utf-8"))
        samples = [
            (item["params"], np.asarray(item["corners"], dtype=np.float64))
            for item in payload["samples"]
        ]
        state = bare_state()
        valid, rejected = state._audit_intr_samples(samples)
        stored_rejected = payload.get("rejected_samples", [])
        if stored_rejected:
            self.assertEqual(
                [item["source_index"] for item in stored_rejected], [5, 14])
            self.assertEqual(len(valid), 46)
            self.assertEqual(rejected, [])
        else:
            self.assertEqual([i for i, _, _, _ in rejected], [23, 30])
            self.assertEqual(len(valid), 29)

    def test_right_sample_fixture_calibrates_with_unit_board_coordinates(self):
        path = Path(os.environ.get(
            "CALIB_TEST_RIGHT_SAMPLES",
            str(RIGHT_SAMPLES_FIXTURE)))
        if not path.is_file():
            self.skipTest("right sample fixture is available on the Jetson")
        payload = json.loads(path.read_text(encoding="utf-8"))
        state = bare_state()
        state.height = 1536
        state.intr_rejected["right"] = payload.get("rejected_samples", [])
        state.intr_samples["right"] = [
            (item["params"], np.asarray(item["corners"], dtype=np.float64))
            for item in payload["samples"]
        ]
        state.log = mock.Mock()
        state._intr_calibrate("right")
        candidate = state.intr_candidates["right"]
        self.assertAlmostEqual(candidate["rms"], 0.694176, places=5)
        self.assertEqual(candidate["used_sample_count"], 46)
        self.assertEqual(candidate["quarantined_sample_count"], 2)
        self.assertEqual(len(candidate["rejected_samples"]), 2)
        self.assertTrue(np.isfinite(candidate["K"]).all())
        self.assertTrue(np.isfinite(candidate["D"]).all())

    def test_seam_skip_never_changes_extrinsics_target(self):
        state = bare_state()
        state.target = "right"
        state.seam_mode = True
        state.seam_complete = False
        state.seam_pair_i = 0
        state.seam_streak = 3
        state.seam_last_refine = 10.0
        ok, _ = state.seam_skip()
        self.assertTrue(ok)
        self.assertEqual(state.target, "right")
        self.assertEqual(state.seam_pair_i, 1)

    def test_placeholder_is_a_decodable_jpeg(self):
        data = calib_web._placeholder_jpeg("TEST", "no live frame", (320, 240))
        self.assertIsNotNone(data)
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.shape[:2], (240, 320))

    def test_stale_multi_session_blocks_mutating_workflows(self):
        state = bare_state()
        state.intr_results = {
            d: {"K": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "D": [0, 0, 0, 0], "image_size": [1920, 1080]}
            for d in calib_web.DIRECTIONS
        }
        state.multi_session = {
            "intrinsics_fingerprint": "different-intrinsics",
            "observations": {d: [] for d in calib_web.DIRECTIONS},
            "solutions": {},
        }
        state.multi_task = None
        state.ext_task = None
        state.bursting = False
        for action in (
            lambda: state.multi_capture_start("front", "unused", 0.0, 0.0),
            state.multi_solve,
            state.multi_commit,
        ):
            ok, message = action()
            self.assertFalse(ok)
            self.assertIn("不一致", message)

    def test_bev_page_defers_stream_until_load_succeeds(self):
        self.assertNotIn('src="/bev/stream"', calib_web.BEV_HTML)
        self.assertIn("if(j.ok)refreshStream();else stopStream();",
                      calib_web.BEV_HTML)
        self.assertIn("bevImage.style.display='none'", calib_web.BEV_HTML)

    def test_seam_diagnostic_rejects_different_checkerboards(self):
        # A small projected floor board mirrors the ~27px base-canvas span in
        # the live diagnostic; a 203px displacement is then a different board.
        x, y = np.meshgrid(np.arange(8), np.arange(6))
        corners = np.stack((x * 3.0, y * 3.0), axis=-1).reshape(-1, 2)
        shifted = corners + np.array([203.0, 0.0])
        stats = calib_web.HOM.evaluate_seam_alignment(
            corners, shifted, np.eye(3), np.eye(3), 8, 6)
        self.assertEqual(stats["status"], "invalid")
        self.assertFalse(stats["same_target"])
        self.assertEqual(stats["reason_code"], "different_target")
        self.assertGreater(stats["center_delta_px"], stats["same_target_limit_px"])

    def test_seam_diagnostic_warns_but_allows_current_small_residual(self):
        x, y = np.meshgrid(np.arange(8), np.arange(6))
        corners = np.stack((x * 3.0, y * 3.0), axis=-1).reshape(-1, 2)
        shifted = corners + np.array([3.6, 0.0])
        stats = calib_web.HOM.evaluate_seam_alignment(
            corners, shifted, np.eye(3), np.eye(3), 8, 6)
        self.assertEqual(stats["status"], "warn")
        self.assertTrue(stats["same_target"])
        self.assertEqual(stats["reason_code"], "alignment_warning")
        self.assertLess(stats["rms_px"], 5.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
