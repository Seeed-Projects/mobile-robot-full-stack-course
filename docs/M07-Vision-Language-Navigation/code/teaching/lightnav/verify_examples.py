#!/usr/bin/env python3
"""Independent offline checks for the LightNav-0 teaching assets; never loads a VLM.

Every number the animations draw is re-derived here from the frozen inputs, not read back
from the renderer: the pointing round trip, the SE(2) composition, the RVQ codeword sum,
the GRPO advantages, and the exported files themselves (dimensions, duration, loop, and
the fact that the ten waypoints in ``teaching-numbers.json`` match ``compose_to_abs``).

    python3 verify_examples.py --assets /path/to/assets/lightnav
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import unittest
from pathlib import Path

import numpy as np
from PIL import Image, ImageSequence

HERE = Path(__file__).resolve().parent
ASSETS: Path | None = None

EXPECTED_FILES = [
    '01-decision-loop.gif', '01-decision-loop.png',
    '02-pointing-grid.gif', '02-pointing-grid.png',
    '03-rvq-residual.gif', '03-rvq-residual.png',
    '04-se2-compose.gif', '04-se2-compose.png',
    '05-training-stages.gif', '05-training-stages.png',
    'teaching-numbers.json',
]


def load_reference():
    spec = importlib.util.spec_from_file_location('m07_lightnav_reference', HERE / 'lightnav_reference.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def compose_reference(deltas):
    """Plain-Python SE(2) composition, written out independently of the copied function."""
    x = y = th = 0.0
    out = []
    for dx, dy, dth in deltas:
        x, y = x + math.cos(th) * dx - math.sin(th) * dy, y + math.sin(th) * dx + math.cos(th) * dy
        th = (th + dth + math.pi) % (2 * math.pi) - math.pi
        out.append((x, y, th))
    return out


class Examples(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = json.loads((HERE / 'example-config.json').read_text())
        cls.rvq = json.loads((HERE / 'rvq-teaching-example.json').read_text())
        cls.ref = load_reference()
        cls.numbers = json.loads((ASSETS / 'teaching-numbers.json').read_text())

    # ── pointing grid ────────────────────────────────────────────────────────
    def test_grid_constants_match_config(self):
        self.assertEqual(self.ref.POINT_GRID_W, self.cfg['point_grid_w'])
        self.assertEqual(self.ref.POINT_GRID_H, self.cfg['point_grid_h'])
        self.assertEqual(self.ref.POINT_CELLS, 1296)
        self.assertEqual(self.ref.APOS_VOCAB_SIZE, self.cfg['apos_vocab_size'])
        self.assertEqual(self.ref.OPOS_VOCAB_SIZE, self.cfg['opos_vocab_size'])
        self.assertEqual(self.ref.APOS_ROTL_ID, 1297)
        self.assertEqual(self.ref.APOS_ROTR_ID, 1298)
        self.assertEqual(self.ref.APOS_STOP_ID, 1299)

    def test_opos_505_round_trip(self):
        fw, fh = self.cfg['demo_frame_wh']
        u, v = self.ref.decode_point_pixel_center(505, fw, fh)
        self.assertAlmostEqual(u, 245.0, places=6)
        self.assertAlmostEqual(v, 105.0, places=6)
        self.assertEqual(self.ref.encode_point_pixel(245, 105, fw, fh), 505)
        k = 505 - 1
        self.assertEqual((k % self.ref.POINT_GRID_W, k // self.ref.POINT_GRID_W), (24, 10))

    def test_every_cell_round_trips_to_its_own_centre(self):
        fw, fh = self.cfg['demo_frame_wh']
        for tid in range(1, self.ref.POINT_CELLS + 1):
            u, v = self.ref.decode_point_pixel_center(tid, fw, fh)
            self.assertGreater(u, 0)
            self.assertLess(u, fw)
            self.assertGreater(v, 0)
            self.assertLess(v, fh)
            self.assertEqual(self.ref.encode_point_pixel(u, v, fw, fh), tid)

    def test_apos_point_differs_from_opos_point(self):
        fw, fh = self.cfg['demo_frame_wh']
        self.assertEqual(self.ref.encode_point_pixel(*self.cfg['demo_apos_pixel'], fw, fh),
                         self.cfg['demo_apos_id'])
        self.assertNotEqual(self.cfg['demo_apos_id'], self.cfg['demo_opos_id'])
        # APOS sits below the OPOS point: floor in front of the fridge.
        self.assertGreater(self.cfg['demo_apos_pixel'][1], self.cfg['demo_pixel'][1])

    def test_sentinels_are_not_grid_cells(self):
        for tid in (0, 1297, 1298, 1299):
            self.assertIsNone(self.ref.decode_point_pixel_center(tid, 480, 270))
        # Clamping: a pixel outside the frame lands on an edge cell, never out of range.
        self.assertEqual(self.ref.encode_point_pixel(-999, -999, 480, 270), 1)
        self.assertEqual(self.ref.encode_point_pixel(9999, 9999, 480, 270), 1296)

    def test_teaching_numbers_pointing_block(self):
        point = self.numbers['point_grid']
        self.assertEqual(point['cells'], 1296)
        self.assertEqual(point['cell_px_on_demo_frame'], [10.0, 10.0])
        block = self.numbers['pointing_example']
        self.assertEqual((block['row'], block['col']), (10, 24))
        self.assertEqual(block['opos_id'], 505)
        self.assertEqual(self.numbers['special_ids']['apos_vocab'], 1300)
        self.assertEqual(self.numbers['special_ids']['opos_vocab'], 1297)

    # ── SE(2) composition ────────────────────────────────────────────────────
    def test_turn_in_place_then_forward(self):
        deltas = np.array([[[0.5, 0.0, 0.0], [0.0, 0.0, math.pi / 2], [0.5, 0.0, 0.0]]])
        out = self.ref.compose_to_abs(deltas)[0]
        self.assertAlmostEqual(out[0][0], 0.5, places=5)
        self.assertAlmostEqual(out[0][1], 0.0, places=5)
        self.assertAlmostEqual(out[1][0], 0.5, places=5)   # in-place turn: x unchanged
        self.assertAlmostEqual(out[1][1], 0.0, places=5)   # ... and y unchanged
        self.assertAlmostEqual(out[1][2], math.pi / 2, places=5)
        self.assertAlmostEqual(out[2][0], 0.5, places=5)   # forward now advances +y
        self.assertAlmostEqual(out[2][1], 0.5, places=5)
        self.assertAlmostEqual(out[2][2], math.pi / 2, places=5)

    def test_straight_ten_steps_accumulate(self):
        deltas = np.zeros((1, 10, 3))
        deltas[0, :, 0] = 0.15
        out = self.ref.compose_to_abs(deltas)[0]
        self.assertAlmostEqual(out[-1][0], 1.5, places=5)
        self.assertAlmostEqual(out[-1][1], 0.0, places=6)
        self.assertEqual([round(float(p[0]), 3) for p in out],
                         [round(0.15 * (j + 1), 3) for j in range(10)])

    def test_copy_matches_independent_composition(self):
        rng = np.random.default_rng(7)
        for _ in range(25):
            deltas = rng.normal(scale=0.25, size=(1, 10, 3))
            got = self.ref.compose_to_abs(deltas)[0]
            want = compose_reference(deltas[0])
            for g, w in zip(got, want):
                for a, b in zip(g, w):
                    self.assertAlmostEqual(float(a), b, places=5)

    def test_wrap_keeps_heading_bounded(self):
        deltas = np.zeros((1, 10, 3))
        deltas[0, :, 2] = math.pi * 0.75
        out = self.ref.compose_to_abs(deltas)[0]
        # float32 rounding of pi puts the wrapped value a few ulp outside [+-pi].
        self.assertTrue(all(-math.pi - 1e-6 <= float(p[2]) <= math.pi + 1e-6 for p in out))

    # ── RVQ ──────────────────────────────────────────────────────────────────
    def test_bundle_manifest_matches_config(self):
        bundle = self.cfg['rvq_bundle']
        self.assertEqual(bundle['levels'], [256, 256, 256])
        self.assertEqual(bundle['horizon'], 10)
        self.assertEqual(bundle['feature_dim'], 30)
        self.assertEqual(bundle['representation'], 'se2_diff')
        self.assertEqual(bundle['heading_weight'], 0.3)
        self.assertEqual(bundle['stop_codes'], [6, 122, 174])
        self.assertEqual(self.rvq['levels'], bundle['levels'])
        self.assertEqual(self.rvq['representation'], bundle['representation'])

    def test_codes_sum_to_the_frozen_deltas(self):
        sums = [row for row in self.rvq['summed_motion_deltas']]
        parts = [np.array(row) for row in self.rvq['codeword_motion_deltas']]
        total = np.sum(parts, axis=0)
        self.assertEqual(len(sums), 10)
        for got, want in zip(sums, total):
            for a, b in zip(got, want):
                self.assertAlmostEqual(a, float(b), places=5)

    def test_decoded_waypoints_match_composition(self):
        deltas = np.array(self.rvq['summed_motion_deltas'])
        got = self.ref.compose_to_abs(deltas[None])[0]
        for row, expect in zip(self.rvq['waypoints_full'], got):
            for a, b in zip(row, expect):
                self.assertAlmostEqual(a, float(b), places=5)

    def test_each_level_reduces_the_gap(self):
        gaps = [row['mean_position_gap_m'] for row in self.rvq['refinement']]
        self.assertEqual(len(gaps), 3)
        self.assertGreater(gaps[0], 0)
        self.assertLess(gaps[1], gaps[0])
        self.assertEqual(gaps[2], 0.0)  # the third level IS the full decode
        heads = [row['mean_heading_gap_rad'] for row in self.rvq['refinement']]
        self.assertLess(heads[1], heads[0])

    def test_partial_paths_share_a_start_and_a_horizon(self):
        # Every prefix starts from the same pose, but the FIRST delta already differs
        # between levels, so only the origin (0, 0) is common -- not the first waypoint.
        for name, codes_used in (('L0 only', 1), ('L0 + L1', 2), ('L0 + L1 + L2', 3)):
            path = self.rvq['waypoints_partial'][codes_used - 1]
            self.assertEqual(len(path), 10, name)
            deltas = np.sum([np.array(row) for row in self.rvq['codeword_motion_deltas'][:codes_used]],
                            axis=0)
            got = self.ref.compose_to_abs(deltas[None])[0]
            for row, expect in zip(path, got):
                for a, b in zip(row, expect):
                    self.assertAlmostEqual(a, float(b), places=5)
            self.assertLess(path[0][0], 0.4, f'{name} first waypoint is a single step away')

    def test_stop_code_is_near_zero_but_identified_by_code(self):
        stop = self.rvq['stop']
        self.assertEqual(stop['l0'], 6)
        self.assertEqual(stop['codes'], [6, 122, 174])
        self.assertLess(stop['codeword_net_m'], 0.01)          # near zero ...
        self.assertGreater(stop['codeword_abs_max'], 0.0)      # ... but not exactly zero
        self.assertEqual(self.cfg['rvq_bundle']['stop_l0'], stop['l0'])

    def test_level_statistics_ordering(self):
        stats = self.rvq['level_stats']
        self.assertEqual([s['level'] for s in stats], [0, 1, 2])
        self.assertGreater(stats[0]['chunk_net_mean_m'], stats[1]['chunk_net_mean_m'])
        self.assertGreater(stats[1]['chunk_net_mean_m'], stats[2]['chunk_net_mean_m'])
        for s in stats:
            self.assertEqual(s['entries'], 256)
            self.assertLessEqual(s['per_step_xy_mean_m'], s['per_step_xy_p90_m'])

    def test_combination_count(self):
        self.assertEqual(self.rvq['combo_count'], 256 ** 3)
        self.assertEqual(self.numbers['rvq']['combinations'], 16777216)

    def test_token_literals(self):
        self.assertEqual(self.rvq['token_literals'],
                         [f'<act_l{i}_{c}>' for i, c in enumerate(self.rvq['codes'])])
        self.assertEqual(self.numbers['rvq']['tokens'], self.rvq['token_literals'])
        self.assertEqual(len(self.cfg['demo_tokens']), 5)
        self.assertEqual(self.cfg['demo_tokens'][2:], self.rvq['token_literals'])

    # ── training ─────────────────────────────────────────────────────────────
    def test_grpo_advantages(self):
        rewards = self.cfg['grpo_rewards']
        mu = sum(rewards) / len(rewards)
        sd = (sum((r - mu) ** 2 for r in rewards) / len(rewards)) ** 0.5
        adv = [(r - mu) / sd for r in rewards]
        for got, want in zip(self.cfg['grpo_advantages'], adv):
            self.assertAlmostEqual(got, want, places=2)
        self.assertAlmostEqual(sum(adv), 0.0, places=6)
        self.assertEqual(self.numbers['training']['grpo_advantages'], self.cfg['grpo_advantages'])

    def test_ce_example(self):
        ce = self.cfg['ce_example']
        self.assertAlmostEqual(ce['loss_before'], -math.log(ce['p_before']), places=3)
        self.assertAlmostEqual(ce['loss_after'], -math.log(ce['p_after']), places=3)
        self.assertGreater(ce['loss_before'], ce['loss_after'])

    # ── exported files ───────────────────────────────────────────────────────
    def test_expected_files_exist(self):
        for name in EXPECTED_FILES:
            self.assertTrue((ASSETS / name).is_file(), f'missing {name}')

    def test_gifs_are_sixteen_second_loops(self):
        manifest_path = ASSETS / 'animation-manifest.json'
        rows = {}
        if manifest_path.is_file():
            manifest = json.loads(manifest_path.read_text())
            self.assertTrue(manifest['not_runtime_evidence'])
            self.assertTrue(manifest['english_only_labels'])
            self.assertEqual(len(manifest['outputs']), 5)
            rows = {row['file']: row for row in manifest['outputs']}
        # Read the actual GIFs even when an optional export manifest is absent.
        for name in (n for n in EXPECTED_FILES if n.endswith('.gif')):
            with Image.open(ASSETS / name) as gif:
                size = gif.size
                loop = gif.info.get('loop')
                duration = sum(f.info.get('duration', 0) for f in ImageSequence.Iterator(gif))
                frames = getattr(gif, 'n_frames', 0)
                animated = sum(1 for f in ImageSequence.Iterator(gif))
            self.assertEqual(list(size), [1200, 675], name)
            self.assertEqual(duration, 16000, name)
            self.assertEqual(loop, 0, f"{name} must loop forever")
            self.assertGreater(animated, 1, f"{name} has no animation")
            if rows:
                row = rows[name]
                self.assertEqual(row['duration_ms'], duration)
                self.assertEqual(row['dimensions'], [1200, 675])
                self.assertEqual(row['gif_frames'], frames)
                self.assertEqual(row['frames_drawn'] - row['hold_frames'], frames)
                self.assertEqual(row['sampling_fps'], 10)

    def test_posters_match_gif_dimensions(self):
        for name in EXPECTED_FILES:
            if not name.endswith('.png'):
                continue
            with Image.open(ASSETS / name) as png:
                self.assertEqual(png.size, (1200, 675), name)
                self.assertEqual(png.mode, 'RGB', name)

    def test_gif_frames_actually_differ(self):
        for name in (n for n in EXPECTED_FILES if n.endswith('.gif')):
            with Image.open(ASSETS / name) as gif:
                frames = [np.asarray(f.convert('RGB'), dtype=int)
                          for f in ImageSequence.Iterator(gif)]
            diffs = [int((np.abs(frames[i] - frames[i - 1]).sum(2) > 12).sum())
                     for i in range(1, len(frames))]
            self.assertGreater(max(diffs), 2000, f"{name} barely changes")

    def test_manifest_hashes_if_present(self):
        path = ASSETS / 'asset-manifest.json'
        if not path.is_file():
            self.skipTest('asset-manifest.json lives with the chapter assets')
        manifest = json.loads(path.read_text())
        for row in manifest['files']:
            target = ASSETS / row['path']
            self.assertTrue(target.is_file(), row['path'])
            self.assertEqual(hashlib.sha256(target.read_bytes()).hexdigest(), row['sha256'], row['path'])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--assets', type=Path, required=True)
    args, rest = ap.parse_known_args()
    global ASSETS
    ASSETS = args.assets.resolve()
    if not ASSETS.is_dir():
        raise SystemExit(f'assets directory not found: {ASSETS}')
    unittest.main(argv=['verify_examples.py'] + rest, exit=True)


if __name__ == '__main__':
    main()
