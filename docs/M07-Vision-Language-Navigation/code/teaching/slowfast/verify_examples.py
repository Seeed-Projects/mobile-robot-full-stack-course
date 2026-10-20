#!/usr/bin/env python3
"""Independent offline checks for the teaching assets; never loads a VLM."""
from __future__ import annotations
import argparse
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from PIL import Image,ImageSequence
import build_visuals as visuals

HERE=Path(__file__).resolve().parent
ASSETS=None

def reference_pool(a,s):
    return np.array([[float(a[r:r+s,c:c+s].mean()) for c in range(0,8,s)] for r in range(0,8,s)])

class Examples(unittest.TestCase):
    def setUp(self):self.cfg,self.selector,self.tiers,self.rows,self.index,self.counts=visuals.example_data()

    def test_exact_240_frame_selector(self):
        expected_mid=[v for start in range(150,205,6) for v in (start,start+1)]
        # Frame indices are chronological; selection is computed by age.
        self.assertEqual(next(r['frame_ids'] for r in self.rows if r['name']=='mid'),expected_mid)
        self.assertEqual(next(r['frame_ids'] for r in self.rows if r['name']=='current'),[238,239])
        self.assertEqual(next(r['frame_ids'] for r in self.rows if r['name']=='fast'),list(range(206,238)))
        self.assertEqual(len(self.index),82)
        self.assertEqual(self.counts,{'anchor':2,'long':26,'mid':20,'fast':32,'current':2})

    def test_anchor_and_pair_removal(self):
        self.assertEqual(next(r['frame_ids'] for r in self.rows if r['name']=='anchor'),[0,1])
        long=next(r['frame_ids'] for r in self.rows if r['name']=='long')
        self.assertNotIn(0,long);self.assertNotIn(1,long);self.assertEqual(len(long)//2,13)
        for r in self.rows:
            if r['name'] in ('mid','long'):
                for a,b in zip(r['frame_ids'][::2],r['frame_ids'][1::2]):self.assertEqual(b,a+1)

    def test_short_history_and_boundaries(self):
        for n in list(range(1,513))+[1200]:
            segs=self.selector.slowfast_segments(n-1,n,self.tiers)
            seen=set()
            for s in segs:
                ids=s['frame_ids'];self.assertEqual(len(ids)%2,0)
                self.assertEqual(ids,sorted(ids));self.assertTrue(all(0<=i<n for i in ids))
                self.assertFalse(seen.intersection(ids));seen.update(ids)
                # An odd dense/anchor segment may repeat its newest frame once.
                dup=len(ids)-len(set(ids));self.assertIn(dup,[0,1])
                if dup:self.assertEqual(ids[-1],ids[-2])
            self.assertIn(n-1,seen)
        first=self.selector.slowfast_segments(0,1,self.tiers)
        self.assertEqual(first[0]['pool_spatial'],1);self.assertEqual(first[0]['frame_ids'],[0,0])
        # At three frames, an incomplete fast segment is structurally padded.
        fast=next(s for s in self.selector.slowfast_segments(2,3,self.tiers) if self.tiers[s['tier']]['name']=='fast')
        self.assertEqual(fast['frame_ids'],[0,0])

    def test_pooling_numerics_and_ratios(self):
        for s,n in [(1,64),(2,16),(4,4)]:
            actual=visuals.avg_pool(visuals.FEATURES,s)
            np.testing.assert_allclose(actual,reference_pool(visuals.FEATURES,s));self.assertEqual(actual.size,n)
        self.assertEqual(float(np.array([[1,3],[5,7]]).mean()),4.)
        self.assertEqual(16/64,.25);self.assertEqual(4/64,.0625)

    def test_relative_timestamps_match_reference(self):
        ns={};exec(compile((HERE/'timestamp_reference.py').read_text(),str(HERE/'timestamp_reference.py'),'exec'),ns)
        ns['os']=SimpleNamespace(environ={'VLN_TIMESTAMP_RELATIVE':'1'})
        f=ns['_calculate_timestamps'];obj=SimpleNamespace(_ts_global_ref=239)
        self.assertTrue(self.cfg['timestamp_relative'])
        self.assertEqual(f(obj,[238,239],4),[.125])
        self.assertEqual(f(obj,[0,1],4),[59.625])
        self.assertEqual((239-80)/4,39.75)

    def test_gif_dimensions_loop_duration_and_changes(self):
        for name in ['time-selection','spatial-pooling']:
            with Image.open(ASSETS/(name+'.gif')) as gif:
                self.assertEqual(gif.size,(1200,675));self.assertEqual(gif.info.get('loop'),0)
                frames=[(f.convert('RGB'),f.info.get('duration',0)) for f in ImageSequence.Iterator(gif)]
                self.assertEqual(sum(d for _,d in frames),16000)
                self.assertGreaterEqual(max(d for _,d in frames),2000)
                self.assertGreater(len({im.tobytes() for im,_ in frames}),10)

    def test_temporal_artifacts_match_exact_selected_ids(self):
        for name in ['time-selection.png','time-selection.gif']:
            with Image.open(ASSETS/name) as image:
                if name.endswith('.gif'):image.seek(image.n_frames-1)
                rgb=image.convert('RGB');selected=[]
                for f in range(240):
                    row,col=divmod(f,60);color=rgb.getpixel((76+col*18+7,341+row*32+11))
                    if min(sum(abs(a-b) for a,b in zip(color,tuple(int(h[i:i+2],16) for i in (1,3,5)))) for h in visuals.COLORS.values())<20:selected.append(f)
                self.assertEqual(selected,sorted(self.index))
        for name in ['time-selection.png','spatial-pooling.png','combined-example.png']:
            with Image.open(ASSETS/name) as im:self.assertEqual(im.size,(1200,675))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--assets',type=Path,required=True);ap.add_argument('--report',type=Path);args=ap.parse_args();ASSETS=args.assets
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(Examples)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    report={'ok':result.wasSuccessful(),'tests_run':result.testsRun,'failures':len(result.failures),'errors':len(result.errors),'selection_counts':visuals.example_data()[-1],'expected_selected':82,'toy_grid_counts':[64,16,4],'relative_timestamp_current_pair_s':.125,'not_tested':['model inference','navigation','Jetson runtime performance']}
    if args.report:args.report.write_text(json.dumps(report,indent=2))
    raise SystemExit(0 if result.wasSuccessful() else 1)
