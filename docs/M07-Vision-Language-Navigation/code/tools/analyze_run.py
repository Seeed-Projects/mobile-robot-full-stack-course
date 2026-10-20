#!/usr/bin/env python3
"""Summarize real Demo evidence offline. Does not infer or control a robot."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


def quantile(values, q):
    values = sorted(values)
    index = (len(values) - 1) * q
    lo, hi = math.floor(index), math.ceil(index)
    return values[lo] * (hi - index) + values[hi] * (index - lo) if hi != lo else values[lo]


def analyze(run, state=None, task_id=None):
    run = Path(run)
    outcome = json.loads((run / 'outcome.json').read_text())
    predictions = []
    latencies = []
    for directory in sorted(run.glob('prediction-*')):
        if not (directory / 'raw.json').is_file():
            continue
        raw = json.loads((directory / 'raw.json').read_text())
        data = raw['response']['data']
        jpeg = directory / 'input.jpg'
        if not jpeg.is_file() or hashlib.sha256(jpeg.read_bytes()).hexdigest() != raw['jpeg_sha256']:
            raise ValueError('Input image absent or hash mismatch: ' + str(directory))
        points = data.get('actions', {}).get('actions')
        if points is not None and (len(points) != 10 or any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points)):
            raise ValueError('Expected ten finite cumulative SE(2) rows')
        elapsed = raw['elapsed_s']
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('Invalid latency')
        latencies.append(elapsed)
        check = directory / 'full-path-check.json'
        geometry = json.loads(check.read_text()) if check.is_file() else None
        predictions.append({'prediction': directory.name, 'seq': raw['sequence'],
                            'request_id': data.get('request_id'), 'rc': data.get('rc'),
                            'stop': data.get('stop'), 'tokens': data.get('raw_text'),
                            'elapsed_s': elapsed, 'input_sha256_verified': True,
                            'waypoint_count': len(points) if points is not None else None,
                            'path_check': geometry.get('check') if geometry else None})
    result = {'source': str(run), 'reason': outcome['reason'],
              'strict_success_reported': outcome['strict_success'],
              'success_independently_reverified': False,
              'predictions_recorded': len(predictions),
              'initial_distance_m': outcome.get('initial_distance_m'),
              'final_distance_m': outcome.get('final_distance_m'),
              'motion_duty_cycle_reported': outcome.get('motion_duty_cycle'),
              'predictions': predictions}
    if latencies:
        result['request_response_latency_s'] = {
            'n': len(latencies), 'mean': statistics.mean(latencies),
            'p50': quantile(latencies, .5), 'p95': quantile(latencies, .95),
            'p99': quantile(latencies, .99),
            'note': 'Linear interpolation; small samples do not establish tail reliability'}
    if state:
        rows = [json.loads(line) for line in Path(state).read_text().splitlines() if line.strip()]
        # Never join different tasks or time discontinuities into an apparent route.
        selected_task = task_id or run.name
        rows = [r for r in rows if r.get('task_id') == selected_task]
        # A finished task ID remains on later idle summaries. Keep its active
        # execution/stop-confirmation window, not minutes of post-task idling.
        rows = [r for r in rows if r.get('active') is True]
        if not rows:
            raise ValueError('No state rows match the selected task')
        if len({r.get('task_id') for r in rows}) != 1 or any(b['time'] <= a['time'] for a, b in zip(rows, rows[1:])):
            raise ValueError('Mixed tasks or non-monotonic state log')
        result['actual_path_m_from_samples'] = sum(math.dist(a['pose'][:2], b['pose'][:2]) for a, b in zip(rows, rows[1:]))
        result['state_samples'] = len(rows)
        result['nonempty_contact_samples'] = sum(bool(r.get('contact')) for r in rows)
        result['state_recording_window_s'] = rows[-1]['time'] - rows[0]['time']
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--state', type=Path)
    parser.add_argument('--task-id', help='Explicit state-log task ID when evidence was renamed')
    args = parser.parse_args()
    print(json.dumps(analyze(args.run, args.state, args.task_id), indent=2, ensure_ascii=False))
