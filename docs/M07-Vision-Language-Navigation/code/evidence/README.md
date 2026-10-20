# Archived Real Evidence: DM-H65 Navigation

[English](README.md) | [中文](README_zh_CN.md)

These are copies of existing records, not new runs performed while rewriting the course.

## Main Sample

The original run is `sim-demo-20261013/evidence/remote/runtime-4/navigation-0001`, dated 2026-10-13. It contains five predictions and four installed paths. The fifth unsafe path was rejected; `strict_success=false`. Runtime-4 predates local path repair.

- [registration.json](registration.json): instruction, start, limits, and evaluation context.
- [outcome.json](outcome.json): complete original outcome and statistics.
- [historical-summary.json](historical-summary.json): archived summary.
- [installed-paths.json](installed-paths.json): original events for the four installed references.
- [actual-state.jsonl](actual-state.jsonl): all state rows extracted from the parent log for `task_id=navigation-0001`, without interpolation or rewriting. Analysis further limits them to the active window, excluding idle summaries that retain the ID after termination.
- [prediction-01/raw.json](prediction-01/raw.json) and [input.jpg](prediction-01/input.jpg): first response, timing, extrinsics, and hash.
- Predictions 02–05 likewise retain original images, responses, and path checks. Later repair values were not inserted into absent historical fields.

Recompute from the course repository root:

```bash
python3 docs/M07-Vision-Language-Navigation/code/tools/analyze_run.py docs/M07-Vision-Language-Navigation/code/evidence \
  --state docs/M07-Vision-Language-Navigation/code/evidence/actual-state.jsonl --task-id navigation-0001
```

Success remains the recorded outcome value; the tool does not independently revalidate two-second scoring conditions. An absence of contact rows in sampled logs does not establish comprehensive physical safety certification.

## GPU and Model Records

[gpu-gate1.json](gpu-gate1.json), [gpu-operations.json](gpu-operations.json), and [model-manifest.json](model-manifest.json) are archived deployment records with historical device paths. Large traces and weights are separate. These copies document past checks and cannot grant a new device a PASS.

## Later Implementation

Limited local repair was added on 2026-10-14. Short-path tracking, telemetry, and scoring display were corrected on 2026-10-15. Later trials still have not established stable strict success. Discuss versions separately rather than attributing later functionality to this earlier sample.
