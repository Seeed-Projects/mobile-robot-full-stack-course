# M7 · Vision-Language Navigation — module notes

This module holds the **frozen course package** for M7: a pinned course overlay on
top of LightNav-0, plus its configuration/lock files, archived evidence, teaching
material and read-only check tools. The English and English and Chinese course chapters live in the course
repository under `docs/M07-Vision-Language-Navigation/`; this directory is the
source tree they are written from.

## Layout

| Path | What it is |
| --- | --- |
| `README.md`, `SHA256SUMS.txt`, `THIRD_PARTY.md`, `overlay-manifest.json` | package index, hash manifest, third-party notes, per-file overlay manifest |
| `overlay/` | the 34-file course overlay applied to a pinned upstream commit |
| `configs/` | original chassis/task registrations, geometry profile, CAD and asset-bundle metadata |
| `locks/` | exact locks for the model environment (py3.10) and the simulation environment (py3.11) |
| `tools/` | read-only checks plus the two explicitly applied preparers |
| `tests/` | course tool/URDF tests (no model import, no network) |
| `evidence/` | archived real responses, images, poses and GPU records |
| `teaching/slowfast/` | offline SlowFast selection/pooling visuals for 7.2 |
| `examples/custom-chassis/` | read-only URDF inventory, part mapping and a standalone MJCF preview |
| `licenses/` | upstream licence and third-party notices |
| `docs/` | these development notes |
| `scripts/m7/` | run/verify entry points |
| `output/` | run evidence, never committed |

## Lesson → material map

| Lesson | Material in this module |
| --- | --- |
| 7.1 VLN fundamentals | package `README.md`; the chapter is conceptual and has no runnable artifact |
| 7.2 Semantic grounding and learned VLN | `teaching/slowfast/`, `overlay/src/lightnav/serving/strict_decode.py`, `tools/analyze_run.py`, `evidence/` |
| 7.3 Run the LightNav navigation demo | `overlay/` (launchers, DM-H65 backend, simulation demo, web UI, tests), `configs/`, `locks/`, `tools/`, `scripts/m7/` |

## Pins

| Item | Pin |
| --- | --- |
| LightNav-0 upstream commit | `3015508b70fb6e30bbd66701e94c9b916dcb8df8` |
| model revision | `826dc5fbfa37afa8293d2e336d329b6ffc0bfb64` |
| course overlay | 34 files, per-file SHA256 in `overlay-manifest.json` |
| package manifest | 106 entries in `SHA256SUMS.txt` |
| DM-H65 course asset bundle | `dm-h65-course-assets-20261015.tar.gz`, 4,104,931 B, sha256 `23f038fddb7e2f606414f37cbe878aaff79f9cb00e51bee53a3d644fa1cd3a35` |

The overlay is **not** a complete LightNav repository and **not** a second VLM.
Model weights, wheel files and the CAD bundle are external: they are never
committed and this package does not download them for you.

## Entry points

```bash
# verify the frozen package (hash manifest + unit tests), read-only
docs/M07-Vision-Language-Navigation/code/scripts/m7/m7_materials_check.sh

# prepare a pinned clone under shared/lightnav/ (dry-run first, then --apply)
docs/M07-Vision-Language-Navigation/code/scripts/m7/m7_prepare_lightnav.sh
docs/M07-Vision-Language-Navigation/code/scripts/m7/m7_prepare_lightnav.sh --apply \
  --upstream /home/seeed/workspace/LightNav-0 \
  --bundle /path/to/dm-h65-course-assets-20261015.tar.gz

# query / start / stop the simulation demo
docs/M07-Vision-Language-Navigation/code/scripts/m7/m7_sim_demo.sh status
```

`m7_prepare_lightnav.sh` never prepares the deployed device project by accident:
writing outside `${WS_ROOT}/shared/lightnav` requires `--allow-external-project`.
`m7_sim_demo.sh start` needs both uv environments and a model service on
`127.0.0.1:8050` — see `M7_DEVICE_RUNBOOK.md`.

## Verification status

See `M7_INTEGRATION_20261019.md` for what was verified when this module was
integrated, and for the checks that were deliberately not run.
