# M7 integration record — 2026-10-19

What was integrated into `modules/m07-vision-language-navigation/`, what was
verified, what was deliberately not run, and what is still open. Read this
before re-running any check or extending the module.

## 1. Provenance

| Item | Value |
| --- | --- |
| Upstream | LightNav-0, commit `3015508b70fb6e30bbd66701e94c9b916dcb8df8` |
| Model revision | `826dc5fbfa37afa8293d2e336d329b6ffc0bfb64` |
| Course overlay | 34 files, per-file SHA256 in `overlay-manifest.json` |
| `overlay-manifest.json` | sha256 `9fdc7190bd24b5c711086344080c4eaf6f726a0ef03efa3b793907cf2da46a4f` |
| Package manifest | `SHA256SUMS.txt`, 96 entries, sha256 `83dba8f94569985eaca3efe767ff39676e198e76d34fe777f479f8753bc83013` (was 89 entries / `bc8ff53e…` before `teaching/lightnav/` landed; see §2.5) |
| DM-H65 asset bundle | `dm-h65-course-assets-20261015.tar.gz`, 4,104,931 B, sha256 `23f038fddb7e2f606414f37cbe878aaff79f9cb00e51bee53a3d644fa1cd3a35` |

The package is a **frozen material set**: the 96 hashed files may only change
when the upstream commit, the model revision or the overlay itself changes.
`check_materials.py` verifies every listed file and ignores unlisted ones, so
development extras (`docs/`, `scripts/m7/`, `output/`) never break the check —
they are intentionally outside the manifest.

Manifest coverage (96 entries):

| Path | Entries |
| --- | --- |
| `overlay/` | 34 |
| `evidence/` | 26 |
| `teaching/slowfast/` | 6 |
| `teaching/lightnav/` | 7 |
| `tools/` | 5 |
| `configs/` | 5 |
| `examples/custom-chassis/` | 4 |
| `licenses/` | 2 |
| `locks/` | 2 |
| `tests/` | 2 |
| `README.md`, `THIRD_PARTY.md`, `overlay-manifest.json` | 3 |

## 2. What was changed during integration

1. Module renamed `modules/m07-vln-sim` → `modules/m07-vision-language-navigation`
   to match the `mNN-topic` convention of the other modules. Four hashed package
   files still referenced the old path and were updated, then `SHA256SUMS.txt`
   was regenerated (8 hash lines changed). Those four files are the only
   frozen-package files whose bytes differ from the original release:
   `README.md`, `evidence/README.md`, `examples/custom-chassis/README.md`,
   `teaching/slowfast/README.md`. Every other hashed file is byte-identical to
   the released package.
2. Companion chapter documents restored under
   `docs/M07-Vision-Language-Navigation/` (10 notes + `validation/`), with
   `SHA256SUMS.txt` (143 entries) regenerated for that directory.
3. Development extras added, both outside the frozen manifest:
   `docs/README.md`, `docs/M7_DEVICE_RUNBOOK.md`, and
   `scripts/m7/{m7_materials_check.sh,m7_prepare_lightnav.sh,m7_sim_demo.sh}`.
4. The workspace `modules/README.md`, `README.md` and `.gitignore` were updated
   for the new module; `shared/lightnav/` stays gitignored as the only place the
   prepare script may write.
5. `teaching/lightnav/` (7 files, the RVQ/SE(2) material behind the 7.2.2 and
   7.2.4–7.2.7 animations) was authored in a parallel session and **landed after
   the first commit**. It is now part of the frozen package: hashed into
   `SHA256SUMS.txt` (89 → 96 entries), verified by `check_materials.py` like
   every other listed file, and synced to the device. It stays offline and
   deterministic — fixed upstream functions, one frozen decode, no model load,
   no demo hot path — so it does not weaken the "frozen material set" property.
   Device commit `83c5126` on top of `05ab5be`.

`docs/M07-Vision-Language-Navigation/7.2_Semantic_Grounding_and_Learned_VLN/`
was **not modified** on purpose — see §6.

## 3. Verification performed

### 3.1 Frozen package (Jetson, `m7_materials_check.sh`)

Green: hash manifest matches all 96 entries, 18 unit tests pass, no network, no
model import. Also green on the authoring machine against the same tree. Re-run
after `teaching/lightnav/` landed (device commit `83c5126`): `CHECK_EXIT=0`,
`Ran 18 tests … OK`.

### 3.2 Fresh-clone rehearsal (Jetson, `/tmp`, isolated)

A clean clone of the pinned upstream was prepared and driven end to end with
the course tools only (`output/rehearsal-20261019-111701.log` on the device):

| Step | Result |
| --- | --- |
| Fresh clone | clean at `3015508b…`, 215 MB |
| `install_overlay.py` dry-run | `files_to_copy: 34` |
| `install_overlay.py --apply` | `files_to_copy: 34` |
| Re-run dry-run | `files_to_copy: 0` — overlay is idempotent |
| Independent sha256 of the 34 applied files | `bad = []` |
| `prepare_assets.py` dry-run then `--apply` | `mesh_count: 45`, 48 files under `course-assets/dm-h65/` |
| `preflight.py` | exit 1, with exactly the two expected gaps: `.venv-lightnav-py310` and `mujoco_demo/.venv-cpu` absent in a fresh clone. `geometry_matches: true` |
| `analyze_run.py` on the archived 5-prediction run | `initial_distance_m: 1.5243` → `final_distance_m: 0.0966`; prediction-05 collision `doorframe_6a1094b9eed158e940bf50ec6c6a9f9f_1_1_2_collision_3`; `request_response_latency_s` mean `1.9910`, p95 `2.7455`; `strict_success_reported: false`, `success_independently_reverified: false` |

### 3.3 Services untouched

Before and after the rehearsal: ports `8765` (SDK WebUI) and `8050`
(lightnav-serve) unchanged; live project HEAD `3015508b…`; live `git status
--porcelain` sha1 `e03dfe75c8505be5c31644c438596275085b08ca` unchanged; managed
status reports `ready: true`. Nothing was started or stopped. The plain
`curl /api/health` on 8050 returning HTTP 426 is expected — that endpoint is
WebSocket-only; use `scripts/jetson_lightnav.sh status`.

### 3.4 Script behaviour

| Test | Result |
| --- | --- |
| `m7_prepare_lightnav.sh` against a path outside `${WS_ROOT}/shared/lightnav` without `--allow-external-project` | refused, exit 1, nothing created |
| dry-run against the prepared clone | exit 0 |
| `m7_sim_demo.sh status` (no venvs) | `{"running": false}` |
| `m7_sim_demo.sh start` (no sim venv) | refused, exit 1 |
| `output/` in the module | ignored by the workspace `.gitignore` (`**/output/`), so run logs are never committed |

### 3.5 Course repository documentation

| Check | Result |
| --- | --- |
| `markdownlint-cli2@0.23.3` | 0 issues in 73 files |
| `lychee 0.24.2 --offline` | 633 total, 460 OK, 172 excluded, **1 error** — only the pre-existing 7.2 embed (see §6.2). The two rename-induced errors are gone: 7.2's old `m07-vln-sim` links were fixed in the parallel session |

## 4. Deliberately not run

- `mujoco_demo/tests/test_sim_demo*.py` and
  `mujoco_demo/tests/sim_demo_overlay.test.cjs` — they need the py3.11 sim
  environment and Node, which the rehearsal clone does not carry.
- Any model service, demo or web UI start/stop. Ports 8050/8091/8765 were
  observed only.
- No `scripts/build.sh`: this module is a non-ROS material package, so colcon
  has nothing to build here.
- The course repository was **not committed** in this round; only the Jetson
  workspace side was staged for a local commit.

## 5. Rehearsal / temp artefacts to clean

- Jetson: `/tmp/m7-verify-20261019` (215 MB clone),
  `/tmp/dm-h65-course-assets-20261015.tar.gz`, `/tmp/m7-rehearsal.sh`.
  All are disposable; every summary worth keeping is in §3.
- The rehearsal log itself,
  `modules/m07-vision-language-navigation/output/rehearsal-20261019-111701.log`,
  stays where it is: `output/` is gitignored.

## 6. Known gaps (open, none of them blocking)

1. ~~**7.2 still points at the old module name.**~~ **Resolved** by the parallel
   session: 7.2 now has zero `m07-vln-sim` references and links to
   `modules/m07-vision-language-navigation/…`. The two lychee errors this caused
   are gone. I never edited that file.
2. **lychee error at 7.2 line 18**: `![[Pasted image 20261016153406.png]]` is an
   Obsidian embed whose PNG lives at the repository root, not next to the
   chapter. Pre-existing, unrelated to this integration, and the only remaining
   link error in the tree.
3. `docs/M07-Vision-Language-Navigation/SHA256SUMS.txt` covers the chapter
   documents and drifts whenever those files are re-edited. At the time of
   writing it has 148 entries and 5 stale hashes (`7.2/README_zh_CN.md`,
   `IMAGE_PROMPTS.md`, `MEDIA_INDEX.md`, `SOURCE_NOTES.md`, `VALIDATION.md`) —
   the parallel session regenerated it mid-edit. Regenerate it with the same
   recipe **after** that session settles; do not race it.
4. Historical `validation/` records keep the old module name `m07-vln-sim`
   inside their archived run descriptions. They are records of past runs and
   were intentionally not rewritten.
5. ~~`teaching/lightnav/` is concurrent in-flight work.~~ **Folded in** — see
   §2.5. It is hashed, checked and synced. Only remaining caveat: the
   `docs/M07-Vision-Language-Navigation/assets/lightnav/` renders it produces
   live in the docs tree, not in the package, so they are covered by the docs
   manifest (§6.3), not by `check_materials.py`.

## 7. Reproducing the verification

```bash
WS_ROOT=/home/seeed/workspace/mobile_robot_ws
M7=$WS_ROOT/modules/m07-vision-language-navigation

# 1. frozen package integrity (read-only, no network, no model)
"$M7/scripts/m7/m7_materials_check.sh"

# 2. prepare an isolated clone (dry-run first, then --apply)
"$M7/scripts/m7/m7_prepare_lightnav.sh" --project /tmp/verify/LightNav-0 \
  --bundle /path/to/dm-h65-course-assets-20261015.tar.gz
"$M7/scripts/m7/m7_prepare_lightnav.sh" --apply --project /tmp/verify/LightNav-0 \
  --upstream /home/seeed/workspace/LightNav-0 \
  --bundle /path/to/dm-h65-course-assets-20261015.tar.gz

# 3. gap report against that clone (exit 1 is expected before the venvs exist).
#    The configs come from the extracted bundle, not from configs/ — that is
#    what the rehearsal used.
"$M7/tools/preflight.py" --project /tmp/verify/LightNav-0 \
  --chassis-config /tmp/verify/LightNav-0/course-assets/dm-h65/config.json \
  --task-config    /tmp/verify/LightNav-0/course-assets/task.json

# 4. offline analysis of the archived evidence, no service required
"$M7/tools/analyze_run.py" "$M7/evidence" --state "$M7/evidence/actual-state.jsonl"
```
