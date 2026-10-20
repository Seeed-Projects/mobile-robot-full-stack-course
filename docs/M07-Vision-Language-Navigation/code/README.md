# M7 Code: LightNav and Chassis Simulation

[English](README.md) | [中文](README_zh_CN.md)

This package combines a **pinned LightNav upstream version, the course overlay, and separately acquired binary/CAD assets**. It supports the DM-H65 simulation on Jetson. The frozen runtime source was collected on 2026-10-15; reading this package does not install dependencies, download models, or run navigation.

Start with [7.3: Run the Demo](../7.3_From_Navigation_Prediction_to_Closed_Loop_Control/README_en_US.md). For first installation, follow the [Lab Guide](../7.3_From_Navigation_Prediction_to_Closed_Loop_Control/LAB_GUIDE.md). Apply the overlay to a fresh pinned clone using the preparation tools, rather than copying this directory into a running project.

## Package Layout

| Directory or file | Purpose |
| --- | --- |
| `scripts/m7/` | Material checks, prepared-project setup, and demo start/stop/status wrappers |
| `overlay/` | 34 runtime files: deployment adaptation, chassis backend, visual history, geometric tracking, web UI, and tests |
| [overlay-manifest.json](overlay-manifest.json) | Per-file SHA256 and upstream baseline hashes |
| `configs/` | Chassis geometry, task registration, CAD inventory, and bundle metadata; the preparer relocates historical paths in a copy |
| `locks/` | Separate dependency locks for model and simulation environments |
| `tools/` | Read-only checks and analysis, plus explicitly applied source/asset preparers |
| `examples/custom-chassis/` | URDF inspection, a part-mapping template, and an independent MJCF preview |
| `tests/` | Offline tests of the course tools |
| `teaching/` | Algorithm-animation builders and numerical checks |
| [evidence/](evidence/README.md) | Archived images, raw responses, poses, and GPU records |
| `licenses/`, [THIRD_PARTY.md](THIRD_PARTY.md) | Source and asset terms |
| `docs/` | Operator and historical integration notes |

## Versions and Dependencies

- Upstream commit: `3015508b70fb6e30bbd66701e94c9b916dcb8df8`.
- Model revision: `826dc5fbfa37afa8293d2e336d329b6ffc0bfb64`.
- Model environment: Python 3.10.21, Seeed PyTorch 2.7.0, torchvision 0.22.0, Transformers 5.8.0; HF/CUDA/BF16/SDPA.
- Simulation environment: Python 3.11.16 and MuJoCo 3.11.0.

The overlay is not a complete upstream repository or another model. It imports unchanged upstream modules, including configuration, RVQ, scene loading, robot base, and shared CasADi types. The demo uses geometric lookahead rather than MPC, but shared imports still require dependencies retained in the lock. Simulation reads geometry JSON and does not require a running physical SDK or CAN access.

## Check and Prepare Materials

Run from the **course repository root**:

```bash
python3 docs/M07-Vision-Language-Navigation/code/tools/check_materials.py
python3 -m unittest discover -s docs/M07-Vision-Language-Navigation/code/tests -v
```

Preparation commands default to dry-run. Add `--apply` only when intentionally preparing a fresh project. The overlay installer rejects destination content that matches neither the upstream baseline nor the course copy.

```bash
python3 docs/M07-Vision-Language-Navigation/code/tools/install_overlay.py --project /path/to/fresh/LightNav-0
python3 docs/M07-Vision-Language-Navigation/code/tools/prepare_assets.py \
  --project /path/to/fresh/LightNav-0 --bundle /path/to/dm-h65-course-assets-20261015.tar.gz
```

The wrappers in `scripts/m7/` locate the course repository from their own location. Their default prepared project remains `shared/lightnav/LightNav-0` at the repository root. Use `--project` for an explicit project; see the [operator runbook](docs/M7_DEVICE_RUNBOOK.md). The existing device deployment at `/home/seeed/workspace/LightNav-0` is a separate location.

The demo uses model port 8050, web port 8091, `/api/health`, and `/ws`. Its start message is `start_sim_navigation`, rather than the strict mode's `start_step_navigation`. Model sessions retain the `login/reset/next` protocol.

## Environment Locks and External Binaries

The [model lock](locks/requirements-lightnav-py310.lock) includes transitive dependencies and hashes. Two `file://` URLs refer to the original deployment's wheel paths. The [simulation lock](locks/requirements-demo-cpu.lock) fixes package versions but lacks per-wheel hashes and contains `-e .`. Run its `uv pip sync` from the new project's `mujoco_demo` directory, following the guide, rather than running project-wide `uv sync`.

| External file | Fixed SHA256 |
| --- | --- |
| torch-2.7.0-cp310-cp310-linux_aarch64.whl | `6eff643c0a7acda92734cc798338f733ff35c7df1a4434576f5ff7c66fc97319` |
| torchvision-0.22.0-cp310-cp310-linux_aarch64.whl | `daabff3a0725996886b92e4b5dd143f5750ef4b181b5c7d01371a9185e8f0402` |
| libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive.tar.xz | `5426a897c73a9b98a83c4e132d15abc63dc4a00f7e38266e7b82c42cd58a01e1` |

The wheel hashes were measured from project downloads; cuSPARSELt was checked against NVIDIA metadata. Obtain wheels through the [Seeed reference](https://github.com/Seeed-Projects/reComputer-Jetson-for-Beginners/tree/main/3-Basic-Tools-and-Getting-Started/3.5-Pytorch) and the model from its [fixed revision](https://huggingface.co/LightOriginsHQ/LightNav-0/tree/826dc5fbfa37afa8293d2e336d329b6ffc0bfb64). Archived checks do not validate a new device.

## CAD, Examples, and Recordings

Bundle name, size, and SHA256 are in [asset-bundle.json](configs/asset-bundle.json). A separately held instructor bundle contains 45 meshes. It is not in Git and has no public download URL. Without it, fresh-device DM-H65 reproduction remains blocked at asset preparation; offline evidence exercises are still available.

The [custom-chassis example](examples/custom-chassis/README.md) offers a read-only URDF inventory, mapping template, and independent preview. It does not automatically adapt arbitrary CAD to the fixed DM-H65 loader.

Raw-speed videos are separate instructor materials, not model or log files committed to Git. Archived doorframe failure videos, the five-prediction sample here, and the later 17-prediction refrigerator video are different runs. The refrigerator video shows model stopping on an unscored custom task and is labeled 3× speed.

## Evidence and Maintenance

The archived main run predates the current overlay. Runtime-4 did not include later local path repair; subsequent code adds limited repair and fixes for short-path tracking. Keep original predictions, execution references, and actual trajectories distinct when discussing them. Current evidence does not establish stable strict task success.

Offline teaching sources are in [SlowFast](teaching/slowfast/README.md) and [LightNav animations](teaching/lightnav/README.md) (Chinese maintenance notes). They need neither a model nor a Jetson connection. Package and chapter checksums describe material integrity; they are not fresh-board installation, GPU, or navigation test results.
