# Third-party notices

This repository is licensed under the Apache License, Version 2.0 (see `LICENSE`).
It contains or interoperates with the third-party work listed below. Where code was
adapted, the original licence continues to apply to that code.

## Vendored / adapted code

### VLN-CE (MIT)

`habitat_server/lightnav_habitat/vlnce_extensions/` (the `VLN-CE-v1` dataset loader
and the `PathLength`, `OracleSuccess`, `StepsTaken` and `NDTW` measures) is adapted
from **VLN-CE: Vision-and-Language Navigation in Continuous Environments**
(Krantz et al., 2020), https://github.com/jacobkrantz/VLN-CE.

```
MIT License

Copyright (c) 2020 Jacob Krantz

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

### Habitat-Lab (MIT)

`habitat_server/lightnav_habitat/objectnav_extensions/objectnav_dataset.py` is a
tolerant re-implementation of the `ObjectNav-v1` dataset loader from
**Habitat-Lab** (Meta Platforms, Inc. and its affiliates),
https://github.com/facebookresearch/habitat-lab, distributed under the MIT License.
Habitat-Lab and Habitat-Sim themselves are not redistributed here; the habitat
server installs them into a conda environment (see `docs/HABITAT_SERVER.md`).

### MolmoSpaces ProcTHOR scene assets (CC BY 4.0)

`mujoco_demo/vln_mujoco/assets/` redistributes the ProcTHOR validation scene
`val_2` and only the THOR meshes, textures, metadata, and scene files that it
references:

- Creator: Allen Institute for AI (Ai2)
- Source: [MolmoSpaces](https://github.com/allenai/molmospaces)
- Pinned revision: `c89e1f5481af56fd25ef4efb76bdced9b726ec6a`
- License: [Creative Commons Attribution 4.0 International (CC BY 4.0)](https://creativecommons.org/licenses/by/4.0/)
- Attribution: Scene and models by the Allen Institute for AI (Ai2), licensed
  under CC BY 4.0.
- Changes: only the files referenced by `val_2_ceiling.xml` were copied; the
  ceiling scene variant is used; environment joints are frozen at runtime.

MolmoSpaces distributes its Objaverse subsets under ODC-BY 1.0 and all other
data subsets under CC BY 4.0. The bundled asset closure contains THOR assets
and **no Objaverse assets** (verified against the manifest: every entry lives
under `objects/thor/` or `scenes/`), so CC BY 4.0 applies. The upstream project
asks users to follow
[Ai2's Responsible Use Guidelines](https://allenai.org/responsible-use).
The exact bundled file list and checksums are recorded in
`mujoco_demo/vln_mujoco/assets/manifest.json`.

## Interoperability only (not redistributed)

### EVT-Bench / TrackVLA (CC BY-NC-SA 4.0)

**EVT-Bench** (the tracking benchmark shipped with TrackVLA,
https://github.com/wsakobe/TrackVLA) — its Habitat-Lab fork, task configs,
`run.py`, `analyze_results.py`, humanoid assets and episode data — is released
under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
License and is **not** redistributed in this repository. Users clone it themselves.

The files under `evt_bench/` are integration glue for that benchmark:

- `evt_bench/trackvla_client_agent.py` — the per-episode evaluation loop
  (`evaluate_agent`) is adapted from EVT-Bench's own evaluation driver
  (`baseline_agent.py`) so that the scores it produces follow the benchmark's published
  protocol. Those adapted portions are therefore provided under CC BY-NC-SA 4.0, not
  under this repository's licence (the file header says so); the WebSocket client class
  and the payload parsing in the same file are original code.
- `evt_bench/run_py.patch` — a small unified diff against upstream `run.py` adding a
  `trackvla` model branch.
- `evt_bench/patch_task_config.py` — writes a patched copy of a benchmark task yaml.

Please observe the NonCommercial and ShareAlike terms of CC BY-NC-SA 4.0 when using
EVT-Bench or results derived from it. See `docs/EVAL_EVT_BENCH.md`.

### MicroDuck robot model and walking policies (Pollen Robotics)

The optional `microduck` backend of the MuJoCo demo (`mujoco_demo/`, `--robot
microduck`) loads two external files at runtime and redistributes neither:

- **Robot model** — the MJCF `robot_allcollisions.xml` and the STL meshes it
  references, from [pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl).
  The repository's code and MJCF are Apache-2.0; its README states that the 3D
  model files are licensed under **Creative Commons BY-NC-SA**. Users clone the
  repository themselves and must observe the NonCommercial and ShareAlike terms
  for the meshes.
- **Walking policy** — `alpha_walking.onnx` from
  [pollen-robotics/microduck-policies](https://huggingface.co/pollen-robotics/microduck-policies)
  on the Hugging Face Hub (Apache-2.0; also vendored in the `policies/`
  directory of [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck)).

`docs/assets/mujoco_microduck.gif` in the README is a screen recording of the
demo and therefore shows renders of the MicroDuck 3D model. Attribution: MicroDuck
by Pollen Robotics, 3D model files CC BY-NC-SA. The model files themselves are
not part of this repository.

### Robot deployment dependencies (`robot_deploy/`)

The ROS 2 workspace under `robot_deploy/` interoperates with, but does not
redistribute:

- **ROS 2 Humble** and its message/client packages (Apache-2.0), installed from
  the ROS repositories.
- **`unitree_sdk2_python`** (BSD-3-Clause, Unitree Robotics,
  https://github.com/unitreerobotics/unitree_sdk2_python) — imported by
  `go2_adapter`; installed by the user.
- **Orbbec ROS 2 driver** (`orbbec_camera` / `orbbec_description`; the ROS
  wrapper is Apache-2.0 and bundles Orbbec's proprietary OrbbecSDK binaries) —
  launched by `vln_bringup`; installed from the ROS repositories.
- **CasADi** (LGPL-3.0, https://casadi.org) — imported by `vln_mpc` as an
  unmodified Python package; installed by the user via pip.
- `aiohttp` (Apache-2.0), `websocket-client` (Apache-2.0), OpenCV (Apache-2.0)
  and NumPy (BSD-3) as regular Python dependencies.

### Benchmark datasets

R2R / RxR (VLN-CE episodes), Matterport3D, HM3D, HM3D-OVON and ObjectNav episode
data are governed by their own licences and terms of use. None of them are
redistributed here; `docs/EVAL_HABITAT.md` describes where they are expected on disk.

## Python dependencies

The package builds on, among others: PyTorch (BSD-3), Hugging Face `transformers`,
`tokenizers`, `safetensors` (Apache-2.0), vLLM (Apache-2.0), NumPy (BSD-3),
Pillow (MIT-CMU), PyYAML (MIT), `websockets` (BSD-3), `pyzmq` (BSD-3 / LGPL for
libzmq). `lightnav.processing.VLNQwen3VLProcessor` subclasses
`transformers`' `Qwen3VLProcessor`; `lightnav.inference.vllm_utils` patches
two vLLM 0.19.x internals at runtime. Model weights are supplied by the user and are
subject to the licence of the checkpoint they came from.

The MuJoCo demo (`mujoco_demo/`) additionally depends on MuJoCo (Apache-2.0),
CasADi (LGPL-3.0-or-later) with Ipopt (EPL-2.0) used through it, aiohttp
(Apache-2.0/MIT), and Pillow (MIT-CMU), plus ONNX Runtime (MIT) when the optional
`microduck` extra is installed — all installed by the user, none redistributed
here.
