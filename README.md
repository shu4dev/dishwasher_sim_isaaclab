# dishwasher_sim_isaaclab

## Overview

Isaac Sim environments for loading a dishwasher with a robot arm — a substrate for
**classical motion planning** (included), **imitation learning**, and **reinforcement
learning** policies.

![fully loaded dishwasher](docs/figures/loaded_iso.png)

The scene: an articulated ArtVIP dishwasher (hinged door, two sliding racks — replaced with
procedurally generated, reference-styled wire racks plus a 3-bay cutlery basket), a UR5e +
Robotiq 2F-85 on a pedestal, and a 15-class kitchen-object library (plates, saucers, bowls,
mugs, cups, tumblers, wine glasses, cutlery, serving utensils, pitcher, food container)
scaled to fit the compact machine, each with measured grasp and placement specifications.

What ships around the scene:

- **`src/dishsim/`** — the environment package: scene/robot configs, per-object registry
  (`config.py`, every tunable in one place), procedural rack + prop generators, USD
  derivation, analytic UR5e IK (8 branches, Pinocchio-validated), and a **Kit-free FCL
  collision world** mirroring the PhysX scene at 100% measured parity — built for
  thousands of fast queries by external planners.
- **Classical planning runner** — OMPL RRT-Connect in 6-D joint space: rack
  reconfiguration, countertop pick with a calibrated contact pinch, collision-monitored
  execution, release, and per-mode placement evaluation (standing cells, plate tine slots,
  bowl lean, cutlery-basket drops).
- **Fully-loaded scene generator** — a deterministic, FCL-validated 34-item fill that
  physically settles a complete load (31 items stable, racks close) — initial states for
  rearrangement planning, IL demonstrations, or RL resets.

Everything runs headless on a single GPU (Isaac Sim **6.0.1** + Isaac Lab **3.0.0**);
media capture needs `--enable_cameras`. One frame convention throughout: robot-base frame,
meters, Z-up, XYZW quaternions. Reference docs: [docs/environment.md](docs/environment.md)
(setup landmines), [docs/joint_report.md](docs/joint_report.md) (measured articulation
numbers), [docs/success_criteria.md](docs/success_criteria.md) (task definitions),
[docs/grasp_calibration.md](docs/grasp_calibration.md) and
[docs/asset_survey.md](docs/asset_survey.md) (measurement provenance).

## Installation

Requires an Isaac Sim 6.0.1 / Isaac Lab 3.0.0 install at `/workspace/isaaclab` (this repo
nests inside that tree as an independent git repo). All Kit-side scripts run through
`scripts/run_kit.sh`; see [docs/environment.md](docs/environment.md) for why, and for the
launcher landmines.

```bash
# 1. planning venv (the wrapper auto-detects this exact path)
/isaac-sim/kit/python/bin/python3 -m venv --system-site-packages /workspace/isaaclab/env_isaaclab
/workspace/isaaclab/env_isaaclab/bin/pip install 'trimesh==4.12.2' ompl python-fcl coacd \
    matplotlib imageio pytest requests pyyaml filelock tqdm fsspec
/workspace/isaaclab/env_isaaclab/bin/pip install -e .

# 2. dishwasher asset (~82 MB) + derived USDs + joint report
scripts/run_kit.sh -c "from huggingface_hub import snapshot_download; \
  snapshot_download(repo_id='X-Humanoid/ArtVIP', repo_type='dataset', \
  allow_patterns=['Articulated_objects/major_appliances/dishwasher/**'], local_dir='assets/artvip')"
scripts/run_kit.sh scripts/00_inspect_scene.py --headless --test_door

# 3. object library (mug from the Isaac asset bucket; the rest from YCB scans + procedural)
scripts/run_kit.sh scripts/01_make_prop_physics_usd.py --object 025_mug
scripts/run_kit.sh scripts/03_build_object_assets.py

# 4. verify: Kit smoke test + planning-stack tests
scripts/run_kit.sh scripts/05_kit_smoke.py --headless --enable_cameras
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest tests/
```

**Fast path** — if you have access to the private asset archive (a Hugging Face dataset
holding the built props and geometry caches; keeps ~1.5 h of rebuilds off the clock), run
step 1, then:

```bash
huggingface-cli login   # once
/workspace/isaaclab/env_isaaclab/bin/python scripts/36_restore_assets.py --with_media
```

> `./isaaclab.sh -p` exits 0 even when the wrapped script crashes — judge success from log
> content (`[RESULT] PASS`, absence of tracebacks), never from the exit code.

## Usage

Run everything from the repo root. Kit scripts take `--headless` (+ `--enable_cameras` for
media); venv scripts run with `/workspace/isaaclab/env_isaaclab/bin/python`.

**Build and validate the collision world** (per machine state: `both_out`, `both_in`,
`placement`, `placement_open`; per carried object via `--object`):

```bash
scripts/run_kit.sh scripts/12_extract_geometry.py --headless --scenario placement   # extract
env_isaaclab/bin/python scripts/13_decompose_meshes.py --scenario placement         # FCL pieces
scripts/run_kit.sh scripts/14_parity_check.py --headless --scenario placement       # FCL vs PhysX
```

**Generate placement goal sets** (slots + IK goal configurations for the active object's
placement mode):

```bash
scripts/run_kit.sh scripts/15_goal_configs.py --headless --enable_cameras --object mug
```

**Run classical planning trials** (pick → RRT-Connect plan → place → evaluate; per-trial
JSON under `results/`, MP4 + stills under `media/`):

```bash
# full choreography incl. robot rack reconfiguration, from an initial machine state
scripts/run_kit.sh scripts/20_plan_and_place.py --headless --enable_cameras \
    --scenario both_out --slots 2,7 --seeds 0-1
# per-object demo in a pre-positioned state
scripts/run_kit.sh scripts/20_plan_and_place.py --headless --enable_cameras \
    --object tumbler --scenario placement --slots 2,7 --seeds 0
```

Success criteria per placement mode are defined in
[docs/success_criteria.md](docs/success_criteria.md); planner internals can be visualized
with `scripts/21_plan_visual.py` (venv, no Kit).

**Generate a fully-loaded scene** (34-item deterministic fill, per-item stability gates,
rack-closability check, timelapse/orbit/still media):

```bash
scripts/run_kit.sh scripts/25_capacity_fill.py --headless --enable_cameras
```

**Add or recalibrate an object**: add its spec to `config.OBJECTS`, build the asset
(`scripts/03_build_object_assets.py`), measure the pinch
(`scripts/10_v0_scene.py --measure`, then `scripts/11_calibrate_grasp.py --object <name>`),
and freeze the measured constants (`scripts/freeze_calibration.py --object <name>`). Never
eyeball-edit measured values — every number traces to a calibration or inspection run.

**Archive / restore the generated artifacts** (private HF dataset):

```bash
env_isaaclab/bin/python scripts/35_archive_assets.py --upload
env_isaaclab/bin/python scripts/36_restore_assets.py --with_media
```

## Credits

- **ArtVIP dishwasher** — [`X-Humanoid/ArtVIP`](https://huggingface.co/datasets/X-Humanoid/ArtVIP)
  dataset (Apache-2.0): the articulated `dishwasher_2` asset (door + sliding racks).
- **YCB Object & Model Set** — Calli et al., *"The YCB Object and Model Set"* (IEEE ICAR
  2015), [ycbbenchmarks.com](https://www.ycbbenchmarks.com/): textured `google_16k` scans
  (plate, bowl, cups, cutlery, spatula, pitcher), used under the YCB dataset terms; the mug
  comes from NVIDIA's Isaac Sim YCB mirror.
- **NVIDIA Isaac Sim / Isaac Lab & Omniverse asset library** — simulator, PhysX ground
  truth, and the pre-assembled UR5e + Robotiq 2F-85 (NVIDIA Omniverse asset EULA; fetched
  at spawn, derived copies stay local).
- **OMPL** — Șucan, Moll, Kavraki, *"The Open Motion Planning Library"* (IEEE RAM 2012):
  RRT-Connect planning.
- **FCL / python-fcl** — Pan, Chitta, Manocha, *"FCL: A general purpose library for
  collision and proximity queries"* (ICRA 2012): the Kit-free collision world.
- **CoACD** — Wei et al., *"Approximate Convex Decomposition for 3D Meshes with
  Collision-Aware Concavity and Tree Search"* (SIGGRAPH 2022): convex decomposition of
  concave bodies.
- **trimesh** — mesh processing throughout the asset and collision pipelines.
- **Pinocchio** — Carpentier et al.: independent validation of the analytic UR5e kinematics
  in the test suite.
- Rack geometry is procedurally generated, styled after publicly documented Whirlpool,
  Bosch, and Frigidaire rack designs (design reference only; no third-party geometry).

Downloaded and derived assets are never committed (`assets/`, `media/`, `results/` are
gitignored); the asset archive must remain private, as it contains NVIDIA-EULA- and
YCB-derived files.
