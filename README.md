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

The work is organised in three phases, mirrored by the layout of `scripts/`:

| Phase | Folder | Does | Writes |
|---|---|---|---|
| **1 — Setup** | `scripts/setup/` | Prepare assets and build the simulation world: derive USDs, calibrate grasps, extract the collision world, derive goal sets, generate loaded scenes | `assets/`, `docs/` reports |
| **2 — Experiment** | `scripts/experiment/` | Run a robotic algorithm in that world and record what happened | `results/experiments/<run_id>/` |
| **3 — Evaluation** | `scripts/evaluation/` | Compute metrics and render videos **from those artifacts** — never re-planning or re-simulating | `results/evaluation/`, `media/` |

**The planner is pluggable.** `run_trials.py --planner <name>` selects the algorithm; the
runner never names one. Shipped: `rrt_connect` (default), `rrt_star`, `bit_star`, `prm`.
Adding another is two steps and touches no experiment code — implement `Planner.plan()` in
`src/dishsim/planners/` and register it there. Planners declare their own capabilities: `prm`
sets `supports_multi_goal = False`, because the roadmap planners in this OMPL build never
terminate on a multi-state goal, so the base class hands it the nearest goal instead.

**Phase 3 is decoupled by construction.** An experiment records the measured scene state every
physics step (~170 KB per trial, versus an 11 MB video); evaluation replays that recording
kinematically to render video. So experiments run fast and headless, any trial can be
re-rendered later from a different angle or resolution, and metrics never touch a simulator.
The replay is verified rather than assumed: `verify_replay.py` reproduces every recorded link
pose to **1e-7 m**, and a rendered clip scores **38 dB PSNR** against a live capture of the
same trial.

What ships around the scene:

- **`src/dishsim/`** — the environment package: scene/robot configs, per-object registry
  (`config.py`, every tunable in one place), procedural rack + prop generators, USD
  derivation, analytic UR5e IK (8 branches, Pinocchio-validated), and a **Kit-free FCL
  collision world** mirroring the PhysX scene at 100 % measured parity — built for thousands
  of fast queries by external planners.
- **A fully-loaded scene generator** — a deterministic, FCL-validated 34-item fill that
  physically settles a complete load (31 items stable, racks close): initial states for
  rearrangement planning, IL demonstrations, or RL resets.

Everything runs headless on a single GPU (Isaac Sim **6.0.1** + Isaac Lab **3.0.0**); only
video rendering needs `--enable_cameras`. One frame convention throughout: robot-base frame,
meters, Z-up, XYZW quaternions. Reference docs: [environment](docs/environment.md) (setup
landmines), [joint report](docs/joint_report.md) (measured articulation numbers),
[success criteria](docs/success_criteria.md) (task definitions),
[grasp calibration](docs/grasp_calibration.md) and [asset survey](docs/asset_survey.md)
(measurement provenance).

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
scripts/run_kit.sh scripts/setup/inspect_scene.py --headless --test_door

# 3. object library (mug from the Isaac asset bucket; the rest from YCB scans + procedural)
scripts/run_kit.sh scripts/setup/make_prop_physics_usd.py --object 025_mug
scripts/run_kit.sh scripts/setup/build_object_assets.py

# 4. verify: Kit smoke test + planning-stack tests
scripts/run_kit.sh scripts/setup/kit_smoke.py --headless --enable_cameras
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest tests/
```

**Fast path** — if you have access to the private asset archive (a Hugging Face dataset
holding the built props and geometry caches; keeps ~1.5 h of rebuilds off the clock), run
step 1, then:

```bash
huggingface-cli login   # once
/workspace/isaaclab/env_isaaclab/bin/python scripts/tools/restore_assets.py --with_media
```

> `./isaaclab.sh -p` exits 0 even when the wrapped script crashes — judge success from log
> content (`[RESULT] PASS`, absence of tracebacks), never from the exit code.

## Usage

Run everything from the repo root. Kit scripts go through `scripts/run_kit.sh`; venv scripts
run with `/workspace/isaaclab/env_isaaclab/bin/python`, written `$PY` below.

### Phase 1 — Setup: build the world

Build and validate the collision world for a machine state (`both_out`, `both_in`,
`placement`, `placement_open`) and a carried object (`--object`):

```bash
scripts/run_kit.sh scripts/setup/extract_geometry.py --headless --scenario placement  # extract
$PY scripts/setup/decompose_meshes.py --scenario placement                            # FCL pieces
scripts/run_kit.sh scripts/setup/parity_check.py --headless --scenario placement      # FCL vs PhysX
```

Derive the placement slots and IK goal sets that experiments plan to:

```bash
scripts/run_kit.sh scripts/setup/goal_configs.py --headless --enable_cameras --object mug
```

Generate a fully-loaded machine (34-item deterministic fill, per-item stability gates,
rack-closability check, timelapse + stills):

```bash
scripts/run_kit.sh scripts/setup/capacity_fill.py --headless --enable_cameras
```

**Add or recalibrate an object**: add its spec to `config.OBJECTS`, build the asset
(`setup/build_object_assets.py`), measure the pinch (`setup/check_scene.py --measure`, then
`setup/calibrate_grasp.py --object <name>`), and freeze the measured constants
(`setup/freeze_calibration.py --object <name>`). Never eyeball-edit measured values — every
number traces to a calibration or inspection run.

### Phase 2 — Experiment: run an algorithm

```bash
# default planner; no cameras needed — the trajectory is recorded either way
scripts/run_kit.sh scripts/experiment/run_trials.py --headless \
    --scenario both_out --object mug --slots 2,7 --seeds 0-1

# choose and tune the algorithm; --save_plan_debug also records the search tree
scripts/run_kit.sh scripts/experiment/run_trials.py --headless --planner rrt_star \
    --planner_param range_rad=0.3 --planner_param budget_s=15 --slots 7 --seeds 0 \
    --save_plan_debug

# --live_video additionally captures inline (only needed for the replay A/B check)
```

Each run writes one self-contained directory:

```
results/experiments/<run_id>/
  manifest.json              run provenance: planner + params, object, scenario, config hash
  trials/<trial>.json        outcome, timings, placement error, failure stage
  trajectories/<trial>.npz   measured state per physics step — the Phase 3 input
  plans/<trial>.npz          the planning query + search tree (with --save_plan_debug)
results/experiments/LATEST   the newest run id
```

`--run_id` names a run (default `<object>_<scenario>_<planner>_<UTC>`). Success criteria per
placement mode are in [docs/success_criteria.md](docs/success_criteria.md).

To compare algorithms, run the same slots and seeds under each planner and let Phase 3 build
the table:

```bash
for p in rrt_connect rrt_star bit_star; do
  scripts/run_kit.sh scripts/experiment/run_trials.py --headless --planner $p \
      --slots 7 --seeds 0 --run_id "cmp_$p"
done
```

### Phase 3 — Evaluation: read the artifacts

None of these re-plan or re-run physics; the first and last need no Kit at all.

```bash
# metrics + figures for the latest run; --all adds the planner-comparison table
$PY scripts/evaluation/compute_metrics.py --all

# prove a recording replays exactly (kinematics only, no cameras, seconds)
scripts/run_kit.sh scripts/evaluation/verify_replay.py --headless \
    --trial results/experiments/<run_id>/trajectories/trial_07_00_0.npz

# render video from recordings — one clip per camera, plus a final still
scripts/run_kit.sh scripts/evaluation/render_videos.py --headless --enable_cameras \
    --run_dir results/experiments/<run_id>/trajectories

# the planner's search tree, drawn from the recorded query
$PY scripts/evaluation/plan_visual.py --slot 7 --seed 0

# A/B a replayed clip against a live capture (the decoupling acceptance test)
$PY scripts/evaluation/compare_videos.py --live <live.mp4> --replay <replay.mp4>
```

Metrics land in `results/evaluation/<run_id>/`, videos in `media/trials/<run_id>/`.

Note the phase boundary: a trial's success verdict depends on live contact forces, so it is
decided in Phase 2 and only *aggregated* in Phase 3. Placement geometry is recomputable from
a recorded pose, and evaluation cross-checks it against the trial record.

### Archive / restore the generated artifacts

```bash
$PY scripts/tools/archive_assets.py --upload
$PY scripts/tools/restore_assets.py --with_media
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
  the RRT-Connect, RRT*, BIT* and PRM implementations behind `dishsim.planners`.
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
