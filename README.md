<div align="center">
  <h1 align="center"> dishwasher_sim_isaaclab </h1>
  <h3 align="center"> Isaac Sim environments for robotic dishwasher loading </h3>
  <p align="center">
    Classical motion planning · Imitation learning · Reinforcement learning
  </p>
  <p align="center">
    <img src="https://img.shields.io/badge/Isaac%20Sim-4.5.0-76B900?style=flat&logo=nvidia&logoColor=white" alt="Isaac Sim 4.5.0"/>
    <img src="https://img.shields.io/badge/Isaac%20Lab-2.1.1-76B900?style=flat&logo=nvidia&logoColor=white" alt="Isaac Lab 2.1.1"/>
    <img src="https://img.shields.io/badge/Python-3.12-3776AB?style=flat&logo=python&logoColor=white" alt="Python 3.12"/>
    <img src="https://img.shields.io/badge/License-BSD--3--Clause-blue?style=flat" alt="BSD-3-Clause"/>
  </p>
</div>

<table align="center">
  <tr>
    <td align="center">
      <img src="docs/figures/loaded_iso.png" width="720" alt="fully loaded dishwasher"/>
      <br/>
      <code>scripts/setup/capacity_fill.py</code> — 29 items placed, 27 settle stably, racks still close
    </td>
  </tr>
</table>

> **Read this first.** Every Kit script runs through `scripts/run_kit.sh` (on the host it
> forwards into the `dishsim-isaac` container), Kit-free python through `scripts/run_py.sh`,
> success is judged from **log content**, never exit codes (`isaaclab.sh -p` exits 0 on
> crashes), and the Isaac Sim 4.5.0 / Isaac Lab 2.1.1 pins must not be changed. The full
> list of launcher landmines and why they exist: [docs/environment.md](docs/environment.md).

## 1 Overview

This project is built on **Isaac Lab** to simulate a **UR5e + Robotiq 2F-85** loading an
articulated **ArtVIP dishwasher**, as a substrate for classical motion planning (shipped),
imitation learning, and reinforcement learning.

The scene: a hinged-door dishwasher whose two sliding racks are replaced with procedurally
generated, reference-styled wire racks plus a 3-bay cutlery basket; a robot on a pedestal; and
a 14-class kitchen-object library scaled to fit the compact machine, each class carrying
*measured* grasp and placement specifications.

Work is organised in three phases, mirrored by the layout of `scripts/`:

| Phase | Folder | Does | Writes |
|---|---|---|---|
| **1 — Setup** | `scripts/setup/` | Prepare assets and build the simulation world: derive USDs, calibrate grasps, extract the collision world, derive goal sets, generate loaded scenes | `assets/`, `docs/` reports |
| **2 — Experiment** | `scripts/experiment/` | Run a robotic algorithm in that world and record what happened | `results/experiments/<run_id>/` |
| **3 — Evaluation** | `scripts/evaluation/` | Compute metrics and render videos **from those artifacts** — never re-planning or re-simulating | `results/evaluation/`, `media/` |

<table align="center">
  <tr>
    <td align="center">
      <img src="docs/figures/object_library.png" width="300" alt="object library"/>
      <br/>
      asset pipeline (git history)
      <br/>
      Kitchen-object library
    </td>
    <td align="center">
      <img src="docs/figures/rack_geometry.png" width="300" alt="procedural rack"/>
      <br/>
      <code>setup/preview_rack.py</code>
      <br/>
      Procedural rack + cutlery basket
    </td>
    <td align="center">
      <img src="docs/figures/slot_detection.png" width="300" alt="slot derivation"/>
      <br/>
      <code>setup/goal_configs.py</code>
      <br/>
      Slot derivation from rack geometry
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="docs/figures/planner_tree.png" width="300" alt="RRT-Connect search tree"/>
      <br/>
      <code>evaluation/plan_visual.py</code>
      <br/>
      RRT-Connect search trees in workspace
    </td>
    <td align="center">
      <img src="docs/figures/trial_replay.png" width="300" alt="placement rendered from a recording"/>
      <br/>
      <code>evaluation/render_videos.py</code>
      <br/>
      Placement rendered <i>from the trajectory file</i>
    </td>
    <td align="center">
      <img src="docs/figures/grasp_force_vs_theta.png" width="300" alt="grasp calibration"/>
      <br/>
      <code>setup/calibrate_grasp.py</code>
      <br/>
      Measured pad force vs. aperture
    </td>
  </tr>
</table>

### 1.1 The planner is pluggable

`run_trials.py --planner <name>` selects the algorithm; the experiment runner never names one.
A planner implements one method — `plan(world, start_q, goal_qs, seed, debug)` — and declares
its own capabilities. Adding one touches no experiment code (§7).

| Planner | Kind | Multi-goal | Defaults |
|---|---|---|---|
| `rrt_connect` *(default)* | Bidirectional RRT, returns on first connection | ✅ | `range_rad=0.5`, `budget_s=20` |
| `rrt_star` | Asymptotically optimal RRT; always uses the full budget | ✅ | `range_rad=0.5`, `budget_s=10` |
| `bit_star` | Batch Informed Trees; optimizing, batch-sampled | ✅ | `budget_s=10` |
| `prm` | Probabilistic roadmap, rebuilt per query | ❌ | `budget_s=10` |

> **Note:** `prm` declares `supports_multi_goal = False` because the roadmap planners in this
> OMPL build never terminate on a multi-state `ob.GoalStates` — measured at raw-OMPL level with
> a trivial validity checker (1 goal solves in 0.1 s, 2 goals never returns), while `bit_star`,
> `rrt_star` and `KPIECE1` solve the same 2-goal query in under a second. Every real query here
> is multi-goal, so the base class hands `prm` the goal nearest the start instead of hanging.

### 1.2 Evaluation is decoupled from execution

An experiment records the measured scene state every physics step (**~170 KB per trial**,
versus 10–14 MB for one rendered clip); evaluation replays that recording kinematically to
render video afterwards. So experiments run fast and headless, any trial can be re-rendered
later from a different camera or resolution, and metrics never touch a simulator.

The replay is verified, not assumed: `verify_replay.py` reproduces every recorded link pose to
**1e-7 m**, and a rendered clip scores **38 dB PSNR** (median 38.9, p1 31.3) against a live
capture of the same trial with perfect frame alignment.

### 1.3 Object library

Sourced from YCB scans or generated procedurally, then **scaled to fit** — the machine is
compact (lower rack 366 × 287 mm, 154 mm inter-rack clearance, 30 mm tine pitch), so a
full-size dinner plate cannot nest between the tines; the plate here is scaled to 141 mm
across. Each `scale` below documents the factor against its source asset; the authoring
pipeline re-measured every built asset and refused to write one that disagreed with the
registry by more than 2 mm (the pipeline was retired with the public-asset release — it lives
in git history, and the archive ships its outputs).

| Class | Source | Scale | Placement mode | Rack |
|---|---|---|---|---|
| `mug` | YCB `025_mug` | 0.85 | `floor_stand` | lower |
| `plate` | YCB `029_plate` | 0.54 | `plate_slot` | lower |
| `saucer` | YCB `029_plate` | 0.42 | `plate_slot` | lower |
| `bowl` | YCB `024_bowl` | 0.68 | `floor_stand` | lower |
| `cup` | YCB `065-a_cups` | 1.10 | `floor_stand` | lower |
| `fork` | YCB `030_fork` | 0.60 | `basket_drop` | basket |
| `spoon` | YCB `031_spoon` | 0.60 | `basket_drop` | basket |
| `knife` | YCB `032_knife` | 0.60 | `basket_drop` | basket |
| `spatula` | YCB `033_spatula` | 0.45 | `flat_lay` | upper |
| `tumbler` | procedural | 1.00 | `floor_stand` | lower |
| `wine_glass` | procedural | 1.00 | `stem_scallop` | upper |
| `serving_spoon` | procedural | 1.00 | `basket_drop` | basket |
| `container` | procedural | 1.00 | `upside_down` | upper |
| `lid` | procedural | 1.00 | `flat_lay` | upper |

Machine states: `both_out` and `both_in` (the two robot-facing initial states, each with a
rack-reconfiguration action), plus the internal states `placement` and `placement_open` that
trials place in.

### 1.4 Reference documentation

| Doc | Contents |
|---|---|
| [docs/environment.md](docs/environment.md) | Hardware/software stack, docker runtime, Isaac Lab 2.1 port notes, OMPL 2.0 nanobind notes, **launcher landmines (canonical)** |
| [docs/architecture.md](docs/architecture.md) | Code structure, the task/planner layer boundary, completed one-off studies |
| [docs/episodes.md](docs/episodes.md) | The multi-object episode runner manual (CLI, slot assignment, rack actions) |
| [docs/success_criteria.md](docs/success_criteria.md) | Placement success definition per placement mode, slot model, the reachability success bar |
| [docs/known_limitations.md](docs/known_limitations.md) | Honest negative results and open items, with measured evidence |
| [docs/extending.md](docs/extending.md) | Add an object class / placement mode / machine state |
| [docs/joint_report.md](docs/joint_report.md) | *Auto-generated* by `setup/inspect_scene.py`: measured articulation numbers every constant derives from |
| [docs/grasp_calibration.md](docs/grasp_calibration.md) | *Auto-generated* by `setup/calibrate_grasp.py`: θ_touch/θ_grasp, per-pad force band, mimic signs |
| [docs/asset_survey.md](docs/asset_survey.md) | Survey of the 7 ArtVIP dishwasher variants justifying the `dishwasher_2` pick |
| [docs/figures/README.md](docs/figures/README.md) | Provenance of every tracked figure (producing command + media source) |

## 2 Environment Setup

### 2.1 Prerequisites

Docker with the NVIDIA container runtime and an NVIDIA driver ≥ 535.129 (the 4.5.0 pin's
documented series). The whole runtime — Isaac Sim 4.5.0, Isaac Lab v2.1.1 and the planning
deps — is baked into the repo-owned image; nothing installs on the host. Developed and
tested on the corallab workstation (3× RTX 3090 / 36 cores; GPU 1 by default, `DISHSIM_GPU`
overrides); OMPL planning is CPU-bound, so core count matters more than the GPU except when
rendering. Everything runs `--headless`; only *rendering* additionally needs
`--enable_cameras`.

### 2.2 Runtime image + container

```bash
docker build -f docker/Dockerfile -t dishsim-isaac:4.5.0 .
docker compose -f docker/compose.yaml up -d      # long-lived container `dishsim-isaac`
scripts/setup/mirror_robot_usd.sh                # UR5e+2F-85 mirror (6.0 asset, see script header)
```

`requirements-planning.txt` pins the measured working set (the table in
[docs/environment.md](docs/environment.md) is the measurement of record); the Dockerfile
installs it plus pytest and the archive tooling into Kit's python — no venv. The compose
file keeps every bulky mutable path (assets, media, results, Kit caches, `HF_HOME`) on
`/media/corallab-s1/2tbhdd/brianshu/dishsim`; the repo's data dirs are symlinks there.

### 2.3 Assets (public archive — the one-command path)

Every asset this project uses is publicly redistributable with attribution (see §8): the
ArtVIP dishwasher (Apache-2.0), YCB-scan-derived objects incl. the mug (YCB dataset terms),
and this project's own procedural props, racks and geometry caches. The robot USD is the one
exception by design — it is fetched from NVIDIA's public asset bucket at spawn time and never
redistributed. One command restores everything, no token needed:

```bash
scripts/run_py.sh scripts/tools/restore_assets.py --repo shu4dev/dishsim-assets --with_media
```

The restore downloads the archive (built props, every geometry cache — the ~1.5 h-of-Kit
part — derived dishwasher USDs, recorded results), re-downloads the ArtVIP originals,
validates every cache's `config_hash` against the current `config.py`, and runs the test
suite. `assets/`, `media/`, `results/` are gitignored; only curated figures under
`docs/figures/` are tracked.

**The archive is the fast path for BOTH machines.** Since the v2 release it also ships the
complete **Bosch 800 digital-twin world**: the self-authored machine USDs
(`assets/machines/bosch800/`), collision caches for all five Bosch rack states baked at the
measured winner mount (`assets/cache/machines/bosch800/`), the mount-sweep scorecards
(`results/base_sweep/bosch800/`), and the first recorded episodes. After a restore, a Bosch
episode runs immediately — no baking:

```bash
scripts/run_kit.sh scripts/experiment/run_task.py --headless --enable_cameras \
    --machine bosch800 --placement side_winner --scenario both_in \
    --spawn "cup=1,tumbler=1" --seed 1 --run_id bosch_repro
```

`--machine bosch800` switches the whole stack (machine USD, caches, cameras, scenarios) via
`config.apply_machine`; `--placement side_winner` is the A2-measured UR5e mount (side-elevated,
x +0.475, y −0.525, z 0.400, yaw +101.25°). The same two flags work on every setup script
(`build_state`, `extract_geometry`, `parity_check`, `goal_configs`).
Bosch numbers and their provenance: [docs/bosch800_source_data.md](docs/bosch800_source_data.md).

**One-command bring-up** — everything in §2.2–2.3 (image build, container start,
archive restore + cache validation) in one idempotent script:

```bash
scripts/tools/bootstrap.sh          # fresh clone -> experiments in ~5 minutes
```

The division of labor this enables is deliberate: everything expensive **runs once and ships
in the archive** — geometry extraction, CoACD decomposition, parity checks, goal funnels
(~2.5 h of Kit across both machines), the base-pose sweep, the capacity plan, the settle
probes. What a clone actually iterates on — **motion-planning experiments** (`run_trials.py`,
`run_task.py`, planner/seed/cost sweeps, `compute_metrics.py`) — bakes nothing and plans
per-call against the restored caches. If a run asks you to bake, either the archive is stale
for your config or you changed a hashed value (see §2.4); baking during an experiment sweep
is always a smell.

### 2.4 Rebaking after a config change

The shipped caches serve reproduction as-is. If you change any hashed config value (base
pose, rack parameters, grasp transform), the affected caches invalidate loudly and are
rebuilt with the pipeline (the runner prints the exact command on a cache miss):

```bash
./scripts/setup/build_state.py --state placement --classes mug,cup,tumbler,plate,bowl,fork
```

If you are rebuilding the world from nothing instead of restoring, first fetch the ArtVIP
source and derive the scene report:

```bash
scripts/run_kit.sh -c "from huggingface_hub import snapshot_download; \
  snapshot_download(repo_id='X-Humanoid/ArtVIP', repo_type='dataset', \
  allow_patterns=['Articulated_objects/major_appliances/dishwasher/**'], local_dir='assets/artvip')"
scripts/run_kit.sh scripts/setup/inspect_scene.py --headless --test_door
```

(The one-time object-library authoring scripts were retired with the public-asset release —
they live in git history; the archive ships their outputs.)

### 2.5 Verify the install

```bash
scripts/run_kit.sh scripts/setup/kit_smoke.py --headless --enable_cameras
scripts/run_py.sh -m pytest tests/
```

`kit_smoke.py` proves the planning stack imports *inside* the Kit process and that headless
camera capture produces non-black frames. The suite is **435 test cases across 25 files**, all
Kit-free.

> **Note:** `run_py.sh` sets `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` itself — hydra's pytest
> plugin breaks collection outside Kit.

## 3 Reproduce the results

The end-to-end path from a fresh setup to the Results table in §5. Kit-free scripts run
through `scripts/run_py.sh` (aliased `$PY` below).

```bash
# 1. install (§2.1–2.2), then restore the public archive (§2.3) — it ships every collision
#    cache prebuilt and validated, so there is nothing to bake for reproduction

# 2. the v0 single-object baseline: mug into the lower rack, both racks out.
#    v4-feasible mug slots are {0, 1, 5, 6, 7} — ids 1 and 7 marginally (64-sample funnels)
scripts/run_kit.sh scripts/experiment/run_trials.py --headless \
    --scenario both_out --object mug --slots 0,6 --seeds 0 --run_id repro_v0

# 3. a multi-object episode from the stowed machine (rack pull + picks + basket drops)
scripts/run_kit.sh scripts/experiment/run_task.py --headless --enable_cameras \
    --scenario both_in --spawn "cup=1,tumbler=1,fork=2" --seed 1 --run_id repro_episode

# 4. the capacity fill (the hero figure) and the metrics
scripts/run_kit.sh scripts/setup/capacity_fill.py --headless --enable_cameras
$PY scripts/evaluation/compute_metrics.py --all
```

Judge every Kit run from its log (`[RESULT] PASS`, no tracebacks) — exit codes lie. Step 3
should report `2/2 trials succeeded`; step 4's episode places the cup and tumbler and pulls
the rack to sub-millimetre error (the forks are an open item — see §6).

## 4 Running

Run from the repo root. Kit scripts go through `scripts/run_kit.sh`; Kit-free scripts use `$PY = scripts/run_py.sh`.

### 4.1 Phase 1 — Setup: build the world

```bash
# collision world for one machine state and carried object (build_state.py chains these)
scripts/run_kit.sh scripts/setup/extract_geometry.py --headless --scenario placement
$PY scripts/setup/decompose_meshes.py --scenario placement
scripts/run_kit.sh scripts/setup/parity_check.py --headless --scenario placement

# placement slots + IK goal sets that experiments plan to
scripts/run_kit.sh scripts/setup/goal_configs.py --headless --enable_cameras --object mug

# a fully-loaded machine: 29-item deterministic fill + rack-closability check
scripts/run_kit.sh scripts/setup/capacity_fill.py --headless --enable_cameras
```

- `--scenario`: machine state — `both_out`, `both_in`, `placement`, `placement_open`
- `--object`: carried object class (any key from the table in §1.3)
- `--force` (decompose): re-decompose even when cached pieces exist

> **Note:** `parity_check.py` is the gate that matters — it samples configurations and compares
> the Kit-free FCL world against Isaac's PhysX contacts. It currently agrees **100 %** over 200
> configurations per state. If you change rack geometry or an object, rerun
> `extract_geometry → decompose_meshes → parity_check` before trusting any planning result.

### 4.2 Phase 2 — Experiment: run an algorithm

```bash
# default planner, no cameras — the trajectory is recorded either way
scripts/run_kit.sh scripts/experiment/run_trials.py --headless \
    --scenario both_out --object mug --slots 0,6 --seeds 0-1

# choose and tune the algorithm; also record the search tree for the planning visual
scripts/run_kit.sh scripts/experiment/run_trials.py --headless --planner rrt_star \
    --planner_param range_rad=0.3 --planner_param budget_s=15 \
    --slots 6 --seeds 0 --run_id my_run --save_plan_debug
```

- `--planner`: `rrt_connect` (default) · `rrt_star` · `bit_star` · `prm`
- `--planner_param K=V`: repeatable override, applied on top of `config.PLANNER_PARAMS`.
  Any planner takes `budget_s`, `resolution_rad`, `simplify`; `range_rad` applies to
  `rrt_connect` and `rrt_star`, `goal_bias` to `rrt_star`, `samples_per_batch` to `bit_star`,
  `max_nearest_neighbors` to `prm`
- `--object` / `--scenario`: what to carry, and the initial machine state
- `--slots` / `--seeds`: comma lists or `a-b` ranges; `--repeats` trials per pair
- `--run_id`: names the run (default `<object>_<scenario>_<planner>_<UTC>`)
- `--save_plan_debug`: also write the planning query + search tree for `plan_visual.py`
- `--skip_existing`: resume, skipping trials whose JSON already exists
- `--enable_cameras`: *additionally* capture video inline (Phase 3 renders from the recording)

Each run writes one self-contained directory:

```
results/experiments/<run_id>/
  manifest.json              [run provenance: planner + params, object, scenario, config hash]
  trials/<trial>.json        [outcome, timings, placement error, failure stage]
  trajectories/<trial>.npz   [measured state per physics step — the Phase 3 input]
  plans/<trial>.npz          [the planning query + search tree, with --save_plan_debug]
results/experiments/LATEST   [the newest run id]
```

To compare algorithms, run the same slots and seeds under each and let Phase 3 build the table:

```bash
for p in rrt_connect rrt_star bit_star; do
  scripts/run_kit.sh scripts/experiment/run_trials.py --headless --planner $p \
      --slots 6 --seeds 0 --run_id "cmp_$p"
done
```

> **Note:** a trial's success verdict depends on live contact forces, so it is decided in
> Phase 2 and only *aggregated* in Phase 3. Placement geometry is recomputable from a recorded
> pose, and evaluation cross-checks it against the trial record.

**Multi-object episodes** (`run_task.py`) spawn N objects at seeded random countertop poses,
settle them, and clear them one at a time in an order the sequencer decides; an episode
starting from a stowed machine opens with a rack action. Full manual:
[docs/episodes.md](docs/episodes.md).

### 4.3 Phase 3 — Evaluation: read the artifacts

None of these re-plan or re-simulate; the first and last need no Kit at all.

```bash
# metrics + figures for the latest run; --all adds the planner-comparison table
$PY scripts/evaluation/compute_metrics.py --all

# prove a recording replays exactly (kinematics only, no cameras, seconds)
scripts/run_kit.sh scripts/evaluation/verify_replay.py --headless \
    --trial results/experiments/<run_id>/trajectories/trial_06_00_0.npz

# render video from recordings — one clip per camera, plus a final still
scripts/run_kit.sh scripts/evaluation/render_videos.py --headless --enable_cameras \
    --run_dir results/experiments/<run_id>/trajectories

# the planner's search tree, drawn from the recorded query
$PY scripts/evaluation/plan_visual.py --slot 6 --seed 0
```

| Tool | Reads | Writes | Needs Kit |
|---|---|---|---|
| `compute_metrics.py` | `trials/*.json` + `manifest.json` | `results/evaluation/<run_id>/metrics.json` + figures | no |
| `verify_replay.py` | `trajectories/*.npz` | pass/fail gate on link-pose error | yes (no cameras) |
| `render_videos.py` | `trajectories/*.npz` | `media/trials/<run_id>/*.mp4` + stills | yes + cameras |
| `plan_visual.py` | `plans/*.npz` | search-tree PNGs | no |

- `compute_metrics.py`: `--run <id>` a specific run · `--all` every run + comparison ·
  `--no_figures`
- `render_videos.py`: `--trial` one file or `--run_dir` a whole run · `--cameras front,iso,top`
  · `--stride N` for a fast preview · `--use_current_cameras` to override recorded poses
- `verify_replay.py`: `--tol_pos_m` (kinematic reconstruction) · `--tol_render_m` (render path)
- `plan_visual.py`: `--run <id>` / `--plan_debug <npz>` to pick the artifact · `--replan` to
  plan a fresh query instead of reading a recorded one

### 4.4 Archive / restore the generated artifacts

```bash
$PY scripts/tools/archive_assets.py --upload      # build tarballs, push to the private dataset
$PY scripts/tools/restore_assets.py --with_media  # download, extract, validate, run tests
```

## 5 Results

Every claim maps to a recorded run; artifacts live under the gitignored `results/` and
`media/` trees (restorable via §2.4 `--with_media`). All numbers measured on the RACK_GEN v4
rack at the front base placement.

| Claim | Run / artifact | Evidence |
|---|---|---|
| The **v0 single-object baseline reproduces on the public-asset mug**: 2/2 mug trials succeed (slots 0 and 6, lateral err 8.1 / 0.8 mm, tilt 0°, weld ≤ 0.3 mm, plans 1.1 / 2.4 s) | `repro_v0_085` | `results/experiments/repro_v0_085/`, `media/trials/` |
| **Reachability success bar met at the front base pose** — plate 2/3 gaps, bowl 3 cells, fork 3/3 bays, floor 5 cells, pick band 0.20 m: a 420-candidate base-pose sweep's winner matches front on every slot criterion and only deepens the pick band (0.40 m), so the front placement was kept | `results/base_sweep/` | `stage4_final.json`, `winner.json`, heatmaps in `media/base_sweep/`; table in [docs/success_criteria.md](docs/success_criteria.md) |
| **Capacity fill is closable**: 29 items planned, 27 settle stably, 0 displaced during the stow (the 2 parked are the wine-glass stemware stretch goal) | `results/fill/capacity.json` | `media/fill/` (timelapse, orbit, stills); mechanisms documented in `fill_plan.py` |
| **Stowed-machine episode**: the robot pulls the lower rack out (error < 1 µm) and places the cup and tumbler with genuine countertop picks (2/4 — the two forks fail on transit contact, an open item) | `bothin_085` | `results/experiments/bothin_085/episodes/ep001.json`, `media/task/bothin_085/ep001.mp4` |
| **First robot bowl placement** — weld-acquired, carried, released, verdict pass; needed `--planner_param budget_s=60` | `platebowl_v4_b60` | `results/experiments/platebowl_v4_b60/episodes/ep000.json`, `media/task/platebowl_v4_b60/ep000.mp4` |
| **Negative result:** plate placement is path-blocked — goal configs exist in both feasible gaps but RRT-Connect finds no path even at a 180 s budget | `plate_b180` | `results/experiments/plate_b180/episodes/ep000.json`; analysis in [docs/known_limitations.md](docs/known_limitations.md) |

## 6 Known limitations

The honest edges, each with measured evidence: plate placement is path-blocked (not
goal-blocked); fork-bay transit contacts past the weld-acquire hover gates; the default 20 s
plan budget predates the v4 rack (the bowl needed 60 s); the stemware lie-in never settles.
Details and next levers: [docs/known_limitations.md](docs/known_limitations.md).

## 7 Extending

### Add a motion planner

Nothing in `scripts/experiment/` changes. Write a module in `src/dishsim/planners/` — for
anything OMPL provides, subclass `OMPLPlanner` and override the one method that constructs the
algorithm:

```python
# src/dishsim/planners/rrt_connect.py — the worked example
from .. import config
from .ompl_base import OMPLPlanner

class RRTConnectPlanner(OMPLPlanner):
    name = "rrt_connect"

    def _make_planner(self, si):
        from ompl import geometric as og

        planner = og.RRTConnect(si)
        planner.setRange(float(self.params.get("range_rad", config.PLAN_RRT_RANGE_RAD)))
        return planner
```

Then add one line to `PLANNERS` in `dishsim/planners/__init__.py`. `--planner my_planner`
now works, and
`tests/test_planners.py` covers it automatically — every registered planner must solve a
stub-world query, respect the joint bounds, never return a colliding path, and describe itself
for the trial record. Parameters are applied **inside** each subclass because the OMPL
planners do not share a parameter interface; a planner that cannot accept a goal set declares
`supports_multi_goal = False` and the base class hands it the goal nearest the start. A
non-OMPL method (lattice A*, CHOMP) implements `Planner` directly — the only thing it needs
from the world is `in_collision(q) -> bool`.

Adding an **object class**, **placement mode**, or **machine state**:
[docs/extending.md](docs/extending.md).

## 8 Assets and licenses

This project builds on the following open-source projects and datasets. Please visit the URLs
for their respective licenses:

1. https://github.com/isaac-sim/IsaacLab — simulation framework (the 3.0 API this targets)
2. https://github.com/isaac-sim/IsaacSim — simulator, PhysX ground truth, and the
   pre-assembled UR5e + Robotiq 2F-85 from the Omniverse asset library (NVIDIA Omniverse
   asset EULA; fetched at spawn, derived copies stay local)
3. https://huggingface.co/datasets/X-Humanoid/ArtVIP — the articulated `dishwasher_2` asset
   (Apache-2.0)
4. https://www.ycbbenchmarks.com — YCB Object & Model Set, Calli et al., *"The YCB Object and
   Model Set"* (IEEE ICAR 2015): textured `google_16k` scans for plate, bowl, cups, cutlery
   and spatula, used under the YCB dataset terms
5. https://github.com/ompl/ompl — Șucan, Moll, Kavraki, *"The Open Motion Planning Library"*
   (IEEE RAM 2012): the RRT-Connect, RRT*, BIT* and PRM implementations behind
   `dishsim.planners`
6. https://github.com/BerkeleyAutomation/python-fcl — Pan, Chitta, Manocha, *"FCL: A general
   purpose library for collision and proximity queries"* (ICRA 2012): the Kit-free collision
   world
7. https://github.com/SarahWeiii/CoACD — Wei et al., *"Approximate Convex Decomposition for 3D
   Meshes with Collision-Aware Concavity and Tree Search"* (SIGGRAPH 2022)
8. https://github.com/mikedh/trimesh — mesh processing throughout the asset and collision
   pipelines
9. https://github.com/stack-of-tasks/pinocchio — Carpentier et al.: independent validation of
   the analytic UR5e kinematics in the test suite

Rack geometry is procedurally generated, styled after publicly documented Whirlpool, Bosch and
Frigidaire rack designs (design reference only; no third-party geometry is redistributed).

Downloaded and derived assets are never committed (`assets/`, `media/`, `results/` are
gitignored); the asset archive must remain private, as it contains NVIDIA-EULA- and
YCB-derived files.
