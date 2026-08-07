<div align="center">
  <h1 align="center"> dishwasher_sim_isaaclab </h1>
  <h3 align="center"> Isaac Sim environments for robotic dishwasher loading </h3>
  <p align="center">
    Classical motion planning · Imitation learning · Reinforcement learning
  </p>
  <p align="center">
    <img src="https://img.shields.io/badge/Isaac%20Sim-6.0.1-76B900?style=flat&logo=nvidia&logoColor=white" alt="Isaac Sim 6.0.1"/>
    <img src="https://img.shields.io/badge/Isaac%20Lab-3.0.0-76B900?style=flat&logo=nvidia&logoColor=white" alt="Isaac Lab 3.0.0"/>
    <img src="https://img.shields.io/badge/Python-3.12-3776AB?style=flat&logo=python&logoColor=white" alt="Python 3.12"/>
    <img src="https://img.shields.io/badge/License-BSD--3--Clause-blue?style=flat" alt="BSD-3-Clause"/>
  </p>
</div>

<table align="center">
  <tr>
    <td align="center">
      <img src="docs/figures/loaded_iso.png" width="720" alt="fully loaded dishwasher"/>
      <br/>
      <code>scripts/setup/capacity_fill.py</code> — 34 items placed, 31 settle stably, racks still close
    </td>
  </tr>
</table>

## Important Notes First

- **Every Kit script must run through `scripts/run_kit.sh`.** With the planning venv present,
  `isaaclab.sh -p` resolves to the bare venv interpreter, which lacks `EXP_PATH` /
  `LD_LIBRARY_PATH`, and `AppLauncher` dies at boot with `KeyError: 'EXP_PATH'`.
- **`./isaaclab.sh -p` exits 0 even when the wrapped script crashes.** Judge success from log
  content (`[RESULT] PASS`, absence of tracebacks), never from the exit code.
- **Do not up/downgrade Isaac Sim or Isaac Lab.** Everything is pinned to 6.0.1 + 3.0.0; the
  code targets the 3.0 API (XYZW quaternions, `ProxyArray.torch`, keyword-only `*_index`
  writes) and 2.x snippets are wrong on both counts.
- Everything runs `--headless`. Only *rendering* needs `--enable_cameras`; experiments do not.
- First run downloads ~85 MB of ArtVIP assets plus YCB scans, and rebuilding the geometry
  caches costs ~1.5 h — or restore a prebuilt archive in one command (§2.4).
- `assets/`, `media/`, `results/` are gitignored; only curated figures under `docs/figures/`
  are tracked. The asset archive must stay **private** (NVIDIA EULA + YCB dataset terms).
- Developed and tested on a single NVIDIA **L4** (23 GB) / 8 vCPU / 30 GiB. OMPL planning is
  CPU-bound, so core count matters more than the GPU except when rendering.

## 1 Introduction

This project is built on **Isaac Lab** to simulate a **UR5e + Robotiq 2F-85** loading an
articulated **ArtVIP dishwasher**, as a substrate for classical motion planning (shipped),
imitation learning, and reinforcement learning.

The scene: a hinged-door dishwasher whose two sliding racks are replaced with procedurally
generated, reference-styled wire racks plus a 3-bay cutlery basket; a robot on a pedestal; and
a 15-class kitchen-object library scaled to fit the compact machine, each class carrying
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
      <code>setup/build_object_assets.py</code>
      <br/>
      15-class object library
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
its own capabilities. Adding one touches no experiment code (§5.1).

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
across. Each `scale` below documents the factor against its source asset, and
`setup/build_object_assets.py` re-measures the result and refuses to write an asset that
disagrees with the registry.

| Class | Source | Scale | Placement mode | Rack |
|---|---|---|---|---|
| `mug` | Isaac YCB `025_mug` | 1.00 | `floor_stand` | lower |
| `plate` | YCB `029_plate` | 0.54 | `plate_slot` | lower |
| `saucer` | YCB `029_plate` | 0.42 | `plate_slot` | lower |
| `bowl` | YCB `024_bowl` | 0.68 | `bowl_lean` | lower |
| `cup` | YCB `065-a_cups` | 1.10 | `floor_stand` | lower |
| `fork` | YCB `030_fork` | 0.60 | `basket_drop` | basket |
| `spoon` | YCB `031_spoon` | 0.60 | `basket_drop` | basket |
| `knife` | YCB `032_knife` | 0.60 | `basket_drop` | basket |
| `spatula` | YCB `033_spatula` | 0.45 | `flat_lay` | upper |
| `pitcher` | YCB `019_pitcher_base` | 0.50 | `floor_stand` | lower |
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
| [docs/environment.md](docs/environment.md) | Hardware/software stack, venv recipe, Isaac Lab 3.0-vs-2.x API deltas, OMPL 2.0 nanobind notes, launcher landmines |
| [docs/success_criteria.md](docs/success_criteria.md) | Placement success definition per placement mode, slot model, frame conventions |
| [docs/joint_report.md](docs/joint_report.md) | *Auto-generated* by `setup/inspect_scene.py`: measured articulation numbers every constant derives from |
| [docs/grasp_calibration.md](docs/grasp_calibration.md) | *Auto-generated* by `setup/calibrate_grasp.py`: θ_touch/θ_grasp, per-pad force band, mimic signs |
| [docs/asset_survey.md](docs/asset_survey.md) | Survey of the 7 ArtVIP dishwasher variants justifying the `dishwasher_2` pick |

## 2 Environment Setup

### 2.1 Prerequisites

An Isaac Sim 6.0.1 / Isaac Lab 3.0.0 install at `/workspace/isaaclab`. This repo nests inside
that tree as an independent git repo. See the
[official installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html),
and [docs/environment.md](docs/environment.md) for this project's pinned versions.

### 2.2 Planning venv

Planning runs on CPU in a venv beside Kit (OMPL, FCL, CoACD are not part of the Kit
environment). Create it at **exactly** this path — `isaaclab.sh` looks for a venv there and
resolves Python to it, which is also why `run_kit.sh` has to re-export the Kit environment.

```bash
/isaac-sim/kit/python/bin/python3 -m venv --system-site-packages /workspace/isaaclab/env_isaaclab
/workspace/isaaclab/env_isaaclab/bin/pip install 'trimesh==4.12.2' ompl python-fcl coacd \
    matplotlib imageio pytest requests pyyaml filelock tqdm fsspec
/workspace/isaaclab/env_isaaclab/bin/pip install -e .
```

### 2.3 Assets

```bash
# dishwasher asset (~85 MB) + derived USDs + the measured joint report
scripts/run_kit.sh -c "from huggingface_hub import snapshot_download; \
  snapshot_download(repo_id='X-Humanoid/ArtVIP', repo_type='dataset', \
  allow_patterns=['Articulated_objects/major_appliances/dishwasher/**'], local_dir='assets/artvip')"
scripts/run_kit.sh scripts/setup/inspect_scene.py --headless --test_door

# object library: the mug from the Isaac asset bucket, the rest from YCB scans + procedural
scripts/run_kit.sh scripts/setup/make_prop_physics_usd.py --object 025_mug
scripts/run_kit.sh scripts/setup/build_object_assets.py
```

### 2.4 Fast path: restore a prebuilt archive

If you have access to the private asset archive (a Hugging Face dataset holding the built
props and every geometry cache), this replaces §2.3 and ~1.5 h of cache rebuilds:

```bash
# authenticate once with a token that can read the private dataset
/workspace/isaaclab/env_isaaclab/bin/python -m huggingface_hub.commands.huggingface_cli \
    login --token hf_xxx
/workspace/isaaclab/env_isaaclab/bin/python scripts/tools/restore_assets.py --with_media
```

> **Note:** the venv installs `huggingface_hub` as a library, not a console script, so
> `huggingface-cli` is not on `PATH` — invoke it as the module above. Pass `--token`
> explicitly; the interactive prompt raises `EOFError` without a TTY.

The restore re-downloads the ArtVIP originals, validates every restored cache's
`config_hash` against the current `config.py`, and runs the test suite.

### 2.5 Verify the install

```bash
scripts/run_kit.sh scripts/setup/kit_smoke.py --headless --enable_cameras
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest tests/
```

`kit_smoke.py` proves the planning stack imports *inside* the Kit process and that headless
camera capture produces non-black frames. The suite is **449 test cases across 26 files**, all
Kit-free.

> **Note:** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is required — the system site-packages carry
> hydra, whose pytest plugin imports `yaml`, a module that only exists inside Kit.

## 3 Running

Run from the repo root. Kit scripts go through `scripts/run_kit.sh`; venv scripts use
`/workspace/isaaclab/env_isaaclab/bin/python`, written `$PY` below.

### 3.1 Phase 1 — Setup: build the world

```bash
# collision world for one machine state and carried object
scripts/run_kit.sh scripts/setup/extract_geometry.py --headless --scenario placement
$PY scripts/setup/decompose_meshes.py --scenario placement
scripts/run_kit.sh scripts/setup/parity_check.py --headless --scenario placement

# placement slots + IK goal sets that experiments plan to
scripts/run_kit.sh scripts/setup/goal_configs.py --headless --enable_cameras --object mug

# a fully-loaded machine: 34-item deterministic fill + rack-closability check
scripts/run_kit.sh scripts/setup/capacity_fill.py --headless --enable_cameras
```

- `--scenario`: machine state — `both_out`, `both_in`, `placement`, `placement_open`
- `--object`: carried object class (any key from the table in §1.3)
- `--force` (decompose): re-decompose even when cached pieces exist

> **Note:** `parity_check.py` is the gate that matters — it samples configurations and compares
> the Kit-free FCL world against Isaac's PhysX contacts. It currently agrees **100 %** over 200
> configurations per state. If you change rack geometry or an object, rerun
> `extract_geometry → decompose_meshes → parity_check` before trusting any planning result.

### 3.2 Phase 2 — Experiment: run an algorithm

```bash
# default planner, no cameras — the trajectory is recorded either way
scripts/run_kit.sh scripts/experiment/run_trials.py --headless \
    --scenario both_out --object mug --slots 2,7 --seeds 0-1

# choose and tune the algorithm; also record the search tree for the planning visual
scripts/run_kit.sh scripts/experiment/run_trials.py --headless --planner rrt_star \
    --planner_param range_rad=0.3 --planner_param budget_s=15 \
    --slots 7 --seeds 0 --run_id my_run --save_plan_debug
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
- `--enable_cameras`: *additionally* capture video inline — only needed for the replay A/B
  check, since Phase 3 renders video from the recording

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
      --slots 7 --seeds 0 --run_id "cmp_$p"
done
```

> **Note:** a trial's success verdict depends on live contact forces, so it is decided in
> Phase 2 and only *aggregated* in Phase 3. Placement geometry is recomputable from a recorded
> pose, and evaluation cross-checks it against the trial record.

#### Multi-object episodes

`run_task.py` spawns N objects at seeded random countertop poses, settles them, and clears them
one at a time in an order it decides:

```bash
# reproducible from the seed alone
scripts/run_kit.sh scripts/experiment/run_task.py --headless --seed 0

# swap the ordering heuristic; the planner and the scene are untouched
scripts/run_kit.sh scripts/experiment/run_task.py --headless --seed 0 --cost_fn shortest_ik
```

- `--seed`: layout seed — with the composition this reproduces the scene exactly
- `--spawn "cup=2-4,mug=1"`: **explicit composition** — a count or an inclusive range per object
  type. A range is drawn per episode from the seed, so the number varies while staying
  reproducible. Term order does not matter (`a=1,b=2` == `b=2,a=1`), and the expanded list is
  shuffled so no type always gets first claim on countertop space. Config equivalent:
  `TASK["spawn_counts"]`
- `--n_objects` / `--classes`: the uniform-draw fallback when no explicit composition is given
- `--rack_lower_m` / `--rack_upper_m`: rack extensions in metres (0 = stowed, −0.20 = fully out;
  the racks are independently articulated). Resolved to a named machine state — extensions are
  part of the collision-cache hash, so a combination that has not been baked fails at startup
  with the command that bakes it. `--scenario NAME` selects one directly. Config equivalent:
  `TASK["rack_state"]`
- `--video_camera`: which camera the episode MP4 is written from (default `episode`)
- `--cost_fn`: pick-order heuristic — `nearest_first` (default) · `shortest_ik` · `farthest_first`
- `--allow_stacking`: spawn objects stacked on each other. Because settling then decides the
  final poses, reachability is re-checked *after* physics and the whole layout is resampled if
  any object ends up unreachable (capped by `TASK["max_layout_retries"]`, then a loud failure)
- `--planner` / `--planner_param`: as for `run_trials.py` — the task layer never names a planner

Per-pick records use the **existing trial schema**, so Phase 3 consumes an episode unchanged;
`episodes/<ep>.json` adds what a per-trial record cannot express (pick order, the cost each
choice scored, why anything was blocked, the support graph).

**Goal slots by object type.** `TASK["type_slots"]` maps an object type to an ORDERED list of
slot **names**, e.g. `{"mug": ("mid_centre", "near_centre")}`. Names are derived from the rack
geometry (`placement.slot_names`) rather than hardcoded ids, because ids are positional and would
silently re-point to different cells if the grid pitch were retuned. Each type consumes its list
in order; a slot already taken, overlapping an assigned one, or with no reachable goal
configurations for that class is skipped, and an object whose list runs out is reported unplaced.
The vocabulary follows each mode's actual grid — `near_centre`/`mid_left1` for the `floor_stand`
3×5, `gap_centre` for the plate tine gaps, `bay_near` for the cutlery bays.

**Starting from a stowed machine.** `--scenario both_in` begins with both racks pushed in, where
**no slot is reachable at all** (0 of 15, measured). The episode opens with a rack action — the
gripper engages the lower rack's handle and pulls it out to −0.20 m while the tool tracks the
moving handle — and then loads the machine in the resulting `placement` state. The episode spans
two collision-cache states; the post-action state is derived from the action, and a rack that
settles beyond `RACK_SLIDE_TOL_M` ends the episode rather than letting later picks plan against a
world the machine no longer matches.

```bash
scripts/run_kit.sh scripts/experiment/run_task.py --headless --enable_cameras \
    --scenario both_in --spawn "cup=1,tumbler=1,fork=2" --seed 1 --run_id bothin_load
```

> Measured: pulling the *upper* rack out as well (`placement_open`) does not open the rear rows —
> it drops cup, tumbler and fork to **0** reachable slots. The lower rack is already at its
> mechanical limit, and 87 mm of its 287 mm depth never leaves the machine mouth.

**Home-anchored trajectories.** Every episode's recording begins and ends at `config.HOME_Q`.
The start is measured, corrected if it drifted during layout settling, and then asserted; the
closing retreat plans back to home and holds for `SETTLE_STEPS`, and runs on the failure and
exception paths too. It happens *before* the recording and media are finalised (so the retreat is
in the `.npz`, the MP4 and the final stills) and *after* the sequencer (so placement verdicts are
already decided and parking the arm can never revise a pass/fail). The record carries
`start_home_err_rad`, `end_home_err_rad`, `home_return_status` and — because the fallback retreat
is a straight interpolation that is not collision-checked — `post_home_displacement_mm` for each
placed object.

**Camera framing.** `config.CAMERA_LENS` makes focal length and aperture configurable (they were
hardcoded), and `config.EPISODE_CAMERA` is a wide view that keeps the countertop and the machine
in frame for the whole episode — the previous `front` view sat 2.4× too close and cropped the
counter out entirely. Vertical FOV is the binding constraint on a 16:9 frame, not horizontal.

> **Note:** episode size is capped by DESTINATION capacity, not countertop room — the counter
> holds far more than the robot can put away. Measured ceiling, per rack structure:
>
> | destination | reachable | simultaneous | why |
> |---|---|---|---|
> | rack floor (`floor_stand`) | 4 of 15 slots | **2** | the 4 form a 2×2 block at 60 mm pitch; a cup needs 73.4 mm between centres, so only the 84.9 mm diagonals fit |
> | cutlery basket (`basket_drop`) | 2 of 3 bays | **2** | fork/knife/spoon — but weld-acquired, not picked |
> | plate tines, bowl slope | **0** | 0 | 87 mm of the rack's 287 mm depth never clears the mouth; extending the upper rack does not help, it makes every count 0 |
>
> So a full robot-loaded lower rack is **4 items** — 2 standing plus 2 in the basket — of which
> 2 are genuine picks. For contrast, `capacity_fill.py` *teleports* 34 items in and 31 settle
> stably: the rack's own capacity is far larger than what this arm can reach into it.
> See [docs/success_criteria.md](docs/success_criteria.md#multi-object-episodes-scriptsexperimentrun_taskpy).

### 3.3 Phase 3 — Evaluation: read the artifacts

None of these re-plan or re-simulate; the first and last need no Kit at all.

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

| Tool | Reads | Writes | Needs Kit |
|---|---|---|---|
| `compute_metrics.py` | `trials/*.json` + `manifest.json` | `results/evaluation/<run_id>/metrics.json` + figures | no |
| `verify_replay.py` | `trajectories/*.npz` | pass/fail gate on link-pose error | yes (no cameras) |
| `render_videos.py` | `trajectories/*.npz` | `media/trials/<run_id>/*.mp4` + stills | yes + cameras |
| `plan_visual.py` | `plans/*.npz` | search-tree PNGs | no |
| `compare_videos.py` | two MP4s | PSNR verdict + worst-frame sheet | no |

- `compute_metrics.py`: `--run <id>` a specific run · `--all` every run + comparison ·
  `--no_figures`
- `render_videos.py`: `--trial` one file or `--run_dir` a whole run · `--cameras front,iso,top`
  · `--stride N` for a fast preview · `--use_current_cameras` to override recorded poses
- `verify_replay.py`: `--tol_pos_m` (kinematic reconstruction) · `--tol_render_m` (render path)
- `plan_visual.py`: `--run <id>` / `--plan_debug <npz>` to pick the artifact · `--replan` to
  plan a fresh query instead of reading a recorded one

### 3.4 Archive / restore the generated artifacts

```bash
$PY scripts/tools/archive_assets.py --upload      # build tarballs, push to the private dataset
$PY scripts/tools/restore_assets.py --with_media  # download, extract, validate, run tests
```

## 4 Code Structure

```
dishwasher_sim_isaaclab/
│
├── scripts/
│   ├── run_kit.sh                    [Kit launcher: exports the Isaac env, then isaaclab.sh -p]
│   │
│   ├── setup/                        [PHASE 1 — assets and the simulation world]
│   │   ├── kit_smoke.py              [dependency + headless-capture gate]
│   │   ├── inspect_scene.py          [articulation survey -> docs/joint_report.md]
│   │   ├── prepare_dishwasher_usd.py [derived dishwasher USDs (door/rack variants)]
│   │   ├── make_prop_physics_usd.py  [Isaac-bucket prop -> physics USD (the mug)]
│   │   ├── build_object_assets.py    [YCB scans + procedural props -> the object library]
│   │   ├── check_scene.py            [scene verification; --measure derives the pad map]
│   │   ├── calibrate_grasp.py        [per-object pinch calibration (force staircase)]
│   │   ├── freeze_calibration.py     [freeze measured constants into config.OBJECTS]
│   │   ├── extract_geometry.py       [dump the settled scene into the collision cache]
│   │   ├── decompose_meshes.py       [convex FCL pieces (CoACD / analytic parts)]
│   │   ├── parity_check.py           [FCL vs PhysX agreement gate]
│   │   ├── goal_configs.py           [slot frames + IK goal sets]
│   │   ├── build_state.py            [bake one machine state's caches for N classes]
│   │   ├── reach_map.py              [measure where on the counter a class can be picked]
│   │   ├── preview_rack.py           [rack geometry preview PNGs]
│   │   └── capacity_fill.py          [fully-loaded scene generator + closability check]
│   │
│   ├── experiment/                   [PHASE 2 — run algorithms, write artifacts]
│   │   ├── run_trials.py             [ONE object: rack reconfigure -> pick -> plan -> place]
│   │   └── run_task.py               [N objects: spawn -> settle -> sequence -> clear]
│   │
│   ├── evaluation/                   [PHASE 3 — reads artifacts only]
│   │   ├── compute_metrics.py        [trial JSONs -> metrics.json + figures + comparison]
│   │   ├── render_videos.py          [trajectory .npz -> MP4s + stills (kinematic replay)]
│   │   ├── verify_replay.py          [replay faithfulness gate, camera-free]
│   │   ├── compare_videos.py         [live-vs-replay PSNR A/B + worst-frame sheet]
│   │   └── plan_visual.py            [planner search tree from the recorded query]
│   │
│   └── tools/
│       ├── archive_assets.py         [tar the generated artifacts, push to a private dataset]
│       └── restore_assets.py         [download, safe-extract, validate caches, run tests]
│
├── src/dishsim/                      [the environment package (installed editable)]
│   ├── config.py                     [EVERY tunable: object registry, grasps, rack params,
│   │                                  planner defaults, cameras, tolerances. Tune here]
│   ├── robots.py                     [UR5e + Robotiq and dishwasher ArticulationCfgs]
│   ├── scene.py                      [scene construction, the wrist weld, gripper control]
│   ├── usd_prep.py                   [derived dishwasher USDs; authors the procedural racks]
│   ├── rack_gen.py                   [procedural wire racks + cutlery basket (Kit-free)]
│   ├── prop_gen.py                   [procedural props: tumbler, wine glass, container, lid]
│   ├── geometry.py                   [USD -> mesh extraction + the collision-cache format]
│   ├── collision_world.py            [Kit-free FCL world; the planners' validity oracle]
│   ├── ur5e_kin.py                   [analytic UR5e FK/IK, 8 branches (Pinocchio-validated)]
│   ├── placement.py                  [slot derivation, goal poses and success per mode]
│   ├── rack_ops.py                   [rack-handle engage + drive-synchronized slide]
│   ├── fill_plan.py                  [deterministic full-load plan + FCL validation]
│   ├── trajectory.py                 [per-step recording format (Phase 2 -> Phase 3)]
│   ├── replay.py                     [kinematic playback of a recording (Phase 3)]
│   ├── plan_debug_io.py              [persist a planning query + search tree]
│   ├── metrics.py                    [Kit-free aggregation over trial records]
│   ├── media.py                      [camera rig, video writer, contact sheets]
│   ├── transforms.py                 [pose helpers (XYZW throughout)]
│   ├── task/                         [the task layer — decides WHAT, never HOW to move]
│   │   ├── sequencer.py              [which object next, in what order; support + grasp gates]
│   │   ├── primitives.py             [one object's pick-and-place choreography]
│   │   ├── motion.py                 [object-agnostic "move A to B" over the planner]
│   │   ├── layout.py                 [seeded random countertop layouts, with stacking]
│   │   ├── support.py                [which object rests on which (contact + geometric)]
│   │   ├── grasp.py                  [state-dependent grasp availability + yaw sweep]
│   │   ├── recovery.py               [bounded recovery ladder (a registry)]
│   │   ├── rack.py                   [open the machine: engage a handle, slide a rack]
│   │   ├── cost.py                   [swappable pick-order heuristics (a registry)]
│   │   └── episode.py                [episode record + aggregation]
│   └── planners/                     [the pluggable planner layer]
│       ├── base.py                   [PlanResult, PlanDebug, the Planner ABC]
│       ├── ompl_base.py              [shared OMPL query: space, validity, goals, solve]
│       ├── rrt_connect.py            [bidirectional RRT (default)]
│       ├── rrt_star.py               [asymptotically optimal RRT]
│       ├── bit_star.py               [Batch Informed Trees]
│       ├── prm.py                    [probabilistic roadmap (single-goal here)]
│       └── registry.py               [name -> class; make_planner(); available()]
│
├── tests/                            [449 cases across 26 files; venv pytest, no Kit]
├── docs/                             [environment, success criteria, measured reports]
├── assets/  media/  results/         [generated, gitignored]
└── pyproject.toml
```

## 5 Extending

### 5.1 Add a motion planner

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

Then add one line to `PLANNERS` in `registry.py`:

```python
PLANNERS: dict[str, type[Planner]] = {
    RRTConnectPlanner.name: RRTConnectPlanner,
    ...
    MyPlanner.name: MyPlanner,
}
```

`--planner my_planner` now works, and `tests/test_planners.py` covers it automatically — every
registered planner must solve a stub-world query, respect the joint bounds, never return a
colliding path, and describe itself for the trial record.

Parameters are applied **inside** each subclass rather than by a generic loop, because the OMPL
planners do not share a parameter interface (`RRTConnect` has `setRange` only; `RRTstar` adds
`setGoalBias`; `PRM` has neither). A planner that cannot accept a goal set declares
`supports_multi_goal = False` and the base class hands it the goal nearest the start. A
non-OMPL method (lattice A*, CHOMP) implements `Planner` directly — the only thing it needs
from the world is `in_collision(q) -> bool`.

### 5.2 Add an object class

1. Add an `ObjectSpec` to `config.OBJECTS` — source (`ycb16k` / `procedural`), scale, mass,
   a `GraspSpec` (family + contact width) and a `PlacementSpec` (mode + rack).
2. Build the asset: `scripts/run_kit.sh scripts/setup/build_object_assets.py --objects <name>`.
   It prints the *measured* dimensions and fails if they disagree with the registry by >2 mm —
   freeze the printed block into the spec.
3. Measure the pinch: `setup/check_scene.py --measure` for the pad map, then
   `setup/calibrate_grasp.py --object <name>`.
4. Freeze the result: `setup/freeze_calibration.py --object <name>`.
5. Rebuild that object's caches: `extract_geometry` → `decompose_meshes` → `goal_configs`.

> **Note:** never eyeball-edit a measured value. Every dimension, aperture and force band in
> `config.py` traces to a calibration or inspection run, and `geometry.config_hash()`
> invalidates the collision caches when any of them changes.

### 5.3 Add a placement mode

In `src/dishsim/placement.py`: write a `derive_<mode>_slots()` returning `SlotFrame`s, add a
branch to `object_pose_for_mode()` for the goal-pose geometry, and a branch to
`evaluate_placement()` for the success criteria. Register the mode name in the object's
`PlacementSpec`, and document the criteria in `docs/success_criteria.md`.

### 5.4 Add a machine state

Add an entry to `config.INTERNAL_STATES` (rack extensions + `min_feasible_slots`), then rebuild
the cache for it: `extract_geometry --scenario <name>` → `decompose_meshes --scenario <name>`
→ `parity_check --scenario <name>` → `goal_configs --scenario <name>`.


## Acknowledgement

This project builds on the following open-source projects and datasets. Please visit the URLs
for their respective licenses:

1. https://github.com/isaac-sim/IsaacLab — simulation framework (the 3.0 API this targets)
2. https://github.com/isaac-sim/IsaacSim — simulator, PhysX ground truth, and the
   pre-assembled UR5e + Robotiq 2F-85 from the Omniverse asset library (NVIDIA Omniverse
   asset EULA; fetched at spawn, derived copies stay local)
3. https://huggingface.co/datasets/X-Humanoid/ArtVIP — the articulated `dishwasher_2` asset
   (Apache-2.0)
4. https://www.ycbbenchmarks.com — YCB Object & Model Set, Calli et al., *"The YCB Object and
   Model Set"* (IEEE ICAR 2015): textured `google_16k` scans for plate, bowl, cups, cutlery,
   spatula and pitcher, used under the YCB dataset terms
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
