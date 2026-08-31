<div align="center">
  <h1 align="center"> dishwasher_sim_isaaclab </h1>
  <h3 align="center"> A physics-validated rearrangement planning benchmark (Isaac Sim) </h3>
  <p align="center">
    Saved problem instances · Closed-loop algorithms · Physics settles every move
  </p>
  <p align="center">
    <img src="https://img.shields.io/badge/Isaac%20Sim-4.5.0-76B900?style=flat&logo=nvidia&logoColor=white" alt="Isaac Sim 4.5.0"/>
    <img src="https://img.shields.io/badge/Isaac%20Lab-2.1.1-76B900?style=flat&logo=nvidia&logoColor=white" alt="Isaac Lab 2.1.1"/>
    <img src="https://img.shields.io/badge/Python-3.10-3776AB?style=flat&logo=python&logoColor=white" alt="Python 3.10"/>
    <img src="https://img.shields.io/badge/License-BSD--3--Clause-blue?style=flat" alt="BSD-3-Clause"/>
  </p>
</div>

<table align="center">
  <tr>
    <td align="center">
      <img src="docs/figures/instance_goal_iso.png" width="720" alt="a benchmark instance's goal arrangement, physically settled"/>
      <br/>
      a benchmark instance's GOAL arrangement (15 items, per-item tinted), physically settled —
      <code>evaluation/instance_views.py</code>
    </td>
  </tr>
</table>

> **Read this first.** Every Kit script runs through `scripts/run_kit.sh` and every Kit-free
> one through `scripts/run_py.sh` (both forward into the `dishsim-isaac` container from the
> host), success is judged from **log content**, never exit codes (`isaaclab.sh -p` exits 0
> on crashes), and the Isaac Sim 4.5.0 / Isaac Lab v2.1.1 pins must not be changed. The full
> list of launcher landmines and why they exist: [docs/environment.md](docs/environment.md).

## 1 Overview

This repo is a **benchmark for dishwasher rearrangement planning**: given a physically
settled initial arrangement and an exact target arrangement in an articulated dishwasher
(experiments run on a self-authored **Bosch 800 digital twin** with a third rack; the ArtVIP
baseline machine ships too), an algorithm moves one object at a time by **teleport** and
**physics judges every move**. The 14-class kitchen-object library is scaled to the machines.
A placement is feasible when it is

- **collision-free** — a Kit-free FCL world (`dishsim.collision_world`) answers "would this
  object, teleported to this pose, interpenetrate the machine or an already-placed object?"
  in milliseconds, in any plain Python process; and
- **physically stable** — Isaac Sim settles the planned arrangement and judges it against
  per-mode measured tolerances (drift, tilt, seating height, rack closability).

There is no robot arm and no motion planning on this branch — Isaac Sim's only jobs are
physics validation and evidence rendering. (The earlier UR5e + OMPL manipulation stack lives
in git history; the RL door-opening pipeline on `archive/rl-door-opening`.)

The pipeline, mirrored by the layout of `scripts/`:

| Stage | Command | Does | Writes |
|---|---|---|---|
| **Plan** | in-process (`capacity.plan_full_load`) | Kit-free greedy capacity plan: derive slots live, pre-scan placeability, certify the load jointly, gate on z-budget + measured settle reliability | (consumed live by Generate) |
| **Generate** | `setup/gen_instances.py` | Seeded rearrangement instances (perturbed plans / random drops), physically settled and saved as artifacts | `results/instances/<machine>/<state>/` |
| **Problem images** | `evaluation/instance_views.py` | One instance's initial-vs-goal stills — the problem, where the episode video is the solving | `media/instances/<machine>/<state>/` |
| **Benchmark** | `experiment/run_rearrange.py` | Closed-loop algorithm episodes: every move teleports + settles; abort on first fault; move budget; `--video` per-episode MP4 | `results/rearrange/<machine>/<state>/`, `media/rearrange/` |

<table align="center">
  <tr>
    <td align="center">
      <img src="docs/figures/slot_detection.png" width="300" alt="slot derivation"/>
      <br/>
      Slot derivation from rack geometry (retired producer, git history)
    </td>
    <td align="center">
      <img src="docs/figures/bosch800_loaded_reveal.png" width="460" alt="Bosch 800 loaded reveal"/>
      <br/>
      A planned Bosch 800 load, physically settled, max drift 1.1 mm (retired producer, git history)
    </td>
  </tr>
</table>

### 1.1 The benchmark

- **Instance** (saved JSON, seeded — every algorithm sees byte-identical inputs): a machine
  rack state, a roster of objects at **measured settled** initial poses, and an exact target
  pose per object (the capacity plan's jointly-certified release pose, carrying its slot so
  the at-goal verdict reuses the per-mode settle tolerances).
- **Move**: `move(object, pose)` — teleport anywhere, a rack slot or the counter buffer band
  (just physical space; its finite capacity emerges from geometry).
- **Episode** (closed-loop): the harness FCL-pre-checks each commanded pose, executes it,
  settles physics, and hands the algorithm the measured state via `next_move(obs)`. The
  episode **aborts on the first fault** — unstable settle or a disturbed neighbor — or at the
  move budget (3× roster by default) or the optional planning-time budget. An **infeasible
  commanded pose is refused and counted** (`infeasible_commands`), not fatal: every algorithm
  can pre-check with the same oracle, so emitting one is a search error worth measuring rather
  than a reason to end the episode — ending it there would flatter pre-checking planners.
- **Scoring** per episode record: solved, fraction-at-goal, moves used, buffer-vs-goal move
  split, travel distance, planning time (per call and total), feasibility queries, seed, and
  whatever an algorithm reports through its optional `stats()` hook.
- **Ground truth**: `dishsim/compat.py` computes the **provably minimum** move count for an
  instance, so results can be quoted as an optimality gap rather than a relative ranking.
  Feasibility is pairwise-decomposable here, so a compatibility table (seconds) makes an exact
  A* cheap. Instances are per-machine artifacts (settle fixed points are engine-relative); on
  this box's three shipped instances the optima are **9, 8, 8** and greedy solves 3/3 in 9
  moves. It is a *geometric-relaxation* optimum — see the caveats in the module docstring.
- **Your algorithm**: one class implementing `reset(instance, world)` / `next_move(obs)`
  (`src/dishsim/rearrange.py`) plus one line in `ALGORITHMS` in
  `scripts/experiment/run_rearrange.py`; accept a `seed=` kwarg if stochastic. A greedy
  baseline ships as the thing to beat — one-blocker lookahead, so swap-cycles defeat it.

### 1.3 Object library

14 classes, sourced from YCB scans or generated procedurally, then **scaled to fit** the
machines; the certified Bosch load uses plates, bowls and forks (drinkware sits out on a
measured settle-reliability gate). The full registry lives in `config.OBJECTS`; adding a
class: [docs/extending.md](docs/extending.md).

### 1.4 Reference documentation

| Doc | Contents |
|---|---|
| [docs/environment.md](docs/environment.md) | Hardware/software stack, container recipe, Isaac Lab 2.1 API notes, **launcher landmines (canonical)** |
| [docs/architecture.md](docs/architecture.md) | Code structure, the Kit-free/Kit-side layering, completed one-off studies |
| [docs/success_criteria.md](docs/success_criteria.md) | Slot model per placement mode, settle tolerances, placeable capacity |
| [docs/known_limitations.md](docs/known_limitations.md) | Honest negative results and open items, with measured evidence |
| [docs/extending.md](docs/extending.md) | Add an object class / placement mode / machine state |
| [docs/joint_report.md](docs/joint_report.md) | Measured articulation numbers every constant derives from (generated by the retired inspect_scene.py, git history) |
| [docs/bosch800_source_data.md](docs/bosch800_source_data.md) | Every Bosch 800 twin number with its provenance |
| [docs/figures/README.md](docs/figures/README.md) | Provenance of every tracked figure (producing command + media source) |

## 2 Environment Setup

### 2.1 Prerequisites

Docker with the NVIDIA container runtime, and an NVIDIA GPU with a 535-series (or newer)
driver — the runtime environment (Isaac Sim **4.5.0** + Isaac Lab **v2.1.1**) is fully baked
into the image built by `docker/Dockerfile`, and nothing installs on the host. Developed and
validated on the corallab workstation (3× RTX 3090, driver 535.230.02, Ubuntu 20.04 — see
[docs/environment.md](docs/environment.md)). Everything runs `--headless`; only *rendering*
additionally needs `--enable_cameras`.

### 2.2 Runtime container

```bash
docker build -f docker/Dockerfile -t dishsim-isaac:4.5.0 .   # once (skipped if present)
docker compose -f docker/compose.yaml up -d                  # long-lived container dishsim-isaac
```

The compose file keeps every bulky mutable path (assets, media, results, Kit caches,
`HF_HOME`) on `/media/corallab-s1/2tbhdd/brianshu/dishsim`; the repo's data dirs are symlinks
there, and the container is the only root-disk artifact. Pick the least-loaded GPU per shell
(`nvidia-smi`, then `DISHSIM_GPU=<n> docker compose ... up -d` — shared machine).
`requirements-planning.txt` pins the measured working set (the table in
[docs/environment.md](docs/environment.md) is the measurement of record); the Dockerfile
installs it plus pytest and the archive tooling into Kit's python — no venv.

### 2.3 Assets (public archive — the one-command path)

Every asset this project uses is publicly redistributable with attribution (see §7): the
ArtVIP dishwasher (Apache-2.0), YCB-scan-derived objects incl. the mug (YCB dataset terms),
and this project's own procedural props, racks and geometry caches. One command restores
everything, no token needed:

```bash
scripts/run_py.sh scripts/tools/restore_assets.py --repo shu4dev/dishsim-assets --with_media
```

The restore downloads the archive (built props, every geometry cache — the ~1.5 h-of-Kit
part — derived dishwasher USDs, recorded results), re-downloads the ArtVIP originals,
validates every cache's `config_hash` against the current `config.py`, and runs the test
suite. `assets/`, `media/`, `results/` are gitignored; only curated figures under
`docs/figures/` are tracked.

**The archive ships the complete Bosch 800 digital-twin world**: collision caches for all
five Bosch rack states baked at the measured `side_winner` anchor
(`assets/cache/machines/bosch800/`); the machine USDs re-author on demand at import. One
post-restore step: the shipped `E_door_4` CoACD pieces predate a static-CoACD param change,
so run `decompose_meshes.py` once per cached context (quickstart step 2; the loud
`missing CoACD pieces` load error names the fix). `--machine bosch800` switches the whole
stack via `config.apply_machine`; `--placement side_winner` selects the frozen base-frame
anchor the Bosch caches are expressed in. Bosch numbers and their provenance:
[docs/bosch800_source_data.md](docs/bosch800_source_data.md).

**One-command bring-up** — everything in §2.2–2.3 (image build if absent, container start,
archive restore + cache validation) in one idempotent script:

```bash
scripts/tools/bootstrap.sh          # fresh clone -> planning in ~5 minutes
```

The division of labor is deliberate: everything expensive **runs once and ships in the
archive** — geometry extraction and CoACD decomposition (~1.5 h of Kit across both machines).
What a clone actually iterates on — instances and algorithms — plans per-call against the
restored caches. If a run asks you to bake, either the archive is stale for your config or
you changed a hashed value (see §4); baking during a benchmark sweep is always a smell.

### 2.4 Rebaking after a config change

The shipped caches serve reproduction as-is; a hashed-config change invalidates loudly and
rebuilds with the two-stage `extract_geometry` → `decompose_meshes` pair (§4). If rebuilding
the world from nothing instead of restoring, first fetch the ArtVIP source:

```bash
scripts/run_py.sh -c "from huggingface_hub import snapshot_download; \
  snapshot_download(repo_id='X-Humanoid/ArtVIP', repo_type='dataset', \
  allow_patterns=['Articulated_objects/major_appliances/dishwasher/**'], local_dir='assets/artvip')"
```

(The one-time authoring/inspection scripts live in git history; the archive ships their
outputs and `docs/joint_report.md` records the measured numbers.)

### 2.5 Verify the install

```bash
scripts/run_kit.sh scripts/setup/kit_smoke.py --headless --enable_cameras
scripts/run_py.sh -m pytest tests/
```

`kit_smoke.py` proves the collision stack imports *inside* the Kit process and that headless
camera capture produces non-black frames (bootstrap runs it automatically). The suite is
deliberately minimal — **4 Kit-free files**: the two frozen-invariant pins (the tripwires that
protect the shipped caches) and the benchmark driver's toy-oracle check.

> **Note:** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is required outside Kit — the site-packages
> carry hydra, whose pytest plugin breaks collection there. `scripts/run_py.sh` bakes it in.

## 3 Quickstart — reproduce the results

The end-to-end path from a fresh clone to the Results table in §5. Judge every Kit run from
its log (`[RESULT] PASS`, no tracebacks) — exit codes lie (`isaaclab.sh -p` exits 0 on
crashes).

```bash
# 0. shared box: pick the least-loaded GPU
nvidia-smi
DISHSIM_GPU=<n> docker compose -f docker/compose.yaml up -d

# 1. bring-up: image build if absent + container + archive restore + the kit_smoke gate
scripts/tools/bootstrap.sh
#    GATE: restore prints "[OK] ... @ <placement>" per cache, then kit_smoke "[RESULT] PASS"

# 2. once per restored context: re-decompose (shipped E_door_4 pieces predate a param change).
#    The anchor MUST match the restore log's "@ side_winner" — a wrong anchor reads as
#    "cache is stale".
scripts/run_py.sh scripts/setup/decompose_meshes.py \
    --machine bosch800 --placement side_winner --scenario placement --object plate
scripts/run_py.sh scripts/setup/decompose_meshes.py \
    --machine bosch800 --placement side_winner --scenario placement --object bowl

# 3. gates: tests + Kit-free capacity sanity
scripts/run_py.sh -m pytest tests/          # GATE: 11 passed
scripts/run_py.sh -c "
import sys; sys.path.insert(0, 'src')
from dishsim import config
config.apply_machine('bosch800'); config.apply_base_placement('side_winner')
from dishsim import capacity
print('total', capacity.plan_full_load(log=lambda *_: None).total_items)"
#    GATE: total 39 (placement 15 = plate 7 + bowl 8)

# 4. generate settled instances (saved artifacts — every algorithm sees identical inputs)
scripts/run_kit.sh scripts/setup/gen_instances.py --headless \
    --mode perturbed --state placement --n 3 --seed 0

# 5. the PROBLEM: one instance's initial + goal stills
scripts/run_kit.sh scripts/evaluation/instance_views.py --headless --enable_cameras \
    --instance results/instances/bosch800/placement/perturbed_s0.json

# 6. the SOLVING: closed-loop episodes with per-episode MP4s
scripts/run_kit.sh scripts/experiment/run_rearrange.py --headless --enable_cameras --video \
    --instances "results/instances/bosch800/placement/*.json" --algorithms greedy
#    GATE: "[RESULT] PASS"; expect 3/3 solved in 9 moves of 45, 0 aborts, 0 infeasible commands
```

Re-rolls during step 4 are healthy (the reproduction gate re-settling an arrangement);
init-mismatch storms are not. Fault/reset thresholds live as module constants in
`src/dishsim/rearrange.py` (deliberately outside `config.py`, so they can never touch
`config_hash`); widen only against a measurement.

## 4 Notes for running and extending

Every multi-object render tints objects **per item** (`config.item_color`): the sourced props
share one dark-red material, so an untinted 15-item load is unreadable. A colour follows the
item id, so the same object keeps it across the initial still, the goal still and the episode
video. Tinting is visual only and never touches physics or collision geometry.

An algorithm implements `reset(instance, world)` / `next_move(obs) -> Move | None`
(`src/dishsim/rearrange.py`; register it in `ALGORITHMS` in `run_rearrange.py`; accept a
`seed=` kwarg if stochastic). Every move teleports one object, settles `SETTLE_STEPS_MOVE`
physics steps, and the episode ABORTS on the first fault — unstable settle or a disturbed
neighbor — or at the move budget. An infeasible commanded pose is refused and counted
(`infeasible_commands`), not fatal. The greedy baseline is the thing to beat — one-blocker
lookahead, so swap-cycles defeat it.

**Rebaking after a hashed-config change** (rack params, machine geometry, an object spec — or
a FROZEN CACHE ANCHOR, which you must not touch): the affected caches invalidate loudly and
rebuild with the two-stage pair, per (object, state):

```bash
scripts/run_kit.sh scripts/setup/extract_geometry.py --headless \
    --machine bosch800 --placement side_winner --scenario <state> --object <class>
scripts/run_py.sh scripts/setup/decompose_meshes.py \
    --machine bosch800 --placement side_winner --scenario <state> --object <class>
```

Restore the public archive any time with
`scripts/run_py.sh scripts/tools/restore_assets.py --repo shu4dev/dishsim-assets`
(the producer side, archive_assets.py, lives in git history).

## 5 Results

Every claim maps to a recorded artifact; artifacts live under the gitignored `results/` and
`media/` trees.

| Claim | Run / artifact | Evidence |
|---|---|---|
| **The benchmark runs closed-loop on a 15-item roster** (corallab, 2026-08-30): greedy solves 3/3 perturbed instances (15/15 at goal in 9 moves of a 45 budget) with **0 harness aborts, 0 infeasible commands**, against provable optima 9/8/8 (`compat.optimal_moves`) | `results/rearrange/bosch800/placement/` | `media/rearrange/bosch800/placement/` (episode MP4s), `media/instances/bosch800/placement/` (initial vs goal) |
| **Bosch 800 full load settles**: a planned multi-rack load teleported to its release poses settles with max drift 1.1 mm — the plan's poses are physically self-consistent | episode-era artifact of record (robot-era media, retired; HF archive + git history) | `docs/figures/bosch800_loaded_reveal.png` |
| **Measured settle-reliability gates**: bowls 59/60 upright on the Bosch lower rack; scaled cups 49/82 and tumblers 64/88 wedge into the OEM wire lattice — which is why drinkware sits out of the certified Bosch count | probe campaign of record (archived: HF tarball `results/plate_settle/`); gates frozen in `capacity.MEASURED_SETTLE_RELIABILITY` | [docs/known_limitations.md](docs/known_limitations.md) |

## 6 Known limitations

The honest edges, each with measured evidence: scaled drinkware does not stand reliably on
the Bosch OEM wire lattice; the loaded lower rack cannot be driven back over the door sill;
the stemware lie-in never settles. Added 2026-08-28: `config_hash` is **not** the only cache
key (a static-CoACD change is invisible to the staleness check, so a restored box must re-run
`decompose_meshes.py` once); CoACD's manifold preprocess inflates authored bodies by millimetres
of phantom volume; only the `placement` rack state is Kit-validated; the move model has **no
insertion-path gate**, so blocking and non-monotonicity are unrepresentable; the stowed lower
rack interpenetrates the tub; and capacity is capped by the half-scale dish library and a
single-rank plate bank rather than by the machine. Details and next levers:
[docs/known_limitations.md](docs/known_limitations.md).

Adding an **object class**, **placement mode**, or **machine state**:
[docs/extending.md](docs/extending.md).

## 7 Assets and licenses

This project builds on the following open-source projects and datasets. Please visit the URLs
for their respective licenses:

1. https://github.com/isaac-sim/IsaacLab — simulation framework (the v2.1.1 API this targets)
2. https://github.com/isaac-sim/IsaacSim — simulator and PhysX ground truth
3. https://huggingface.co/datasets/X-Humanoid/ArtVIP — the articulated `dishwasher_2` asset
   (Apache-2.0)
4. https://www.ycbbenchmarks.com — YCB Object & Model Set, Calli et al., *"The YCB Object and
   Model Set"* (IEEE ICAR 2015): textured `google_16k` scans for plate, bowl, cups, cutlery
   and spatula, used under the YCB dataset terms
5. https://github.com/BerkeleyAutomation/python-fcl — Pan, Chitta, Manocha, *"FCL: A general
   purpose library for collision and proximity queries"* (ICRA 2012): the Kit-free collision
   world
6. https://github.com/SarahWeiii/CoACD — Wei et al., *"Approximate Convex Decomposition for 3D
   Meshes with Collision-Aware Concavity and Tree Search"* (SIGGRAPH 2022)
7. https://github.com/mikedh/trimesh — mesh processing throughout the asset and collision
   pipelines

Rack geometry is procedurally generated, styled after publicly documented Whirlpool, Bosch and
Frigidaire rack designs (design reference only; no third-party geometry is redistributed).

Downloaded and derived assets are never committed (`assets/`, `media/`, `results/` are
gitignored); the public archive redistributes only what its sources' licenses allow, with
attribution.
