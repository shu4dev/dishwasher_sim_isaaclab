<div align="center">
  <h1 align="center"> dishwasher_sim_isaaclab </h1>
  <h3 align="center"> A physics-validated rearrangement benchmark (Isaac Sim) </h3>
  <p align="center">
    <img src="https://img.shields.io/badge/Isaac%20Sim-6.2.0-76B900?style=flat&logo=nvidia&logoColor=white" alt="Isaac Sim 6.2.0"/>
    <img src="https://img.shields.io/badge/Isaac%20Lab-3.0.0-76B900?style=flat&logo=nvidia&logoColor=white" alt="Isaac Lab 3.0.0"/>
    <img src="https://img.shields.io/badge/Python-3.12-3776AB?style=flat&logo=python&logoColor=white" alt="Python 3.12"/>
    <img src="https://img.shields.io/badge/License-BSD--3--Clause-blue?style=flat" alt="BSD-3-Clause"/>
  </p>
</div>

Run this first, about 5 minutes from a fresh clone:

```bash
hf download shu4dev/dishsim-assets --repo-type dataset --local-dir data/assets
code/util/tools/bootstrap.sh
```

The first line downloads the public asset dataset into `data/assets`; `bootstrap.sh`
builds the `dishsim-isaac` image if absent, starts the container and runs the Kit smoke test.
Pass = `[RESULT] PASS` at the end.

## What this is

An algorithm moves one dish at a time by teleport inside a digital twin of the Frigidaire FDPC4221AS
dishwasher, loaded with the HOTEC wheat-straw set (plates, bowls, cups), and Isaac settles every move. A
benchmark sample is a messy start (dishes stacked on the counter and dropped into the racks) and a goal: the
score-greedy load, each dish in a random order at the free site and tilt with the highest gain of the fair
exposure score S, built in Isaac one dish at a time and gated. The goal and its S_ref are the ground truth.
Track A (greedy_offline, rrt_connect) must reach the goal: solved within a 60 s planning budget, raw moves.
Track B (first_fit, move-level MCTS) builds its own load: S / S_ref. There is no robot arm in the benchmark;
a UR5e + Robotiq 2F-85 arm was studied separately, a hardware-limits study closed on 2026-10-30
([code/frigidaire/docs/robot.md](code/frigidaire/docs/robot.md)).

Rosters come from explicit flags; each sample carries three level labels: kinds 1 / 2 / all, count 1-8 / 9-16 /
17-24 and counter allowance n+3 / n+2 / n+1, each easy / medium / hard.

## Run the benchmark

Every Kit command goes through `code/util/run_kit.sh`, every Kit-free one through
`code/util/run_py.sh`; both forward into the container. Judge each Kit run by its log line
`[RESULT] PASS` and the absence of tracebacks. Exit codes lie.

1. Assets and bring-up: the dataset download and `code/util/tools/bootstrap.sh` above (5 min).
2. Tests: `code/util/run_py.sh -m pytest code/tests/ code/frigidaire/tests/ -q`.
3. The empty-machine baseline (Kit, 1 min):
   `code/util/run_kit.sh code/initialization/frigidaire/frigidaire_initial_state_validate.py --headless --out-dir data/results/benchmark/frigidaire_hotec/baseline`.
4. The fair score's ceilings on this stack: the two grid commands in [CLAUDE.md](CLAUDE.md#commands), then
   `PYTHONPATH=code/src python3 -m bench.score --write-ceilings`.
5. One roster, every stage (inside the container; about an hour on one L4 core pair):
   ```bash
   code/util/run_py.sh code/planner/bench.py --kinds bowl --count bowl=7 --counter-allowance 3 --seed 0 --to report \
       --algorithms greedy_offline,rrt_connect,first_fit,mcts --out data/results/bench/v1
   ```
   Samples land in `data/results/bench/v1/` (episodes in its `runs/`), stills, videos and the page (`index.html`) in `data/media/bench/v1/`.

## Add your algorithm

1. Implement `reset(instance, world)` and `next_move(obs)` in one class (`code/src/dishsim/rearrange.py` shows the greedy baseline).
2. Add it to `EPISODES` in `code/planner/frigidaire/frigidaire_bench_kit.py` (and to `ALGORITHMS` in `code/src/bench/sample.py`). Accept `seed=` if stochastic.
3. Rerun the run and report stages on a stored sample: `code/util/run_py.sh code/planner/bench.py --from run --to report --input <sample.json> --algorithms <name>`.

## Results on record

The first run of the merged benchmark (three rosters, seed 0, Isaac Sim 6.2.0): [results/2026-11-01-merged-benchmark.md](results/2026-11-01-merged-benchmark.md).
Earlier results (the Bosch 800 slot benchmark, the 4.5 HOTEC bench, the 27 sampler pairs) ran on Isaac Sim 4.5 and are
history: their notes are under [results/](results/), the code in git history (tag `isaac45-final`).

## Where things live

| Path | What |
|---|---|
| `code/planner/bench.py` | the benchmark CLI |
| `code/src/bench/` | the benchmark's stages (`load` ... `graph`, `run`, `report`) |
| `code/src/dishsim/` | the episode driver (`rearrange.py`), `rrt.py`, the Kit launcher (`app.py`), media, a cache-based FCL world |
| `code/{initialization,planner,execution,util}/` | the stages' scripts, in order ([code/README.md](code/README.md)) |
| `code/frigidaire/` | the Frigidaire twin: loaders, scorer, `docs/`, tests (tracked in git; the generator ships in `data/assets/frigidaire_fdpc4221as/source/`) |
| `data/assets/` | gitignored; a copy of the HF dataset `shu4dev/dishsim-assets`: `hf download` to restore, `hf upload` to publish |
| `data/media/ data/results/ data/logs/ data/outputs/` | gitignored |
| `docs/figures/` | the only tracked media, with provenance in its README |

## Rules that bite

1. Never upgrade or downgrade Isaac Sim / Isaac Lab in place; the box image is `dishsim-isaac:6.2.0`.
2. The baseline must be recorded at the twin's rack travel; a baseline from another travel ends in `baseline_failure`.
3. The fair score's ceilings come from `data/results/dot_rules/ceiling_62/ceilings.json`; a stage without it fails.
4. Every Kit script calls `dishsim.media.release_sim_for_close()` before closing, so the `[RESULT]` line is flushed.
5. Files the container writes are root-owned: move or delete them with `docker exec dishsim-isaac ...`, never host sudo.

## Reference documentation

| Doc | Contents |
|---|---|
| [docs/environment.md](docs/environment.md) | Stack, container recipe, Isaac Lab 3.0 API notes, launcher landmines (canonical) |
| [docs/architecture.md](docs/architecture.md) | Code structure, Kit-free vs Kit-side layering |
| [docs/repo_map.txt](docs/repo_map.txt) | The whole repo on one page: assets, the sampler (scorer, robot), the two branches, the final output |
| [docs/maps/](docs/maps/) | One detail map per block of the repo map |
| [docs/known_limitations.md](docs/known_limitations.md) | Negative results and open items, with measurements |
| [docs/hotec_wheatstraw_asset.md](docs/hotec_wheatstraw_asset.md) | The HOTEC wheat-straw plate/bowl/cup USD assets: parameters, capacities, Frigidaire load render |
| [code/frigidaire/docs/geometry.md](code/frigidaire/docs/geometry.md) | Frigidaire FDPC4221AS rack, basket and appliance geometry (tape re-measured 2026-09-28) |
| [code/frigidaire/docs/hotec_bench.md](code/frigidaire/docs/hotec_bench.md) | The 2026-09-23 HOTEC bench whose start generator, fault rule and episodes the benchmark keeps |
| [code/frigidaire/docs/exposure.md](code/frigidaire/docs/exposure.md) | Exposure scorer and fair score reference ([quickstart](code/frigidaire/docs/exposure_quickstart.md) for humans) |
| [code/frigidaire/docs/dependency.md](code/frigidaire/docs/dependency.md) | The insertion build, certify and the physical-cause graph |
| [code/frigidaire/docs/robot.md](code/frigidaire/docs/robot.md) | UR5e + Robotiq 2F-85 in the Frigidaire twin: bring-up and the hardware-limits study (closed 2026-10-30) |

## Assets and licenses

This project builds on the following open-source projects; visit each URL for its license:

1. https://github.com/isaac-sim/IsaacLab, simulation framework (v3.0.0 API)
2. https://github.com/isaac-sim/IsaacSim, simulator and PhysX ground truth
3. https://github.com/NVIDIA/warp, the ray casts of the exposure score
4. https://github.com/BerkeleyAutomation/python-fcl, Pan, Chitta, Manocha, "FCL: A general purpose library for collision and proximity queries" (ICRA 2012)
5. https://github.com/mikedh/trimesh, mesh processing throughout the asset and collision pipelines

The Frigidaire twin and the HOTEC set are self-authored from tape measurements and the products' listings; no
third-party geometry is redistributed. Downloaded and derived assets are never committed to this repo. The asset
dataset `shu4dev/dishsim-assets` keeps the retired Bosch 800, ArtVIP and YCB-prop folders as history, under their
sources' licenses (ArtVIP `dishwasher_2`, Apache-2.0; YCB Object & Model Set, Calli et al., IEEE ICAR 2015).
