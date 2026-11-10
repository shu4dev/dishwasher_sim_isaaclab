# sampler

Block 1 SAMPLER of `docs/repo_map.txt` (generate data the hand can execute): a messy start and a valid goal for one
roster, with its dependency graph, every dish of both pickable and placeable by the robot (the hardware gate). The
package is `sampler` (`from sampler.hwgate import ...`): the benchmark's stages, each `stage(state, params)` over the
sample file (metadata, initial, goal, dag), and the CLI `scripts/bench.py` that runs them, through `stills`; the run,
video and report stages are `code/paths/`. Its two sub-blocks have their own READMEs: 1a SCORER
[scorer/](scorer/README.md) and 1b ROBOT [robot/](robot/README.md).

| Path | What |
|---|---|
| `sample.py` | the stage list, the ledger, run_stages, the four blocks, runs/<sample>.json and index.jsonl |
| `loader.py`, `schema/`, `writers/` | machine and dish manifests, checked against the schemas |
| `slots.py` | candidate sites per dish kind from the manifests |
| `difficulty.py` | level labels of an explicit roster; n from the seed |
| `initial.py`, `gate.py` | the start (Kit) and the gates (gate_initial, gate_goal) |
| `build.py` | the fit-score-prove goal: CP-SAT fit, S ascent, DAG order and its Isaac build |
| `hwgate.py` | the hardware-aware goal: the fit's packing gated by the robot's Kit-free gate |
| `goal.py`, `graph.py` | certify (fresh-process rebuild), the physical-cause graph |
| `stills.py` | the stills stage: the initial and goal stills (Kit) |
| `convert.py` | schema-1 pair files -> samples, in place |
| `order.py`, `order_check.py`, `pick.py` | the DAG build-order studies (history) |
| `initial_state_candidates.py`, `initial_state_runtime.py` | the FCL collision checker of load states; the joint Isaac validation |
| `random_poses.py`, `random_pose_experiment.py`, `random_pose_runtime.py` | the random-pose sampling rules, trial scheduling and Isaac backend |
| `scripts/` | the CLI and the Kit and Kit-free scripts the stages call (below) |
| `tests/` | the sample pipeline, slots and order tests, the initial-state and random-pose tests (Kit-free) |
| `docs/` | `hotec_bench.md` (the 2026-09-23 bench the benchmark keeps), `dependency.md` (the insertion build, certify, the graph), `history/` |

## Initialization

Everything that must exist before a planner is asked anything: the Frigidaire demo, start-state gate and
random-load farm. Since 2026-10-21 the appliance twin, the dish assets, and the scripts that generate and certify
them ship in the asset dataset (`data/assets`, a copy of the HF dataset `shu4dev/dishsim-assets`: `hf download` to
restore, `hf upload` to publish; each folder's `source/` holds its generator and a README with the exact command).
The benchmark's pairs (messy start + goal) are made by `code/sampler/scripts/bench.py`; its start generator is
`code/sampler/scripts/frigidaire_bench_kit.py --start`.

- Input: the asset dataset downloaded to `data/assets`, the Frigidaire geometry in
  `data/assets/frigidaire_fdpc4221as/parameters.json`.
- Output: the empty-machine baseline (`frigidaire_initial_state_validate.py --out-dir
  data/results/benchmark/frigidaire_hotec/baseline`), gate records, farm loads.

The Isaac start-state gate of the Frigidaire FDPC4221AS twin (its demo is `code/asset/scripts/frigidaire_demo.py`), plus the random-load farm
(`frigidaire_random_load_kit.py`, one long-lived Kit worker building random Isaac-settled 24-dish loads, and
`frigidaire_random_load_farm.py`, its host-side launcher and harvester; usage in their docstrings). The 18 scripts
that built and certified the twin (build, stage, inspect, package, assembly evidence, rack previews, clearance, dimensions, tape check,
cutlery search, load planning, claims, full-load evidence and renders, render settings, loaded-scene smoke) moved on
2026-10-21 to the asset dataset's `data/assets/frigidaire_fdpc4221as/source/` (the Bosch lower-rack polish pair
left with the Bosch asset folder, history on the hub since 2026-11-01); each folder's README gives the exact commands. Defaults are defined in the script
itself unless another file is named. Two defaults come from `code/asset/paths.py`:
`ASSET_DIR` = `data/assets/frigidaire_fdpc4221as/usd`, `IMAGE_DIR` = `data/media/frigidaire_fdpc4221as/images`.
Kit = run through `code/util/run_kit.sh ... --headless`; Kit-free = `code/util/run_py.sh` (or host `python3`
where noted). Reference: `code/asset/README.md`, `code/asset/docs/geometry.md`.

### Commands, in order

```bash
hf download shu4dev/dishsim-assets --repo-type dataset --local-dir data/assets
code/util/run_kit.sh code/sampler/scripts/frigidaire_initial_state_validate.py --out-dir data/results/benchmark/frigidaire_hotec/baseline --headless
code/util/run_kit.sh code/sampler/scripts/frigidaire_retract_check.py --headless   # that baseline's racks retracted with the gate's ramp, both orders: peak speed + stalls, [RESULT] PASS retract check
```

Frigidaire twin: the stage, build, inspect, evidence and package commands moved with their scripts to
`data/assets/frigidaire_fdpc4221as/source/` (its README) on 2026-10-21; the dish assets' generator is in
`data/assets/hotec_wheatstraw/source/`.

## The benchmark

Where each dish goes and in what order. Every plan is checked in Isaac by one teleport per move and a settle
(an episode aborts on the first fatal fault); scoring, tables, figures and the results page belong here.

- `scripts/bench.py`: the benchmark CLI (plans/2026-11-01-merged-benchmark.md). One sample per (roster, seed): a messy start,
  the score-greedy goal and its S_ref, then the four algorithms' episodes and the report page. Stages in
  `code/sampler/` (`load` ... `graph`, `stills`), `code/paths/` (`run`, `video`, `report`).
- `scripts/` and the block folders' scripts: the Kit and Kit-free scripts the stages call (the start generator and episodes in
  `frigidaire_bench_kit.py`, own-load plans in `frigidaire_bench.py`, the build in `frigidaire_dependency_kit.py`, the
  scorer and its ceiling grids), plus the exposure, top-5 and MCTS studies.
- Output: samples and `index.jsonl` under `data/results/bench/v1/`, the episodes in its `runs/` (stage work in
  `_work/<sample>/`), media and the page under
  `data/media/bench/v1/`. The 4.5 bench's records under `data/results/benchmark/frigidaire_hotec/` are history.

### Commands

```bash
code/util/run_py.sh code/sampler/scripts/bench.py --kinds bowl --count bowl=7 --counter-allowance 3 --seed 0 --to report \
    --algorithms greedy_offline,rrt_connect,first_fit,mcts --out data/results/bench/v1
code/util/run_py.sh code/sampler/scripts/bench.py --from run --to report --input data/results/bench/v1/<sample>.json
```

## Scripts

| Script | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `frigidaire_initial_state_validate.py` | Kit (defaults `--device cpu`, headless). One fresh Isaac process: a measured baseline (no `--manifest`) or the joint validation of a load manifest; the HOTEC bench and the top-5 search call it as their gate. | `--manifest`, `--out-dir` (required), `--order` (`upper_first`), `--max-wall-seconds` (240), `--peak-penetration-m` (`LIMITS` in `code/sampler/random_poses.py`), `--usd` (`data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc`) | the manifest, the USD | `<out-dir>/result.json` |
| `frigidaire_bench_kit.py` | Kit, one process per call (the bench stages launch it). `--start`: one start attempt of the initial stage (counter pile and rack drops, joint settle, reproduction gate, unstack rehearsal). `--sequence-plan`: build an own load one dish at a time and certify the order. `--run`: every episode of one sample (`--sample`) or old instance (`--instance`), each move teleported and settled, then the end check. `--stills`: the initial and goal stills of one sample only (the stills stage). | `--count`, `--n`, `--cap` (with `--start`), `--seed` (0), `--attempt` (0), `--pile-coverage` (0.5, `PILE_COVERAGE` in the script), `--max-wall-seconds` (7200), `--gate` (the load's joint gate, with `--sequence-plan`), `--algorithms`, `--media` (stills folder), `--usd` (`data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc`), `--baseline` (`data/results/benchmark/frigidaire_hotec/baseline/result.json`); settle limits `DEV_MAX_M` (0.08 m) and `BUILD_TICKS` in the script | a sample, an instance or a plan | `--out`: `instances/attempts/s<seed>_a<attempt>/start.json`, episode records; stills under `--media` with cameras |
| `frigidaire_initial_state_render.py` | Kit. Renders one measured accepted state with Isaac RTX, without advancing physics, plus a saved-versus-rendered transform audit. As a module: `CAMERA`, `WIDTH`, `HEIGHT`, `item_tints`, `caption_fonts`, `digest`. | `--state`, `--out-dir` (both required), `--usd` (`data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc`) | the state JSON, the USD | `<id>_initial.png` (1920x1440), `<id>_render_evidence.json` |
| `frigidaire_initial_state_report.py` | Kit-free. A factual report from saved multi-dish initial-state records; missing states stay explicit. As a module: `validate_state`, `pose_error`. | `--out-dir` (required, the run folder) | the run's records | `<out-dir>/experiment_report.md` |

Also in `scripts/` (usage in their docstrings): `frigidaire_random_load_kit.py` and `frigidaire_random_load_farm.py`
(the random-load farm), `frigidaire_retract_check.py`, `frigidaire_dependency_kit.py`, `frigidaire_dependency_clips.py`
and `frigidaire_dependency_page.py` (reference `docs/dependency.md`), `frigidaire_plate_rest.py`.

The two render helpers. Scripts in `code/sampler/scripts/`, `code/asset/scripts/`, `code/paths/scripts/` and
`code/sampler/robot/scripts/` import them by bare name after putting `code/sampler/scripts` on `sys.path`: `frigidaire_bench_kit.py`, `frigidaire_hotec_load.py`,
`frigidaire_planner_video.py`, `frigidaire_robot_replay.py`. Keep the two files together: the render helper
imports the report helper from its own folder. Defaults are defined in the script itself.

## Retired experiments (2026-09-29)

The v3-era experiments (single-dish random poses, multi-dish packing, organized counterparts, exposure
demos, the pool-search planner) ran on the v3 photo-fitted racks and were retired on 2026-09-29: their
scripts, tests and protocol docs are in git history (last present at commit 4455813) and their result
folders wait in `dishsim/_trash_20260929/` until 2026-10-29 (`MANIFEST.txt` inside; undo = `mv` back).
Their headline numbers (600 random-pose proposals with 176 accepted, packing loads up to 35 dishes,
7 of 11 organized inventories) are history and were never regenerated on the tape-measured racks.
Modules that keep v3-sounding names are live: `random_pose_{experiment,runtime}.py`,
`initial_state_runtime.py`, `initial_state_candidates.py` (collision checker only), `organization.py`.
(`random_pose_assets.py` split on 2026-10-21: its input validation moved to the asset folder's
`source/`, its runtime half into `random_pose_runtime.py`.)

Benchmark: [docs/hotec_bench.md](docs/hotec_bench.md); the insertion build, certify and the graph:
[docs/dependency.md](docs/dependency.md).
