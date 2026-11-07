# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A **physics-validated rearrangement benchmark** (plans/2026-11-01-merged-benchmark.md): the Frigidaire FDPC4221AS twin
and the user's HOTEC wheat-straw set. A sample is a messy start (n dishes in counter stacks, n uniform in [N/4, 3N/4]
from the seed, the rest dropped into the racks, settled, reproduction and unstack rehearsals) and a goal: the
fit-score-prove load (plans/2026-11-02-fit-score-prove.md: an exact CP-SAT packing of the roster over the packer's
poses with no collision and no insertion cycle, raised by the score ascent on the fair score S, then built in Isaac one
dish at a time in its DAG order and gated). The goal load and its S_ref are the ground truth. Four
algorithms move one dish at a time by **teleportation** and Isaac settles every move: track A (greedy_offline,
rrt_connect) reaches the goal, reported as solved within the 60 s planning budget and raw moves; track B (first_fit,
move-level MCTS) builds its own load, scored S / S_ref. A nudge is counted, and it is fatal only when the nudged dish
was at its goal or leaves the counter and the racks. Rosters come from explicit flags; their level labels (kinds
1 / 2 / all, count 1-8 / 9-16 / 17-24, allowance n+3 / n+2 / n+1 = easy / medium / hard) are stored in the sample.

One CLI, `code/planner/bench.py`, runs the stages of `code/src/bench` in order:

1. **Bring-up**: clone the assets into `data/assets` (Commands below), record the empty-machine baseline at the
   flush rack travel (`data/results/benchmark/frigidaire_hotec/baseline/result.json`).
2. **Sample**: `load` (manifests, sites) -> `initial` (start, Kit) -> `gate_initial` -> `build` (fit-score-prove +
   Isaac build) -> `gate_goal` (joint gate) -> `certify` (witness rebuilt in a fresh process) -> `score` (S_ref at the full
   ray budget) -> `graph` (physical-cause edges) -> `stills` (the initial and goal stills, `frigidaire_bench_kit.py --stills`).
3. **Run**: the four algorithms' episodes (`frigidaire_bench_kit.py --run`) and stills, into `<out>/runs/<sample>.json`.
4. **Video**: a stop-motion video of each recorded episode (`frigidaire_planner_video.py`: the stored poses on the
   kit's scene, kinematic dishes, no physics).
5. **Report**: one HTML page over a folder's samples (`index.jsonl`, `runs/`; tables, stills, videos).

The Bosch 800 slot pipeline, the ArtVIP twin and the YCB props left the repo on 2026-11-01 (git history; the hub
folders stay as history). A UR5e + Robotiq 2F-85 hardware-limits study in the Frigidaire twin ran from 2026-10-23 to
2026-10-30 and is closed; the grasp-and-place rounds of 2026-11-04 (the arm picks each dish of s0 from the counter spot and
places it, the pads alone holding it) stand at 12 of 15 placed in one run after the fifteen-of-fifteen rounds of 2026-11-04/05 (results/2026-11-05-fifteen-r3.md), see `code/frigidaire/docs/robot.md` and "Validation status".

Runs on Isaac Sim **6.2.0** + Isaac Lab **v3.0.0** (python 3.12) on the Brev box since 2026-11-01, baked into the docker
image `dishsim-isaac:6.2.0` (results/2026-11-01-stack-upgrade.md); HEAD's Kit code is 3.0-only. The repo is
bind-mounted at `/workspace/dishsim` inside the long-lived container `dishsim-isaac`. Never upgrade or downgrade Isaac
Sim / Isaac Lab in place. Everything runs `--headless`; media capture additionally needs `--enable_cameras` (scripts boot
Kit through `dishsim.app`, which keeps both flags on 3.0). Full environment write-up (canonical launcher landmines):
`docs/environment.md`. corallab is frozen on Isaac Sim 4.5 + Isaac Lab 2.1 at tag `isaac45-final` with the retired
Bosch pipeline; nothing of HEAD runs there.

**Brev box** (`dishsim-l4` since 2026-10-22: g6.2xlarge, 1x L4 on driver 595.91.07, AMD EPYC 7R13
with 4 cores / 8 threads, 30 GiB, 117 GB root disk, per results/2026-10-22-new-box-bringup.md; recipe under
"Research workflow (repo)"). The image `dishsim-isaac:6.2.0` is built from `code/util/docker/Dockerfile` (Docker on
overlay2 since 2026-11-01: the containerd image store held 61 GB for the 4.5 image); the long-lived container
`dishsim-isaac` (`--gpus all`, root, `--restart unless-stopped`, caches on the NVMe) bind-mounts the repo at
`/workspace/dishsim`. Usually three or four parallel Kit workers, one physical core each (cpus j and j+4, pinned with
taskset inside the container; a host taskset only pins the docker CLI): the sampler rounds ran 4 slots on j = 0..3, the
2026-10-23 robot rounds 3 Kit processes with core 3 left for Kit-free, GPU or scoring jobs
(results/2026-10-22-sampler-rate-easy-medium.md, results/2026-10-23-hardware-limits-mouth-up.md). On 6.2
`--enable_cameras` works (`kit_smoke.py` renders a non-black still; the first RTX boot after a box start compiles
shaders for about 4 min); the 4.5-era stills came from `code/planner/frigidaire/frigidaire_dot_rules_render.py` (Warp
ray tracing of the same meshes). Its `data/assets` is a copy of the HF asset dataset (`hf download`). The container runs
as root, so files it writes are root-owned: move or delete them via `docker exec dishsim-isaac ...`.

**Git is handled by the user, not by Claude** — no branches, commits, or pushes from
sessions; end each work phase with a summary and a suggested commit message (imperative
~50-char subject, no AI attribution/co-author lines).

## Commands

Both wrappers are dual-mode: on the host they `docker exec` themselves into `dishsim-isaac`
(mapping the cwd); inside they exec directly. `run_kit.sh` boots Kit; `run_py.sh` runs
Kit-free python (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` baked in). There is NO venv in the container (the Mac's Kit-free
checks use `.venv-kitfree/bin/python` with `PYTHONPATH=code/src`). Run from the project root:

```bash
# assets: data/assets is a copy of the HF dataset; hf download restores and updates it
hf download shu4dev/dishsim-assets --repo-type dataset --local-dir data/assets

# tests (Kit-free)
code/util/run_py.sh -m pytest code/tests/ code/frigidaire/tests/ -q

# the empty-machine baseline at the flush travel (Kit; [RESULT] baseline_ready)
code/util/run_kit.sh code/initialization/frigidaire/frigidaire_initial_state_validate.py --headless \
    --out-dir data/results/benchmark/frigidaire_hotec/baseline

# the fair score's ceilings on this stack (GPU; theta 30, cells inside the arms' discs), then the ceilings file
code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_grid.py --kinds plate,bowl,cup --tilts=-60:0:5 \
    --spots "<h,d nodes, d <= 20 cm>" --theta 30 --N 262144 --out data/results/dot_rules/ceiling_62/inward
code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_grid.py --kinds plate,bowl,cup --tilts=0:180:5 \
    --spots "<the same>" --theta 30 --N 262144 --out data/results/dot_rules/ceiling_62/outward
PYTHONPATH=code/src python3 -m bench.score --write-ceilings     # -> data/results/dot_rules/ceiling_62/ceilings.json

# one roster through every stage (inside the container; its Kit stages call run_kit.sh themselves)
code/util/run_py.sh code/planner/bench.py --kinds bowl --count bowl=7 --counter-allowance 3 --seed 0 --to report \
    --algorithms greedy_offline,rrt_connect,first_fit,mcts --out data/results/bench/v1
code/util/run_py.sh code/planner/bench.py --kinds plate,bowl --count plate=8,bowl=7 --counter-allowance 2 --seed 0 --to report --out data/results/bench/v1
code/util/run_py.sh code/planner/bench.py --kinds plate,bowl,cup --count plate=8,bowl=7,cup=8 --counter-allowance 1 --seed 0 --to report --out data/results/bench/v1

# rerun stages on a stored sample (poses are stored, never re-derived from the seed)
code/util/run_py.sh code/planner/bench.py --from run --to report --input data/results/bench/v1/<sample>.json

# a sample with its two stills and no episode (--to stills; kept apart in samples/,
# reused later with --from run --input <sample>.json --out data/results/bench/samples --algorithms <names>)
code/util/run_py.sh code/planner/bench.py --kinds plate,bowl,cup --count plate=5,bowl=5,cup=5 --counter-allowance 3 --seed 0 --to stills \
    --out data/results/bench/samples

# schema-1 pair files -> samples in place (Mac, Kit-free; the original stays at _work/<sample>/schema1.json)
PYTHONPATH=code/src python3 -m bench.convert data/results/bench/v3
```

Outputs: samples and `index.jsonl` under `data/results/bench/v1/`, the algorithms' results in its `runs/<sample>.json`
(stage work in `_work/<sample>/`), stills, videos and the page (`index.html`) under `data/media/bench/v1/`, logs under
`data/logs/bench/`.

`data/assets` is a plain copy of the HF dataset `shu4dev/dishsim-assets` (since 2026-10-21): `hf download`
to restore, `hf upload` to publish (`hf auth login` with write access; `hf repo-files delete` removes hub files);
never rebake what it carries. The experiments load `frigidaire_fdpc4221as` and `hotec_wheatstraw`; `bosch800`,
`artvip_dishwasher_2`, `bosch_props` (history since 2026-11-01) and `frigidaire_prototypes` stay on the hub. Each
folder has `source/` (the generator or fetch code moved out of the repo; its README gives the exact command and the git
SHA of its last run) beside what experiments load. The repo no longer generates assets; evidence and results stay out
of the dataset. Index: `data/assets/README.md`.

**Exit codes never judge a Kit run.** On the box `run_kit.sh` execs `/isaac-sim/python.sh` and
every script of the 2026-11-01 stack-upgrade window returned 0, the `baseline_failure` gate included.
Judge every Kit run from log content (`[RESULT] PASS`, absence of tracebacks / `free(): invalid pointer`). A 6.2 Kit
process can also die 20 s into the scene build with `malloc(): corrupted top size` (no Python traceback) and then sit in
the crash handler with no `[RESULT]` line until it is killed: a chain driver must treat that log line as the dish's end
(the 2026-11-04 rerun lost 25 min waiting out a 1500 s limit, and an external kill trips a chain's no-state-file FAIL path).

**Every standalone Kit script calls `dishsim.media.release_sim_for_close()` before
`simulation_app.close()`.** On Isaac Lab 3.0 it only flushes stdio, which keeps the `[RESULT]`
line the previous rule depends on. New Kit scripts must keep this pattern.

## Validation status

The merged benchmark's first run (2026-11-01, dishsim-l4, Isaac Sim 6.2.0): results/2026-11-01-merged-benchmark.md. It
found no valid pair: every joint gate failed its 0.10 m/s rack-speed cap because the lower rack's wheels caught the hinge
seam on the way in (the rack-speed "flake" of the 4.5 era was that seam; probes in `data/logs/bench/probes/`). Since the
rack-wheels round (results/2026-11-01-rack-wheels-visual.md) the 8 wheels are visual only, both racks retract at 0.080
m/s, and the easy pair `bowl7_a3__s0` in `data/results/bench/v2/` is the first to pass every stage on 6.2 (S_ref 0.4134;
greedy_offline, rrt_connect and mcts solved it and have stop-motion videos since the video-port round,
results/2026-11-01-video-port.md; first_fit's own load never built). Until 2026-11-03 every lower_rear plate of every goal built on 6.2 (13 of 13) perched on the tine tips at tilt 83 deg, 71-76 mm above its packed pose, because the move released it 3 mm toward +x and the build accepted any landing within 80 mm. Since then a rear plate is released 3 mm toward -x and a plate lands only within 20 mm dz and 16 deg tilt of its packed pose, which seats every rear plate of the two samples in data/results/bench/samples within 10 mm and 5 deg (results/2026-11-03-rear-plate-drop.md).
Two 6.2 traps were found and fixed in the first window. The ground slab fought the
twin's maximal-coordinate world weld, so its pairs with the appliance are filtered in `initial_state_runtime.py`.
Frigidaire Kit processes can hang in `app.close()` after their `[RESULT]` line, so `bench.run()` kills them 90 s later.
Pairs are PER-STACK artifacts (PhysX settle fixed points differ between stacks): the 4.5 bench's instances, the 27
sampler pairs and the Bosch pins are history. The fault knobs are module constants in
`code/src/dishsim/rearrange.py` and `frigidaire_bench_kit.py` and are widened only against a measurement, never to
make a run pass. `code/tests/` and `code/frigidaire/tests/` hold the Kit-free tests; the packed-model test
`test_r05_00027_access_edges_and_witness` fails since 2026-10-23.
The robot layer (2026-11-04, results/2026-11-04-s0-rerun.md): the UR5e with the side-mounted 2F-85 (posture_v1, the
hardware frozen) spawns each dish of s0 at the counter spot (0.20, -0.12), picks it and places it with the pads alone
(`--attach none`), one Kit process per dish in two state chains, one per rack with the other rack pushed in. The arm's
access adds an ordering constraint the dag never models (plate_05 at its goal blocks plate_03's descent, so the lower
chain runs plate_03 first). 7 of 15 placed (seated on the goal wires and at goal): cups by the mouth-down side pinch
lowered until the flange sensor trips (3 of 3 attempted), bowls by the foot-ring hold under the carry rule (the held dish
5 mm from the scene on every carry sample, RRT edges at 0.02 rad, the tilt capped at max(20 deg, the descent-start tilt),
descent starts with straight legs first), plates by the wall-normal pinch 20 mm down the wall from a counter fixture (the
plate mouth down in its goal orientation on a 58 deg plank with a lip; gate 10 of 10, the first plates this hand picked
from the counter). The ceiling on this mount: plate_01 and plate_02 are unreachable (the arm against the
cabinet, the basket or the pedestal at every option; bowl_04 was too, until the side ring hold below), bowl_02 has no carry path under its cap, the build's release seats
neither bowl_03 nor cup_05, and cup_02 never ran (the malloc crash). The simulated pads hold bowls and cups at 1.4 to
3.5 N against the real hand's 20 N minimum (the plate pinch at 76 to 99 N): measured, not changed. Kit-free trap: build
the FCL/USD world inside each worker; a world built before forking a pool deadlocks every worker. The side ring hold
(results/2026-11-04-ring-side-hold.md: the cup pinch's geometry on the foot ring, the hand beside the bowl) picks 10 of 10
and reaches upper_bowl_11's descent start, but a pads-alone pinch resists a torque about the line through its two contacts
by torsional friction only: at the simulated 2 N per pad the bowl turned 38 to 50 deg in the hand on the way to its 48 deg
goal tilt, on a straight leg as on an RRT leg, so a goal tilt about the closing axis is a grip-force question, never a
planner one. Calibrated the same day (results/2026-11-04-grip-force.md): a PhysX PD drive near its joint velocity limit
delivers almost no force when stalled (finger_joint at stiffness 400 / damping 1 under the 1 rad/s limit leaves 0.17 N m
of its 8 at the ring's 0.57 rad error; damping of at least 0.57 times the stiffness keeps the squeeze; the inner finger
joints have no drive at all). At stiffness 4 / damping 2.5 the pads hold 22 N, the side ring hold placed bowl_04 at
upper_bowl_11 (2.5 mm, 2.2 deg) and bowl_01, and the cup pinch held 10 of 10 at 20 N, while the plate wall pinch needs
the fast 400 / 1 drive (2 of 10 at 4 / 2.5, the jaws short of the wall inside the 1 s close): from the next round the
drive is a per-hold command, as the real hand's force and speed are. The fifteen-of-fifteen rounds (2026-11-04/05, plans/2026-11-04-fifteen-of-fifteen.md; results/2026-11-04-fifteen-r1.md, -r2.md, results/2026-11-05-fifteen-r3.md; 251.6 billed min) moved the pedestal to posture_v2 (base 0.60, -0.70, 0.60, yaw +7 deg, spot 0.35, -0.12; the winner of a 60-mount Kit-free sweep, `frigidaire_mount_sweep.py`), lifted the silverware basket out for the robot rounds (`--no-basket`: the Kit prim's colliders deactivated, the FCL body popped; the goal poses, the benchmark samples and S_ref keep the basket; rows record basket false), lifted the bowl tilt cap (bowl_02 turned 0.99 deg in the pads at 22 N), chose the order and the holds Kit-free (`frigidaire_access_order.py`, `frigidaire_placing_loads.py --fifteen`), and added three releases: a tine-guided drop for a plate the hand cannot seat (lowered along the tine axis to the lowest height where the open hand keeps 2 mm, 60 to 65 mm for plate_05 beside plate_02, the descent line biased 5 mm toward the lean side because wrist 3 lags 2.3 deg under the plate drive, a wait until the plate rests, then the rise), the guarded insertion bounded at the Kit-free predicted stop (the bowls slide on their tines and never reach the 3.5 N trip), and cup_05's side pinch at 5 mm below the base (the palm sits 1 mm from the upper rack at the 11 mm pinch). Round 3's chain placed 12 of 15 in one run: plates 5 of 5 (plate_02 and plate_05 together), cups 4 of 5, bowls 3 of 5. Open: bowl_02 and bowl_03 slide from the predicted stop to within AT_GOAL but end off their bottom wires (14.1 deg tilt; the floor wire 2.95 mm off), and cup_05 topples 22.6 deg onto pad_L after any release at its site. The gates' holds at posture_v2 are in `data/results/robot/samples_v1/<s0>__fifteen/partA_*`, the 5 mm cup gate in `<s0>__fifteen_r3/partA_cup_side_down5`; the episode reads the closed-jaw hand from the load dir (`hold_gate_run`). Since 2026-11-05 the Warp replay (`frigidaire_robot_replay.py --load --rows --states --video-order`) draws the twin in the bench stills' look: cabinet and door from components.json (no basket: it was out in every run), the stills' per-item tints and BENCH_CAMERA as a 1280 x 960 main view with the dish-following inset, the other rack's dishes as its own chain had placed them at that point of the video order, failed dishes at their landed poses (results/2026-11-05-replay-like-stills.md; the ground quad must stay 40 m, a 200 m quad slows Warp's BVH 6x).

## Launcher landmines (why the scripts look the way they do)

Full write-up in `docs/environment.md` (canonical); violating these produces native crashes
or silent import shadowing, not clean errors:

1. **Boot-first**: every Kit entry script calls `dishsim.app.launch(args)` (over
   `isaaclab.app.launch_simulation`) *before* importing the rest of `dishsim`, `isaaclab.*`
   scene modules, or `pxr`.
2. **The package is `dishsim`, deliberately not matching the repo directory name** — Kit's
   extension scan turns a same-named directory into a shadowing namespace package. No
   `dishsim` module imports Kit at module scope. The benchmark package is `bench` (code/src/bench):
   a script that puts `code/planner` on `sys.path` would shadow it with `code/planner/bench.py`, so
   that CLI replaces its own folder on `sys.path`.
3. **Isaac Lab 3.0 API**: the PROJECT convention is XYZW everywhere (configs, caches,
   records) and 3.0 is XYZW too, so the two functions in `dishsim/quats.py` are the identity;
   they still stand at every boundary crossing and no other file reorders a quaternion. `.data` reads end in `.torch`, writes use
   `*_to_sim_index`, `SimulationCfg(physics=PhysxCfg(...))` comes from `isaaclab_physx`; the
   full up-port delta list is in `docs/environment.md`.
4. **Fabric staleness**: live-stage prim transforms are stale mid-sim; extract geometry with
   `use_fabric=False` (physics-backed `.data` buffers are always correct).

## Architecture traps (full tree: docs/architecture.md)

- The sample file is the unit (plans/2026-11-02-sample-schema.md, `code/src/bench/schema/sample.json`): exactly four
  blocks. `metadata` holds the params, the ledger, the maker summary, provenance (the `_work` start and build record
  paths, settle, gate, certify and score figures) and media (the two stills); `initial` and `goal` hold {dish: world
  pose} and the rack joints, plus the counter, or the slots and the score; `dag` the certified edges, layers and build
  order. Every stage reads and rewrites it with a ledger entry and rewrites `index.jsonl`; run, video and report write
  `runs/<sample>.json` and their ledger entries go there. Poses are stored, never re-derived from the seed. A seed is
  spent on the sampling stages (load..graph); a sample that passed them goes on to stills, run, video and report.
- `initial` records a **reproduction gate**, not merely a settled pose: the start generator rehearses the episode reset
  (teleport -> re-settle -> compare) and the unstack, re-rolling failures. Do not "simplify" this to a single settle.
- `build` packs by CP-SAT (ortools) over the packer's level-0 masks, its collision and two-cycle conflicts cached per family
  digest beside them (the upper bowl family was resampled on 2026-11-02: caches of the old digest are stale), then
  ascends at N / 16 rays (`bench.build.GAIN_RAY_FRACTION`); `score` scores the chosen load once at N 262144 for S_ref.
  Both read the ceilings from `data/results/dot_rules/ceiling_62/ceilings.json` (the 6.2 grids); a missing file fails
  the stage, the 4.5 ceilings are never a default.
- `rearrange.py` is the episode driver, Kit-free by the oracle/world seam. New algorithms: implement
  `reset(instance, world)` / `next_move(obs)` and add them to the kit's `EPISODES`; accept a `seed=` kwarg if stochastic
  (the runner delivers a per-(instance, algorithm) sha256-derived seed). `obs` carries `counter_cap` / `counter_count` —
  a move onto a full counter is refused (`counter-full`, non-fatal, counted); 25 straight refusals abort `refusal-loop`.
  `failed-settle` (a drifting settle) is NON-fatal (the oracle teleports the item back, the move counts; 25 straight
  abort `settle-loop`); a put-back that does not reproduce (`unstable-settle`) and `disturbed` stay fatal.
- The empty-machine baseline is the frame of every start, build and gate: the start kit, the dependency kit and the
  joint-gate manifests read `data/results/benchmark/frigidaire_hotec/baseline/result.json` at call time. It must be
  recorded at the twin's rack travel (flush since 2026-10-30); a baseline from another travel ends in
  `baseline_failure`.

## Frigidaire exposure scorer (2026-09-17)

A Kit-free ray-cast proxy that ranks arrangements of the SAME objects in the Frigidaire
FDPC4221AS by how much of their food-contact surface the spray arms can reach; it is not a
cleaning measurement. Lives in the `code/frigidaire/` package (tracked in git): `exposure.py` (the
scorer, used by the HOTEC bench and top-5 code) and `code/frigidaire/tests/test_exposure.py` (two of
its tests run on settled states kept in `code/frigidaire/tests/fixtures/settled_states/`). The v3-era
demo, summary, search and settled-best scripts, their results, and the organized/packing
pipelines they read were retired 2026-09-29: code in git history (last at commit 4455813), result
folders in `dishsim/_trash_20260929/` until 2026-10-29; their numbers ran on the v3 racks and are
history. Reference and landmines: `code/frigidaire/docs/exposure.md`; quickstart
`code/frigidaire/docs/exposure_quickstart.md`. Live outputs: `data/results/exposure/frigidaire/{hotec,hotec_heightfix}`
(pre-2026-09-28 racks, history). Trap: Warp kernels must live in a file (never `python -c`).

## Frigidaire planner helpers (live half of the 2026-09-20 arrangement planner)

The pool-search arrangement planner (ExposurePlanner, FirstFitBaseline, the geometric pool, the
`frigidaire_planner_{instances,generate,run}.py` scripts, `data/results/planner/frigidaire/`) was retired
2026-09-29 (code in git history, results in `dishsim/_trash_20260929/` until 2026-10-29); the HOTEC
bench and the MCTS replaced it. What remains in `code/frigidaire/src/dishsim_frigidaire/planner.py` serves
the bench: the FCL `PlannerWorld` (appliance mirror, counter slab at top 0.914 m, objects), the support
graph and the greedy sequencer with the counter cap, on the episode driver `rearrange.run_episode`;
reference `code/frigidaire/docs/planner.md`; `frigidaire_planner_video.py` renders episodes. Two traps:
the backend's rack-speed gate flakes (retry the other rack order), and `code/util/run_py.sh` exports
Kit's USD extension so `pxr` imports Kit-free (the FCL checker needs it).
Geometry (2026-09-28): the twin's racks and basket are tape-measured (48/64 tines, 1x4 basket 320 x 95 x 130 with a 220 mm handle; revisions `upper_tines_4x13_v5` / `lower_tines_6x12_v4` / `basket_1x4_320x95_v4` since the 2026-09-28 re-measurement: non-uniform column/row gaps, front/left margins from the tape, rear/right margins the rim's leftover, lower rack 561 deep and 108 tall), claims variant B retired; since 2026-10-21 the twin ships in `data/assets/frigidaire_fdpc4221as/` (`usd/`, `parameters.json`, `components.json`, which the runtime `geometry.py`/`asset.py` read) with the generator, staging, packaging, preview and evidence scripts in its `source/`, and the v3, v4 and v5 builds are not kept (they were only ever in corallab's `data/build/frigidaire_collection/history/`, never in the HF archive; the code that built each is in this repo's git history); reference `code/frigidaire/docs/geometry.md`. Every Isaac result before 2026-09-28 (HOTEC v11-v13, exposure, the whole HOTEC benchmark) is stale on the new racks.

## HOTEC wheat-straw dinnerware assets (2026-09-21)

Parametric plate/bowl/cup USDs of the user's real set, built Kit-free by `hotec_gen.py` (in
`data/assets/hotec_wheatstraw/source/` with its test since 2026-10-21). The dataset ships v2 only:
`data/assets/hotec_wheatstraw/usd/{plate,bowl,cup}.usda`, `parameters.json` and `catalog.json` one level up
(metres, Z-up, base-centred origin, 192/192/168 convex pieces, four colour variants; the user's measured masses
as `physics:mass` on the default prim only: plate 0.094625 / bowl 0.067375 / cup 0.0595 kg, 8-piece averages;
the massless v1 was only ever on corallab's `data/assets/models/hotec_wheatstraw/v1/` and is not in the dataset). Every non-listing value is an
`estimated` parameter in the module table; further measurements rebuild with `--set kind.name=value` (the
generator never overwrites a folder), the dataset keeping the latest version and its git history the earlier
ones. Brimful capacities are computed to the lowest rim
point (bowl 764 vs 769 mL advertised; cup 283 vs 355 mL — the envelope cannot hold 12 oz, documented,
not tuned). Report: `docs/hotec_wheatstraw_asset.md`; the Frigidaire
load/settle/orbit script is `code/planner/frigidaire/frigidaire_hotec_load.py` (`--layout-only`
first, then Kit). Landmine: the loading helpers assume centre-origin pieces — place by centroid.
Top-5 exposure loads of all 24 pieces (2026-09-29): `code/planner/frigidaire/frigidaire_hotec_top5.py --all`
(nesting scored, dishes may touch 0.8 mm, front-bank plates, joint gate with a 5 mm peak); see the report's dated section.

## HOTEC rearrangement benchmark (2026-09-23, merged 2026-11-01)

The merged benchmark of "What this is" keeps the 2026-09-23 bench's start generator, fault rule, episodes and track B
own-load certification (plan -> joint gate -> one-dish-at-a-time build -> resequence) and replaces its goal (the
coordinate-ascent load, retired with `--goal`, `--finalize`, `--generate` and `--full`) with the score-greedy pair. The
first-fit planner is `first_fit` (named `baseline` until 2026-11-01). Kit-free library `code/planner/frigidaire/frigidaire_bench.py`
(`instance_of_sample`, the own-load plans), Kit side `frigidaire_bench_kit.py` (`--start`, `--sequence-plan`, `--run`, `--stills`);
reference, decisions and landmines of the old bench: `code/frigidaire/docs/hotec_bench.md`; its 4.5 outputs under
`data/results/benchmark/frigidaire_hotec/` are history (the baseline there is live). Two traps: pass host paths into
the container only through `rel()` (a `/home/...` path lands in the container's writable layer), and stop jobs by
PID, never broad `pkill -f` in the shared container.

## Cleanup state (hold folders)

Hold folders (each with a `MANIFEST.txt`; undo = `mv` back): on corallab's 2 TB drive `dishsim/_trash_20260917/` (delete after 2026-10-17) and `dishsim/_trash_20260929/` (v3-era results + HOTEC v2-v5; delete after 2026-10-29), both delete-after dates now passed (2026-10-31), on the Brev box dishsim-l4 `data/_trash_20261021/` (what the 2026-10-21 asset restructure dropped; delete after 2026-11-21), the Mac's own `data/_trash_20261021/` and `data/_trash_20261029/` (the -0.49 / -0.44 rack-travel twin files and the HOTEC bowl v2 and v3 files, each with a MANIFEST.txt; delete after 2026-11-29), and `data/_trash_20261101/` on the Mac and on the box (the local copies of the removed asset folders `bosch800`, `artvip_dishwasher_2` and `bosch_props`; delete after 2026-12-01).

## Pages and the fair score (2026-10-22)

The user's two pages, private until the user shares them: **Dot Score Rules**
https://claude.ai/artifact/F5vrhrNM4XnUk3ZXhJycY2, Version 30 (2026-10-22: Version 23 of 2026-10-21, with the rules,
the New+J explorer with "relative to the kind's best" and the per-dot figures, plus the merged one-panel tilt sketch
under the explorer, added by hand patch; results/2026-10-22-tilt-score-check.md; the 2026-10-08 original, N = 64, is
`docs/Dot Score Rules/`), and **Dependency graphs** https://claude.ai/artifact/AkiJZLtFZnFtXYUxSEHJmm, Version 18
(2026-10-23: one complete valid sample per difficulty, easy 7 bowls, medium 15 dishes, hard 23, each with its initial
still, goal and S, witness video and graphs, beside r05_00027 and the 70-packing yield;
results/2026-10-23-one-sample-per-difficulty.md).

- **Fair score** (`code/frigidaire/docs/exposure.md`, beside revision 5): a dish's object-level New+J over its kind's
  ceiling under its own arm, clipped at 1, and 0 for a dish that pools (interior more than 2 mm below its rim); a load's
  S is the plain mean over its dishes, "surface reached" the area-weighted mean. Since 2026-10-22 the ceilings are
  computed per arm, never typed: the largest `<kind>_obj` New+J at θ 30° over every grid cell inside the arm's disc and
  every tilt of the outward payload and the inward grid (both cast under the lower arm's disc). The computed values, as
  saved in `ceilings_kind` of `data/results/dot_rules/dependency/samples_v5/r05_00027.relative.json` (and listed in
  results/2026-10-22-inward-ceilings-and-smokes.md): lower arm (r 0.233 m) plate 0.8758, bowl 0.6012, cup 0.2287;
  middle arm (r 0.191 m) plate 0.8587, bowl 0.5525, cup 0.2099. `frigidaire_dot_rules_relative.py` computes them and
  writes them with S into the saved `.relative.json` files.
- **Samples and graph** (`frigidaire_dependency_kit.py --sample`, reference `code/frigidaire/docs/dependency.md`): a
  restricted packing is built one dish at a time by the insertion move (lowered from 20 mm above the rim in 5 mm steps,
  plates upright and 3 mm toward their packed lean's side in the front bank and toward the other side in the rear bank, let go 20 mm above where the rack stops it), in a random order with each plate bank filled from its
  lean side, with re-posing, deferral and the joint gate's 1 mm overlap rule after each drop. Valid = the built poses
  pass the joint gate, the witness order rebuilds all 24 in a fresh Kit process, and every arrow is a physical cause (a
  hard support confirmed by drops, or a covered insertion path).
- **Rebuild** (Mac, Kit-free): `python3 code/planner/frigidaire/frigidaire_dependency_page.py --records
  r05_00027=data/results/dot_rules/dependency/samples_v5/r05_00027.json --clips
  r05_00027=data/media/dot_rules/dependency_samples_v6/r05_00027 --state
  data/logs/dot_rules/samples_v6b/scheduler_state_merged.json --out <dir>` gave Version 13 byte for byte before the
  2026-10-22 rescore (Version 18 adds `--pairs` and the three tier samples: the command is in
  results/2026-10-23-one-sample-per-difficulty.md);
  `python3 code/planner/frigidaire/frigidaire_dot_rules_page.py --grid-payload data/media/dot_rules/page_v22/index.html
  --figures-dir data/media/dot_rules/page_v22/media --out <dir>` gives Version 23 except the numbers only the box's grid
  holds (header tiles, the rules' cases, the per-dot captions: nan); Versions 24 to 30 are hand patches of that page
  (results/2026-10-22-tilt-score-check.md), Version 30 in `data/media/dot_rules/page_v30/`. On the box
  `frigidaire_dot_rules_figures.py`, then
  `frigidaire_dot_rules_page.py --grid-dir data/results/dot_rules/grid/N0262144 --grid-obj-dir
  data/results/dot_rules/grid_obj/N0262144` build everything (pass level dirs, not the root; always give
  `frigidaire_dot_rules_grid.py` an `--out`, its default overwrites the old levels).
- **Publish** with the Artifact tool to the same URL, `root` = the page dir and a files map of its `media/*` (set a
  dropped file to null); index.html must stay under 15 MB (the Dot Score Rules page script palettises PNGs above
  10 MiB). Each version's plan, run note and open questions are in plans/, results/ and research/.

## Ground rules

- `data/assets/`, `data/media/`, `data/results/`, `data/logs/`, `data/outputs/` are gitignored; never commit them
  to this repo (`data/assets` is a copy of the HF dataset, published with `hf upload`).
  `data/experiments/` is a symlink index over them (machine → experiment → trial); edit the real
  folders, and add new runs there as one entry with a short README.
  Curated figures go to `docs/figures/` (tracked, provenance in its README).
- Media is on-demand (`--video` needs `--enable_cameras`); JSON records are the primary
  artifacts. **Tint objects per item in any multi-object render** (the Frigidaire renders use
  `frigidaire_initial_state_render.item_tints`) — untinted loads render as one dark-red mass.
- One frame convention everywhere, asserted in code: base frame, meters, Z-up, XYZW.
- The dishwasher base stays fixed (`fix_root_link=True`); the door stays locked open.
- Ask the user before: downloads over 2 GB, runs expected to exceed 30 minutes, opening
  ports, or installs that restructure the container.

## Research workflow (repo)
- Files: research/ (lit notes), plans/ (plans), results/ (run notes; the ones written to data/results/ before 2026-10-15 moved here). Templates: research/TEMPLATE.md, plans/TEMPLATE.md.
- Runners: experiment-runner for experiments, refactorer for structure changes. Invoke with the plan path.
- How to run things here: plan on the Mac, run on the Brev box (see "Brev box" above). It bills while running: start it only to run, stop it right after.
  - Environment: the `dishsim-isaac` container on the box, which runs what is pushed on `plan` (the user commits and pushes, see the git rule above).
    1. Mac: `brev start dishsim-l4`; wait until `ssh dishsim-l4 true` works (`brev refresh` if the hostname changed).
    2. `ssh dishsim-l4`, then `cd ~/isaac-launchable/dishwasher_sim_isaaclab && git pull --ff-only` (public repo, no credentials).
    3. From the repo root: `code/util/run_kit.sh <script> --headless` (Kit) or `code/util/run_py.sh <script>` (Kit-free); both docker exec into the container. Long or parallel jobs: `setsid nohup docker exec -w /workspace/dishsim -e OMP_NUM_THREADS=2 dishsim-isaac taskset -c <j>,<j+4> nice -n 10 code/util/run_kit.sh ... > data/logs/<area>/<log>.log 2>&1 &`.
    4. Write results/<slug>.md and end with a suggested commit message. The box has no GitHub credentials, so commits made there leave through the Mac: `git pull --ff-only brev plan && git push` (remote `brev` = `dishsim-l4:isaac-launchable/dishwasher_sim_isaaclab`).
    5. Mac: `brev stop dishsim-l4`.
  - Smoke run, under 2 min: `code/util/run_py.sh -m pytest code/tests/ -q` (Kit-free; with `code/frigidaire/tests/` added it took 64 s on dishsim-l4 on 2026-10-22, 263 passed and 24 subtests, results/2026-10-22-window-sampling-v7-and-section2.md; `kit_smoke.py --enable_cameras` passes there since the 6.2.0 port of 2026-11-01, when pytest gave 264 passed, 24 subtests, 2 failed: test_compat before its re-pin and the packed-model test failing since 2026-10-23).
  - Data / checkpoints / logs: on the box only, gitignored under data/: outputs `data/results/<area>/`, media and pages `data/media/<area>/`, logs `data/logs/<area>/`.
