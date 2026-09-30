# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A **physics-validated rearrangement planning benchmark**: given a settled initial arrangement
and an exact target arrangement of kitchen objects in an articulated dishwasher (the
self-authored Bosch 800 twin; the ArtVIP baseline ships too), an algorithm moves one object at
a time by **teleportation** and Isaac settles every move. Feasible = **collision-free**
(Kit-free FCL pose query, `CollisionWorld.object_in_collision`) + **physically stable** (Isaac
settle validation). The Bosch benchmark has no robot arm, no grasping, no motion planning (the
old robot stack lives in git history / other branches); a fresh UR5e + Robotiq bring-up in the
Frigidaire twin is in progress, see `frigidaire/docs/robot.md`. `dishsim/compat.py` computes the
provably minimum move count per instance, so results are quoted as an optimality gap.

The minimal pipeline, in order (this is the whole Bosch benchmark):

1. **Bring-up** — `scripts/tools/bootstrap.sh`: image build if absent, `compose up`, archive
   restore (validates every cache's `config_hash`), kit_smoke install gate.
2. **Generate** — `gen_instances.py`: seeded, physically settled problem instances (JSON).
3. **Problem images** — `instance_views.py`: initial-vs-goal stills per instance.
4. **Benchmark** — `run_rearrange.py [--video]`: closed-loop episodes; every move teleports +
   settles, episode aborts on the first fault; records + progress MP4s.

Runs on Isaac Sim **4.5.0** + Isaac Lab **v2.1.1**, baked into the docker image
`dishsim-isaac:4.5.0` on the corallab workstation (the host's 535-series driver caps Isaac
Sim at 4.5.0). The repo is bind-mounted at `/workspace/dishsim` inside the long-lived
container `dishsim-isaac`. Never upgrade or downgrade Isaac Sim / Isaac Lab. Everything runs
`--headless`; media capture additionally needs `--enable_cameras`. Full environment write-up
(canonical launcher landmines): `docs/environment.md`.

**Shared machine**: pick the least-loaded GPU per shell (`nvidia-smi`, then
`DISHSIM_GPU=<n> docker compose -f docker/compose.yaml up -d`); never touch other users'
containers, images, or directories. All mutable data (assets/media/results/logs/outputs, Kit
caches) lives on the 2 TB drive under `/media/corallab-s1/2tbhdd/brianshu/dishsim/` — the
repo's data roots are symlinks there and the root disk must gain nothing. The container runs
as root, so files it writes are root-owned: clean them via `docker exec dishsim-isaac rm`,
never host sudo.

**Git is handled by the user, not by Claude** — no branches, commits, or pushes from
sessions; end each work phase with a summary and a suggested commit message (imperative
~50-char subject, no AI attribution/co-author lines).

## Commands

Both wrappers are dual-mode: on the host they `docker exec` themselves into `dishsim-isaac`
(mapping the cwd); inside they exec directly. `run_kit.sh` boots Kit; `run_py.sh` runs
Kit-free python (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` baked in). There is NO venv. Run from
the project root:

```bash
# tests (Kit-free). test_rack_gen_frozen.py digests are pinned PER numeric environment
# (this box: numpy 1.26 / Kit py3.10); it MUST pass in this container — failing HERE is drift.
scripts/run_py.sh -m pytest tests/

# Kit-free capacity sanity (seconds; the plan is recomputed in-process by gen_instances)
scripts/run_py.sh -c "
import sys; sys.path.insert(0, 'src')
from dishsim import config
config.apply_machine('bosch800'); config.apply_base_placement('side_winner')
from dishsim import capacity
plan = capacity.plan_full_load(log=lambda *_: None)
print('total', plan.total_items)  # expect 39 (placement 15 = plate 7 + bowl 8)"

# generate settled benchmark instances (saved artifacts; per rack state)
scripts/run_kit.sh scripts/setup/gen_instances.py --headless \
    --mode perturbed --state placement --n 3 --seed 0

# one instance's initial-vs-goal stills (the PROBLEM)
scripts/run_kit.sh scripts/evaluation/instance_views.py --headless --enable_cameras \
    --instance results/instances/bosch800/placement/perturbed_s0.json

# run algorithms closed-loop (the SOLVING; one Kit session per state batch)
scripts/run_kit.sh scripts/experiment/run_rearrange.py --headless --enable_cameras --video \
    --instances "results/instances/bosch800/placement/*.json" --algorithms greedy

# BENCHMARK tiers (dishsim/tiers.py: 3 presets + 9 ablation cells; counter-occupancy cap,
# authored swap cycles, spun per-object goal rotations, cap-aware compat certificate in
# every instance's meta). Generate a cell, run cells, aggregate:
scripts/run_kit.sh scripts/setup/gen_instances.py --headless --cell medium --n 10 --seed 0
scripts/run_kit.sh scripts/experiment/run_rearrange.py --headless \
    --cells easy,medium,hard --algorithms greedy          # 60 s planning budget default
scripts/run_py.sh scripts/evaluation/compare_algorithms.py  # -> results/compare/summary.{csv,md}

# rebake ONE (object, state) cache after a hashed-config change (extract -> decompose):
scripts/run_kit.sh scripts/setup/extract_geometry.py --headless \
    --machine bosch800 --placement side_winner --scenario placement --object cup
scripts/run_py.sh scripts/setup/decompose_meshes.py \
    --machine bosch800 --placement side_winner --scenario placement --object cup
```

Caches ship in the public archive — restore first (`bootstrap.sh` / `restore_assets.py`),
never rebake what the archive carries. Archive tags dated 2026-09-10 or later are cut from THIS
box and already carry the exact E_door_4 pieces (`COACD["E_door_4"]["preprocess_mode"] = "off"`);
only a restore of an older tag needs the one-off `decompose_meshes.py` pass described in
docs/known_limitations.md. Check archive state with
`scripts/run_py.sh scripts/tools/archive_assets.py --status` (read-only, no token).

**`./isaaclab.sh -p` exits 0 even when the wrapped script crashes.** Judge every Kit run from
log content (`[RESULT] PASS`, absence of tracebacks / `free(): invalid pointer`), never the
exit code.

**Every standalone Kit script calls `dishsim.media.release_sim_for_close()` before
`simulation_app.close()`.** Without it, isaaclab 2.1's on-stop callback spins the shutdown
forever at full CPU, and Kit's fast-exit discards block-buffered stdout — including the
`[RESULT]` line the previous rule depends on. New Kit scripts must keep this pattern.

## Validation status

**`placement` is green on this box** (2026-08-30, corallab / Isaac 4.5 port, bosch800 @
side_winner): pytest → restore PASS → kit_smoke → capacity 39/15 → gen_instances →
instance_views → run_rearrange all `[RESULT] PASS`; greedy solved 3/3 perturbed 15-item
instances in 9 moves of 45 (provable optima 9/9/9 — `compat.optimal_moves`; the first pins of
9/8/8 were a table-reuse bug, re-pinned 2026-08-31, and that run left no records). `pytest
tests/` is 65 tests, about 70 s (2026-09-29). Instances are
PER-MACHINE artifacts (PhysX 4.5 settle fixed points differ from the retired 6.0.1 cloud
box), so the at-goal/optima pins in `tests/test_compat.py` belong to this box's instances.
Other rack states (`third_out`, `middle_out`) have never run under Kit here; the fault knobs
are module constants in `src/dishsim/rearrange.py` — deliberately outside `config.py` so they
can never touch `config_hash` — and are widened only against a measurement, never to make a
run pass.

## Launcher landmines (why the scripts look the way they do)

Full write-up in `docs/environment.md` (canonical); violating these produces native crashes
or silent import shadowing, not clean errors:

1. **Boot-first**: every Kit entry script launches `AppLauncher` *before* importing `dishsim`,
   `isaaclab.*` scene modules, or `pxr`.
2. **The package is `dishsim`, deliberately not matching the repo directory name** — Kit's
   extension scan turns a same-named directory into a shadowing namespace package. Only
   `scene.py`/`machine.py` import Kit at module scope.
3. **Isaac Lab 2.1 API**: the PROJECT convention is XYZW everywhere (configs, caches,
   records); isaaclab 2.1 is **WXYZ** at its surface — every quaternion crossing the boundary
   goes through `dishsim/quats.py`, and nowhere else. Plain tensors (no `.torch`), plain
   write methods (no `*_index`).
4. **Fabric staleness**: live-stage prim transforms are stale mid-sim; extract geometry with
   `use_fabric=False` (physics-backed `.data` buffers are always correct).

## Architecture traps (full tree: docs/architecture.md)

- **FROZEN CACHE ANCHORS** — `config.py` keeps robot-era constants (`HOME_Q`,
  `GRIPPER_APERTURE_GRASP_RAD`, `T_WRIST3_TCP_*`, `GRASP_TCP_OBJ_*`, `ROBOT_BASE_*`/
  `BASE_PLACEMENTS`) ONLY because they feed `geometry.config_hash()`, which keys every
  shipped cache. Never tune them; a change silently invalidates every bake. The base frame
  every cached coordinate is expressed in is the robot-era mount.
- **`config_hash()` is NOT the only cache key**: `geometry.coacd_dir_for` digests mesh bytes
  + the body's `COACD` params, which are absent from the hash — a static-CoACD edit is loud
  at load ("missing CoACD pieces") but invisible to the staleness check; re-decomposing is
  Kit-free and invalidates nothing. Hash-SAFE knobs: `COLLISION_MARGIN_M`, `RELEASE_HOVER_M`,
  `TASK[...]`, cameras, everything in `rearrange.py`. Hashed (full Kit rebake): `RACK_GEN`,
  `MACHINE_GEN`, object `spec.coacd`, the frozen anchors.
- `gen_instances.py` records a **reproduction gate**, not merely a settled pose: it rehearses
  the runner's episode reset (teleport → re-settle → compare) and stores the re-settled fixed
  point, re-rolling failures. Do not "simplify" this to a single settle.
- `placement.py`/`capacity.py`: certify at the mode's RELEASE HOVER, never zero hover — a
  resting object touches its own support, so the inflated hull at rest collides with it by
  construction. At-goal is judged on the SETTLED pose via `evaluate_placement`.
- `rearrange.py` is the benchmark core, Kit-free by the oracle/world seam. New algorithms:
  implement `reset(instance, world)` / `next_move(obs)`, one line in `ALGORITHMS` in
  `run_rearrange.py`; accept a `seed=` kwarg if stochastic (the runner delivers a
  per-(instance, algorithm) sha256-derived seed). `obs` carries `counter_cap` /
  `counter_count` — a move onto a full counter is refused (`counter-full`, non-fatal,
  counted); 25 straight refusals abort `refusal-loop`. `failed-settle` (a drifting settle)
  is NON-fatal (oracle teleports the item back, the move counts; 25 straight abort
  `settle-loop`); a put-back that does not reproduce (`unstable-settle`) and `disturbed`
  stay fatal. The optimality gap in `compare_algorithms.py` reads each instance's cap-aware
  `meta.optimum` (computed at generation; never re-solved).
- Every number is a *measured* value (`docs/joint_report.md`, `docs/bosch800_source_data.md`)
  — never eyeball-edit. Spawn poses place the articulation **root link** (`E_body_5`, not the
  asset origin).

## Frigidaire exposure scorer (2026-09-17)

A Kit-free ray-cast proxy that ranks arrangements of the SAME objects in the Frigidaire
FDPC4221AS by how much of their food-contact surface the spray arms can reach; it is not a
cleaning measurement. Lives in the `frigidaire/` package (tracked in git): `exposure.py` (the
scorer, used by the HOTEC bench and top-5 code) and `frigidaire/tests/test_exposure.py` (two of
its tests run on settled states kept in `frigidaire/tests/fixtures/settled_states/`). The v3-era
demo, summary, search and settled-best scripts, their results, and the organized/packing
pipelines they read were retired 2026-09-29: code in git history (last at commit 4455813), result
folders in `dishsim/_trash_20260929/` until 2026-10-29; their numbers ran on the v3 racks and are
history. Reference and landmines: `frigidaire/docs/exposure.md`; quickstart
`frigidaire/docs/exposure_quickstart.md`. Live outputs: `results/exposure/frigidaire/{hotec,hotec_heightfix}`
(pre-2026-09-28 racks, history). Trap: Warp kernels must live in a file (never `python -c`).

## Frigidaire planner helpers (live half of the 2026-09-20 arrangement planner)

The pool-search arrangement planner (ExposurePlanner, FirstFitBaseline, the geometric pool, the
`frigidaire_planner_{instances,generate,run}.py` scripts, `results/planner/frigidaire/`) was retired
2026-09-29 (code in git history, results in `dishsim/_trash_20260929/` until 2026-10-29); the HOTEC
bench and the MCTS replaced it. What remains in `frigidaire/src/dishsim_frigidaire/planner.py` serves
the bench: the FCL `PlannerWorld` (appliance mirror, counter slab at top 0.914 m, objects), the support
graph and the greedy sequencer with the counter cap, on the Bosch driver `rearrange.run_episode`;
reference `frigidaire/docs/planner.md`; `frigidaire_planner_video.py` renders episodes. Two traps:
the backend's rack-speed gate flakes (retry the other rack order), and `scripts/run_py.sh` exports
Kit's USD extension so `pxr` imports Kit-free (the FCL checker needs it).
Geometry (2026-09-28): the twin's racks and basket are tape-measured (48/64 tines, 1x4 basket 320 x 95 x 130 with a 220 mm handle; revisions `upper_tines_4x13_v5` / `lower_tines_6x12_v4` / `basket_1x4_320x95_v4` since the 2026-09-28 re-measurement: non-uniform column/row gaps, front/left margins from the tape, rear/right margins the rim's leftover, lower rack 561 deep and 108 tall), claims variant B retired, the v3, v4 and v5 builds archived under `build/frigidaire_collection/history/`; reference `frigidaire/docs/geometry.md`. Every Isaac result before 2026-09-28 (HOTEC v11-v13, exposure, the whole HOTEC benchmark) is stale on the new racks.

## HOTEC wheat-straw dinnerware assets (2026-09-21)

Parametric plate/bowl/cup USDs of the user's real set, built Kit-free by `src/dishsim/hotec_gen.py`
into `assets/models/hotec_wheatstraw/v1/` (metres, Z-up, base-centred origin, 192/192/168 convex
pieces, four colour variants; v1 has no MassAPI, `v2/` adds the user's measured masses as `physics:mass` on the
default prim only: plate 0.094625 / bowl 0.067375 / cup 0.0595 kg, 8-piece averages). Every non-listing
value is an `estimated` parameter in the module table; further measurements go into the next folder
(`v3/`) with `--set kind.name=value`, never overwriting an earlier one. Brimful capacities are computed to the lowest rim
point (bowl 764 vs 769 mL advertised; cup 283 vs 355 mL — the envelope cannot hold 12 oz, documented,
not tuned). Report: `docs/hotec_wheatstraw_asset.md`; tests: `tests/test_hotec_gen.py`; the Frigidaire
load/settle/orbit script is `frigidaire/scripts/evaluation/frigidaire_hotec_load.py` (`--layout-only`
first, then Kit). Landmine: the loading helpers assume centre-origin pieces — place by centroid.
Top-5 exposure loads of all 24 pieces (2026-09-29): `frigidaire/scripts/evaluation/frigidaire_hotec_top5.py --all`
(nesting scored, dishes may touch 0.8 mm, front-bank plates, joint gate with a 5 mm peak); see the report's dated section.

## HOTEC rearrangement benchmark (2026-09-23)

Frigidaire twin + HOTEC v2 set: 7 bowls (easy, counter allowance n+3), + 8 plates (medium, n+2), + 8 cups
(hard, n+1; both +1 since 2026-09-29, and goals must pass the sequencing certificate); n dishes start in messy
counter stacks, the rest dropped into the racks. No hand-given goal: the
highest-exposure load found by coordinate ascent (track A: greedy_offline / rrt_connect reach it; track B: first-fit
/ move-level MCTS (`dishsim_frigidaire/mcts.py`, since 2026-09-29; the ascent "exposure planner" is retired) build their OWN load, which is Isaac sequence-gated like the goal before replay, scored by S,
success needs a pooling-free load; no row is copied between tracks, 2026-09-28). Teleport
moves, Isaac settle per move, end check = retract both racks + containment. Kit-free library/CLI/scheduler
`frigidaire/scripts/experiment/frigidaire_bench.py`, Kit side `frigidaire_bench_kit.py`, results page
`frigidaire/scripts/evaluation/frigidaire_bench_page.py`; reference, decisions and landmines:
`frigidaire/docs/hotec_bench.md`; outputs under `results/benchmark/frigidaire_hotec/`. Two traps: pass host
paths into the container only through `rel()` (a `/home/...` path lands in the container's writable layer),
and stop jobs by PID, never broad `pkill -f` in the shared container.

## Cleanup state and landmines (2026-09-29)

- Hold folders on the 2 TB drive: `dishsim/_trash_20260917/` (only `outputs/viewer_validation`, 675 MiB, the
  retired PDF exporters' offline browser, is left of value; delete after 2026-10-17) and
  `dishsim/_trash_20260929/` (v3-era experiment results + HOTEC v2-v5, 1.3 GiB; `MANIFEST.txt`; undo = `mv`
  back to the same relative path under `repo_data/`; delete after 2026-10-29). The 98 byte-identical media
  copies deleted the same day are listed with sha256 and surviving copy in its
  `MANIFEST_4a_media_duplicates.txt` (undo = `cp -p` the survivor back; the v2-v5 survivors live in this hold folder).
- `outputs/archive/` holds the local 20260910 tarballs: the only local backup of the pinned Bosch instances.
  Never run `archive_assets.py --upload` with the documented kinds: `assets` walks all of `results/` (private
  outputs) and `models` includes HOTEC. Never hard-link or symlink two files that both sit inside `assets/ media/
  results/`: the archive tooling stores such a pair as a link member that `restore_assets.py` refuses. The only
  links are history v4/v5 claims sharing inodes with v3 (all under `build/.../history`) and the two media gallery
  zips sharing inodes with `history/{v1,v2}/archives` (the partner is outside every archive kind, so it packs as
  a plain file).
- `package_frigidaire.py --check` is red (4 assembly gates) until the two-job Kit evidence refresh
  (`frigidaire_asset_evidence.py`, physics-only then render-only): untracked `mcts.py` (and `robot/` on refresh)
  are not in the 2026-09-28 evidence hashes, and any edit to a top-level `frigidaire/src/dishsim_frigidaire`
  file stales it. Batch package edits before refreshing.
- Benchmark history traces (`stopped_20260928`, `pre_rebuild_20260928`, `history_planner_20260929`,
  `smoke_20260923`) are `*trace.jsonl.xz`; robot-era arm meshes were removed from `assets/cache`.
- In progress and untracked (commit before any cleanup; `git clean` would destroy it): `mcts.py`,
  `robot/` + `frigidaire_robot_*`/`frigidaire_grasp_test.py` (`frigidaire/docs/robot.md`), the top-5 load,
  `docs/scoring_note/`.

## Ground rules

- `assets/`, `media/`, `results/`, `logs/`, `outputs/` are gitignored; never commit them.
  `experiments/` is a symlink index over them (machine → experiment → trial); edit the real
  folders, and add new runs there as one entry with a short README.
  Curated figures go to `docs/figures/` (tracked, provenance in its README).
- Media is on-demand (`--video` needs `--enable_cameras`); JSON records are the primary
  artifacts. **Tint objects per item in any multi-object render** (`config.item_color`,
  plumbed via the `"color"` spec key) — untinted loads render as one dark-red mass. The
  episode camera is `config.EPISODE_CAMERA`.
- One frame convention everywhere, asserted in code: base frame, meters, Z-up, XYZW.
- The dishwasher base stays fixed (`fix_root_link=True`); the door stays locked open.
- Ask the user before: downloads over 2 GB, runs expected to exceed 30 minutes, opening
  ports, or installs that restructure the container.

## Research workflow (Fable plans, Opus executes)
- If your instructions say you execute research experiment plans, you are the runner: follow the plan you are given. Any other session is for discussion and planning only: do not edit code or launch runs; the only files you write are under plans/.
- An idea becomes an experiment by becoming plans/<date>-<slug>.md, filled in from plans/TEMPLATE.md. A plan is approved only when it has a numeric success criterion, a kill rule, and a compute budget.
- Execution is delegated to the experiment-runner agent (Opus). Invoke it with the plan path. Never execute a plan in this session.
- When the runner returns, read results/<slug>.md, state what was learned in 5 lines, and propose the next experiment.
