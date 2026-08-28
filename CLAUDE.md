# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A **physics-validated rearrangement planning benchmark**: given a settled initial arrangement
and an exact target arrangement of kitchen objects (14-class library) in an articulated
dishwasher (experiments: the self-authored Bosch 800 twin; the ArtVIP baseline ships too),
an algorithm moves one object at a time by **teleportation** and Isaac settles every move.
Feasible = **collision-free** (Kit-free FCL pose query, `CollisionWorld.object_in_collision`)
+ **physically stable** (Isaac settle validation). There is no robot arm, no grasping, and no
motion planning on this branch (that stack lives in git history / `main`; the old RL
door-opening pipeline on `archive/rl-door-opening`). Isaac Sim is the physics validator and
evidence renderer only; planning runs in the plain venv.

The pipeline is a **rearrangement benchmark**: bake a machine state's collision caches
(`build_state.py`) → generate settled problem instances (`gen_instances.py`: exact target
poses from the capacity plan, seeded initial arrangements, saved JSON artifacts) → run
algorithms closed-loop (`run_rearrange.py`: one persistent Kit session; every move teleports
one object, settles, and ABORTS the episode on the first fault — colliding command, unstable
settle, disturbed neighbor — under a move budget) → render evidence on demand
(`reveal_render.py` for a planned load, `instance_views.py` for one instance's initial-vs-goal
pair, `run_rearrange.py --video` for the episode MP4). A capacity plan is additionally
settle-certified by `capacity_fill.py` (items arrive one at a time under physics; FCL-placeable
is not the same claim as physically seated). The benchmark core is `dishsim/rearrange.py` (Kit-free
episode driver + FCL arrangement mirror + greedy baseline; algorithms implement
`reset(instance, world)` / `next_move(obs)`). `dishsim/capacity.py` stays as the target
generator and packing baseline. Slots derive live from the cached rack geometry
(`placement.derive_slots`) — there is no slot/goal bake. Experiments run on the Bosch only;
the robot-era layout/support/phases modules live in git history.

Runs on Isaac Sim **6.0.1-rc.7** + Isaac Lab **3.0.0** at `/workspace/isaaclab` (this repo is
nested inside that tree, but is an independent git repo). Never upgrade or downgrade Isaac Sim /
Isaac Lab. Everything runs `--headless` on a single NVIDIA L4 / 8 vCPU / 30 GiB; media capture
additionally needs `--enable_cameras`.

The parent tree's `CLAUDE.md`/`AGENTS.md` (IsaacLab contributor rules) mostly targets the Isaac
Lab source itself — its changelog-fragment and `./isaaclab.sh -f` pre-commit workflow do **not**
apply here. What does carry over: commit-message conventions and **no AI attribution/co-author
lines in commits**.

**Git is handled by the user, not by Claude** — no branches, commits, or pushes from sessions;
end each work phase with a summary and a suggested commit message instead.

## Commands

Isaac-side work goes through `scripts/run_kit.sh` (exports the Isaac env, then
`exec isaaclab.sh -p "$@"` — required because the venv-resolved interpreter lacks
`EXP_PATH`/`LD_LIBRARY_PATH`; bare `isaaclab.sh -p` dies at Kit boot with `KeyError: 'EXP_PATH'`).
Pure planning work uses the venv python directly. Run from this project root:

```bash
# planning-stack tests (venv, no Kit; plugin autoload off — hydra's plugin breaks outside Kit).
# The suite is 3 Kit-free files: two frozen-invariant pins + the benchmark's toy-oracle check.
# test_rack_gen_frozen.py is float-byte sensitive — it may fail under a non-pinned Python
# stack (e.g. a dev Mac), but it MUST pass in this box's venv; failing HERE is real drift.
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest tests/

# plan the placeable full load (Kit-free; needs restored/baked caches)
/workspace/isaaclab/env_isaaclab/bin/python scripts/setup/plan_full_load.py \
    --machine bosch800 --placement side_winner

# generate settled benchmark instances (saved artifacts; per rack state)
scripts/run_kit.sh scripts/setup/gen_instances.py --headless \
    --mode perturbed --state placement --n 10 --seed 0

# run algorithms closed-loop against them (one Kit session per state batch)
scripts/run_kit.sh scripts/experiment/run_rearrange.py --headless \
    --instances "results/instances/bosch800/placement/*.json" --algorithms greedy

# render a planned load, settled (stills + orbit)
scripts/run_kit.sh scripts/evaluation/reveal_render.py --headless --enable_cameras \
    --plan results/capacity/bosch800/side_winner/full_load_plan.json

# settle-certify a capacity plan: items arrive one at a time, unstable ones are parked
# (honest capacity, never aborts); verdict comes from the FINISHED tableau, gated on
# seated AND at-goal. --video writes the fill timelapse.
scripts/run_kit.sh scripts/setup/capacity_fill.py --headless --enable_cameras --video \
    --plan results/capacity/bosch800/side_winner/full_load_plan.json --state placement

# one instance's initial-vs-goal stills (the PROBLEM, where the episode video is the solving)
scripts/run_kit.sh scripts/evaluation/instance_views.py --headless --enable_cameras \
    --instance results/instances/bosch800/placement/perturbed_s0.json

# bake a machine state's collision caches (extract -> decompose; per class)
scripts/setup/build_state.py --state placement --classes mug,cup,tumbler
scripts/setup/build_state.py --machine bosch800 --placement side_winner --state placement --classes cup

# inspect slots Kit-free (table + placeability + slot_detection.png)
/workspace/isaaclab/env_isaaclab/bin/python scripts/setup/derive_slots.py --object cup --scenario placement

# scene inspection: regenerates docs/joint_report.md, stability + passive-door tests
scripts/run_kit.sh scripts/setup/inspect_scene.py --headless --test_door

# entry points: scripts/ is split by phase (see README Usage)
#   setup/      kit_smoke, inspect_scene, extract_geometry, decompose_meshes, build_state,
#               derive_slots, preview_rack, gen_instances, plan_full_load, capacity_fill
#   experiment/ run_rearrange
#   evaluation/ reveal_render, instance_views
#   tools/      archive_assets, restore_assets, bootstrap.sh
# Caches ship in the public archive — restore first (tools/restore_assets.py), never rebake
# what the archive already carries. ONE exception until the archive is re-cut: the shipped
# E_door_4 CoACD pieces predate COACD["E_door_4"]["preprocess_mode"] = "off" (2026-08-28), so a
# restore-only box must run `scripts/setup/decompose_meshes.py` once per context — it is Kit-free
# and takes seconds. See "config_hash is not the only cache key" in docs/known_limitations.md.
```

**`./isaaclab.sh -p` exits 0 even when the wrapped script crashes.** Always verify success from
log content (`[RESULT] PASS`, absence of tracebacks / `free(): invalid pointer`), never from the
exit code.

## Kit validation status (per rack state — extend only as states actually go green)

**`placement` is green** (2026-08-28, bosch800 @ side_winner): `kit_smoke` → `gen_instances` →
`run_rearrange` → `reveal_render` → `capacity_fill` all reached `[RESULT] PASS`. Greedy solved
2 of 3 perturbed 15-item instances in 9-10 moves of a 45 budget; the third gave up at 14/15
at goal — an algorithm limit, not a harness fault (`results/rearrange/bosch800/placement/`).
The certified lower-rack load is settle-verified 15/15 seated, 15/15 at goal, 0 neighbours
disturbed (`results/capacity/bosch800/side_winner/settled_verification_placement.json`).

Do NOT read that as the stack being validated:

- **`third_out` (24 forks) has never run under Kit.** Probe it with the order below; expect fork
  drop-bounce (60 mm release hover) to trip `"disturbed"` aborts. Sanctioned knobs:
  `DISTURB_POS_M`, or a `flat_lay_third` carve-out in `IsaacOracle`.
- **`middle_out` plans zero items by design** — cup fails the measured settle-reliability gate,
  tumbler fails the z-budget. Nothing to run until that verdict is re-audited.

```bash
scripts/run_kit.sh scripts/setup/kit_smoke.py --headless --enable_cameras
# boot + FCL-under-Kit + caches + plan, all in one shot:
scripts/run_kit.sh scripts/setup/gen_instances.py --headless \
    --mode perturbed --state third_out --n 3 --seed 0
scripts/run_kit.sh scripts/experiment/run_rearrange.py --headless \
    --instances "results/instances/bosch800/third_out/*.json" --algorithms greedy
# then once per state with --video --enable_cameras for visual confirmation
```

What to watch, and where the knobs live (module constants in `src/dishsim/rearrange.py` —
deliberately NOT in config.py, so they can never touch `config_hash`). `placement` spent two of
these already; widen further only against a measurement, never to make a run pass:
- many `"init-mismatch"` aborts → recorded initials don't re-settle reproducibly. `placement`
  needed BOTH available fixes: `gen_instances.py` now double-settles (the reproduction gate —
  see the Architecture traps) and `INIT_MATCH_ROT_DEG` went 10 → 15 deg on a measured 10.5 deg
  cross-session re-settle.
- `"disturbed"` aborts → `DISTURB_ROT_DEG` is 20, not 10, because a bowl re-seats 15.9 deg at
  3.6 mm when a neighbour lands *without* leaving its goal; the 10 mm position gate is what
  still catches a genuine knock-off.
- `"unstable-settle"` on legitimate placements → check `MOVE_DEV_MAX_M` (0.06 absorbs the
  hover drop + roll-to-tine; measured, git history).

## Algorithm-comparison benchmark (MS-MCTS study) — in progress

Goal: compare MS-MCTS (arXiv:2305.17175) against flat MCTS, the greedy baseline, a monotone
floor, and a **proven optimum**, reporting success rate at a planning-time budget plus an
optimality gap. The design is settled (18 decisions); the foundation is built and verified.

**BUILT (2026-08-28, verified):**
- **The feasibility oracle is now fast enough to search against**: `move_collides`
  **429 ms -> 2.8 ms** (~21,000 queries per 60 s, was 133) via inflated-hull caching, persistent
  collision objects and a broadphase for placed items in `collision_world.py`. Equivalence
  checked on 252 verdict records + 1,800 mutation-fuzz checks: byte-identical.
- **`compat.py` — the ground truth.** Feasibility here is exactly pairwise-decomposable, so a
  compatibility table (~5k FCL queries, seconds) makes an A* over arrangements cheap.
  `compat.optimal_moves(instance)` returns the provably minimum move count. Measured on the
  shipped instances: **s0 = 9, s1 = 10, s2 = 9**.
- **Harness fairness + instrumentation**: the runner deep-copies the `Instance` per (instance,
  algorithm); `--seed` and `--time_budget_s`; `time_left_s` in `obs`; an `algo.stats()` channel;
  `ArrangementWorld.n_queries`, `snapshot()`/`restore()`; records now carry `travel_m`,
  `buffer_moves`/`goal_moves`, `T_base_from`, `planning_time_total_s`, `seed`, `instance_meta`.
- **An infeasible commanded move no longer aborts the episode** — it is refused and counted
  (`infeasible_commands`). The old behaviour systematically flattered planners that pre-check
  (the baseline does) over sampling planners that do not.
- `dishsim/instance_gen.py` holds `sample_initials` so a Kit-free generator can share it.

**NOT BUILT — the study itself:** the algorithms (`dishsim/msmcts.py`: one parameterized UCT
whose `decomposed` x `guided` flags yield all four factorial arms, plus a naive UCT and a
monotone floor), the adversarial instance families (`scripts/setup/gen_adversarial.py`), the
geometry-only sweep, and the aggregator (`scripts/evaluation/compare_algorithms.py`).

**Traps this work already paid for — do not re-derive them:**
- `compat.py` separates two things that look like one: `static_ok` (is this a legal
  DESTINATION?) from `compatible` (do objects at these two locations overlap?). A settled pose
  is statically blocked — a resting object touches its own support — yet an object sitting
  there still blocks its neighbours. Conflating them makes every move look illegal.
- Whether an item **starts at goal** is decided by the harness's own `rearrange.at_goal`, never
  by snapping its pose to a cell: recorded initials are SETTLED poses, 12.8-22.0 mm below the
  nominal hover cell. Snapping either misses them or inflates every optimum.
- `hash()` on a str is salted per process, so it cannot derive a reproducible seed
  (`run_rearrange._episode_seed` uses sha256).
- Greedy is a one-blocker-lookahead baseline (`rearrange.py`, its own docstring says
  swap-cycles need a real planner) — it is the floor, not a rival.

**What to weigh before running the study** (all measured, all in `docs/known_limitations.md`):
1. **Occlusion ordering cannot be tested in `placement`** — 0 of 15 goal insertions are blocked
   by another item under a 300 mm straight-down sweep. MS-MCTS's depth heuristic can only be
   made *wrong* here via resource contention, not occlusion. Real occlusion lives in `both_in`,
   which needs the hashed `rail_z` fix and a full rebake.
2. **Plates and bowls are provably decoupled**, so the 15-item problem factorizes into 7 + 8.
   Cross-class blocking has to be hand-authored (a flat plate on the bowl floor blocks 6-9 of
   the 28 placeable bowl slots).
3. **Buffer scarcity is a convention, not a rule** — `Move` accepts any pose, so an algorithm
   can park anywhere unless the harness gains a legality predicate.
4. The optimum is a **geometric-relaxation** optimum, and discretizing placements removes
   MS-MCTS's continuous buffer sampling. Both belong in any write-up.

## Launcher landmines (why the scripts look the way they do)

Full write-up in `docs/environment.md` (the canonical home); violating these produces native
crashes or silent import shadowing, not clean errors:

1. **Boot-first**: every Kit entry script launches `AppLauncher` *before* importing `dishsim`,
   `isaaclab.*` scene modules, or `pxr`.
2. **The package is `dishsim`, deliberately not matching the repo directory name** — Kit's
   extension scan turns a same-named directory into a namespace package that shadows the real
   one. Keep module-scope `pxr`/`omni` imports out of the package (convention: only
   `scene.py`/`machine.py` import Kit at module scope — a violation crashes venv imports).
3. **Isaac Lab 3.0 API**: quaternions are **XYZW** everywhere; data buffers are `ProxyArray` —
   append `.torch` (and `.torch.clone()`); kinematic writes use the keyword-only `*_index`
   methods.
4. **Fabric staleness**: live-stage prim transforms are stale mid-sim. Extract geometry from
   file stages pre-boot, right after `sim.reset()`, or with `use_fabric=False`. Physics-backed
   `.data` buffers are always correct.

## Architecture

`src/dishsim/` (single project package; `scripts/*` add `src/` to `sys.path`, the venv installs
it editable). Full module tree + layering: `docs/architecture.md`. Load-bearing traps:

- `config.py` machine selector — `apply_machine("bosch800")` swaps the world to the Bosch 800
  twin (self-authored USD, own cache root `assets/cache/machines/bosch800/`, own scenarios
  incl. `third_out`/`middle_out`, per-machine TASK/camera overrides);
  `apply_base_placement("side_winner")` selects the Bosch cache's base-frame anchor. Order
  matters: machine → object → scenario → placement, all BEFORE scene imports. The v1 baseline
  restores byte-stable. Every Bosch number traces to `docs/bosch800_source_data.md`
  (`estimated` rows = Stage-B calipers).
- **FROZEN CACHE ANCHORS** — `config.py` keeps robot-era constants (`HOME_Q`,
  `GRIPPER_APERTURE_GRASP_RAD`, `T_WRIST3_TCP_*`, `GRASP_TCP_OBJ_*` via
  `GraspSpec`/`grasp_transform`, `ROBOT_BASE_*`/`BASE_PLACEMENTS`) ONLY because they feed
  `geometry.config_hash()`, which keys every shipped cache in the public archive. Never tune
  them; a change silently invalidates every bake (≈2.5 h Kit to redo, and the archive stops
  validating). The base frame every cached coordinate is expressed in is the robot-era mount —
  that is why `ROBOT_BASE_*` survives as the world anchor.
- **`config_hash()` is NOT the only cache key.** `geometry.coacd_dir_for` digests mesh bytes +
  the body's `COACD` params, and `config.COACD` is absent from the `config_hash` payload. So a
  static-CoACD edit leaves every manifest validating green while pointing at a piece directory
  that does not exist yet — loud at load (`missing CoACD pieces`), but invisible to the
  staleness check. That is also the good news: re-decomposing a static is **Kit-free** and
  invalidates nothing (`scripts/setup/decompose_meshes.py`, seconds). Tiering, for tuning:
  hash-SAFE = `COLLISION_MARGIN_M`, `RELEASE_HOVER_M`, `TASK[...]`, `PLACEMENT_MODES`, cameras,
  everything in `rearrange.py`; static `COACD` = re-decompose only; hashed (full Kit rebake) =
  `RACK_GEN`/`MACHINE_GEN`, object `spec.coacd`, the frozen anchors above.
- **CoACD's manifold preprocess adds an isotropic skin** proportional to body size — measured
  `E_door_4` +4.09 mm, `E_body_5` +8.24 mm. The door's phantom volume alone FCL-blocked 5 of 8
  lower-rack plate gaps whose TRUE clearance is 7.3–7.8 mm. Watertight authored bodies take
  `"preprocess_mode": "off"` and decompose exactly (door: 0.000 mm overhang, 1.000x volume,
  0.3 s). Raising `preprocess_resolution` instead is strictly worse and already tried: 200 still
  left 1.02 mm, exploded 2 pieces into 45, and took 474 s. **When a slot or pose is FCL-blocked
  by a decomposed authored body, measure hull overhang against the source mesh before believing
  the geometry** — this bug family has bitten twice (counter buffer, then the door).
- `COLLISION_MARGIN_M` (2 mm) is a **pre-filter, not the certificate** — the Isaac settle gates
  are. At 5 mm it vetoed real clearances of 1.6–3.3 mm and halved certified capacity. Runtime
  only: it is not hashed, so it never invalidates a bake.
- `rearrange.BUFFER_EXTRA_HOVER_M` exists because the counter's CoACD hull tops ~8 mm above the
  raw slab (`E_body_5` still carries its skin — it is far more concave and did not converge
  unpreprocessed). Without it every counter-buffer cell is FCL-blocked and the greedy baseline
  gives up at move 0 with nowhere to relocate a blocker.
- `gen_instances.py` records a **reproduction gate**, not merely a settled pose: it rehearses
  the runner's own episode reset (teleport into the settled contact poses, re-settle, compare
  under `INIT_MATCH_*`) and stores the re-settled FIXED POINT, re-rolling arrangements that
  fail. A drop-from-hover bowl wedged 10–25° on a wire does not survive a cold teleport and
  would abort every episode as `init-mismatch`. Do not "simplify" this back to a single settle.
- `config.py` — every tunable, defined ONCE; calibrated values are measured, not eyeballed.
  Tune here, not inline. `geometry.config_hash()` keys the collision caches on config values
  (including the whole `RACK_GEN` dict) — a value change silently invalidates every bake.
- `collision_world.py` — Kit-free; no `pxr`/`isaaclab` imports here, ever. The hot path is
  `object_in_collision(pieces, T_base_obj)`: candidate convex pieces (margin-inflated) vs
  statics + every `add_object` obstacle.
- `placement.py` — slots derive live (deterministic given the hash-guarded cache); certify and
  record placements at the mode's RELEASE HOVER, never at zero hover — a resting object touches
  the wire floor it stands on, so the inflated hull at rest collides with its own support by
  construction. The settle pass drops the last few millimetres.
- `capacity.py` — placeable ≠ settled: the plan certifies collision-free RELEASE-HOVER poses
  (both sides of the joint check hover — a resting object touches its own support); the
  measured settle-reliability gates (`MEASURED_SETTLE_RELIABILITY`) and the z-budget
  closability gate are what keep the count honest. `capacity_fill.py` closes the remaining
  gap by physically seating the plan item-by-item — run it before quoting a capacity number.
  Evaluation aggregates plan/settle artifacts, never re-derives verdicts. Greedy assignment is
  NOT the limiter (measured: it equals the exact maximum independent set on the bowl grid), so
  when the count looks wrong suspect the placeability funnel, not the packer.
- `rearrange.py` — the benchmark core, Kit-free by an oracle/world seam: `run_episode` is
  driven by an oracle (`IsaacOracle` in `run_rearrange.py`; a toy oracle in the test) and the
  `ArrangementWorld` FCL mirror, so algorithms and the driver never import Kit. Episodes
  abort on the FIRST fault; commanded poses are release-hover poses (never zero hover — see
  the placement.py trap); at-goal is judged on the SETTLED pose via `evaluate_placement`.
  All fault/settle knobs are module constants here, deliberately outside `config.py` so they
  can never feed `config_hash`. New algorithms: implement `reset(instance, world)` /
  `next_move(obs)`, add one line to `ALGORITHMS` in `run_rearrange.py`.
- `usd_prep.py` — derived dishwasher USDs; downloaded originals are never modified.
- `machine.py` — dishwasher ArticulationCfgs; requires `apply_machine`/`apply_scenario`
  BEFORE import (the derived USD binds at import time).

Numeric provenance: every prim path, joint name, frame offset, and placement number is a
*measured* value recorded in `docs/joint_report.md` (generated by `scripts/setup/inspect_scene.py`)
and `docs/asset_survey.md`. If you change the dishwasher variant or scene layout, re-run the
inspection script and take the new numbers from the report — don't eyeball them. Two traps
encoded there: spawn poses place the articulation **root link** frame (for the dishwasher
that's `E_body_5`, not the asset origin), and this Isaac Lab is XYZW-quaternion /
`.torch`-ProxyArray throughout (2.x tutorial snippets are wrong on both counts).

## Ground rules

- `assets/`, `media/`, `results/`, `logs/` are gitignored; never commit them (asset sources +
  licenses in README.md). Curated report figures go to `docs/figures/` (tracked, with
  provenance in `docs/figures/README.md`).
- Media capture is **on-demand** (`--video`, needs `--enable_cameras`): instance/episode JSON
  records are the primary artifacts of a benchmark run; render PNG/MP4 under `media/<phase>/`
  when a result needs visual evidence (the user cannot watch the viewport). Roots in use:
  `media/rearrange/<machine>/<state>/`, `media/capacity/<machine>/[<state>/]`,
  `media/instances/<machine>/<state>/`.
- **Tint objects per item in any multi-object render** (`config.item_color(item_id)`, plumbed
  through the `"color"` key of a `scene._add_object` spec). Every sourced prop carries the same
  dark-red material, so an untinted 15-item load renders as one undifferentiated mass and the
  arrangement is unreadable. The colour follows the item id — not roster order, which differs
  between scripts — so an object keeps its colour across stills and video. Visual only: it
  never touches physics or collision geometry.
- The Bosch episode camera is a **top-down plan view** framing the counter buffer band and the
  extended lower rack together; `run_rearrange.py` reads `config.EPISODE_CAMERA` directly (NOT
  `TASK["video_camera"]`, which belongs to the retired robot-era runner).
- One frame convention everywhere, asserted in code: base frame, meters, Z-up, XYZW.
- The dishwasher base stays fixed (`fix_root_link=True`); the door stays locked open.
- Ask the user before: downloads over 2 GB, runs expected to exceed 30 minutes, opening ports,
  or installs that restructure the container (ROS/MoveIt especially). GUI verification happens
  only via the streaming client — pause and ask the user to connect.
- Suggested commit style (user commits): imperative ~50-char subject, wrapped body explaining
  what/why, one commit per phase, no AI attribution lines.
