# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A **physics-validated rearrangement planning benchmark**: given a settled initial arrangement
and an exact target arrangement of kitchen objects in an articulated dishwasher (the
self-authored Bosch 800 twin; the ArtVIP baseline ships too), an algorithm moves one object at
a time by **teleportation** and Isaac settles every move. Feasible = **collision-free**
(Kit-free FCL pose query, `CollisionWorld.object_in_collision`) + **physically stable** (Isaac
settle validation). There is no robot arm, no grasping, no motion planning on this branch
(that stack lives in git history / other branches). `dishsim/compat.py` computes the provably
minimum move count per instance, so results are quoted as an optimality gap.

The minimal pipeline, in order (this is the whole repo):

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
never rebake what the archive carries. ONE exception until the archive is re-cut: the shipped
E_door_4 CoACD pieces predate `COACD["E_door_4"]["preprocess_mode"] = "off"`, so a restore-only
box runs `decompose_meshes.py` once per context (Kit-free, seconds) — pass
`--machine/--placement/--scenario/--object` matching the restore log's `[OK] ... @ <placement>`
line; a wrong anchor mis-reports "cache is stale". See docs/known_limitations.md.

**`./isaaclab.sh -p` exits 0 even when the wrapped script crashes.** Judge every Kit run from
log content (`[RESULT] PASS`, absence of tracebacks / `free(): invalid pointer`), never the
exit code.

**Every standalone Kit script calls `dishsim.media.release_sim_for_close()` before
`simulation_app.close()`.** Without it, isaaclab 2.1's on-stop callback spins the shutdown
forever at full CPU, and Kit's fast-exit discards block-buffered stdout — including the
`[RESULT]` line the previous rule depends on. New Kit scripts must keep this pattern.

## Validation status

**`placement` is green on this box** (2026-08-30, corallab / Isaac 4.5 port, bosch800 @
side_winner): pytest 11/11 → restore PASS → kit_smoke → capacity 39/15 → gen_instances →
instance_views → run_rearrange all `[RESULT] PASS`; greedy solved 3/3 perturbed 15-item
instances in 9 moves of 45 (provable optima 9/8/8 — `compat.optimal_moves`). Instances are
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
  counted); 25 straight refusals abort `refusal-loop`. `unstable-settle` is NON-fatal
  (oracle teleports the item back; move counts as `failed-settle`); `disturbed` stays
  fatal. The optimality gap in `compare_algorithms.py` reads each instance's cap-aware
  `meta.optimum` (computed at generation; never re-solved).
- Every number is a *measured* value (`docs/joint_report.md`, `docs/bosch800_source_data.md`)
  — never eyeball-edit. Spawn poses place the articulation **root link** (`E_body_5`, not the
  asset origin).

## Ground rules

- `assets/`, `media/`, `results/`, `logs/`, `outputs/` are gitignored; never commit them.
  Curated figures go to `docs/figures/` (tracked, provenance in its README).
- Media is on-demand (`--video` needs `--enable_cameras`); JSON records are the primary
  artifacts. **Tint objects per item in any multi-object render** (`config.item_color`,
  plumbed via the `"color"` spec key) — untinted loads render as one dark-red mass. The
  episode camera is `config.EPISODE_CAMERA`.
- One frame convention everywhere, asserted in code: base frame, meters, Z-up, XYZW.
- The dishwasher base stays fixed (`fix_root_link=True`); the door stays locked open.
- Ask the user before: downloads over 2 GB, runs expected to exceed 30 minutes, opening
  ports, or installs that restructure the container.
