# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Isaac Sim **environments** for the dishwasher **arrangement planning problem**: decide where
each object of a 14-class kitchen-object library goes in an articulated dishwasher (ArtVIP
baseline with procedural RACK_GEN v4 racks, or the self-authored Bosch 800 twin). Feasible =
**collision-free** (Kit-free FCL pose query, `CollisionWorld.object_in_collision`) +
**physically stable** (Isaac settle validation). Object motion is **teleportation** — there is
no robot arm, no grasping, and no motion planning on this branch (that stack lives in git
history / `main`; the old RL door-opening pipeline on `archive/rl-door-opening`). Isaac Sim is
the physics validator and evidence renderer only; planning runs in the plain venv.

The pipeline is a **rearrangement benchmark**: bake a machine state's collision caches
(`build_state.py`) → generate settled problem instances (`gen_instances.py`: exact target
poses from the capacity plan, seeded initial arrangements, saved JSON artifacts) → run
algorithms closed-loop (`run_rearrange.py`: one persistent Kit session; every move teleports
one object, settles, and ABORTS the episode on the first fault — colliding command, unstable
settle, disturbed neighbor — under a move budget) → render evidence on demand
(`reveal_render.py`, `--video`). The benchmark core is `dishsim/rearrange.py` (Kit-free
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
# planning-stack tests (venv, no Kit; plugin autoload off — hydra's plugin breaks outside Kit)
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

# bake a machine state's collision caches (extract -> decompose; per class)
scripts/setup/build_state.py --state placement --classes mug,cup,tumbler
scripts/setup/build_state.py --machine bosch800 --placement side_winner --state placement --classes cup

# inspect slots Kit-free (table + placeability + slot_detection.png)
/workspace/isaaclab/env_isaaclab/bin/python scripts/setup/derive_slots.py --object cup --scenario placement

# scene inspection: regenerates docs/joint_report.md, stability + passive-door tests
scripts/run_kit.sh scripts/setup/inspect_scene.py --headless --test_door

# entry points: scripts/ is split by phase (see README Usage)
#   setup/      kit_smoke, inspect_scene, extract_geometry, decompose_meshes, build_state,
#               derive_slots, preview_rack, gen_instances, plan_full_load
#   experiment/ run_rearrange
#   evaluation/ reveal_render
#   tools/      archive_assets, restore_assets, bootstrap.sh
# Caches ship in the public archive — restore first (tools/restore_assets.py), never rebake
# what the archive already carries.
```

**`./isaaclab.sh -p` exits 0 even when the wrapped script crashes.** Always verify success from
log content (`[RESULT] PASS`, absence of tracebacks / `free(): invalid pointer`), never from the
exit code.

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
- `capacity.py` — placeable ≠ settled: the plan certifies collision-free release poses; the
  measured settle-reliability gates (`MEASURED_SETTLE_RELIABILITY`) and the z-budget
  closability gate are what keep the count honest. Evaluation aggregates plan/settle
  artifacts, never re-derives verdicts.
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
  when a result needs visual evidence (the user cannot watch the viewport).
- One frame convention everywhere, asserted in code: base frame, meters, Z-up, XYZW.
- The dishwasher base stays fixed (`fix_root_link=True`); the door stays locked open.
- Ask the user before: downloads over 2 GB, runs expected to exceed 30 minutes, opening ports,
  or installs that restructure the container (ROS/MoveIt especially). GUI verification happens
  only via the streaming client — pause and ask the user to connect.
- Suggested commit style (user commits): imperative ~50-char subject, wrapped body explaining
  what/why, one commit per phase, no AI attribution lines.
