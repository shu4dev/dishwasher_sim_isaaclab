# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Isaac Sim **environments** for dishwasher loading — the substrate for classical motion
planning (shipped), imitation learning, and RL policies. A UR5e + Robotiq 2F-85 loads a
14-class kitchen-object library (per-object grasp/placement specs in `config.OBJECTS`) into
an articulated ArtVIP dishwasher (door locked open at 90°, procedural RACK_GEN v4 racks + a
3-bay cutlery basket). OMPL (RRT-Connect, 6-D joint space) plans against a standalone Kit-free
FCL collision world (`src/dishsim/collision_world.py`); the carried object rides a calibrated
contact pinch with a hidden wrist weld bearing the load; placement stability is verified per
mode. `scripts/setup/capacity_fill.py` physically settles a full 30-item load. The old RL
door-opening pipeline lives on the `archive/rl-door-opening` branch. See the README for the
full overview and `docs/architecture.md` for the code tree.

Two runners, deliberately separate. `run_trials.py` is the frozen single-object baseline (one
object, one slot, per trial) and is held at **zero diff** — it anchors the v0 mug result.
`run_task.py` runs multi-object EPISODES over the layered task stack in `src/dishsim/task/`
(sequencer → pick-and-place → motion → planner); see `docs/episodes.md`.

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
# scene inspection: regenerates docs/joint_report.md, stability + passive-door tests
scripts/run_kit.sh scripts/setup/inspect_scene.py --headless --test_door

# derive the physics-enabled YCB plate (mug fallback: --object 025_mug)
scripts/run_kit.sh scripts/setup/make_prop_physics_usd.py --object 029_plate

# planning-stack tests (venv, no Kit; plugin autoload off — hydra's plugin breaks outside Kit)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest tests/

# one multi-object episode from a stowed machine (robot pulls the lower rack out first)
scripts/run_kit.sh scripts/experiment/run_task.py --headless --enable_cameras \
    --scenario both_in --spawn "cup=1,tumbler=1,fork=2" --seed 1 --run_id bothin_load

# bake a machine state's collision caches (what run_task.py prints when a cache is missing)
scripts/setup/build_state.py --state placement --classes mug,cup,tumbler

# entry points: scripts/ is split by phase (see README Usage)
#   setup/      kit_smoke, inspect_scene, make_prop_physics_usd, build_object_assets,
#               check_scene, calibrate_grasp, freeze_calibration, extract_geometry,
#               decompose_meshes, parity_check, goal_configs, preview_rack, capacity_fill,
#               build_state, reach_map, base_pose_sweep (completed study)
#   experiment/ run_trials (single object, FROZEN), run_task (episodes)
#               --planner selects the algorithm for both
#   evaluation/ compute_metrics, render_videos, verify_replay, plan_visual
#   tools/      archive_assets, restore_assets
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
   one. Keep module-scope `pxr`/`omni` imports out of the package.
3. **Isaac Lab 3.0 API**: quaternions are **XYZW** everywhere; data buffers are `ProxyArray` —
   append `.torch` (and `.torch.clone()`); kinematic writes use the keyword-only `*_index`
   methods.
4. **Fabric staleness**: live-stage prim transforms are stale mid-sim. Extract geometry from
   file stages pre-boot, right after `sim.reset()`, or with `use_fabric=False`. Physics-backed
   `.data` buffers are always correct.

## Architecture

`src/dishsim/` (single project package; `scripts/*` add `src/` to `sys.path`, the venv installs
it editable). Full module tree: `docs/architecture.md`. Load-bearing traps, encoded in the
modules' docstrings and not to be "simplified" away:

- `robots.py` — `articulation_root_prim_path="/root_joint"` (the asset has a second, disabled
  ArticulationRootAPI on the gripper subtree); gripper armature 0.001 + damping 0.05 (the
  near-massless mimic-joint finger cluster resonates and explodes without them). Only
  `finger_joint` is ever *commanded to a pose* (0 = open, ~0.8 = closed — inverted vs. Franka),
  and only between two calibrated apertures; the two stiff `.*_inner_finger_joint` drive
  *targets* are kept mimic-consistent by `scene.hold_targets`; the three zero-stiffness
  knuckle joints stay untouched always.
- `usd_prep.py` — derived dishwasher USDs; downloaded originals are never modified.
- `config.py` — every tunable, defined ONCE; calibrated values are measured, not eyeballed.
  Tune here, not inline. `geometry.config_hash()` keys the collision caches on config values
  (including the whole `RACK_GEN` dict) — a value change silently invalidates every bake.
- `collision_world.py` — Kit-free; no `pxr`/`isaaclab` imports here, ever.
- `planners/` — `Planner.plan(world, start, goals, seed, debug)` is the whole interface;
  world and seed are per-CALL. `prm` sets `supports_multi_goal = False` (measured).
- `task/` — the one boundary this repo enforces mechanically (`tests/test_layer_boundary.py`,
  AST-based): sequencer decides WHICH object, primitives run one object's choreography,
  `motion.py` is object-agnostic, and only then the planner. **No task concept may reach
  `planners/`.** `motion.ExecContext` is a Protocol the runner implements after Kit boots.
  Four traps: (1) episodes are home-anchored (start measured + asserted; closing retreat runs
  before `rec.end` and after the sequencer); (2) a rack drive override stays latched after a
  successful action and accumulates — replace, don't merge, and never release mid-sequence;
  (3) an episode with a rack action spans two collision-cache states; a rack settling beyond
  `RACK_SLIDE_TOL_M` ends the episode; (4) weld-acquire ≠ pick — `WELD_ACQUIRE_FAMILIES`
  classes snap to the carry transform at the hover, records label them `acquired: "weld"`,
  and the two success rates must never be summed.
- `trajectory.py` / `replay.py` — the Phase 2 → Phase 3 handoff. Three traps in `replay.py`'s
  docstring: (1) `sim.forward()` updates physics but NOT the renderer — playback must
  `sim.step()`; (2) without `scene.write_default_states` the dishwasher renders ~25 cm off;
  (3) drive targets must be set to the recorded pose or the PD controllers drag links ~21 mm
  per frame.
- `metrics.py` — contact-derived verdicts (success) stay in Phase 2; evaluation aggregates,
  never re-derives them.

Numeric provenance: every prim path, joint name, frame offset, and placement number is a
*measured* value recorded in `docs/joint_report.md` (generated by `scripts/setup/inspect_scene.py`)
and `docs/asset_survey.md`. If you change the dishwasher variant, robot home pose, or scene
layout, re-run the inspection script and take the new numbers from the report — don't eyeball
them. Two traps encoded there: spawn poses place the articulation **root link** frame (for the
dishwasher that's `E_body_5`, not the asset origin), and this Isaac Lab is XYZW-quaternion /
`.torch`-ProxyArray throughout (2.x tutorial snippets are wrong on both counts).

## Ground rules

- `assets/`, `media/`, `results/`, `logs/` are gitignored; never commit them (asset sources +
  licenses in README.md). Curated report figures go to `docs/figures/` (tracked, with
  provenance in `docs/figures/README.md`).
- **Every phase that produces a result inside Isaac Sim must also produce PNG/MP4 evidence**
  under `media/<phase>/` (the user cannot watch the viewport). Headless capture =
  `--headless --enable_cameras`; fixed front/top/iso cameras from `config.py`; short 720p clips.
- One frame convention everywhere, asserted in code: robot base frame, meters, Z-up, XYZW.
- The dishwasher base stays fixed (`fix_root_link=True`); the v0 door stays locked open; the
  carried object stays welded to the TCP until the release step (the visible pinch is real —
  calibrated pad contact — but the weld carries the load during motion; jaws open on camera
  before the weld drops).
- Ask the user before: downloads over 2 GB, runs expected to exceed 30 minutes, opening ports,
  or installs that restructure the container (ROS/MoveIt especially). GUI verification happens
  only via the streaming client — pause and ask the user to connect.
- Suggested commit style (user commits): imperative ~50-char subject, wrapped body explaining
  what/why, one commit per phase, no AI attribution lines.
