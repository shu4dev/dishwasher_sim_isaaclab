# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Isaac Sim **environments** for dishwasher loading — the substrate for classical motion
planning (shipped), imitation learning, and RL policies. A UR5e + Robotiq 2F-85 loads a
15-class kitchen-object library (per-object grasp/placement specs in `config.OBJECTS`) into
an articulated ArtVIP dishwasher (door locked open at 90°, procedural realistic racks + a
3-bay cutlery basket). The shipped classical stack: OMPL (RRT-Connect, 6-D joint space)
plans against a standalone Kit-free FCL collision world
(`src/dishsim/collision_world.py`, built for thousands of external-planner queries); the
carried object rides a calibrated contact pinch with a hidden wrist weld bearing the load;
placement stability is verified per mode. `scripts/setup/capacity_fill.py` physically settles
a full 30-item load — initial states for rearrangement/IL/RL. The old RL door-opening
pipeline lives on the `archive/rl-door-opening` branch.

Two runners, deliberately separate. `run_trials.py` is the frozen single-object baseline (one
object, one slot, per trial) and is held at **zero diff** — it anchors the v0 mug result.
`run_task.py` runs multi-object EPISODES over the layered task stack in `src/dishsim/task/`
(sequencer → pick-and-place → motion → planner). An episode can start from a stowed machine
(`--scenario both_in`), in which case the robot pulls the lower rack out before loading it.

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
#   setup/      kit_smoke, inspect_scene, prepare_dishwasher_usd, make_prop_physics_usd,
#               build_object_assets, check_scene, calibrate_grasp, freeze_calibration,
#               extract_geometry, decompose_meshes, parity_check, goal_configs,
#               preview_rack, capacity_fill, build_state, reach_map
#   experiment/ run_trials (single object, FROZEN), run_task (episodes)
#               --planner selects the algorithm for both
#   evaluation/ compute_metrics, render_videos, verify_replay, compare_videos, plan_visual
#   tools/      archive_assets, restore_assets
```

**`./isaaclab.sh -p` exits 0 even when the wrapped script crashes.** Always verify success from
log content (`[RESULT] PASS`, absence of tracebacks / `free(): invalid pointer`), never from the
exit code.

## Launcher landmines (why the scripts look the way they do)

Full write-up in `docs/environment.md`; violating these produces native crashes or silent
import shadowing, not clean errors:

1. **Boot-first**: every Kit entry script launches `AppLauncher` *before* importing `dishsim`,
   `isaaclab.*` scene modules, or `pxr`. Standalone scripts construct
   `SimulationContext(sim_utils.SimulationCfg(..., physics=PhysxCfg()))` directly — no gym, no
   PresetCfg machinery.
2. **The package is `dishsim`, deliberately not matching the repo directory name** — Kit's
   extension scan turns a same-named directory into a namespace package that shadows the real
   one. Keep module-scope `pxr`/`omni` imports out of the package (lazy in-function imports,
   see `src/dishsim/usd_prep.py`).
3. **Isaac Lab 3.0 API**: quaternions are **XYZW** everywhere; data buffers are `ProxyArray` —
   append `.torch` (and `.torch.clone()`, since tensor methods aren't forwarded); kinematic
   writes use the keyword-only `*_index` methods (`write_joint_position_to_sim_index`, ...).
4. **Fabric staleness**: `use_fabric=True` (default) means live-stage prim transforms are stale
   mid-sim. Extract geometry from file stages pre-boot, or right after `sim.reset()`, or run
   extraction/parity scripts with `use_fabric=False`. Physics-backed `.data` buffers are always
   correct.

## Architecture

`src/dishsim/` (single project package; `scripts/*` add `src/` to `sys.path`, the venv installs
it editable):

- `robots.py` — `UR5E_ROBOTIQ_2F_85_CFG` + `DISHWASHER_CFG` (+ `DISHWASHER_V0_CFG` from Phase
  C). Load-bearing details: `articulation_root_prim_path="/root_joint"` (the asset has a second,
  disabled ArticulationRootAPI on the gripper subtree); gripper armature 0.001 + damping 0.05
  (the near-massless mimic-joint finger cluster resonates and explodes without them). Only
  `finger_joint` is ever *commanded to a pose* on the gripper (0 = open, ~0.8 = closed —
  inverted vs. Franka), and only between two calibrated apertures: fully open at trial
  endpoints, the contact-pinch aperture (`docs/grasp_calibration.md`) during all planned
  motion — matching the frozen-aperture FCL cluster. The mimic-driven joints are never
  position-written; the two stiff `.*_inner_finger_joint` drive *targets* are additionally
  kept mimic-consistent by `scene.hold_targets` (signs measured by
  `scripts/setup/calibrate_grasp.py`) so they don't fight the mimic constraint; the three
  zero-stiffness knuckle joints stay untouched always.
- `usd_prep.py` — derived dishwasher USDs. Removes ArtVIP's world-weld `FixedJoint` (body1 set,
  no body0 — pins the machine at its authored pose and blows up any relocated spawn).
  `make_dishwasher_rl_usd` also zeroes the authored door drive (passive door, used by the
  inspection script); `make_dishwasher_v0_usd` is the static variant (door locked open). Downloaded
  originals are never modified.
- `config.py` — every tunable: grasp transform (defined ONCE), calibrated grasp
  aperture + pad-force bands (measured by `scripts/setup/calibrate_grasp.py`, not eyeballed),
  slot tolerances, CoACD params, collision margin, plan budgets, camera poses. Tune here, not
  inline.
- `collision_world.py` — Kit-free FCL world loaded from `assets/cache/` +
  `scene_state.json` manifest (frames asserted at load). No `pxr`/`isaaclab` imports here, ever.
- `planners/` — the pluggable planner layer. `Planner.plan(world, start, goals, seed, debug)`
  is the whole interface; `OMPLPlanner` holds the shared query machinery and subclasses
  override only `_make_planner(si)`. World and seed are per-CALL because one trial plans
  against three different collision worlds. Registered: rrt_connect (default), rrt_star,
  bit_star, prm. `prm` sets `supports_multi_goal = False` — measured: the roadmap planners in
  this OMPL build never terminate on a multi-state `ob.GoalStates`.
- `task/` — the multi-object task layer, and the one boundary this repo enforces mechanically
  (`tests/test_layer_boundary.py`, AST-based): `sequencer.py` decides WHICH object next,
  `primitives.py` runs one object's choreography, `motion.py` is object-agnostic "move A to B",
  and only then the planner. **No task concept may reach `planners/`.** `motion.ExecContext` is
  a Protocol the runner implements after Kit boots — a conformance test compares it to `_Ctx`
  signature-for-signature, which is how a protocol method added without its implementation gets
  caught. Also here: `rack.py` (open the machine — engage a handle, slide a rack, using
  `rack_ops.py` geometry), `layout.py`, `support.py`, `grasp.py`, `recovery.py`, `cost.py`,
  `episode.py`. Four traps encoded in their docstrings:
  1. **Episodes are home-anchored** — every recording starts and ends at `config.HOME_Q`. The
     start is measured, corrected and asserted; the closing retreat runs before `rec.end` and
     the media finish (so it lands in the artifacts) and after the sequencer (so it can never
     revise a placement verdict).
  2. **A rack drive override stays latched** after a successful action and accumulates across a
     sequence. The scene's standing targets come from the PRE-action scenario, so releasing it
     drives the rack back in; replacing rather than merging the dict lets an earlier rack shut.
  3. **An episode with a rack action spans two states** — the rack phase plans in the start
     state (scenario-level mug cache), every pick in the derived `POST_STATE` (per-class
     caches). A rack that settles beyond `RACK_SLIDE_TOL_M` ends the episode: every later plan
     assumes it arrived.
  4. **Weld-acquire ≠ pick.** `config.WELD_ACQUIRE_FAMILIES` classes are snapped to the carry
     transform at the hover, never descended to; records label them `acquired: "weld"` and the
     two success rates must never be summed. Their reachability and grasp gates stop at the
     hover to match — gating on the descent rejected 344 of 400 fork layout draws.
- `trajectory.py` / `replay.py` — the Phase 2 → Phase 3 handoff. Experiments record measured
  state every physics step; evaluation replays it kinematically. Three traps are encoded in
  `replay.py`'s docstring and must not be "simplified" away: (1) `sim.forward()` updates
  physics but NOT the renderer, so playback must `sim.step()`; (2) without
  `scene.write_default_states` the dishwasher renders ~25 cm off its spawn pose; (3) drive
  targets must be set to the recorded pose or the PD controllers drag links ~21 mm per frame.
- `metrics.py` — Kit-free aggregation over trial JSONs. Contact-derived verdicts (success)
  stay in Phase 2; evaluation aggregates, never re-derives them.

Numeric provenance: every prim path, joint name, frame offset, and placement number is a
*measured* value recorded in `docs/joint_report.md` (generated by `scripts/setup/inspect_scene.py`)
and `docs/asset_survey.md`. If you change the dishwasher variant, robot home pose, or scene
layout, re-run the inspection script and take the new numbers from the report — don't eyeball
them. Two traps encoded there: spawn poses place the articulation **root link** frame (for the
dishwasher that's `E_body_5`, not the asset origin), and this Isaac Lab is XYZW-quaternion /
`.torch`-ProxyArray throughout (2.x tutorial snippets are wrong on both counts).

## Ground rules

- `assets/`, `media/`, `results/`, `logs/` are gitignored; never commit them (asset sources +
  licenses in README.md). Curated report figures go to `docs/figures/` (tracked).
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
