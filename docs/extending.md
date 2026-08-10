# Extending the environment

Adding a motion planner is covered in the README (it is the headline extension point); this
page covers the object/mode/state extension workflows.

## Add an object class

1. Add an `ObjectSpec` to `config.OBJECTS` — source (`ycb16k` / `procedural`), scale, mass,
   a `GraspSpec` (family + contact width) and a `PlacementSpec` (mode + rack).
2. Build the asset. The one-time authoring pipeline (`build_object_assets.py`, retired with
   the public-asset release) lives in git history — restore it from there for new classes.
   It downloads/generates the source mesh, prints the *measured* dimensions and fails if they
   disagree with the registry by >2 mm — freeze the printed block into the spec.
3. Measure the pinch: `setup/check_scene.py --measure` for the pad map, then
   `setup/calibrate_grasp.py --object <name>`.
4. Freeze the result: `setup/freeze_calibration.py --object <name>`.
5. Rebuild that object's caches: `extract_geometry` → `decompose_meshes` → `goal_configs`
   (or one `setup/build_state.py --state <state> --classes <name>`).

> **Note:** never eyeball-edit a measured value. Every dimension, aperture and force band in
> `config.py` traces to a calibration or inspection run, and `geometry.config_hash()`
> invalidates the collision caches when any of them changes.

## Add a placement mode

In `src/dishsim/placement.py`: write a `derive_<mode>_slots()` returning `SlotFrame`s, add a
branch to `object_pose_for_mode()` for the goal-pose geometry, and a branch to
`evaluate_placement()` for the success criteria. Register the mode name in the object's
`PlacementSpec`, and document the criteria in `docs/success_criteria.md`.

## Add a machine state

Add an entry to `config.INTERNAL_STATES` (rack extensions + `min_feasible_slots`), then rebuild
the cache for it: `extract_geometry --scenario <name>` → `decompose_meshes --scenario <name>`
→ `parity_check --scenario <name>` → `goal_configs --scenario <name>` (or one
`setup/build_state.py --state <name> --classes ...`).
