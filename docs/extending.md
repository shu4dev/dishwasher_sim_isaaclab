# Extending the environment

The object/mode/state extension workflows.

## Add an object class

1. Add an `ObjectSpec` to `config.OBJECTS` — source (`ycb16k` / `procedural`), scale, mass,
   a `GraspSpec` (family + contact width) and a `PlacementSpec` (mode + rack). The `GraspSpec`
   no longer drives anything, but a new class still needs one: its values feed the frozen
   cache-anchor hash (`geometry.config_hash()`).
2. Build the asset. The one-time authoring pipeline (`build_object_assets.py`, retired with
   the public-asset release) lives in git history — restore it from there for new classes.
   It downloads/generates the source mesh, prints the *measured* dimensions and fails if they
   disagree with the registry by >2 mm — freeze the printed block into the spec.
3. Rebuild that object's caches: `extract_geometry` → `decompose_meshes` (or one
   `setup/build_state.py --state <state> --classes <name>`). Slots derive live from the
   cached rack geometry — check the class's placeability with `setup/derive_slots.py`.

> **Note:** never eyeball-edit a measured value. Every dimension in `config.py` traces to a
> calibration or inspection run, and `geometry.config_hash()` invalidates the collision caches
> when any of them changes — the robot-era constants in the hash are FROZEN CACHE ANCHORS,
> kept untouched so every shipped cache stays valid.

## Add a placement mode

In `src/dishsim/placement.py`: write a `derive_<mode>_slots()` returning `SlotFrame`s, add a
branch to `object_pose_for_mode()` for the release-pose geometry, and a branch to
`evaluate_placement()` for the success criteria. Register the mode name in the object's
`PlacementSpec`, and document the criteria in `docs/success_criteria.md`.

## Add a machine state

Add an entry to `config.INTERNAL_STATES` (rack extensions + `min_feasible_slots`), then rebuild
the cache for it: `extract_geometry --scenario <name>` → `decompose_meshes --scenario <name>`
(or one `setup/build_state.py --state <name> --classes ...`).
