# initialization

Everything that must exist before a planner is asked anything: the problem instances (settled start states and
goals), plus the Frigidaire demo, start-state gate and random-load farm. Since 2026-10-21 the appliance twins,
dish assets and collision caches, and the scripts that generate and certify them, ship in the asset dataset
(`data/assets`, a copy of the HF dataset `shu4dev/dishsim-assets`: `hf download` to restore, `hf upload` to
publish; each folder's `source/` holds its generator and a README with the exact command).

- Input: the asset dataset downloaded to `data/assets`, the hashed config in `code/src/dishsim/config.py`, the
  Frigidaire geometry in `data/assets/frigidaire_fdpc4221as/parameters.json`.
- Output: Bosch instances under `data/results/instances/<machine>/<state>/`.
- HOTEC instances (Frigidaire) are made by `code/planner/frigidaire/frigidaire_bench.py --generate`: the bench
  files stay whole in `planner/`.

Folders: [bosch/](bosch/README.md), [frigidaire/](frigidaire/README.md).

## Commands, in order

Bosch. The dataset carries the caches; extract and decompose (in `data/assets/bosch800/source/`) only rebake one
(object, state) cache after a hashed-config change.

```bash
hf download shu4dev/dishsim-assets --repo-type dataset --local-dir data/assets
code/util/run_kit.sh data/assets/bosch800/source/extract_geometry.py --headless --machine bosch800 --placement side_winner --scenario placement --object cup
code/util/run_py.sh data/assets/bosch800/source/decompose_meshes.py --machine bosch800 --placement side_winner --scenario placement --object cup
code/util/run_kit.sh code/initialization/bosch/gen_instances.py --headless --mode perturbed --state placement --n 3 --seed 0
code/util/run_kit.sh code/initialization/bosch/instance_views.py --headless --enable_cameras --instance data/results/instances/bosch800/placement/perturbed_s0.json
```

Frigidaire twin: the stage, build, inspect, evidence and package commands moved with their scripts to
`data/assets/frigidaire_fdpc4221as/source/` (its README) on 2026-10-21.

HOTEC instances (the dish assets' generator is in `data/assets/hotec_wheatstraw/source/`):

```bash
python3 code/planner/frigidaire/frigidaire_bench.py --generate --tiers easy medium hard --seeds 0 --kit-jobs 3 --py-jobs 2
```

## Package modules

First needed here; a later stage that imports a module is named beside it.

- `dishsim`: `machine`, `scene` (planner), `geometry`, `rack_layout` (planner), `collision_world` (planner),
  `placement` (planner), `slotting`, `capacity` (planner), `instance_gen`, `tiers`.
- `dishsim_frigidaire`: `geometry` (planner), `asset` (planner), `tableware` (planner),
  `loading` (planner, execution), `load_validation`, `random_poses` (planner, execution),
  `random_pose_experiment` (execution), `random_pose_runtime` (planner, execution),
  `initial_state_candidates` (planner, execution), `initial_state_runtime` (planner, execution).
