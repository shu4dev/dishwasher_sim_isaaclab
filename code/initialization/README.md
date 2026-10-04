# initialization

Everything that must exist before a planner is asked anything: the appliance twins and dish assets, collision
caches, the evidence that certifies a twin, and the problem instances (settled start states and goals).

- Input: the public archive (restore it first with `code/util/tools/bootstrap.sh`), the hashed config in
  `code/src/dishsim/config.py`, the Frigidaire geometry in `code/frigidaire/src/dishsim_frigidaire/geometry.py`.
- Output: Bosch collision caches under `data/assets/cache/` and instances under
  `data/results/instances/<machine>/<state>/`; the Frigidaire build and its evidence under
  `data/build/frigidaire_collection/`; HOTEC dish assets under `data/assets/models/hotec_wheatstraw/`.
- HOTEC instances (Frigidaire) are made by `code/planner/frigidaire/frigidaire_bench.py --generate`: the bench
  files stay whole in `planner/`.

Folders: [bosch/](bosch/README.md), [frigidaire/](frigidaire/README.md).

## Commands, in order

Bosch. The archive carries the caches; extract and decompose only rebake one (object, state) cache after a
hashed-config change.

```bash
code/util/run_kit.sh code/initialization/bosch/extract_geometry.py --headless --machine bosch800 --placement side_winner --scenario placement --object cup
code/util/run_py.sh code/initialization/bosch/decompose_meshes.py --machine bosch800 --placement side_winner --scenario placement --object cup
code/util/run_kit.sh code/initialization/bosch/gen_instances.py --headless --mode perturbed --state placement --n 3 --seed 0
code/util/run_kit.sh code/initialization/bosch/instance_views.py --headless --enable_cameras --instance data/results/instances/bosch800/placement/perturbed_s0.json
```

Frigidaire twin (host staging needs NumPy, Matplotlib, Pillow and `pdftoppm`; one Kit job at a time):

```bash
python3 code/initialization/frigidaire/stage_frigidaire_collection.py --out-dir data/build/frigidaire_collection
code/util/run_py.sh code/initialization/frigidaire/build_frigidaire.py --out-dir data/build/frigidaire_collection/usd
code/util/run_py.sh code/initialization/frigidaire/frigidaire_collection_inspect.py --collection-dir data/build/frigidaire_collection
code/util/run_kit.sh code/initialization/frigidaire/frigidaire_asset_evidence.py --headless --device cpu --assembly-only --physics-only --usd data/build/frigidaire_collection/usd/fdpc4221as.usdc --out-dir data/build/frigidaire_collection/images/assembly
code/util/run_kit.sh code/initialization/frigidaire/frigidaire_asset_evidence.py --headless --device cpu --assembly-only --enable_cameras --render-only --usd data/build/frigidaire_collection/usd/fdpc4221as.usdc --out-dir data/build/frigidaire_collection/images/assembly
python3 code/initialization/frigidaire/package_frigidaire.py --collection-dir data/build/frigidaire_collection --check
```

HOTEC dish assets and instances:

```bash
code/util/run_py.sh -m dishsim.hotec_gen --out data/assets/models/hotec_wheatstraw/v1
python3 code/planner/frigidaire/frigidaire_bench.py --generate --tiers easy medium hard --seeds 0 --kit-jobs 3 --py-jobs 2
```

## Package modules

First needed here; a later stage that imports a module is named beside it.

- `dishsim`: `machine`, `scene` (planner), `usd_prep`, `rack_gen`, `prop_gen`, `hotec_gen`, `geometry`,
  `collision_world` (planner), `placement` (planner), `slotting`, `capacity` (planner), `instance_gen`, `tiers`.
- `dishsim_frigidaire`: `geometry` (planner), `asset` (planner), `tableware` (planner), `lower_rack_asset`,
  `loading` (planner, execution), `claims` (planner), `load_validation`, `random_poses` (planner, execution),
  `random_pose_assets`, `random_pose_experiment` (execution), `random_pose_runtime` (planner, execution),
  `initial_state_candidates` (planner, execution), `initial_state_runtime` (planner, execution).
