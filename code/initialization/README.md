# initialization

Everything that must exist before a planner is asked anything: the Frigidaire demo, start-state gate and
random-load farm. Since 2026-10-21 the appliance twin, the dish assets, and the scripts that generate and certify
them ship in the asset dataset (`data/assets`, a copy of the HF dataset `shu4dev/dishsim-assets`: `hf download` to
restore, `hf upload` to publish; each folder's `source/` holds its generator and a README with the exact command).
The benchmark's pairs (messy start + goal) are made by `code/planner/bench.py`; its start generator is
`code/planner/frigidaire/frigidaire_bench_kit.py --start`.

- Input: the asset dataset downloaded to `data/assets`, the Frigidaire geometry in
  `data/assets/frigidaire_fdpc4221as/parameters.json`.
- Output: the empty-machine baseline (`frigidaire_initial_state_validate.py --out-dir
  data/results/benchmark/frigidaire_hotec/baseline`), gate records, farm loads.

Folder: [frigidaire/](frigidaire/README.md).

## Commands, in order

```bash
hf download shu4dev/dishsim-assets --repo-type dataset --local-dir data/assets
code/util/run_kit.sh code/initialization/frigidaire/frigidaire_initial_state_validate.py --out-dir data/results/benchmark/frigidaire_hotec/baseline --headless
code/util/run_kit.sh code/initialization/frigidaire/frigidaire_retract_check.py --headless   # that baseline's racks retracted with the gate's ramp, both orders: peak speed + stalls, [RESULT] PASS retract check
```

Frigidaire twin: the stage, build, inspect, evidence and package commands moved with their scripts to
`data/assets/frigidaire_fdpc4221as/source/` (its README) on 2026-10-21; the dish assets' generator is in
`data/assets/hotec_wheatstraw/source/`.

## Package modules

First needed here; a later stage that imports a module is named beside it.

- `dishsim`: `app` (every Kit script), `media` (planner, execution), `quats`, `transforms`, `checks`,
  `collision_world` and `geometry` (a cache-based FCL world; no shipped machine carries a cache today).
- `dishsim_frigidaire`: `geometry` (planner), `asset` (planner), `tableware` (planner),
  `loading` (planner, execution), `load_validation`, `random_poses` (planner, execution),
  `random_pose_experiment` (execution), `random_pose_runtime` (planner, execution),
  `initial_state_candidates` (planner, execution), `initial_state_runtime` (planner, execution).
