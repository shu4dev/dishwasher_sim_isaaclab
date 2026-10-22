# initialization/bosch

Problem instances for the Bosch 800 twin. Defaults are defined in the script itself unless another file is named.
Kit = run through `code/util/run_kit.sh ... --headless`. The collision-cache scripts (`extract_geometry.py`,
`decompose_meshes.py`) and the standalone asset's `bosch800_asset_evidence.py` moved to
`data/assets/bosch800/source/` on 2026-10-21 (its README gives the exact commands); the caches they write ship in
`data/assets/<machine>/cache/<state>/<object>/`.

| Script | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `gen_instances.py` | Kit. Seeded instances: targets from the capacity plan, a seeded subset displaced, teleported and settled in Isaac, then the reproduction gate; the settled poses are the stored initials. | `--mode` (`perturbed`, or `random`), `--state` (`placement`), `--n` (10), `--seed` (0), `--displace` (all for random, ceil(n_items/2) for perturbed), `--cell` (a key of `CELLS` in `code/src/dishsim/tiers.py`; overrides `--mode` and `--displace`) | collision caches, the capacity plan (in-process) | `data/results/instances/<machine>/<state>/` (`--out`, default `data/results/instances`) |
| `instance_views.py` | Kit. Initial-vs-goal stills of one instance (the problem), objects tinted per class. | `--instance` (required), `--settle_steps` (90) | the instance JSON | `data/media/instances/<machine>/<state>/<cell>/` (`--out`) |
