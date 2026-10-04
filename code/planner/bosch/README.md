# planner/bosch

The Bosch rearrangement benchmark: run algorithms on saved instances, then aggregate. Defaults are defined in the
script itself unless another file is named. New algorithms: implement `reset(instance, world)` /
`next_move(obs)` and add one line to `ALGORITHMS` in `run_rearrange.py`.

| Script | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `run_rearrange.py` | Kit. One Kit session per (machine, placement, rack state) batch: park the pool, teleport the roster to the recorded initials, settle, verify the reproduction, then the closed-loop episode; every move teleports and settles, the first fatal fault aborts. | `--instances` (a JSON glob) or `--cells` (`dishsim.tiers.CELLS` names, under `--instances_root`, default `data/results/instances/bosch800/placement`), `--algorithms` (`greedy`; registry `ALGORITHMS`: greedy, greedy_offline, rrt, rrt_connect, rrt_star), `--budget_mult` (3.0: move budget = ceil(mult x n_items), <= 0 unlimited), `--time_budget_s` (60.0 s of planning per episode), `--seed` (0; each (instance, algorithm) gets a derived seed), `--video` (one MP4 per episode, needs `--enable_cameras`); the fault knobs are module constants in `code/src/dishsim/rearrange.py` | instance JSONs, collision caches | episode records in `--out` (`data/results/rearrange`); MP4s under `data/media/rearrange/` |
| `compare_algorithms.py` | Kit-free. Groups episode records by (cell, algorithm): success over completed episodes, moves, optimality gap against each instance's `meta.optimum`, planning time. | `--records` (globs, default `data/results/rearrange/bosch800/placement/*/*.json` and `data/results/rearrange/bosch800/placement/*.json`) | episode records | `--out` (`data/results/compare`): `summary.csv`, `summary.md` |
