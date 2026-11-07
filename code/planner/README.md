# planner

Where each dish goes and in what order. Every plan is checked in Isaac by one teleport per move and a settle
(an episode aborts on the first fatal fault); scoring, tables, figures and the results page belong here.

- `bench.py`: the benchmark CLI (plans/2026-11-01-merged-benchmark.md). One sample per (roster, seed): a messy start,
  the score-greedy goal and its S_ref, then the four algorithms' episodes and the report page. Stages in
  `code/src/bench/` (`load` ... `graph`, `stills`, `run`, `report`).
- `frigidaire/`: the Kit and Kit-free scripts the stages call (the start generator and episodes in
  `frigidaire_bench_kit.py`, own-load plans in `frigidaire_bench.py`, the build in `frigidaire_dependency_kit.py`, the
  scorer and its ceiling grids), plus the exposure, top-5 and MCTS studies.
- Output: samples and `index.jsonl` under `data/results/bench/v1/`, the episodes in its `runs/` (stage work in
  `_work/<sample>/`), media and the page under
  `data/media/bench/v1/`. The 4.5 bench's records under `data/results/benchmark/frigidaire_hotec/` are history.

Folder: [frigidaire/](frigidaire/README.md).

## Commands

```bash
code/util/run_py.sh code/planner/bench.py --kinds bowl --count bowl=7 --counter-allowance 3 --seed 0 --to report \
    --algorithms greedy_offline,rrt_connect,first_fit,mcts --out data/results/bench/v1
code/util/run_py.sh code/planner/bench.py --from run --to report --input data/results/bench/v1/<sample>.json
```

## Package modules

First needed here; a later stage that imports a module is named beside it.

- `bench`: every stage of the pipeline (`loader`, `slots`, `difficulty`, `initial`, `gate`, `build`, `goal`, `score`,
  `graph`, `episodes`, `report`, `order`).
- `dishsim`: `rearrange` (the episode driver), `rrt`.
- `dishsim_frigidaire`: `planner` (execution), `mcts`, `exposure`, `dot_rules`, `organization`.
