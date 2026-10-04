# planner

Where each dish goes and in what order. Every plan is checked in Isaac by one teleport per move and a settle
(an episode aborts on the first fatal fault); scoring, tables, figures and the results page belong here.

- Input: the instances from `initialization/` (Bosch: `data/results/instances/`; HOTEC:
  `data/results/benchmark/frigidaire_hotec/instances/`), the twins and caches they were certified against.
- Output: episode records under `data/results/rearrange/` (Bosch) and `data/results/benchmark/frigidaire_hotec/`
  (HOTEC: plans, episodes, `compare/summary.{md,json}`), media under `data/media/rearrange/` and
  `data/media/benchmark/frigidaire_hotec/`.
- HOTEC instances are generated here too (`frigidaire_bench.py --generate`, see `initialization/`).

Folders: [bosch/](bosch/README.md), [frigidaire/](frigidaire/README.md).

## Commands, in order

Bosch:

```bash
code/util/run_kit.sh code/planner/bosch/run_rearrange.py --headless --enable_cameras --video --instances "data/results/instances/bosch800/placement/*.json" --algorithms greedy
code/util/run_kit.sh code/planner/bosch/run_rearrange.py --headless --cells easy,medium,hard --algorithms greedy
code/util/run_py.sh code/planner/bosch/compare_algorithms.py
```

HOTEC (the orchestrator runs on the host with `python3` because it needs docker; every stage runs in
`dishsim-isaac`):

```bash
python3 code/planner/frigidaire/frigidaire_bench.py --plan-all --tiers easy medium hard --seeds 0 --kit-jobs 3 --py-jobs 2
python3 code/planner/frigidaire/frigidaire_bench.py --run-all --tiers easy medium hard --seeds 0 --kit-jobs 3 --cameras
python3 code/planner/frigidaire/frigidaire_bench.py --analyze-all --tiers easy medium hard --py-jobs 2
python3 code/planner/frigidaire/frigidaire_bench.py --videos --tiers easy medium hard --kit-jobs 3
python3 code/planner/frigidaire/frigidaire_bench.py --collect
code/util/run_py.sh code/planner/frigidaire/frigidaire_bench_page.py
```

`--full` runs generation, plans, episodes and analysis through one scheduler, then `--collect`.

## Package modules

First needed here; a later stage that imports a module is named beside it.

- `dishsim`: `rearrange` (execution), `rrt`, `compat`.
- `dishsim_frigidaire`: `planner` (execution), `mcts`, `exposure`, `organization`.
