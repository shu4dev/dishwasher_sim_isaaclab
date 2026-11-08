# arrangement_optimization

Block 2b ARRANGEMENT OPTIMIZATION of `docs/repo_map.txt` (build one): builds its own load from the start alone, scored
S / S_ref. Track B: `first_fit` and the move-level MCTS. The package is `arrangement_optimization`:
`frigidaire_bench.py` (first_fit, `instance_of_sample`, track B's own-load plans; also a CLI, run as
`code/util/run_py.sh code/arrangement_optimization/frigidaire_bench.py ...`; importers use
`arrangement_optimization.frigidaire_bench`) and `mcts.py` (move-level Monte Carlo tree search for the open track).
Tests in `tests/`; reference `code/sampler/docs/hotec_bench.md`.

## Scripts

Defaults are defined in the script itself unless another file is named. Host = `python3` on the host; Kit = through
`code/util/run_kit.sh`; Kit-free = `code/util/run_py.sh`.

| Script | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `frigidaire_bench.py` (package root) | Kit-free library and CLI of the benchmark's run stage (code/paths/episodes.py): the sample -> instance conversion (`instance_of_sample`), track B's own-load plans (`--plan`, `--ban-plan`, `--resequence`), the episode world, the 4.5 bench's tables and host modes (history). Track A: greedy_offline, rrt_connect reach the sample's goal; track B: first_fit, mcts build their own load. The instance generator (`--goal`, `--finalize`, `--generate`, `--full`) retired 2026-11-01. | `--plan` (an instance), `--algorithm` (open track: `first_fit`, `mcts`; `TRACK_ALGORITHMS`), `--attempt` (0), `--out` (the results folder; its own-load bans count beside the old bench's), `--resequence` + `--instance`, `--final`; host modes `--plan-all`, `--run-all`, `--analyze-all`, `--videos`, `--collect` (the 4.5 bench); the learned placement rules are constants in the script (`FRONT_RIGHT_MAX_X_M`, `DROPPED_SLOTS`, `AT_GOAL`) | the Frigidaire build, the HOTEC assets, an instance | `<out>/plans/open/<tier>/`; history: `data/results/benchmark/frigidaire_hotec/` |
| `frigidaire_mcts_sweep.py` | Kit-free. S versus planning budget of the track-B MCTS planner, on paper (nothing certified in Isaac). | `--instances` (required), `--budgets` (180 600 s) | instances | `data/results/benchmark/frigidaire_hotec/mcts_budget_sweep/`, one JSON per (instance, budget) |
| `frigidaire_mcts_figure.py` | Kit-free. Method figure of the MCTS planner: the search, what it found in 60 s, S versus budget. | none | `plans/open/<tier>/<id>__mcts.json`, `mcts_budget_sweep/*.json` | `data/media/benchmark/frigidaire_hotec/mcts_method.png` |
