# Planning note

Short note on how the HOTEC x Frigidaire benchmark samples its problems and how its four algorithms work, in the
layout of `docs/scoring_note`: Section 1 the idea in seven steps (start, catalogue, goal, greedy, RRT-Connect,
first-fit, MCTS), Section 2 the math of each step with one figure, then a comparison table and the parameters.

- Build here: `make here` -> `build/planning_note.pdf` (Tectonic on the 2 TB drive).
- Build elsewhere: `make latexmk`, or upload this folder to Overleaf.
- Figures: `python3 figures/make_figures.py` (host) draws Figs. 1, 2, 3, 6 and 7b from the benchmark records under
  `data/results/benchmark/frigidaire_hotec/` (2026-09-29 run) and `data/build/planning_note/catalogue.npz` at the repo root.
  Recreate that npz with `code/scripts/run_py.sh docs/planning_note/figures/extract_figdata.py > out.txt` (container;
  prints a base64 npz between NPZ_BEGIN / NPZ_END) and decode it to that path. Figs. 4, 5 and 7a are TikZ files in
  `figures/`.
- Source of every rule and number: `frigidaire_bench.py`, `frigidaire_bench_kit.py`, `code/src/dishsim/rearrange.py`,
  `code/src/dishsim/rrt.py`, `code/frigidaire/src/dishsim_frigidaire/{planner,mcts}.py`; results `code/frigidaire/docs/hotec_bench.md`.
