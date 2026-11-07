# code/

The code follows the blocks of the repo map (`docs/repo_map.txt`): six folders, each the Python package of its name, plus
`util/`. A block keeps its modules at the package root, its entry scripts in `scripts/`, its tests in `tests/` and its
docs in `docs/`; the scorer (1a) and the robot (1b) are the sub-packages `sampler.scorer` and `sampler.robot` with the
same layout. Every folder has a README.

| Block | Folder | What it holds | Hands to the next block |
|---|---|---|---|
| 0 ASSETS | [asset/](asset/README.md) | the Frigidaire twin's runtime: geometry, USD loading (`twin.py`), tableware, load validation, asset paths; the twin and the dishes themselves ship in `data/assets` (the HF dataset shu4dev/dishsim-assets) | USDs and manifests |
| 1 SAMPLER | [sampler/](sampler/README.md) | the benchmark's stages (`load` ... `graph`, `stills`), the CLI `scripts/bench.py`, the start generator, gates, build and certify scripts; 1a SCORER [sampler/scorer/](sampler/scorer/README.md), 1b ROBOT [sampler/robot/](sampler/robot/README.md) | samples under `data/results/bench/v1/`: start, goal, dag |
| 2a MOTION PLANNING | [motion_planning/](motion_planning/README.md) | the episode driver `rearrange.py`, the RRT planners `rrt.py`, the FCL planner world `planner.py` (track A) | moves, faults, solved |
| 2b ARRANGEMENT OPTIMIZATION | [arrangement_optimization/](arrangement_optimization/README.md) | first_fit and `instance_of_sample` (`frigidaire_bench.py`), the move-level MCTS (track B) | its own load, S |
| 3a PATHS | [paths/](paths/README.md) | the run stage (`episodes.py`), the video stage, the report page | `runs/<sample>.json`, videos and the page under `data/media/bench/v1/` |
| 3b OPTIMAL ARRANGEMENT | [optimal_arrangement/](optimal_arrangement/README.md) | track B's own loads: plan, joint gate, build, resequence, S_final (`certify.py`) | S / S_ref in `runs/<sample>.json` |
| util | [util/](util/README.md) | the launchers, the container files, bring-up and the install gate, the Kit core `dishsim` (`app`, `media`, `quats`, ...) | used by every block |

Imports name the packages (`from sampler.hwgate import ...`, `from asset.loading import visual_points`); `code` and
`code/util` are the two import roots (`PYTHONPATH=code:code/util`, or the editable install of `pyproject.toml`). Run
everything from the repo root through the launchers: `code/util/run_kit.sh <script> --headless ...` boots Kit,
`code/util/run_py.sh <script>` runs Kit-free python; both forward into the `dishsim-isaac` container. Judge a Kit run by
its `[RESULT]` line, never by the exit code.
