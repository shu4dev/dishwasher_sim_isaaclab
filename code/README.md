# code/

The scripts are sorted into four stages, run in this order. Each stage folder has a README and a `frigidaire/`
folder (the one machine since 2026-11-01); every script sits three folders below the repo root, except the benchmark
CLI `code/planner/bench.py`.

| Stage | What it does | Hands to the next stage |
|---|---|---|
| [initialization/](initialization/README.md) | Everything that must exist before a planner is asked anything: the empty-machine baseline and the Isaac start-state gate. The twin and the dish assets come from the asset dataset (`data/assets`, a copy of the HF dataset shu4dev/dishsim-assets: `hf download` to restore, `hf upload` to publish; their generators live in each folder's `source/`). | the baseline under `data/results/benchmark/frigidaire_hotec/baseline/` |
| [planner/](planner/README.md) | The benchmark (`bench.py`: messy start, score-greedy goal, four algorithms, report) and where each dish goes and in what order. Every plan is checked in Isaac by one teleport per move and a settle; scoring, tables, figures and the results page belong here. | samples and episode records under `data/results/bench/v1/`, media and the page under `data/media/bench/v1/` |
| [execution/](execution/README.md) | A UR5e + Robotiq 2F-85 carries out an instance's moves, no teleport (Frigidaire only, in progress). | robot run records under `data/results/robot/runs/`, trial logs and media under `data/artifacts/` |
| [util/](util/README.md) | Launchers, container files, bring-up, the install gate, and the two render helpers that planner and execution scripts import. | used by every stage |

What does not live here: the packages stay where they are, `code/src/bench` (the benchmark's stages),
`code/src/dishsim` (the episode driver and shared helpers) and `code/frigidaire/src/dishsim_frigidaire` (the
Frigidaire twin); each stage README lists the modules it is the first to need. Tests: `code/tests` and `code/frigidaire/tests`.

Run everything from the repo root through the launchers: `code/util/run_kit.sh <script> --headless ...` boots
Kit, `code/util/run_py.sh <script>` runs Kit-free python; both forward into the `dishsim-isaac` container. Judge a
Kit run by its `[RESULT]` line, never by the exit code.
