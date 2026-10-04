# code/

The scripts are sorted into four stages, run in this order. Each stage folder has a README and one
folder per machine (`bosch/`, `frigidaire/`); every script sits three folders below the repo root.

| Stage | What it does | Hands to the next stage |
|---|---|---|
| [initialization/](initialization/README.md) | Everything that must exist before a planner is asked anything: the appliance twins and dish assets, collision caches, the evidence that certifies a twin, and the problem instances (settled start states and goals). | twin builds and caches under `data/assets/` and `data/build/`; Bosch instances under `data/results/instances/`; HOTEC instances under `data/results/benchmark/frigidaire_hotec/instances/` (made by the bench in `planner/frigidaire/`) |
| [planner/](planner/README.md) | Where each dish goes and in what order. Every plan is checked in Isaac by one teleport per move and a settle; scoring, tables, figures and the results page belong here. | plans and episode records under `data/results/rearrange/` (Bosch) and `data/results/benchmark/frigidaire_hotec/` (HOTEC) |
| [execution/](execution/README.md) | A UR5e + Robotiq 2F-85 carries out an instance's moves, no teleport (Frigidaire only, in progress). | robot run records under `data/results/robot/runs/`, trial logs and media under `data/artifacts/` |
| [util/](util/README.md) | Launchers, container files, bring-up, archive and restore, the install gate, and the two render helpers that planner and execution scripts import. | used by every stage |

What does not live here: the two packages stay where they are, `code/src/dishsim` (Bosch benchmark core and
shared helpers) and `code/frigidaire/src/dishsim_frigidaire` (the Frigidaire twin); each stage README lists the
modules it is the first to need. Tests: `code/tests` and `code/frigidaire/tests`.

Run everything from the repo root through the launchers: `code/util/run_kit.sh <script> --headless ...` boots
Kit, `code/util/run_py.sh <script>` runs Kit-free python; both forward into the `dishsim-isaac` container. Judge a
Kit run by its `[RESULT]` line, never by the exit code.
