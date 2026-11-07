# Code structure

```
dishwasher_sim_isaaclab/
│
├── code/README.md                    [the stages in order and what each hands to the next;
│                                      every stage and machine folder has its own README]
│
├── code/initialization/              [STAGE 1 — the empty-machine baseline and the Isaac
│   │                                  start-state gate; the twin, the dish assets and their
│   │                                  generators ship in data/assets (each folder's source/)]
│   └── frigidaire/                   [the demo, the Isaac start-state gate (the baseline's
│                                      recorder and every joint gate), the random-load Kit
│                                      worker and its host farm]
│
├── code/planner/                     [STAGE 2 — the benchmark and where each dish goes]
│   ├── bench.py                      [THE CLI: one sample per (roster, seed) through the stages
│   │                                  of code/src/bench, then the four algorithms and the page]
│   └── frigidaire/                   [the Kit and Kit-free scripts the stages call: the start
│                                      generator and episodes (frigidaire_bench_kit.py), the
│                                      own-load plans (frigidaire_bench.py), the insertion build,
│                                      certify and graph (frigidaire_dependency_kit.py), the
│                                      scorer and its ceiling grids; MCTS, HOTEC loads, top-5]
│
├── code/execution/
│   └── frigidaire/                   [STAGE 3 — the UR5e + Robotiq 2F-85 bring-up and robot
│                                      episode (hardware-limits study closed 2026-10-30)]
│
├── code/util/
│   ├── run_kit.sh                    [Kit launcher: docker-execs into dishsim-isaac from the
│   │                                  host, then /isaac-sim/python.sh]
│   ├── run_py.sh                     [Kit-free python the same way; bakes
│   │                                  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1]
│   ├── tools/
│   │   ├── kit_smoke.py              [install gate: planning stack imports inside Kit +
│   │   │                              headless capture is non-black; wired into bootstrap.sh]
│   │   └── bootstrap.sh              [fresh-box bring-up: image build if absent + compose up +
│   │                                  the kit_smoke gate; data/assets is downloaded separately
│   │                                  with hf download]
│   ├── frigidaire/                   [the two render helpers planner and execution scripts import]
│   └── docker/                       [Dockerfile (build record) + compose.yaml (the runtime)]
│
├── code/src/bench/                   [the benchmark's stages, each stage(state, params) over
│   │                                  the sample file: metadata, initial, goal, dag (the
│   │                                  sampler until 2026-11-01)]
│   ├── sample.py                     [the stage list, the ledger, run_stages, the four blocks,
│   │                                  runs/<sample>.json and index.jsonl]
│   ├── loader.py, schema/, writers/  [machine and dish manifests, checked against the schemas]
│   ├── slots.py                      [candidate sites per dish kind from the manifests]
│   ├── difficulty.py                 [level labels of an explicit roster; n from the seed]
│   ├── initial.py, gate.py           [the start (Kit) and the gates (gate_initial, gate_goal)]
│   ├── build.py                      [the fit-score-prove goal: CP-SAT fit, S ascent, DAG order + its Isaac build]
│   ├── goal.py, graph.py             [certify (fresh-process rebuild), the physical-cause graph]
│   ├── score.py                      [the fair score, S_ref, the 6.2 ceilings file]
│   ├── stills.py                     [the stills stage: the initial and goal stills (Kit)]
│   ├── episodes.py                   [the run stage: own loads, episodes, S_final, videos]
│   ├── report.py                     [the bench page over a results folder]
│   ├── convert.py                    [schema-1 pair files -> samples, in place]
│   └── order.py, order_check.py, pick.py   [the DAG build-order studies (history)]
│
├── code/src/dishsim/                 [shared helpers (installed editable by the container
│   │                                  entrypoint)]
│   ├── rearrange.py                  [the episode driver: instances, run_episode, the greedy
│   │                                  baseline, the fault knobs]
│   ├── rrt.py                        [arrangement-space RRT / RRT-Connect / RRT* planners
│   │                                  (Kit-free): a state is a full arrangement, an edge one
│   │                                  legal teleport]
│   ├── app.py                        [the Kit launcher every Kit script boots first]
│   ├── quats.py                      [XYZW <-> WXYZ at the isaaclab boundary (identity on 3.0)]
│   ├── media.py                      [camera rig, video writer, release_sim_for_close]
│   ├── collision_world.py, geometry.py, config.py
│   │                                 [a Kit-free FCL world over a cache of meshes + manifest;
│   │                                  no shipped machine carries a cache today]
│   ├── transforms.py                 [pose helpers (XYZW throughout)]
│   └── checks.py                     [pass/fail gate helpers for scripts]
│
├── code/tests/                       [the bench package (manifests, ledger, labels, slots,
│                                      order), the driver's and the RRT planners' toy-oracle
│                                      checks; run via code/util/run_py.sh -m pytest code/tests/]
├── code/frigidaire/                  [the Frigidaire FDPC4221AS twin, a separate package
│                                      (dishsim_frigidaire) with its own src/ tests/ docs/,
│                                      tracked in git: loaders, the planner world, MCTS, the
│                                      exposure scorer and dot rules, the robot block. See
│                                      code/frigidaire/README.md]
├── docs/                             [environment, the repo map, measured reports]
├── data/assets/                      [gitignored; a copy of the HF dataset
│                                      shu4dev/dishsim-assets (hf download to restore, hf upload
│                                      to publish), each folder with its generator in source/]
├── data/media/ data/results/ data/logs/ data/outputs/
│                                     [generated, gitignored]
├── requirements-planning.txt         [pinned planning deps, baked into the image]
└── pyproject.toml
```

## Layering

Kit-free planning on one side, Kit-side validation on the other:

```
bench stages (code/src/bench): load, slots, difficulty, build's packing, score, graph, report
        │   Kit-free python via run_py.sh (FCL in-process, Warp on the GPU)
        ▼
sample file (data/results/bench/v1/<sample>.json), runs/<sample>.json, index.jsonl, and the
stage records under _work/<sample>/
        │
        ▼
Kit scripts by subprocess (code/util/run_kit.sh): the start, the insertion build, the joint
gates, certify, the own-load builds, the episodes, the stills and videos
```

Boundary convention: no `dishsim` or `bench` module imports `isaaclab`/`pxr`/`omni` at module scope
(function-local Kit imports in `media.py` stay legal); a Kit script boots through `dishsim.app`
first. A violation fails loudly — Kit-free scripts crash at import.

The Bosch 800 slot pipeline (machine and scene authoring, slot placement, capacity, the compat
optimum, tiers, the instance generator and runner) left the repo on 2026-11-01: git history,
tag `isaac45-final`.
