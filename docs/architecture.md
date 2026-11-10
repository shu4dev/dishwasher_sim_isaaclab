# Code structure

```
dishwasher_sim_isaaclab/
│
├── code/README.md                    [the blocks of docs/repo_map.txt, one table row each; every
│                                      block folder is the package of its name and has a README]
│
├── code/asset/                       [0 ASSETS — the Frigidaire twin's runtime: twin.py (spawn,
│   │                                  modes), geometry, loading, tableware, load_validation,
│   │                                  organization, paths; the twin, the dish assets and their
│   │                                  generators ship in data/assets (each folder's source/)]
│   ├── scripts/                      [the demo, the HOTEC load/settle/orbit script]
│   └── tests/, docs/                 [the loading and tableware tests; geometry.md]
│
├── code/sampler/                     [1 SAMPLER — the benchmark's stages, each stage(state, params)
│   │                                  over the sample file: metadata, initial, goal, dag]
│   ├── sample.py                     [the stage list, the ledger, run_stages, the four blocks,
│   │                                  runs/<sample>.json and index.jsonl]
│   ├── loader.py, schema/, writers/  [machine and dish manifests, checked against the schemas]
│   ├── slots.py                      [candidate sites per dish kind from the manifests]
│   ├── difficulty.py                 [level labels of an explicit roster; n from the seed]
│   ├── initial.py, gate.py           [the start (Kit) and the gates (gate_initial, gate_goal)]
│   ├── build.py, hwgate.py           [the fit-score-prove goal: CP-SAT fit, S ascent, DAG order + its
│   │                                  Isaac build; the hardware-aware goal]
│   ├── goal.py, graph.py             [certify (fresh-process rebuild), the physical-cause graph]
│   ├── stills.py                     [the stills stage: the initial and goal stills (Kit)]
│   ├── convert.py                    [schema-1 pair files -> samples, in place]
│   ├── order.py, order_check.py, pick.py   [the DAG build-order studies (history)]
│   ├── initial_state_*.py, random_pose*.py [the FCL checker of load states, the joint Isaac
│   │                                  validation, the random-pose rules and Isaac backend]
│   ├── scripts/                      [THE CLI bench.py: one sample per (roster, seed) through the
│   │                                  stages, then the four algorithms and the page; the Kit and
│   │                                  Kit-free scripts the stages call: the start generator and
│   │                                  episodes (frigidaire_bench_kit.py), the insertion build,
│   │                                  certify and graph (frigidaire_dependency_kit.py), the Isaac
│   │                                  start-state gate (the baseline's recorder and every joint
│   │                                  gate), the random-load Kit worker and its host farm, the two
│   │                                  render helpers]
│   ├── scorer/                       [1a SCORER (sampler.scorer): exposure.py, dot_rules.py,
│   │                                  score.py (the fair score, S_ref, the 6.2 ceilings file);
│   │                                  scripts/ the ceiling grids, dot rules, HOTEC exposure, top-5]
│   ├── robot/                        [1b ROBOT (sampler.robot): the native 6.2 UR5e with an EPick
│   │                                  suction body since 2026-11-09 (the pinch layer in git at
│   │                                  9f700e1), scripts/ the EPick test, arm model and replay]
│   └── tests/, docs/                 [the bench package, initial-state and random-pose tests;
│                                      hotec_bench.md, dependency.md, history/]
│
├── code/motion_planning/             [2a MOTION PLANNING (track A)]
│   ├── rearrange.py                  [the episode driver: instances, run_episode, the greedy
│   │                                  baseline, the fault knobs]
│   ├── rrt.py                        [arrangement-space RRT / RRT-Connect / RRT* planners
│   │                                  (Kit-free): a state is a full arrangement, an edge one
│   │                                  legal teleport]
│   └── planner.py                    [the FCL planner world, support graph, greedy sequencer]
│
├── code/arrangement_optimization/    [2b ARRANGEMENT OPTIMIZATION (track B): frigidaire_bench.py
│                                      (first_fit, instance_of_sample, the own-load plans),
│                                      mcts.py; scripts/ the MCTS sweep and figure]
├── code/paths/                       [3a PATHS: episodes.py (the run stage: episodes, S_final),
│                                      video.py, report.py (the bench page); scripts/ the planner
│                                      video and the benchmark's results page]
├── code/optimal_arrangement/         [3b OPTIMAL ARRANGEMENT: certify.py (own loads: plan, joint
│                                      gate, build, resequence; S_final)]
│
├── code/util/
│   ├── run_kit.sh                    [Kit launcher: docker-execs into dishsim-isaac from the
│   │                                  host, then /isaac-sim/python.sh]
│   ├── run_py.sh                     [Kit-free python the same way; bakes
│   │                                  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1]
│   ├── dishsim/                      [the Kit core (installed editable by the container
│   │   │                              entrypoint)]
│   │   ├── app.py                    [the Kit launcher every Kit script boots first]
│   │   ├── quats.py                  [XYZW <-> WXYZ at the isaaclab boundary (identity on 3.0)]
│   │   ├── media.py                  [camera rig, video writer, release_sim_for_close]
│   │   ├── collision_world.py, geometry.py, config.py
│   │   │                             [a Kit-free FCL world over a cache of meshes + manifest;
│   │   │                              no shipped machine carries a cache today]
│   │   ├── transforms.py             [pose helpers (XYZW throughout)]
│   │   └── checks.py                 [pass/fail gate helpers for scripts]
│   ├── tools/
│   │   ├── kit_smoke.py              [install gate: planning stack imports inside Kit +
│   │   │                              headless capture is non-black; wired into bootstrap.sh]
│   │   └── bootstrap.sh              [fresh-box bring-up: image build if absent + compose up +
│   │                                  the kit_smoke gate; data/assets is downloaded separately
│   │                                  with hf download]
│   └── docker/                       [Dockerfile (build record) + compose.yaml (the runtime)]
│
├── docs/                             [environment, the repo map, measured reports]
├── data/assets/                      [gitignored; a copy of the HF dataset
│                                      shu4dev/dishsim-assets (hf download to restore, hf upload
│                                      to publish), each folder with its generator in source/]
├── data/media/ data/results/ data/logs/ data/outputs/
│                                     [generated, gitignored]
├── requirements-planning.txt         [pinned planning deps, baked into the image]
└── pyproject.toml                    [packages from code/ and code/util/; the tests of every
                                       block (run via code/util/run_py.sh -m pytest code/)]
```

Imports name the packages (`from sampler.hwgate import ...`); `code` and `code/util` are the two import roots
(`PYTHONPATH=code:code/util`, or the editable install).

## Layering

Kit-free planning on one side, Kit-side validation on the other:

```
bench stages (code/sampler, code/paths): load, slots, difficulty, build's packing, score, graph, report
        │   Kit-free python via run_py.sh (FCL in-process, Warp on the GPU)
        ▼
sample file (data/results/bench/v1/<sample>.json), runs/<sample>.json, index.jsonl, and the
stage records under _work/<sample>/
        │
        ▼
Kit scripts by subprocess (code/util/run_kit.sh): the start, the insertion build, the joint
gates, certify, the own-load builds, the episodes, the stills and videos
```

Boundary convention: no module of `dishsim` or of the block packages imports `isaaclab`/`pxr`/`omni` at module scope
(function-local Kit imports in `media.py` stay legal); a Kit script boots through `dishsim.app`
first. A violation fails loudly — Kit-free scripts crash at import.

The Bosch 800 slot pipeline (machine and scene authoring, slot placement, capacity, the compat
optimum, tiers, the instance generator and runner) left the repo on 2026-11-01: git history,
tag `isaac45-final`.
