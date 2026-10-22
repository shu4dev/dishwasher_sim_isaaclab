# Code structure

```
dishwasher_sim_isaaclab/
│
├── code/README.md                    [the stages in order and what each hands to the next;
│                                      every stage and machine folder has its own README]
│
├── code/initialization/              [STAGE 1 — problem instances and start states; the twins,
│   │                                  assets, caches and their generators ship in data/assets
│   │                                  (each folder's source/) since 2026-10-21]
│   ├── bosch/
│   │   ├── gen_instances.py          [settled rearrangement instances (perturbed / random)]
│   │   └── instance_views.py         [one instance's initial-vs-goal stills, one Kit boot]
│   └── frigidaire/                   [the demo, the Isaac start-state gate, the random-load
│                                      Kit worker and its host farm]
│
├── code/planner/                     [STAGE 2 — where each dish goes and in what order]
│   ├── bosch/
│   │   ├── run_rearrange.py          [benchmark runner: persistent Kit session, closed-loop
│   │   │                              episodes, per-move settle + fault gates, --video]
│   │   └── compare_algorithms.py     [Kit-free: aggregates episode records into the per-cell
│   │                                  comparison table, data/results/compare/summary.{csv,md}]
│   └── frigidaire/                   [HOTEC benchmark, MCTS, HOTEC loads and top-5, results
│                                      page, episode videos]
│
├── code/execution/
│   └── frigidaire/                   [STAGE 3 — the UR5e + Robotiq 2F-85 bring-up and robot
│                                      episode (in progress)]
│
├── code/util/
│   ├── run_kit.sh                    [Kit launcher: docker-execs into dishsim-isaac from the
│   │                                  host, then isaaclab.sh -p]
│   ├── run_py.sh                     [Kit-free python the same way (/isaac-sim/python.sh);
│   │                                  bakes PYTEST_DISABLE_PLUGIN_AUTOLOAD=1]
│   ├── tools/
│   │   ├── kit_smoke.py              [install gate: planning stack imports inside Kit +
│   │   │                              headless capture is non-black; wired into bootstrap.sh]
│   │   └── bootstrap.sh              [fresh-box bring-up: image build if absent + compose up +
│   │                                  the kit_smoke gate; data/assets is downloaded separately
│   │                                  with hf download]
│   ├── frigidaire/                   [the two render helpers planner and execution scripts import]
│   └── docker/                       [Dockerfile (build record) + compose.yaml (the runtime)]
│
├── code/src/dishsim/                 [the environment package (installed editable by the
│   │                                  container entrypoint)]
│   ├── config.py                     [EVERY tunable: object registry, rack params, placement
│   │                                  modes, cameras, tolerances. Tune here. Machine selector:
│   │                                  apply_machine("bosch800") swaps the world to the Bosch 800
│   │                                  twin; apply_base_placement selects the frozen base-frame
│   │                                  anchor. FROZEN CACHE ANCHOR sections are robot-era
│   │                                  constants that feed config_hash — never tune them.
│   │                                  item_color/display_color + ITEM_COLOR_PALETTE tint
│   │                                  renders per item (media only, never physics).
│   │                                  Numbers: docs/bosch800_source_data.md]
│   ├── machine.py                    [dishwasher ArticulationCfg; loads the per-state machine
│   │                                  USD from data/assets/<machine>/usd/ (racks built in, incl.
│   │                                  the Bosch third rack; config.machine_asset_dir)]
│   ├── scene.py                      [Kit scene construction: statics + objects, rack drives.
│   │                                  Object spec dict: name, usd_path, pos, quat, and optional
│   │                                  contact_filters / color (per-item render tint)]
│   ├── quats.py                      [XYZW <-> WXYZ conversion at the isaaclab boundary — the
│   │                                  ONLY place quaternion order converts]
│   ├── rack_layout.py                [the rack layout arithmetic that slot derivation and
│   │                                  capacity read (z levels, plate-tine grid, basket bays,
│   │                                  tray zones), verbatim from the rack generator; the
│   │                                  generators (racks, machine USDs, props, HOTEC dishes)
│   │                                  moved to their asset folders' source/ on 2026-10-21]
│   ├── compat.py                     [Kit-free ground truth: pairwise compatibility table +
│   │                                  A* optimal solver (optimal_moves) for the benchmark's
│   │                                  optimality-gap metric. static_ok (legal destination) and
│   │                                  compatible (pair overlap) are deliberately separate]
│   ├── instance_gen.py               [sample_initials — Kit-free, shared by the Kit generator
│   │                                  and any offline instance builder]
│   ├── geometry.py                   [the collision-cache format (the extraction half went to
│   │                                  data/assets/bosch800/source/cache_dump.py) +
│   │                                  TWO independent cache keys: config_hash keys the
│   │                                  manifests, coacd_dir_for digests mesh bytes + the body's
│   │                                  COACD params. config.COACD is NOT in config_hash, so a
│   │                                  static re-decomposition is invisible to the staleness
│   │                                  check — see docs/known_limitations.md]
│   ├── collision_world.py            [Kit-free FCL world; object_in_collision(pieces, T) is
│   │                                  the teleport-feasibility oracle]
│   ├── placement.py                  [slot derivation (live, no bake), release poses and
│   │                                  settle success per mode]
│   ├── rearrange.py                  [benchmark core: instances, closed-loop episode driver,
│   │                                  FCL arrangement mirror, greedy baseline]
│   ├── rrt.py                        [arrangement-space RRT / RRT-Connect / RRT* planners
│   │                                  (Kit-free): a state is a full arrangement, an edge one
│   │                                  legal teleport; registered in run_rearrange.ALGORITHMS]
│   ├── tiers.py                      [difficulty tiers: 3 presets (easy/medium/hard) + 9
│   │                                  one-knob ablation cells off medium; module constants,
│   │                                  deliberately outside config.py so they never touch
│   │                                  config_hash]
│   ├── capacity.py                   [greedy placeable-capacity planner: slots -> placeable
│   │                                  pre-scan -> joint certification -> z-budget/settle gates;
│   │                                  gen_instances calls plan_full_load() in-process]
│   ├── slotting.py                   [candidate slots, occupancy conflicts (Kit-free)]
│   ├── media.py                      [camera rig, video writer, release_sim_for_close]
│   ├── transforms.py                 [pose helpers (XYZW throughout)]
│   └── checks.py                     [pass/fail gate helpers for scripts]
│
├── code/tests/                       [5 files / 37 tests (2026-10-21): the frozen config-hash
│                                      pin, the compat ground truth, the harness's and the RRT
│                                      planners' toy-oracle checks, the instance sampler's
│                                      tier knobs; run via
│                                      code/util/run_py.sh -m pytest code/tests/]
├── code/frigidaire/                  [the Frigidaire FDPC4221AS twin, a separate package
│                                      (dishsim_frigidaire) with its own src/ tests/ docs/,
│                                      tracked in git: loaders (the generator moved to
│                                      data/assets/frigidaire_fdpc4221as/source/), HOTEC
│                                      benchmark helpers, exposure scorer; the arm bring-up in
│                                      src/dishsim_frigidaire/robot/ is in progress; its
│                                      scripts sit in the stage folders' frigidaire/.
│                                      See code/frigidaire/README.md]
├── docs/                             [environment, success criteria, measured reports]
├── data/assets/                      [gitignored; a copy of the HF dataset
│                                      shu4dev/dishsim-assets (hf download to restore, hf upload
│                                      to publish): six asset folders, each with its generator
│                                      in source/]
├── data/build/ data/media/ data/results/ data/logs/ data/outputs/
│                                     [generated, gitignored — symlinks onto the 2 TB drive]
├── requirements-planning.txt         [pinned planning deps, baked into the image]
└── pyproject.toml
```

## Layering

Kit-free planning on one side, Kit-side validation on the other:

```
config / geometry (cache format, config_hash)
        │
        ▼
collision_world ── placement ── capacity / rearrange ── slotting
        (Kit-free python via run_py.sh: plan an arrangement, certify it collision-free)
        │
        ▼  artifacts (capacity plans, data/results/instances/*.json, episode records)
        │
scene / machine / media   (Kit-side: teleport, settle, judge stability, render)
```

Boundary convention: only `scene.py` and `machine.py` import `isaaclab`/`pxr`/`omni` at
module scope; everything else must import in a plain Python process (function-local Kit
imports in `media.py` stay legal). A violation fails
loudly — Kit-free scripts crash at import.

## Completed one-off studies

These produced results cited in [success_criteria.md](success_criteria.md); the code was
retired to git history when the study froze, and the measured outcomes live in the docs:

- **Base-pose sweep (robot era)** — a 420-candidate sweep over robot base (x, y, yaw) that
  selected the Bosch `side_winner` anchor. The winner survives as a FROZEN base-frame anchor
  in `config.BASE_PLACEMENTS` (the Bosch caches are baked and expressed in its frame).
- The rack-*design* harness that produced the v4 rack layout (`rack_design.py`) was retired
  after the design froze; its measured design rules are recorded in
  [known_limitations.md](known_limitations.md).
- **Capacity certification + reveal renders** (`capacity_fill.py`, `reveal_render.py`,
  `plan_full_load.py` CLI, `build_state.py`, `derive_slots.py`, `preview_rack.py`,
  `inspect_scene.py`) — retired in the minimal-version cut (git history); the capacity
  PLANNER itself lives on in `capacity.py`. The HF archive tooling (`archive_assets.py`,
  2026-09, and `restore_assets.py`) was removed on 2026-10-21 when `data/assets` became a plain
  copy of the dataset (`hf download` / `hf upload`).
