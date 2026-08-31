# Code structure

```
dishwasher_sim_isaaclab/
│
├── scripts/
│   ├── run_kit.sh                    [Kit launcher: docker-execs into dishsim-isaac from the
│   │                                  host, then isaaclab.sh -p]
│   ├── run_py.sh                     [Kit-free python the same way (/isaac-sim/python.sh);
│   │                                  bakes PYTEST_DISABLE_PLUGIN_AUTOLOAD=1]
│   │
│   ├── setup/                        [PHASE 1 — assets, the simulation world, collision caches]
│   │   ├── kit_smoke.py              [install gate: planning stack imports inside Kit +
│   │   │                              headless capture is non-black; wired into bootstrap.sh]
│   │   ├── extract_geometry.py       [dump the settled statics + object mesh into the cache]
│   │   ├── decompose_meshes.py       [convex FCL pieces (CoACD / analytic parts)]
│   │   └── gen_instances.py          [settled rearrangement instances (perturbed / random)]
│   │
│   ├── experiment/
│   │   └── run_rearrange.py          [benchmark runner: persistent Kit session, closed-loop
│   │                                  episodes, per-move settle + fault gates, --video]
│   │
│   ├── evaluation/
│   │   └── instance_views.py         [one instance's initial-vs-goal stills, one Kit boot]
│   │
│   └── tools/
│       ├── restore_assets.py         [download, safe-extract, validate cache hashes, run tests]
│       └── bootstrap.sh              [fresh-box bring-up: image build if absent + compose up +
│                                      restore + the kit_smoke gate]
│
├── src/dishsim/                      [the environment package (installed editable by the
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
│   ├── machine.py                    [dishwasher ArticulationCfg; machine-aware USD derivation
│   │                                  incl. the Bosch third rack]
│   ├── scene.py                      [Kit scene construction: statics + objects, rack drives.
│   │                                  Object spec dict: name, usd_path, pos, quat, and optional
│   │                                  contact_filters / color (per-item render tint)]
│   ├── quats.py                      [XYZW <-> WXYZ conversion at the isaaclab boundary — the
│   │                                  ONLY place quaternion order converts]
│   ├── usd_prep.py                   [derived dishwasher USDs; authors the procedural racks;
│   │                                  make_bosch800_usd authors the Bosch machine from scratch]
│   ├── rack_gen.py                   [procedural wire racks + cutlery basket + the Bosch
│   │                                  third-rack tray (Kit-free)]
│   ├── prop_gen.py                   [procedural props: tumbler, wine glass, container, lid]
│   ├── compat.py                     [Kit-free ground truth: pairwise compatibility table +
│   │                                  A* optimal solver (optimal_moves) for the benchmark's
│   │                                  optimality-gap metric. static_ok (legal destination) and
│   │                                  compatible (pair overlap) are deliberately separate]
│   ├── instance_gen.py               [sample_initials — Kit-free, shared by the Kit generator
│   │                                  and any offline instance builder]
│   ├── geometry.py                   [USD -> mesh extraction + the collision-cache format +
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
│   ├── capacity.py                   [greedy placeable-capacity planner: slots -> placeable
│   │                                  pre-scan -> joint certification -> z-budget/settle gates;
│   │                                  gen_instances calls plan_full_load() in-process]
│   ├── slotting.py                   [candidate slots, occupancy conflicts (Kit-free)]
│   ├── media.py                      [camera rig, video writer, release_sim_for_close]
│   ├── transforms.py                 [pose helpers (XYZW throughout)]
│   └── checks.py                     [pass/fail gate helpers for scripts]
│
├── tests/                            [4 files / 11 tests: the frozen-invariant pins, the
│                                      compat ground truth, the harness's toy-oracle check;
│                                      run via scripts/run_py.sh -m pytest]
├── docs/                             [environment, success criteria, measured reports]
├── docker/                           [Dockerfile (build record) + compose.yaml (the runtime)]
├── assets/  media/  results/         [generated, gitignored — symlinks onto the 2 TB drive]
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
        ▼  artifacts (capacity plans, results/instances/*.json, episode records)
        │
scene / machine / usd_prep / media   (Kit-side: teleport, settle, judge stability, render)
```

Boundary convention: only `scene.py` and `machine.py` import `isaaclab`/`pxr`/`omni` at
module scope; everything else must import in a plain Python process (function-local Kit
imports in `geometry.py`'s extraction half and `media.py` stay legal). A violation fails
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
  `inspect_scene.py`, `archive_assets.py`) — retired in the minimal-version cut (git
  history); the capacity PLANNER itself lives on in `capacity.py`.
