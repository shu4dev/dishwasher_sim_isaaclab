# Code structure

```
dishwasher_sim_isaaclab/
│
├── scripts/
│   ├── run_kit.sh                    [Kit launcher: exports the Isaac env, then isaaclab.sh -p]
│   │
│   ├── setup/                        [PHASE 1 — assets and the simulation world]
│   │   ├── kit_smoke.py              [dependency + headless-capture gate]
│   │   ├── inspect_scene.py          [articulation survey -> docs/joint_report.md; also authors
│   │   │                              the passive-door derived USD it inspects]
│   │   ├── check_scene.py            [scene verification; --measure derives the pad map]
│   │   ├── calibrate_grasp.py        [per-object pinch calibration (force staircase)]
│   │   ├── freeze_calibration.py     [freeze measured constants into config.OBJECTS]
│   │   ├── extract_geometry.py       [dump the settled scene into the collision cache]
│   │   ├── decompose_meshes.py       [convex FCL pieces (CoACD / analytic parts)]
│   │   ├── parity_check.py           [FCL vs PhysX agreement gate]
│   │   ├── goal_configs.py           [slot frames + IK goal sets]
│   │   ├── build_state.py            [bake one machine state's caches for N classes]
│   │   ├── reach_map.py              [measure where on the counter a class can be picked]
│   │   ├── preview_rack.py           [rack geometry preview PNGs]
│   │   └── capacity_fill.py          [fully-loaded scene generator + closability check]
│   │
│   ├── experiment/                   [PHASE 2 — run algorithms, write artifacts]
│   │   ├── run_trials.py             [ONE object: rack reconfigure -> pick -> plan -> place;
│   │   │                              FROZEN — anchors the v0 baseline result]
│   │   └── run_task.py               [N objects: spawn -> settle -> sequence -> clear]
│   │
│   ├── evaluation/                   [PHASE 3 — reads artifacts only]
│   │   ├── compute_metrics.py        [trial JSONs -> metrics.json + figures + comparison]
│   │   ├── render_videos.py          [trajectory .npz -> MP4s + stills (kinematic replay)]
│   │   ├── verify_replay.py          [replay faithfulness gate, camera-free]
│   │   └── plan_visual.py            [planner search tree from the recorded query]
│   │
│   └── tools/
│       ├── archive_assets.py         [tar the generated artifacts, push to a private dataset]
│       └── restore_assets.py         [download, safe-extract, validate caches, run tests]
│
├── src/dishsim/                      [the environment package (installed editable)]
│   ├── config.py                     [EVERY tunable: object registry, grasps, rack params,
│   │                                  planner defaults, cameras, tolerances. Tune here.
│   │                                  Machine selector: apply_machine("bosch800") swaps the
│   │                                  world to the Bosch 800 twin (MACHINE_GEN, per-machine
│   │                                  RACK_GEN/scenarios/BASE_PLACEMENTS); v1 stays the
│   │                                  byte-stable default. Numbers: docs/bosch800_source_data.md]
│   ├── robots.py                     [UR5e + Robotiq and dishwasher ArticulationCfgs;
│   │                                  machine-aware v0 USD derivation incl. the third rack]
│   ├── scene.py                      [scene construction, the wrist weld, gripper control]
│   ├── usd_prep.py                   [derived dishwasher USDs; authors the procedural racks;
│   │                                  make_bosch800_usd authors the Bosch machine from scratch]
│   ├── rack_gen.py                   [procedural wire racks + cutlery basket + the Bosch
│   │                                  third-rack tray (Kit-free)]
│   ├── prop_gen.py                   [procedural props: tumbler, wine glass, container, lid]
│   ├── geometry.py                   [USD -> mesh extraction + the collision-cache format]
│   ├── collision_world.py            [Kit-free FCL world; the planners' validity oracle]
│   ├── ur5e_kin.py                   [analytic UR5e FK/IK, 8 branches (Pinocchio-validated)]
│   ├── placement.py                  [slot derivation, goal poses and success per mode]
│   ├── rack_ops.py                   [rack-handle engage + drive-synchronized slide]
│   ├── fill_plan.py                  [deterministic full-load plan + FCL validation]
│   ├── trajectory.py                 [per-step recording format (Phase 2 -> Phase 3)]
│   ├── replay.py                     [kinematic playback of a recording (Phase 3)]
│   ├── plan_debug_io.py              [persist a planning query + search tree]
│   ├── metrics.py                    [Kit-free aggregation over trial records]
│   ├── media.py                      [camera rig, video writer, contact sheets]
│   ├── transforms.py                 [pose helpers (XYZW throughout)]
│   ├── quats.py                      [XYZW <-> WXYZ at the Isaac Lab 2.1 boundary]
│   ├── task/                         [the task layer — decides WHAT, never HOW to move]
│   │   ├── sequencer.py              [which object next, in what order; support + grasp gates]
│   │   ├── primitives.py             [one object's pick-and-place choreography]
│   │   ├── motion.py                 [object-agnostic "move A to B" over the planner]
│   │   ├── layout.py                 [seeded random countertop layouts, with stacking]
│   │   ├── support.py                [which object rests on which (contact + geometric)]
│   │   ├── grasp.py                  [state-dependent grasp availability + yaw sweep]
│   │   ├── recovery.py               [bounded recovery ladder]
│   │   ├── rack.py                   [open the machine: engage a handle, slide a rack]
│   │   ├── cost.py                   [swappable pick-order heuristics (a registry)]
│   │   └── episode.py                [episode record + aggregation]
│   └── planners/                     [the pluggable planner layer; the name registry
│       │                              (PLANNERS/make_planner/available) lives in __init__.py]
│       ├── base.py                   [PlanResult, PlanDebug, the Planner ABC]
│       ├── ompl_base.py              [shared OMPL query: space, validity, goals, solve]
│       ├── rrt_connect.py            [bidirectional RRT (default)]
│       ├── rrt_star.py               [asymptotically optimal RRT]
│       ├── bit_star.py               [Batch Informed Trees]
│       └── prm.py                    [probabilistic roadmap (single-goal here)]
│
├── tests/                            [Kit-free pytest via scripts/run_py.sh]
├── docs/                             [environment, success criteria, measured reports]
├── assets/  media/  results/         [generated, gitignored]
├── requirements-planning.txt         [pinned planning deps, baked into the docker image]
└── pyproject.toml
```

## The layer boundary

The task layer (`src/dishsim/task/`) decides WHICH object moves next and WHAT the goal is;
`motion.py` is object-agnostic "move A to B"; only then comes the pluggable planner. **No task
concept may reach `planners/`** — enforced mechanically by `tests/test_layer_boundary.py`
(AST-based). `motion.ExecContext` is a Protocol the episode runner implements after Kit boots;
a conformance test compares it to the runner's implementation signature-for-signature.

## Completed one-off studies

These produced results cited in [success_criteria.md](success_criteria.md) and are kept for
reproducibility; they are not part of the routine pipeline:

- **Base-pose sweep** — a 420-candidate sweep over robot base (x, y, yaw) proving the v4
  rack, not the base pose, was the binding reachability constraint: the winner matches the
  front placement on every slot criterion and only deepens the countertop pick band
  (see "Reachability success bar" in success_criteria.md). Scorecards live in
  `results/base_sweep/`; both winners are frozen in `config.BASE_PLACEMENTS`. The tooling
  (`base_pose_sweep.py` CLI + `base_sweep.py` engine + its tests) is retired to git history —
  restore from there to sweep a new machine or mount.
- The rack-*design* harness that produced the v4 rack layout (`rack_design.py`) was retired
  after the design froze; it lives in git history, and its measured design rules are recorded
  in [known_limitations.md](known_limitations.md).
