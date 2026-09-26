# Arrangement planner (Frigidaire FDPC4221AS)

Stale (2026-09-22): the candidate pool, instances and results under `results/planner/frigidaire/` were produced on the v3 racks and basket, archived as `build/frigidaire_collection/history/v3`. The racks and basket were rebuilt to the user's tape measurements on 2026-09-22 ([geometry.md](geometry.md)); nothing here has been regenerated on the v4 geometry. The pool records v3 usdc hashes and the old basket seat, so the planner pool tests skip as stale.

Reference for agents working on `frigidaire/src/dishsim_frigidaire/planner.py` and its scripts.
Read [exposure.md](exposure.md) first: the planner's objective is that scorer, revision 5.

## The problem

A messy pile of dishes on a worktop above the machine (random position and rotation, stacked
where they landed) and unorganized dishes already inside the racks. A plan is a sequence of
teleport moves, one object at a time, that ends with every object inside the dishwasher; the
counter is also the buffer during the plan. Settled 2026-09-20 (grill): no robot arm, the
benchmark's move loop, bowls and mugs only for now.

```
maximise   S = sum_o A_o E_o / sum_o A_o        revision-5 exposure at racks-in poses;  report W = min_o E_o
subject to every object inside a rack at the end
           no object pools (E.pools)                                          hard gate
           goal poses = pool candidates or keep-in-place; pairwise FCL clearance >= 5 mm, no nesting
           each move = one teleport; FCL clearance vs appliance, slab and every other object
           support order: an object is never moved while another still rests on it ("disturbed", fatal)
           counter cap: at most K objects on the counter at once, K = objects that start there
order:     feasibility, then S; moves reported.   gap = S_reference - S
S_reference = best feasible proposal of the REFERENCE SEARCH over the plain pool (seed 0, 50 random greedy
             draws, 3 MILP rounds; deterministic): the best arrangement ignoring how to get there. The
             planner's first stage runs that same search verbatim, so gap > 0 measures the cost of sequencing.
```

## How it works

- **Driver**: `dishsim.rearrange.run_episode` (the Bosch benchmark's loop) unchanged. The
  planner supplies a `PlannerWorld` (FCL mirror: appliance components at the instance's
  measured racks-out frames, the counter slab, every object; `InitialCollisionChecker` with
  its 1 mm allowance) and a `GeometricOracle` (a move lands as commanded; the only fault is
  moving an object that still supports another; every move is scored, 0.16 s on CUDA).
- **Goals**: the screened organized candidate pool (bowls and mugs, 103 settled poses) with
  GEOMETRIC conflicts only (separation < 5 mm, nesting, same slot: 477 of the stored 554
  edges; the organized policy's opening-exposure edges are dropped because the score measures
  occlusion itself), plus one keep-in-place candidate per inside object that rests on the
  rack alone and does not pool. Proposals with exact per-kind counts come from two searches:
  the reference search over the plain pool (verbatim, so every proposal the reference could
  see is on the table), then random greedy and MILP rounds with no-good cuts
  (`organized_candidates.solve_inventory`) over the merged pool with the episode's own seed,
  within the 60 s budget. Each proposal is scored; pooling drops it; the best one the
  sequencer can complete is the goal.
- **Sequencer**: the benchmark's greedy rule with counter objects first (top of the pile
  first), the support gate, and the cap mirrored so no refused command is ever emitted:
  pass 1 sends home any misplaced unsupported object whose goal is free; pass 2 parks one
  blocker on the counter while the band holds fewer than the cap.
- **Baseline**: first-fit in pool order (first compatible candidate per object, no keep, no
  scoring), same sequencer.
- **Instances**: `frigidaire_planner_instances.py` (Kit): inside set = a random feasible
  subset of the packing catalog; counter set = random poses above the slab, FCL-screened;
  all released in one Isaac session (`IsaacInitialStateBackend` with the slab as an extra
  static; counter objects need no rack support and may be outside the tub). The settled
  contact pairs give the support graph. Cap = the counter start count (half the inventory).
  References (`--certify`, Kit-free): the reference search's best over the plain pool, the
  organized state with the same inventory, and a sequenceability certificate.
- **Final gate**: the planned arrangement (every object at its goal, rack-local poses) goes
  through `frigidaire_initial_state_validate.py` unchanged: settle, retract both racks,
  containment. Its settled snapshot is re-scored the way the plan was scored.
- **Counter**: worktop slab 1.2 x 0.6 x 0.04 m, top at 0.914 m, centred over the cabinet
  (runtime geometry only; the asset has no counter). Band predicate: over the slab and above
  its top. Buffer cells: a 0.13 m grid over the slab, FCL decides.

## Files

| File | Role |
|---|---|
| `src/dishsim_frigidaire/planner.py` | world, support graph, oracle, geometric pool, keep-in-place, proposal search, sequencer, `ExposurePlanner`, `FirstFitBaseline`, instance IO, certify, gate manifest, settled score |
| `src/dishsim_frigidaire/initial_state_runtime.py` | `extra_statics` hook (three lines after the ground plane) |
| `scripts/experiment/frigidaire_planner_instances.py` | Kit: one instance per process, `[RESULT] PASS` or `REROLL` |
| `scripts/experiment/frigidaire_planner_generate.py` | Kit-free driver: the 10-instance set with re-rolls |
| `scripts/evaluation/frigidaire_planner_run.py` | Kit-free: `--prepare-pool`, `--certify`, episodes + gate manifests, `--collect`, `--figure` |
| `scripts/evaluation/frigidaire_planner_video.py` | Kit, `--enable_cameras`: stop-motion MP4 of one episode |
| `tests/test_planner.py` | 10 Kit-free tests (band, support edges, unstack fault, cap, keep rules, pool edges, composition, determinism) |
| `scripts/run_py.sh` | now exports Kit's USD extension so `pxr` imports Kit-free (the FCL checker reads colliders from USD) |

Outputs under `results/planner/frigidaire/`: `pool_geometric.json`, `instances/`, `episodes/`
(`<inst>_<algo>.json` records, `.manifest.json` gate inputs), `gates/`, `summary.{csv,md}`,
`method_<inst>.png`, `video/`.

## Commands

```bash
scripts/run_py.sh -m pytest frigidaire/tests/test_planner.py                                              # 10 s
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_planner_run.py --prepare-pool                  # 3 s
scripts/run_py.sh frigidaire/scripts/experiment/frigidaire_planner_generate.py --seeds 0 1 2 3 4 --sizes 9 18   # Kit, ~1.5-3 min per instance
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_planner_run.py --certify --instances "results/planner/frigidaire/instances/*.json"   # ~45 s each
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_planner_run.py --instances "results/planner/frigidaire/instances/*.json" --algorithms planner,baseline   # ~30 s each
for m in results/planner/frigidaire/episodes/*.manifest.json; do
  scripts/run_kit.sh frigidaire/scripts/experiment/frigidaire_initial_state_validate.py --manifest $m \
      --out-dir results/planner/frigidaire/gates/$(basename $m .manifest.json) --max-wall-seconds 240 --headless --device cpu; done   # ~1.5 min each
scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_planner_video.py --instance results/planner/frigidaire/instances/<inst>.json \
    --episode results/planner/frigidaire/episodes/<inst>_planner.json --out-dir results/planner/frigidaire/video --headless --enable_cameras   # ~3-5 min
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_planner_run.py --collect --figure <inst>
```

## Records

Instance (`instances/s<seed>_n<n>.json`, not a state file): `objects` (`object_id`, `kind`,
`start` = Counter | rack, `rack_local_pose` for in-rack objects, `pose_world` racks-out),
`initial_snapshot` (joints, poses of the five components and every object, `contact_pairs`),
`support` (supporter, supported), `counter` (slab, cap), `baseline` (for the gate manifest),
`validation`, `hashes`, `references` (after `--certify`).

Episode (`episodes/<inst>_<algo>.json`): the driver record (`solved`, `abort`, `moves` with
`kind` goal|buffer, `score_after`, `disturbed`, `counter_full_refusals`, `infeasible_commands`,
`planning_time_total_s`, `algo_stats`) plus `goals`, `planned {score, worst, feasible}`,
`reference_score`, `organized_score`, `gap`, `moves_lower_bound`, `kept`, `score_trace`.

## Results on record (2026-09-20, 10 instances, 5 seeds x {9, 18} objects)

| | planner | first-fit baseline |
|---|---|---|
| solved (all inside, no fault) | 10/10 | 10/10 |
| final Isaac gate accepted | 10/10 | 10/10 |
| S, 9 objects (reference 0.310, organized 0.256) | 0.310 to 0.315 | 0.264 |
| S, 18 objects (reference 0.272, organized 0.242) | 0.272 | 0.224 |
| gap to the reference | -0.005 to 0 (keep-in-place beats the pool twice) | +0.046 / +0.048 |
| moves (lower bound 9 / 18) | 8 to 9 / 18; 0 buffer moves | 9 / 18 to 19; 2 buffer moves |
| settled minus planned S | at most 4e-4 | at most 1e-5 |
| planning time | 19 to 26 s | 1 to 2 s |

Full table: `results/planner/frigidaire/summary.md` (written by `--collect`). Deliverables:
`results/planner/frigidaire/video/s0_n18_planner.mp4` (18 moves, 17 s) and `method_s0_n18.png`.
Instance generation needed re-rolls (rack-speed flake, pile penetration); every accepted
instance's physics record is under `instances/attempts/`.

## Plates and cutlery (2026-09-21)

The bowl/mug set above was limited by two facts: the screened organized pool has one plate
slot (on-edge plate candidates failed 11 of 12 screenings, never at rest in the 12 s window),
and the initial-state validators rejected cutlery kinds and the basket. The extension:

- **Kinds**: `dinner_plate`, `fork`, `knife`, `tablespoon`, `teaspoon` join bowls and mugs.
  Inventories: 12 = 3 bowls, 3 mugs, 2 plates, one of each utensil; 24 = 6, 6, 4, two of each.
  Five seeds each (`s<seed>_n12`, `s<seed>_n24`). The bowl/mug instances and their pool file
  (`pool_geometric.json`) are untouched; the mixed pool is `pool_geometric_v2.json` (`--pool`).
- **Plate goals**: the packing catalog's plate candidates that passed its rack-penetration
  screen (52 of 225) and do not pool (29; lower and upper rack). Slot = the source random-pose
  trial, so variants of one trial conflict. These are FCL-screened perturbations (10 mm,
  10 deg) of settled single-dish poses, not individually settled; the final Isaac gate decides.
- **Cutlery goals**: the frozen head-down basket patterns (`cutlery_candidates.json`), first
  5 variants of each of the 9 slots per kind (45 per kind); compartments are kind-fixed.
  Conflicts = same slot or FCL overlap with the checker's 1 mm allowance (no 5 mm rule
  inside a compartment). Dish vs cutlery never conflicts: the basket volume is disjoint from
  every rack slot, measured at pool build (`cross_kind_overlaps`, 0).
- **Frames**: the frozen patterns live in the static basket frame; the settled basket sits
  about 7 mm lower and 2 mm forward. `racks_in_pose` composes basket poses on the instance's
  settled basket (`basket_in`) and fails closed without it; FCL goals use the instance's
  racks-out basket frame; the gate re-settles from the same rack-local poses.
- **Search**: the MILP knows only dish kinds, so proposals are drawn over the dish sub-pool
  (plates get a synthetic `row_metadata`) and each is extended by a random-greedy cutlery
  assignment over the cutlery sub-pool with the same adjacency. Keep-in-place applies to inside
  dishes and utensils alike (a kept utensil blocks its own slot).
- **Starts**: cutlery may start in the pile (sampled like dishes) or unorganized in the basket
  (a random compatible subset of the patterns, released 3 mm above the pattern pose). If a
  draw fails six times the generator falls back to basket-only cutlery starts and records
  `cutlery_start` in the instance.
- **Validation**: the initial-state path accepts the new kinds through explicit sets
  (`random_poses.OBJECT_KINDS` / `OBJECT_RACKS`) at its three checks (manifest ingest, FCL
  checker, report validator). `RACKS`, which drives rack motions, is not widened; the support
  rule already resolves a basket assignment. Cutlery is the flakiest physics in the repo
  (thin, light, 2 mm peak-penetration gate): settle displacement per kind is what to watch.

```bash
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_planner_run.py --prepare-pool                       # v2 pool, ~5 s
scripts/run_py.sh frigidaire/scripts/experiment/frigidaire_planner_generate.py --seeds 0 1 2 3 4 --sizes 12 24 --max-attempts 9
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_planner_run.py --certify --instances "results/planner/frigidaire/instances/s*_n12.json"   # and n24
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_planner_run.py --instances "results/planner/frigidaire/instances/s*_n12.json" --algorithms planner,baseline   # and n24
# gates: the validate loop over episodes/s*_n{12,24}_*.manifest.json; then the video (s0_n24) and --collect --figure s0_n24
```

### Results on record, mixed inventories (2026-09-21, 9 instances)

Generated: all five 12-object instances (2 of them basket-only cutlery starts) and four of
five 24-object instances; the fifth (seed 4) failed 45 draws on pile penetration (a bowl or
mug landing on a plate) and is reported as not generated. The 24-object piles only settled
with a third of the load on the counter (`--counter-share 0.3334`, cap 8); half never passed
the 2 mm peak-penetration gate.

| | planner | first-fit baseline |
|---|---|---|
| solved (all inside, no fault) | 9/9 | 9/9 |
| final Isaac gate accepted | 9/9 (three after banning one gate-rejected cutlery pose and re-planning, the claims precedent) | 8/9 (one rack-speed closure flake, both orders) |
| S, 12 objects (reference 0.383) | 0.383 | 0.316 |
| S, 24 objects (reference 0.290) | 0.290 to 0.299 | 0.237 |
| gap to the reference | -0.009 to 0 | +0.053 to +0.067 |
| moves (lower bound 12 / 24), buffer moves | 12 / 24 to 26, up to 2 parked | 12 / 24 to 25, up to 1 parked |
| settled minus planned S | at most 0.003 | at most 0.003 |
| max settle displacement of one object | 27 to 38 mm (always cutlery) | 19 to 47 mm (always cutlery) |

Cutlery is what moves at the gate: the frozen basket poses are release poses that drop and
lean, a spoon or fork ends 2 to 5 cm from where it was placed, but its share of the
food-contact area is small, so the score barely changes. Three planner arrangements were
first rejected by the gate for a single utensil hitting a basket rib beyond 2 mm; banning that
pose (`--ban`) and re-planning produced an accepted arrangement each time.

Deliverables: `results/planner/frigidaire/video/s1_n24_planner.mp4` (24 moves, 22 s) and
`method_s1_n24.png`. Full table: `results/planner/frigidaire/summary.md`.

## Limitations to state with any result

Per-move validation is geometric (FCL plus the support graph); only the final arrangement is
settled in Isaac. Goal poses are limited to the pools plus keep-in-place, so the reference is
the best pool proposal, not a global optimum. Plate goal variants are FCL-screened
perturbations of settled poses, not individually settled; cutlery goals are frozen release
poses whose physics is marginal. The counter is an assumed slab; pile generation is sampled
poses released to settle, not a human's mess. The score is the revision-5 proxy with all its
assumptions.
