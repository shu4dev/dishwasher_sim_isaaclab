# Arrangement planner (Frigidaire FDPC4221AS)

Retired 2026-09-29 (in git history at HEAD 4455813): the pool-search planner (geometric pool,
`ExposurePlanner`, `FirstFitBaseline`, 19 instances, Isaac gates, result tables, scripts
`frigidaire_planner_{instances,generate,run}.py`), which ran on the v3 racks; its results are in
the hold folder `/media/corallab-s1/2tbhdd/brianshu/dishsim/_trash_20260929/results/planner/frigidaire/`
until 2026-10-29. Its dish goal poses kept pairwise FCL clearance >= 5 mm with no nesting; the
HOTEC benchmark uses 3 mm (`GOAL_CLEARANCE_M` in `frigidaire_bench.py`).

Reference for agents working on `code/frigidaire/src/dishsim_frigidaire/planner.py`, the part that
stays: the FCL world, support gate, counter band and sequencer under the HOTEC benchmark (about
250 lines, Kit-free; no scoring, no goal search). Score: [exposure.md](exposure.md). Benchmark:
[hotec_bench.md](hotec_bench.md).

## The problem it serves

A messy pile of dishes on a worktop above the machine (random position and rotation, stacked
where they landed) and unorganized dishes already inside the racks. A plan is a sequence of
teleport moves, one object at a time, that ends with every object inside the dishwasher; the
counter is also the buffer during the plan. No robot arm: the driver is the Bosch benchmark's
move loop, `dishsim.rearrange.run_episode`, unchanged.

```
each move    one teleport; refused (non-fatal, counted as infeasible) when FCL finds a collision
             with the appliance, the counter slab or another object
support      the sequencer never moves an object that still supports another; a move that
             displaces one anyway is the oracle's "disturbed" fault
counter cap  at most `cap` objects in the counter band at once; a move into a full band is
             refused (`counter-full`, non-fatal, counted); 25 straight refusals abort `refusal-loop`
```

## How it works

- **Driver**: `dishsim.rearrange.run_episode` (the Bosch benchmark's loop), unchanged. The caller
  supplies the algorithm, a world (the FCL mirror below) and an oracle that executes each move
  (`execute(move) -> (poses, fault, info)`, `at_goal(item, T)`). The benchmark's oracle is
  `BenchOracle` in `frigidaire_bench_kit.py` (one Isaac settle per move); the retired planner used
  a geometric one (a move lands as commanded; the only fault is moving an object that still
  supports another).
- **World**: `PlannerWorld(instance)` is the FCL mirror with the driver's world duck-type
  (`sync`, `snapshot`, `clear`, `move_collides`, `blockers`, `buffer_poses`, `in_counter`,
  `resting_on`, `certify`). An instance stores racks-OUT world poses (the Isaac scene); FCL checks
  run there with the instance's measured component frames (`initial_snapshot.poses`). A move is
  checked against the appliance (`InitialCollisionChecker`, 1 mm penetration allowance), the
  counter slab (`slab_body`, an FCL box) and every other object. `certify(T)` marks a pose as
  settled-certified: the appliance check is skipped for it, pair checks never are (goal poses are
  certified upstream). Counter objects are neither scored nor occluders. `BenchWorld` in
  `frigidaire_bench.py` (`make_bench_world`) subclasses it: the instance's own counter, plate-sized
  parking cells, the support rule only for dishes on the counter, goal-order constraints and
  goal-pair exemptions (decision table in [hotec_bench.md](hotec_bench.md)).
- **Support gate**: `support_edges(contact_pairs, poses, ids)` turns settled dish-dish contacts
  into directed (supporter, supported) edges: the higher centre, by more than `SUPPORT_MIN_DZ_M`
  = 5 mm, is the supported one; ids that are not dishes (racks) are ignored. `resting_on(item)`
  lists the objects that rest on `item` and have not moved since the start.
- **Sequencer**: `sequence(world, goals, order, cap, max_moves=None)` is the benchmark's greedy
  rule with buffering, dry-run on the mirror (restored afterwards): a `Move` list, or `None` when
  it cannot finish (default limit ten moves per object). Callers pass `item_order(instance)`:
  counter objects first (top of the pile first), then inside objects in file order. Pass 1 sends
  home any misplaced object whose goal is free and that supports nothing; pass 2 parks one
  blocker on the counter, only while the band holds fewer than `cap` objects, so no refused
  command is ever emitted.
- **Instances**: `to_rearrange_instance(instance)` builds the driver's `Instance` (machine
  `frigidaire`, state `racks_out`, no targets; meta `counter_cap`, `inventory`, `seed`,
  `n_objects`; the planner instance rides along as `planner_instance`), and
  `goal_T(world, rack, local)` turns a rack-local goal pose into a world pose on the instance's
  racks-out frame of that rack. Fields read: `instance_id`, `seed`, `inventory`, `counter` (`cap`;
  `size_m`, `center_m` and `top_z_m` in the benchmark), `objects` (`object_id`, `kind`, `start` =
  `Counter` or a rack, `pose_world`), `initial_snapshot.poses` (Cabinet, Door, LowerRack,
  UpperRack, SilverwareBasket) and `support`. The benchmark's instance files are listed under
  Outputs in [hotec_bench.md](hotec_bench.md).
- **Counter**: `COUNTER` is the default worktop slab, 1.2 x 0.6 x 0.04 m, top at 0.914 m,
  centred over the cabinet (runtime geometry only; the asset has no counter); the benchmark
  instances carry their own 1.8 x 0.6 x 0.04 m slab with the same top. Band predicate
  `in_counter_band`: over the slab and above its top minus 1 cm (`BAND_TOLERANCE_M`: a dish
  resting flat has its origin at the top). Buffer cells: a `BUFFER_PITCH_M` = 0.13 m grid over the
  slab, inset 0.05 m from its edge, hover 2 mm, FCL decides which are free (`BenchWorld`: 0.25 m
  cells).

## Files

| File | Role |
|---|---|
| `src/dishsim_frigidaire/planner.py` | `PlannerWorld`, `slab_body`, support edges, counter band, `sequence`, instance glue (`item_order`, `to_rearrange_instance`, `goal_T`, pose helpers) |
| `src/dishsim_frigidaire/initial_state_candidates.py` | `InitialCollisionChecker`, the FCL checker the world wraps (its candidate catalog and packing solvers were retired 2026-09-29) |
| `src/dishsim_frigidaire/initial_state_runtime.py` | Isaac backend; the `extra_statics` hook authors the counter slab before `sim.reset()` (`frigidaire_bench_kit.py` passes it) |
| `code/planner/frigidaire/frigidaire_bench.py` | `make_bench_world` (`BenchWorld(PlannerWorld)`), the `sequence` calls, the CLI |
| `code/planner/frigidaire/frigidaire_bench_kit.py` | Kit side: the slab through `extra_statics`, `BenchOracle`, episodes |
| `code/planner/frigidaire/frigidaire_planner_video.py` | Kit, `--enable_cameras`: stop-motion MP4 of one episode; `frigidaire_bench.py` calls it for the benchmark's episodes |
| `tests/test_planner.py` | 7 Kit-free tests (counter band, support edges, unstack fault, sequencer cap and buffer, counter-first order and support gate, instance glue, allowed kinds do not widen racks) |
| `code/util/run_py.sh` | exports Kit's USD extension so `pxr` imports Kit-free (the FCL checker reads colliders from USD) |

## Commands

```bash
code/util/run_py.sh -m pytest code/frigidaire/tests/test_planner.py
```

Instance generation, episodes and videos run through `frigidaire_bench.py` (`--generate`,
`--run-all`, `--videos`; Commands in [hotec_bench.md](hotec_bench.md)).

## Limitations to state with any result

The mirror is geometric: FCL (1 mm penetration allowance) plus the support graph; it settles
nothing (the settle is the oracle's). The sequencer is greedy with one-blocker buffering: it
returns `None` when it cannot finish and searches no further. `PlannerWorld` and `sequence` work
on racks-out poses and take the goal poses as given: how goals are chosen, the score and the end
check (retraction, containment) belong to the benchmark. The counter is an assumed slab; the
piles are generated, not a human's mess.
