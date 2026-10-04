# HOTEC rearrangement benchmark (Frigidaire FDPC4221AS twin)

Reference for agents working on `code/frigidaire/scripts/experiment/frigidaire_bench.py` (Kit-free library, CLI,
scheduler) and `frigidaire_bench_kit.py` (Kit: start generation and episodes). Read [exposure.md](exposure.md)
for the score and [planner.md](planner.md) for the planner this reuses. Designed with the user in six grilling
rounds on 2026-09-23; the approved plan is `~/.claude/plans/i-am-creating-a-majestic-feather.md`.

## The problem

> **Regenerated 2026-09-28/29 on the re-measured racks** (`upper_tines_4x13_v5`, `lower_tines_6x12_v4`,
> `code/frigidaire/docs/geometry.md`): baseline snapshot, instances, plans, episodes and `compare/` under
> `data/results/benchmark/frigidaire_hotec/` are new-rack artefacts (see "Results" below); the previous racks' artefacts
> are in `pre_rebuild_20260928/`, the stopped 2026-09-28 medium/hard run in `stopped_20260928/`. Measured numbers
> quoted inside the decision table are from the run that motivated each decision (old racks before 2026-09-28).

```
tiers      easy   7 bowls                     counter allowance n + 3
           medium 8 plates + 7 bowls          n + 2 (n + 1 until 2026-09-28)
           hard   8 plates + 7 bowls + 8 cups n + 1 (n + 0 until 2026-09-28)   n = dishes on the counter at the start, uniform in [N/4, 3N/4]
           (7 bowls: the 8th did not fit physically beside plates, and without them only once in 300 packings)
start      n dishes in messy STACKS on a 1.8 x 0.6 m counter slab (top 0.914 m); the rest dropped messily into the
           extended racks (plates lower rack, bowls and cups either rack); built in Isaac one dish at a time
goal       no hand-given goal: the highest-exposure complete load found by coordinate ascent over the HOTEC rack
           families (start: keeps + first-fit, shuffled on re-rolls, else an Isaac-validated goal of the tier; top 6
           + 2 random alternatives per dish, until no gain, max 2 sweeps; seeded per instance AND attempt);
           candidate pairs that penetrated together in a goal gate are banned for every later search;
           rack dishes may stay where they start (keep-in-place). The goal POSES are those of an Isaac build one
           dish at a time (lowest first); the build's support edges are goal-order constraints (a command to b's
           goal waits until the dishes b leans on are at their goals). S_ref = S of the built poses.
track A    reach the sampled goal: every dish within the at-goal tolerance (bowls/cups 15 mm lateral, 20 mm dz,
           15 deg tilt; plates 18 mm / 20 mm / 16 deg), then the end check. gap = moves - lower bound,
           lower bound = counter dishes + rack dishes the goal does not keep.
track B    open: success = every dish racked, the end check passes AND no dish pools (the score's puddle rule,
           2026-09-28); reported S_final (racked dishes, full resolution) vs S_ref.
algorithms track A: greedy_offline, rrt_connect (Bosch, live) on the sampled goal. Track B: first-fit baseline
           and move-level MCTS (since 2026-09-29; the coordinate-ascent "exposure planner" is retired, its rows
           in history_planner_20260929/), each with its OWN load (Kit-free plan -> Isaac sequence build of that load
           -> re-sequenced under the build's order -> replayed). No row is copied from the other track
           (2026-09-28; before, the planner/baseline shared one goal-track run and the Bosch pair's goal-track
           runs were re-scored as open rows).
own loads  --plan-all per algorithm: plan attempt a<k> -> `frigidaire_bench_kit.py --sequence-plan` builds the
           load one dish at a time in the instance's extended racks (landing = settle deviation <= 8 cm and
           contact depth < 2 mm, no placed dish moved > 10 mm / 20 deg; the built poses become the targets)
           -> `--resequence` re-sequences the moves under the build's order and writes <id>__<algo>.json
           (certified); a failed build bans what it blamed (<plan>.bans.json, global like the gate's bans)
           and re-plans, up to 6 attempts (a baseline re-plan is the deterministic first-fit under the bans,
           falling back to a few seeded shuffles of the family order when that no longer packs); still failing =
           the row's failure `load-not-buildable`, not a
           blind replay. Planning time = the sum over attempts (the Isaac builds are not charged).
moves      teleport only; FCL refusal (non-fatal, counted); Isaac settle 150 + 60 ticks at 120 Hz; another dish
           moved > 10 mm / 20 deg is FATAL (disturbed) if it was at its goal before the move or ends off the
           counter and out of the racks, otherwise a recorded non-fatal "nudge" (a messy start dish); lifting a
           counter dish another rests on = disturbed; drift past 8 cm or the moved dish left > 2 mm into anything
           (the gates' contact-depth limit) = failed settle (put back, non-fatal);
           refusal-loop and settle-loop at 25; no move budget; 60 s planning budget per episode.
end check  tub-wall clearance (rack-frame |x| <= 0.277 m for every visual point), both racks retracted,
           every dish contained; failure = not solved.
```

## Start generation (Kit, `--start`)

One attempt = one Kit process, rng `[seed, tier index, 1000 + attempt]`, accepted only if ALL hold:

1. **Build.** Extended empty baseline; rack dishes are dropped one at a time (random pose in the rack box, lifted
   until FCL-clear of the appliance, the slab and every settled dish), then the counter stacks, bottom dish first.
   Stacks: one kind each, 1-4 high (cups alone), bowls and cups face-down half the time; per dish a random yaw, an
   offset <= 15 mm and a tilt <= 4 deg; stack bases cluster (summed base footprint = half the box) with every rim
   2 cm inside the slab edge. Each dish settles 1 s before the next.
2. **Rest.** The backend's settle-and-observation hold: 1 s windows over 5 s with peak AND median contact depth
   under `LIMITS` (the same 2 mm gate as every Frigidaire load), motion settled, rack dishes supported. Landing
   transients during the build are recorded (`build_transient_penetration_m`), not gated.
3. **Counter band and racks.** Exactly n dishes on the counter, every other dish racked (`RACK_BOX`).
4. **Reproduction.** Teleport every dish back to its settled pose, 300 ticks, within 10 mm / 15 deg.
5. **Unstack rehearsal.** Clear the scene top-down in Isaac: lift away a dish nothing rests on (resting dishes
   first), park it, settle 150 + 60 ticks, every remaining dish within 10 mm / 20 deg. A disturbing removal is
   undone (whole pre-removal state teleported back, must reproduce) and the next free dish tried (3 per step):
   accepted = some clearing order exists, so no instance is unsolvable by pile physics alone.

Then Kit-free `--goal` (the ascent; goal dishes keep >= 3 mm pairwise clearance), the Isaac goal gate
(`frigidaire_initial_state_validate.py` on the goal manifest: joint settle, retract both racks, containment; a
rack-speed flake is retried lower-first once; a dish-on-dish or dish-into-appliance penetration is banned),
the Isaac **sequence gate** (`frigidaire_bench_kit.py --sequence`: the goal built one dish at a time, lowest
centre first, each at its jointly settled pose + 3 mm and settled like a move: it must land within the at-goal
tolerance with < 2 mm contact depth and move no placed dish > 10 mm / 20 deg; a failing pose or pair is banned;
the built poses become the targets; the goal-order constraints are the build's support edges, the order it
learned from failed builds (a dish that knocks a placed one goes in before it; one that does not land goes in
after its neighbours; up to 8 builds), and the certified order between every pair of same-rack neighbours within
15 cm, the only order known to work locally) and Kit-free
`--finalize` (tub check, S_ref, lower bound, the planner's sequencing certificate, recorded not gated). Up to 12 attempts per (tier, seed).

## Commands

The orchestrator runs on the host (`python3`, it needs docker); every stage runs in `dishsim-isaac`.

```bash
B="nice -n 10 python3 code/frigidaire/scripts/experiment/frigidaire_bench.py"
$B --capacity                                   # gate G2 (Kit-free, in the container): 8 / 16 / 24 racked
$B --generate --tiers easy medium hard --seeds 0 1 2 --kit-jobs 3 --py-jobs 2 --max-attempts 12
$B --plan-all --tiers easy medium hard --seeds 0 1 2 --kit-jobs 3 --py-jobs 2   # own loads: plan (Warp, 60 s) -> Isaac build -> resequence
$B --run-all  --tiers easy medium hard --seeds 0 1 2 --kit-jobs 3 --cameras   # 5 Isaac episodes + stills each
$B --analyze-all --tiers easy medium hard --py-jobs 2                 # S_final, S per move, score-detail figure
$B --videos --tiers easy medium hard --kit-jobs 3                     # first instance per tier, 5 videos each
$B --collect                                                          # compare/summary.{md,json}
code/scripts/run_py.sh code/frigidaire/scripts/evaluation/frigidaire_bench_page.py --label "..."   # results page folder
```

Scheduler: one container (`dishsim-isaac`, GPU 1), at most 3 Kit jobs (a camera Kit job is about 2.9 GB of
GPU memory) plus 2 Kit-free jobs, each on its own cores (`taskset` 0-7 / 8-15 / 16-23 Kit, 24-27 / 28-31
Kit-free) with BLAS pools sized to them, `nice -n 10`, Kit launches staggered 90 s, each waiting for 6 GB free.
Logs: `data/logs/benchmark_frigidaire_hotec/<unit>.log`, judged by the `[RESULT]` line.

## Outputs

`data/results/benchmark/frigidaire_hotec/`: `baseline/`, `families_appliance_free.json` (cached by family digest),
`instances/attempts/<tier>_s<seed>_a<k>/` (start, goal, gate, generation report), `instances/<tier>/<id>.json`,
`plans/open/<tier>/`, `episodes/{goal,open}/<tier>/<id>__<algorithm>.json` (+ `.analysis.json`),
`episodes/_kit/` (per-instance run reports), `compare/summary.{md,json}`. Media under
`data/media/benchmark/frigidaire_hotec/`: `stills/<tier>/<id>/` (initial, goal, finished per episode),
`analysis/<tier>/`, `video/<tier>/`, `page/` (the published folder). Smoke runs of 2026-09-23 are kept under
`*_smoke_20260923/` with one folder per failed design (see below).

## Results (2026-09-29, re-measured racks)

Run: host `python3 code/frigidaire/scripts/experiment/frigidaire_bench.py --full --tiers easy medium hard --seeds 0 1 2
--max-attempts 8 --kit-jobs 3 --py-jobs 2`, `[RESULT] PASS full (9 instances)` at 05:46 local; pilot = the seed-0
chains (published 03:45). Easy s3-s5 (generated and run 2026-09-28 on the same racks) are in the tables too. Tables:
`data/results/benchmark/frigidaire_hotec/compare/summary.md` (`--collect` must run in the container: `compare/` is
root-owned, the host collect at the end of `--full` fails quietly); page: `data/media/benchmark/frigidaire_hotec/page/`.

| track | tier | algorithm | success | moves | S / S_ref | failures |
|---|---|---|---|---|---|---|
| A | easy (6) | greedy_offline | 6/6 | 7.8 | 1.002 | - |
| A | easy (6) | rrt_connect | 6/6 | 8.3 | 1.002 | - |
| A | medium (3) | greedy_offline | 1/3 | 20.0 | 1.003 | not_all_racked 2 (gave up at move 6 and 11) |
| A | medium (3) | rrt_connect | 2/3 | 33.0 | 1.000 | disturbed 1 (move 37) |
| A | hard (3) | greedy_offline | 3/3 | 28.3 | 1.006 | - |
| A | hard (3) | rrt_connect | 2/3 | 52.5 | 1.006 | not_all_racked 1 (60 s budget spent, 10 004 nodes, 0 moves) |
| B | easy (6) | baseline | 5/6 | 7.0 | 0.666 | pooling 1 |
| B | medium (3) | baseline | 3/3 | 19.7 | 0.975 | - |
| B | hard (3) | baseline | 2/3 | 28.5 | 0.964 | not_all_racked 1 (replay move 14 of 27 refused, bowl_05 blocked by the settled cup_02; the blind replay cannot adapt) |
| B | easy (6) | mcts | 6/6 | 7.8 | 1.036 | - |
| B | medium (3) | mcts | 3/3 | 21.0 | 1.019 | - |
| B | hard (3) | mcts | 3/3 | 29.0 | 1.008 | - |

- (Retired planner, 2026-09-29 morning run; rows in `history_planner_20260929/`: easy 6/6 at 0.929, medium 2/3 at
  0.962, hard 3/3 at 0.971 S/S_ref.) Its certified load WAS first-fit's on medium s0, medium s2 and hard s0 (0 of
  15/23 dishes differ): its 45 s ascent from the first-fit load found no improving move in full racks. Only hard s1 and s2 differ (by 8 and
  6 dishes; settled S 0.170 vs 0.165 and 0.174 vs 0.172). On easy (7 bowls, free space) it gains 0.93 vs 0.67 of S_ref.
- greedy_offline is not the certificate's sequencer (see the decision table): it gives up on certified medium goals.
- Generation attrition, attempts -> accepted instances, by the first failing step (Kit-free classifier over
  `instances/attempts/`; no attempt failed the sequencing certificate at n + 2 / n + 1):

| tier | attempts | accepted | start never rests | joint gate: penetration | joint gate: settle | one-at-a-time build |
|---|---|---|---|---|---|---|
| easy (2026-09-28) | 12 | 6 | 2 | 1 | 1 | 2 |
| medium | 15 | 3 | 5 | 5 | 1 | 1 |
| hard | 9 | 3 | 1 | 2 | 0 | 3 |

## Track B planner: move-level MCTS (2026-09-29)

`code/frigidaire/src/dishsim_frigidaire/mcts.py`, called by `plan_open(..., "mcts")` inside the same 60 s budget; the
certification (joint gate, one-at-a-time build, bans, 6 attempts, no repeated load) is first-fit's. Replaced the
coordinate-ascent planner, which ended on first-fit's load on 3 of 5 medium/hard instances (45 s covered part of
one pass) and had no sequential decision in it.

```
state     every dish's current pose on the FCL mirror (counter pile, messy rack drops) + the dishes COMMITTED
          to their final pose
action    place(d, pose): one of d's 6 candidate poses = its 3 best stand-alone-exposure catalogue poses that fit,
          then poses from a pool of complete packings; keep(d): a rack dish stays; buffer(d): park on the counter
          (below the cap). Legal = FCL on the mirror (move_collides) + the load rules (one per slot, no nesting,
          >= 3 mm, bans), cached pairwise (Conflicts)
pool      first-fit + up to 12 seeded packs (plates in first-fit order; bowls/cups shuffle the first 200 poses of
          the first-fit order or the first 50 of the exposure ranking), <= 15 s; the root starts from the best one
rollout   LOCAL repair of the parent's complete load: the moved dish takes its pose, clashing uncommitted dishes are
          re-placed (the vacated pose first, then first-fit, squeaky-wheel restarts); a pose that cannot be
          repaired is dropped for the rest of the search. Value = low-res S (120 x 32), 0 if it pools
tree      UCT (c = 1/sqrt 2 on min-max normalised values), progressive widening 2 N^0.5, lazy children
          (placements round-robin over dishes, parks last), dead ends not expanded
answer    the best complete load found (ties: fewer moves) = the tree's move prefix + planner.sequence completion;
          --resequence replays that order re-targeted to the built poses when it is legal under the build's
          goal order (`order_source` "mcts"), else resequences ("resequenced")
```

| Design step | Why (measured, Kit-free, medium_s2 / hard_s2, 60 s) |
|---|---|
| Pairwise conflict cache + AABB-gated FCL managers (`DishSet._bounds`, `_MANAGER_CACHE`) | one first-fit completion took 24 s (nests() re-posing 1814 bowl point clouds, FCL managers rebuilt per test); now 0.1-0.6 s |
| Placements before parks | parks change no final pose: 5 of the first 10 root children were parks with the parent's value |
| Squeaky-wheel repair, then LOCAL repair from the parent's load | completing from scratch failed 11/12 (plain first-fit) and 37/39 (repair) medium rollouts once a bowl sat on a high-exposure pose |
| Pool of complete packings; root = the best | shuffled whole packs completed 0/6 on medium; head-shuffled (first 200) 6/6; the root at the best pool load lifted planned S 0.173 -> 0.182 (medium_s2), 0.169 -> 0.183 (hard_s2) |
| Result (12 instances, Isaac) | every load certified (attempts used: medium 4/5/3, hard 2/1/1, easy 1-2); settled S / S_ref 1.036 easy, 1.019 medium, 1.008 hard vs first-fit 0.666 / 0.975 / 0.964, all solved; replay order: MCTS's own on 2 of 12 (easy s4, s5), resequenced on 10 (the build's lean-on order is learned after planning); planning wall summed over attempts (medium 222 s = up to 5 x 60 s); failed attempts = joint-gate transients 2.1-4.3 mm vs 2 mm (limit kept) and not-sequenceable builds |
| Where the gain comes from | the pooled root does most of it; the tree adds 0.7-3.6 % (easy), 1.9 % (medium s0), ~0 (hard) of low-res S within 60 s (`data/media/.../mcts_method.png`) |
| Budget sweep (Kit-free, planned S / first-fit's, a0 seed) | 60 / 180 / 600 s: medium 1.043-1.066 / 1.043-1.081 / 1.043-1.081, hard 1.071-1.094 / 1.078-1.120 / 1.090-1.120; ~120-170 / 600-1100 / 2300-3800 simulations (`mcts_budget_sweep/`) |

## Decisions after approval (2026-09-23)

| Change | Why (measured) |
|---|---|
| Front-right bowls stay at x <= 0.165 (a 10 mm tub-margin rule was tried and reverted) | poses at x .170-.190 clear the wall as commanded but settle off their 60 mm lift to a lip at x .286 > .277 and caught the cabinet on retraction in every medium/hard goal gate |
| No-nesting rule also needs overlapping xy footprints | the claims rule (axial depth slabs only) called side-by-side bowls "nested" whatever their distance |
| Ban goal pairs that penetrated in a gate (`bans/`, `attempts/*/bans.json`) | the third rear bowl (X150 x .05) wedged 6.5 mm into the lifted Y120 x -.065 bowl in three medium/hard gates |
| Every tier holds 7 bowls (was 8) | with 8 plates in the front bank the 8th bowl needs either the outer front-right pose or the banned rear pair, both rejected by Isaac; easy (no plates) can rack 8 (two front-right bowls at x .065 / .160, accepted twice), but first-fit found such a packing once in 300 shuffles, so the open-track planners could not rack easy within 60 s |
| Re-rolls start from a shuffled first-fit or a validated goal | the deterministic packing repeated the same failing goal; easy's only 8-bowl packing without the ban turned up once in 300 shuffles |
| Retraction sweep on families and keeps (Kit-free, 25 slide positions, cached) | a necessary check on the commanded poses; the settled poses are still judged by the Isaac goal gate and the finalize tub check |
| Sequence gate + goal-order constraints (a "solo" any-order gate was tried and retired) | a jointly settled goal leans dishes on each other: placed first, a rear bowl slid 39 mm and the plate placed next to it pushed it 19 mm (medium_s0, every algorithm). Requiring each dish to stand alone and not touch its neighbours banned the standard upper bowl pair and adjacent plates, collapsing capacity: dense HOTEC loads are order-dependent by nature, so the gate certifies one order and the planners get it as constraints |
| Goal dishes keep >= 3 mm apart | per-move placement next to a touching neighbour pushes it; 5 mm (planner.md) costs every tier a bowl (6 / 14 / 22) |
| Support rule only for dishes on the counter | in the racks an upright plate touching a lying bowl has the higher centre and read as "resting on" it; rack neighbours are judged by the disturbance check |
| Displacing a dish is fatal only if it was at its goal or leaves the counter/racks; else a counted "nudge" | messy rack drops lean on each other: placing next to or pulling out a leaning start dish moved neighbours > 10 mm, aborting nearly every pilot episode within 1-6 moves (the Bosch rule assumes separated items) |
| Goal order also fixes neighbour pairs (same rack, < 15 cm) | a first-try build learns nothing, yet greedy placing bowl_01 before its neighbour bowl_03 (the reverse of the build) got bowl_01 shoved 43.8 mm; with it: easy 3, medium 17, hard 23 constraints |
| Episode reset retried once | a stacked start reproduced to 10.25 mm (gate 10 mm) on one reset after passing at 0.27 mm in generation |
| Per-move contact-depth check (2 mm, put back) | finished loads failed the end check at retraction step 1 with 7-9 mm resting depth: a placement had wedged a dish |
| Episode support refreshed at reset (contacts over 0.5 s) | a one-tick start snapshot missed a counter-stack edge, so a planner lifted the bottom bowl of a stack |
| One container, 3 Kit jobs on GPU 1 | a second container's pip layer lands on the root disk (977 MB now); GPU 0 carries another user's job |
| Goal-pair exemption: a neighbour whose last command was its own goal | settled neighbours sit a few mm (track A) to 4 cm (lifted family poses, track B) off their commands |
| Unstack rehearsal in acceptance | a propped bowl fell 45 mm when the bowl above was lifted: every algorithm aborted on move 1 |
| Messy stacks instead of a random heap | random heaps of thin HOTEC shells: 0 of 6 clearable at coverage 0.5-1.0, some never rest, one plate thrown 1.2 m |
| Build one dish at a time; gate the resting state | joint drops of a stacked heap failed the 2 mm gate in 3 of 4 (2.1-3.6 mm dish-on-dish transients) |
| **2026-09-28 (evaluation-setting fixes after the seed-0 pilot review)** | |
| Own loads are sequence-gated like the goal (`--sequence-plan`, `--resequence`, bans + re-plans) | every medium/hard open-track row failed `disturbed`: the planner's load was scored on paper and replayed blind, and a plate leaning 24 deg was pulled through a placed neighbour |
| The goal-pair FCL exemption applies only to certified (Isaac-built) goals (`mark_goals(certified=)`) | open-track "goals" were the plan's unsettled poses, up to 4 cm off; a command to one skipped the pair check against a neighbour that was not there |
| Track-B success needs a pooling-free final load (`failure_of`: `pooling`) | the pilot's only open-track "success" (easy planner) left two bowls holding water (W 0.0002) |
| No copied rows: track A = greedy_offline, rrt_connect; track B = baseline, planner | half of each table was the other table; the failure column also named the abort (`give-up`) instead of the end-check outcome |
| rrt_connect: nearest neighbour as an int code matrix; the goal tree's edges are checked on the REPLAYED move | hard_s0 rrt gave up at 57 s with 0 moves: O(nodes x items) Python NN, and the goal tree could exceed the counter cap and command a goal before the dishes it leans on (forward check on backward edges) |
| Unstack rehearsal uses the episode's displacement rule (fatal only off the counter/racks) | generation re-rolled starts the episode would accept (any displaced dish failed it) |
| **2026-09-28 (full experiments on the re-measured racks)** | |
| Baseline re-plans: deterministic first-fit under the accumulated bans, then up to 5 seeded shuffles as a fallback; `PLAN_ATTEMPTS` 3 -> 6 | every medium/hard own load ended `load-not-buildable` with 3 deterministic attempts; shuffling FIRST (the first version, live 2 h) returned "no load" on every medium attempt (scattered plates take the bowls' space, 20 failed packs = 280 s per attempt) |
| A baseline re-plan must differ from every earlier attempt's load (`load_key`, `previous`); a repeat falls to the shuffle fallback, and if no new pack exists the attempt has no load (no Isaac) | live run 14:20: a certified first-fit load that is "not sequenceable under its build order on the counter cap" bans nothing, so attempts 1-5 rebuilt the identical load in Isaac (digest 0d4c0196 on medium s2 AND s3: first-fit ignores the start, so every medium instance gets the same load) |
| Instances are gated on the sequencing certificate (`stage_finalize` re-rolls an unsequenceable goal) | medium s1-s3 of the 2026-09-28 run were accepted with `sequenceable False`; greedy_offline gave up at move 2-6 and every own load was "not sequenceable". The certificate is `planner.sequence` (order- and support-aware greedy with buffering), NOT greedy_offline (`rearrange.Greedy`, one buffer trip per blocker): on 2026-09-29 greedy_offline still gave up on the certified medium s1 and s2 goals (6 and 11 moves) that RRT solved |
| Counter allowance medium n + 1 -> n + 2, hard n + 0 -> n + 1 | Kit-free replay (`data/build/frigidaire_diagnostics/scratch/seq_diag.py`): the greedy parks until the counter is full; cap + 2 sequences medium s1/s2, cap + 4 s3; RRT solved s1 in 47 moves, so the goals were reachable, the sequencer is incomplete |
| ~~Plate goals only in every second gap of a bank~~ REVERTED 2026-09-29, every gap stays | the rule was meant to cut plate-bank chains, but `--capacity` failed medium/hard at bowl_07: 8 plates in 5 front + 3 rear even gaps close the rear bowl zone; chains are left to the n + 2 / n + 1 allowances and the certificate gate |
| Lossless resume (2026-09-29): `generate_unit` skips steps whose output exists (`done_step`), `gate_unit` reuses an existing gate result, the own-load build and ban steps skip existing records, and `full_unit` reruns `--run` until all 4 episodes exist | every stage refuses to overwrite its output, so each restart abandoned the attempt in flight (3 attempts lost in one restart) |
| `--kit-cores` / `--py-cores` on `frigidaire_bench.py` (and the top-5 script) | two schedulers can share the container on disjoint cores with at most 3 Kit jobs in total on GPU 1 |
| `--full`: one scheduler chains generate -> plan -> run -> analyze per instance (videos per tier after its first instance) | stage-by-stage runs idle the 3 Kit slots at every stage boundary and never overlap Kit-free planning with Kit; the user asked for the parallelisable steps |

## Landmines

- `AppLauncher` strips its own flags from the namespace at boot: read `--enable_cameras` before launching.
- `rel()` maps host paths to container paths WITHOUT resolving symlinks (`data/results/` is a 2 TB symlink); a host
  `/home/...` path passed into the container silently lands in the container's writable layer (root disk).
- Everything the container writes is root-owned: move or delete it with `docker exec dishsim-isaac ...`.
- Stop jobs by recorded PID or background task id, never broad `pkill -f` (it kills the calling shell and can
  hit other sessions' jobs in the shared container).
- The driver times `next_move` only; the Kit runner's `Timed` wrapper adds `reset` (where offline planners
  plan) and reports thread CPU time. Open-track rows report the Kit-free plan's wall and process CPU time.
- Stills need 48 RTX renders after a teleport, or moved dishes leave ghosts.
