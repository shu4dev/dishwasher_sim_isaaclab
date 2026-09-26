# HOTEC rearrangement benchmark (Frigidaire FDPC4221AS twin)

Reference for agents working on `frigidaire/scripts/experiment/frigidaire_bench.py` (Kit-free library, CLI,
scheduler) and `frigidaire_bench_kit.py` (Kit: start generation and episodes). Read [exposure.md](exposure.md)
for the score and [planner.md](planner.md) for the planner this reuses. Designed with the user in six grilling
rounds on 2026-09-23; the approved plan is `~/.claude/plans/i-am-creating-a-majestic-feather.md`.

## The problem

```
tiers      easy   7 bowls                     counter allowance n + 3
           medium 8 plates + 7 bowls          n + 1
           hard   8 plates + 7 bowls + 8 cups n + 0        n = dishes on the counter at the start, uniform in [N/4, 3N/4]
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
track B    open: success = every dish racked and the end check passes; reported S (all dishes) vs S_ref.
algorithms greedy_offline, rrt_connect (Bosch, live), exposure planner, first-fit baseline (Kit-free plans,
           replayed). Track A: the planner and baseline are both the fixed-goal sequencer (one run, two rows);
           track B: the Bosch pair's input is the sampled goal, so their track-A runs are scored again.
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
B="nice -n 10 python3 frigidaire/scripts/experiment/frigidaire_bench.py"
$B --capacity                                   # gate G2 (Kit-free, in the container): 8 / 16 / 24 racked
$B --generate --tiers easy medium hard --seeds 0 1 2 --kit-jobs 3 --py-jobs 2 --max-attempts 12
$B --plan-all --tiers easy medium hard --seeds 0 1 2 --py-jobs 2      # open-track plans (Warp, 60 s each)
$B --run-all  --tiers easy medium hard --seeds 0 1 2 --kit-jobs 3 --cameras   # 5 Isaac episodes + stills each
$B --analyze-all --tiers easy medium hard --py-jobs 2                 # S_final, S per move, score-detail figure
$B --videos --tiers easy medium hard --kit-jobs 3                     # first instance per tier, 5 videos each
$B --collect                                                          # compare/summary.{md,json}
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_bench_page.py --label "..."   # results page folder
```

Scheduler: one container (`dishsim-isaac`, GPU 1), at most 3 Kit jobs (a camera Kit job is about 2.9 GB of
GPU memory) plus 2 Kit-free jobs, each on its own cores (`taskset` 0-7 / 8-15 / 16-23 Kit, 24-27 / 28-31
Kit-free) with BLAS pools sized to them, `nice -n 10`, Kit launches staggered 90 s, each waiting for 6 GB free.
Logs: `logs/benchmark_frigidaire_hotec/<unit>.log`, judged by the `[RESULT]` line.

## Outputs

`results/benchmark/frigidaire_hotec/`: `baseline/`, `families_appliance_free.json` (cached by family digest),
`instances/attempts/<tier>_s<seed>_a<k>/` (start, goal, gate, generation report), `instances/<tier>/<id>.json`,
`plans/open/<tier>/`, `episodes/{goal,open}/<tier>/<id>__<algorithm>.json` (+ `.analysis.json`),
`episodes/_kit/` (per-instance run reports), `compare/summary.{md,json}`. Media under
`media/benchmark/frigidaire_hotec/`: `stills/<tier>/<id>/` (initial, goal, finished per episode),
`analysis/<tier>/`, `video/<tier>/`, `page/` (the published folder). Smoke runs of 2026-09-23 are kept under
`*_smoke_20260923/` with one folder per failed design (see below).

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

## Landmines

- `AppLauncher` strips its own flags from the namespace at boot: read `--enable_cameras` before launching.
- `rel()` maps host paths to container paths WITHOUT resolving symlinks (`results/` is a 2 TB symlink); a host
  `/home/...` path passed into the container silently lands in the container's writable layer (root disk).
- Everything the container writes is root-owned: move or delete it with `docker exec dishsim-isaac ...`.
- Stop jobs by recorded PID or background task id, never broad `pkill -f` (it kills the calling shell and can
  hit other sessions' jobs in the shared container).
- The driver times `next_move` only; the Kit runner's `Timed` wrapper adds `reset` (where offline planners
  plan) and reports thread CPU time. Open-track rows report the Kit-free plan's wall and process CPU time.
- Stills need 48 RTX renders after a teleport, or moved dishes leave ghosts.
