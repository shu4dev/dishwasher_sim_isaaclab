# Session notes — 2026-08-09 — v4 wrap-up: sweep verdict, closable fill, validation

## Which session to continue with

Two Claude Code sessions were showing this conversation (an artifact of teleporting to a new
machine while code-server dropped the old terminal). The old process (PID 2822, `pts/0`) was
killed at 15:04 UTC after its background sweep completed — all its state (transcript, results)
is on disk and nothing was lost. **Continue with the teleported session**:

```bash
cd /workspace/isaaclab/dishwasher_sim_isaaclab && claude --continue
```

or from claude.ai (Remote Control). Note `/effort` (ultracode) was session-only.

## TL;DR

1. **The v4 rack meets the full reachability success bar at the FRONT base pose — the base
   was never the binding constraint.** The 420-candidate sweep's best pose matches front on
   every slot criterion; front stays, Phase B (switchable `side` placement) was skipped by
   the pre-agreed rule. No new bake needed.
2. **The capacity fill is CLOSABLE again**: 27/30 stable, 0 displaced on stow (was 29/33,
   NOT closable, 2 displaced). Cost: 2 saucers + the lower mug dropped, each with a measured
   geometric justification (see below).
3. Full venv pytest: **461 passed**. Docs updated to v4 numbers throughout.
4. First real v4 episodes ran (see Validation): cup/tumbler picks, both rack actions, and —
   with a raised plan budget — **the project's first robot bowl placement** all work; the v0
   freeze smoke reproduces **2/2**. Two open findings: fork-bay execution contact and the
   plate path (goals exist, no path at 180 s — needs the un-adopted corridor approach).

## Sweep verdict and the placement decision

Final ranking (`results/base_sweep/stage4_final.json`, 64-sample funnels, front force-included):

| pose | criteria met | plate | bowl | fork | floor | pick band |
|---|---|---|---|---|---|---|
| winner `x+0.3375 y−0.375 yaw−18.75°` | 5/5 | 2 | 3 | 3 | 5 | **0.40 m**, 157 cells |
| **front `x0 y0 yaw0` (kept)** | 5/5 | 2 | 3 | 3 | 5 | 0.20 m, 86 cells |

Decision rule (approved in plan): adopt the winner only if it meets ≥1 more success-bar
criterion than front without losing any. It meets zero more — its only edge is pick-band
depth, and both poses clear the 0.12 m bar — so **front stays** and `run_trials.py` remains
zero-diff. The top-7 scorecards are retained in `results/base_sweep/` as data for a future
multi-station (movable-base) mode; heatmaps in `media/base_sweep/`.

The success bar and measured v4 values are now recorded in
`docs/success_criteria.md` → "Reachability success bar".

## Capacity fill: from NOT CLOSABLE to closable (3 runs)

| run | plan | stable | closable | displaced on stow |
|---|---|---|---|---|
| baseline (prev. session) | 33 | 29 | **NO** | saucer_02 (21 mm), mug_00 (33 mm) |
| iter 1: saucers→gaps 0-2, plates→3-9, lower mug dropped | 32 | 29 | **NO** | saucer_01 (22 mm), saucer_02 (26 mm) |
| **iter 2: single saucer in cane-braced gap 0 (shipped)** | **30** | **27** | **YES** | none |

Measured mechanisms (now documented in `fill_plan.py`):
- **Saucers**: v3's outer-gap saucers were braced by the old basket sitting under the bank.
  In v4 nothing braces a mid-gap saucer — its 55 mm disc reach is a knife-edge contact that
  tips 32–42° when the rack accelerates during the stow (plates' 71 mm reach braces across
  both tine rows and survives everywhere, incl. over the robot bank). Only the end gap, where
  the candy-cane hook arcs in as a lateral brace, holds a saucer through the stow.
- **Lower mug**: the strip right of the relocated basket is 86.3 mm wall-to-basket; the mug
  belly is 93.0 mm. It physically cannot stand there — that was the measured ~3 mm
  interference and the 33 mm stow slip.
- Parked as unstable (honest-capacity mechanism, not closability blockers): both wine glasses
  (stemware lie-in stretch goal, parked in every run) and saucer_00 (tips 3.9° vs the 3.0°
  gate in the final run — the saucer class is marginal even braced).

Fresh media under `media/fill/`; curated `docs/figures/loaded_{front,top,iso}.png` refreshed.

## Validation runs (Phase C)

- **`both_in` episode** (`--spawn "cup=1,tumbler=1,fork=2" --seed 1`, run `bothin_v4`):
  status **partial 2/4**. Rack pull: err 0.4 µm. cup→`near_left2` ✅ and
  tumbler→`near_right2` ✅ (both genuine countertop picks). Both forks failed:
  `fork_01` at the acquire-approach hover (finger contact 3.9 N gate), `fork_00` aborted
  mid-place ("carried object externally loaded", 7.1 N). **Open finding**: v4 fork-bay goal
  funnels accept generously (6016–6560/6400-scale), but funnel gates for weld-acquire classes
  stop at the hover — real carried-fork transit has contact the gates don't cover. Evidence:
  `results/experiments/bothin_v4/episodes/ep001.json`, `media/task/bothin_v4/ep001.mp4`.
- **plate+bowl episode** (`--spawn "plate=1,bowl=1" --seed 0`, the headline v4 capability),
  three attempts telling one story:
  1. First run crashed at startup → the `primitives.py` latent-bug fix above.
  2. Default 20 s plan budget (`PLAN_TIME_BUDGET_S`): rack push perfect (1e-10 m err), both
     slots validly assigned (bowl→`near_left2`, plate→`gap_centre`), both picks
     **planner-timeout** — the carried-disc path into the front band is a narrow-passage
     problem the goal-only funnels never exercised.
  3. `--planner_param budget_s=60`: **bowl PLACED — the project's first robot bowl
     placement** (weld-acquired, carried, released, verdict pass). Plate still times out.
     Evidence: `results/experiments/platebowl_v4_b60/episodes/ep000.json`,
     `media/task/platebowl_v4_b60/ep000.mp4`. A 180 s plate-only probe ran last (result
     below).
- **plate-only 180 s probe** (`--spawn "plate=1" --planner_param budget_s=180`): still
  **planner-timeout**. Plates are path-blocked, not slow: goal configs exist (funnel accepts
  1968–2208) but RRT-Connect cannot connect the countertop to `gap_centre` in 3 minutes with
  the carried disc. The likely missing piece is the plate corridor / release-lean approach
  that `rack_design.py` prototyped (`plate_corridor_y`, `plate_release_lean_deg`) but
  `placement.py` never adopted — see open items.
- **v0 freeze smoke** (`run_trials.py --slots 0,6 --seeds 0`, mug, front): **PASS 2/2**
  (lateral 3.3 / 8.5 mm, tilt ≤ 0.01°, plans 3.3 / 3.9 s, weld ~1 mm). The frozen baseline
  fully reproduces on the v4 world — note the v4-feasible mug slots are now {0, 5, 6}
  (`near_left2`, `mid_left2`, `mid_left1`), not v3's {1, 2, 6, 7}, so freeze smokes must
  target those ids. Evidence: `results/experiments/freeze_v4/`, `media/trials/freeze_v4/`.
- **Full venv pytest**: 461 passed (161 s).

## Code changes this session (working tree, uncommitted)

- `src/dishsim/task/primitives.py` — **latent crash fix found by the first plate/bowl
  episode**: `GraspProfile.build` did `float(g.force_min_n)` but weld-acquire classes with no
  calibrated pinch (plate, bowl — the jaws never close on them, nothing to calibrate) have
  `None` bands; the path was unreachable pre-v4 (plate/bowl had zero feasible slots). Weld
  classes now get vacuous sentinels (0 / inf — the carry stays protected by the external-load
  gate); non-weld classes with missing bands raise with the calibrate-and-freeze hint.
- `src/dishsim/fill_plan.py` — closability fixes + v4 docstring (see mechanisms above).
- `tests/test_fill_plan.py` — saucer minimum 3→1 (matches the shipped fill).
- `src/dishsim/config.py` — stale-comment fixes only, no value changes (scenario
  `min_feasible_slots` comments; `placement_open` block rewritten to record the v4 inversion:
  the *stowed* upper rack no longer shadows the front-band destinations, the *extended* one
  does — measured plate 2/3 & bowl 3/15 in `placement` vs 0 everywhere in `placement_open`).
- `src/dishsim/rack_gen.py` — candy-cane comment corrected (both v4 banks keep canes;
  measured zero cost on the robot bank).
- `tests/test_scenarios.py` — one comment.
- Docs: `README.md` (fill caption + numbers, bowl→`floor_stand`, destination table → 13 v4
  destinations, `placement_open` note), `docs/success_criteria.md` (mode table, state
  comparison table incl. plate/bowl rows, slot names, capacity ceilings, new "Reachability
  success bar" section), `CLAUDE.md` (30-item fill), `docs/figures/loaded_*.png` refreshed.

## Open items (not blocking, for a future session)

0. **Plan budget for v4 narrow passages**: the default `PLAN_TIME_BUDGET_S = 20.0`
   (config.py:752) predates v4's front-band destinations. The bowl needed ~60 s; consider
   raising the default or giving weld-acquire/disc classes a per-class budget. (Frozen
   `run_trials.py` keeps its own path — untouched.)
1. **Plate path planning** (the one unmet v4 promise in execution): goals exist in
   `gap_centre`/`gap_right1` but no path connects at 180 s. Next lever: adopt the
   `rack_design.py` corridor/release-lean approach into `placement.py` (goal poses that give
   the planner a straight descent corridor), or try `--planner bit_star` for narrow passages.
2. **Fork-bay execution** (from the `both_in` episode): decide whether to extend the
   weld-acquire funnel gates past the hover for basket transit, or calibrate the approach.
   The measured failure signatures are in the episode record.
3. **`placement_open` bakes are now dead weight** (0 feasible for every class). Kept as a
   reference state; could be dropped from `build_state.py` defaults to save bake time.
4. **Saucer/stemware realism**: saucer_00 parks at 3.9° vs the 3.0° gate; wine-glass lie-in
   never settles. Either relax the per-item gate for these classes or model a stemware shelf.
5. **Multi-station mode**: the sweep's runner-up scorecards (2× pick band at
   `x+0.3375 y−0.375 yaw−18.75°`) are the starting data.
6. `results/experiments/LATEST` still points at `pushfix_assay` (4/6 on the v3-era caches:
   1 execution-collision, 1 planner timeout) — superseded by this session's runs.

## Suggested commit message

```
Close v4 loop: closable fill, sweep verdict, v4 docs

The 420-candidate base-pose sweep confirms the v4 rack meets the full
reachability bar at the front base pose (plate 2/3, bowl 3, fork 3/3,
floor 5, pick band 0.20 m); the winner only deepens the pick band, so
the front placement stays and its scorecards are kept as multi-station
data.

Make the capacity fill closable again (27/30 stable, 0 displaced):
keep one saucer in the cane-braced end gap (mid-gap saucers tip on
stow now that the old basket no longer braces them), move plates to
gaps 3-9, and drop the lower mug (86 mm strip < 93 mm belly).

Fix a latent crash in GraspProfile.build for weld-acquire classes
with no calibrated pinch band (plate and bowl, first reachable in
v4). Update README/success-criteria/CLAUDE numbers to v4, record the
success bar, and fix stale v3 comments (candy canes,
min_feasible_slots, placement_open inversion).
```

(Per repo convention: user commits; no AI attribution lines.)
