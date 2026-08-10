# Known limitations and open items

Honest negative results and open engineering items, with the measured evidence for each.
Numbers below trace to recorded runs (see the Results section in the README) or to
`docs/success_criteria.md`.

## Plate placement is path-blocked, not goal-blocked

Plate goal configurations exist in both feasible robot-bank gaps (64-sample funnels accept
1968 and 2208 samples at `gap_centre`/`gap_right1`), but RRT-Connect cannot connect the
countertop to either goal while carrying the 141 mm disc — measured failures at the default
20 s budget, at 60 s, and at 180 s (run `plate_b180`). The passage through the machine mouth
into the front band is a narrow-passage planning problem the goal-only funnels never
exercised. The most promising lever is a corridor/release-lean goal-pose refinement that gives
the planner a straight descent corridor (prototyped in the retired `rack_design.py` harness —
in git history), or a batch-sampling planner (`--planner bit_star`) for narrow passages.

## Fork-bay execution contacts past the hover gates

The v4 fork-bay goal funnels accept generously (6016–6560 accepted samples per bay), but the
funnel gates for weld-acquire classes deliberately stop at the hover — and real carried-fork
transit has contact the gates don't cover. In episode `bothin_v4`, one fork failed at the
acquire-approach hover (finger contact 3.9 N) and one aborted mid-place ("carried object
externally loaded", 7.1 N). Open choice: extend the weld-acquire gates past the hover for
basket transit, or calibrate the approach.

## The default plan budget predates the v4 rack

`PLAN_TIME_BUDGET_S = 20.0` (config.py) was tuned for the v3-era mug workload. The first
successful robot bowl placement (run `platebowl_v4_b60`) needed `--planner_param budget_s=60`.
Consider raising the default or giving disc/weld-acquire classes a per-class budget. The
frozen `run_trials.py` baseline keeps its own path and is unaffected.

## The bowl lean fixture was retired (RACK_GEN v4)

A gripper carrying a rim-edge bowl stabs the rack at every hover of the 48° lean, so v4
removed the lean fixture and bowls stand upright on the floor grid (`floor_stand`, 3 feasible
cells). The `bowl_lean` placement mode was removed from the code with RACK_GEN v4 adoption.

## `placement_open` is a 0-feasible reference state

With the v4 robot destinations in the lower rack's front band, the *extended* upper rack
shadows all of them: every class measures 0 feasible slots in `placement_open` (both racks
out). The state is kept as a bake-able reference and as the only real case exercising the
rack-state tie-break in `config.resolve_rack_state`; no v4 flow places in it.

## Capacity-fill realism edges

- **The saucer class is retired from the fill** (it stays in the object library). Measured
  over four fills: mid-gap saucers slip 21–26 mm / 32–42° during the rack stow (v3's apparent
  stability came from the old basket bracing them from below), and even the candy-cane-braced
  end gap is marginal — parked at 3.9° drift in one run, tipped 46° at stow in another.
  Capacity trades three saucers for a reliably closable rack.
- The wine-glass stemware lie-in (a physics stretch goal) has never settled within the
  stability gate; both glasses park in every run.

## Multi-station base placement (future work)

The base-pose sweep's runner-up scorecards (`results/base_sweep/stage4_final.json`) record a
pose (`x +0.3375, y −0.375, yaw −18.75°`) that doubles the countertop pick band (0.40 m / 157
cells vs 0.20 m / 86 at front) while matching front on every slot criterion — the starting
data for a future movable-base/multi-station mode.

## Measured rack-design rules (from the retired design harness)

Recorded 2026-08-09 during the v4 redesign diagnostics; they constrain any future rack change:

- A goal pose that *leans* the payload onto a fixture is unplaceable by construction — it sits
  inside the `COLLISION_MARGIN_M` inflation. Robot-facing goals must hover in free corridor
  space and let gravity settle the lean (the `basket_drop` pattern).
- Tie wires cross the tine gaps *through the disc plane* — a robot-facing plate bank must not
  have them (`tine_tie_frac: None`).
- The fork-drop stack (gripper above a hanging fork) clears the machine only through the open
  mouth front: drop columns must stay at rack-frame y where the world-x of the column is in
  front of the shell top (bays at y ≲ 0.10 measured safe, ≥ 0.126 measured capped).
- Slots at rack y ≳ 0.14 risk the stowed upper rack's roof (y ≥ 0.200, z ≥ 0.154) through the
  cluster's lateral extent; feasibility there is yaw-dependent — measure, don't assume.
