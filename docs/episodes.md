# Multi-object episodes — `scripts/experiment/run_task.py`

The episode runner spawns N objects at seeded random countertop poses, settles them, and
clears them one at a time in an order it decides. This is the operator manual; the episode
*semantics* (success criteria, sequencer gates, failure taxonomy) live in
[success_criteria.md](success_criteria.md#multi-object-episodes-scriptsexperimentrun_taskpy).

```bash
# reproducible from the seed alone
scripts/run_kit.sh scripts/experiment/run_task.py --headless --seed 0

# swap the ordering heuristic; the planner and the scene are untouched
scripts/run_kit.sh scripts/experiment/run_task.py --headless --seed 0 --cost_fn shortest_ik
```

## CLI

- `--seed`: layout seed — with the composition this reproduces the scene exactly
- `--spawn "cup=2-4,mug=1"`: **explicit composition** — a count or an inclusive range per object
  type. A range is drawn per episode from the seed, so the number varies while staying
  reproducible. Term order does not matter (`a=1,b=2` == `b=2,a=1`), and the expanded list is
  shuffled so no type always gets first claim on countertop space. Config equivalent:
  `TASK["spawn_counts"]`
- `--n_objects` / `--classes`: the uniform-draw fallback when no explicit composition is given
- `--rack_lower_m` / `--rack_upper_m`: rack extensions in metres (0 = stowed, −0.20 = fully out;
  the racks are independently articulated). Resolved to a named machine state — extensions are
  part of the collision-cache hash, so a combination that has not been baked fails at startup
  with the command that bakes it. `--scenario NAME` selects one directly. Config equivalent:
  `TASK["rack_state"]`
- `--video_camera`: which camera the episode MP4 is written from (default `episode`)
- `--cost_fn`: pick-order heuristic — `nearest_first` (default) · `shortest_ik` · `farthest_first`
- `--allow_stacking`: spawn objects stacked on each other. Because settling then decides the
  final poses, reachability is re-checked *after* physics and the whole layout is resampled if
  any object ends up unreachable (capped by `TASK["max_layout_retries"]`, then a loud failure)
- `--planner` / `--planner_param`: as for `run_trials.py` — the task layer never names a planner

Per-pick records use the **existing trial schema**, so Phase 3 consumes an episode unchanged;
`episodes/<ep>.json` adds what a per-trial record cannot express (pick order, the cost each
choice scored, why anything was blocked, the support graph).

## Goal slots by object type

`TASK["type_slots"]` maps an object type to an ORDERED list of slot **names**, e.g.
`{"mug": ("mid_centre", "near_centre")}`. Names are derived from the rack geometry
(`placement.slot_names`) rather than hardcoded ids, because ids are positional and would
silently re-point to different cells if the grid pitch were retuned. Each type consumes its list
in order; a slot already taken, overlapping an assigned one, or with no reachable goal
configurations for that class is skipped, and an object whose list runs out is reported unplaced.
The vocabulary follows each mode's actual grid — `near_centre`/`mid_left1` for the `floor_stand`
3×5, `gap_centre` for the plate-bank gaps and the x-split basket bays alike.

## Starting from a stowed machine

`--scenario both_in` begins with both racks pushed in, where **no slot is reachable at all**
(0 of 15, measured). The episode opens with a rack action — the gripper engages the lower
rack's handle and pulls it out to −0.20 m while the tool tracks the moving handle — and then
loads the machine in the resulting `placement` state. The episode spans two collision-cache
states; the post-action state is derived from the action, and a rack that settles beyond
`RACK_SLIDE_TOL_M` ends the episode rather than letting later picks plan against a world the
machine no longer matches.

```bash
scripts/run_kit.sh scripts/experiment/run_task.py --headless --enable_cameras \
    --scenario both_in --spawn "cup=1,tumbler=1,fork=2" --seed 1 --run_id bothin_load
```

> Measured: pulling the *upper* rack out as well (`placement_open`) does not open the rear rows —
> it drops every class (cup, tumbler, fork, plate and bowl) to **0** reachable slots, because the
> extended upper rack shadows the lower rack's front band where all v4 destinations live. The
> lower rack is already at its mechanical limit, and 87 mm of its 287 mm depth never leaves the
> machine mouth.

## Home-anchored trajectories

Every episode's recording begins and ends at `config.HOME_Q`. The start is measured, corrected
if it drifted during layout settling, and then asserted; the closing retreat plans back to home
and holds for `SETTLE_STEPS`, and runs on the failure and exception paths too. It happens
*before* the recording and media are finalised (so the retreat is in the `.npz`, the MP4 and
the final stills) and *after* the sequencer (so placement verdicts are already decided and
parking the arm can never revise a pass/fail). The record carries `start_home_err_rad`,
`end_home_err_rad`, `home_return_status` and — because the fallback retreat is a straight
interpolation that is not collision-checked — `post_home_displacement_mm` for each placed
object.

## Camera framing

`config.CAMERA_LENS` makes focal length and aperture configurable, and `config.EPISODE_CAMERA`
is a wide view that keeps the countertop and the machine in frame for the whole episode.
Vertical FOV is the binding constraint on a 16:9 frame, not horizontal.

## Destination capacity bounds episode size

Episode size is capped by DESTINATION capacity, not countertop room — the counter holds far
more than the robot can put away. Measured ceiling, per rack structure (RACK_GEN v4, 64-sample
goal funnels):

| destination | reachable | why |
|---|---|---|
| rack floor (`floor_stand`) | 5 of 15 cells (cup∩tumbler; mug 5, bowl 3) | the left/centre-front block clears the machine mouth; the far column and right edge funnel to zero on collision |
| plate bank (`plate_slot`) | 2 of 3 robot gaps | the centre-most gap sits behind the measured arm-access boundary (~rack x ≥ 0.28 alive) |
| cutlery basket (`basket_drop`) | 3 of 3 bays | fork/knife/spoon — weld-acquired, not picked |

So the v4 rack offers **13 robot destinations** (v3: 6) — 2 plate gaps + 3 bowl cells + 3
basket bays + 5 floor cells, with bowl cells drawn from the floor cells, so a mixed load shares
them. The floor placements are genuine countertop picks. For contrast, `capacity_fill.py`
*teleports* 29 items in and 27 settle stably: the rack's own capacity is still larger than what
this arm can reach into it.

## Full-load episodes (Bosch 800, v2)

A full load spans machine states — the lower rack loads in `placement`, the middle rack in
`middle_out`, the third rack in `third_out` — so a FULL-LOAD episode is an ordered list of
loading PHASES with a **scripted rack transition** between them (drive-target ramps, arm parked
at HOME; the robot only picks and places). Within a phase the counter is stocked in restock
WAVES: as many items as fit the reachable spawn band (`TASK["wave_fill_factor"]` of its area),
cleared, then restocked.

```bash
# 1. plan the reachable full load (run_py.sh, no Kit) — writes the plan artifact + reach figure
scripts/setup/plan_full_load.py --machine bosch800 --placement side_winner

# 2. run it (ask before launching: the certified 22-item load is a 1.5-2.5 h run; --cap in
#    the planner produces a smaller demonstration load, e.g. --cap fork=8,bowl=4)
scripts/run_kit.sh scripts/experiment/run_task.py --headless --enable_cameras \
    --machine bosch800 --placement side_winner --full_load --seed 1 \
    --run_id bosch_full_load_s1 --orbit

# debug composition without a plan (slots assigned at runtime):
scripts/run_kit.sh scripts/experiment/run_task.py --headless --enable_cameras \
    --machine bosch800 --placement side_winner \
    --phases "middle_out:cup=1;third_out:fork=2" --seed 1 --run_id phase_smoke
```

**What "full load = N" means** (`src/dishsim/capacity.py`): a slot counts only if its baked
goal set is non-empty (reachable), the load is grown one item at a time with every earlier
item's body added to the collision world (the *same* re-filter `primitives._place` runs live,
so the planned N predicts the runtime funnel), and a class may only ride a rack through a
transition if its worst-case tolerance pose clears the overhead geometry by ≥ 15 mm
(`capacity.z_budget_clearance_m` — measured: a middle-rack tumbler misses the third rack's
underside by 15.8 mm, a cup clears by 20.6 mm, so the middle rack takes cups only).

Mechanics worth knowing before touching the code:

- **Transition target dicts are COMPLETE** (every rack joint in every dict): the scene-side
  override is replace-not-merge and falls back to the BUILD scenario's targets for any joint
  omitted — a phase-2 dict without the lower joint would silently drive the loaded lower rack
  back out. Structural, tested (`tests/test_task_phases.py`).
- **Riders are gated**: per placed item, slip (net of the rack's measured ride) must stay
  within `TASK["transition_slip_gates"]` per effective mode; the rack itself must settle
  within 2× `RACK_SLIDE_TOL_M` under load.
- **A failed transition DEGRADES, it does not error**: earlier phases' picks planned against
  their own worlds and remain valid evidence; remaining phases are skipped, the episode
  reports `partial` (an `error` needs a failed transition AND nothing placed).
- **Artifacts flush incrementally**: the episode JSON is atomically rewritten after every wave
  and phase, trials at each phase end, and the trajectory recorder is segmented per phase
  (`ep000_p0.npz`, …) so the 40k-step buffer never overflows on a 60-90k-step episode.
- **Waves park off-workspace**: every phase's prims exist from scene build, parked at
  x ≈ −2 m on the ground (the capacity_fill idiom), and teleport onto the counter per wave
  with the arm verified at HOME. Contact filters pair items within a phase only.
- `run_task.py` now writes a `manifest.json` (run id, argv, per-state config hashes, plan
  path) in every mode — episode runs previously wrote none and Phase 3 lost provenance.
