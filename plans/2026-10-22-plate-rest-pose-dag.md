# Measured plate rest poses per gap, the plate DAG at those poses, and the three-tier rerun
Slug: 2026-10-22-plate-rest-pose-dag | From: results/2026-10-22-plate-pose-model.md (follow-up, no lit review)

## Goal
Replace the plate half of the pre-build DAG's pose model with a measured table: where a lone plate rests when the
insertion move drops it into each lower-rack gap from each lean side. Check the table against the 32 settled plates,
then rerun easy, medium and hard with plates ordered from the DAG at those poses and bowls and cups on the existing
re-pose and deferral, against the per-stage tables of the two earlier notes.

## Decisions (grilled 2026-10-22 14:05 ET)
- Settles: one plate alone in the Frigidaire twin, dropped by the insertion move exactly as the build does it
  (`upright_pose(slot, side)` with the 3 mm lean bias, lowered from 20 mm above the rim in 5 mm steps against the
  appliance, released 20 mm above the stop, settled 150 + 60 ticks at 120 Hz), into every lower-rack gap (front 10,
  rear 9) from each lean side (-1, +1): 38 settles in one Kit session. Record per (bank, gap, side): the rest pose
  (position, quaternion XYZW, tilt from vertical), the lowest visual point's height (gap or tines), the settle
  deviation, as `data/results/sampler/plate_rest/plate_rest.json` with the code SHA and the move's constants.
  Rejected: fitting a template to the loads' settled plates.
- Check: every settled plate of the 32 (records r05_00027, r00_00038, r07_00012, r02_00017) against the table entry of
  its (bank, gap, lean side): tilt within 3°, origin within 10 mm. Report k of 32, the bank medians versus the table,
  and the 5 r07_00012 front plates separately (they were off the bank median before). The table stands if at least 27
  of 32 agree; otherwise stop before the rerun and report.
- Pre-build DAG with the table: plates take the table pose of their slot and lean side; bowls and cups take their
  packed poses but contribute no edges (their poses are unpredictable; the earlier note). Access edges among plates
  only (and plate-over-bowl edges where a plate's rest pose covers a bowl's descent, since the plate pose is now
  trusted); support edges off. Build order: a topological order of that DAG for the plates, the lean-side chain as the
  tie-break, bowls and cups where the seeded permutation puts them; re-pose (up to 5 free poses) and deferral stay on
  for every dish, no strict mode. A plate cycle re-draws the packing (seed*1000+k, k < 10) as before.
- Rerun: `sample.py --difficulty easy|medium|hard --from build --to certify --seed 0..19`, 4 slots, the per-stage
  table before (easy 20/19/16/12, medium 12/0/0/0, hard 13 CLI packings/0/0/0 and the v7 loop's 0 of 40) and after,
  plus the cycle row and the DAG stats. Hard = the preset's 23 dishes.
- Box: the settles and check first (about 10 minutes), then the rerun (about 70 minutes); stop right after; hard cap
  2 hours, then stop anyway.

## Change
- New Kit script `code/planner/frigidaire/frigidaire_plate_rest.py` (boot-first; the move's helpers; writes the table).
- `code/src/sampler/order.py`: `plate_rest_poses(table)` and a `pose_model="rest"` path for `access_edges`/`dag_order`
  (plates from the table, no support edges, no bowl-bowl edges); `build.py`: `build.order=dag-rest` as the default for
  this run, strict off; the kit's `--order` path keeps re-pose and deferral (the strict flag off).
- `order_check.py`: the 32-plate check against the table (model R). The r05 test gains the R case.

## Run
Box: `code/util/run_kit.sh code/planner/frigidaire/frigidaire_plate_rest.py --headless --out data/results/sampler/plate_rest`,
then `code/util/run_py.sh code/src/sampler/order_check.py --models R ...`, then the rerun by the scheduler of
plans/2026-10-22-sampler-rate-easy-medium.md; `brev stop`; confirm STOPPED.

## Works when
`results/2026-10-22-plate-rest-pose-dag.md` holds the 38-row table summary, the 32-plate check (k of 32, worst
error), the before/after per-stage table for the three tiers with the cycle row and DAG stats, the failed seeds with
reasons, the box window and a suggested commit message; medium builds more than 0 of 20 and easy at least 12 of 20
valid, or the note says which half failed and why; the box is STOPPED.

## Assumed / skipped
Assumed: a lone plate's rest pose in a gap is what a plate in a bank rests at too (the chain lean is second order, as
the 22-of-27 agreement suggests). Skipped: bowl and cup rest poses, the strict mode, the kit's --sample loop, pages.

## Decision after the gate (user, 2026-10-22 15:50 ET)
The 32-plate check gave 24 of 32 (all 8 misses on tilt, 3.3-6.5°; 31 of 32 within 10 mm; the 4 gap-0 front plates lean
3-5° less in a load, resting on the gap-1 plate; the other 4 are r07_00012's). The gate is relaxed to "plate access
edges exact on r05_00027 and positions within 10 mm", both met, and the three-tier rerun goes ahead with the table as
it is. The gap-0 chain lean is a recorded deviation, modelled in a later round if an edge ever depends on it.
