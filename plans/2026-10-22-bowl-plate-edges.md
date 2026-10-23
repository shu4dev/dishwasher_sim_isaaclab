# Bowl-plate edges from packed bowl poses in the pre-build DAG; the three-tier rerun
Slug: 2026-10-22-bowl-plate-edges | From: results/2026-10-22-plate-rest-pose-dag.md (follow-up, no lit review)

## Goal
Remove the blocker the rest-pose round left: in 19 of its 21 medium and hard build failures a bowl placed earlier
covered a plate's descent. Give bowls (and cups) edges against plates in the pre-build DAG, using the packed bowl pose
as the proxy for where the bowl sits, and rerun easy, medium and hard with the same 20 seeds.

## Decisions (grilled 2026-10-22 18:20 ET)
- Edges added, bowl-plate only, both directions: bowl B at its packed pose covering plate P's descent (P's path from
  20 mm above the rim to its rest pose, the 1 mm allowance) gives P before B; plate P at its rest-table pose covering
  bowl B's descent (B's path from 20 mm above the rim at its packed orientation to where the appliance stops it)
  gives B before P. Cups count as bowls. A bowl-plate cycle re-draws the packing (seed*1000+k, k < 10), counted in
  the cycle row. Rejected: bowl-bowl edges (reversed at packed poses in the diagnostic; they stay on re-pose and
  deferral), fitted bowl poses, physics settles for bowls.
- Everything else as the rest-pose round: plates at the table poses with plate-plate edges, the lean-side chain as the
  tie-break, bowls and cups where the seeded permutation puts them subject to the new edges, re-pose and deferral on
  for every dish, no strict mode.
- Rerun (user's call): easy, medium and hard, seeds 0-19, `--from build --to certify`, 4 slots; before = the
  rest-pose round (easy 20/20/16/12, medium 12/1/1/0, hard 13/3/3/2, 0 cycles) and the baseline before it. Report per
  stage, the cycle row, DAG stats (plate-plate, bowl-plate edges, layers), the failed seeds with reasons, and for every
  build failure whether a bowl still blocked a plate.
- On r05_00027 the new edges are checked against the witness order (no new edge may point against it) and reported;
  the certified hits have no bowl-plate pair, so this is a consistency check, not the pass rule.
- Box: drive the rerun from a box-side script (nohup, as the sampling loops were), so a Mac sleep cannot stall it; the
  Mac fetches at the end. About 70 minutes of box; stop right after; hard cap 2 hours.

## Change
- `code/src/sampler/order.py`: bowl-plate edges in the `rest` pose model (descent paths from the move's helpers;
  bowls at the packed pose); `dag_order` unchanged otherwise. `order_check.py`/`test_sampler_order.py`: the r05
  consistency check. No kit change expected (the order file already carries the model).

## Run
Box: the scheduler of the rest-pose round (`data/outputs/sampler_rate/sched_rest.py`) with out dirs
`data/results/sampler/bp_<difficulty>/`, logs `data/logs/sampler_bp/`; tally; `brev stop`; confirm STOPPED.

## Works when
`results/2026-10-22-bowl-plate-edges.md` holds the before/after per-stage tables for the three tiers, the cycle row
and DAG stats, the r05 consistency check, the failed seeds with the blocked-by-bowl count, the box window and a
suggested commit message; medium builds more than 1 of 20, hard more than 3 of 20, easy stays at 12 of 20 valid or
the note says which failed and why; the box is STOPPED.

## Assumed / skipped
Assumed: a bowl's packed pose is a usable proxy for the column it occupies (its landing is 40-110 mm off, mostly in
height and within its zone). Skipped: bowl-bowl edges, cup-specific handling, the packer's "no free pose" seeds, pages.
