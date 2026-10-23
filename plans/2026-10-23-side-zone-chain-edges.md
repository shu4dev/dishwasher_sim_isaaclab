# Side-zone chain edges for lower-rack bowls, the measured table for upper bowls and cups, the three-tier rerun
Slug: 2026-10-23-side-zone-chain-edges | From: results/2026-10-23-lower-bowl-chain-check.md (follow-up, no lit review)

## Goal
Close the last build-failure mode (a bowl blocked by an earlier bowl) with what the certified loads support: in the two
lower-rack side zones bowls go in from the plate bank outward (24 of 26 hits), upper bowls and cups rest where the
lone-settle table says (52 of 58, 24 of 32), and the rear row is left to re-pose and deferral. Rerun easy, medium and
hard with the same 20 seeds against the bowl-plate round.

## Decisions (2026-10-23 00:45 ET, the session's; the user said implement and rerun)
- Lower-rack side zones (`lower_frontright`, `lower_rearright`): a chain edge between every two bowls of the same zone,
  the one with the smaller packed rack x first (the bank or wall side); the zone's chain direction is a constant of
  the zone read from the measured hits, not fitted per load. Their bowl-plate edges stay as in the bowl-plate round
  (bowl at its packed pose). No access path check is run between two side-zone bowls (the lone poses are wrong there).
- Upper-rack bowls and cups: poses from `data/results/sampler/bowl_rest/bowl_rest.json` (the lone-settle table) in the
  `rest` model; bowl-bowl and bowl-plate edges from the path check at those poses, both directions.
- Rear row (`lower_rear`): no bowl-bowl edges (10 of 18 hits follow any rule); bowl-plate edges at the packed pose as
  before; re-pose and deferral handle its pairs. Its failures are counted separately in the note.
- Plates unchanged: the plate table, plate-plate edges, the lean-side chain as tie-break. Cycles re-draw (seed*1000+k,
  k < 10). Re-pose and deferral on for every dish, no strict mode.
- Rerun: medium, then hard, then easy, seeds 0-19, `--from build --to certify`, 4 slots, driven from a box-side script;
  before = the bowl-plate round (easy 19/15/11 of 19, medium 6/6/5, hard 4/4/1). Hard cap 2 hours from RUNNING.
- Check on the certified loads before the rerun (Kit-free, on the box): the side-zone chain edges contradict no
  certified witness order (a 2-of-26 miss is the known rate); report the count.

## Change
- `code/src/sampler/order.py`: zone chain edges for the two side zones (constant `SIDE_ZONE_CHAIN = {zone: +x}`
  with the source note), the bowl table for upper bowls and cups, no bowl-bowl path edges in the rear row.
- `order_check.py` / `test_sampler_order.py`: the witness-consistency count for the chain edges.

## Run
Box: `git pull` (the chain-check note and this plan are on `plan`; the bowl settle script, table and edge code are
uncommitted on the Mac and get copied as before), the witness check, then the rerun with the scheduler of the earlier
rounds, out dirs `data/results/sampler/sz_<tier>/`, logs `data/logs/sampler_sz/`; tally; copy back; restore the clone;
`brev stop`; confirm STOPPED.

## Works when
`results/2026-10-23-side-zone-chain-edges.md` holds the witness check, the before/after per-stage tables for the three
tiers with the cycle row, the DAG stats (plate-plate, bowl-plate, side-zone chain, upper edges), the failed seeds with
reasons and which zone blocked, the box window and a suggested commit message; medium exceeds 5 valid of 20 or hard
exceeds 1, or the note says which failed and why; the box is STOPPED.

## Assumed / skipped
Assumed: the side-zone direction (+x = away from the bank) is the same in every packing of those zones. Skipped: the
rear row's order test, support edges, the packer's "no free pose" seeds, pages and clips for the new valid pairs.
