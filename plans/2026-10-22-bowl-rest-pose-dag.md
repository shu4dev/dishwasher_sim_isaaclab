# Measured bowl and cup rest poses per site and variant, bowl-bowl edges, the three-tier rerun
Slug: 2026-10-22-bowl-rest-pose-dag | From: results/2026-10-22-bowl-plate-edges.md (follow-up, no lit review)

## Goal
Apply the plate recipe to bowls and cups: measure where a lone bowl or cup rests when the insertion move drops it at
each site and orientation the packer uses, check the table against the settled bowls of the certified loads, add
bowl-bowl edges (and bowl-plate edges at the rest pose) to the pre-build DAG, and rerun easy, medium and hard with
the same 20 seeds. The bowl-plate round left exactly one build-failure mode: a bowl blocked by an earlier bowl.

## Decisions (2026-10-22 21:15 ET, the session's; the user delegated this round)
- Settles: one bowl (or cup) alone in the twin, dropped by the move exactly as the build does it (packed orientation,
  lowered from 20 mm above the rim in 5 mm steps against the appliance, released 20 mm above the stop, 150 + 60 ticks
  at 120 Hz), at every (site, orientation and offset variant) of the packer's level-0 bowl and cup families, the
  `lift` variants collapsed (the lift only changes the start height). One Kit session, expected 60-150 settles.
  Table `data/results/sampler/bowl_rest/bowl_rest.json` keyed by (kind, site, variant-without-lift): rest pose, tilt,
  lowest point height, settle deviation, the code SHA and the move's constants. Rejected: fitting to the loads.
- Check: every settled bowl and cup of the certified records (r05_00027; the valid easy, medium and hard pairs of the
  rest-pose and bowl-plate rounds, `data/results/sampler/{rest,bp}_<tier>/`) against its table entry: tilt within 3°,
  origin within 10 mm; reported as k of n per kind and tier.
- Gate (as settled for plates): bowl-bowl edges at the table poses reproduce the certified bowl-bowl hits exactly on
  the loads that have them (r05_00027's 2; the valid easy records' hits) and give no cycle there; the pose agreement is
  information. If the hits fail on more than 2 of those loads, stop before the rerun and report.
- DAG: plates at the plate table, bowls and cups at the bowl table; plate-plate, bowl-plate (bowl now at its rest
  pose) and bowl-bowl edges, both directions where a pose covers a descent; support edges off; cycles re-draw
  (seed*1000+k, k < 10); re-pose and deferral on, no strict mode; lean-side chain as tie-break.
- Rerun: medium, then hard, then easy (the regression check last, so the cap cannot cut the tiers under test), seeds
  0-19, `--from build --to certify`, 4 slots, driven from a box-side script; before = the bowl-plate round
  (easy 19/15/11 of 19, medium 6/6/5, hard 4/4/1) and the earlier rounds. Hard cap 2 hours from RUNNING.

## Change
- `code/planner/frigidaire/frigidaire_plate_rest.py` generalised (or a sibling `frigidaire_rest_table.py`) with
  `--kinds bowl,cup`; `code/src/sampler/order.py`: the bowl table in the `rest` model, bowl-bowl edges;
  `order_check.py`: model R for bowls and cups, the bowl-hit check; `test_sampler_order.py`: the r05 bowl case.

## Run
Box: the settles (`run_kit.sh`, `taskset -c 0,4`), the check (`run_py.sh`), then the rerun with the scheduler of the
earlier rounds, out dirs `data/results/sampler/br_<tier>/`, logs `data/logs/sampler_br/`; tally; copy back; restore
the clone; `brev stop`; confirm STOPPED.

## Works when
`results/2026-10-22-bowl-rest-pose-dag.md` holds the table summary, the k of n check per kind and tier, the gate
result, the before/after per-stage tables for the three tiers with the cycle row and DAG stats, the failed seeds with
reasons, the box window and a suggested commit message; medium exceeds 5 valid of 20 or hard exceeds 1, or the note
says which failed and why; the box is STOPPED.

## Assumed / skipped
Assumed: a lone bowl's rest pose at a site is what a bowl in a load rests at too (its neighbours only shade it).
Skipped: support edges, nesting, the packer's "no free pose" seeds, pages, clips for the new valid pairs.
