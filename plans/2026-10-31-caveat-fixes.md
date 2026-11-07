# Caveat fixes after the hardware study
Slug: 2026-10-31-caveat-fixes | From: research/2026-10-25-tuesday-status-sampler-hardware.md, plans/2026-10-30-bowl-drop100.md

## Goal
The docs, the sampler default, the run notes and the repo root say what the code and the records say, with no new experiment.

## Decisions (from grilling, 2026-10-31)
- Scope (user): fix the caveat groups in parallel, no new box run. Rejected one group at a time.
- Chain edges (user): delete `chain_edges` and `SIDE_ZONE_CHAIN` from the sampler; the measured direction stays in results/2026-10-23-lower-bowl-chain-check.md. Rejected a `build.chain` variant with default off (code no round would use) and keeping the chain on (measured worst: medium 3/20, hard 0/20).
- gate_initial (user, round 2): the stage records what covers the initial today, from the stored start record: the start generator's settle (peak and median contact depth under 2 mm over 5 s), reproduction (within 10 mm / 15 deg) and unstack rehearsal, PASS when all three hold, FAIL otherwise, never SKIP. The fresh-process re-settle of the stored initial poses is the first step of the next sampler round, together with the extended baseline at the flush rack travel it needs (the pairs on record were sampled at -0.49 / -0.44). Rejected wiring it now (a runner hour plus a box round of about 45 min) and the rack-only joint gate (leaves the counter stacks uncovered).
- Hygiene (user): the root `src/`, `scripts/`, `tests/` pyc leftovers deleted; REPORT.md moved to results/2026-09-30-robot-phase0-report.md with a header line (the git rename is the user's); the flush-travel twin folder uploaded to the hub (the four files that differ; machine.json stays unpublished as before).
- Page (user): one clause "hand side-mounted, as built in the twin" in the header line of the Hardware Limits Report, in the builder and in the live index.html; Version 27 published after the user saw the line (pop-up go, 2026-10-31). Rejected a new paragraph (the tables-only rule).
- Docs (no decision needed): robot.md, the execution README, CLAUDE.md, exposure.md, dependency.md and README.md brought to the records; every number from a record or run note, cited in the edit.
- Notes (no decision needed): the three disagreements listed in the 2026-10-25 status note checked against the records and the losing text corrected.

## Change
- code/src/sampler/gate.py: stage_gate_initial judges the stored start record (settle, reproduction, unstack), PASS or FAIL with the numbers.
- code/src/sampler/order.py, code/src/sampler/order_check.py, code/tests/test_sampler_order.py: the chain code and its tests removed; code/planner/frigidaire/frigidaire_dependency_kit.py verdict text says the roster size instead of 24.
- code/frigidaire/docs/robot.md, code/execution/frigidaire/README.md, CLAUDE.md, code/frigidaire/docs/exposure.md, code/frigidaire/docs/dependency.md, README.md: stale facts replaced (box, page versions, ceilings, script table, status).
- results/2026-10-23-lower-bowl-chain-check.md, results/2026-10-23-hardware-limits-mouth-up.md: text matched to the records.
- code/execution/frigidaire/frigidaire_robot_report_page.py and data/media/robot/report_v1/index.html: the header clause.

## Run
`.venv-kitfree/bin/python -m pytest code/tests/test_sampler_order.py code/tests/test_sampler.py -q -p no:cacheprovider` (Mac, Kit-free)

## Works when
The sampler tests pass with `SIDE_ZONE_CHAIN` gone (`grep -rn SIDE_ZONE_CHAIN code/` empty); each doc line changed names its source; the hub's twin folder matches the local one except machine.json.

## Assumed / skipped
- Assumed: no sampler round is re-run. The deletion restores the bowl-plate round's configuration for the lower-rack bowl pairs only; the upper-rack bowls and cups keep the bowl table and its path edges from the bowl-rest round (9d84df3), which the bowl-plate round (0ebdf5a) did not have (runner's check).
- Skipped this round: the fresh-process re-settle (next sampler round, with the flush-travel baseline), the hard tier's 23-vs-24 roster mismatch.
