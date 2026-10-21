# A re-posing sampler that yields valid loads with valid dependency graphs
Slug: 2026-10-15-reposing-sampler-five-samples | From: results/2026-10-15-insertion-drop-dependency.md (follow-up, no lit review)

## Goal
The sampler places each dish by a path-checked insertion and re-poses a dish that cannot go in, so every accepted load is reachable by construction; it keeps sampling until 5 loads each have a graph the library confirms, and the page shows per sample the load, its graph and the animation of its order.

## Decisions (from grilling, 2026-10-15)
- The move, fixed once: vertical descent along the rack's up axis from the dish's lowest collision point 20 mm above its rack's rim, the start capped so its highest point stays under the tub ceiling (TubCeiling in geometry.py), 5 mm steps, the 1 mm allowance against placed dishes and appliance; the dish is released at rest at the lowest clear pose raised 20 mm, then settles 150 + 60 ticks. Plates enter UPRIGHT in their slot (the packing's lean is not a target), released 3 mm off-centre toward the packing's lean side, and settle into a lean; bowls and cups enter at their packed orientation. Rejected: along-the-lean paths, bowls level-then-tip, 0 or 50 mm release gaps, the rack's up axis uncapped.
- Sampler: a uniformly random order per sample. A dish whose descent is blocked, that does not land (within DEV_MAX_M of its packed target, contact < 2 mm) or that disturbs a placed dish beyond its tolerance is re-posed: up to 5 other free poses of its kind from the packing's candidate list whose descent is clear (Kit-free screen against the placed dishes at their settled poses), the first that lands wins; if none, the dish goes to the back of the order; the load fails only when a dish still cannot go in after everything else is placed. The built poses are the goal, the order its witness, S rescored. Rejected: lowest-first or the packing's order, defer-only, re-roll on the first block.
- Graph: swap tests of the witness in the states it passed through (round 1), the swap and library drops under the same move, margins kept. Valid graph = every library order (witness, one violation per edge, 10 random obeying, 10 random single-violation) behaves as predicted. A single violation that still builds makes its edge conditional: B needs one of {A, C...} where C are dishes placed before B in that order but absent in the swap context, each C confirmed by one drop of B with A absent and C present; 10 fresh random obeying orders re-check the refined graph once. Still failing = the sample is invalid (reported, not counted). Rejected: report-only, re-roll until a graph passes unchanged.
- Yield: keep sampling (packings from `sequential_fresh_order.txt`, r07_00012 first, in parallel) until 5 samples are valid; stop point: 20 packings attempted or 4 h of box time, whichever first, then report the yield and why the rest failed. Rejected: one pilot, every admitted packing.
- Per valid sample the page shows the tuple: the load (both-racks still, S), the graph (edges with reasons, conditional edges drawn as a fan), and the witness animation; plus the per-edge clips and the library table behind a sample selector. The height-sweep panels go (kept in the run note). Same artifact.
- Study only; the Kit-free screen runs on the box in the same window, before the Isaac jobs.

## Change
- `frigidaire_bench.py`: `insertion_release` gains the ceiling cap from TubCeiling, the 20 mm release gap, the upright plate pose with the 3 mm lean bias (`upright_pose(slot, lean_side)`).
- `frigidaire_bench_kit.py` `build_sequence`: random order, `repose=` callback (Kit-free candidate screen), deferral, the failure rule above.
- `frigidaire_dependency_kit.py`: `--edges swaps --release insertion --sample` (the loop over packings with the stop point), conditional-edge refinement after `--merge-library`, record `dependency/samples/<id>.json` with `valid`, `witness`, `edges` (plain and conditional), `library`, `refinement`.
- `frigidaire_dependency_clips.py`: unchanged apart from the conditional-edge clip set; `frigidaire_dependency_page.py`: sample selector, the tuple layout, the conditional-edge fan.

## Run
`code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --sample --target-valid 5 --max-packings 20 --release insertion --edges swaps --library 10 --out-dir data/results/dot_rules/dependency/samples --max-wall-seconds 14400 --headless` as a host-side scheduler over cores 0-13 (screen first; one Kit session per packing, library orders as sessions), then clips, page, publish to the same URL.

## Works when
`samples/` holds 5 records with valid true (24 placed, library and refinement as predicted), each with a witness clip and per-edge clips, the page's selector shows the 5 tuples, and the run note gives the yield (valid / attempted) and the failure reasons of the rest.

## Assumed / skipped
- Assumed: swap tests judge landing within the at-goal tolerance of the built goal; a conditional edge counts as obeyed when any member precedes B; the stop point also ends a window early so the box never idles; about 45 min of box per sample at full parallelism, one window of 3-4 h.
- Skipped this round: the bench, Mac-side Kit-free setup (separate task), more than one refinement round, three-body swap tests beyond the conditional edges.
