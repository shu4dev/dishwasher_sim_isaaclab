# Lean-side-first plate order: valid samples with valid dependency graphs
Slug: 2026-10-16-lean-side-plate-order | From: results/2026-10-15-reposing-sampler-five-samples.md (follow-up, no lit review)

## Goal
The re-posing sampler places every plate bank from its lean side outward, so shingled plates stop blocking each other; the pipeline of the parent plan then yields valid samples (load, graph confirmed by the library, order animation), 5 of them or the stop point.

## Decisions (from grilling, 2026-10-15/16)
- The finding behind it: an upright plate leans about 30° toward +x and overhangs the next 3-4 slots, so a plate placed before its lean-side neighbours blocks them for good; 0 of 20 packings built in a random order. Order = a uniformly random order of all 24 dishes in which only the plates' internal order is replaced: within each bank the slot furthest on the lean side goes first and the bank fills outward, each plate leaning onto the one before (the way people load shingled plates); bowls and cups stay interleaved at random among the plates, with the parent's re-posing and deferral (a bowl placed before the plate it rests on fails, is deferred and retried). Rejected: all plates before the bowls and cups (fewer deferrals, but every witness would say "plates first" by construction), re-posing only into non-overhung slots (the first plate in a bank's middle still kills 3-4 slots), contact-pushing insertion (needs Isaac for every move, crossings failed at 100 mm), a fully random order.
- A plate that fails to land or disturbs a placed dish is re-posed within its bank at the next slot in the chain order (the chain keeps its direction); if no slot is free it is deferred to the end of the plates; a load fails as in the parent.
- Everything else as the parent plan: the move (upright plates with the 3 mm lean bias, ceiling cap, 20 mm gap, path check), the swap tests in the witness's states, the 26-order library, one conditional-edge refinement, validity, 5 valid samples or 20 packings or 4 h of box, the same 20 packings first (r07_00012 and the next 19 of `sequential_fresh_order.txt`), then the following packings if needed, the page tuple behind the sample selector on the same artifact.
- Expected graph shape, to be measured, not assumed: a chain per plate bank (block edges from the overhang), need edges from bowls resting on plates, few edges among cups.

## Change
- `frigidaire_bench_kit.py` `build_sequence`: `order=` becomes a builder: a random permutation of all 24 dishes whose plate positions are refilled bank by bank in chain order from `lean_side` (frigidaire_bench.py); plate re-posing walks the chain; everything else unchanged.
- `frigidaire_dependency_kit.py`: `--sample --plate-order lean-side` (default for this round), recorded in the sample's `witness` with the bank and chain index per plate; `--out-dir data/results/dot_rules/dependency/samples_v2`.
- `frigidaire_dependency_page.py`: the yield table names the order rule; the tuple layout is unchanged.

## Run
`python3 code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --sample --target-valid 5 --max-packings 20 --plate-order lean-side --release insertion --edges swaps --library 10 --out-dir data/results/dot_rules/dependency/samples_v2 --max-wall-seconds 14400` as the host-side scheduler on the box (the Kit-free screen of the chain order on the 20 packings first, in the same window), then clips, page, publish to the same URL.

## Works when
`samples_v2/` holds valid samples with 24 placed, library and refinement as predicted, a witness clip and per-edge clips each; the page's selector shows their tuples; the run note gives valid / attempted, why the rest failed, and the edge counts by reason per sample.

## Assumed / skipped
- Assumed: a bank is a rack's row of plate slots sharing a lean direction (from the packing's families); the chain index is recorded so the page can draw the chain; the screen's per-kind table is reported again under the new order; one window, about 3-4 h at most.
- Skipped this round: the bench, the Mac-side Kit-free setup (its session died in the outage; restart it separately), contact-pushing insertion, more than one refinement round.
