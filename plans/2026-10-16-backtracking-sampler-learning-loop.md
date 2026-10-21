# Backtracking sampler and a learning loop: valid samples with valid dependency graphs
Slug: 2026-10-16-backtracking-sampler-learning-loop | From: results/2026-10-16-lean-side-plate-order.md (follow-up, no lit review)

## Goal
The sampler backtracks past the rear bank and bowls placed before their plates, "valid" means every obeying order builds and every violation fails, and an obeying order that fails teaches the graph an edge; the page shows valid samples as tuples, 5 of them or the stop point.

## Decisions (from grilling, 2026-10-16)
- Validity = every obeying library order lands all 24 and every single-edge violation fails somewhere; the dish it fails at and the predicted dish are both recorded and shown, never required to match (a cascade in a new context is physics, not a wrong edge). Rejected: exact prediction (no pairwise graph can honour it; both built samples of the parent failed on it).
- Learning loop: an obeying order that fails at X adds a learned edge from the build's own diagnosis (the dish X hit or needed before X; if none is named, the dish placed just before X in that order), reason "learned"; 10 fresh obeying orders re-check; up to 3 rounds, then the sample is invalid. A violation that still builds becomes a conditional edge as in the parent. Rejected: one round, no learning.
- Backtracking: a dish with no clear pose after its re-poses names its blockers; the most recent placed blockers, up to 3, are undone (the undo of the parent: park, restore the others, 60 ticks), the dish is placed, the undone dishes go back in the same order and are deferred if they fail now; at most 2 backtracks per dish; the load fails only when a dish is still out after everything else is in. Rejected: a rear-plates-before-rear-bowls rule and a two-gap rear chain (specific, keeps failing elsewhere), both together (the rule is unnecessary once backtracking works).
- Candidate pairs: the 20 mm rule plus every pair the sampler's builds saw knock, block or need (the parent's rule, now including backtracks). Everything else as the parents: the move, the lean-side chains in a random order with bowls and cups interleaved, re-posing, the 26-order library, the same 20 packings first then the next ones, 5 valid or 20 packings or 4 h of box, the page tuple on the same artifact.

## Change
- `frigidaire_bench_kit.py` `build_sequence`: `backtrack=` (depth 3, 2 per dish) using the existing undo; the trace records each backtrack (dish, undone dishes, outcome).
- `frigidaire_dependency_kit.py`: `verdict` under the new validity (fields `failed_at`, `predicted_at`, `as_predicted` = built-or-failed match); `--learn-rounds 3` after `--merge-library` (learned edges, fresh obeying orders, re-merge); the sample record gains `learning` (rounds, edges added, orders per round).
- `frigidaire_dependency_page.py`: library table shows failed-at vs predicted-at; learned edges drawn dotted; the sample is valid when the last round passes.

## Run
`python3 code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --sample --target-valid 5 --max-packings 20 --plate-order lean-side --backtrack 3 --release insertion --edges swaps --library 10 --learn-rounds 3 --out-dir data/results/dot_rules/dependency/samples_v3 --max-wall-seconds 14400` as the host-side scheduler on the box (Kit-free screen first in the same window), then clips, page, publish to the same URL.

## Works when
`samples_v3/` holds valid samples (24 placed, last library round: obeying all build, violations all fail), each with a witness clip and per-edge clips; the page's selector shows their tuples with the learned and conditional edges marked; the run note gives valid / attempted, failure reasons, backtracks per sample and edges by reason.

## Assumed / skipped
- Assumed: the learning-round orders are drawn from the refined graph; a learned edge that makes the graph cyclic is dropped and reported; backtracking counts against the same 4 h; one window.
- Skipped this round: the bench, the Mac-side Kit-free setup (its session died in the outage; restart separately), more than 3 learning rounds, three-body swap tests.
