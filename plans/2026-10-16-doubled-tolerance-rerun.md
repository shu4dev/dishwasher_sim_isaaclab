# Doubled landing tolerance: rerun the two rebuildable samples, no backtracking
Slug: 2026-10-16-doubled-tolerance-rerun | From: results/2026-10-16-backtracking-sampler-learning-loop.md (follow-up, no lit review)

## Goal
Under a doubled at-goal tolerance the libraries of r07_00012 and r05_00027 are rerun in Isaac; if their graphs validate, sampling continues without backtracking toward 5 valid samples within the window's cap.

## Decisions (from grilling, 2026-10-16)
- Tolerance: AT_GOAL doubled for landing and disturbance judgments in the swap tests and the library: bowl and cup 30 mm lateral / 40 mm height / 30°, plate 36 / 40 / 32°, contact depth < 4 mm (`AT_GOAL_LIBRARY`, a second table next to AT_GOAL so the bench's own rule is untouched). The sampler's own-load rule (DEV_MAX_M 8 cm) is unchanged. Rejected for now: a Kit-free diagnosis first (the user chose to test the tolerance directly), a slot-level goal (the fallback if this fails).
- Backtracking dropped: `--backtrack 0`; only builds without a restore count; a dish still out after re-posing and deferral fails the load. Rejected: physical chronological backtracking (several times the drops), restores with a rebuild check (0 of 7 rebuilt).
- Order of work in one window: (1) re-judge the two samples' saved swap drops under the new tolerance, Kit-free, so the graphs match the rule (edges may drop); (2) rerun their libraries in Isaac (witness, per-edge violations, 10 obeying, 10 single-violation) and the learning loop (up to 3 rounds) under the new rule; (3) only if both validate, continue sampling new packings (the next ones of `sequential_fresh_order.txt`) without backtracking until 5 valid samples or the cap. Cap: 3 h of box in all.
- Free add-on, reported but not gating: from the parent's saved library records, every order re-judged rack-locally and each dish's landing spread across orders, so the note says whether rack-local judgment alone would have validated them.
- Validity, learning, refinement, clips and the page tuple as the parent plan; the page states the tolerance in force.

## Change
- `frigidaire_bench.py`: `AT_GOAL_LIBRARY` and `within_goal(kind, distance, table=AT_GOAL)`; `frigidaire_dependency_kit.py`: `--tolerance library` selecting the table for swaps, library and learning, recorded in the sample as `tolerance`; `--rejudge` (Kit-free) rewrites a sample's edges from its saved drops under the selected table; `--backtrack 0` default.
- `frigidaire_dependency_page.py`: the tolerance in the method section and the yield table.

## Run
On the box: `code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_kit.py --rejudge --tolerance library --records data/results/dot_rules/dependency/samples_v3_r07/r07_00012.json data/results/dot_rules/dependency/samples_v3/r05_00027.json --out-dir data/results/dot_rules/dependency/samples_v4`, then the scheduler `python3 ... --sample --resume-from data/results/dot_rules/dependency/samples_v4 --target-valid 5 --max-packings 20 --plate-order lean-side --backtrack 0 --tolerance library --release insertion --edges swaps --library 10 --learn-rounds 3 --out-dir data/results/dot_rules/dependency/samples_v4 --max-wall-seconds 10800`, then clips, page, publish to the same URL.

## Works when
`samples_v4/r07_00012.json` and `r05_00027.json` carry `tolerance: library` and a library verdict (valid or not, with every order's failed-at and predicted-at); any valid sample has its tuple on the page; the run note gives the per-dish landing spread and the rack-local re-judgment of the parent's orders.

## Assumed / skipped
- Assumed: a swap edge that disappears under the new tolerance loses its clip; the learning diagnosis is unchanged; if neither sample validates the window ends after step 2 and the note says what the doubled tolerance still missed.
- Skipped this round: backtracking of any kind, the slot-level goal, the bench, the Mac-side Kit-free setup.
