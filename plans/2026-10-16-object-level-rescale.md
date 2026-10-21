# Object-level rescaling: S becomes each dish's score over the best it could do at its spot
Slug: 2026-10-16-object-level-rescale | From: results/2026-10-14-newj-curve-and-leaning-pairs.md (relative S column; follow-up, no lit review)

## Goal
A dish's score is its object-level New+J divided by the best object-level New+J the same dish alone can reach at the same height and distance under the same arm (max over tilt), floored at 0.05; this rescaled S replaces absolute New+J as the load score everywhere it is shown, with absolute New+J kept as a column.

## Decisions (from grilling, 2026-10-16)
- Why: the single dot is already normalised (New divides by the best-facing weight at its spot) but a dish never reaches 1 because its own shape hides part of its surface (plate 0.81, cup 0.18 facing the arm). Dividing at the object level says how well a dish is placed against its own best. Rejected: the facing pose as the reference (round 1's relative S: can exceed 1, plate 1.60), one best per kind (also penalises distance from the arm).
- Reference = the dish alone at its spot, max over tilt. Computed Kit-free from what exists: round 1's exact facing reference per dish (`*.relative.json`, the engine run on the dish alone, axis straight down, at its centre under its arm) times the grid's tilt gain at the nearest cell, max over tilt of `<kind>_obj` divided by its 0° value (`grid_obj/N0262144`, lower-arm disc; the gain is assumed to carry over to the middle arm's 4 cm smaller disc). Rejected for this round: engine sweeps over tilt per dish (about 50,000 runs, hours of GPU).
- relative = score / max(reference, 0.05); the page states the floor and how many dishes hit it. Rejected: dropping dishes with tiny references.
- Adopt: rescaled S ranks every load and sample (farm loads, restricted loads, the dependency samples); absolute New+J stays as a column; best/worst selections are recomputed. The explorer's "relative to 0°" becomes "relative to the best tilt at the spot" (JS, max over tilt from the grid payload), so the relative curve is at most 1.

## Change
- `code/planner/frigidaire/frigidaire_dot_rules_relative.py`: `--reference best` (facing reference times the grid tilt gain), writes `S_rel_best` per dish and per load next to the existing fields; `--summary` prints Spearman against absolute S and the new top and bottom five.
- `frigidaire_dependency_page.py` and `frigidaire_dependency_kit.py`: sample captions show rescaled S with absolute New+J in parentheses.
- `frigidaire_dot_rules_page.py`: the explorer's relative mode divides by the max over tilt at the spot; its prose and legend updated; republish Dot Score Rules (same URL, Version 22).

## Run
On the box, Kit-free: `code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_relative.py --reference best --loads-dir data/results/dot_rules/loads/N0262144 --dependency-dir data/results/dot_rules/dependency/samples_v5 --restricted-dir data/results/dot_rules/restricted --grid-obj-dir data/results/dot_rules/grid_obj/N0262144 --summary`, then the page scripts on the Mac and both republishes.

## Works when
Every scored dish has `S_rel_best` at most 1 (floored dishes counted), both pages show the rescaled S with absolute beside it, the explorer's relative curves stay at or below 1, and the run note gives the Spearman and the new top and bottom five farm loads.

## Assumed / skipped
- Assumed: the grid's tilt gain at the nearest (height, distance, θ) cell stands in for the dish's own tilt sweep; the floor applies after the gain; a dish whose kind has no `_obj` grid falls back to the facing reference and is flagged; about 10 min of box, one window.
- Skipped this round: an engine tilt sweep per dish, the middle-arm grid, regenerating any load.
