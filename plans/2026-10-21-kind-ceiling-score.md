# A fair score across objects: each dish over its kind's best, dishes weighted equally
Slug: 2026-10-21-kind-ceiling-score | From: results/2026-10-16-object-level-rescale.md (follow-up, no lit review)

## Goal
A dish's score is its object-level New+J divided by one ceiling per kind (the plate, bowl or cup alone at its best pose and spot under its arm), the load score is the plain mean over its dishes, and both pages show it, with the area-weighted number kept as "surface reached".

## Decisions (from grilling, 2026-10-21)
- Why: each kind has its own shape ceiling (facing the arm, alone: plate 0.81, bowl 0.50, cup 0.18 under New+J), so absolute scores only compare arrangements of the same objects, and area weighting lets a plate count 2.3 times a cup. Dividing by the kind's best anywhere makes 1 mean "perfectly placed" for every kind and keeps distance from the arm as a real penalty. Rejected: the per-spot reference (forgives distance, needs inward tilts to be a maximum), keeping absolute and comparing within kind only.
- Ceiling per kind and per arm: the maximum of the object-level New+J grid over every height, distance and tilt at θ 30° (`grid_obj/N0262144`, lower arm); the middle arm uses the same constant this round (its disc is 4 cm smaller; a per-arm engine sweep is a later round). Read Kit-free from the grid payload embedded in the Dot Score Rules page or the grid files; stated on the pages with the three numbers.
- Per dish: `S_kind = object-level New+J / ceiling[kind]`, no floor (a constant divisor), clipped at 1 only as a guard with the count reported. Load: `S = mean over dishes of S_kind`; `surface reached` = the area-weighted mean of the same values, a second column. Per-kind means (plates, bowls, cups) shown beside the load score. Rejected: worst-dish load score, area weighting as the primary.
- Rule: New+J at the object level, as now. Rejected: the revision-5 rule (the one-spray-point flaw).
- Pooling: a dish that pools water (the scorer's `pools` test: interior below the rim by more than 2 mm) scores 0, as the revision-5 feasibility rule intended; the pooling count stays visible. Rejected: unchanged-and-flagged (as now), halved.
- Middle arm: the same constant per kind this round (user: no further grilling); over 1: clipped with the count reported; the explorer changes with the loads.
- Rankings recomputed for the 252 farm loads, the 24 restricted loads and the dependency samples; the explorer's relative mode becomes "relative to the kind's best" (one constant per kind) so its curves keep their shape and are at most 1.

## Change
- `frigidaire_dot_rules_relative.py`: `--reference kind` (ceilings from `--grid-obj-dir` or `--grid-payload <index.html>`), fields `S_kind` per dish, `S` (mean), `S_area` (area-weighted), per-kind means; `--summary` prints the three ceilings, Spearman against absolute and against the per-spot rescale, the new top and bottom five.
- `frigidaire_dependency_page.py`: captions show S (mean), surface reached and the per-kind means; the defining sentence names the ceilings.
- `frigidaire_dot_rules_page.py`: the explorer's relative mode divides by the kind constant; prose and legend updated; republish both pages.

## Run
Mac, Kit-free: `python3 code/planner/frigidaire/frigidaire_dot_rules_relative.py --reference kind --grid-payload data/media/dot_rules/page_v22/index.html --loads-dir data/results/dot_rules/loads/N0262144 --restricted-dir data/results/dot_rules/restricted --dependency-dir data/results/dot_rules/dependency/samples_v5 --summary` (per-dish object-level scores are in the saved `.relative.json` files; the box only if a load has none), then both page rebuilds and republishes.

## Works when
Every scored dish has `S_kind` in [0, 1] (clipped count reported), both pages show the mean score with surface reached and the per-kind means, the explorer's relative curves are at most 1, and the run note gives the three ceilings, the Spearman values and the new top and bottom five farm loads.

## Assumed / skipped
- Assumed: the lower-arm ceiling serves the middle arm this round; the grid maximum at θ 30° is the ceiling (the loads are scored at θ 30°); loads with no saved per-dish object-level score are re-scored on the box in one short window.
- Skipped this round: a per-arm engine sweep for the middle-arm ceiling, inward tilts, regenerating any load.
