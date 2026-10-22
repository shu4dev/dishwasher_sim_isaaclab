# Per-object tilt score: New+J over its best orientation, checked against New and New+J from -60 to 180 degrees
Slug: 2026-10-22-tilt-score-check | From: results/2026-10-21-kind-ceiling-score.md (follow-up, no lit review)

## Goal
For the plate, the bowl and the cup, one figure and one table show New, New+J and the proposed score (New+J divided by its maximum over orientations at the same spot) against tilt, including tilts toward the arm's centre, so that the expectation can be checked by eye: 1 at each dish's best orientation, 0 at 180 degrees, about 0.12 on edge, and each dish's own sensitivity in between.

## Decisions (discussion 2026-10-22)
- Fair = relative to the dish's own best orientation at its spot; 1 there for every object. The reference is the maximum over orientations, not the 0-degree value (the cup's best lies a few degrees toward the arm near the disc edge; Original even puts the cup's best at 25 degrees over the centre). Rejected: dividing by 0 degrees (climbs past 1 for the cup), forcing 0 at 90 degrees (the cone really reaches an edge-on food side, about 12% of the best for all three), forcing anything at 180 (New+J is 0 there by geometry: every food-contact normal faces away from every jet; this holds because the scorer has no spray from above, ceiling weight 0).
- Tilt axis for the check: -60 to 180 degrees in 5-degree steps, negative = the food side leaned toward the arm's centre (the direction the old grid never swept), positive = away, as the grid defines it. Spots: 14 cm up at 0, 10, 20 and 25 cm out, and 8 and 20 cm up over the centre (six spots). Theta 30 degrees, N = 262,144 lines per set, dot level and object level, lower arm. Each dish alone, its own mesh as the only blocker, as in the grid.
- Rules shown: New, New+J (absolute), and score = New+J / max over the sweep at that spot. Pass criteria written into the note: score = 1 at exactly one tilt per dish and spot (the argmax, reported in degrees), score(180) < 0.005 for every dish, score(90) between 0.08 and 0.20, no value above 1 by construction; where the argmax is not 0 degrees the note says by how much it beats 0 degrees.

## Change
- `code/planner/frigidaire/frigidaire_dot_rules_grid.py`: a `--tilts` range that admits negative values (lean toward the arm's centre) and a `--spots` list, writing to its own `--out` (never the default levels).
- A small Kit-free figure script (or a function in `frigidaire_dot_rules_figures.py`): per dish one panel, three curves, the argmax marked, the six spots as rows; a table of the numbers at -30, 0, 30, 45, 60, 90, 120, 150, 180.

## Run
On `dishsim-l4`, after the bring-up window has stopped the box (one new window): `code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_grid.py --kinds plate,bowl,cup --tilts -60:180:5 --spots "14,0 14,10 14,20 14,25 8,0 20,0" --theta 30 --N 262144 --out data/results/dot_rules/tilt_check` (3 kinds x 49 tilts x 6 spots = 882 cells, about 5 minutes on the L4), then the figure script on the Mac from the copied JSON; `brev stop`.

## Works when
`data/results/dot_rules/tilt_check/` holds the sweep, the figure shows the three curves per dish for the six spots, and the note states the pass criteria per dish and spot with the measured numbers; the box window is under 20 minutes.

## Assumed / skipped
- Assumed: the inward lean is a rotation about the same axis as the grid's outward tilt (the dish's food side turning toward the disc centre); the single-dot curve is included as a fourth entity for reference; nothing is published and no page changes this round.
- Skipped this round: the middle arm, other thetas, the load score, any page.
