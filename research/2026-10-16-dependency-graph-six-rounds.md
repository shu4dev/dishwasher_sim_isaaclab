# Six rounds toward a valid sample with a valid dependency graph: what blocks it
Slug: 2026-10-16-dependency-graph-six-rounds | Lit review: none (retrospective over results/2026-10-15-* and results/2026-10-16-*)

## Question
Can a sampler produce a 24-dish HOTEC load that a one-dish-at-a-time placement from above can build, together with a dependency graph ("A before B") that every order obeying it builds and every order breaking one edge fails?

## What happened (rounds 1-6, 2026-10-15/16)
| Round | Rule change | Packings | Built all 24 | Valid | Why it stopped |
|---|---|---|---|---|---|
| 1 | teleport to the goal, clear hover | 16 | 1 | not judged | the move puts dishes where no placement from above can; 2 of 10 random violations built (three-body) |
| 2 | straight-down insertion, drops of 0/20/50/100 mm | 24 | 0 | 0 | every leaning plate hits the tine tips 35-77 mm above its goal; at 100 mm plates cross and bowls tumble |
| 3 | upright plates, re-posing, random order | 20 | 0 | 0 | a plate leans ~30° and overhangs 3-4 slots; a random order blocks them for good |
| 4 | plate banks filled from the lean side | 20 | 2 | 0 | front banks fixed (97 of 107 first try); rear plates near upright block each other; bowls before their plates |
| 5 | backtracking by state restore, learning loop | 20 | 6 | 0 | restored states are not reachable by their recorded order (0 of 7 rebuilt); bowls land outside 15 mm / 15° in other orders |
| 6 | doubled tolerance, no backtracking | 2 | 2 | 0 | nested bowls pressed 5-8 mm at rest; a block that depends on where an earlier bowl landed; session history changes outcomes |

Yield: about 100 sampler attempts on about 40 packings; 9 loads reached the library; 0 valid. Box: about 7.3 h in windows plus 2.3 h idle (two outages).

## Findings
1. Most packings cannot be loaded one dish at a time from above (about 80 of 100 attempts fail in the sampler). The restricted packings were made for exposure, not for insertion: shingled plates lean over their neighbours' slots, and the rear bank's near-upright plates stand one slot apart. Filling each bank from its lean side fixes the front; the rear bank and bowls placed before the plates they rest on still fail about half the loads.
2. A bowl's resting pose is not order-independent. Across obeying orders one bowl landed anywhere in 0-68 mm and 0-65°; plates and cups stay inside the bench tolerance. An exact pose with a tolerance is the wrong definition of a bowl's goal; a slot-level goal (same slot, same supporting dishes, same lean side) is what the physics keeps.
3. Nested bowls press 5-8 mm into each other at rest (hull contact depth), beyond the 2 mm and the doubled 4 mm limit; this is a steady state, so no tolerance on position fixes it. The landing rule needs a nesting-aware contact test (depth against the rack and against dishes the goal does not rest on; a nesting pair judged by its settled distance instead).
4. Pairwise edges cannot hold blocks that depend on where an earlier dish happened to land (r07_00012: bowl_01's landing decides whether bowl_04 can enter), nor three-body needs; the learning loop adds edges without converging. A graph over slots rather than poses, with disjunctive needs ("B needs one of {A, C}"), is the smallest model that can.
5. The test harness is not order-pure: a library order's outcome depends on which build ran before it in the same Isaac session (8 of 53 orders changed for that reason alone; racks move < 0.3 mm, so it is not the rack). Every library order needs its own Isaac session or a full scene reset; until then no verdict, valid or invalid, is trustworthy.
6. State-restore backtracking is not a move: its witness is not an order (0 of 7 rebuilt). Backtracking must be physical (remove and re-place) or absent.

## Gap
No round tested the slot-level goal, the nesting-aware contact rule, or an order-pure harness; every "invalid" verdict so far was made under at least one of the defects in findings 3 and 5, so the method is untested rather than refuted.

## Candidate ideas (max 3)
1. Order-pure harness + slot-level landing: one Isaac session per library order (about 20 s of boot each, 26-56 orders per sample), landed = same slot, same supports, same lean side, contact judged against the rack and non-support dishes only; rerun r07_00012 and r05_00027. Might work because every remaining miss is one of findings 2, 3 or 5. Might fail because slot membership needs a definition for bowls on plates. Cheapest test: the two samples, about 2 h of box.
2. Goals from a sequential sampler only (no packing targets): drop the dishes into free slots from above in a lean-side order and let the result be the goal, scored afterwards; the graph over slots. Might work because reachability is by construction and no pose target exists to miss. Might fail because exposure scores fall below the packings'. Cheapest test: 10 loads, about 2 h of box.
3. Stop: keep the sequential-goal pipeline and the page as a documented negative result; use the bench's own sequencing certificate for goals. No run.

## Picked
Not picked: the user paused after round 6 to decide from this note.

## To grill
- Whether a slot-level goal still serves the planners (they move dishes to poses, not slots).
- The cost of one Isaac session per order against a full in-session reset, which has never been verified to be order-pure.
- Whether the restricted packings should be regenerated with insertion in mind rather than repaired by the sampler.
