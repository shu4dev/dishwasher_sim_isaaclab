# Run note: do lower-rack bowls chain from a support side? (Mac only, stdlib, 2026-10-23 00:30 ET)
From: results/2026-10-22-bowl-rest-pose-dag.md (the lone-bowl table failed for lower-rack bowls: 8 of 90). Question: on the
certified loads, is the prerequisite of a lower-rack bowl-bowl access hit predictable from the packing, the way a plate's
is (the lean-side chain)? Data: the 32 certified valid records on the Mac (r05_00027 and the valid pairs of the rest-pose
and bowl-plate rounds, `data/results/sampler/{rest,bp}_<tier>/`), 44 lower-rack bowl-bowl hits (a needs b first) in
three zones: lower_frontright 17, lower_rear 18, lower_rearright 9.

| rule for b, the prerequisite | all 44 | frontright 17 | rear 18 | rearright 9 |
|---|---|---|---|---|
| b has the smaller packed x (nearer the plate bank / wall) | 34 (77 %) | **16** | 10 | **8** |
| b has the larger packed x | 6 | 1 | 4 | 1 |
| b rests higher than a (settled z, > 5 mm) | 33 | 17 | 10 | 6 |
| b packed higher than a | 21 (48 %) | | | |
| b on the side a's axis leans toward (settled) | 9 | 6 | 1 | 2 |
| b on the opposite side | 28 | 10 | 11 | 7 |
| b has the larger packed tilt angle | 10 | 6 | 2 | 2 |
| b has the larger lift | 21 | 5 | 11 | 5 |

- **Side zones chain, the rear row does not.** In lower_frontright and lower_rearright the prerequisite is the bowl
  nearer the small-x end in 24 of 26 hits: bowls go in from the plate bank (or wall) outward, the plates' rule again.
  In lower_rear x decides 10 of 18 (4 pairs share an x), lift 11 of 18, nothing reaches 80 %: the rear row has no
  single support side along x, and which bowl must go first there is set by the settled geometry, not the packing.
- **What the hit is physically:** the later bowl (a) ends lower (33 of 44 by more than 5 mm; 17 of 17 in the front-right
  zone), the pair never has a resting contact (0 of 44), and the hit is at the first step of a's descent (lift 54-100 mm):
  the bowl that settles lower spreads into the column its neighbour comes down through. Packed heights do not carry
  this (48 %), and the lone-rest table cannot (the lone bowl tips over).
- **Witness orders are not chains** (front-right pairs monotone along +x in only 9 of 27): the build order is the seeded
  permutation with only the constraints enforced, so the chain shows in the hits, not in the order.
- **Verdict on the plan's bar (10 of 12 loads):** not met as a general rule; met zone by zone for the two side zones.

## Next smallest step
Bowl-bowl edges in the two side zones from the zone's chain direction (smaller x first), the measured table for upper
bowls and cups, the rear row left to re-pose and deferral; then the three-tier rerun. If the rear row still blocks,
its pairs need an order test at build time (the two bowls' descents against each other's settled pose after the first
lands), not a static rule.

## Correction (2026-10-23 03:50 ET, found by the side-zone run)
The hit direction above is read backwards: a certified hit {a, b} means **a goes in before b** (a is the prerequisite,
`graph.py`: "a must be in place before b"), not "a needs b first". So in the side zones the prerequisite is the bowl
with the LARGER packed x (24 of 26 hits), the chain runs from the large-x end inward, and "the later bowl settles
lower" reads "the earlier bowl settles lower". The side-zone run (`results/2026-10-23-side-zone-chain-edges.md`) used
the corrected direction (11 of 13 hits held) and still lowered the yield: an x order forces pairs that sit side by side
along y, which have no constraint, and cyclic draws rose from 49 to 78. The rear-row finding (no rule) stands.
