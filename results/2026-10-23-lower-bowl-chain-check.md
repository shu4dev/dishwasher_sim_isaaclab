# Run note: do lower-rack bowls chain from a support side? (Mac only, stdlib, 2026-10-23 00:30 ET)
From: results/2026-10-22-bowl-rest-pose-dag.md (the lone-bowl table failed for lower-rack bowls: 8 of 90). Question: on the
certified loads, is the prerequisite of a lower-rack bowl-bowl access hit predictable from the packing, the way a plate's
is (the lean-side chain)? Data: the 32 certified valid records on the Mac (r05_00027 and the valid pairs of the rest-pose
and bowl-plate rounds, `data/results/sampler/{rest,bp}_<tier>/`), 44 lower-rack bowl-bowl hits in their
`goal.certify.hits` (r05_00027: `access.hits`). A hit {a, b} means b's final pose blocks a's insertion path, so a goes in
before b: a is the prerequisite. By a's zone: lower_frontright 17, lower_rear 18, lower_rearright 9 (3 with b in the same
zone, 6 with b in lower_rear). The 11 valid bp_easy records repeat rest_easy ones (identical `goal.entries`); without
them there are 21 records and 28 unique pairs.

| rule for a, the prerequisite (a goes in before b) | all 44 | unique 28 | frontright 17 | rear 18 | rearright 9 |
|---|---|---|---|---|---|
| a has the larger packed x | 34 (77 %) | 21 | **16** | 10 | **8** |
| a has the smaller packed x | 6 | 5 | 1 | 4 | 1 |
| a rests lower than b (settled z, > 5 mm) | 33 | 21 | 17 | 10 | 6 |
| a packed lower than b | 21 (48 %) | 15 | 5 | 11 | 5 |
| b on the side a's axis leans toward (settled, within 75°) | 9 | 7 | 6 | 1 | 2 |
| b on the opposite side (beyond 105°) | 28 | 16 | 10 | 11 | 7 |
| a has the smaller packed tilt angle | 10 | 7 | 6 | 2 | 2 |
| a has the smaller lift | 21 | 15 | 5 | 11 | 5 |

- **Side zones chain, the rear row does not.** With a in lower_frontright or lower_rearright, a has the larger packed x
  in 24 of 26 hits: 18 of 20 when both bowls are in one side zone, and 6 of 6 when a lower_rearright bowl goes before a
  lower_rear one. The unique pairs give 15 of 17 (11 of 13 inside one zone). The chain runs from the large-x end inward.
  In lower_rear the larger x goes first in 10 of 18 (4 pairs share an x) and the smaller lift in 11 of 18; no rule
  reaches 80 %. The rear row has no single support side along x, and which bowl must go first there is set by the
  settled geometry, not the packing.
- **What the hit is physically:** the earlier bowl (a) ends lower (33 of 44 by more than 5 mm; 17 of 17 in the
  front-right zone), and the later bowl (b) rests on it in 33 of 44 (`contacts.supports` in the kit record that
  `goal.record` names); in 26 the confirmation drops find a hard support (`confirmations[].hard`). The hit comes at the
  first step of a's descent in 25 of 44 (lift 54-95 mm) and lower in the rest (steps 2 to 13): the bowl that settles
  higher sits in the column the lower one comes down through. Packed heights do not carry this (48 %), and the
  lone-rest table cannot (the lone bowl tips over).
- **Witness orders are not chains** (front-right pairs put the larger x first in 18 of 27): the build order is the
  seeded permutation with only the constraints enforced, so the chain shows in the hits, not in the order.
- **Verdict on the plan's bar (10 of 12 loads):** not met as a general rule; met zone by zone for the two side zones.

## Next smallest step
Bowl-bowl edges in the two side zones from the zone's chain direction (the larger packed x first), the measured table
for upper bowls and cups, the rear row left to re-pose and deferral; then the three-tier rerun. If the rear row still
blocks, its pairs need an order test at build time (the two bowls' descents against each other's settled pose after the
first lands), not a static rule.

## History
- First version (00:30 ET): it read a hit {a, b} as "a needs b first", so b was its prerequisite and the table said the
  smaller x goes first; `access_hits` (`code/planner/frigidaire/frigidaire_dependency_kit.py`) records a hit when b's
  final pose blocks a's insertion path, which means a goes in before b, so the counts stand with the labels swapped
  (found by the side-zone run at 03:50 ET).
- The side-zone run (`results/2026-10-23-side-zone-chain-edges.md`) used the larger-x direction (11 of 13 unique hits
  held) and still lowered the yield: an x order forces pairs that sit side by side along y, which have no constraint,
  and cyclic draws rose from 49 to 78. The rear-row finding (no rule) stands.
- 2026-10-31: the body was rewritten in the a-before-b reading from the records, with the unique column added and the
  lean rows' rule stated (a 75° cone, which reproduces their counts). Two claims of the first version were wrong: "the
  pair never has a resting contact (0 of 44)" (b rests on a in 33) and "the hit is at the first step of a's descent"
  (25 of 44).
