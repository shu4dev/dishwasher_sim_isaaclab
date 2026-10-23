# Build order from the packing's access and support DAG; cyclic packings rejected before Isaac
Slug: 2026-10-22-dag-build-order | From: results/2026-10-22-sampler-rate-easy-medium.md (follow-up, no lit review)

## Goal
Buildability becomes a property of the packing, decided Kit-free before any drop: the insertion order is a topological
order of the access and support constraints at the final poses, and a packing whose constraints contain a cycle is
rejected at the packer. Measured on easy, medium and hard, 20 seeds each, against today's per-stage table.

## Decisions (grilled 2026-10-22 11:05 ET)
- Root cause: the sampler discovers by trial (random order, drop, re-pose, defer) a constraint the packing fixes: 41 of
  46 hard and 12 of 12 medium build failures are a dish whose descent path a neighbour's final pose covers. The lean-side
  chains, re-posing and deferral were partial readings of it. Rejected: backtracking, a wider re-pose set.
- Access edge (b before a): the kit's `access_hits` (frigidaire_dependency_kit.py:529, the move's 1 mm path check) run
  on the packing's final poses before the build, not after. Support edge (a before b): at the packing poses b's lowest
  point lies within 2 mm above a's surface inside b's footprint (FCL distance on the same collision pieces; a below b).
- Order: a topological order of the DAG; ties broken first by the lean-side chain rule within a plate bank
  (`chain_order` as tie-breaker), then by the seeded permutation. Deferral and the chain walk go. `repose` stays only
  for a physical miss (a dish that settles > 80 mm off on a clear path): up to 5 free poses whose own path check is
  clear and which add no cycle.
- Cycle: the packer draws again (seed*1000+k, k < 10) and the rate table gains a row "packing had a cycle"; the record
  stores the cycle's dishes. Rejected: local repair of the cycle.
- Scope: order and packer gate only. Round 2 (separate plan): one overlap function for build and gate, one landing
  criterion for build and witness (the 1.04-1.25 mm gate rejections and the witness-rebuild misses).
- Measure: `sample.py --difficulty easy|medium|hard --from build --to certify --seed 0..19`, hard = the preset's 23
  dishes, the per-stage table before (today's note; hard before = 0 of 40 v7 and 0 of 12 CLI seeds) and after, plus per
  seed the DAG's edge counts and layer count, and whether the built load's witness order is topological.

## Change
- New Kit-free `code/src/sampler/order.py`: `access_edges(entries)`, `support_edges(entries)`, `dag_order(entries, seed)`
  (returns order, edges, layers or the cycle). Reuses `access_hits` and the bench's FCL checker; no new dependency.
- `code/src/sampler/build.py`: after `pack`, `dag_order`; a cycle re-draws; the order is written beside the load file and
  passed to the kit as `--order <file>`; the stage note records edges/layers.
- `frigidaire_dependency_kit.py --load`: with `--order`, `build_sequence(order=..., chains=tie-break only)`, no deferral,
  re-pose only on a physical miss. Without `--order` the old behaviour stays (the loop is untouched this round).
- Test: on r05_00027's packing (`data/results/dot_rules/restricted/loads/r05_00027.json`) `access_edges` reproduces
  the 11 access hits of its record and `dag_order` returns an order consistent with its witness.

## Run
Box, 4 slots as in plans/2026-10-22-sampler-rate-easy-medium.md, easy then medium then hard, 20 seeds each; today's
`tally.py` extended with the cycle row and the DAG columns; about 1.5 h of box; stop it right after.

## Works when
Medium builds more than 0 of 20 and easy builds at least 19 of 20 (no regression); every built load's witness order is a
topological order of its pre-build DAG; the test passes; the note has the before/after table, the cycle rate and the
DAG stats; the box is STOPPED.

## Assumed / skipped
Assumed: the vertical insertion move stays; the packer's own "no free pose" failures are not addressed. Skipped: the
kit's --sample loop, round 2's tolerance unification, pages.
