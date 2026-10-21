# Physical-cause dependency graphs on gated, certified loads
Slug: 2026-10-16-physical-cause-graph | From: research/2026-10-16-dependency-graph-six-rounds.md (follow-up, no lit review)

## Goal
A valid sample is three things and nothing more: a load that passes the joint gate, a witness order that rebuilds in a fresh Isaac session, and a graph whose every edge is a physical cause in the finished load (support from resting contacts, access from a covered insertion path); the page shows 5 such samples or the stop point.

## Decisions (from grilling, 2026-10-16)
- The claim per sample: (1) physically possible and stable = the built poses pass the bench's joint gate (loaded hold, peak 5 mm / median 1 mm, both racks retract, containment); (2) one sequential order exists = the witness, replayed from empty racks in a fresh Kit process under the same move, lands all 24; (3) A before B only for a physical cause in the finished load. No library, no sufficiency claim, no coverage number: the page says that no edge means no physical cause was found, not that any order works. Rejected: swap-test edges (context-relative), knock and learned edges, any statement about untested orders.
- Support edges: from Isaac's resting contacts after the certified rebuild (contact force pointing up from A into B; the build already records rests_on). B resting on several dishes gives one fan edge "B needs one of {A, C, ...}"; a single support a plain edge. Each is confirmed by dropping B into the finished load with all its supports removed (it must fail to land) and with each support alone present (which ones suffice is recorded). Rejected: geometry-only support, one edge per support, the main support only.
- Access edges: "B before A" when A's final pose intersects B's descent path under the move's own path check at the final poses (Kit-free). Evidence is a still of the covered path; no Isaac drop (it would repeat the path check). Rejected: drops for access edges.
- If A before B and B before A both occur, no edge is drawn and nothing is claimed about independence; the fans keep three-body supports visible.
- Sampler unchanged (upright plates with the lean bias, lean-side chains in a random order, re-posing, deferral, no backtracking, the own-load rule). r07_00012 and r05_00027 first (gate, fresh-session rebuild, edges), then new packings until 5 valid, 20 packings or 3 h of box.
- Page, per sample: load still with S and the gate numbers; the witness clip from the fresh-session rebuild; the graph (support solid, access dashed, fans for disjunctions, free dishes marked); per support edge its confirmation clip, per access edge its covered-path still. The library section goes.

## Change
- `frigidaire_dependency_kit.py`: `--edges physical`: after the sampler (a) gate the built poses (manifest from the built entries, `frigidaire_initial_state_validate.py` with the bench's flags), (b) `--certify`: rebuild the witness in a fresh Kit process, (c) edges from the rebuild's resting contacts plus the Kit-free access check (`insertion_release` at the final poses), (d) confirmation drops for support edges; record `samples_v5/<id>.json` with `gate`, `certified`, `edges` (support, access, fan), `confirmations`.
- `frigidaire_dependency_clips.py`: confirmation-drop clips and covered-path stills; `frigidaire_dependency_page.py`: the three-item claim, the new legend, no library.

## Run
`python3 code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --sample --target-valid 5 --max-packings 20 --plate-order lean-side --backtrack 0 --release insertion --edges physical --resume-from data/results/dot_rules/dependency/samples_v4 --out-dir data/results/dot_rules/dependency/samples_v5 --max-wall-seconds 10800` on the box (scheduler over cores 0-13), then clips, page, publish to the same URL.

## Works when
`samples_v5/` holds valid samples with the gate accepted, `certified` true (fresh-session rebuild 24 of 24), every support edge with a failed confirmation drop and its clip, every access edge with its still; the page shows the tuples; the run note gives valid / attempted and per sample the edge counts (support, access, fans) and the gate numbers.

## Assumed / skipped
- Assumed: the gate runs on the built state in the extended racks as the bench's sequence gate does; a confirmation drop uses the move (insertion, 20 mm gap) at B's goal; a dish with no support contact and no covered path is free; about 30 min of box per sample; one window.
- Skipped this round: the library, three-body access, the bench, the Mac-side Kit-free setup.
