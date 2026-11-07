# A goal sampler that admits only poses this arm can place
Slug: 2026-11-05-hardware-aware-sampler | Lit review: research/2026-11-05-hardware-aware-sampler-litreview.md

## Question
Can the fit-score-prove sampler produce a goal load for s0's roster that the UR5e with the side-mounted 2F-85 at posture_v2 places 15 of 15 by direct releases, by admitting only poses the Kit-free robot gate passes in a pack, gate, forbid, re-pack loop, and what does that cost in S?

## What's known (max 5)
- Dantam et al. (2018, IJRR): an SMT task planner takes its cuts from the collision set at three strengths (forbid the plan, forbid the operator in that state, forbid it while the blockers stay). Motion-planning time became sub-exponential in the number of blocking objects. Our analogues are a unary pose ban, a pairwise conflict and a whole-load no-good.
- Quintero-Peña et al. (2023, ICRA): shrinking each cut to an irreducible infeasible subset solved 100% of 14-action problems in 19.98 s against 4% in 301.80 s without. Lagriffoul and Andres (2016) solved nothing in one scenario without culprit detection. The size of a cut decides whether the loop converges.
- Erdem et al. (2016): with all 29,282 feasibility checks precomputed the first plan took 888 s, against 2,007 s when checks ran lazily. Precomputation wins when the check set is enumerable and context free, which holds for empty-rack IK and the open-hand hull at each of our 1487 candidate poses.
- Wang et al. (2022, ICAPS): order rules read from the arm's overlap at grasp poses, with motion planning only on branches that reach the goal, solved 82.4% of 16-object problems against 25.8%. The goal arrangement stays fixed in every robotics source; only the OR papers (Iori et al., 2007) re-choose the packing under order-dependent loading checks, and they have no arm.
- Qin et al. (2026, preprint) placed 35 of 35 objects with the gripper modelled in the placement shape, 7 of 35 without. The three dishwasher or rack sources (Jiang 2012, Voysey 2021, Jain 2023) score or settle rack poses and never model arm feasibility.

## Gap
No source re-chooses an exact packing after an arm-and-hand check fails, none tests the open-hand release as its own check, and none reports how many re-packs a lazy loop needs. In our repo the gap is concrete: s0's goal fails a direct-release rule for 5 of its 15 dishes (three inserts, two drops), the packer has a bans seam the score ascent also obeys but no seam for a one-way order edge, and the stored goal poses are the Isaac-settled ones, up to 44 mm from the packed poses.

## Candidate ideas (max 3)
1. The lazy loop the user chose: CP-SAT packs, the access-order tool gates the load in dag order with the earlier dishes in the scene, each failure becomes a cut chosen by its collision set (a ban on the pose when it fails alone, a pair no-good when it fails only beside one named dish, a whole-load no-good otherwise), and the packer re-solves until every dish passes a direct release; then the Isaac build, gate, certify and score as now, and the settled goal is gated once more. Might work because 10 of s0's 15 dishes already pass directly, the fit takes 3 s and a scene-aware gate 8 to 16 min, so a handful of iterations is affordable. Might fail because cups have only 24 candidate poses, the settled pose can drift from the gated pose, and the box's score ascent moves dishes to poses the gate never saw. Cheapest test: s0's roster at seed 0, the loop on the Mac, then one box window for the Isaac stages and the full robot chain.
2. A context-free pre-filter: before CP-SAT, check empty-rack IK and the open-hand hull at every candidate's release pose against the rack with the basket in, drop the failures, and leave pairs to the lazy loop. Might work because 1487 poses times a dozen spins is enumerable with a static FCL check. Might fail because the release pose needs the hold per kind and the carry is not static. Cheapest test: build the table for s0's three families and count survivors per kind.
3. A learned feasibility classifier over the gate's verdicts that orders candidates or sets CP-SAT hints, with the exact gate as the final judge. Might fail now for lack of labels (a few hundred rows). No test this round.

## Picked
1, because the user chose it in the first grilling round and the literature says its convergence hinges on cut size, which the gate's named blockers already give us; 2 can join later as the loop's first pass if the iteration count runs high.

## To grill
- Which pose the Kit-free gate judges before Isaac exists: the packed pose, or the rest model's predicted landing, with the Isaac-settled goal gated again after the build.
- The cut per failure: how a named neighbour becomes a pair no-good (the packer has no one-way edge), how far a deletion filter shrinks it, and whether a carry timeout is a retractable cut.
- Whether the score ascent runs at all for the hardware-aware sample, since its moves are not gated, and how its S is reported against s0's S_ref 0.513.
- Where the loop runs (the Mac has no Warp) and how many iterations the budget allows before the sample is called infeasible.
- The proof: one box window with the Isaac stages and the full robot chain, judged as the fifteen rounds were.
