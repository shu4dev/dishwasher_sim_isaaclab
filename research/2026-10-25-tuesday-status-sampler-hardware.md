# Tuesday status, two parts: the sampler and the hardware. What is firm, what is limited, what is only a hypothesis
Slug: 2026-10-25-tuesday-status-sampler-hardware | Lit review: none | For: the 2026-10-27 report
From: results/2026-10-22-*.md and results/2026-10-23-*.md (sampler, score, hardware), plans/2026-10-23-hardware-limits-mouth-up.md,
research/2026-10-23-robot-grasp-reach-limits*.md, code at 5b3f2d7. Every number below is quoted from a run note with its n;
"box" = the Brev box dishsim-l4 (g6.2xlarge, 4 cores) unless marked corallab. Nothing here was re-run for this note; the
D4 `counted` field and `SIDE_ZONE_CHAIN = -1` were checked against the records on the Mac and the code at HEAD.

## Part 1. The sampler (goal loads, build order, score, page)

### Say with confidence
- **It exists and runs end to end.** `code/src/sampler/sample.py`, 8 stages: load, initial, gate_initial, build, gate_goal,
  certify, score, graph. Build = restricted packer, then a DAG order (`build.order=dag-rest`, cyclic packings redrawn up to
  10 times), then the Isaac build with re-pose and deferral; gate = joint gate; certify = witness rebuild in a fresh Kit
  process; graph = physical-cause arrows. Pair file and stage ledger rewritten after every stage. (window-sampling-v7 note,
  repo-refactor-sampler note; 263 tests pass in 64 s.)
- **Valid loads on record: 27 distinct, easy 17 / medium 7 / hard 3** (7 bowls; +8 plates = 15; +8 cups = 23 dishes),
  deduplicated by the goal's chosen entries, all scored under the same per-arm fair score. Best single rounds: easy 12/20
  (rate and plate-rest rounds), medium 5/19 (bowl-plate round), hard 2/20 (plate-rest round). (best-score-sample-per-tier.)
- **Why medium never built before the DAG:** 0/20 built; in all 12 build failures a dish's insertion start was blocked by an
  already-placed neighbour. Crowding, not the move. (sampler-rate-easy-medium.)
- **A DAG at the packer's own poses is useless:** 83/83 medium and 91/91 hard draws cyclic. (dag-build-order.)
- **The measured plate rest table works.** 38/38 lone-plate settles; 31/32 loaded plates within 10 mm of the table (24/32
  within 3 deg and 10 mm); front/rear bank medians 122.2/83.4 deg match. It removed every cycle and gave the first valid
  23-dish loads (hard 2/20). On every built load the witness order was a topological order of the DAG (24/24). (plate-rest-pose-dag.)
- **Bowl-plate edges removed bowl-over-plate blocking** (20 of 21 build failures blocked by a bowl before, 0 of 12 after)
  and gave the best medium, 5/19. Hard fell 2 -> 1 because the new edges re-drew its two valid packings. (bowl-plate-edges.)
- **A lone-bowl rest table does not predict loaded lower-rack bowls:** 8/90 within 3 deg and 10 mm; 20/90 tip onto their
  side alone. Upper bowls 52/58 and cups 24/32 do match. (bowl-rest-pose-dag.)
- **In the two lower side zones, packed x decides which bowl goes first** (24 of 26 certified hits; the larger x first,
  after the 03:50 correction). The rear row has no rule (10/18). (lower-bowl-chain-check.)
- **Encoding that as chain edges made things worse:** medium 3/20, hard 0/20, cyclic draws 78 vs 49, 7 front-right build
  failures caused by the chain. The round was stopped during easy. (side-zone-chain-edges.)
- **The hard tier's failure mode is bowls blocking bowls:** 12/12 build failures are a bowl that never goes in, 11/12 first
  blocked by another bowl, 6/12 with plates let in by deferral that the bowl had to precede. 7 of 20 hard seeds and 8 of 20
  medium seeds never pack at all ("no free pose"), identically in every round. (bowl-plate-edges; every failed-seed list.)
- **Cost:** median seed 285-312 s easy, 134-359 s medium, 157-555 s hard; a 60-seed round is about 67 min on 4 slots.
  The retired kit `--sample` loop on this box: 0 valid of 50 packings; r05_00027 is the only load it ever produced.
- **Score (fair S):** per dish, object-level New+J over its kind's ceiling, the ceiling computed from the grids as the best
  the kind reaches alone inside the arm's disc (lower arm, theta 30 deg: plate 0.8758, bowl 0.6012, cup 0.2287; middle arm
  0.8587 / 0.5525 / 0.2099), 0 for a pooling dish, S = plain mean. Reproduces the earlier constants from their inputs;
  independent recompute matches 280/280 states; the ray cast is bit-identical across the two boxes (192 cells). Tilt check:
  inside the disc the best tilt is 0 deg, on-edge keeps ~12 %, upside down gives 0 (18/18 pairs). (inward-ceilings-and-smokes;
  tilt-score-check.)
- **Page:** Dependency graphs Version 18 (live) shows one complete sample per tier, initial + goal + S + graph + clips:
  easy S 0.2826, medium 0.1647, hard 0.3889, plus r05_00027 at 0.3545. The highest-S picks (easy 0.3931, medium 0.2320,
  hard 0.3889) were built as Version 19 on the box and are not recorded as published.

### Known limits
- `gate_initial` always records SKIP: no pair is gated end to end; the section-2 "works when" needs both gates.
- **HEAD's default is the worst-measured configuration.** `SIDE_ZONE_CHAIN = -1` is committed (9d84df3) with no flag to turn
  the chain edges off; `sample.py` today reproduces medium 3 / hard 0, not the bowl-plate round's 5 / 1.
- The packer caps the tiers: at most 12/20 medium and 13/20 hard seeds can ever be valid. No round touched this.
- No pre-build predictor of a bowl's landing pose exists; bowl-bowl order rests on re-pose and deferral, and the rear row has
  no rule at all. The chain rule is right in direction and wrong in scope (only pairs at least 15 mm apart in x).
- Hard is 23 dishes, not 24: not comparable with r05_00027 or the old loop; the kit's verdict text still says "24 of 24".
- The plate table missed its own gate (24/32 vs 27 required) and the gate was relaxed; plate edges are validated on one load.
- n is small: 19-20 seeds per tier, hard 0-2 valid. A 2 -> 1 -> 0 change in hard is within noise.
- The bp round removed dag_accept's bowl/cup shortcut at the same time as adding edges: the two effects are confounded, and
  bowl re-pose got much slower (hard median 555 s).
- One pytest fails by design (the packed-model r05 test): the smoke suite is not green.
- The page's yield table still counts the retired loop's 70 packings, not the 27 valid loads.
- **S is a line-of-sight proxy by construction**, never validated against a wash or a spray simulation. Call it an exposure
  ranking, never "cleaner".
- Middle-arm ceilings are lower-arm grids cut to r 0.191 m, argmax at the grid's top height and still rising: slightly low
  (middle-arm cups already reach 0.960 of the ceiling). The tilt sweep stops at -60 deg; the disc-limited rule is a decision.
- Kinds reach their ceilings unevenly in racked loads (plates about 0.08-0.10, cups about 0.68): S ranks loads within a tier;
  medium 0.232 vs hard 0.389 is kind mix, not quality.
- A graph arrow is a found cause; no arrow means no cause found, not "any order works". Sufficiency was never claimed.
- The Dot Score Rules explorer's "relative" view divides by the outward-only constants, the load pages by the per-arm ceilings.

### Untested hypotheses, with the smallest test
1. Chain edges only for pairs at least 15 mm apart in x (none for side-by-side bowls) beat the bowl-plate round. One flag in
   `chain_edges`, medium + hard seeds 0-19, about 90 min.
2. Side-zone edges do not regress easy: 12 seeds (5, 9-19) never ran. About 20 min.
3. A DAG-respecting deferral removes the 6/12 "plates got in first" failures. Re-queue a deferred dish only ahead of its
   successors; rerun medium + hard.
4. Rear-row bowl pairs can be ordered at build time by a descent check against the settled neighbour. Kit-free on the 6
   rear-row failures and 18 hits first.
5. One shared overlap function removes the gate-preflight rejections at 1.04-1.25 mm. Kit-free, minutes.
6. The first-dish certify failures (medium s3, hard s10) are reset bugs, not physics. Re-certify twice each.
7. The build order, not PhysX variance, decides which easy seeds pass (rate vs rest: both 12/20, valid sets overlap 6/12;
   bp repeated rest byte for byte). Rerun easy under dag-rest once.
8. Box-hours per valid load: derivable from the logs already on the box (`tally.py`); no run.
9. Higher S means a cleaner wash. Only a real wash answers it (easy winner 0.3931 vs the lowest easy 0.1836).
10. The kind ceiling equalises kinds. Per-kind histogram of S_kind over the 586 scored dishes, Mac, minutes.
11. The middle arm needs its own grid (heights above 26 cm). One object-level grid.
12. The two new winners are robust (margins 0.0025 and 0.0018, one build each). Rescore the witness-rebuild poses.
13. Graph-obeying orders build the load (sufficiency). 5 random obeying orders in fresh Kit processes.

### Fix before Tuesday (notes disagree)
- Chain direction: the plan and the memory say smaller-x first; the code, the correction and the rerun say larger-x first.
  The chain-check table body still reads backwards. The hit counts (24/26 vs 11/13 unique) do not reconcile.
  (fixed 2026-10-31, see results/2026-10-23-lower-bowl-chain-check.md; the plan and the memory were not edited)
- `exposure.md` and CLAUDE.md quote the superseded ceilings 0.8718 / 0.5481 / 0.2134 and "the middle arm uses the same
  constants"; the code computes per-arm ceilings. `dependency.md` describes the 24-dish sampler and S 0.3551 (now 0.3545). (fixed 2026-10-31: exposure.md, CLAUDE.md and dependency.md carry the per-arm ceilings, the tiers' rosters and S 0.3545, each with its source)
- CLAUDE.md names the old box (isaac-launchable-2dd875, 14 workers); everything since 2026-10-22 ran on dishsim-l4. (fixed 2026-10-31 in CLAUDE.md; the Mac's `brev` git remote still points at the old box, the user's to change)
- Dot Score Rules: CLAUDE.md says Version 23; Version 30 is live (hand patches; a rebuild gives 23). (fixed 2026-10-31)

### Do not quote
The 2026-10-15/16 sampler rounds (old box, old validity definition), every S under the outward-only or "anywhere" ceilings,
the v3-rack exposure results, the HOTEC exposure S before 2026-09-28.

## Part 2. The hardware (UR5e + Robotiq 2F-85, mouth-up, Frigidaire twin)

### Say with confidence
Everything here is Isaac 4.5 on the box plus the Kit-free FCL model, at one fixed mount (0.45, -0.55, 0.90; yaw 2.60), one
counter spot (0.20, -0.16), one dish in an empty rack. No trial ran on a real robot. (hardware-limits-mouth-up, rounds 1-3.)
- **Grasp (G3: lift 0.15 m, hold 5 s, n = 10, clearance on):** bowl 10/10, every bowl pivoting about 21 deg onto the knuckles
  and then hanging still; cup 9/10. Plate: with the 15 mm clearance on, no approach exists; with it waived, 3/10, and the
  three holds pivot 33-44 deg.
- **The plate surface slide (the literature's recipe) fails 0/25:** the 2F-85's outer linkage hangs about 77 mm below the TCP,
  so the slide pushes the plate 50-53 mm and the jaws close on nothing.
- **No packer goal pose is reachable with a top-down rim pinch:** 0/50 sites pass G1 over 12 yaws (bowl and cup families are
  mouth-down, plates fail at the counter grasp). Isaac agreed on all 5 rejections it was given.
- **A level mouth-up dish has a collision-free path at most sites:** G2 at 15/19 bowl sites and 8/12 cup slots (Kit-free,
  HOME_Q seed, numpy RRT-Connect 3 s). Isaac blocked 3 of the 15 bowl sites.
- **Level bowls place reliably at the upper slots and two lower-rear sites, never at the right-side sites** (pooled, n 6-9):
  upper 01/02/03/07 8/8, 9/9, 6/6, 6/6; lower rear x -0.12 and -0.07 9/9 each; x -0.17 3/9; front-right 0/9 x 3, rear-right
  0/8, the bowl tilting 21-35 deg on the tines. Lowering until contact: 2/12, median tilt 30 -> 17 deg.
- **Cups do not rest where released:** in-air level 0/24 (drop 61-68 mm); contact release 3/24, left slots only; the right
  slots' early release was the detector's lag rule (0.050-0.054 rad against 0.05); fixed, still 1/12 and 20-32 mm of drop.
- **Orientation:** Kit-free, no site has a path at 150 deg (bowls at 0/30/60/90/120/150: 8/10/10/5/0/0 of 15; cups
  6/7/5/2/2/0 of 8). Isaac, n = 5: bowl upper 02 level 5/5, 30 deg 3/5, 60 deg and up 0/5; cup right glass 1 level 4/5,
  30 deg 0/5.
- **The round-1 Isaac vs Kit-free disagreement is explained:** the IK seed, not the arm models (36/36 cells agree with the
  same seed; HOME_Q seed 12/12 vs Isaac-joint seed 7/12, 0/12, 0/12 at the three hovers).
- **Literature (28 sources, all verified to exist):** rim pinch on upright bowls and cups works with a 2F-85 (Srinivas 2023,
  0/36 bowl failures); a flat plate is a gripper limit, not a reach limit (Voysey 2021: pinch 12/24, slide 23/24, other
  gripper); path-gated placement beats endpoint-gated (Wang & Hauser 2019, 79 % vs 17 %); mouth-down grasps are untested
  in the literature, not disproven.
- The plan's success bar for round 1 was met by the note's own verdict; the box was stopped after each of the three windows.

### Known limits
- **The carried-contact invariant D4 fires in every bowl and cup trial** (the rim rests on the inner knuckles), and the records
  carry `counted = 0` for all of them, while the plan said a trial counts only after the invariants pass. The 10/10 and 9/10
  are physical outcomes with an invariant tripped. Whether the knuckle contact is real or a collider artefact was not checked.
- The three objects were measured under different conditions: plate pinch with the clearance waived, plate slide with the
  gripper allowed to touch the counter, no plate G4 at all. They cannot be ranked on one condition.
- The lowered-until-contact G4 is judged against the pose at release, not the site's target: a cup set 62 mm low "passes".
- Placement and the slide used the legacy 0.08 rad motion tolerances; the grasp gate used the defaults.
- Every reach and placement is to a level mouth-up target at the site's clear height, never the benchmark's goal pose.
  Executing HOTEC goals needs a regrasp or a wrist flip, which does not exist.
- The Kit-free counts depend on the HOME_Q seed and the 3 s numpy RRT budget (no OMPL in the container); budget timeouts were
  never counted apart, although the plan asked.
- One mount (chosen on corallab for easy_s0's dishes, not for rack slots; yaw never searched; z at the top of the grid), one
  counter spot (geometric), one dish, empty rack. Every "unreachable" holds only there.
- n is small: G4 n = 3 in round 1, 6-9 per site in round 2, n = 5 at one site per class and the first G2 yaw in round 3.
- The jaw-angle filter stands in for object detection; cups (0.78 rad) and plates (0.456-0.468) sit at its edges.
- Simulation with privileged state (dish and in-hand poses read from the simulator, gravity off on the robot links); the
  Isaac 6.0 asset on Isaac 4.5 with de-instanced gripper colliders; videos are Warp replays of collision hulls, not renders.
- No datasheet number appears in any note: the 2F-85 stroke, grip force and payload and the UR5e reach would be quoted from
  memory. The plan asked for pad forces per trial; none are reported.
- "Mouth-up only" is this round's scope, not a result that mouth-down is infeasible. The round's decisions (fixed mount,
  flat-plate pinch as the plate test, n = 3) were made with the user away.

### Untested hypotheses, with the smallest test
1. The knuckle contact is real, not a collider artefact. One bowl G3 trial, dump the contact partner and force per tick,
   compare the inner-knuckle hull with the visual mesh at 21 deg.
2. The reach table holds with seed-free IK (2 pi unwrap inside the limits). Rerun the level sweep; compare with 15/19, 8/12
   and Isaac's 3 blocked sites.
3. Some G2 failures are 3 s budget timeouts. Count the "G2: budget" rows; rerun those cells at 30 s.
4. The right-side sites tilt a bowl past 15 deg because of the tines, not the robot. Robot-free teleport + settle, n = 5.
5. A cup lowered past where the arm stops rests at the target. Continue the descent past the 0.15 rad block, right slots, n = 5.
6. The right-glass level pass (4/5 vs 0/3) came from the yaw. Rerun with both yaws, n = 5 each.
7. A plate primitive that fits the 2F-85 works: a pitched slide with the lower pad 2 mm above the counter, or a rim pinch on
   a propped plate, G3 n = 25 with the clearance on (the literature's bar: about 20/25).
8. Packer poses become reachable with a regrasp or a wrist flip. Kit-free G0-G2 at one bowl site.
9. Another mount reaches more sites. Collision-checked mount search, then the level sweep for the top two.
10. The passing bowl cells hold at n = 10 and with neighbours in the rack. `--interleave` to n = 10; one loaded-rack cell.
11. Mouth-down plates and bowls are ungraspable (the user's premise). G3 counter gate, n = 10, foot-ring clamp.
12. The 2F-85 grips wet ceramic hard enough (the literature's 39 N is another gripper). Report the logged pad forces.

### Fix before Tuesday (notes disagree)
- The counting rule vs the headline numbers (above). (open: not changed this round; the mouth-up gate numbers are superseded by the foot-ring and side-pinch gates on the page)
- The cup grasp record (gate2_cup) ran at hover 0.15 m; the note's deviation says 0.10 m. (fixed 2026-10-31, see
  results/2026-10-23-hardware-limits-mouth-up.md)
- Round 1 and round 3 give different Kit-free level counts (15/19 and 8/12 vs 8/15 and 6/8): round 1 carried the dish at
  its unpivoted grasp pose, round 3 at the measured 21 deg pivot; round 3 rejects rear x -0.07 at G1 where Isaac placed 9/9.
  (fixed 2026-10-31, see results/2026-10-23-hardware-limits-mouth-up.md)
- The mount search: the README says arm collisions are not checked, the code and robot.md say they are. (fixed 2026-10-31: the README row and the script's docstring say they are checked, free_reach via collide.ArmWorld.free)
- `robot.md` and the execution README predate the rounds (OMPL, bowl-only grasp, none of the reach, replay and page scripts). (fixed 2026-10-31: robot.md has the study section and the seven scripts, the README their rows)
- The memory note overclaims the front glass slots at 120-150 deg (the table: 2/8 at 120 deg, 0/8 at 150 deg). (fixed 2026-10-31 in the session memory: two cup slots at 120 deg, none at 150 deg)
- The Robot Hardware Limits page is published (Version 5, 2026-10-23), private.

### Do not quote
robot.md's corallab numbers (upright3 3/3, mount 7/7 and 6/7, the ~30 mm pivot, 40 N per pad, knuckles 21-49 N); round 1's
"unexplained hover disagreement"; cups 0/24 as the cup answer (superseded by rounds 2 and 3).

## The two sentences for Tuesday
- Sampler: certified goal loads exist on all three tiers (27, of which hard 3), the build order is now measured rest poses
  plus bowl-plate edges, and the remaining failure is bowls blocking bowls in the lower rack; the committed default is the
  worse side-zone variant, and S is an exposure ranking that has never met a wash.
- Hardware: in simulation at one mount, the 2F-85 rim pinch holds bowls and cups and places level bowls in the upper slots,
  cannot pick a flat plate by either primitive, reaches none of the benchmark's own goal poses, and every bowl and cup grasp
  rests on the knuckles, which the harness's own invariant rejects.
