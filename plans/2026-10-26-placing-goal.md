# Placing the goal: grasp summary (Part A) and goal placement with the dish already in hand (Part B)
Slug: 2026-10-26-placing-goal | From: plans/2026-10-26-hardware-exhaustive.md, results/2026-10-26-hardware-exhaustive-checkpoint2.md

## Goal
The report answers, per dish of three single-kind easy loads, at which step the robot fails to put the dish at its exact goal pose when it starts with the dish in the gripper; Part A summarises the grasp results already measured.

## Decisions (from grilling, 2026-10-26 15:50-16:10 EDT)
- Split: Part A grasping = summary of gate2 bowl 10/10, cup 9/10, plate 3/10 (clearance off) and slide 0/25, the knuckle verdict; the counter grid becomes a footnote. Nothing new runs for Part A.
- Part B starts with the dish in the gripper at the MEASURED in-hand pose (bowl pivoted 21 deg, cup upright); the plate gets an idealized upright rim hold because no plate grasp exists. Rejected an idealized hold for all: it reports a robot that does not exist.
- The dish is rigidly attached to the gripper (a fixed joint) from the start through reorientation and carry, broken at release: nothing slips or drops before S4 (user: "assume that if the object is grasped it would never fall off").
- Goals: three single-kind easy loads, n = 1 per dish: the 7 bowls of rest_easy seed 5; the 8 plates of bp_medium seed 5; the 8 cups of rest_hard seed 2, each at its certified settled goal pose. Rejected sampler-generated single-kind loads (reopens the sampler, one box hour) and hand-made loads (not benchmark goals). Each subset is settled once alone in Kit to confirm it stands without the other kinds.
- Order = the dependency graph's witness order restricted to the subset; dishes placed earlier are present at their goal poses when the next is placed.
- Steps labelled per dish, first failure wins: S1 reorient in hand to the goal orientation above the rack (IK); S2 pre-place reachable (G1); S3 collision-free path and insertion (G2); S4 insert and release executed; S5 settled within the G4 tolerance (15 mm lateral, 20 mm height, 15 deg) with the actual errors listed; S6 no earlier dish disturbed beyond that tolerance. S1-S3 Kit-free, S4-S6 Isaac.
- Variant W (wrist rotation, same hold) first for every dish; variant R (flip in hand, set down mouth-down on the counter spot, re-pick from above by the foot ring, bowl and plate, or the base for the cup, re-attach, then S1-S6) for every dish W fails at S1 or S2, in the same round. Rejected labelling only: the user wants the regrasp answer tonight.
- Pass = the G4 tolerance; every placed dish's lateral, height and tilt error is listed so "exact" can be judged.
- Tonight, window 3 of about 2.5 h (box start about 16:20, stop by 19:15 EDT). Part B goes into the Hardware Limits Report as new sections above the per-object detail. Rejected a separate page.
- Cup grasp (user, 16:15 EDT, after the runners started): the WALL pinch (both pads outside the cup wall, top-down, pads about 35 mm below the rim) is the cup's main grasp in Part A and Part B; a new n = 10 grasp gate measures it and its in-hand pose; the rim-pinch cup rows stay as 'earlier' and the rim re-test is deferred until after round 3. Bowl and plate unchanged: a wall pinch cannot span the 148 mm bowl, the plate is flat.
- Outcome (window 3, 16:28-17:55 EDT, box STOPPED 18:02, nothing cut; results/2026-10-26-placing-goal.md): 0/23 placed under W, R and R-ideal. W fails at S1 for 19 dishes (no collision-free IK with the dish at its goal orientation) and S2 for 4; R fails at R1 for all 23 by construction (the dish is mouth-down only when the TCP points up, so the set-down puts the gripper in the slab); R-ideal (flip teleported, approved by the session 16:50) fails at R2 for bowls (jaws jam on the base) and plates (no foot-ring approach), and at S2 for 7 of 8 cups after a passing base re-pick. Subsets stand alone. Cup wall pinch gate 10/10, cup centred (0.2 mm, 0.1 deg) against 26.7 mm / 3.6 deg for the rim pinch; pad depth 24 mm under the rim, not the planned 35 (knuckles jam at 48). Report Version 6 (two publish batches).

## Change
- frigidaire_robot_episode.py: `--place-goal <load.json>`: spawn earlier dishes at their goal poses, attach the current dish to the gripper (fixed joint) at the in-hand pose, run S1-S6 with the first failure labelled, `--regrasp` for variant R (flip, set down, foot/base re-pick, re-attach).
- A load builder (Kit-free): from the three sampler goal records to data/results/robot/placing_v1/<load>/load.json (dish, kind, settled goal pose, witness order, graph edges within the subset).
- Subset settle check: teleport the subset at its goal poses, settle, record max drift (Kit).
- Records: placing_v1/<load>/dishes.jsonl (one row per dish per variant) and summary.json; media placing_v1/ with videos.json.
- frigidaire_robot_report_page.py: Part A summary table and Part B tables at the top (another agent).

## Run
Box: load builder; 3 subset checks; `frigidaire_robot_episode.py --headless --no-media --place-goal <load> --max-wall-seconds 9000` for easy_bowls, easy_plates, easy_cups; `--regrasp` pass for the W failures; replays and stills; rsync; stop. Mac: page rebuild, publish.

## Works when
For each of the 23 dishes: a W row with its first failing step and reason (or pass with errors), an R row where W failed at S1/S2, the subset settle verdict per load, at least one pass and one fail clip per load; results/2026-10-26-placing-goal.md; the report page shows Part A and Part B.

## Assumed / skipped
- Assumed: the certified goal records on the Mac hold settled poses and the witness order; the plate's idealized hold = the rim-pinch frame with zero pivot; the cup's regrasp hold = a pinch across the inverted cup's base.
- Skipped: medium and hard loads, n > 1, idealized holds for bowl and cup, the mount search, the sampler.
