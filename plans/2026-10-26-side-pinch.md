# Cup side pinch: horizontal wall grasp (Part A) and the side hold at the goal (Part B, variant S)
Slug: 2026-10-26-side-pinch | From: plans/2026-10-26-placing-goal.md, results/2026-10-26-placing-goal.md

## Goal
The report says whether the cup can be pinched across its wall with the TCP HORIZONTAL (gripper beside the cup, not above it) and, with that hold welded in hand, at which step each of the 8 easy_cups goals fails (or passes, with errors).

## Decisions (from grilling, 2026-10-26 21:30-21:45 EDT)
- Side pinch = TCP approach horizontal, closing axis horizontal, both pads outside the Ø71 mm wall across the diameter, pad centres `below_top` under the rim. New family `side` in robot/grasp.py beside `wall_grasps` (which stays top-down). Rejected changing `wall_grasps`: Part A's 10/10 row must stay quotable.
- Pad height and approach yaw are swept Kit-free (heights 11 / 45 / 80 mm under the rim x 12 horizontal yaws), not chosen: the wrist vs the slab at the counter and the gripper vs the 95 mm (45 mm in rows 2-3) tines at the goal pull in opposite directions.
- Part A: n = 10 grasp gate at the same counter spot, cup mouth-up, lift 0.15 m + 5 s hold, drift <= 5 mm, real contacts, best collision-free candidate; records the in-hand pose. Lift and hold only (user); no roll in the gate.
- Part B: variant S ONLY (user): the cup fixed-jointed at the measured side in-hand pose, S1 = collision-free IK with the cup at its goal orientation 0.10-0.35 m above the goal (the 180 deg roll is implicit in the hold), S2-S6 as in the parent, earlier cups present at their goals, witness order, same tolerance. A candidate (yaw, height) counts only if it has collision-free IK at the counter pick AND at the goal; Isaac runs the best candidate per cup; no candidate = S1 or S2 with the reason. Rejected R-real (side pinch, roll, set down, base re-pick): R-ideal already showed the base pinch dies at S2 for 7/8 cups.
- Run to completion, no wall-clock cap (user). Box on at start, off right after the rsync. Bowl and plate get no side grasp this round (148 mm bowl; the plate edge pinch is untested, later).
- Same report page, Version 7: Part A gets a 'cup side pinch' row, the Part B cup table an S column, one pass and one fail clip.

- Outcome (round 4, box 21:58-22:33 EDT, 36 billed min; results/2026-10-26-side-pinch.md): Part A side pinch 10/10 at 11 mm and 10/10 at 45 mm under the rim (pads only, no knuckle contact, cup centred within 0.3 mm), 0/10 at 80 mm (gripper into the slab). Part B variant S 0/8, no Isaac motion: 24 of 72 candidates pick at the counter, 0 of 72 pass S1+S2 at any goal even with the pick waived; cause = wrist_3 rolls about TCP x, so wrist_2 hangs 0.10 m off the TCP and the roll that keeps it above the cup at the counter puts it below the TCP at a mouth-down goal (S1 x4, S2 x4: upper rack, pedestal, earlier cups). Report Version 8 (two publish batches, 22:37 EDT); Version 10 at 23:36 EDT after the user removed R-ideal from the page; Version 11 at 00:05 EDT on 2026-10-27 after the user removed variant R as well (only W and S remain); Version 12 at 00:20 EDT with Part A as the user's simple grasp table (object, grasp, held).

## Change
- code/frigidaire/src/dishsim_frigidaire/robot/grasp.py: `side_grasps(T_obj, n_yaw=12, below_top=...)`, kind "side".
- code/execution/frigidaire/frigidaire_robot_episode.py: `--grasp-family side` (grasp gate, in-hand pose record), `--place-goal ... --variant S` with the side in-hand pose (candidate sweep over yaw x height, pick + goal IK/FCL gates).
- Kit-free sweep output: data/results/robot/placing_v1/side/candidates.json (per cup: candidates tried, which pass pick, goal, both).
- Records: placing_v1/partA_cup_side/ (gate2 format), placing_v1/easy_cups/dishes.jsonl rows with variant "S"; media placing_v1/ + videos.json; frigidaire_robot_report_page.py: the row, the column, the clips.

## Run
Box dishsim-l4: Kit-free sweep; `frigidaire_robot_episode.py --headless --no-media --grasp-gate --grasp-family side --n 10 --max-wall-seconds 9000`; `--place-goal <easy_cups load> --variant S --max-wall-seconds 9000`; replays; rsync to the Mac; `brev stop`. Mac: page rebuild (/opt/anaconda3/bin/python3), publish (two batches).

## Works when
Part A: k/10 side pinches with jaw angle, drift and the in-hand pose. Part B: 8 S rows, each with its first failing step and reason or a pass with lateral / height / tilt errors; candidates.json; results/2026-10-26-side-pinch.md; report Version 7 shows the row, the column and the clips.

## Assumed / skipped
- Assumed: the FCL G1 check with the robot meshes is the collision authority (hits_counter only guards the slab volume); the fixed-joint attach from the parent works unchanged for a horizontal hold.
- Skipped: R-real, the roll gate, bowl/plate side grasps, the cup rim re-test, medium/hard loads, the sampler.
