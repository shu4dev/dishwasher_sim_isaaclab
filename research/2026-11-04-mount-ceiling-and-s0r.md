# Three s0 goals the frozen mount cannot reach: what to do with them
Slug: 2026-11-04-mount-ceiling-and-s0r | Lit review: none (a decision note from this repo's measurements)

## Question
Sample s0 has three goal sites (bowl_04 at upper_bowl_11, plate_01 at lower_rear_05, plate_02 at lower_front_07) that the UR5e with the side-mounted 2F-85 on posture_v1 cannot reach by any carry or release found so far; should the robot demo change the hardware, change the goal, or report the ceiling?

## What's known (max 5)
- Site sweep (data/results/robot/samples_v1/<s0>__fixes/sites/kitfree.md, 2026-11-04): over 24 pinch spins, both elbow branches, pitch to 90 deg, tilt from -30 to 30 deg and 8 slide directions, bowl_04 is blocked by the gripper and wrists 2 and 3 against the Cabinet at every descent start, plate_01 by wrists 1 and 2 against the Cabinet and the gripper against the SilverwareBasket, plate_02 by wrists 1 to 3 against the Pedestal. No perched alternative counts (a tilted dish that seats more than 20 mm up the descent axis is a perch).
- The same sweep found the three other rejects of the first round a way in: bowl_02, bowl_03 and cup_05 by the build's own release, plate_04 by a spun pinch; the three-fixes window then placed cup_05 at goal and showed bowl_03's 80 mm fall lands differently from the build's (results/2026-11-04-three-fixes.md).
- The hardware study is closed and the mount is frozen (code/frigidaire/docs/robot.md; the side mount is the baseline since 2026-10-31, the mount fix was reverted at the user's request on 2026-10-30).
- The benchmark's own algorithms move dishes by teleportation (CLAUDE.md, What this is); the robot layer is a feasibility study on top of it, so a goal the robot cannot reach is still a valid benchmark goal.
- The build packs the roster by CP-SAT over the packer's poses (plans/2026-11-02-fit-score-prove.md); a pose mask is a natural input to that stage.

## Gap
Nothing has been checked about whether s0's roster packs at level 0 when the three unreachable sites, and every other site the Kit-free reach gate rejects for that kind, are masked out of the packer's poses, nor what S_ref such a load would score.

## Candidate ideas (max 3)
1. A reach-masked goal variant, s0r, built as a separate sample (never overwriting s0): the same roster and seed through bench.py from the build stage with the packer's poses masked by the reach gate per kind, then the robot run on it. Might work because the lower front bank and the upper bowl row have more sites than s0 uses. Might fail because the mask may leave no level-0 packing for 5 plates, or S_ref may drop well below s0's 0.513. Cheapest test: the Kit-free fit alone with the mask (no Isaac), which says in minutes whether a packing exists and what the ascent scores.
2. Move the arm: a pedestal position nearer the machine's centre line, or the straight mount. Might work because plate_02 is blocked by the robot's own pedestal and bowl_04 by the cabinet wall the arm wraps around. Fails the frozen-hardware rule; every grasp gate and reach result would have to be remeasured.
3. Report the ceiling: s0 stays the sample, the three sites are reject rows with their blockers, and the demo shows 12 of 15 as this hardware's number. Might fail only as a demo: the video never shows a full rack.

## Picked
3 for s0 itself, with 1 offered as the user's choice: s0 is the sample the user asked for, the three sites are provably out of reach on this mount, and a different goal is a different sample. If the user wants a full-rack demo on this hardware, 1 is the way, and its first step costs no box time.

## To grill
- Does the user want a robot-feasible goal at all, given that the benchmark's movers teleport and the robot is a feasibility layer?
- If s0r is built, is the mask the Kit-free reach gate alone (cheap, geometric) or the measured per-site Isaac result (slower, truer)?
- Is a 12 of 15 demo on s0 enough for the current purpose, or is the full rack the point?
