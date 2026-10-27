# Checkpoint for Monday 11:00-12:00 EDT: hardware exhaustive, after window 1
From: plans/2026-10-26-hardware-exhaustive.md; results/2026-10-26-hardware-exhaustive-w1.md. Box dishsim-l4: STOPPED since 06:26 EDT. Window 1 ran 04:33 to 06:18, nothing cut. Draft report page: https://claude.ai/artifact/Fv7YPrB7mHdBStZSBEYs5H (private).

## Done in window 1
- **Knuckle check: the contact is real.** After the 21 deg pivot the bowl rests on the left inner knuckle at 49 N; that knuckle's mesh is 0.17 mm from the bowl, 0.7 mm below the rim. The bowl and cup grasps are rim-on-knuckle rests, not pad-only pinches, and the invariant D4 flags them correctly.
- **Tines test: the rack tilts a level bowl at all four right-side sites with no robot present** (median 32 to 36 deg front-right, 17.6 deg rear-right, 19 of 20 past 15 deg). Those 0/10 placements are the rack's geometry, not the arm.
- **Seed-free IK: no site-level change** (bowl G2 15/19, cup 8/12). The reach table stands; the HOME_Q seed was not the limit.
- **Placement at level, n = 10 at every site with a path.** Bowls: 10/10 at upper 01/02/03/07 and lower rear x -0.07/-0.12; 3/10 at x -0.17 and x -0.02; 0/10 at the four right-side sites (the rack); 0/10 never released at x -0.20, upper 00 and 04 (the planner refuses the yaw-0 pose). Cups 15/80: glass_left_2 6/10, left_1 4/10, left_3 and right_0 2/10, left_0 1/10, right_1 to 3 0/10; 11 trials never released (no IK at the hover).
- **Counter grid: bowls and cups pick from every spot.** Kit-free 25/25 spots each; Isaac 3x3 bowl 53/53, cup 42/45 (three misses). Plate 0/25 Kit-free (the 15 mm clearance).

## What can be concluded now
1. **Graspability, mouth-up, this gripper, in sim:** bowls and cups are graspable from anywhere on the counter; the plate from nowhere (0/25; 3/10 only with the clearance check off; slide 0/25). Pick position is not a limit for bowls and cups.
2. **The bowl grasp is a knuckle rest.** It never dropped in 63 holds, but it is not the pad pinch the harness was written for. Either accept the knuckle rest as the grasp (retire D4 for bowls and cups) or redesign the grasp. Your call.
3. **Bowl placement by position splits cleanly:** 6 sites at 10/10, 2 marginal at 3/10, 4 impossible at level because the rack tilts the bowl, 3 blocked by the planner at yaw 0.
4. **Cup placement is the weak result** (best slot 6/10). The failure is the release: the cup is let go 35 to 62 mm off the level pose and drops, and a quarter of the trials never reach a hover IK. Not a reach limit: 8 of 12 slots have a path.

## Per test: continue, fix, stop, or not going to work
- **Continue, window 2 as planned: the tilt range per site** for bowls and cups (adaptive 15/30/45/60, n = 5). The one data point (upper 02: 30 deg 3/5, 60 deg 0/5) puts the limit between 30 and 60; 15-degree steps pin it. About 2 h. This is the missing half of "which rotation works".
- **Continue, window 2: plates in hand.** The only way to say anything about plate placement; Kit-free minutes plus about 15 Isaac trials.
- **Fix, small, add to window 2: the three bowl sites never released at yaw 0.** Retry with the other G2 yaws from the Kit-free sweep before calling them unreachable. About 20 min.
- **Fix, larger, defer past Tuesday: the cup release.** Continue the descent past the contact-lag block, or set the cup on the channel floor and judge against the target. Failure mode identified; not a Tuesday item.
- **Not going to work as is: picking a flat plate off the counter with this gripper** (0/25, slide 0/25). Needs a different primitive or a propped plate. And **level placement at the four right-side bowl sites**: the rack tilts the bowl there whatever the robot does; a tilted release is the only thing left to try, and window 2's sweep covers it (rear-right at 30 deg was 3/5 in round 3).
- **Stop:** nothing; the box is off.

## Decision for you: window 2, about 2.5 h of box time this afternoon
Recommended: go as planned, plus the yaw retry at the three blocked bowl sites. Window 2 output: results/2026-10-26-hardware-exhaustive-w2.md, the final report page by about 22:00.
