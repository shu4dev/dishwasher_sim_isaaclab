# Checkpoint 2, Monday 2026-10-26 about 15:00 EDT: hardware exhaustive, after window 2
From: plans/2026-10-26-hardware-exhaustive.md; results/2026-10-26-hardware-exhaustive-w2.md; checkpoint 1 in results/2026-10-26-hardware-exhaustive-checkpoint.md. Box dishsim-l4: STOPPED since 14:46 EDT. Window 2 ran 12:21 to 14:36, nothing cut: 195 Isaac tilt trials, 26 yaw-retry trials, two Kit-free sweeps. Report page: https://claude.ai/artifact/Fv7YPrB7mHdBStZSBEYs5H, Version 4 (final after window 2: 49 media files, two publish batches, private).

## Done in window 2
- **Tilt range per site, bowls (13 sites) and cups (8 slots), n = 5 per tilt, limit = the first 0/5.** The largest tilt either dish places is 30 deg. Bowls: upper 02 and 03 pass 15 and 30 deg 5/5 and fail 45 (limit 45); upper 01 passes 15, fails 30; upper 00, 04, 07 and the three front-right sites fail at 15 already; lower rear x -0.02 and rear-right pass 30 deg 2/5; rear x -0.07 and -0.12 have no Kit-free path below 45 and 60 deg, and their 0/5 there are planner refusals (never released). Cups: glass_right_2 15 deg 4/5, 30 deg 1/5; left_1, left_3, right_1 pass 15 deg 1 to 2 of 5; left_0, left_2, right_0 fail at 15. Right cups tip over at 30 to 45 deg; left cups settle 13 to 14 deg, out of tolerance.
- **Yaw retry: the three sites the planner refused at the default yaw all place at reach yaw 8.** Lower rear x -0.20 10/10, upper_bowl_00 10/10, upper_bowl_04 6/10 (4 refused at the hover). Reliable level bowl sites go from 6 to 8.
- **Plates in hand: no gap is reachable.** With the plate held at round 1's median pinch pose (pivoted 38 deg), 0 of 19 gaps pass G2 Kit-free: 101 of 228 cells have no IK at the release, 127 no collision-free IK at the pre-place. No Isaac plate trial ran.
- **Opposite tilt direction:** no Kit-free path at -30 deg at any of the 5 sites that passed +30. Kit-free only.

## What can be concluded now
1. **Bowls: 8 level sites at 10/10** (upper 00, 01, 02, 03, 07; lower rear x -0.07, -0.12, -0.20), one at 6/10 (upper 04), two marginal at 3/10, four rack-limited at 0/10. Level or 15 deg is the working regime; 30 deg only at upper 02 and 03 reliably; nothing places at 45 deg or more. A 7-bowl level goal on the 8 reliable sites is plausible from single-dish numbers; neighbours in the rack are untested.
2. **Cups stay the weak result.** Tilting does not help (best 1/5 to 2/5 at 30 deg). The cause is unchanged: the rim pinch cannot set the base down, so the cup drops after release. The side-wall grasp is the untested fix.
3. **Plates: nothing works end to end with this gripper as set up.** No pick from the counter (0/25 Kit-free, slide 0/25, pinch 3/10 only with the clearance off), and even with the plate already in hand at the measured pinch pose no gap is reachable. The pivoted in-hand pose plus the 15 mm clearance is the blocker, not only the pick: a plate needs a less-pivoted grasp pose or a different primitive before any more plate trials are worth running.
4. **Several lower-rear tilt limits are the planner's, not the bowl's:** every 0/5 at rear x -0.02, rear-right 45 deg and x -0.12 60 deg stopped with an IK branch jump while lowering, never released.
5. **Grasp yaw matters more than the reach table showed.** One other yaw turned three unreachable sites into reachable ones; the Kit-free 12-yaw sweep had predicted a path at yaw 8. Default-yaw numbers understate reach.

## Per test: continue, fix, stop, or not going to work
- **Not going to work, as is:** picking a flat plate (0/25 twice) and placing a plate from the pivoted in-hand pose (0/19). Also level or tilted bowl placement at the four right-side sites (the rack tilts the bowl, 0/10 level and 0/5 at 15 deg).
- **Fix, one window of about 2 h, tonight if wanted:** (a) the cup side-wall grasp (pads on the outer wall, set the base down, release), the one plausible cup fix; (b) a deeper bowl rim pinch, which may remove the 21 deg knuckle pivot and, for plates, give a less-pivoted in-hand pose that opens gaps; (c) a render of a physical tip-over clip (upper 02 at 45 deg, glass_right_2 at 30 deg) for the report, since both current 45 deg fail clips show the planner refusing, not a dish falling.
- **Continue later, not for Tuesday:** a 7-bowl full set placed level into the 8 reliable sites (the direct test of placing an easy goal, neighbours present); the mouth-down foot-ring clamp; a mount search.
- **Stop:** the single-dish exhaustive sweep for bowls and cups is complete; nothing more to run in that frame.

## Decision for you
Window 3 tonight (about 2 h: cup wall grasp, deeper bowl pinch, tip-over clips), or stop here and finalize the report from windows 1 and 2.
