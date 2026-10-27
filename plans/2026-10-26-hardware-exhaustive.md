# Hardware limits, exhaustive: graspability and reachability per object, one dish at a time
Slug: 2026-10-26-hardware-exhaustive | From: results/2026-10-23-hardware-limits-mouth-up.md (supersedes the withdrawn 2026-10-26 sprint plan)

## Goal
By Tuesday 08:00 EDT a full report answers, per object (bowl, cup, plate): where on the counter it can be picked and at what rate, which rack sites it can be placed at and through what tilt range, with success and failure videos and stills of the passing poses. The sampler waits until this is exhausted.

## Decisions (from grilling, 2026-10-26 03:00-03:20 EDT)
- Hardware first, sampler deferred. Rejected splitting the windows between them: the user wants the hardware exhausted.
- About 5 h of box time in two windows: window 1 now (about 2.5 h), window 2 Monday afternoon (about 2.5 h). Rejected one window: n = 10 plus the tilt range does not fit in 2 h.
- Cases: single dish, empty rack, every site (as measured so far, deepened). In: Isaac tilt range per site, counter pick grid, plates placed in hand. Out: mouth-down grasps, full sets of one kind, loaded racks. Rejected mouth-down: later, by the user.
- Tilt: one axis, the round-3 convention; adaptive 15/30/45/60 deg, n = 5, climbing until a tilt scores 0/5, which is the site's limit. Rejected both directions (about 4 h) and a fixed n = 3 grid (cruder per cell).
- Counter grid: Kit-free G1/G2 on a 5x5 grid of pick spots per object; the Isaac grasp gate at the 3x3 subset, n = 5, bowl and cup. Rejected Kit-free only: the user wants rates.
- n = 10 at level at every site with a Kit-free G2 path: bowls by in-air level release (the pooled mode, topped up from 6-9), cups by contact release at lag 0.15 rad (the only mode that passes). Session's call.
- Window 1 order: knuckle-contact dump (1 bowl G3 trial), robot-free tines test (level bowl teleported and settled, n = 5, at front-right x +0.06/+0.08/+0.13 and rear-right), seed-free IK level reach rerun (Kit-free, in parallel), the n = 10 top-ups, the counter grid. Window 2: the adaptive tilt sweep at every G2 site; plates in hand (Kit-free reach at the 19 gaps with the round-1 median in-hand pose, then Isaac placement n = 5 at the G2 gaps); the opposite tilt direction at passing sites if time remains.
- Grasp numbers are physical outcomes (bowl 10/10, cup 9/10) with the knuckle rest named and the invariant's verdict beside them; the knuckle check sets the final wording.
- Checkpoint between 11:00 and 12:00 EDT Monday (user, 03:45; first said as '12am', then fixed to lunchtime): a written go/no-go after window 1 in results/2026-10-26-hardware-exhaustive-checkpoint.md: what is done, what can be concluded, and per test whether to continue, stop, fix, or call it not going to work. Window 2 starts only on the user's go. A second checkpoint follows window 2 before the final report.
- Report: a new private page, one section per object (pick-rate table over counter spots, placement table and rack map by site, tilt limit per site, success and fail videos, stills of the passing pose at release), plus results/ notes. Draft after window 1 (about 07:00), final after window 2 (about 22:00). Rejected extending the limits page: one report to summarise from.
- Checkpoint 1 (user, 2026-10-26 07:20 EDT, after window 1): GO for window 2 as planned, plus a yaw retry at the three bowl sites the planner refused at yaw 0 (lower_rear x -0.20, upper_bowl_00, upper_bowl_04) with the other Kit-free G2 yaws. Grasp headline = the physical outcome (bowl 10/10, cup 9/10 held) with the knuckle rest named and the invariant's verdict beside it; the knuckle contact measured real (49 N, meshes 0.17 mm apart). Rejected counting only pad-held trials: the rest holds (0 drops in 63 holds).
- Draft report published after window 1: https://claude.ai/artifact/Fv7YPrB7mHdBStZSBEYs5H (Version 2, 39 media files, two publish batches). Checkpoint note: results/2026-10-26-hardware-exhaustive-checkpoint.md.
- Window 2 ran 12:21-14:36 EDT (the brief's 07:25 clock was wrong; the runner caught it and the cap moved to 15:45), nothing cut, core 3 allowed as a 4th Kit slot. Results: largest placing tilt 30 deg for bowls (upper 02/03 5/5) and cups (1/5); yaw 8 recovers the three refused bowl sites (10/10, 10/10, 6/10); plates in hand reach 0/19 gaps; no path at -30 deg anywhere. Checkpoint 2: results/2026-10-26-hardware-exhaustive-checkpoint2.md; window 3 (cup wall grasp, deeper bowl pinch, tip-over clips) is the user's call.
- Final report published 2026-10-26 about 15:40 EDT as Version 4 of https://claude.ai/artifact/Fv7YPrB7mHdBStZSBEYs5H (49 media files, 98.6 MB, two batches): leads per object, Isaac tilt limit = first 0/n from 15 deg, physical failures ranked first in Videos, planner refusals captioned as such. Build: `/opt/anaconda3/bin/python3 code/execution/frigidaire/frigidaire_robot_report_page.py --records data/results/robot/{limits_v1,exhaustive_w1,exhaustive_w2} --media data/media/robot/{limits_v1,exhaustive_w1,exhaustive_w2} --out data/media/robot/report_v1`.

## Change
- code/execution/frigidaire/frigidaire_robot_episode.py: `--dump-contacts` (partner collider, normal force per tick, knuckle-mesh gap at the pivot); `--tilts-adaptive` (stop at the first 0/n); `--counter-grid` (3x3 pick spots); `--plate-in-hand` (spawn the plate in the gripper, skip the grasp).
- code/execution/frigidaire/frigidaire_robot_reach.py: seed-free IK (2 pi unwrap inside the limits) behind a flag; `--counter-grid` 5x5 for G1/G2; plate-in-hand reach at the 19 gaps.
- Robot-free tines test: teleport and benchmark settle at the four right-side sites (reuse the dependency kit's teleport and settle).
- A report page script from frigidaire_robot_limits_page.py: per-object sections, rack map, tilt table, Warp-replay videos, stills; output data/media/robot/report_v1/.

## Run
Window 1 (box, now): the three checks; `frigidaire_robot_episode.py --headless --no-media --place-slot bowl --target level --interleave --n 10` and `--place-slot cup --target level --down-orient target --contact-lag 0.15 --n 10` at the G2 sites; `--grasp-gate bowl|cup --counter-grid --n 5`; stop the box. Window 2 (box, afternoon): `--place-slot bowl|cup --target tilt --tilts 15,30,45,60 --tilts-adaptive --n 5`; plates in hand; stop. Mac after each window: the results note and the report page.

## Works when
Per object: a pick-rate table over counter spots (n = 5 each), a placement table over every rack site at level (n = 10), a tilt limit per site (the first 0/5), the knuckle and tines verdicts, at least one success and one failure video per object and a still of each passing pose; results/2026-10-26-hardware-exhaustive-w1.md and -w2.md; the page private at a claude.ai link.

## Assumed / skipped
- Assumed: box dishsim-l4, 3 Kit slots on cores 0-2 (core 3 for Kit-free), about 2 min per Isaac trial per slot; the runner edits on the box and the Mac pulls through the brev remote; brev is logged in on the Mac before the runner starts (it runs `brev ls` first and stops with a note if logged out).
- Assumed: a site with no Kit-free G2 path at level gets no Isaac trials (G2 is the prerequisite) and is reported as unreachable.
- Skipped until the hardware is exhausted: mouth-down grasps, plate grasp primitives, mount search, loaded rack, full sets, everything on the sampler.
