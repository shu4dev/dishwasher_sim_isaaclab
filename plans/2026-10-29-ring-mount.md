# Mount search and reach map for the foot-ring hold (Kit-free)
Slug: 2026-10-29-ring-mount | From: plans/2026-10-29-ring-place.md, results/2026-10-29-ring-place.md, data/results/robot/kitfree_v1/mount_seats/

## Goal
Kit-free on the Mac: which pedestal mount lets the arm place the most bowl goals with the ring hold while keeping the cups, the front plates and the counter picks, and, at the current mount and the best one, the nearest reachable pose to each bowl goal (offsets and leans), so the next round can place every bowl as close to its sampled pose as the arm allows.

## Decisions (from grilling, 2026-10-29 19:30-19:40 EDT)
- Why (user): 4 of 7 bowl goals are out of reach at the posture-fix mount (02 the tub, 05 the upper rack, 01 the basket and bowl_07, 06 the lower rack), the rack states are already exploited, and the 825-candidate mount search of 2026-10-28 was scored for the rim pinch; the ring hold needs room above the bowl, so the search is re-scored for it.
- Scoring (user): over the old 825 candidates (kitfree_v1/mount_seats table), keep as constraints the 8 cup drops, the 4 front plate seats and the counter picks (the mouth-down bowl ring pick at the gate's spot included), then maximise the number of bowl goals reachable with the ring hold (the F-out in-hand pose, the hand above the bowl, 15 mm arm / 2 mm held margins, a free 50 mm descent); ties to the lowest pedestal. Rejected bowls-only scoring.
- Reach map (user): around each goal, offsets +-20 mm in 5 mm steps in the rack plane, lean +-10 deg in 5 deg steps, 12 spins; a pose counts with reach, clearance, hand above and the free descent; per bowl the nearest feasible pose by centroid distance then axis angle, and whether it lies in the goal's slot; at the current mount and the best one. Rejected offsets only.
- Goals: goals_v4 now (placing_v1/easy_bowls/goals_v4.json; bowl_05's flagged); re-run on easy_bowls_v4 when the sampler delivers it (plans/2026-10-29-sampler-v4.md).

## Change
- kitfree_v1/ring_mount/: a script on the mount_seats machinery (ms.py's re-mounted World, the arm model, the F-out hold from placing_v1/partA_bowl_foot_out/inhand.json) with the ring-hold bowl term and the constraints above; outputs table.json (per candidate: constraints, bowl goals reached, pedestal), best.json, and the reach map reach_map.json / .md (per bowl: nearest feasible pose, deviation, in-slot, at both mounts); README lines.

## Run
Mac, `.venv-kitfree`: the search (minutes to an hour; parallelise over candidates with a process pool), then the reach map at the two mounts. No box, no page.

## Works when
best.json names a mount with its constraint check and bowl count, table.json covers the 825 candidates, reach_map.md gives the nearest feasible pose per bowl at both mounts with its deviation, results/2026-10-29-ring-mount.md summarises both and says whether a mount change is worth a round.

## Assumed / skipped
- Assumed: the rack states as the placement rounds (the loaded rack out, the other in); the earlier bowls at their goals in the witness order; the counter edge -0.17; the 12 spins 0..330; the deviation metric = centroid mm then axis deg.
- Skipped: Isaac, the page, cups and plates re-scoring beyond the constraints.

## Outcome (2026-10-29 20:05 EDT, Mac, about 8 min of compute; results/2026-10-29-ring-mount.md)
- Mount search: 825 candidates, 173 pass every constraint. Best by the rule (0.45, -0.70, 0.45) yaw 2.56 = the current mount 15 cm lower: 4/7 by the hand rule (06 at spin 330 only), but the cup episode start has no free elbow-up home IK there; best with the cup start free (0.25, -0.80, 0.60) yaw 2.30, also 4/7; the current mount 3/7 (rank 66). Base yaw never changes a bowl result. Counting the bowl's own 50 mm descent path, every mount reaches 3/7 (07, 04, 03): bowl_06's v4 goal has two tine tips 6.9 and 3.6 mm through its wall and its descent keeps the bowl at least 9.5 mm inside the rack (the goal needs a re-settle, like bowl_05's).
- Reach map (4,860 poses per bowl at three mounts): 07, 04, 03 at 0 mm / 0 deg in slot; 06 at 0 mm / 5 deg by the hand rule, none with its own path clear; 02 none (gripper vs cabinet or upper rack at every pose), 01 none (gripper vs lower rack, bowls 05 and 07), 05 none (no IK or forearm vs upper rack; its 6 clear poses have no free arm IK at any of the 825 mounts). Verdict: a mount change is not worth a box round; the three unreachable goals are unreachable within +-20 mm / +-10 deg at any mount.
- Found: checker.pair's max_penetration_m stops at the first contact over 1 mm, so earlier penetration depths were lower bounds (2.4 vs 6.9 mm). Deviations: plate seats recomputed with the mirror hold; a face-up plate counter pick added; the cup pick reused is the 45 mm one.
