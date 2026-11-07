# Cup side pinch, mouth-down drop: the base-side hold, the counter edge pulled back to y = -0.17 (Part B, variant D-side)
Slug: 2026-10-27-cup-side-drop | From: plans/2026-10-27-drop-test.md, plans/2026-10-26-side-pinch.md

## Goal
For each of the 8 easy_cups cups, the cup welded in hand by a base-side pinch (wrist toward the cup's base) measured at the pulled-back counter, the arm carries it flipped mouth-down to the lowest collision-free pose above its goal and lets go; the page says how many settle mouth-down and where each landed, with the goal and landed stills and one replay.

## Decisions (from grilling, 2026-10-27 10:05-10:40 EDT)
- Setting: the counter slab's front edge moves from y = -0.30 to y = -0.17 for every robot run from now on (the extended racks, already at their slide limits -0.49 / -0.44, end in front of it); the benchmark's COUNTER stays; the earlier tables on the page state -0.30. Rejected -0.21 (the cabinet front) and removing the slab over the appliance. Fact: the old slab covered only bowl_05 and bowl_07 of the 14 no-reach goals; every cup goal lies in front of it.
- Hold: the base-side pinch (grasp.py side_grasps, wrist_side "base"), pads 45 mm under the rim, picked near the new edge so the wrist (10 cm under the TCP at the counter) hangs in front of the slab; the cup flips in the carry, so the wrist rides above the mouth-down cup at the drop. Rejected the measured mouth-side hold flipped (wrist 10 cm below the gripper at the drop: variant S's cause) and running both holds.
- Part A gate: n = 10 at the new counter, restricted to wrist_side base at 45 mm, lift 0.15 m + 5 s hold, drift <= 5 mm, as the side-pinch gates; its median hold feeds Part B. 0/10: the measured mouth hold (partA_cup_side_h45) rolled 180 deg about the approach, and the page says so.
- Counter spot: x = 0.20; y swept Kit-free from the edge + 0.05 to + 0.12 m in 1 cm steps; the first y where the base-side pick (hover IK, approach line, lift) passes for the most yaws is the gate's and the drop's spot (session decision).
- Drop pose: the hold flipped (cup axis straight down, TCP horizontal), turned about the vertical in 12 yaws, the cup centroid over the goal centroid's x, y, 25-350 mm above it in 25 mm steps, the lowest collision-free IK; carried by transit_to; the earlier cups where they landed, the lower rack pushed in; witness order; "no reach" after DROP_TRIES = 6 poses without a path (user, 10:45 EDT; the parent's 3 raised: its 7 no-path dishes were RRT timeouts at 15 s). Otherwise as the parent.
- Success = settled at rest (bench_settle) AND mouth down (axis z < 0); settled-any the second count; lateral / height / tilt from the goal, landed on, earlier cups moved: listed.
- Pre-flight: the Kit-free check also reports per cup whether a collision-free drop IK exists at any height and yaw; Isaac runs regardless (user, 10:45 EDT: a measured 0/8 beats a prediction; the gate alone pays for the window).
- Page: Part B opens with the new section "Cups: side pinch, mouth-down drop" (goal still, landed still, one replay, per-cup table, the edge at -0.17); the first drop test stays below, labelled -0.30; Part A gets the base-side row. Version 15.
- Run now (user, 10:40 EDT): one box window of about 45 min, box off right after the copy-back. Bowls and plates: discussed after this run (user).
- Outcome (box 11:35-12:07 EDT, about 35 billed min; results/2026-10-27-cup-side-drop.md): base-side gate 0/10 (no collision-free grasp at any yaw: forearm and wrists hit the counter or cabinet; the spot sweep found 0/12 yaws at every y from -0.12 to -0.05, spot y = -0.12 by the rule), so Part B ran the fallback (h45 mouth hold rolled 180 deg). Drops: settled mouth-down 6/8, settled-any 6/8, no reach 2 (cup_05, cup_08: RRT timeouts, nothing blocked); 4 of the 6 within 15 mm / 20 mm / 15 deg of the goal, all on the upper rack, no earlier cup moved. Report Version 15 (one publish: index.html + 2 new media files, 61 in all).

## Change
- frigidaire_robot_episode.py: `--counter-front-y` (default -0.17) rewriting B.COUNTER (size, centre) at startup for the Kit slab, the FCL slab and the counter band, grasp.COUNTER_BOX following; `--side-wrist-side base` for the gate; `--variant D --hold side-base` (attach at the gate's median hold, the drop pose from the hold flipped); rows variant "D-side"; DROP_TRIES 6.
- frigidaire_robot_reach.py: the same counter override; `--side-spot`: the counter-spot sweep and the per-cup drop-IK pre-flight (information).
- frigidaire_robot_replay.py / frigidaire_robot_report_page.py: the new section, the Part A row.
- Records: placing_v1/partA_cup_side_base45/, easy_cups/dishes.jsonl (+D-side rows), media placing_v1/{drop,landed}_easy_cups_side.{mp4,png}, videos.json.

## Run
Box: Kit-free spot sweep; gate `--grasp-gate --grasp-family side --side-below-top .045 --side-wrist-side base --n 10`; `--place-goal easy_cups/load.json --variant D --hold side-base --max-wall-seconds 9000`; replay + landed still; copy back; stop. Mac: page rebuild, publish (two batches).

## Works when
10 gate rows (held k/10, the in-hand pose), the spot's y, 8 D-side rows (settled, mouth, drop height, landing, errors), the replay and the landed still, results/2026-10-27-cup-side-drop.md, Report Version 15.

## Assumed / skipped
- Assumed: the gripper is symmetric about the approach (the base side = the mouth side rolled 180 deg); the fixed-joint attach holds a flipped carry; the first drop test's goal still is reused; the Part A side-pinch rows stay quotable (nothing re-runs at the new edge).
- Skipped: bowls, plates, n > 1, the no-path check of the first drop test, a re-run of the first cup drop at the new edge.
