# Cup side pinch (Part A) and variant S (Part B): run note, 2026-10-26/27
Plan: plans/2026-10-26-side-pinch.md. Records: `data/results/robot/placing_v1/{partA_cup_side_h11,partA_cup_side_h45,partA_cup_side_h80}` (gate2 format), `placing_v1/side/candidates.json` (+ `candidates_nominal.json`, the same sweep before the gates, nominal holds), 8 variant "S" rows in `placing_v1/easy_cups/dishes.jsonl` (run `w4_easy_cups_S_p0`). Media: `data/media/robot/placing_v1/partA_cup_side_h11_pass_t00.mp4` (entry in `videos.json`). Logs: `data/logs/robot/w4_*.log`.

## Headline
- **Part A: the horizontal side pinch holds 10/10 at pads 11 mm under the rim and 10/10 at 45 mm; 80 mm is 0/10 (no reach).**
  - 11 mm: jaw 0.172 rad, pivot median 0.06° (max 0.75), 5 s drift median 0.005 mm (max 0.21). 45 mm: jaw 0.212 rad, pivot ≤ 0.08°, drift ≤ 0.011 mm.
  - No invariant fired (counted 20/20): only the pads touch the cup, unlike the wall pinch's knuckle contact.
  - In-hand pose (median-pivot trial, TCP frame): cup origin (93.8, 0.0, 10.0) mm at 11 mm and (59.9, 0.0, 10.2) mm at 45 mm, within 0.3 mm of the planned grasp; cup axis 0.05-0.17° off -TCP x (one trial 1.1 mm / 0.7°). Both gates used yaw 1 (30°), wrist on the mouth side.
  - 80 mm: no collision-free grasp in all 10 trials (the gripper reaches into the counter slab; upper arm vs cabinet/counter/pedestal). The Kit-free sweep agrees.
- **Part B (variant S): 0/8 placed, no Isaac run.** No cup has a candidate that passes both the counter pick and the goal. S1 ×4, S2 ×4:

| # | cup | slot | S | reason (over the 24 candidates that pass the counter pick: 11 and 45 mm, wrist on the mouth side, 12 yaws) |
|---|---|---|---|---|
| 1 | cup_01 | glass_right_4 | S1 | 3-4 IK each at +0.10 m, all hit (UpperRack 24, Cabinet 17, Pedestal 16) |
| 2 | cup_07 | glass_left_1 | S2 | S1 passes 15 (raised 0.25-0.35 m); release / pre-place hit Pedestal 15, UpperRack 15 |
| 3 | cup_02 | glass_right_3 | S1 | 3-4 IK each at +0.10 m, all hit (UpperRack 24, Pedestal 16, cup_01 14) |
| 4 | cup_06 | glass_right_2 | S2 | S1 passes 3; S2 hits UpperRack, cup_02, Pedestal |
| 5 | cup_08 | glass_left_2 | S1 | no IK at all at +0.10 m for any of the 24 (the record reads "commonest hits []"; the page renders it as no IK solution) |
| 6 | cup_04 | glass_right_1 | S1 | no IK at all at +0.10 m for any of the 24 (the same) |
| 7 | cup_05 | glass_left_4 | S2 | S1 passes 2 (at 0.35 m); S2 hits Pedestal, UpperRack, cup_02 |
| 8 | cup_03 | glass_left_3 | S2 | S1 passes 8; S2 hits UpperRack 8, cup_08 8, Pedestal 7 |

- **Candidates:** 72 per cup (3 heights × 12 yaws × 2 wrist sides).
  - Counter pick: 24/72 pass, all at 11 and 45 mm with the wrist on the mouth side. 80 mm passes 0/24, and the base side passes 0/36.
  - Goal (S1 and S2): **0 of 72 for every cup, even with the counter pick waived.** S1 alone passes 0-53 per cup, mostly with the wrist on the base side.
- **Why it fails at the goal.** In this model wrist_3's joint axis is TCP x, so wrist_2 hangs 0.10 m from wrist_3 along -TCP x. The roll that keeps the wrist above the TCP at the counter therefore puts it 0.10 m below the TCP at a mouth-down goal. The release and pre-place (S2) then collide with the upper rack, the pedestal or the earlier cups for every candidate.

## Deviations
- **Part A heights.** Part A ran n = 10 at each of the three heights (parallel slots), not one best candidate, because Part B needs a measured hold at the height of each cup's candidate. The page shows one row per height.
- **Wrist side.** The 180° roll about the approach (`grasp.WRIST_SIDES`) is a third candidate parameter. The first launch (01:59Z) had only the base side, which puts TCP x along the cup axis. Every counter pick failed there: wrist_1/2 sat inside the slab. Those records are kept as `_bad_*_wristbase` (its Part B wrote no rows: a `GROUPS` variable name bug, also fixed).
- **Goal yaw.** At the goal the yaw is the cup turned about its own axis; the cup is symmetric and the tolerance checks tilt only. So the counter yaw and the goal yaw combine freely, but the wrist side does not (it is fixed in the cup's frame by the hold). `pass_both_same_yaw` is kept too (0 everywhere).
- **Grasp depth.** The cup axis sits 10 mm beyond the TCP. This comes from the gripper collider hulls: the pads span 20 mm behind to 18 mm beyond the TCP, and the palm face is 40 mm behind it. The wall pinch's 13 mm-behind pad centre would put the palm 7 mm inside the wall.
- **Where the sweep lives.** The candidate sweep is `frigidaire_robot_reach.py --side-candidates` (Kit-free); the episode reads `candidates.json`.
- **Clips.** One pass clip (11 mm, trial 0) and no fail clip: nothing failed physically. The 80 mm trials and all 8 S rows are planner refusals before any motion, so they leave no trajectory to replay.
- **Box git.** The box clone was clean at 5b3f2d7 and behind origin/plan, with no box-only edits, so it was fast-forwarded with `git pull --ff-only`; the coordinator was told. My edits remain uncommitted there, identical to the Mac (md5).

## Box window and files
- **Box (dishsim-l4).** `brev start` 01:56:36Z went RUNNING at 01:58:01Z on the first try, and ssh worked at 01:58:28Z.
  - The wrong-roll chain ran 01:59:44-02:02:45Z; the fixed chain ran 02:09:24-02:24:13Z.
  - The copy-back finished at 02:24:58Z and the stop was sent at 02:24:59Z (21:25 EDT).
  - `brev ls` showed STOPPED at 02:33:40Z (22:33 EDT): `dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`.
  - About 36 billed minutes (01:58-02:34Z).
- **Changed files.**
  - `robot/grasp.py`: `side_grasps`, `SIDE_BELOW_TOP_M`, `SIDE_AXIS_BEYOND_TCP_M`, `WRIST_SIDES`.
  - `frigidaire_robot_episode.py`: `--grasp-family side`, `--side-below-top`, `--variant S`, `--side-candidates`.
  - `frigidaire_robot_reach.py`: `--side-candidates`.
  - `frigidaire_placing_loads.py`: S counts.
  - `frigidaire_robot_report_page.py`: the S column, the Part A side clip and the mouth-down sentence. Variant R-ideal is left off the page (user, 22:45 EDT): its column, counts, histogram share and clips, with the records untouched. The plate's hold reads "assumed rim hold", the tines test "set straight onto the site", and a load without clips reads "clips: none recorded".
- **Page.** Rebuilt into `data/media/robot/report_v1`: `[RESULT] PASS 7 sections 50 media files 99418313 bytes`.
  - `grep -ci "r-ideal\|idealized\|teleport"` gives 0, and `grep -c "commonest hits \[\]"` gives 0.
  - The cup_04 / cup_08 S cells read "S1: no IK solution at +0.10 m for any of the 24 candidates" (`s_cell`).
  - Histogram: R1 = W 0 / R 23 / S 0; S1 = 19 / 0 / 4; S2 = 4 / 0 / 4.
  - The three R-ideal mp4 copies were deleted from `report_v1/media/placing_v1/`; the originals stay in `data/media/robot/placing_v1/`.
- **Suggested commit:** `Add cup side pinch: gate at three heights, variant S`
