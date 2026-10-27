# Placing the goal (Part B) and the cup wall pinch (Part A cup), window 3: run note, 2026-10-26
Plan: plans/2026-10-26-placing-goal.md (Part B, plus the coordinator's 16:15 change: the cup's grasp is now a wall pinch, with a new cup gate, and the approved variant R-ideal). Records: `data/results/robot/placing_v1/<load>/{load.json,dishes.jsonl,summary.json,subset_settle.json}`, run folders `placing_v1/w3_*` and `placing_v1/partA_cup_wall/` (gate2 format). Media: `data/media/robot/placing_v1/` (4 files and `videos.json`). Logs: `data/logs/robot/w3_*.log`.

## Headline
- **No dish reaches its goal in any variant: 0/23 placed under W, R and R-ideal.**
- **Subset settle: all three loads stand alone** (each load teleported at its goals, 150 ticks): bowls max 0.74 mm / 0.39°, cups 0.0 / 0.0, plates 0.02 mm / 0.0°.
- **Cup wall pinch gate (Part A): 10/10 held.** Jaw 0.159–0.163 rad, pivot 0.09–0.47° (median 0.14), 5 s drift ≤ 0.018 mm. The pads carry the cup (peaks 70–76 N). Both inner knuckles touch the rim at the close (≤ 1 s, ≤ 87 N), so D4 fired in every trial, as the bowl's knuckle rest does. One pass clip; there was no failure to clip.
- **The fixed joint held exactly.** Every one of the 71 attachments shows 0.0 mm drift. The pin fallback was never used.

| load (kind, grasp) | W | R | R-ideal |
|---|---|---|---|
| easy_bowls (7 bowls, rim pinch, pivot 20.8°) | 0/7: S1 ×7 | 0/7: R1 ×7 | 0/7: R2 ×7 |
| easy_cups (8 cups, wall pinch) | 0/8: S1 ×7, S2 ×1 | 0/8: R1 ×8 | 0/8: S1 ×1, S2 ×7 (R2 passed 8/8) |
| easy_plates (8 plates, idealized rim hold) | 0/8: S1 ×5, S2 ×3 | 0/8: R1 ×8 | 0/8: R2 ×8 |

Failing-step histogram over all 69 final rows:

| step | rows |
|---|---|
| R1 | 23 |
| S1 | 20 |
| R2 | 15 |
| S2 | 11 |
| S3–S6 | none |

No dish reached S4. So there are no settle errors to list and no disturbed neighbours. Every dish's `error` is null and `disturbed` is [].

## Per load, dishes in the witness order (W: S1 = no collision-free IK with the dish at its goal orientation 0.10–0.35 m above the goal)
**easy_bowls.** The goals are all mouth down: lower rack X105–X150 / Y150, upper rack tilt 135°.

| # | dish | rack/slot | W | R | R-ideal |
|---|---|---|---|---|---|
| 1 | bowl_06 | lower_rear | S1: 0 IK | R1 | R2: jaws 0.00 rad |
| 2 | bowl_02 | upper_bowl_09 | S1: 6 IK, arm vs pedestal/upper rack | R1 | R2: jaws 0.00 |
| 3 | bowl_07 | lower_rearright | S1: 0 IK | R1 | R2: jaws 0.00 |
| 4 | bowl_04 | lower_frontright | S1: 0 IK | R1 | R2: jaws 0.00 |
| 5 | bowl_03 | upper_bowl_04 | S1: 2 IK, forearm vs upper rack/bowl_02 | R1 | R2: jaws 0.00 |
| 6 | bowl_05 | lower_rear | S1: 0 IK | R1 | R2: jaws 0.00 |
| 7 | bowl_01 | lower_rear | S1: 0 IK | R1 | R2: jaws 0.00 |

**easy_cups.** Upper-rack glass channels; the goals are mouth down and leaning.

| # | dish | slot | W | R | R-ideal (base re-pick passed, jaw 0.237 rad) |
|---|---|---|---|---|---|
| 1 | cup_01 | glass_right_4 | S1: 4 IK, arm vs counter/cabinet | R1 | S2: pre-place + release, forearm vs cabinet/counter |
| 2 | cup_07 | glass_left_1 | S2: 0 free at pre-place/release (upper rack) | R1 | S2: release, forearm/wrist_1/gripper vs upper rack |
| 3 | cup_02 | glass_right_3 | S1: 4 IK, arm vs counter/cabinet | R1 | S2: forearm vs cabinet/counter |
| 4 | cup_06 | glass_right_2 | S1: 4 IK, arm vs counter/cabinet | R1 | S2: release, forearm vs cabinet/upper rack |
| 5 | cup_08 | glass_left_2 | S1: 0 IK | R1 | S2: forearm vs cabinet/counter |
| 6 | cup_04 | glass_right_1 | S1: 0 IK | R1 | S2: forearm vs counter/cabinet |
| 7 | cup_05 | glass_left_4 | S1: 5 IK, arm vs upper rack/cup_02 | R1 | S1: 0 IK |
| 8 | cup_03 | glass_left_3 | S1: 4 IK, arm vs upper rack/cup_06 | R1 | S2: forearm vs cabinet/counter |

**easy_plates.** Upright in the lower rack's rear and front banks.

| # | dish | slot | W | R | R-ideal |
|---|---|---|---|---|---|
| 1 | plate_08 | lower_rear_06 | S1: 1 IK, arm vs cabinet/counter | R1 | R2: no foot-ring approach |
| 2 | plate_05 | lower_rear_07 | S1: 1 IK, arm vs cabinet/counter | R1 | R2: same |
| 3 | plate_01 | lower_rear_08 | S1: 6 IK, arm vs counter/pedestal/plate_08 | R1 | R2: same |
| 4 | plate_04 | lower_rear_04 | S1: 0 IK | R1 | R2: same |
| 5 | plate_07 | lower_front_04 | S2: 0 IK at pre-place and release (S1 at +0.35 m) | R1 | R2: same |
| 6 | plate_03 | lower_front_03 | S2: 0 IK (S1 at +0.30 m) | R1 | R2: same |
| 7 | plate_06 | lower_front_02 | S2: 0 IK (S1 at +0.35 m) | R1 | R2: same |
| 8 | plate_02 | lower_front_00 | S1: 0 IK | R1 | R2: same |

**What the R and R-ideal failures were:**
- **R1** (all 23): no free release IK in any cell of the set-down search (tilts 0–60° × 12 yaws).
- **R-ideal bowls, R2:** the straight-down foot clamp was refused at all 12 yaws (no free hover IK; forearm too close). A candidate leaned 15° (yaw 5.24 rad) passed the planner and ran. The jaws stuck at 0.00 rad: the inner finger lands on the bowl's base inside the ring, robot.md landmine 5.
- **R-ideal plates, R2:** all 84 candidates (12 yaws × 7 tilts) were refused:
  - wrist_1 within 15 mm of the plate: 46;
  - no free hover IK: 18;
  - wrist_1 against the counter: 11;
  - forearm against the plate: 7.

## R1 is impossible by construction
- With a rim pinch, the wall pinch or the plate's rim hold, the dish's axis is fixed along the TCP approach. So the dish is mouth down only when the TCP points up.
- The gripper then sits under the dish's rim. Setting the dish on the counter would put the gripper inside the slab, and for the bowl the inner pad would have to be inside the bowl's dome.
- At the slab's front edge, the pinched rim would need about 60 mm of overhang to clear the inner finger. That overhang tips the dish off.
- The planner agrees: 0 of 48 cells (tilts 0–60° × 12 yaws) has a collision-free release IK, for every one of the 23 dishes.

## Cup wall pinch (robot/grasp.py `wall_grasps`, family "wall"; "base" when the cup is mouth down)
- **Geometry:** TCP on the cup axis pointing down. The closing axis is horizontal, with 12 yaws. The TCP is 24 mm under the cup's top, so the pad centres are about 11 mm under the rim and the pads cover the top 30 mm of the wall (Ø about 67–71 mm). The jaw window is 0.08–0.45 rad, and the gripper may touch the cup along the whole 0.10 m approach.
- **Why not 35 mm:** the brief's 35 mm pad depth puts the TCP 48 mm under the rim, which jammed the rim between the inner knuckles (32 N each, jaws 0 rad).
  - Measured on the logged open-gripper hulls, the palm reaches 40 mm behind the TCP and the knuckles converge to 7 mm from the centreline.
  - At a 24 mm depth the pads touch first (9.2 mm gap vs 11.9 mm for the knuckles). At 28 mm the knuckles would touch first.
- **Held pose compared with the rim pinch** (median holds):

  | | wall pinch | rim pinch (gate2_cup) |
  |---|---|---|
  | cup origin, offset from the approach line | 0.2 mm (on the line) | 26.7 mm (rim radius side) |
  | cup origin, depth below the TCP | 83.4 mm | 81.4 mm |
  | cup axis vs the approach | 0.1° | 3.6° |
  | jaw | 0.16 rad | 0.78 rad |

  So the cup now hangs centred and straight under the hand.
- **Base pinch** (R-ideal re-pick of the inverted cup): 8/8 closed at 0.237 rad, with the cup centred 24 mm behind the TCP.

## Deviations
- **Cup depth and approach allowance.** The two settings above replace the brief's 35 mm and the 60 mm allowance. The two refused tries are kept as `placing_v1/_bad_partA_cup_wall_{margin,depth48}`.
- **Insertion line.** It is vertical: the pre-place is 0.10 m straight above the release, as the sampler's insertion move.
- **S6 frame.** S6 is judged in each rack's frame. One rack is pushed in while the robot works: the upper rack for a lower-rack dish, the lower rack for an upper-rack dish.
- **Start pose.** The start is the home TCP on its first IK branch that is free with the dish held. All 71 starts were free.
- **R2 search.** R2 leans the foot or base grasp by ±15/25/35° when straight down is blocked.
  - The first R-ideal bowl run tried straight down only. It is kept as `_bad_w3_easy_bowls_Rideal_straightonly`.
  - Its 2 rows in `dishes.jsonl` are superseded by later rows: the summary takes the last row per (dish, variant).
- **Plate hold.** The plate's idealized rim hold closed the jaws only to 0.11 rad, because the inner finger meets the plate surface inside the rim. The joint holds the plate regardless.
- **Media.**
  - Nothing was placed, so there are no pass clips and no release stills.
  - Fail clips exist only where Isaac moved: bowl_06 R-ideal (the jaw jam) and cup_01 R-ideal (the base re-pick passes, then S2 stops).
  - plate_08 R-ideal shows only the teleported set-down.
  - The gate pass clip is `partA_cup_wall_pass_t00.mp4`.
- **Nothing was cut.**

## Box window and files
- **Box.** dishsim-l4: five `brev start` attempts sat STOPPED (20:12–20:24Z). The sixth reached RUNNING at 20:28:26Z (16:28 EDT).
  - The first Kit launch (20:30Z) died on a relative `--out-root`. It was relaunched at 20:32:26Z.
  - All Kit runs ended at 21:29:36Z. The Mac stalled from 21:15 to 21:53Z, so the box idled about 25 min.
  - Media ran 21:54–21:55Z, and the copy-back finished at 21:55:28Z. The clone was restored to 5b3f2d7 and is clean; its sources matched the Mac by md5.
  - Stop was sent at 21:55:45Z (17:55 EDT); `brev ls` showed STOPPED at 22:02:59Z (18:02 EDT).
- **Changed files.**
  - `code/execution/frigidaire/frigidaire_robot_episode.py`: `--place-goal`, `--regrasp`, `--regrasp-ideal`, `--settle-only`, `--dishes`, `--attach`, `--inhand-cup`, `--grasp-family wall`, and `place_goal_trials`.
  - `code/frigidaire/src/dishsim_frigidaire/robot/grasp.py`: `wall_grasps` and the plate's `KIND_FOOT`.
  - `code/execution/frigidaire/frigidaire_robot_reach.py`: `measured_M(kind, run)`.
  - `code/execution/frigidaire/frigidaire_robot_replay.py`: also draws the earlier dishes.
  - New: `code/execution/frigidaire/frigidaire_placing_loads.py` (the load builder and `--summarize`).
- **Suggested commit:** `Add placing-goal runs: cup wall pinch, S1-S6 steps`
