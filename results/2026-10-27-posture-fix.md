# Posture fix: un-wound elbow-up IK, posture-aware drop, a lower pedestal (Part B, cups at the new mount): run note, 2026-10-27
Plan: plans/2026-10-27-posture-fix.md. Records: `data/results/robot/mount/posture_v1.json` (the 25-row mount table; the episode's `--mount`), `data/results/robot/placing_v1/c1_easy_cups_Dside_m2/`, 8 variant "D-side-m2" rows in `placing_v1/easy_cups/dishes.jsonl` (now 56 lines). Media: `data/media/robot/placing_v1/{drop,landed}_easy_cups_side_m2.{mp4,png}` plus 2 `videos.json` entries. Logs: `data/logs/robot/p1_*` (`p1_chain.sh` is the box chain). No gate ran, so `partA_cup_side_base45_m2` does not exist.

## Headline
**8 of 8 cups settled mouth down (round 6: 6/8) at the searched mount (0.45, -0.70, 0.60) m, yaw 2.56 rad. It scored 23; today's mount (0.45, -0.55, 0.90) scored 22. The elbow stayed up throughout and no joint wound a turn.** Max |q| over the run: pan 60°, lift 109°, elbow 150°, wrist 1 165°, wrist 2 104°, wrist 3 188°. Round 6 (old mount, old rules, read from its trial log's joint samples): 32, 120, 140, 357, 270, 309°.

- **The ≤ 180° criterion is missed by wrist 3 alone (181-188° in every cup).** The cause is the start posture. At this mount, the home IK nearest HOME_Q (wrist 3 at 0°) collides with the held cup. The episode therefore starts on the next free branch, [-43, -86, 37, 48, 104, 180]°, with wrist 3 exactly at 180°. The ±190° band then lets it turn 1-8° further.
- **Distance to goal.** Only cup_07 is inside the old S5 tolerance (15 mm / 20 mm / 15°); round 6 had four. Cups 01, 06 and 04 rest tilted 44-46° at 28.7 mm height (axis z -0.85), still mouth down.
- **Other checks.** No earlier cup moved. Every attachment was a fixed joint with 0.0 mm drift (jaw 0.212 rad). arm_or_palm_contact fired for cup_06, cup_04 and cup_05 (round 6: cup_06).

## Mount search (Kit-free, 135 s on 6 workers, all 25 scored before the 30 min cap)
- **Ties at 23.** Four mounts scored 23 (goals 21/23 = bowls 5/7, plates 8/8, cups 8/8; picks 2/2): (0.45, -0.70, 0.60), (0.60, -0.55, 0.60), (0.45, -0.70, 0.75) and (0.45, -0.85, 0.75). The lowest pedestal, then the first visited, wins. Today's mount and every z 0.90 mount scored 22 (bowls 4/7).
- **Episode start.** The start fails at every z 0.45 mount and at 3 of the 6 z 0.60 mounts: no elbow-up home IK is free with the cup held, and (0.60, -0.85, 0.45) has no home IK at all.
- **Base-side pre-check.** 0/12 yaws passed at every mount. The gate therefore did not run, and the drop used round 6's fallback hold (the h45 mouth hold rolled 180°).
- **Plates.** 8/8 plates have a drop IK at every mount that has a home IK; in the drop test, 4 had none. What changed: the counter edge is at -0.17 (it was -0.30) and the earlier plates sit at their goals. This is an IK check only, not a path.

| cup | slot | drop height, tries, yaw, max joint move | settled, mouth (axis z) | lateral / height mm, tilt ° | max \|q\| pan, lift, elbow, w1, w2, w3 (°) | carry travel sum |
|---|---|---|---|---|---|---|
| cup_01 | glass_right_4 | 125 mm, 4, 0°, 196° | yes, down (-0.85) | 19.9 / 28.7, 44.1 | 47, 109, 149, 165, 104, 181 | 705° |
| cup_07 | glass_left_1 | 100 mm, 1, 30°, 194° | yes, down (-0.99) | 10.2 / 6.1, 9.7 | 43, 102, 123, 165, 104, 188 | 554° |
| cup_02 | glass_right_3 | 100 mm, 1, 60°, 203° | yes, down (-1.00) | 23.0 / 0.4, 14.6 | 60, 109, 150, 165, 104, 181 | 764° |
| cup_06 | glass_right_2 | 100 mm, 1, 270°, 72° | yes, down (-0.85) | 19.5 / 28.7, 46.3 | 43, 102, 106, 165, 104, 181 | 223° |
| cup_08 | glass_left_2 | 100 mm, 1, 60°, 194° | yes, down (-0.99) | 19.1 / 5.5, 7.7 | 43, 102, 118, 165, 104, 181 | 563° |
| cup_04 | glass_right_1 | 100 mm, 1, 270°, 73° | yes, down (-0.85) | 19.9 / 28.7, 46.3 | 43, 102, 111, 165, 104, 181 | 245° |
| cup_05 | glass_left_4 | 100 mm, 1, 30°, 194° | yes, down (-0.96) | 16.4 / 0.1, 3.9 | 43, 102, 104, 165, 104, 181 | 607° |
| cup_03 | glass_left_3 | 100 mm, 1, 30°, 194° | yes, down (-1.00) | 26.3 / 1.7, 19.0 | 43, 102, 111, 165, 104, 181 | 528° |

Round 6 had no reach for cup_08 and cup_05 and needed 225 mm on its 6th pose for cup_01. The per-joint travel is in each row's `posture` dict. In the six long carries, wrist 1 travels 186-204° and wrist 2 194-262°. Several maxima are the home posture itself: the reset teleports the arm to home, and max |q| counts teleports.

## Commands (box, in `dishsim-isaac`; `$P` = /workspace/dishsim/data/results/robot/placing_v1)
1. Mount search: `run_py.sh code/execution/frigidaire/frigidaire_robot_reach.py --mount-search --workers 6`
2. Gate: skipped, because the pre-check passed 0 yaws (the chain runs it only for ≥ 1).
3. Drops: `run_kit.sh code/execution/frigidaire/frigidaire_robot_episode.py --headless --no-media --place-goal $P/easy_cups/load.json --variant D --hold side-base --side-base-run partA_cup_side_base45_m2 --row-variant D-side-m2 --mount data/results/robot/mount/posture_v1.json --shortcut-s 10 --max-wall-seconds 9000 --out-root $P --run-id c1_easy_cups_Dside_m2`
4. Replay: `run_py.sh code/execution/frigidaire/frigidaire_robot_replay.py --load-dir data/results/robot/placing_v1/easy_cups --out data/media/robot/placing_v1 --variant D-side-m2`
5. Page (Mac): `/opt/anaconda3/bin/python3 code/execution/frigidaire/frigidaire_robot_report_page.py --records data/results/robot/limits_v1 data/results/robot/exhaustive_w1 data/results/robot/exhaustive_w2 --media data/media/robot/limits_v1 data/media/robot/exhaustive_w1 data/media/robot/exhaustive_w2 --out data/media/robot/report_v1` gave `[RESULT] PASS 9 sections 63 media files 135042386 bytes`. Not published.

## Deviations
- **Eligibility rule (not in the plan).** A mount can be picked only if the episode can start there: an elbow-up home IK free with the cup held. With T_home and z_safe kept, the episode raises at a mount without a home IK.
- **Search details the plan left open.**
  - Yaw: facing the work centre, the mean x, y of the 23 goals and the 2 picks: (-0.004, -0.404).
  - Each goal's IK is seeded with the episode's start posture for that kind; the earlier dishes sit at their goals.
  - The wall-pinch pick lets the pads straddle the cup over the whole approach (0.12 m), as the episode's pick does.
- **Shortcutting.** The try cap is now 30 per second (300 at 10 s); with the old 60-try cap, the 10 s would never bind.
- **New flags.** `--side-base-run` and `--row-variant` keep the m2 gate and rows apart. "≥ 5/10 held" is implemented as ≥ 5 held trials.
- **Page.**
  - A reference row with round 6's joint range, read from its trial log.
  - Part A skips `*_m2` gate folders.
  - Every earlier section carries an "Old mount" line, and the setup paragraph names both mounts.
  - The section title reads "at the lower mount".
- **Replay re-encoded on the Mac.** 28.9 MB to 9.7 MB at CRF 30, all 2207 frames kept.

## Box window and files
- **Box (dishsim-l4).**
  - Start attempt 1 (18:02:44Z) sat STOPPED; attempt 2 was RUNNING at 18:06:41Z.
  - Code rsynced: all 141 `code/*.py` md5-identical to the Mac.
  - Chain: start 18:07:26Z, search done 18:09:42Z, drops done 18:33:08Z, replay and ALL DONE 18:34:50Z.
  - Copy-back finished 18:35:09Z; stop sent 18:35:13Z.
  - The Mac slept (lid closed) from 18:40:47Z to 18:55:58Z. STOPPED was first seen at 18:56:17Z: `dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`.
  - **About 37 billed minutes**: 28.5 running plus the usual ~8 minutes of STOPPING.
- **Changed files.**
  - `robot/rig.py`: `POSTURE_LIMITS`, `unwind`, `within_limits` at ±190°, `ELBOW_UP_ONLY = True`.
  - `robot/collide.py`: `_bounds`, `shortcut_s`.
  - `frigidaire_robot_episode.py`: `goal_qs` (smallest move first), the drop yaw chosen by joint move, the posture log and row `posture`/`mount`, `--shortcut-s`, `--side-base-run`, `--row-variant`.
  - `frigidaire_robot_reach.py`: `--mount-search`, `World(mount)`, `counter_part(family=)`.
  - `frigidaire_robot_replay.py`: `--variant D-side-m2`, the pedestal read from the rows.
  - `frigidaire_robot_report_page.py`: `posture_block`, the old-mount labels.
- **Suggested commit:** `Add posture fix: un-wound elbow-up IK, lower mount`
