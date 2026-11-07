# Bowls and plates down (Part B, posture-fix mount): run note, 2026-10-27
Plan: plans/2026-10-27-bowls-plates-down.md.
- **Records.** `data/results/robot/placing_v1/{c1_easy_bowls_Dflip,c1_easy_plates_I}/`, 7 "D-flip" rows in `easy_bowls/dishes.jsonl` and 8 "I" rows in `easy_plates/dishes.jsonl` (now 44 lines each).
- **Media.** `data/media/robot/placing_v1/{drop_easy_bowls_flip.mp4, landed_easy_bowls_flip.png, insert_easy_plates.mp4, landed_easy_plates_insert.png}` and 4 `videos.json` entries. The goal stills are reused from the drop test.
- **Logs.** `data/logs/robot/r8_*`: `r8_chain.sh` is the box chain and `r8_preview.py` the Kit-free preview.

## Headline
**Bowls: 1 of 7 settled mouth down; all 7 settled. Plates (assumed hold): 2 of 8 settled, both in their slots; 5 had no reach.**

- **Flip tilt: 75° for every bowl.** No bowl was flagged palm.
  - In the Kit-free fall check, every tilt from 0° to 60° touched the gripper within 20-35 mm of fall.
  - At 75°, the turn of +108° about the closing axis cleared by 1.8 mm, also at the drop posture with wrists 1 and 2 checked. The other direction (-107°) cleared by 0.6 mm.
- **Wrist 3 starts at 180.0° for every bowl and plate.**
  - With a bowl or plate in hand, 2 of the 4 elbow-up home IK branches are free, and both have wrist 3 at 180°. The 2 branches with wrist 3 at 0° collide with the held dish.
  - Both free branches tie on max |q| (180°), so the tie goes to nearest HOME_Q, measured as the largest single-joint difference. That picks [-174, -130, 37, 93, -27, 180]° (182.6° from HOME_Q, on wrist 1) over round 7's start [-43, -86, 37, 48, 104, 180]° (194.0°, on wrist 2).
  - Effect: every carry swings the pan joint 164-229° (round 7: 32-60°), and the carry travel sums are 730-986° (round 7: 223-764°).
- **Joint range.**
  - Max |q| over the bowls: pan 174°, lift 130°, elbow 134°, wrist 1 174°, wrist 2 104°, wrist 3 184°. Wrist 3 is past 180° in every row (181-184°).
  - The elbow stayed up throughout, except plate_02: 26 ticks while it was pressed against the earlier plates.
- **Bowls.** The 75° carry holds the bowl nearly on its side.
  - Six bowls landed mouth up (axis z 0.29-0.97).
  - bowl_04 counts as down, but at axis z -0.01 it lies on its side: 60 mm lateral, 50.5° off its goal.
  - Every bowl touched the robot as the jaws opened or the hand retreated (arm_or_palm_contact: palm, forearm, wrists 1 and 2). The fall check only moved the bowl straight down against the gripper and wrists.
  - bowl_02's carry brushed the tub (3.8 N). bowl_05 moved bowl_06 by 10.5 mm / 12.5°.
- **Plates: no reach.** The 4 rear-bank plates and plate_06 (lower_front_02) have no collision-free IK at 25-350 mm on either face; the Kit-free preview agrees. The rear bank lies under the pushed-in upper rack.
- **Plates: the three that reached.** Each stopped on contact with the rack after 45-55 mm of lowering. None was blocked, and none reached the goal height. Each ended with the gripper on the mouth face after 2 tries and 2 face switches. The only reading of those counts and the IK heights is that the base face, the lowest with an IK at +100 mm, was tried first and found no path.
  - plate_07 settled 0.1 mm and 0.0° from its goal.
  - plate_03 settled in its slot, but 40 mm, 34.5 mm and 16.9° off its goal.
  - plate_02 did not settle (26 mm drift); it ended out of its slot, resting on plate_03.

| bowl | slot | drop height, tries, yaw, max joint move | settled, mouth (axis z) | lateral / height mm, tilt ° | landed on | max \|q\| pan, lift, elbow, w1, w2, w3 (°) | carry travel |
|---|---|---|---|---|---|---|---|
| bowl_06 | lower_rear | 275 mm, 2, 0°, 236° | yes, up (0.91) | 48.5 / 57.1, 141.6 | LowerRack | 174, 130, 87, 165, 104, 183 | 840° |
| bowl_02 | upper_bowl_09 | 225 mm, 1, 0°, 238° | yes, up (0.97) | 5.7 / 51.9, 150.5 | UpperRack | 174, 130, 107, 165, 104, 183 | 781° |
| bowl_07 | lower_rearright | 250 mm, 1, 0°, 259° | yes, up (0.83) | 20.8 / 54.0, 132.6 | LowerRack | 174, 130, 94, 167, 104, 183 | 986° |
| bowl_04 | lower_frontright | 275 mm, 1, 120°, 162° | yes, down (-0.01) | 60.4 / 2.1, 50.5 | LowerRack | 174, 130, 134, 165, 104, 184 | 790° |
| bowl_03 | upper_bowl_04 | 250 mm, 1, 0°, 236° | yes, up (0.29) | 92.1 / 63.9, 122.0 | UpperRack | 174, 130, 116, 165, 104, 183 | 808° |
| bowl_05 | lower_rear | 300 mm, 3, 90°, 162° | yes, up (0.61) | 35.8 / 69.5, 117.7 | bowl_06 | 174, 130, 98, 174, 104, 183 | 730° |
| bowl_01 | lower_rear | 300 mm, 3, 120°, 161° | yes, up (0.93) | 40.2 / 1.9, 89.4 | LowerRack | 174, 130, 95, 165, 104, 184 | 781° |

| plate | slot | gripper face (switches), start height | lowered, stop | settled | lean from goal (final vs goal lean) | lateral / height mm | landed on | in slot | attach |
|---|---|---|---|---|---|---|---|---|---|
| plate_08, 05, 01, 04 | lower_rear_06/07/08/04 | no reach: no collision-free IK at 25-350 mm on either face | – | – | – | – | – | – | fixed joint |
| plate_07 | lower_front_04 | mouth (2), 125 mm | 45 mm (TCP 44), contact (LowerRack) | yes | 0.0° (-32.2 vs -32.2) | 0.1 / 0.0 | LowerRack | yes | fixed joint |
| plate_03 | lower_front_03 | mouth (2), 125 mm | 45 mm (TCP 43), contact (LowerRack) | yes | 16.9° (-42.8 vs -32.2) | 40.0 / 34.5 | LowerRack, pad_L, plate_07 | yes | pin (fallback) |
| plate_06 | lower_front_02 | no reach: no collision-free IK at 25-350 mm on either face | – | – | – | – | – | – | pin (fallback) |
| plate_02 | lower_front_00 | mouth (2), 125 mm | 55 mm (TCP 42), contact (LowerRack) | no (drift 26.4 mm) | 14.3° (-36.9 vs -34.0) | 63.0 / 29.1 | LowerRack, plate_03 | no | pin (fallback) |

## Commands (box, in `dishsim-isaac`; `$P` = /workspace/dishsim/data/results/robot/placing_v1; one chain, `r8_chain.sh`)
1. **Preview** (Kit-free, cores 2/6): `run_py.sh data/logs/robot/r8_preview.py` gave `[RESULT] PASS flipcheck preview in 11 s`.
2. **Plates** (cores 1/5, run beside steps 1 and 3):
   `run_kit.sh code/execution/frigidaire/frigidaire_robot_episode.py --headless --no-media --place-goal $P/easy_plates/load.json --variant I --mount data/results/robot/mount/posture_v1.json --shortcut-s 10 --legacy-tolerances --max-wall-seconds 9000 --out-root $P --run-id c1_easy_plates_I`
   Result: `[RESULT] PASS place-goal easy_plates I: settled 2/8, failed at {'no reach': 5, 'not settled': 1}`.
3. **Bowls** (cores 0/4): the same command with `--place-goal $P/easy_bowls/load.json --variant D --hold rim-flip --row-variant D-flip --run-id c1_easy_bowls_Dflip`.
   Result: `[RESULT] PASS place-goal easy_bowls D-flip: settled mouth down 1/7, settled any 7, failed at {'mouth up': 6}`.
4. **Replays:** `run_py.sh code/execution/frigidaire/frigidaire_robot_replay.py --load-dir data/results/robot/placing_v1/easy_bowls --out data/media/robot/placing_v1 --variant D-flip`, then the same with `easy_plates` and `--variant I`.
5. **Page** (Mac): the round-7 command, `/opt/anaconda3/bin/python3 code/execution/frigidaire/frigidaire_robot_report_page.py ... --out data/media/robot/report_v1`.
   Result: `[RESULT] PASS 11 sections 67 media files 151664486 bytes`. Not published.

## Deviations
- **Tolerance flag.** `--legacy-tolerances` (the 0.08 rad carried-dish settle tolerance) was on both runs; the plan's command lacked it. A welded bowl or plate lags wrist 3, and that lag must not decide reach.
- **Tilt scan.**
  - The measured hold pivoted about TCP x, so the bowl axis cannot point straight down: tilts 0° and 15° both give the full flip, whose axis sits 24.3° from straight down.
  - Tilts of 30° and more come in two turn directions, and both were checked.
  - The fall check is an FCL distance with no margin. The gripper and wrist 3 are checked without a posture; wrists 1 and 2 are checked at the drop search's first pose.
- **Plate hold.**
  - The KIND_RIM pinch sits at the rim point highest at the goal: approach along the plate's -axis, closing toward the top edge. "base" is the TCP frame turned 180° about its x axis.
  - After the release, the hand backs out 0.10 m along its approach instead of rising, because rising would drag the lower finger through the rim.
- **In-slot test** (my rule). Every plate point within 5 mm of the slot's two tine rows and below the tine tops must lie between the column pair's tine axes at its own height, ± the tine radius. All 8 goal poses pass this test (checked on the Mac).
- **Side-switch artifact.**
  - A face switch reruns the episode's reset, which opens the jaws with the plate detached at home. The plate falls for about 0.75 s before it is teleported back into the hand.
  - plate_03's fall touched plate_07 (5 N); no earlier plate moved more than 10 mm / 20°.
  - The base-face weld sits 8-12 mm inside the right finger. On plate_03 the fixed joint drifted 3.7 mm, so plates 03, 06 and 02 were pinned (pad force 438-560 N).
  - Fix (not made): park the dish before reset() opens the jaws.
- **Unchecked arm contacts.** The 0.10 m back-off along the approach drove wrist 2 into the silverware basket (2, 331 and 633 N). plate_02's lowering pressed the wrist and palm onto plates 07 and 03; the stop rule watches only the plate's own contacts.
- **Replay re-encoding.** Both mp4s were re-encoded on the Mac at CRF 30: 29.2 to 9.4 MB and 20.1 to 6.6 MB.

## Box window and files
- **Box (dishsim-l4).**
  - Start attempts 1-3 sat STOPPED. Attempt 4 went STARTING at 21:02:47Z and RUNNING at 21:04:33Z, after a STOPPED blip.
  - Code rsynced: all 141 `code/*.py` files md5-identical to the Mac.
  - Chain: start 21:05:42Z, preview passed 21:05:54Z, plates done 21:22Z, bowls done 21:30:26Z, ALL DONE 21:32:32Z.
  - Copy-back finished 21:33:25Z; stop sent 21:33:29Z.
  - `brev ls` showed STOPPED at 21:42:00Z: `dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`.
  - **About 39 billed minutes**, from STARTING to STOPPED.
- **Changed files.**
  - `robot/collide.py`: `ArmWorld.fall_clearance`.
  - `frigidaire_robot_episode.py`: `--variant I`, `--hold rim-flip`, `flip_orients`, `plate_holds`, `choose_flip`, `first_drop`, `insert`, `in_slot`, `release(back=)`, the home rule in `reset()` (`q_start_deg` in rows), and the row and RESULT labels.
  - `frigidaire_robot_replay.py`: `--variant D-flip` / `I`.
  - `frigidaire_robot_report_page.py`: the two new Part B sections (`flip_block`, `insert_block`, `contact_events`) and the setup paragraph.
- **Suggested commit:** `Add bowl partial-flip drop and plate top-edge insert`
