# Bowls from the nearest home branch, plates with a checked retreat (Part B round 10, posture-fix mount): run note, 2026-10-28
Plan: plans/2026-10-27-carry-rules.md.
- **Records.** `data/results/robot/placing_v1/{c1_easy_bowls_Dlip2,c1_easy_plates_I3}/`, 7 "D-lip2" rows in `easy_bowls/dishes.jsonl` (now 58 lines) and 8 "I3" rows in `easy_plates/dishes.jsonl` (now 60 lines).
- **Media.** `data/media/robot/placing_v1/{drop_easy_bowls_lip2.mp4, landed_easy_bowls_lip2.png, insert_easy_plates_3.mp4, landed_easy_plates_insert_3.png}` and 4 new `videos.json` entries. The goal stills are reused from the drop test.
- **Page.** Version 19, built into `data/media/robot/report_v1` and not published. The 4 Version 18 copies it no longer uses moved to `data/media/robot/report_v1_v18_dropped/`; its MANIFEST.txt lists the paths to null on publish.
- **Logs.** `data/logs/robot/r10_*`: `r10_chain.sh` is the box chain, `r10_easy_{bowls,plates}.log` the Kit runs, `r10_replay_*.log` the replays.

## Headline vs Works-when
**Bowls: the largest joint travel went down on 6 of 6 reached bowls, by 122-175° each (≥ 30° on 6/6). The Kit-free prediction held within 10° on 4 of 6. Plates: plate_07 stayed in its slot after the retreat; 3 of 8 plates are in their slots (round 9: 1).**
- **Bowls, joint travel.** Largest single-joint travel per carry, round 9 → 10: 252-320° → 125-152°. Wrist travel sum (w1 + w2 + w3): 478-584° → 283-338°. Travel sum over all joints: 604-805° → 481-554°.
  - Wrist 1 fell from 252-320° to 111-152°, and wrist 2 from 153-196° to 4-79°.
  - Wrist 3 rose from 63-111° to 117-139°, because every bowl now starts with wrist 3 at 180°. It peaks at 184° in each carry (posture `past_180`: wrist_3). The elbow stayed up throughout.
- **Bowls, prediction vs measurement.** Predicted / measured: bowl_06 144.7 / 145.6, bowl_04 152.2 / 152.3, bowl_01 129.4 / 129.8, bowl_03 117.0 / 125.3, bowl_02 110.7 / 128.2 (+17.5), bowl_05 116.4 / 130.7 (+14.3).
  - The executed leg types matched the Kit-free prediction on 6/6.
  - Leg 1 (start → the drop pose at 1.25 m) was straight for all six. Leg 2 needed RRT-Connect on bowls 06, 02, 05 and 01: no combo at their lowest free height was straight. The two misses are RRT legs.
  - Every bowl started at [-42.8, -85.8, 37.4, 48.4, 104.0, 180.0]. That branch was the cheapest of the 4 free home branches for all six. Round 9's start was [-42.8, -101.9, 86.8, -164.9, -104.0, 0.0].
  - The first ranked combo reached on every bowl (1 try).
- **Bowls, drops (recorded, not the metric).** Mouth down 0/7, settled 5/7, all mouth up (round 9: 0/7, 3/7). bowl_02 did not settle (90 mm drift). bowl_07 had no reach: no free combo at any of 14 heights × 19 spins × 4 home branches (forearm vs tub on 279/279 IK solutions).
  - The wider spin scan lowered three drops: bowl_06 175 → 150 mm, bowl_05 200 → 150 mm, bowl_01 125 → 50 mm.
- **Plates.** 4/8 settled (the rear bank had no reach, as in round 9). In slot: plate_07, plate_06 and plate_02; plate_03 was not.
  - plate_07: in slot, 9.3 mm lateral, 48.1 mm high, lean -18.6° vs -32.2°. It rests against the open left pad.
- **Plate retreat.** No plate backed off at all: the checked line stopped every one at its first waypoint (0 mm).
  - plate_07: wrist 2 within 15 mm of the basket at the release pose itself.
  - plates 03, 06, 02: the gripper within 15 mm of the earlier plates in the neighbouring slots.
  - The rim test let only plate_03 rise (13.3 mm clear). It settled leaning on plate_07, out of slot. Plates 07, 06 and 02 recorded "skipped: fingers at the rim" (0.0 / 0.0 / 6.7 mm), and the hand stayed where it let go.
- **Plate attachment.** At a plate_03 re-weld (the row has 2 face switches) the fixed joint drifted 5.8 mm in its 12-tick check. Plates 03, 06 and 02 then ran on the existing pin fallback, as in round 8.
  - Those three log `carried_contact` and `pad_force` invariants (pad_L 438 N); plate_02 also logs `arm_or_palm_contact`.
  - plate_02 moved plate_03 by 58.6 mm and plate_06 by 59.6 mm.

| bowl | slot | start w2 / w3 (r9 → r10) | spin | height mm | largest travel ° r9 → r10 (predicted) | travel sum ° | pan swing ° | legs r10 (predicted) | settled, mouth (axis z) |
|---|---|---|---|---|---|---|---|---|---|
| bowl_06 | lower_rear | -104 / 0 → 104 / 180 | -30 → -45 | 175 → 150 | 268 → **146** (145) | 754 → 519 | 73 → 52 | straight / rrt (same) | yes, up (0.98) |
| bowl_02 | upper_bowl_09 | -104 / 0 → 104 / 180 | 0 → -5 | 100 → 100 | 273 → **128** (111) | 655 → 501 | 48 → 46 | straight / rrt (same) | no, up (0.97) |
| bowl_07 | lower_rearright | no reach (no IK): forearm vs tub 279/279, forearm vs upper rack 189/279 | – | – | – | – | – | – | – |
| bowl_04 | lower_frontright | -104 / 0 → 104 / 180 | 0 → +10 | 125 → 125 | 320 → **152** (152) | 803 → 554 | 74 → 76 | straight / straight (same) | yes, up (1.00) |
| bowl_03 | upper_bowl_04 | -104 / 0 → 104 / 180 | 0 → -10 | 125 → 125 | 252 → **125** (117) | 604 → 481 | 44 → 40 | straight / straight (same) | yes, up (0.94) |
| bowl_05 | lower_rear | -104 / 0 → 104 / 180 | 0 → +5 | 200 → 150 | 278 → **131** (116) | 761 → 487 | 67 → 61 | straight / rrt (same) | yes, up (0.63) |
| bowl_01 | lower_rear | -104 / 0 → 104 / 180 | 0 → +30 | 125 → 50 | 305 → **130** (129) | 805 → 504 | 66 → 55 | straight / rrt (same) | yes, up (0.97) |

| plate | slot | face (switches), start | lowered, stop | back (free) | back stopped by | rim clear | rise | settled | lean vs goal | lateral / height mm | in slot |
|---|---|---|---|---|---|---|---|---|---|---|---|
| plate_08, 05, 01, 04 | lower_rear_06/07/08/04 | no reach on both faces (forearm vs tub, held plate vs upper rack), as round 9 | – | – | – | – | – | – | – | – | – |
| plate_07 | lower_front_04 | mouth (2), +125 mm | 45 mm, contact LowerRack | 0 mm (0) | at 0 mm: wrist 2 vs basket | 0.0 mm | skipped: fingers at the rim | yes | -18.6 vs -32.2 | 9.3 / 48.1 | **yes** |
| plate_03 | lower_front_03 | mouth (2), +125 mm | 45 mm, contact LowerRack | 0 mm (0) | at 0 mm: gripper vs plate_07 | 13.3 mm | yes | yes | -17.5 vs -32.2 | 11.8 / 64.9 | no |
| plate_06 | lower_front_02 | mouth (2), +200 mm | 100 mm, contact plate_03 | 0 mm (0) | at 0 mm: gripper vs plate_03, plate_07 | 0.0 mm | skipped: fingers at the rim | yes | -17.7 vs -32.2 | 20.0 / 63.3 | yes |
| plate_02 | lower_front_00 | mouth (2), +200 mm | 125 mm, contact LowerRack | 0 mm (0) | at 0 mm: gripper vs plate_03, plate_06, plate_07 | 6.7 mm | skipped: fingers at the rim | yes | -29.2 vs -34.0 | 23.9 / 88.1 | yes |

## Kit-free checks (Mac, before the box)
- **Search smoke** (`.venv-kitfree`, the World arm model, earlier bowls at round 9's landed poses): bowl_06 returned home [-42.8, -85.8, 37.4, 48.4, 104.0, 180.0], +150 mm, spin -45°, cost 144.7° (legs straight / rrt). All 7 bowls took 108 s.
  - For bowls 06, 02, 04 and 03 the box chose the same combo. For bowls 05 and 01 it chose others (150 mm +5°, 50 mm +30°): the earlier bowls lay where this run left them.
  - On the box the searches took 9-170 s per bowl (bowl_02: 48 combos, all path-checked, none straight).
- **Plate check** (round 9's release poses):
  - The open gripper sits 3.6 mm from the plate at the hold.
  - plate_07's wrist 2 is already within 15 mm of the basket at the release pose.
  - Backing off the mouth face clears the plate by 15 mm after 25 mm; wrists 1 and 2 meet the basket after 35-65 mm.
- **Dry run** (fake Kit layer, real IK and FCL, round 9's harness): every new path ran in both loads. That covers the search, ranked drop tries with a forced first failure, executed and predicted legs, the I3 back-off with truncation, the rim test and the skipped rise. Bowls took 131 s, plates 5 s.

## Commands (box, in `dishsim-isaac`; `$P` = /workspace/dishsim/data/results/robot/placing_v1; one chain, `r10_chain.sh`)
1. **Bowls** (cores 0/4), in parallel with step 2:
   `run_kit.sh code/execution/frigidaire/frigidaire_robot_episode.py --headless --no-media --mount data/results/robot/mount/posture_v1.json --shortcut-s 10 --legacy-tolerances --max-wall-seconds 9000 --out-root $P --place-goal $P/easy_bowls/load.json --variant D --hold rim-lip --carry nearest --row-variant D-lip2 --run-id c1_easy_bowls_Dlip2`
   Result: `[RESULT] PASS place-goal easy_bowls D-lip2: settled mouth down 0/7, settled any 5, failed at {'no reach': 1, 'not settled': 1, 'mouth up': 5}`. The run took 1522 s.
2. **Plates** (cores 1/5): the same options with `--place-goal $P/easy_plates/load.json --variant I --row-variant I3 --run-id c1_easy_plates_I3`.
   Result: `[RESULT] PASS place-goal easy_plates I3: settled 4/8, failed at {'no reach': 4}, attach {'pin': True, ...}`. The run took 1435 s. No tracebacks in any log.
3. **Replays:** `run_py.sh code/execution/frigidaire/frigidaire_robot_replay.py --load-dir data/results/robot/placing_v1/easy_bowls --out data/media/robot/placing_v1 --variant D-lip2` (1536 frames). Then the same with `easy_plates` and `--variant I3` (1555 frames).
   - On the Mac both mp4s were re-encoded at CRF 30: 30.3 to 9.9 MB and 30.2 to 9.8 MB, with the same frame counts. The raw files are in the scratchpad.
4. **Page** (Mac): `/opt/anaconda3/bin/python3 code/execution/frigidaire/frigidaire_robot_report_page.py --records data/results/robot/limits_v1 data/results/robot/exhaustive_w1 data/results/robot/exhaustive_w2 --media data/media/robot/limits_v1 data/media/robot/exhaustive_w1 data/media/robot/exhaustive_w2 --out data/media/robot/report_v1` gave `[RESULT] PASS 7 sections 16 media files 54126130 bytes` (index.html 22.2 KB).

## Deviations
- **Drop tries.** A failed try falls back only to the chosen home branch's ranked combos at the lowest free height. The arm already stands on that branch after the teleport; another branch would need a second teleport. No bowl needed a second try.
- **Back-off check.** The gripper alone may touch the released plate in the first 60 mm (line_free's allowance for the dish being released, measured from the dish). Without it every line would start in collision: the open gripper is 3.6 mm from the plate at the hold. Nothing is in `ignore`.
  - In the run the allowance never mattered: every line stopped at 0 mm on a pair outside it (wrist 2 vs basket, or the gripper vs an earlier plate).
- **Rim test.** It reads the `gripper` entry of `ArmWorld.fall_clearance(T_backed, plate, drops=(0,))`, so wrist 3 is not counted. The plate pose is its Isaac pose after the back-off.
- **Rack order (carry mode).** reset() sets the dish's rack state before the home-branch IK and the search, so the search sees this dish's racks.
- **Leg types.** `plan_to` now logs each plan's type (`plan_log`). Rows carry the executed legs (`legs`) and the Kit-free ones (`legs_pred`).
- **Measured cost.** `carry_cost_deg` = max of `posture.carry_travel_deg`, the carry from the drop start to the release, every try.
- **No Kit-free preview gate on the box.** The Mac smoke and dry run took its place.
- **Cap.** The Mac end driver's cap was set at 05:14Z, 51 min after STARTING, so that STOPPED would land inside 60 min. It was not reached.

## Box window and files
- **dishsim-l4.** Start attempt 1 at 04:21:30Z; STARTING 04:22:54Z; RUNNING 04:23:44Z. All 141 `code/*.py` md5-identical to the Mac; the box data files matched by md5.
  - Chain: start 04:24:44Z, plates 04:49:20Z (replay 04:50:28Z), bowls 04:50:46Z (replay 04:51:55Z), ALL DONE 04:51:55Z.
  - Copy-back 04:52:25-04:52:44Z; stop sent 04:52:47Z; STOPPED 05:01:53Z.
  - **About 39 billed minutes**, STARTING to STOPPED (cap 60).
  - `brev ls` at 05:01:55Z: `dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`.
- **Changed files.**
  - `frigidaire_robot_episode.py`:
    - New flag `--carry nearest` and the D-lip2 label.
    - New constants and helpers: `CARRY_SPINS_DEG`, `two_legs`, `carry_search`, `combo_rec`; `plan_log` in `plan_to`.
    - reset(): the search and the rack order. drop(): the ranked combos and the new row fields (`start_branch`, `carry_cost_pred` / `carry_cost_deg`, `carry_sum_*`, `carry_travel_pred`, `legs`, `legs_pred`, `combo`, `combo_rank`, `reset.carry_search`).
    - release(back=True), I3: `RELEASE_BACK_MAX_M`, `BACK_STEP_M`, `RIM_CLEAR_M`. The `retreat` fields are `back_mm`, `back_free_mm`, `back_followed`, `back_blocked_by`, `rim_clear_mm` and `rise`.
  - `frigidaire_robot_replay.py`: `--variant D-lip2` / `I3` and their captions.
  - `frigidaire_robot_report_page.py`: Version 19 (`ROUNDS` + `TABLE_ONLY`, `lip2_table`, `insert3_table`, `travel_down`, `starts_of`, the setup text).
  - New: `data/logs/robot/r10_chain.sh`.
- **Suggested commit:** `Add bowl carry search and checked plate retreat`
