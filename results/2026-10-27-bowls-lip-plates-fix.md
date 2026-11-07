# Bowls lip pinch, plates bugs fixed (Part B round 9, posture-fix mount): run note, 2026-10-27
Plan: plans/2026-10-27-bowls-lip-plates-fix.md.
- **Records.** `data/results/robot/placing_v1/{c1_easy_bowls_Dlip,c1_easy_plates_I2}/`, 7 "D-lip" rows in `easy_bowls/dishes.jsonl` (now 51 lines) and 8 "I2" rows in `easy_plates/dishes.jsonl` (now 52 lines).
- **Media.** `data/media/robot/placing_v1/{drop_easy_bowls_lip.mp4, landed_easy_bowls_lip.png, insert_easy_plates_2.mp4, landed_easy_plates_insert_2.png}` and 4 new `videos.json` entries. The goal stills are reused from the drop test.
- **Page.** Version 18, built into `data/media/robot/report_v1` and not published. The 55 Version 17 copies it no longer uses moved to `data/media/robot/report_v1_v17_dropped/`; its MANIFEST.txt lists the paths to null on publish.
- **Logs.** `data/logs/robot/r9_*`: `r9_chain.sh` is the box chain, `r9_preview.py` the Kit-free preview and `r9_preview.log` its output.

## Headline
**Bowls: 0 of 7 settled mouth down; 3 of 7 settled. Plates: 4 of 8 settled, 1 in its slot; 4 had no reach.**
- **Bowls.** Each bowl tipped about the finger at the lip as the jaws opened (bowl_06: axis z went from -0.83 to +0.43 within 0.6 s).
  - Three settled mouth up and three did not settle. bowl_04 rode up on the open left pad and stayed hanging there, 173 mm above its goal.
  - The Kit-free fall clearance was 0.0 mm for every bowl: the bowl falling straight down meets the gripper 35-40 mm into the fall.
  - bowl_07 had no reach: no free IK at any of 14 heights x 5 spins. Its 77 IK solutions all hit forearm vs tub; 59 also hit forearm vs upper rack.
- **Plates, rear bank** (plates 08, 05, 01, 04): no reach on both faces. Commonest pairs: forearm vs tub (24/28 solutions on the mouth face), held plate vs upper rack (20/24-28), wrist 1 and 2 vs upper rack.
- **Plates, the four that reached** (plate_06 included: its round-8 no reach was the quaternion bug):
  - plate_07 stopped on the rack after 45 mm of lowering. It held its lean through the jaw opening and the 25 mm back-off. The 0.10 m rise then lifted it about 65 mm by its rim, and it fell flat (lean 88° vs -32°, out of slot). Round 8, which backed out 0.10 m along the approach, left plate_07 0.1 mm from its goal.
  - plates 03 and 06 then stopped on the flat plate_07 after 10-15 mm and settled leaning 70-73°.
  - plate_02 settled in its slot, 14.7 mm / 15.6° off its goal. It moved plate_03 (16.6 mm) and plate_06 (45.4 mm).
  - No plate fell during a face switch (plate_07 switched twice). Every plate stayed on the fixed joint, with 0.0 mm drift and no pin fallback.
- **Start posture and joints.**
  - Bowls start at [-43, -102, 87, -165, -104, 0]°: wrist 3 at 0, all 4 home branches free at the 2 mm margin. Plates start at [-43, -86, 37, 48, 104, 180]°: the wrist-3-at-0 branches overlap the plate (held|wrist_1_link).
  - Pan swing per carry: bowls 44-74°, plates 35-42° (round 8: 164-229°). Carry travel sums: bowls 603-805°, plates 583-699°.
  - Max |q| over the bowls: pan 43°, lift 109°, elbow 138°, wrist 1 165° (the start itself), wrist 2 104°, wrist 3 94°. No bowl joint went past 180°.
  - Plates: wrist 3 reached 181.4° (starts at 180). Their trial max wrist 1 of 165° comes from the reset's q_home teleport; the carry max is 86-105°.
  - The elbow stayed up throughout, in every row.

| bowl | slot | spin, drop height, tries | settled, mouth (axis z) | lateral / height mm, tilt ° | lean vs goal (° from down) | landed on | fall clearance | max \|q\| pan, lift, elbow, w1, w2, w3 | pan swing |
|---|---|---|---|---|---|---|---|---|---|
| bowl_06 | lower_rear | -30°, 175 mm, 1 | yes, up (0.23) | 49.4 / 87.5, 112.4 | 103 vs 31.5 | LowerRack | 0.0 mm (gripper @35) | 43, 102, 87, 165, 104, 56 | 73° |
| bowl_02 | upper_bowl_09 | 0°, 100 mm, 1 | no (7.6° drift), up (0.92) | 12.1 / 48.0, 131.5 | 157 vs 43.8 | UpperRack | 0.0 mm (gripper @35) | 43, 102, 123, 165, 104, 69 | 48° |
| bowl_07 | lower_rearright | no reach: forearm vs tub 77/77, forearm vs upper rack 59/77, wrist 1 vs upper rack 43/77 | – | – | – | – | – | – | – |
| bowl_04 | lower_frontright | 0°, 125 mm, 1 | no (10.4 mm drift), down (-0.10) | 30.0 / 173.3, 48.0 | 84 vs 38.7 | pad_L | 0.0 mm (gripper @35) | 43, 109, 138, 165, 104, 65 | 74° |
| bowl_03 | upper_bowl_04 | 0°, 125 mm, 1 | yes, up (0.98) | 11.2 / 56.9, 131.2 | 168 vs 36.8 | UpperRack, bowl_02 | 0.0 mm (gripper @35) | 43, 102, 131, 165, 104, 63 | 44° |
| bowl_05 | lower_rear | 0°, 200 mm, 1 | no (9.4° drift), up (0.93) | 52.4 / 41.3, 156.9 | 158 vs 45.1 | LowerRack | 0.0 mm (gripper @35) | 43, 102, 98, 165, 104, 74 | 67° |
| bowl_01 | lower_rear | 0°, 125 mm, 1 | yes, up (0.56) | 27.9 / 22.9, 58.3 | 124 vs 68.5 | LowerRack, bowl_06 | 0.0 mm (gripper @40) | 43, 102, 124, 165, 104, 94 | 66° |

| plate | slot | face (switches), start | lowered, stop | settled | lean vs goal | lateral / height mm | in slot | blocking pair (no reach) |
|---|---|---|---|---|---|---|---|---|
| plate_08, 05, 01, 04 | lower_rear_06/07/08/04 | no reach | – | – | – | – | – | forearm vs tub (mouth 24/28), held plate vs upper rack (base 18-20/22-26) |
| plate_07 | lower_front_04 | mouth (2), 125 mm | 45 mm, contact LowerRack | yes | 87.8 vs -32.2 | 24.5 / 12.0 | no | – |
| plate_03 | lower_front_03 | mouth (0), 125 mm | 15 mm, contact plate_07 | yes | 70.2 vs -32.2 | 150.6 / 32.9 | no | – |
| plate_06 | lower_front_02 | mouth (0), 125 mm | 10 mm, contact plate_07 | yes | 73.1 vs -32.2 | 21.8 / 37.0 | no | – |
| plate_02 | lower_front_00 | mouth (0), 125 mm | 30 mm, contact plate_06 | yes | -18.5 vs -34.0 | 14.7 / 4.8 | yes | – |

## Kit-free checks (Mac, then the box: `r9_preview.py`, identical output)
- **Quaternion fix.** The goal quaternions were off unit length by up to 1.9e-7. Branch counts went from 0 to 8 for plate_06 (both faces, every height), from 4 to 8 for plate_07, and from 0 to 8 for the cup_04 / cup_08 S1 targets. bowl_05's S1 target stayed at 0 (out of reach, not the bug); its lip target has 8 branches either way. Solutions found both ways differ by under 6e-7 rad.
- **Lip reach** (first free pose, heights outer, spins inner, the earlier bowls at their goals): bowl_06 +175 mm at spin -30 (the exact lip only at +325), bowl_02 +100, bowl_04 +125, bowl_03 +125, bowl_05 +100, bowl_01 +25 at spin +15, bowl_07 none. The same at the 15 and 2 mm held margins.

## Commands (box, in `dishsim-isaac`; `$P` = /workspace/dishsim/data/results/robot/placing_v1; one chain, `r9_chain.sh`)
1. **Preview** (Kit-free, cores 2/6; gates both Kit runs): `run_py.sh data/logs/robot/r9_preview.py` gave `[RESULT] PASS r9 preview in 62.0 s`.
2. **Bowls** (cores 0/4), in parallel with step 3:
   `run_kit.sh code/execution/frigidaire/frigidaire_robot_episode.py --headless --no-media --mount data/results/robot/mount/posture_v1.json --shortcut-s 10 --legacy-tolerances --max-wall-seconds 9000 --out-root $P --place-goal $P/easy_bowls/load.json --variant D --hold rim-lip --row-variant D-lip --run-id c1_easy_bowls_Dlip`
   Result: `[RESULT] PASS place-goal easy_bowls D-lip: settled mouth down 0/7, settled any 3, failed at {'no reach': 1, 'not settled': 3, 'mouth up': 3}`.
3. **Plates** (cores 1/5): the same options with `--place-goal $P/easy_plates/load.json --variant I --row-variant I2 --run-id c1_easy_plates_I2`.
   Result: `[RESULT] PASS place-goal easy_plates I2: settled 4/8, failed at {'no reach': 4}`. No tracebacks in any log.
4. **Replays:** `run_py.sh code/execution/frigidaire/frigidaire_robot_replay.py --load-dir data/results/robot/placing_v1/easy_bowls --out data/media/robot/placing_v1 --variant D-lip` (2007 frames). Then the same with `easy_plates` and `--variant I2` (1338 frames). Both mp4s were re-encoded on the Mac at CRF 30: 37.9 to 12.3 MB and 27.3 to 8.9 MB.
5. **Page** (Mac): `/opt/anaconda3/bin/python3 code/execution/frigidaire/frigidaire_robot_report_page.py --records data/results/robot/limits_v1 data/results/robot/exhaustive_w1 data/results/robot/exhaustive_w2 --media data/media/robot/limits_v1 data/media/robot/exhaustive_w1 data/media/robot/exhaustive_w2 --out data/media/robot/report_v1` gave `[RESULT] PASS 7 sections 16 media files 55750964 bytes` (index.html 20.7 KB).

## Deviations
- **Home start at the 2 mm held margin** (coordinator, option B). The bowl frees the wrist-3-at-0 branches, so bowls start with wrist 3 at 0, not the plan's [-43, -86, 37, 48, 104, 180]. Every reset dict records the held margin, each home branch with its hits, and `home_why`.
- **The 2 mm held margin also applied to the I2 plates** (one rule for the welded dish). In the preview it changed no plate's reach.
- **Generic fixes.** reset() parks the dish before the jaws open for every variant. Rig.ik / ik_all orthonormalise every target (SVD), besides T_of normalising the quaternion.
- **Extra row fields.** `posture.carry_max_abs_deg` (the trial max counts the q_home teleport); `blocking` on every no-reach row with no free IK; `retreat` on I2 rows (both legs followed on all four plates).
- **Run order.** The preview ran first and gated both Kit runs (round 8 ran it beside the plates). The Mac's videos.json had one entry the box's lacked (the side-pinch gate clip), so it was pushed to the box before the replays.
- **Page.** The setup and Part B tables sit under the H1 without H2s, so the 7 requested sections are the only H2s. The Part B table has one row per kept round. The cup's start is shown with the plate's (no cup row this round; the preview gives the same branch).

## Box window and files
- **dishsim-l4.** Start attempt 1 at 00:12:06Z; STARTING 00:13:13Z; RUNNING 00:14:37Z after a STOPPED blip. All 141 `code/*.py` md5-identical to the Mac.
  - Chain: start 00:16:03Z, preview 00:17:06Z, plates 00:34:11Z, bowls 00:43:54Z, ALL DONE 00:45:31Z.
  - Copy-back 00:46:45Z; stop sent 00:46:49Z; STOPPED 00:55:49Z.
  - **About 43 billed minutes**, STARTING to STOPPED (the 75-minute cap was 01:28Z).
  - `brev ls` at 00:56Z: `dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`.
- **Changed files.**
  - `robot/rig.py`: T_of normalises, `orthonormal()` in ik / ik_all.
  - `robot/collide.py`: `HELD_ARM_MARGIN_M`, `ArmWorld.held_margin`.
  - `frigidaire_robot_episode.py`: `--hold rim-lip`, `LIP_SPINS_DEG`, `lip_tcp`, `lean_deg`, `blocking_pairs`, `RELEASE_BACK_M`, park before reset, back-then-rise release, the sum tiebreak with home branches, the held margin.
  - `frigidaire_robot_replay.py`: `--variant D-lip` / `I2`.
  - `frigidaire_robot_report_page.py`: Version 18 (`ROUNDS`, `setup_block`, `partb_block`, `round_block`, the per-round tables).
- **Suggested commit:** `Add bowl lip-pinch drop and plate insert fixes`
