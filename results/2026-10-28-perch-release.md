# Bowls lowered on a sensed contact from an over-lean lip carry (perch-and-drop, Part B round 11, posture-fix mount): run note, 2026-10-28
Plan: plans/2026-10-28-perch-release.md.
- **Records.** `data/results/robot/placing_v1/c1_easy_bowls_Dperch/` and 7 "D-perch" rows in `easy_bowls/dishes.jsonl` (now 65 lines).
- **Media.** `data/media/robot/placing_v1/{drop_easy_bowls_perch.mp4, landed_easy_bowls_perch.png}` and 2 new `videos.json` entries; the goal still is reused. The mp4 was re-encoded on the Mac at CRF 30: 31.4 to 10.2 MB, 1400 frames both (the raw file is in the scratchpad).
- **Page.** Version 20, built into `data/media/robot/report_v1`, not published. Its 2 round-10 bowl copies moved to `data/media/robot/report_v1_v19_dropped/`; its MANIFEST.txt lists the paths to null on publish.
- **Kit-free copies.** `data/results/robot/kitfree_v1/` (guarded, pinchdepth, overlean, tangent, perch_smoke; README names each file's origin).
- **Logs.** `data/logs/robot/r11_*`: `r11_chain.sh` (the box chain), `r11_easy_bowls.log` (Kit), `r11_replay_bowls.log`.

## Headline vs Works-when
**Settled and mouth down 3/7 (bowl_06, bowl_04, bowl_05; round 10: 0/7), settled any 6/7, bowl_07 no perch. The sensed stop was within 2 mm of the predicted perch on 1 of 6 lowered bowls (bowl_05, -0.9 mm). bowl_01's sensor never tripped.**
- **Works-when.** 7 D-perch rows, one replay, one still and Version 20 exist. The 2 mm perch criterion failed on 5 of 6 lowered bowls.
- **Stops.** The sensor tripped on 5/6, each time on a bowl-vs-scene pair; no stop came from the hand.
  - Lower rack: bowl_06 3.51 N, bowl_04 9.38 N. Upper rack: bowl_02 7.17 N, bowl_03 5.14 N.
  - bowl_05: lower rack 7.34 N plus bowl_06 2.68 N, 10.02 N in all. A 0.25-0.55 N graze on bowl_06 ran for 114 ticks before that, under the threshold.
  - bowl_01: no contact down to the bound (83 mm of descent, 0.0 N on every tick). It was let go 111.8 mm above its goal.
  - Sensor facts: the UR5e F/T is rated range 50 N, precision 3.5 N, accuracy 4.0 N; the bowl weighs 0.66 N; the pads squeeze the welded bowl at 88 / 50 N (excluded from the sum).
- **Perch, predicted / measured (mm).** bowl_06 89 / 81.3 (-7.7), bowl_02 68 / 85.2 (+17.2), bowl_04 88 / 90.4 (+2.4), bowl_03 76 / 85.0 (+9.0), bowl_05 73 / 72.1 (-0.9), bowl_01 147 / no contact.
  - Every bowl reached its descent start 8.2-9.7 mm and 2.5-2.9° off the planned pose: arm lag 0.041-0.043 rad under the welded bowl, TCP 5.3-6.4 mm off, the bowl 5-7 mm low.
  - Kit-free check after the run (Mac, `kitfree_v1/perch_smoke/stop_check.py`): the bowl at its logged stop orientation, lowered alone, first touches the rack at 85 / 91 / 80 mm on bowls 02 / 04 / 03 (measured 85.2 / 90.4 / 85.0).
  - On bowls 06 and 02 the first touch did not trip the sensor at once. bowl_06 touched at 84.7 mm (3.48 N), fell to 0.2 N, then rose about 0.1 N per tick for 30 ticks and tripped at 81.3 mm. bowl_02 touched at 3.77 N, read 1.2-1.9 N for 7 ticks, then 4.36 and 7.17 N.
- **bowl_01.** Its search ran 7 ticks after the reset teleported the arm home from bowl_05's rim. bowl_05 then fell 67.8 mm off the open pad_L and stayed mouth down. The predicted perch on bowl_05 (147 mm) was no longer there.
- **Roll prediction vs outcome: 3/6 matched.**
  - Matched: bowl_06 (down, down), bowl_03 (up, up), bowl_05 (down, down).
  - Missed: bowl_02 predicted down, ended up; bowl_04 predicted up, ended down; bowl_01 predicted down on bowl_05, fell from the bound and ended up.
- **Hand away?** bowls 06 and 05 ended with the fingers at the rim (backed 54.7 / 99.6 mm, rim clearance 0.2 / 0.0 mm), so their settle was judged resting on pad_L.
  - When the next reset teleported the arm home they moved 47.8 / 67.8 mm and stayed mouth down (axis z -0.51 to -0.72, -0.46 to -0.65).
  - bowls 02, 04, 03 and 01 backed off the full 100 mm and rose.
- **Descent.** Mean speed 19.9-20.2 mm/s; 96-100 % of the 6-tick windows within 10 %. Weld drift at the stop 0.35-0.43 mm (0.35-0.42 at the start). No other arm link touched the scene during a descent.
- **Carry.** Every lowered bowl started on [-42.8, -85.8, 37.4, 48.4, 104.0, 180.0] and took its first ranked combo (1 try).
  - Largest joint travel, measured (predicted): bowl_06 150.5° (127.2), bowl_02 109.8 (85.3), bowl_04 90.2 (85.8), bowl_03 83.9 (77.5), bowl_05 137.3 (133.3), bowl_01 118.5 (108.9).
  - Within 10° on 4/6; the two misses are RRT second legs. Round 10: 146 / 128 / 152 / 125 / 131 / 130°.
  - The search took 10.7-26.3 s per bowl on the box (bowl_07: 45.5 s with its blocking pairs).

| bowl | slot | tilt goal → carried | spin | perch pred / measured mm | start / descent mm | stop: pair, force, ticks | roll pred (COM mm) → mouth (axis z) | settled | lean final vs goal | largest travel (pred) | retreat |
|---|---|---|---|---|---|---|---|---|---|---|---|
| bowl_06 | lower_rear | 31.5 → 66.5° | -20° | 89 / **81.3** | 150 / 61.9 | lower rack, 3.51 N, 388 | down (+5.5) → **down** (-0.51) | yes | 59.3 vs 31.5 | 150.5 (127.2) | 54.7 mm, fingers at the rim |
| bowl_02 | upper_bowl_09 | 43.8 → 73.8° | 0° | 68 / **85.2** | 125 / 32.8 | upper rack, 7.17 N, 207 | down (+4.6) → up (0.96) | yes | 163.9 vs 43.8 | 109.8 (85.3) | 100 mm, rose |
| bowl_07 | lower_rearright | no perch: no free IK at the contact pose for any of 13 tilts × 19 spins; forearm vs tub on 207/208 IK solutions | – | – | – | – | – | – | – | – | – |
| bowl_04 | lower_frontright | 38.7 → 68.7° | +45° | 88 / **90.4** | 150 / 54.9 | lower rack, 9.38 N, 330 | up (-18.3) → **down** (-0.73) | yes | 42.8 vs 38.7 | 90.2 (85.8) | 100 mm, rose |
| bowl_03 | upper_bowl_04 | 36.8 → 86.8° | +5° | 76 / **85.0** | 150 / 59.4 | upper rack, 5.14 N, 358 | up (-10.5) → up (0.97) | yes | 166.0 vs 36.8 | 83.9 (77.5) | 100 mm, rose |
| bowl_05 | lower_rear | 45.1 → 70.2° | -10° | 73 / **72.1** | 125 / 47.2 | lower rack + bowl_06, 10.02 N, 303 | down (+10.4) → **down** (-0.46) | yes | 62.9 vs 45.1 | 137.3 (133.3) | 99.6 mm, fingers at the rim |
| bowl_01 | lower_rear | 68.5 → 90.0° | +5° | 147 (on bowl_05) / none | 200 / 83.0 | none: 0 N down to the bound | down (+8.4) → up (0.71) | yes | 135.2 vs 68.5 | 118.5 (108.9) | 100 mm, rose |

bowl_04 settled 1.5 mm in height and 4.2° from its goal orientation, 40.1 mm lateral.

## Kit-free smoke (Mac, before the box)
- **Search smoke.** `.venv-kitfree/bin/python smoke_overlean.py` (now `data/results/robot/kitfree_v1/perch_smoke/`): `overlean_search` on all 7 bowls in 43 s, the earlier bowls at round 10's landed poses.
  - The perch and roll per tilt matched the scratch tables at every tilt. `overlean_tcp` equals `lip_tcp` at the goal tilt (largest difference 3e-16).
  - Picks: bowl_06 66.5° spin -20 perch 89 mouth-down; bowl_02 73.8° 0 68 down; bowl_07 no perch; bowl_04 68.7° +45 88 up; bowl_03 86.8° +5 76 up; bowl_05 55.1° +5 136 (on bowl_06) down; bowl_01 73.5° +30 34 up.
- **Plan correction.** The smoke caught the plan's bowl_03 entry: the rule's lowest perch is 86.8° (76 mm), not 61.8° (91 mm, its smallest passing tilt). The coordinator corrected the plan before the window.
- **Box vs smoke.** Bowls 06, 02, 04 and 03 took the smoke's combos on the box. bowl_05 took 70.2° -10 (perch 73 mm, lower rack) and bowl_01 took 90.0° +5 (perch 147 mm on bowl_05, mouth-down), because bowls 06 and 05 lay elsewhere in this run.

## Commands (box, in `dishsim-isaac`; `$P` = /workspace/dishsim/data/results/robot/placing_v1; one chain, `r11_chain.sh`)
1. **Bowls** (cores 0/4): `run_kit.sh code/execution/frigidaire/frigidaire_robot_episode.py --headless --no-media --mount data/results/robot/mount/posture_v1.json --shortcut-s 10 --legacy-tolerances --max-wall-seconds 9000 --out-root $P --place-goal $P/easy_bowls/load.json --variant D --hold rim-lip --carry overlean --row-variant D-perch --run-id c1_easy_bowls_Dperch`
   - Result: `[RESULT] PASS place-goal easy_bowls D-perch: settled mouth down 3/7, settled any 6, failed at {'no perch': 1, 'mouth up': 3}, attach {'pin': False, 'why': None}`.
   - It ran 21.1 min with no tracebacks. Invariants: one `carried_contact` (bowl_05 vs bowl_06, 0.41 N).
2. **Replay:** `run_py.sh code/execution/frigidaire/frigidaire_robot_replay.py --load-dir data/results/robot/placing_v1/easy_bowls --out data/media/robot/placing_v1 --variant D-perch`
   - Result: `[RESULT] PASS drop replay easy_bowls D-perch: drop_easy_bowls_perch.mp4 (1400 frames, 7 dishes), landed_easy_bowls_perch.png; settled mouth down 3/7, settled 6/7`.
3. **Page** (Mac): `/opt/anaconda3/bin/python3 code/execution/frigidaire/frigidaire_robot_report_page.py --records data/results/robot/limits_v1 data/results/robot/exhaustive_w1 data/results/robot/exhaustive_w2 --media data/media/robot/limits_v1 data/media/robot/exhaustive_w1 data/media/robot/exhaustive_w2 --out data/media/robot/report_v1`
   - Result: `[RESULT] PASS 7 sections 16 media files 54439381 bytes` (index.html 27.4 KB; media/ holds exactly the 16 referenced files).

## Deviations
- **Descent bound.** It is 30 mm under the predicted perch, which is 80-105 mm below the start, not a flat 80 mm. The start is rounded up to the 25 mm grid, so a bound 80 mm below it could end only 5 mm under the perch.
- **Perch per spin.**
  - The bowl alone is lowered from 350 mm once per tilt, at spin 0: it is axisymmetric, so the spin moves only the hand.
  - Each spin's perch is re-lowered from that perch + 3 mm. The 192 convex pieces differ by under 1 mm (bowl_06 66.5° spin +5 touches at 88 mm vs 89).
- **Bowl first.**
  - It is checked from the descent start down to the perch (1 mm grid; the open and closed gripper hulls and the links upper arm to wrist 3), not from 350 mm as the scratch did.
  - The contact pose and the descent start must also be free at 15 mm with both jaw models. No free combo failed bowl-first, in the smoke or on the box.
- **Ranking.** It stops once the chosen home branch has 6 passing combos and no later tilt can tie them (perch ± 1 mm). Straight legs are not a ranking key, because the plan's order is roll, perch, travel; transit_to still moves straight where the path is free.
- **Model inputs.**
  - The closed-jaw hulls come from the bowl gate's median hold (gate2 trial 1, last frame), as in the scratch.
  - The centre of mass is the visual mesh's uniform-density centre (dish z 33.87 mm), as in the scratch; PhysX's, from the convex pieces, is at 34.5 mm.
- **Sensor scope.** The forearm and wrists 1-2 are not in the sum: the flange sensor sits past wrist 3. Their scene contacts during a descent are listed in `stop.other_arm_contacts` (none in this run).
- **Waypoints.** One IK waypoint per physics tick (1/6 mm), at constant speed from the first tick, with no ramp; the measured speed is in `stop.speed_mm_s`.
- **No perch rows.** They carry the arm model's blocking pairs over every contact pose, as round 10's no-reach rows do.
- **Drop tries.** A failed try falls back only to the chosen home branch's ranked combos, as in round 10; no bowl needed a second try.
- **Chain.** It writes no ALL DONE after a Kit or replay failure, so the Mac decides; the end driver's cap is the backstop. This was not triggered.
- **Replay header.** The D-perch caption is cut at the frame's right edge (about 120 characters), so the settled and mouth results are not visible in the mp4. The page's caption and table carry them.
- **Not fixed (found in the run).** The reset runs the search 7 ticks after it teleports the arm home. A bowl left resting on the open fingers is still falling then, which is what moved bowl_01's predicted perch.

## Box window and files
- **dishsim-l4.**
  - Start attempt 1 at 07:23:46Z. The 07:24:19Z poll read STOPPED and the 07:24:36Z poll RUNNING; STARTING was not seen between them.
  - ssh at 07:25:04Z after `brev refresh`; self-stop running (PID 644); container Up; 22 GB free.
  - All 141 `code/*.py` md5-identical to the Mac after the rsync; the 8 data inputs matched by md5.
  - Chain: start 07:25:53Z; Kit done 07:46:59Z; replay and ALL DONE 07:48:02Z.
  - Copy-back 07:48:27-07:48:33Z; stop sent 07:48:37Z; STOPPED 07:57:08Z.
  - **About 33 billed minutes**, STARTING (at most 07:24:36Z) to STOPPED (cap 60; the end driver's cap of 08:12:19Z was not reached).
  - `brev ls` at 07:58:14Z: `dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`.
- **Changed files.**
  - `frigidaire_robot_episode.py`:
    - `--carry overlean` and the D-perch label.
    - Constants `OVERLEAN_STEP_DEG`, `PERCH_*`, `DESCENT_*`, `SENSOR_*`, `PERCH_LINKS`.
    - Helpers `overlean_tilts`, `goal_hor`, `overlean_tcp`, `closed_gripper_hulls`, `scene_targets`, `min_clear`, `contact_pos`, `part_name`, `perch_descent`, `hand_objs`, `hand_first`, `overlean_search`, `perch_rec`.
    - reset(): the over-lean search (`reset.perch_search`) and the rack order.
    - Closures `closed_hulls`, `flange_force`, `lower_until_contact`, `perch_drop`; "no perch" in the summary.
  - `frigidaire_robot_replay.py`: `--variant D-perch` (files `_perch`) and its caption.
  - `frigidaire_robot_report_page.py`: Version 20 (ROUNDS; TABLE_ONLY gains D-lip2; `perch_table`, `kitfree_block`, `perch_ok`; the no-perch failure mode; `--kitfree`; the setup and start text).
  - New: `data/logs/robot/r11_chain.sh`, `data/results/robot/kitfree_v1/`.
- **Suggested commit:** `Add over-lean perch carry with a sensed bowl stop`
