# Bowls placed with the foot-ring grasp: mouth-down carry, supported release, hand above (Part B round 17): run note, 2026-10-29
Plan: plans/2026-10-29-ring-place.md. One box window on dishsim-l4, the v4 bowl (bowl.usda md5 f1cafd8b, HF da381dee).
- **Records.**
  - `data/results/robot/placing_v1/easy_bowls/goals_v4.json` (+ `goals_v4_{lower,upper}.json`).
  - 8 "D-ring" rows in `easy_bowls/dishes.jsonl` (now 87 lines; the box copy as run is `data/logs/robot/box_r17/dishes_box_r17.jsonl`).
  - `placing_v1/c1_easy_bowls_Dring/`: the run report and 4 trajectories.
  - `data/results/robot/goal_refresh/easy_bowls_{lower,upper}/`.
  - `kitfree_v1/insert_bowl/goal_supports_v4.json`.
  - bowl_07's row was re-judged after the window (`kitfree_v1/insert_bowl/rejudge_ring_axis.py`); the as-run verdict is in its `seated_as_run`.
- **Media.** `data/media/robot/placing_v1/{drop_easy_bowls_ring.mp4, landed_easy_bowls_ring.png}`, plus 2 `videos.json` entries.
  - The mp4 was re-encoded on the Mac at CRF 30: 18.4 to 5.7 MB, 1095 frames both.
  - The still's header line was redrawn on the Mac with the re-judged count (2/3 to 3/3). The as-run header was first reproduced pixel-exactly with the replay's font and layout. The as-run still is `data/logs/robot/box_r17/landed_easy_bowls_ring_as_run.png`.
  - The clip's bowl_07 caption is cut before its verdict, so the clip is unchanged.
- **Page.** Version 25 is built into `data/media/robot/report_v1`, not published: `[RESULT] PASS 11 sections 23 media files 64526357 bytes`, index.html 72 KB.
  - The bowls' best clip on top is now D-ring.
  - The D-perch clip is no longer referenced. It moved to `data/media/robot/report_v1_v24_dropped/`; its MANIFEST.txt names the path to null on publish.
- **Logs.** `data/logs/robot/box_r17/` holds the chain, 2 refresh logs, the placement and the replay, plus `data/logs/robot/r17_*.{sh,py}`.

## Headline vs Works-when
**3 of the 3 carried bowls are seated mouth down at their refreshed goals (bowl_07, bowl_04, bowl_03), so 3 of 7 overall.**
- bowl_07's goal has no bottom wire, so it is judged by the decided goal-pose fallback: centroid within 2 mm, axis within 5°. It passes at 0.37 mm and 3.33°.
- As run, the fallback compared the full rotation at spin 0 and failed bowl_07 at 107°. It was re-judged Kit-free after the window, at the coordinator's word that the decided rule is axis-based.
- Every stop came within 2 mm of its prediction (3/3, bowl vs rack). No gripper touched a bowl after the open (0/3). Every carry stayed mouth down with the hand above.
- The other 4 bowls are no-reach rows, with these blockers: 02 the tub; 05 the upper rack and the tub; 01 the basket, the lower rack and bowl_07; 06 the lower rack. bowl_06's flagged 10 mm try has no passing spin in either rack state.
- **Works-when: met.**
  - goals_v4.json holds 7 refreshed goals with their drift.
  - There are 8 D-ring rows, one replay and one still.
  - This note and Version 25 exist.

| bowl | spin / try | stop: measured vs predicted (pair, N) | jaw after open | mouth / seated / AT_GOAL (lateral, dz, tilt) | carry: max axis z, min TCP above centroid |
|---|---|---|---|---|---|
| bowl_06 (15 mm) | no reach: round 16's Kit-free table, gripper vs lower rack at 10.2 mm; teleported | – | – | – | – |
| bowl_06 (10 mm, flagged, its own turn) | no reach, upper rack in and both racks out alike: per rack state 23 descent starts collide (forearm/wrist 1 vs upper rack, held bowl vs lower rack), 7 no IK; teleported | – | – | – | – |
| bowl_02 | no reach: tub; teleported | – | – | – | – |
| bowl_07 | 255°, try 4 (spins 135/150/165: no RRT path down to the descent start under the mouth-down rule) | 0.5 vs 0.0 (lower rack, 4.18) | none | down / **seated** by the axis rule (0.37 mm, 3.33°; as run not seated: the full rotation at spin 0, 107°) / yes (0.3, 0.2, 3.3°) | -0.002, 0.1 mm |
| bowl_04 | 30°, try 1 | 0.1 vs 0.0 (lower rack, 3.84) | none | down / **seated** / yes (0.2, 0.0, 0.1°) | -0.036, 2.1 mm |
| bowl_03 | 15°, try 1 | 0.3 vs 0.0 (upper rack, 6.48) | none | down / **seated** / yes (0.9, 0.2, 0.6°) | -0.766, 44.3 mm; brushed bowl_02 on the carry (7.8 N, `carried_contact`; bowl_02 not moved) |
| bowl_05 | no reach: upper rack; teleported | – | – | – | – |
| bowl_01 | no reach: basket, lower rack, bowl_07; teleported | – | – | – | – |

**Goal refresh (each bowl alone, released 2 mm above its recorded goal, 150 + 60 ticks):**

| bowl | drift vs recorded | last-60-tick window | rests on |
|---|---|---|---|
| bowl_06 | 3.77 mm / 10.83° | 0.001 mm / 0.001° | lower rack |
| bowl_02 | 7.47 mm / 4.48° | 0 / 0 | upper rack |
| bowl_07 | 2.65 mm / 0.53° | 0.42 mm / 1.29° | lower rack |
| bowl_04 | 2.00 mm / 5.95° | 0 / 0 | lower rack |
| bowl_03 | 1.60 mm / 2.97° | 0 / 0 | upper rack |
| bowl_05 | **18.57 mm / 8.31° (flagged)** | **7.14 mm / 3.50°: still moving** | lower rack, bowl_06 |
| bowl_01 | 2.49 mm / 0.73° | 0.002 mm / 0.004° | lower rack |

- The v4 supports keep bowl_05 on bowl_06 (gap 0.04 mm). bowl_07 has no floor wire within 2 mm, so its seated test is the goal-pose fallback.
- **bowl_05's refreshed goal is not a rest pose.** It drifted 18.6 mm and was still sliding on bowl_06 (7.1 mm in the last 60 ticks). That goal needs the sampler re-run on v4.

## Commands
Box, in `dishsim-isaac`, with `$P` = /workspace/dishsim/data/results/robot/placing_v1. One chain, `data/logs/robot/r17_chain.sh`.
1. **Refresh** (cores 0/4 and 1/5): `run_kit.sh code/execution/frigidaire/frigidaire_drop_curve.py --headless --refresh-goals --load $P/easy_bowls/load.json --dishes bowl_06,bowl_07,bowl_04,bowl_05,bowl_01 --refresh-tag lower`. The upper run is the same with `--dishes bowl_02,bowl_03 --refresh-tag upper`.
   - Result: 2 × `[RESULT] PASS drop curve easy_bowls`, 70 / 56 s.
   - Merge: `run_py.sh data/logs/robot/r17_merge_goals.py $P/easy_bowls`.
2. **Supports** (Mac): `.venv-kitfree/bin/python data/results/robot/kitfree_v1/insert_bowl/goal_table_v4.py data/results/robot/placing_v1/easy_bowls/goals_v4.json data/results/robot/kitfree_v1/insert_bowl/goal_supports_v4.json`.
   - Result: `[RESULT] PASS goal supports v4: 7 bowls`. The file was pushed and copied into place as root.
3. **Placement** (cores 2/6, launched beside the refresh; it waited 44 s for the goals and supports): `run_kit.sh code/execution/frigidaire/frigidaire_robot_episode.py --headless --no-media --mount data/results/robot/mount/posture_v1.json --shortcut-s 10 --legacy-tolerances --max-wall-seconds 9000 --out-root $P --place-goal $P/easy_bowls/load.json --goals $P/easy_bowls/goals_v4.json --bowl-supports /workspace/dishsim/data/results/robot/kitfree_v1/insert_bowl/goal_supports_v4.json --variant D --hold foot-out --carry above --row-variant D-ring --no-reach bowl_02,bowl_05,bowl_01,bowl_06 --extra-row bowl_06:0.010 --run-id c1_easy_bowls_Dring`.
   - Result: `[RESULT] PASS place-goal easy_bowls D-ring: seated mouth down 2, settled 2/8, failed at {'no reach': 5, 'not seated': 1}, stops {'bowl': 3}, within 2 mm 3`. This is as run, before bowl_07's re-judge.
   - It ran 883 s with 0 tracebacks. Per carried bowl: 07 392 s, 04 138 s, 03 130 s.
4. **Replay:** `run_py.sh code/execution/frigidaire/frigidaire_robot_replay.py --load-dir data/results/robot/placing_v1/easy_bowls --out data/media/robot/placing_v1 --variant D-ring --goals data/results/robot/placing_v1/easy_bowls/goals_v4.json`.
   - Result: `[RESULT] PASS drop replay easy_bowls D-ring: drop_easy_bowls_ring.mp4 (1095 frames, 4 dishes) ... seated mouth down 2/3, at goal 3/3, jaw contact after the open 0/3`. This is as run; the manifest captions now read 3/3.
5. **Page** (Mac): `/opt/anaconda3/bin/python3 code/execution/frigidaire/frigidaire_robot_report_page.py --records data/results/robot/limits_v1 data/results/robot/exhaustive_w1 data/results/robot/exhaustive_w2 --media data/media/robot/limits_v1 data/media/robot/exhaustive_w1 data/media/robot/exhaustive_w2 --out data/media/robot/report_v1`.

## Deviations (where the plan was silent or the code was wrong)
1. **The lift.** The lift from the counter spot is the gate's 0.15 m (G3), not up to z_safe 1.25 m. z_safe is out of reach there: the elbow straightens at +0.16 m (Kit-free). The z-safe via-point above the descent start is unchanged. Retries start where the last try left the arm.
2. **The carry rule on every leg.** "Mouth down throughout, hand above" is enforced on every planned config: the straight-leg checks and the RRT validity, through `ArmWorld.free`'s `carry_ok` hook. It means bowl axis z < 0 and the TCP above the bowl centroid. Without it, the dry run's RRT turned bowl_07 mouth up. The coordinator approved both changes.
3. **bowl_06's 10 mm try.** It ran at its own turn, right after its no-reach row, per the coordinator. "After the seven" would have put bowl_05 on its goal. It was tried with the upper rack pushed in, then with both racks out; neither has a spin. The bowl was then teleported to its goal.
4. **The goal-pose fallback deviated from the decided rule, and is fixed.** The decided rule is centroid within 2 mm and axis within 5°, per the coordinator, citing the perch-roll plan grill.
   - `pose_seated` compared the full rotation, which fails any bowl placed at a spin. bowl_07, at spin 255, failed at 107°.
   - The code is now axis-based. That also covers the rear plates' fallback, and matches the drop curve's cup rule, which already used the axis tilt.
   - bowl_07's row now reads seated (0.37 mm, 3.33°), with `failed_at` cleared. Its `seated_as_run` keeps the as-run dict, `failed_at`, `why` and a one-line note.
   - I could not find the "axis" wording in plans/2026-10-29-perch-roll.md itself. Round 14's note says only "the 2 mm / 5° pose rule".
5. **Launch order.** The placement launched beside the refresh and polled for goals_v4.json and goal_supports_v4.json, which saved its Kit boot. Both files were written atomically.
6. The no-reach blockers are round 16's Kit-free goal table (v3 bowl, recorded goals), as decided.
7. The user raised the cap to 40 mid-window.

## Missing / open
- bowl_05's refreshed goal is not a rest pose: 18.6 mm drift, still sliding on bowl_06 (7.1 mm in the last 60 ticks). Its row is a teleport. That goal needs the sampler re-run on v4.
- The carry rule held literally but narrowly on bowl_07: axis z -0.002 is within 0.1° of sideways, and the TCP passed 0.1 mm above the centroid.
- The section shows the old goal still (recorded goals). No still of the refreshed goals was rendered.
- The descent start, 50 mm up the rack's axis, puts bowl_06 into the lower rack at every spin. A different descent axis is untested.

## Window (UTC)
- `brev start`: attempts 1-4 failed silently (22:38:39-22:50:25Z, unbilled). Attempt 5 went STARTING at about 22:50:50Z and RUNNING at 22:51:37Z.
- ssh at 22:52:15Z after `brev refresh`; the self-stop and the container were up.
- Push and checks ran at 22:52:16-22:52:37Z: code md5 142/142 after one stale list line, PY310_OK. The asset step was MD5_OK at 22:52:38Z (9 files, v4).
- The chain started at 22:52:50Z. The goals merged at 22:54:01Z and the supports were in place at 22:54:09Z. The placement finished at 23:08:11Z and the replay at 23:09:15Z (ALL DONE).
- Copy-back took to 23:09:40Z; `brev stop` at 23:09:44Z. STOPPED at 23:17:16Z (STOPPING 7.5 min): **about 26.5 billed min of the 40 cap** (STARTING about 22:50:50Z to STOPPED; within the first 35 too). `brev ls`: `dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`.

**Suggested commit:** `Place bowls by the foot ring onto refreshed v4 goals`
