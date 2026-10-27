# Hardware limits, exhaustive, window 1: run note, 2026-10-26
Plan: plans/2026-10-26-hardware-exhaustive.md (window 1 only). Records: `data/results/robot/exhaustive_w1/<run>/trials.jsonl` (the existing format) and `verdicts.json` (the report page's input). Media: `data/media/robot/exhaustive_w1/`. Logs: `data/logs/robot/w1_*.log`. The tables pool the limits_v1 records where the brief says to.

## Works when (window-1 share)
- **Pick rate per counter spot, n = 5: met for bowl and cup** at the 3x3 subset (bowl n = 5–6 per spot, cup n = 5). The plate was Kit-free only, as planned: 0/25 spots.
- **Placement at level, n = 10: met at every site with a Kit-free G2 path.** That is 15 bowl sites and 8 cup slots, each at pooled n = 10. Sites without a G2 path get no Isaac trials: bowl upper_bowl_08–11 and cup glass_*_4/5.
- **Tilt limit per site:** window 2.
- **Knuckle and tines verdicts: met** (see Verdicts).
- **Videos and stills.**
  - Placement: one pass and one fail video per object (4 mp4).
  - Counter grid: `grid_bowl_pass`, `grid_cup_pass`, `grid_cup_fail`. There is no bowl grid failure video because all 53 bowl grasps held.
  - Release-step stills: bowl upper, bowl lower rear, cup glass left, cup glass right.
- **Results notes and page:** this note covers window 1; -w2.md comes after window 2. Another agent builds the page.

## Verdicts
1. **Knuckle contact: real.** Run `--grasp-gate bowl --n 1 --dump-contacts` (`w1_knuckle_dump`, 943 ticks dumped).
   - At the close, the right inner knuckle strikes the rim and tips the bowl. Its render mesh overlaps the bowl mesh for 80 ticks, with up to 36 N on its collider.
   - After the 21.3° pivot, the bowl rests on the left inner knuckle for the whole 5 s hold: 48.7–49.5 N on the collider (the pads carry 91 N and 42 N).
   - That knuckle's render mesh is 0.17–0.18 mm from the bowl's visual mesh, 0.7 mm below the rim.
   - The knuckle collider is the convex hull of that same render mesh (the asset has no separate visual mesh), so the meshes themselves touch.
   - D4 is unchanged and fires (`carried_contact`).
2. **Tines (robot-free teleport plus benchmark settle, n = 5 per site, ±5 mm): the rack tilts the bowl at all four sites.**

   | site | median tilt | past 15° |
   |---|---|---|
   | front-right x +0.06 | 32.1° | 5/5 |
   | front-right x +0.08 | 36.3° | 5/5 |
   | front-right x +0.13 | 32.5° | 5/5 |
   | rear-right | 17.6° | 4/5 |

   The robot trials tilted about as much: 21–35° front-right, 20–24° rear-right. Only one teleport, at rear-right, settled at the target.
3. **Seed-free IK reach** (`--targets level --seed-free`, 372 jobs, 31 min):

   | kind | G1 seed-free (round 1) | G2 seed-free (round 1) |
   |---|---|---|
   | bowl | 16/19 (15/19) | 15/19 (15/19) |
   | cup | 8/12 (8/12) | 8/12 (8/12) |

   - The only site that changed is upper_bowl_08: G0 → G1, still no G2.
   - Per-yaw G2 counts moved both ways (rear-right 2 → 5, front-right 6/4/6 → 4/3/5). The RRT's 3 s wall budget ran under CPU contention, so only the site-level counts are comparable.
   - The HOME_Q seed did not limit reach at level.

## Placement at level, pooled n = 10 (G4 = released, settled, at target; D4 counted = 0 in every new trial)
Bowl: in-air level release with legacy tolerances. Round 1–2 records plus the new top-ups (seeds 3, 4 and 5).

| site | new | passes | 95 % CI | median tilt of new releases |
|---|---|---|---|---|
| upper_bowl_01 / 02 / 03 / 07 | 2 / 1 / 4 / 4 | 10/10 each | 0.69–1.00 | 5.7 / 6.7 / 6.4 / 7.4° |
| lower_rear x −0.07 / −0.12 | 1 / 1 | 10/10 each | 0.69–1.00 | 10.0 / 7.2° |
| lower_rear x −0.17 / −0.02 | 1 / 2 | 3/10 each | 0.07–0.65 | 27.7 / 14.7° |
| lower_frontright x +0.06 / +0.08 / +0.13 | 1 each | 0/10 each | 0.00–0.31 | 29.8 / 32.7 / 29.5° |
| lower_rearright x −0.02 | 2 | 0/10 | 0.00–0.31 | 23.2° |
| lower_rear x −0.20, upper_bowl_00, upper_bowl_04 | 7 each | 0/10 each | 0.00–0.31 | never released |

The last row was never released. Isaac's planner refuses the yaw-0 level pose every time: x −0.20 with "IK branch jump on the line", and slots 00 and 04 with the gripper hitting the upper rack 21–27 mm from the end of the line.

Cup: contact release (`--release down --down-orient target --contact-lag 0.15`). New trials are 7 per slot, pooled with r2_place_cup_down (left) and r3_cup_right_fix (right).

| slot | passes | 95 % CI | released (new) | median tilt of new releases |
|---|---|---|---|---|
| glass_left_0 | 1/10 | 0.00–0.45 | 7 | 19.0° |
| glass_left_1 | 4/10 | 0.12–0.74 | 7 | 7.6° |
| glass_left_2 | 6/10 | 0.26–0.88 | 7 | 5.2° |
| glass_left_3 | 2/10 | 0.03–0.56 | 4 | 18.6° |
| glass_right_0 | 2/10 | 0.03–0.56 | 4 | 8.7° |
| glass_right_1 / 2 / 3 | 0/10 each | 0.00–0.31 | 5 / 6 / 5 | 14.4° each |

- Cups total 15/80.
- 11 new cup trials were never released: "no collision-free IK at the hover" at the target orientation.
- Left cups are released a median 62 mm below the level pose, on the channel floor; right cups 35 mm below.
- Arm or palm contact was flagged in 22 of the 28 left trials.

## Counter pick grid
- **Kit-free (5x5, 12 yaws, upper rack pushed in).**
  - Bowl and cup: 25/25 spots G2, 12/12 yaws everywhere.
  - Plate: 0/25 spots: G1 fails at the grasp (the planner's 15 mm clearance).
  - Spots: x 0.10–0.30 in 50 mm steps. The y range is set by the slab's front edge: bowl −0.221 to −0.099 (30.5 mm steps), cup −0.259 to −0.061 (49.7 mm), plate −0.181 to −0.139 (10.3 mm).
- **Isaac G3 (lift 0.15 m plus 5 s hold) at the 3x3 subset.**
  - Bowl: 53/53 held, n = 5–6 per spot, every spot 1.00 (95 % CI 0.48–1.00 at n = 5, 0.54–1.00 at n = 6). The median pivot is 20.8–21.3° at every spot.
  - Cup: 42/45 held. Spots (0.10, −0.259), (0.30, −0.259) and (0.30, −0.16) are 4/5 each, all three failures misses ("no wall between the pads"); the other six spots are 5/5.
  - D4 counted = 0 in every trial.

## Deviations
- **Wall-budget abort.**
  - The first three top-up runs hit the episode's default `--max-wall-seconds 1700` at 09:02Z (BudgetExpired, 1400 s after the start), after 11, 10 and 12 trials.
  - The continuations (`*2`) ran with `--max-wall-seconds 9000` and seed 4, so no trial repeats a seed-3 perturbation.
  - The chained grid and blocked-site runs had started with the same budget. They were stopped at 09:08Z after 4, 4 and 2 trials (seed 0 grid, seed 3 blocked); those rows are pooled and the reruns used new seeds.
- **Cup pooling.** The pooled records used down-orient "hang" (r2 also used lag 0.05). The new trials use `--down-orient target` as briefed.
- **Bowl seeds and order.** The 3 planner-blocked bowl sites were topped up too (`w1_bowl_blocked*`), split across two slots.
- **Grid trial order.** Grid trials run trial-major across spots (`--interleave`), so a cut would leave every spot with a similar n.
- **Stills.** They come from the logged Isaac trajectory at the last closed-jaw frame (new `--release-still`), not from the Kit-free `--still` render of the pre-place pose.
- **Videos.** `bowl_level_fail`, `grid_*` and `grid_cup_fail` play at 2x (`--every 2`).
- **Cores.** The knuckle and tines checks ran on core 3 beside the Kit-free jobs.

## Box window and files
- **Box window.**
  - dishsim-l4 `brev start` at 08:33:28Z (04:33 EDT); ssh at 08:35:50Z; `brev refresh` was needed at 08:37Z.
  - Kit runs 08:38:29Z–10:15:49Z; copy-back at 10:17Z.
  - Stop sent at 10:18:05Z (06:18 EDT); `brev ls` showed STOPPED at 10:26:46Z (06:26 EDT).
  - The box clone was restored to 5b3f2d7 and is clean. The changed sources matched the Mac by md5 before the restore.
- **Changed files.**
  - `code/execution/frigidaire/frigidaire_robot_episode.py` (+191/−9): `--site-n`, `--out-root`, `--dump-contacts`, `--counter-grid`/`--grid-g2-only`/`--grid-part`, interleaved grid.
  - `code/execution/frigidaire/frigidaire_robot_reach.py` (+99/−4): `--seed-free`, `--counter-grid`.
  - `code/execution/frigidaire/frigidaire_robot_replay.py` (+15): `--release-still`, `--every`.
  - `code/frigidaire/src/dishsim_frigidaire/robot/harness.py` (+2): per-step contact rows with collider paths.
- **New file:** `code/execution/frigidaire/frigidaire_tines_test.py` (162 lines).
- **Suggested commit:** `Add exhaustive hardware window 1: checks, top-ups, grid`
