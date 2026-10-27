# Hardware limits, exhaustive, window 2: run note, 2026-10-26
Plan: plans/2026-10-26-hardware-exhaustive.md (window 2, with the checkpoint-1 yaw retry). Records: `data/results/robot/exhaustive_w2/<run>/` (trials.jsonl and limits_<kind>.json; all 24 runs `[RESULT] PASS`) and `verdicts.json`. Media: `data/media/robot/exhaustive_w2/` (10 files and `videos.json`); window 1's manifest is now `data/media/robot/exhaustive_w1/videos.json` (11 files). Logs: `data/logs/robot/w2_*.log`.

## Works when (window-2 share)
- **Tilt limit per site: met at every site with a Kit-free path.**
  - Setup: limit = the first 0/5 tilt. Tilts were 15, 30, 45 and 60° about the round-3 axis, n = 5 per tilt. The release was at the target tilt and lowered until contact (`--down-orient target --contact-lag 0.15`), with seed 6.
  - Coverage: 13 of 15 bowl sites and all 8 cup slots climbed. Bowl rear x−0.17 and x−0.20 have no Kit-free G2 path at 15–60°, so they got no Isaac trials.
  - Scale: 195 Isaac trials in total. The carried-contact check (D4, the knuckle rest) fired in all 195, as in every earlier bowl and cup trial. G4 is the physical outcome.
- **Yaw retry: all three refused sites are reachable at reach yaw 8.** x−0.20 is 10/10, upper_bowl_00 is 10/10 and upper_bowl_04 is 6/10.
- **Plates in hand: no gap is reachable.** The Kit-free sweep finds G2 at 0/19 gaps, so there were no Isaac trials.
- **Opposite tilt direction: tried Kit-free only.** None of the 5 sites that passed 30° has a Kit-free path at −30°.
- **Nothing was cut.** No cup slot reached 60°.

## Bowl tilt limits (G4 per tilt, n = 5; "–" = no Kit-free G2 at that tilt)
| site | 15° | 30° | 45° | 60° | limit |
|---|---|---|---|---|---|
| upper_bowl_02, upper_bowl_03 | 5/5 | 5/5 | 0/5 | | 45° |
| upper_bowl_01 | 5/5 | 0/5 | | | 30° |
| upper_bowl_00, upper_bowl_04 | 0/5 | | | | 15° |
| upper_bowl_07 | 0/5 | – | | | 15° |
| lower_rear x−0.02, lower_rearright x−0.02 | – | 2/5 | 0/5 (never released) | – | 45° |
| lower_rear x−0.07 | – | – | 0/5 | | 45° |
| lower_rear x−0.12 | – | – | – | 0/5 (never released) | 60° |
| lower_frontright x+0.06, +0.08, +0.13 | 0/5 each | | | | 15° |

- The largest passing tilt is 30°. Upper slots 02 and 03 pass it 5/5 (final tilt 25° and 22°); rear x−0.02 and rear-right pass 2/5.
- Upper 02/03 at 45°: released at 40° and 46°, but they settle to 22° and 35°, out of tolerance.
- Front-right: released at 8°, the tines tilt the bowl to 34–35°, the same as at level.
- Upper 00/04/07 at 15°: released at 17–21°, they settle at 10°, 23° and 46°.
- "Never released": all 5 trials stop with "IK branch jump lowering to the rack". That is a planner limit, not a physical failure.

## Cup tilt limits (same protocol)
| slot | 15° | 30° | 45° | limit |
|---|---|---|---|---|
| glass_right_2 | 4/5 | 1/5 | 0/5 | 45° |
| glass_right_3 | – | – | 0/5 | 45° |
| glass_left_1, glass_left_3, glass_right_1 | 1/5, 2/5, 1/5 | 0/5 | | 30° |
| glass_left_0, glass_left_2, glass_right_0 | 0/5 | | | 15° |

- The largest passing tilt is 30° (glass_right_2, 1/5).
- How cups fail at 30–45°: in the right-hand slots they tip over (final 61–85°); in the left-hand slots at 30° they settle at 13–14°, out of tolerance.
- Two cup trials were never released: glass_left_1 at 15° ("lifting to the safe height: line collides") and glass_right_2 at 45° ("IK branch jump").

## Yaw retry
- Setup: level target, in-air release, legacy tolerances, seed 7, n = 3 per yaw in reach.jsonl order. The first yaw that released was topped up to 10.
- How the yaw was set: the target was turned by 2πk/12 minus the held grasp's yaw. The pick always takes grasp 7, so the turn was 30°, and the TCP sits at counter grasp k's yaw.
- Each row carries `yaw_retry: true` and `yaw` (the integer from reach.jsonl).
- Every site released at yaw 8 on its first trial, so the other G2 yaws were not tried.

| site | released at yaw | G4 | median final tilt |
|---|---|---|---|
| lower_rear x−0.20 | 8 | 10/10 | 5.3° |
| upper_bowl_00 | 8 | 10/10 | 9.4° |
| upper_bowl_04 | 8 | 6/10 (4 refused: "no collision-free IK at the hover") | 7.6° |

## Plates in hand
- Kit-free sweep (`frigidaire_robot_reach.py --plate-in-hand`, `exhaustive_w2/inhand/`): 19 gaps × 12 yaws. The target is the gap's upright family pose, turned about the plate axis.
- In-hand pose: round 1's median hold (gate2_plate t1, pivot 38.1°).
- Start: the home TCP on its first IK branch that is free with the plate held.
- Result: G2 at 0/19 gaps, and every gap stops at G0. 101 cells have no IK at the release; the other 127 have no collision-free IK at the pre-place.
- The Isaac spawn (`--plate-in-hand`) is implemented but was never exercised, because no gap reached G2.

## Deviations
- **Clock.** The brief's 10:15 EDT cap had already passed at the start: the Mac clock and a network Date header both said 12:06 EDT. The box stayed STOPPED until the coordinator set a new cap (stop by 15:45 EDT). The coordinator also allowed core 3 as a 4th Kit slot once the Kit-free sweeps ended.
- **Scheduling.** The box-side drivers are `w2_sched.py`, `w2_slot3.py`, `w2_plates.py`, `w2_opp.py` and `w2_media.py`, kept in the session scratchpad, not in the repo.
  - One Kit process ran per site, from a priority queue: yaw retry, then bowls, then cups.
  - Each budget ended at 19:13Z; all Kit work ended at 18:34Z.
  - When the main scheduler re-launched a cup run that core 3 had already finished, the episode refused to overwrite the record. But it truncated the original Kit log: the logs of glass_left_0, glass_right_1, glass_right_2 and glass_right_3 now hold only that refusal. Their records are intact.
- **Climb rule.** A climb skips a tilt with no Kit-free path, as in round 3, and stops at the first Isaac 0/5. Legacy tolerances were on, as in round 3.
- **Plate-in-hand start.** Two earlier Kit-free sweeps are kept as `_inhand_bad_home` and `_inhand_bad_liftpose`, both with a start inside the 15 mm margin:
  - the home branch nearest HOME_Q puts the plate within 15 mm of wrist_1;
  - the measured lift TCP puts the upper arm within 15 mm of the counter.
- **Opposite direction.** `orient_neg/` holds −15° and −30° for all 23 sites; the only G2 cells there are bowl upper_00 and cup glass_left_0/1 and glass_right_0/1.
- **Media.**
  - Pass clips: the first passing trial at 30° in run order (bowl rear x−0.02 t3, cup glass_right_2 t2).
  - Fail clips: both 45° clips (bowl rear x−0.02 t0, cup glass_right_2 t0) show the planner's IK branch jump, not a tip-over. A physical-fail clip (upper_02 at 45°, or glass_right_2 t1–t4) needs a box render.
  - There is no plate clip.
  - Stills: bowl upper, rear and rear-right at 30°; cup glass_left at 15° and glass_right at 30°. Front-right never passed, so it has no still.
- **Mac sleep.** The Mac slept 16:48–17:25Z and about 18:20–18:33Z; the box-side scheduler ran on.

## Box window and files
- **Box.** dishsim-l4: two `brev start` attempts sat STOPPED (16:18Z, 16:20Z). The third, at 16:21:48Z (12:21 EDT), reached RUNNING at 16:23:41Z.
  - Kit ran 16:24:24Z–18:34:21Z and the media until 18:35:25Z; copy-back took 18:36:04–18:36:47Z.
  - Stop sent at 18:36:47Z (14:36 EDT); `brev ls` showed STOPPED at 18:46:34Z (14:46 EDT).
  - The clone is restored to 5b3f2d7 and clean.
- **Changed files.**
  - `frigidaire_robot_episode.py` (+93 over window 1): `--tilts-adaptive`, `--yaw-retry`/`--yaw-top`, `--plate-in-hand`.
  - `frigidaire_robot_reach.py` (+103): `--tilts` for `--orient`, `--plate-in-hand`; `orient_table` accepts any kind; `measured_M` also returns the TCP.
- **Suggested commit:** `Add exhaustive hardware window 2: tilt limits, yaw retry`
