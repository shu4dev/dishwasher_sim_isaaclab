# Posture fix: un-wound elbow-up IK, posture-aware drop goals, a lower pedestal picked Kit-free (Part B, cups at the new mount)
Slug: 2026-10-27-posture-fix | From: plans/2026-10-27-cup-side-drop.md, code/frigidaire/docs/robot.md

## Goal
At a pedestal picked by a Kit-free search, the arm carries each of the 8 easy_cups cups (welded in hand, flipped mouth-down) to the lowest free drop pose with no joint beyond 180 deg and the elbow up; the page says how many settle mouth-down, each joint's range and travel, and shows the replay above round 6's.

## Decisions (from grilling, 2026-10-27 13:00-13:40 EDT)
- Scope: the IK and planner rules AND a lower mount (user). Rejected rules only (the crane posture stays: the hand works 0.27 m from the base axis at a counter-height base) and mount only (the winding stays).
- No winding: IK solutions wrapped to (-180, 180] deg and unwrapped toward the seed only within +-190 deg (the elbow +-180); RRT bounds +-190 deg; the table reports max |q| per joint and flags values past 180. Rejected hard +-180 (a crossing would be a no-reach) and wrist 3 free.
- Elbow up only (`Rig.ELBOW_UP_ONLY = True`); `goal_qs` ranks free solutions by the smallest max joint move from the current joints (the "wrist_1 highest" rule retires; the collision model keeps the wrist housing off the dishes).
- Posture-aware drop goal: the lowest free height as before; within it the yaw with the smallest joint move wins; RRT shortcutting 10 s (was 2 s); every carry logs its joint travel. Rejected leaving the carry and straight moves only.
- Mount search (Kit-free, `frigidaire_robot_reach.py --mount-search`, 6 workers, 30 min cap): x in {0.45, 0.60}, y in {-0.55, -0.70, -0.85}, z in {0.45, 0.60, 0.75, 0.90}, yaw facing the work centre, plus today's mount (0.45, -0.55, 0.90, yaw 2.60) as the baseline; per mount: the 23 easy goals (bowl rim pinch 21 deg, plate assumed rim hold 38 deg, cup h45 hold flipped) with an un-wound elbow-up collision-free drop IK at any height and yaw (early exit) + the two measured counter picks (bowl rim pinch, cup wall pinch) at the spot (0.20, -0.12); ties: the cup base-side pick passes a yaw, then the lowest pedestal. The rule picks (written to data/results/robot/mount/posture_v1.json, the episode's `--mount`). Rejected the user picking from the table (two windows) and a hand-set mount.
- Search order and cap (plan grill, user): mounts visited lowest pedestal first; at the 30 min cap the best so far wins and the table marks the skipped mounts. Score as written (23 goals + the 2 measured picks, base-side only a tiebreak); z_safe 1.25 m kept so the travel shows what a low pedestal costs. Rejected goals-only, 3 picks, a mount-relative safe height, a shrunk grid, no cap.
- Gate: when the search's base-side pre-check passes >= 1 yaw at the picked mount, the Isaac gate runs (n = 10, as round 6); the drop uses its median hold when >= 5/10 hold, else round 6's fallback (the h45 mouth hold rolled 180 deg). Rejected skipping it and always running it.
- Success: settled mouth-down count (round 6's 6/8 is the reference, not a gate), max |q| <= 180 deg on every joint for the whole run, elbow up throughout, travel per carry listed. Otherwise as round 6: 12 yaws, 25-350 mm, DROP_TRIES 6, RRT 15 s, counter edge -0.17, witness order, lower rack pushed in, `T_home` and z_safe 1.25 m unchanged (the start posture un-wound, elbow up, nearest HOME_Q).
- Window: one box window of about 90 min (user yes: search <= 30, gate <= 20, drop ~35, replay + copy-back 10), box off right after.
- Outcome (box 14:07-14:35 EDT, about 37 billed min; results/2026-10-27-posture-fix.md): the search (135 s, all 25 mounts) picked (0.45, -0.70, 0.60) yaw 2.56, score 23 (goals 21/23, picks 2/2; four mounts tied, lowest pedestal won; today's mount 22); the base-side pre-check passed 0/12 yaws at every mount, so no gate ran and the drop used the fallback hold. Drops 8/8 settled mouth down (round 6: 6/8), elbow up throughout, max |q| pan 60, lift 109, elbow 150, wrist 1 165, wrist 2 104, wrist 3 188 deg (the one miss: the start posture at this mount has wrist 3 at 180 because the home IK nearest HOME_Q collides with the held cup); only cup_07 inside the old tolerance, cups 01/06/04 tilted 44-46 deg. Deviation: a mount counts only if the episode can start there (z 0.45 mounts all fail that). Report Version 16.
- Page: Version 16: a new Part B section above round 6's ("Cups: side pinch, mouth-down drop at the lower mount": the mount table, the joint-range table, the drop table, the replay, the landed still); a mount note in the setup section; every earlier table labelled with the old mount.

## Change
- robot/rig.py: `ELBOW_UP_ONLY = True`; `ik`/`ik_all` wrap, then unwrap toward the seed only inside +-190 deg (`within_limits` against +-190, elbow +-180); nearest-to-seed ordering kept.
- robot/collide.py: `rrt_connect_np` bounds +-190 deg (elbow +-180); the shortcut budget a parameter.
- frigidaire_robot_episode.py: `goal_qs` ordering; the drop yaw chosen by joint move; `--shortcut-s 10`; per-carry travel and max |q| per joint in a `posture` dict of every row; rows variant "D-side-m2"; the pedestal from the mount file as today.
- frigidaire_robot_reach.py: `--mount-search` (grid, score, the base-side pre-check, the mount file with the full table).
- frigidaire_robot_replay.py / frigidaire_robot_report_page.py: the new section, the joint-range table, the old-mount labels.
- Records: mount/posture_v1.json, placing_v1/{partA_cup_side_base45_m2/, c1_easy_cups_Dside_m2/}, easy_cups/dishes.jsonl (+8 rows), media placing_v1/{drop,landed}_easy_cups_side_m2.{mp4,png}, videos.json.

## Run
Box: `run_py.sh frigidaire_robot_reach.py --mount-search --workers 6`; the gate if the pre-check passes; `frigidaire_robot_episode.py --headless --no-media --place-goal .../easy_cups/load.json --variant D --hold side-base --mount data/results/robot/mount/posture_v1.json --shortcut-s 10 --run-id c1_easy_cups_Dside_m2 --max-wall-seconds 9000`; replay + still; copy back; stop. Mac: page rebuild, publish (index.html + the new media).

## Works when
posture_v1.json with the 25-row mount table, 8 rows with settled / mouth / drop height / landing / errors / posture (max |q| per joint, travel), no joint past 180 deg in any row, the replay and the still, results/2026-10-27-posture-fix.md, Report Version 16.

## Assumed / skipped
- Assumed: the counter spot and the 12 gate yaws as round 6; the Part A tables stay quotable at the old mount; the search's cup hold is round 6's fallback hold (no base-side hold is measured).
- Skipped: bowls, plates, re-running the other gates at the new mount, the no-path check, the stack upgrade.
