# EPick places s0 from random counter spawns, runs D and U: run note, 2026-11-10 (corallab4)
Plan: plans/2026-11-10-suction-random-spawn.md. Code at f905ebd on plan; Isaac Sim 6.2 in the dishsim-isaac container on corallab4.

The Kit-free check gives 5 of 15 dishes a hold in run D (cup_01, bowl_05, cup_05, cup_04, cup_03, all by the base) and 2 of 15 in run U (bowl_04 and bowl_01, by the outer side wall). Isaac placed all 7: run D 5 of 5 attempted and run U 2 of 2, each picked, carried, released, at rest, at goal and mouth down, at most 0.3 mm lateral, 0.0 mm vertical and 0.3 deg from the stored goal. No plate has a hold in either run, so run D has no plate at goal, and that Works-when item fails. The other items hold: check.json has a verdict and a cause for all 45 holds, every Isaac process of the final runs logged `[RESULT] PASS` with no traceback, no `free(): invalid pointer` and no `malloc(): corrupted top size`, and run U's 13 dishes without a hold are listed for the flip below. Two mp4s exist instead of four, because no lower-rack dish has a hold in either run, so the lower chains started no Kit process.

Commands, all on corallab4 after `git pull --ff-only origin plan` (the s0 used is a copy of the Mac's, data/results/robot/suction_s0/s0.json):
- `taskset -c 1,9 nice -n 10 code/util/run_kit.sh code/sampler/robot/scripts/frigidaire_arm_model.py --headless` (PASS: one hull per arm link plus the EPick body, X_link the identity).
- `code/util/run_py.sh code/sampler/robot/scripts/frigidaire_suction_check.py --sample data/results/robot/suction_s0/s0.json --mount data/results/robot/mount/posture_v2.json --out data/results/robot/suction_s0` (32 s).
- `taskset -c <j>,<j+8> nice -n 10 code/util/run_kit.sh code/sampler/robot/scripts/frigidaire_suction_episode.py --headless --enable_cameras --check data/results/robot/suction_s0/check.json --run D --chain upper --video`, then `--run U --chain upper`, `--run D --chain lower` and `--run U --chain lower`; the smoke added `--only cup_01` and placed cup_01 in 60 s.

The check, per rack chain in the dag order, with the chain's earlier dishes at their goals, the dish's rack out, the other pushed in and the basket out. The base hold puts the cup on the outer bottom centre with the cup axis along the dish axis. The side hold puts it on the outer wall 60 mm (bowl) or 65 mm (cup) above the base, along the inward wall normal, at 12 azimuths; the table names the first passing azimuth, or the failing one that got furthest, and the row table names the one the run used (it also had to pick at the spawn). The inside hold puts it on the inner floor centre. "gripper" in a cause is the EPick body or its cup.

| dish | slot | base (run D) | side wall (run U) | inside (run U) |
|---|---|---|---|---|
| cup_01 | glass_right_2 | pass, straight down from 100 mm | fail, at the release, gripper vs UpperRack (side@0) | fail, the EPick body does not fit inside |
| bowl_05 | upper_bowl_01 | pass, straight down from 100 mm | fail, at the release, wrist_2_link vs UpperRack (side@0) | fail, at the release, forearm_link vs UpperRack |
| bowl_04 | upper_bowl_11 | fail, at the release, forearm_link vs Counter | pass, straight down from 100 mm (side@60) | fail, at the release, forearm_link vs UpperRack |
| cup_05 | glass_left_1 | pass, straight down from 100 mm | fail, at the release, forearm_link vs UpperRack (side@0) | fail, the EPick body does not fit inside |
| bowl_03 | upper_bowl_03 | fail, at the release, gripper vs UpperRack | fail, straight down from 100 mm: held vs UpperRack 69 mm before the release (side@30) | fail, at the release, forearm_link vs UpperRack |
| cup_04 | glass_left_2 | pass, straight down from 50 mm | fail, at the release, gripper vs UpperRack (side@0) | fail, the EPick body does not fit inside |
| cup_03 | glass_right_3 | pass, straight down from 100 mm | fail, at the release, gripper vs UpperRack (side@0) | fail, the EPick body does not fit inside |
| bowl_01 | upper_bowl_07 | fail, at the release, wrist_2_link vs Cabinet | pass, straight down from 100 mm (side@60) | fail, at the release, forearm_link vs UpperRack |
| cup_02 | glass_left_3 | fail, down the rim plane from 100 mm: held vs UpperRack 82 mm before the release | fail, at the release, wrist_2_link vs UpperRack (side@0) | fail, the EPick body does not fit inside |
| plate_01 | lower_rear_05 | fail, no IK at the release | fail, a plate has no side wall | fail, no IK at the release |
| bowl_02 | lower_frontright | fail, straight down from 100 mm: held vs LowerRack 62 mm before the release | fail, no IK at the release (side@0) | fail, no IK at the release |
| plate_02 | lower_front_07 | fail, straight down from 100 mm: held vs LowerRack 67 mm before the release | fail, a plate has no side wall | fail, no IK at the release |
| plate_05 | lower_front_04 | fail, straight down from 100 mm: held vs LowerRack 67 mm before the release | fail, a plate has no side wall | fail, no IK at the release |
| plate_03 | lower_front_00 | fail, no IK at the release | fail, a plate has no side wall | fail, no IK at the release |
| plate_04 | lower_rear_00 | fail, no IK at the release | fail, a plate has no side wall | fail, no IK at the release |

The plates fail on reach or on the way in. plate_01, plate_03 and plate_04 have no elbow-up IK solution at the release with either hold. plate_02 and plate_05 lean 32 deg from vertical against a tine at their goals: straight down they hit the LowerRack 67 mm before the release, and down their own plane from 150 mm they hit bowl_02 119 mm before the release (plate_02) or the LowerRack 96 mm before it (plate_05); from 100 mm that start is already within 5 mm of the rack.

Spawns: seed 0, per dish (x, y, yaw) uniform on the counter as round 3 pulled it (x -0.9 to 0.9 m, y -0.17 to 0.30 m), with the dish's radius plus 5 mm kept inside every edge. Every dish evaluates 200 draws; its spawn is the first draw where each run with a hold can pick. The share of the counter the arm can pick from, as passing draws: run D bowls 24 of 200 (12 %), run D cups 48 of 800 (6 %), run U bowls 18 of 400 (4.5 %). The spawns were accepted at draws 1 to 47, and all seven lie at x 0.13 to 0.75 m and y -0.12 to -0.06 m, within 0.12 m of the counter's front edge. Dishes without a hold get no spawn.

One row per dish and run (pose error = lateral, vertical and tilt from the stored goal, in the rack's current frame):

| dish | run | spawn x, y (m), yaw (deg) | hold | picked | carried | released | at rest | at goal | pose error (mm, mm, deg) |
|---|---|---|---|---|---|---|---|---|---|
| cup_01 | D | 0.614, -0.116, 263 | base | yes | yes | yes | yes | yes | 0.1, 0.0, 0.1 |
| cup_01 | U | - | none | stays out (no hold, see the check table) | | | | | |
| bowl_05 | D | 0.751, -0.074, 85 | base | yes | yes | yes | yes | yes | 0.0, 0.0, 0.0 |
| bowl_05 | U | - | none | stays out (no hold, see the check table) | | | | | |
| bowl_04 | D | - | none | stays out (no hold, see the check table) | | | | | |
| bowl_04 | U | 0.470, -0.075, 146 | side@90 | yes | yes | yes | yes | yes | 0.3, 0.0, 0.0 |
| cup_05 | D | 0.451, -0.083, 200 | base | yes | yes | yes | yes | yes | 0.1, 0.0, 0.1 |
| cup_05 | U | - | none | stays out (no hold, see the check table) | | | | | |
| bowl_03 | D | - | none | stays out (no hold, see the check table) | | | | | |
| bowl_03 | U | - | none | stays out (no hold, see the check table) | | | | | |
| cup_04 | D | 0.620, -0.090, 93 | base | yes | yes | yes | yes | yes | 0.1, 0.0, 0.3 |
| cup_04 | U | - | none | stays out (no hold, see the check table) | | | | | |
| cup_03 | D | 0.713, -0.098, 245 | base | yes | yes | yes | yes | yes | 0.3, 0.0, 0.1 |
| cup_03 | U | - | none | stays out (no hold, see the check table) | | | | | |
| bowl_01 | D | - | none | stays out (no hold, see the check table) | | | | | |
| bowl_01 | U | 0.127, -0.057, 153 | side@120 | yes | yes | yes | yes | yes | 0.2, 0.0, 0.2 |
| cup_02 | D | - | none | stays out (no hold, see the check table) | | | | | |
| cup_02 | U | - | none | stays out (no hold, see the check table) | | | | | |
| plate_01 | D | - | none | stays out (no hold, see the check table) | | | | | |
| plate_01 | U | - | none | stays out (no hold, see the check table) | | | | | |
| bowl_02 | D | - | none | stays out (no hold, see the check table) | | | | | |
| bowl_02 | U | - | none | stays out (no hold, see the check table) | | | | | |
| plate_02 | D | - | none | stays out (no hold, see the check table) | | | | | |
| plate_02 | U | - | none | stays out (no hold, see the check table) | | | | | |
| plate_05 | D | - | none | stays out (no hold, see the check table) | | | | | |
| plate_05 | U | - | none | stays out (no hold, see the check table) | | | | | |
| plate_03 | D | - | none | stays out (no hold, see the check table) | | | | | |
| plate_03 | U | - | none | stays out (no hold, see the check table) | | | | | |
| plate_04 | D | - | none | stays out (no hold, see the check table) | | | | | |
| plate_04 | U | - | none | stays out (no hold, see the check table) | | | | | |

Counts per kind, placed of 5: run D cups 4, bowls 1, plates 0; run U bowls 2, cups 0, plates 0. Every attempted dish attached on the first close at 0 mm stand-off, 8.3 ms after the close command, rose 98.5 to 99.4 mm on the 100 mm lift, went from home to the pre-pick by a straight joint move and was carried by RRT-Connect (2.6 to 4.7 s of planning), drifting at most 0.34 mm in the cup. At the end of each chain every placed dish is still within AT_GOAL: bowl_04 moved to 0.7 mm and 1.0 deg from its goal during bowl_01's process, and the rest stayed within 0.3 mm and 0.3 deg.

Flip list, run U's dishes without a hold (13): cup_01, bowl_05, cup_05, plate_01, bowl_02, plate_02, plate_05, bowl_03, plate_03, plate_04, cup_04, cup_03, cup_02. cup_02, bowl_02, bowl_03 and the five plates have no hold in run D either.

Timings: the arm model export took under 2 min; the check 32 s on corallab4 (20 s on the Mac); a dish process 60 to 85 s with the Kit boot, of which the episode took 41 to 66 s; the D upper chain 350 s, the U upper chain 170 s, the lower chains under 1 s each. Three Kit boots hung right after `KitLauncher initialization complete` (a zombie python child, the log silent) and were killed by PID inside the container: the arm model's first try (10 min), D upper's first cup_01 (3.5 min) and U upper's second bowl_04 (3 min). The last two came while another of my Kit processes was booting or running in the same container. The first U upper run placed bowl_04 the same way (0.3 mm, 0.0 deg), then its bowl_01 process crashed while writing its row (a Path in the JSON, fixed in f905ebd). Both upper chains were rerun from their start, U after D. Those outputs are under data/results/robot/suction_s0/try1/, data/media/robot/suction_s0/try1/ and data/logs/robot/suction_try1/ and suction_try2/.

Files changed: code/sampler/robot/collide.py (link_hulls reads the tool's colliders from /World/Robot/epick, cylinders included), code/sampler/robot/scripts/frigidaire_arm_model.py (authors the EPick, reads it back, fails on a missing hull), code/sampler/robot/scripts/frigidaire_suction_check.py and frigidaire_suction_episode.py (new). data/results/robot/arm_model.json was exported again on corallab4 and copied to the Mac. Records: data/results/robot/suction_s0/ (check.json, s0.json, D and U per chain: rows.jsonl, row_*.json, after_*.json; smoke/); media: data/media/robot/suction_s0/suction_D_upper.mp4 (81 s) and suction_U_upper.mp4 (51 s), both 1280x960 at 15 fps, the per-dish clips and the release stills under D/upper and U/upper; logs: data/logs/robot/suction_*.

Deviations:
- Scene queries are switched on in the episode's before_reset by setting PhysxSceneAPI enableSceneQuerySupport on the physics scene prim, after Isaac Lab wrote its own value; code/sampler/initial_state_runtime.py is unchanged.
- The plan names no approach direction. The check tries straight down, then down the slope of the dish's rim plane, each from 100, 150 and 50 mm. The release is the goal raised 3 mm, and the retreat backs 50 mm along the cup axis.
- The side hold uses one contact height per kind at 12 azimuths. The FCL tool adds a 40 by 24 mm cylinder for the real cup, which is visual only in Isaac. A hold may back off up to 5 mm along the cup axis until the tool clears its own dish; no passing hold needed it.
- Clearances: 2 mm on every straight line (the held dish by the checker's 1 mm pair rule), 5 mm for the arm and the held dish on the transit legs and at their ends. Home is the cup straight down at (0.45, -0.30, 1.15) m, and the arm is teleported there at each process start, before any physics step.
- The driver kills and reruns a dish process that has not built its scene after 6 min, and U upper ran after D upper rather than beside it.

Missing: a hold for the plates, bowl_02, bowl_03 and cup_02; the flip for the 13 dishes above (next round); the seal test, the mount sweep, the wire tests, s10 and hwgate (skipped by the plan). The judge checks rest, AT_GOAL and mouth down only, with no wire test.
