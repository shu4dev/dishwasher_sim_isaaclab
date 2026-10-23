# Hardware limits, mouth-up (UR5e + 2F-85): run note, 2026-10-23
Plan: plans/2026-10-23-hardware-limits-mouth-up.md. Page: `data/media/robot/limits_v1/index.html` (11 Warp mp4s plus strips; build with `python3 code/execution/frigidaire/frigidaire_robot_limits_page.py --notes data/media/robot/limits_v1/notes.json`). Records: `data/results/robot/limits_v1/`. Logs: `data/logs/robot/`.
- **Works when: met.** All three objects have a grasp-gate table and videos, the reach table covers all three, and the box is STOPPED.
- **Grasp gate (G3, n = 10, upper rack in).** Run: `code/util/run_kit.sh code/execution/frigidaire/frigidaire_robot_episode.py --headless --no-media --grasp-gate <kind> --n 10 --seed 0 [--waive-dish-clearance]`. Bowl 10/10, every bowl pivoting 21° onto the knuckles. Cup 9/10 (1 miss). Plate 3/10, and only with the planner's 15 mm clearance waived: with the clearance, no plate approach exists. The carried-contact invariant (D4) fires in all 20 bowl and cup trials; 9/10 plate trials are clean.
- **Reach sweep (Kit-free, 12 yaws x 50 sites).** Run: `code/util/run_py.sh code/execution/frigidaire/frigidaire_robot_reach.py --workers 2` (plus `--kinds plate --targets family --waive-counter-grasp`). Packer poses: 0/50 sites pass G1. The bowl and cup families are mouth-down, so the pre-place collides in every yaw; plates fail at the counter grasp and, waived, at the pre-place. Mouth-up level at the site: G2 at 15/19 bowl and 8/12 cup sites.
- **Placement (G4, Isaac, level target, n = 3; flags `--place-slot <kind> --target level --legacy-tolerances`).** Bowls 21/46. All 13 upper-slot trials that released (slots 01, 02, 03, 07) pass, and lower rear x −0.12 and −0.07 pass 3/3. Front-right and rear-right sites are 0/12: the bowl tilts 21–35° on the tines. Isaac blocked 3 of the 15 bowl G2 sites. Cups 0/24: they are released and stay in the rack, but drop about 65 mm below the release. Isaac also rejects all 5 Kit-free rejections it was given.
- **Deviations.**
  - Counter spot (0.20, −0.16).
  - The approach hover is 0.10 m with a separate 0.15 m lift. With a 0.15 m hover, Isaac found no bowl approach while the Kit-free model found 12/12; this disagreement is unexplained.
  - The plate runs with an unchecked approach.
  - The box container has no OMPL, so a numpy RRT-Connect (`collide.rrt_connect_np`, shortcuts time-bounded) stands in.
  - Two placement runs stalled at their first upper-rack trial; they were killed and rerun as `place_bowl_upper_*`. Round 2 found the cause: the arm blew up during the trial reset's rack ramp (stale PD targets after the teleport). The RRT was not the cause.
  - Placement uses R7 0.08 rad lag tolerances.
  - The level target was added because no family pose reaches G1.
  - The mount file was rewritten from robot.md's numbers, and the USD was converted with a uv venv and `LD_LIBRARY_PATH`.
  - Superseded runs are kept as `gate_*`, `_strict_*` and `_r7a_*`.
- **Missing.** n = 10 placement trials; placement into the packer's poses (needs a regrasp or flip); the plate slide primitive; the mount search; a loaded rack; a check that the knuckle contact is real; a lowered-until-contact release for cups.
- **Files.**
  - Changed: `frigidaire_robot_episode.py` (+257/−11, grasp-gate and place-slot modes, replay logging), `robot/grasp.py` (+15/−7, cup and plate rim families), `robot/collide.py` (+67/−2, numpy RRT fallback).
  - New: `frigidaire_robot_reach.py` (360 lines), `frigidaire_robot_replay.py` (138), `frigidaire_robot_limits_page.py` (280).
  - Data: `data/results/robot/mount/easy_s0_any.json`.
- **Box window.** dishsim-l4 started 09:44Z, the stop was sent at 11:11Z, and `brev ls` showed it STOPPED at 11:18:34Z. The clone was restored to f2f0246 and is clean.
- **Suggested commit:** `Add robot limits round: grasp gate, reach sweep, Warp replay`

## Round 2 (plan section "Round 2")
- **Box window.** Started 11:26Z, RUNNING at 11:27Z. Three Kit processes plus Kit-free diagnostics on core 3. Kit cutoff 12:44:18Z; stop sent 12:47:50Z, and `brev ls` showed it STOPPED at 12:55:07Z. The clone was restored to f2f0246 and is clean.
- **Cups, lowered until contact** (`--place-slot cup --target level --release down`, the 8 G2 slots, n = 3): **G4 3/24 (95 % CI 0.03–0.32)**, all three on the left slots.
  - Left: released 16–63 mm below the level height. The passes are the cups set down on the channel floor about 62 mm below; arm or palm contact was flagged in 10 of the 12 left trials.
  - Right: the contact detector fired 47–87 mm above the level height in all 10 releases, and every cup dropped (0/12).
- **Bowls released at the level height, pooled over both rounds** (n = 6–9 per site; round 2 ran in `--interleave` laps): 52/99.
  - Upper slots: 01 8/8 (0.63–1.00), 02 9/9 (0.66–1.00), 03 6/6 (0.54–1.00), 07 6/6 (0.54–1.00).
  - Lower rear: x −0.12 9/9 and x −0.07 9/9 (0.66–1.00), x −0.17 3/9 (0.07–0.70), x −0.02 2/8 (0.03–0.65).
  - Front-right x +0.06, +0.08, +0.13: 0/9 each (0.00–0.34). Rear-right: 0/8 (0.00–0.37).
- **Bowls lowered until contact** (front-right and rear-right, n = 3): 2/12. The median tilt fell from 30° to 17°; rear-right stayed at 21–23°. In 3 of the 12 releases the detector fired 49–79 mm above the rack. Every released round-2 dish ended in its intended rack (34/34).
- **Hover diagnostic** (`--diag-hover`, `data/outputs` helpers): with the same IK seed, Isaac's and the Kit-free arm models agree in 36/36 cells, and their calibrations differ by at most 4e-7. The round-1 gap was the IK seed (2π unwrap plus joint limits).
  - Kit-free model, HOME_Q seed: a free approach for 12/12 yaws at 0.10, 0.125 and 0.15 m.
  - Kit-free model, Isaac's joints as seed: 7/12, 0/12, 0/12.
- **Deviations.**
  - The round-1 stalls were an arm blow-up during the trial reset's rack ramp (stale PD targets), not the RRT. Fixed with `rig.step(5)` before the ramp after it hit `r2_place_bowl_level_a` at its first rack switch; that run was killed and relaunched as `_a2`/`_b2`.
  - The down-release G4 is judged against the dish pose at the release.
  - n = 10 was not reached: the 08:44 ET cutoff left 6–9 per site.
- **Files.**
  - `frigidaire_robot_episode.py` now +323/−11 in all: `--release down`, `--diag-hover`, `--interleave`, the reset fix.
  - `frigidaire_robot_limits_page.py` is now 324 lines: round-2 tables with Clopper-Pearson intervals, the diagnostic block.
  - New videos: `cup_down_pass.mp4`, `bowl_down_pass.mp4`.
- **Suggested commit:** `Add limits round 2: contact release, pooled CIs, IK seed check`

## Round 3 (plan section "Round 3")
- **Box window.** Started 14:14:50Z, RUNNING at 14:17Z; driver launched 14:17:43Z; Kit cutoff 15:42:58Z; stop sent 15:46:36Z, and `brev ls` showed it STOPPED at 15:53:30Z (1 h 39 min from start). Three Kit processes plus the Kit-free sweep and the replays on core 3. The clone was restored to f2f0246.
- **Orientation, Kit-free** (`frigidaire_robot_reach.py --orient`): the 23 round-1 G2 sites at tilts 0–150°, 12 yaws each, 1656 jobs in 21 min. Sites with a G2 path:

  | tilt | 0° | 30° | 60° | 90° | 120° | 150° |
  |---|---|---|---|---|---|---|
  | bowls (of 15) | 8 | 10 | 10 | 5 | 0 | 0 |
  | cups (of 8) | 6 | 7 | 5 | 2 | 2 | 0 |
- **Orientation, Isaac** (`--target tilt --release down --down-orient target --contact-lag 0.15`, n = 5 per tilt):
  - Bowl upper slot 02: 0° 5/5, 30° 3/5, 60° 0/5, 90° 0/5.
  - Bowl rear-right: 0° 0/5, 30° 3/5.
  - Bowl front-right x +0.08: 0° 0/5, 30° 0/5, 60° 0/3 (cut at the cutoff).
  - Bowl rear x −0.07: 60° 0/5 (0° and 30° stopped at Kit-free G1).
  - Cup glass right 1: 0° 4/5, 30° 0/5. Cup glass left 1: 0° 0/5, 30° 0/5.
  - Largest tilt placed: bowl 30° (upper slots, rear-right), cup 0° (glass right).
- **Cup detector: one cause fixed.** The round-2 right-slot triggers were the lag rule (hand-to-cup motion under 1 mm, lag 0.050–0.054 rad). With the lag threshold at 0.15 rad the right slots give G4 1/12: cups are released 36–44 mm below the level height and still drop 20–32 mm.
- **Plate surface slide** (`--grasp-gate plate --plate-slide`, n = 25, clearance on): 0/25, all misses. The TCP ends 77–79 mm above the counter, the plate is pushed 50–53 mm, and the jaws close on nothing.
- **Deviations.**
  - The first sweep used the measured in-hand pose on the counter lift line and a clear-height search from below. Every cell failed, so it was rerun fixed (`orient_bad.*` kept).
  - Isaac ran representative sites per class, not every G2 cell.
  - The site stop rule changed mid-run from "stop at the first failure" to "run every G2 tilt". The skipped tilts were rerun as `*2`/`*1b`.
  - The slide's 15 mm clearance applies to the arm links and the hover; the gripper may touch the counter and the plate on the descent and slide.
  - The Isaac tilt runs use the sweep's first G2 yaw.
- **Videos.**
  - Failure modes: `fm_bowl_side_tilt`, `fm_bowl_rear_marginal`, `fm_bowl_tilt_limit`, `fm_cup_early_contact`, `fm_cup_level_drop`, `fm_cup_tilt_limit`, `fm_plate_pinch_slip`, `fm_plate_slide` (.mp4), and the stills `fm_bowl_packer_preplace.png`, `fm_cup_packer_preplace.png`.
  - Passes: `bowl_tilt_pass.mp4`, `cup_tilt_pass.mp4`, `cup_right_fixed.mp4`.
  - The page folder is 102 MB in 26 media files, so it needs two publish batches.
- **Files.**
  - `frigidaire_robot_episode.py` (+490/−11 in all): plate slide pick, `--target tilt` with `--tilts`/`--orient`, `--down-orient`, `--contact-lag`, the per-trial function.
  - `frigidaire_robot_reach.py` (590 lines): `--orient` sweep.
  - `frigidaire_robot_replay.py` (209): `--still`.
  - `frigidaire_robot_limits_page.py` (443): orientation table, failure gallery, round-3 rows.
- **Suggested commit:** `Add limits round 3: tilt sweep, cup detector fix, plate slide`
