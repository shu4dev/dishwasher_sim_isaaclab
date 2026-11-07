# Cup side pinch, mouth-down drop (Part A base-side gate, Part B variant D-side): run note, 2026-10-27
Plan: plans/2026-10-27-cup-side-drop.md. Records: `data/results/robot/placing_v1/{partA_cup_side_base45, c1_easy_cups_Dside, side/spot.json}`; 8 variant "D-side" rows in `placing_v1/easy_cups/dishes.jsonl`, which now has 48 lines. Media: `data/media/robot/placing_v1/{drop,landed}_easy_cups_side.{mp4,png}`, plus 2 new `videos.json` entries. Logs: `data/logs/robot/c1_*.log` (`c1_chain.sh` is the box chain).

## Headline
**6 of 8 cups settled mouth down; 6 of 8 settled at all; 2 had no reach.** Four of the six lie within the old S5 tolerance of their goals (15 mm lateral, 20 mm height, 15° tilt). No earlier cup moved. For comparison, the first cup drop (wall pinch, edge at -0.30) settled 7/8, all mouth up.

- **Part A base-side gate: held 0/10.** All 10 trials are no reach: none of the 12 yaws gives a collision-free grasp. The commonest first hits are forearm, wrist_1 and wrist_2 against the counter (62, 55, 34), the same links against the cabinet (17, 27, 22), and the pedestal. Part B therefore used the plan's fallback: the h45 mouth-side median hold (trial 8, pivot 0.04°) rolled 180° about the approach.
- **Spot: y = -0.12 (x 0.20).** The Kit-free sweep found 0/12 yaws at every y from -0.12 to -0.05. At 9-12 yaws per y there is no collision-free hover IK. At the other yaws the approach line puts wrist_2 into the cabinet or the counter. With 0/12 everywhere, the rule (the first y with the most yaws) picks -0.12.
- **Pre-flight (Kit-free, earlier cups at their goals):** a collision-free drop IK exists for 8/8 cups, the lowest at 100 mm (cup_05: 125 mm).

| cup | slot | drop height | landed on | settled, mouth (axis z) | lateral / height mm, tilt ° |
|---|---|---|---|---|---|
| cup_01 | glass_right_4 | 225 mm (6th pose tried) | UpperRack | yes, down (-0.97) | 0.4 / 0.2, 1.4 |
| cup_07 | glass_left_1 | 100 mm | UpperRack | yes, down (-0.98) | 1.5 / 0.2, 3.6 |
| cup_02 | glass_right_3 | 125 mm | UpperRack | yes, down (-0.98) | 7.8 / 0.8, 4.3 |
| cup_06 | glass_right_2 | 100 mm | UpperRack | yes, down (-0.84) | 18.9 / 28.3, 47.4 (arm_or_palm_contact fired) |
| cup_08 | glass_left_2 | no reach: no path to 6 poses, last +125 mm | – | – | – |
| cup_04 | glass_right_1 | 100 mm | UpperRack | yes, down (-0.95) | 3.4 / 0.7, 4.2 |
| cup_05 | glass_left_4 | no reach: no path to 6 poses, last +150 mm | – | – | – |
| cup_03 | glass_left_3 | 100 mm | UpperRack | yes, down (-0.99) | 26.5 / 2.5, 20.8 |

Notes on the table:
- Every settled cup has a drift of 0.0 mm and 0.0° over the settle window.
- Every attachment was a fixed joint with 0.0 mm drift; the jaw read 0.212 rad.
- Both no-path reasons are "no path above the target": the RRT failed at 15 s with nothing blocked.

## Commands (box, in `dishsim-isaac`)
1. Spot sweep: `run_py.sh frigidaire_robot_reach.py --side-spot --workers 6`
2. Gate: `run_kit.sh frigidaire_robot_episode.py --headless --no-media --max-wall-seconds 9000 --out-root /workspace/dishsim/data/results/robot/placing_v1 --grasp-gate cup --grasp-family side --side-below-top 0.045 --side-wrist-side base --n 10 --seed 0 --counter-y=-0.120 --run-id partA_cup_side_base45`
3. Drops: the same script with `--place-goal .../easy_cups/load.json --variant D --hold side-base --run-id c1_easy_cups_Dside`
4. Replay: `run_py.sh frigidaire_robot_replay.py --load-dir data/results/robot/placing_v1/easy_cups --out data/media/robot/placing_v1 --variant D-side`

The counter edge -0.17 is the new default (`--counter-front-y`).

## Deviations
- **Part B hold is the 0/10 fallback,** as the plan's Decisions specify. The page says so in the legend's "Hold:" sentence and in the Part A row's note.
- **Spot defaults.** The gate ran at the spot by flag (`--counter-y=-0.120`). After the run, `LIMITS_COUNTER_XY` and reach's `COUNTER_XY` both default to (0.20, -0.12), because (0.20, -0.16) sits 1 cm behind the new edge.
- **Pre-flight hold.** It is the nominal base-side hold of yaw 1, the best-facing of all 12 yaws, since none passed. spot.json labels it "passing yaw"; the label was fixed in the code after the run.
- **New default edge everywhere.** -0.17 is now the default for every episode and reach mode. To rerun an earlier round, pass `--counter-front-y=-0.30`; `--test-case upright3` needs it too, because its counter spots at y -0.20 now lie in front of the edge. `grasp.hits_counter` now reads `COUNTER_BOX` at call time.
- **Mac-side fixes.** The replay was re-encoded on the Mac (16.1 MB to 5.5 MB, CRF 30, all 1150 frames). `videos.json` was merged on the Mac: the box copy lacked the `partA_cup_side_h11_pass_t00.mp4` entry.
- **Box clone is behind on three changes.** It holds this run's code, but not the post-run default spot or the label fix. Rsync those before the next window. There was no git on the box.

## Box window and files
- **Box (dishsim-l4).**
  - Starts 1-4 sat STOPPED and were killed. Start 5, issued 15:31:59Z, was RUNNING at 15:33:58Z.
  - Chain: start 15:35:03Z, spot 15:35:27Z, gate 15:37:49Z, ALL DONE 15:57:30Z.
  - Copy-back 15:58:34Z; stop sent 15:58:38Z; STOPPED 16:07:15Z (`dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`).
  - **About 35 billed minutes.**
- **Changed files.**
  - `robot/grasp.py`: `pull_counter`, `COUNTER_FRONT_Y`, `hits_counter`.
  - `frigidaire_robot_episode.py`: `--counter-front-y`, `--counter-y`, `--side-wrist-side`, `--hold side-base`, `flip_down`, `drop_tcp`, `DROP_TRIES` 6, the D-side rows with the "mouth up" failure, `LIMITS_COUNTER_XY`.
  - `frigidaire_robot_reach.py`: the pull in `bench()`, `side_pick`, `--side-spot` (spot sweep and pre-flight), `COUNTER_XY`.
  - `frigidaire_robot_replay.py`: `--variant D-side`, and the counter read from the rows.
  - `frigidaire_robot_report_page.py`: `side_drop_block`, the PART_A row, and the -0.30 sentences.
- **Page:** `[RESULT] PASS 8 sections 61 media files 125112254 bytes` (data/media/robot/report_v1, not published).
- **Suggested commit:** `Add cup side-pinch drop with the counter at -0.17`
