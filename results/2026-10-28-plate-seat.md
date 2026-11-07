# Plates seated along the tine axis with a sensed stop (Part B round 12, posture-fix mount): run note, 2026-10-28
Plan: plans/2026-10-28-plate-seat.md.
- **Records.** `data/results/robot/placing_v1/{c1_easy_plates_I4, c1_easy_plates_I4b, c1_easy_plates_I4c}/` (three Kit runs, see Commands) and 9 "I4" rows in `easy_plates/dishes.jsonl` (now 69 lines). The page and the replay read the last row per plate: 08 / 05 / 01 / 04 from I4, 03 from I4b, 07 from I4c. Plates 06 and 02 have no row.
- **Media.** `data/media/robot/placing_v1/{insert_easy_plates_4.mp4, landed_easy_plates_insert_4.png}` (60 frames, 0.12 MB) and 2 new `videos.json` entries; the goal still is reused.
- **Page.** Version 21, built into `data/media/robot/report_v1`, not published. Round 10's 2 plate copies moved to `data/media/robot/report_v1_v20_dropped/`; its MANIFEST.txt lists the paths to null on publish.
- **Kit-free copies.** `data/results/robot/kitfree_v1/{insert_plate, insert_bowl, plate_seat_smoke}/` (README lines added).
- **Logs.** `data/logs/robot/r12_*`, `r12b_*`, `r12c_*`: the chains, the Kit logs, the replay and the Mac end drivers.

## Headline vs Works-when
**No plate was lowered: none of the 9 I4 rows reached a descent start, so the sensor, the release, the retreat and the seated test never ran in Isaac. The rear bank had no IK at any descent start, as planned. Plates 07 and 03 failed in the carry from the arm's home branch to above the descent start. Plates 06 and 02 never ran: the window's cap came first.**
- **Works-when.** Failed.
  - 6 of 8 plates have I4 rows (06 and 02 missing). Front plates seated: 0/4, none lowered. Seat predicted vs measured: none measured.
  - One replay and one still exist (60 frames, every plate no reach). Version 21 is built.
- **Carry failures** (from the home branch [-173.7, -160.6, 86.8, -106.2, 26.8, 0]°).
  - Run 1, plate_03: leg 1's RRT path ran to its end, then the 0.08 rad legacy settle check failed it. Wrist 1 lagged 0.105 rad at waypoint [303, 303].
  - Run 1, plate_07: the first try moved the arm (travel up to 287° on wrist 1). The other five failed "lifting to the safe height: IK branch jump on the line". The first try's reason was not kept (run 1 recorded only the last why).
  - Run 2 (completed legs pass below the 0.15 rad blocked limit): RRT-Connect found no path to above the start on 6 of 6 tries for both 07 and 03 (15 s budget).
  - Run 3: the same for 07 at a 45 s budget, 6 of 6.
- **Root cause, Kit-free after the window** (`kitfree_v1/plate_seat_smoke/carry_diag*.json`, `leg2.log`).
  - The reset takes the free home branch with the smallest max |q| (round 10's plate rule): 173.7° here, vs 180° for the other two free branches.
  - With the base-face plate, the held plate meets wrist 1 at waypoint 3-5 of the straight joint path from that branch. RRT-Connect (numpy; the container has no OMPL) finds no path in 15 s or 45 s, on the box and on the Mac.
  - From the other two free branches, leg 1 to the start's z_safe point plans easily:
    - [-173.7, -130.0, 37.4, 92.6, -26.8, 180]: a collision-free straight joint path.
    - Round 10's plate start [-42.8, -85.8, 37.4, 48.4, 104.0, 180.0]: an RRT path in 1.6-2.1 s.
  - Leg 2 in joint space (z_safe down to the +50 mm start) is the second obstacle:
    - The continuous start branch (wrist 3 at -122°): the straight path hits held vs wrist 1; RRT finds nothing in 15 s.
    - The other branch (wrist 3 at 58°): RRT finds a path in 14.2 s, but its descent breaks 58 mm down (a 180° wrist flip), so the run's continuity check rightly dropped it.
  - The Cartesian line along the tine axis, from the z_safe height (783 mm up the axis) down to the start, is collision-free on the arm model for both branches.
- **Next smallest step.** Carry to the top of the tine-axis line at z_safe from the home branch whose leg 1 plans, then go straight down the tine axis to the start and on into the guarded descent.
  - The plan's "carry as round 10 (nearest home branch, straight legs first)" meant the bowls' `carry_search`. I kept the plates' home rule.
  - `carry_search` alone would not have fixed this: no combo is straight, and its fallback picks the same branch. A version with a Kit-free RRT check per leg also found no combo whose legs both plan in 8 s (`smoke_home.json`).
- **Pin fallback in every run.** The fixed joint drifted 5.0 / 4.9 / 4.9 mm in its 12-tick check at each run's first reset (plates 08, 07, 07), so every plate ran on the pin fallback (written to the TCP each step), as round 10's plates 03, 06 and 02 did. The held plate touching wrist 1 is the likely cause (run 1 logged `carried_contact` plate_07 vs wrist_1_link).

| plate | slot | row | Kit-free seat (along the tine axis; supports) | hand at the seat, closed / open jaws (mm, in the run's scene) | outcome | why |
|---|---|---|---|---|---|---|
| plate_08 | lower_rear_06 | I4 | +15 mm (r4c7, r5c7) | 76.1 (outer finger vs upper rack) / 71.4 | no reach (no IK) | no free IK continuous to the bound at any start +50..150 mm; wrist 2 vs upper rack 8/8, wrist 1 vs upper rack 7/8 |
| plate_05 | lower_rear_07 | I4 | +15 mm (r4c8, r5c8) | 79.2 (palm vs upper rack) / 71.1 | no reach (no IK) | wrist 1 vs upper rack 9/9, wrist 2 vs upper rack 9/9 |
| plate_01 | lower_rear_08 | I4 | +15 mm (r4c9, r5c9) | 75.8 / 70.5 | no reach (no IK) | wrist 2 vs upper rack 10/10, wrist 1 vs upper rack 9/10 |
| plate_04 | lower_rear_04 | I4 | +15 mm (r4c5, r5c5) | 76.1 / 72.1 | no reach (no IK) | wrist 2 vs upper rack 5/5, wrist 1 4/5; no IK at the seat itself |
| plate_07 | lower_front_04 | I4c | 0 mm (rail 1, r0c5, r1c5) | 81.3 (outer finger vs lower rack) / 59.8 | no reach (no path) | 6 starts +50..+75 mm, each "no path above the target (RRT none)" at 45 s |
| plate_03 | lower_front_03 | I4b | 0 mm (rail 1, r0c4, r1c4) | 81.3 / 59.7 (plate_07 parked, not landed) | no reach (no path) | 6 starts, each RRT none at 15 s (I4: a completed leg failed at 0.105 rad) |
| plate_06 | lower_front_02 | – | – | – | not run | run stopped at the cap |
| plate_02 | lower_front_00 | – | – | – | not run | run stopped at the cap |

## Kit-free smoke (Mac, before the box)
- **Depth rule.** It had no solution in {6..14} mm; the user chose option B: rim 6 mm for every plate, all four front plates lowered, a stop on the pad recorded as kind "hand".
  - Source: `insert_plate/depth5.json`, from the coordinator's Kit-free agent.
- **The wedge.** At the goals, plate_03's rim top touches plate_07's underside (-0.23 mm, `task1.json`), and plate_06's rim top touches plate_03's (+0.02 mm).
  - The base-face hold's inner pad sits in that wedge, so it touches at every depth at which the pads still reach the rim.
  - Closed pad length on the plate (left / right, mm within 1 mm of the plate): 3 / 10 at depths 19..2, 1 / 7 at -6, 0 / 3 at -10, 0 / 0 at -14.
- **Depth table.** Goal scene, tine-axis seat, base face; FCL distance of the closed / open jaws in mm, with the nearest body.

| plate | rim 19 mm (rounds 8-10) | rim 6 mm (picked) |
|---|---|---|
| plate_07 | 75.3 (lower rack) / 54.7 | 81.3 (lower rack) / 59.8 |
| plate_03 | 0.0 (plate_07) / 0.0 | 0.0 (plate_07) / 0.0 |
| plate_06 | 0.0 (plate_07) / 0.0 | 0.0 (plate_03) / 0.0 |
| plate_02 | 0.0 (plate_06) / 5.8 | 1.7 (plate_06) / 18.8 |

- **Seat and hand smoke** (`plate_seat_smoke/smoke_seat_d6.json`, the episode's own functions, goal scene, 15 s for 8 plates).
  - Every front plate seats at its goal (0 mm along the tine axis); the next millimetre overlaps 1.3-1.5 mm.
    - plate_07: rail 0 + r0c5 / r1c5, as the scratch found.
    - plate_03: plate_07, rails 0 / 1, r0c4 / r1c4.
    - plate_06: plate_03, rail 0, r0c3 / r1c3.
    - plate_02: long wire 02, r0c1 / r1c1.
  - Predicted hand stop (the closed jaws' first touch on the way down):
    - plate_07: none down to the bound.
    - plate_03: +11 mm (right finger pad vs plate_07).
    - plate_06: +11 mm (right finger pad vs plate_03).
    - plate_02: -3 mm, below its seat, so its stop would be the plate's.
  - Descent start: +50 mm with one free IK continuous to the bound for each front plate; the rear bank has none. The seated test passes at the predicted seats.

## Commands (box, in `dishsim-isaac`; `$P` = /workspace/dishsim/data/results/robot/placing_v1)
1. **Run 1** (cores 0/4, `r12_chain.sh`, 01:45:25Z): `run_kit.sh code/execution/frigidaire/frigidaire_robot_episode.py --headless --no-media --mount data/results/robot/mount/posture_v1.json --shortcut-s 10 --legacy-tolerances --max-wall-seconds 9000 --out-root $P --place-goal $P/easy_plates/load.json --variant I --insert seat --rim-depth-m 0.006 --row-variant I4 --run-id c1_easy_plates_I4`
   - I stopped it at 01:56Z, 6 rows in (07 and 03 had failed in the carry).
2. **Run 2** (`r12b_chain.sh`, 01:57:17Z): the same plus `--dishes plate_07,plate_03,plate_06,plate_02 --run-id c1_easy_plates_I4b`, with the carry-settle change below. Stopped at 02:03:43Z, 2 rows in.
3. **Run 3** (`r12c_chain.sh`, 02:03:56Z): `--dishes plate_07,plate_03 --transit-rrt-s 45 --run-id c1_easy_plates_I4c`. Stopped at 02:10:55Z, 1 row in.
   - None of the three runs printed a `[RESULT]` line (all stopped by hand). No log has a traceback.
4. **Replay:** `run_py.sh code/execution/frigidaire/frigidaire_robot_replay.py --load-dir data/results/robot/placing_v1/easy_plates --out data/media/robot/placing_v1 --variant I4`
   - Result: `[RESULT] PASS drop replay easy_plates I4: insert_easy_plates_4.mp4 (60 frames, 6 dishes), landed_easy_plates_insert_4.png; seated 0/6, settled 0/6, in slot 0/6`.
5. **Page** (Mac): `/opt/anaconda3/bin/python3 code/execution/frigidaire/frigidaire_robot_report_page.py --records data/results/robot/limits_v1 data/results/robot/exhaustive_w1 data/results/robot/exhaustive_w2 --media data/media/robot/limits_v1 data/media/robot/exhaustive_w1 data/media/robot/exhaustive_w2 --out data/media/robot/report_v1`
   - Result: `[RESULT] PASS 7 sections 16 media files 44828196 bytes` (index.html 31.0 KB; media/ holds exactly the 16 referenced files).

## Deviations
- **Seat rule.** The seat is the last 1 mm height before the overlap exceeds 1 mm (the scratch's `lower_fast`), not the first height with two supports.
  - The far column's two tips are two distinct wires, so "two supports" could stop on them above the rail or floor wire, where the plate can still slide down.
  - The supports at the seat are recorded: 3-5 per front plate.
- **Overlap depth.** Per contact: FCL convex piece vs wire capsule (`depth_of`, as the scratch's `signed`); touching = FCL distance ≤ 0.
- **Descent start.** The lowest of 50, 55, .. 150 mm along the tine axis that has a collision-free IK continuous to the bound (1 mm steps, 0.1 rad). Every front plate got +50.
- **Home branch.** Round 10's plate rule (smallest max |q|), not `carry_search`. This was my reading of "the carry as round 10", and it was wrong; see the headline.
- **Carry settle (runs 2 and 3).** A completed carry leg fails only above `rig.lag_max` (0.15 rad), following the standing rule that no gate decides placing. `row.carry_settle_tol_rad` records it.
- **Run budget.** RRT budget 45 s in run 3 (15 s in runs 1 and 2). Plates 06 and 02 were dropped from run 3 to fit the cap.
- **Stopping runs.** Each run was stopped by hand: the Mac driver first, then the box chain, then the Kit process.
- **Rows.** The earlier runs' rows for 07 and 03 stay in `dishes.jsonl`.
- **Code state (window 1).** At window 1's end the repo's episode file was byte-identical (md5 d95c374c) to the code of runs 2 and 3 (window 2's code, md5 50822936, has replaced it); run 1 ran it without the carry-settle change and the per-try `carry_fails` list. A post-window variant with the home choice in reset() is kept as `kitfree_v1/plate_seat_smoke/episode_with_seat_plan.py`, not in the repo.
- **Page.** Tables (e) and (f) sit in the Kit-free block of the bowl section, where that block lives. Plates 06 and 02 read "not run".
- **Stop kind.** A trip with both the plate's and the tool's pairs is labelled "plate+hand". No trip happened in this round.

## Box window and files
- **dishsim-l4.**
  - Start attempt 1 at 01:43:15Z; STARTING at 01:43:30Z; RUNNING at 01:44:29Z.
  - ssh at 01:44:48Z after `brev refresh`; self-stop running (PID 627); container Up; 22 GB free.
  - All 141 `code/*.py` were md5-identical after the rsync; the episode was re-pushed and re-checked at 01:57Z. The 8 data inputs matched by md5.
  - Runs: see Commands. Replay 02:11:00Z; copy-back 02:11:11-17Z; stop sent 02:11:20Z; STOPPED at 02:21:48Z.
  - **About 38.5 billed minutes**, start to STOPPED (cap 45).
  - `brev ls` at 02:21:57Z: `dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`.
- **Changed files.**
  - `frigidaire_robot_episode.py`:
    - Flags `--insert seat` and `--rim-depth-m`.
    - Constants `RIM_DEPTH_TODAY_M`, `SEAT_*`, `SEATED_TOL_M`, `SEAT_TOUCH_M`, `RACK_NAMES_JSON`, `PLATE_GATE`.
    - Helpers `rack_labels`, `closed_hulls_named`, `tine_axis`, `depth_of`, `rack_aabbs`, `dish_contacts`, `seat_search`, `hull_objs`, `hand_touch`, `seated_test`.
    - `lower_until_contact`: descends along `cb["dir"]`; plate / hand / plate+hand stop kinds; `dish_h_mm`.
    - Closures `seat_lib`, `settle_landed`, `insert_seat`; the I4 hold (base face, TCP moved back 13 mm); the I4 row fields; stop counts in the summary.
  - `frigidaire_robot_replay.py`: `--variant I4` (`insert_<load>_4.mp4`, `landed_<load>_insert_4.png`) and its caption.
  - `frigidaire_robot_report_page.py`: Version 21.
    - `ROUNDS` gets plate-seat on top; `TABLE_ONLY` gains I3.
    - The I4 result and note; `seat_table`, `wire_short`, `kitfree_ef` (tables (e) and (f)); `starts_of` reads I4.
  - New: `data/logs/robot/r12{,b,c}_{chain,driver}.sh`; `data/results/robot/kitfree_v1/{insert_plate,insert_bowl,plate_seat_smoke}/`.
- **Suggested commit:** `Add tine-axis plate seat rows with a sensed stop`

## Window 2 (2026-10-29, the mirrored hold, the carry by leg 1; shared box window with plans/2026-10-29-bowl-last.md)
- **Records.** `data/results/robot/placing_v1/c1_easy_plates_I4d/` and 4 new "I4" rows in `easy_plates/dishes.jsonl` (now 73 lines). The page and the replay read the last row per plate: the front four from I4d, the rear four from window 1's I4. Media: `insert_easy_plates_4.mp4` (1119 frames, re-encoded on the Mac at CRF 30: 19.4 to 6.2 MB) and `landed_easy_plates_insert_4.png`. Logs `data/logs/robot/r13_*`.

### Headline vs Works-when
**All four front plates ended seated on their goal's wires and settled (4/4). plate_07 stopped on the plate (3.7 N on the lower rack), 1.7 mm from its predicted seat. Plates 03, 06 and 02 stopped on the hand against the front neighbour, 31.7-34.9 mm above their seats, 6.0-10.8 mm higher than the Kit-free hand stop; let go there, each slid down the tines into its seat. The fixed joint held on every plate (no pin fallback).**
- **Works-when.** Met for the seats (4/4 on the goal's wires) and the rows. The 2 mm check holds on the one plate stop (1.7 mm); the three hand stops miss their prediction by 6.0-10.8 mm.
- **Carry.** Every plate started on home branch H1 [-42.8, -85.8, 37.4, 48.4, 104.0, 180.0] (the only free branch with the mirrored hold). Leg 1 was a straight joint move to the top of the tine-axis line at the TCP height 1.25 m; the line down the axis was free for its full 773-794 mm on all four. Descent starts: +50 mm (07), +70 mm (03, 06, 02).
- **Descent.** 20.0 mm/s on all four; weld drift 0.10-0.15 mm at the stop. The first sensed contact came 1-16 ticks before the trip.
- **Retreat.** plate_07 backed off the full 100 mm and rose. Plates 03, 06 and 02: the checked back-off line stopped at 0 mm (the open gripper near plate_07 / plate_03 / plate_06), the rim test passed (26-28 mm), and the hand rose. No earlier plate moved.

| plate | start | seat pred / at the stop (mm) | hand stop pred | stop: kind, pair, force | settled: lateral / height / tilt, lean (goal) | wires touched (settled = goal's set?) | retreat |
|---|---|---|---|---|---|---|---|
| plate_07 | +50 | 0.0 / **1.7** | none (closed jaws 70.7 mm from the basket at the seat) | plate: lower rack vs plate_07, 3.72 N, 288 ticks | 0.1 / 0.0 mm / 0.0°, -32.2° (-32.2°) | rails 0+1, r0c4, r0c5, r1c4, r1c5, cross 04+05: the goal's set | back 100.1 mm, rose |
| plate_03 | +70 | 0.0 / **31.7** | 20.9 (+10.8) | hand: pad_L vs plate_07, 9.78 N, 223 ticks | 0.0 / 0.0 mm / 0.0°, -32.2° (-32.2°) | rails 0+1, r0c3, r0c4, r1c3, r1c4, cross 04+05: the goal's set | back 0 mm (gripper vs plate_07), rim 26.2 mm, rose |
| plate_06 | +70 | 0.0 / **32.8** | 24.9 (+7.9) | hand: left outer finger vs plate_07, 13.52 N, 216 ticks | 0.4 / 0.1 mm / 0.1°, -32.0° (-32.2°) | rails 0+1, long 04, r0c2, r0c3, r1c2, r1c3, cross 04+05 | back 0 mm (gripper vs plates 03, 07), rim 27.2 mm, rose |
| plate_02 | +70 | 0.0 / **34.9** | 28.9 (+6.0) | hand: left outer finger vs plate_06, 13.64 N, 202 ticks | 1.8 / 0.7 mm / 4.3°, -29.7° (-34.0°) | long 02, rails 0+1, r0c1, r1c1, cross 04+05 | back 0 mm (gripper vs plates 03, 06), rim 28.1 mm, rose |

### Kit-free smoke and dry run (Mac, before billing)
- **Hold.** The coded base-face hold put the inner finger through the held plate (open jaws -11.8 to -13.3 mm at 6 mm). The mirrored hold (the mouth hold reflected through the rim wall, `insert_plate/hold_base/hold_base.json`, the coordinator's pick) clears it by 27-30 mm with the open jaws; its closed-jaw overlap is an artefact of the gate pose, so closed jaws were read against the scene only.
- **Depth rule.** No solution again with the mirrored hold: the closed pads touch the front neighbour at the seat for 03, 06 and 02 at every depth; 6 mm, option B. The rim-offset study (`hold_base/offset/`, 160 rows, 30-90° both sides, both holds, 6 and 10 mm) found no pinch that clears the neighbour.
- **Dry run.** `kitfree_v1/dryrun/fakekit.py` (the real IK and arm model; the arm tracks exactly, a welded dish follows the FixedJoint prim, the sensor from FCL): four plates, every path ran (49 s). It caught one bug: the first plate's seat plan ran before the racks moved.

### Window 2 deviations
- **Descent-start line.** The line up the tine axis is walked in 5 mm steps from the descent start and ends at the last IK-continuous (0.3 rad per step), free waypoint (at least 100 mm); in this run it reached the TCP height 1.25 m on all four.
- **Leg 1.** Planned Kit-free in reset() (a straight joint move first, else RRT-Connect up to 45 s) and the planned path executed as is; transit_to is not used for the plates. Its home-branch rule: straight first, then the smallest largest-joint move.
- **Front four only.** `--dishes plate_07,plate_03,plate_06,plate_02`; the rear four keep window 1's rows.
- **Carry settle.** As in window 1's runs 2-3: a completed leg fails only above 0.15 rad.

### Window 2 commands (box; `$P` = /workspace/dishsim/data/results/robot/placing_v1; chain `r13_chain.sh`, cores 0/4)
- `run_kit.sh code/execution/frigidaire/frigidaire_robot_episode.py --headless --no-media --mount data/results/robot/mount/posture_v1.json --shortcut-s 10 --legacy-tolerances --max-wall-seconds 9000 --out-root $P --place-goal $P/easy_plates/load.json --variant I --insert seat --rim-depth-m 0.006 --plate-hold data/results/robot/kitfree_v1/insert_plate/hold_base/hold_base.json --plate-hold-kind mirror --row-variant I4 --dishes plate_07,plate_03,plate_06,plate_02 --run-id c1_easy_plates_I4d`
  - `[RESULT] PASS place-goal easy_plates I4: seated and settled 4/4, settled any 4, failed at {}, stops {'hand': 3, 'plate': 1}, within 2 mm 1, attach {'pin': False, 'why': None}`; 12.5 min; no tracebacks.
- Replay: `[RESULT] PASS drop replay easy_plates I4: insert_easy_plates_4.mp4 (1119 frames, 8 dishes), landed_easy_plates_insert_4.png; seated 4/8, settled 4/8, in slot 4/8`.
- Box window, page and files: see results/2026-10-29-bowl-last.md (one shared window).
