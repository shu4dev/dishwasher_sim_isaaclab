# Grasp and place s0: spawn, pick, carry, place in dag order
Slug: 2026-11-04-grasp-place-s0 | From: plans/2026-10-26-placing-goal.md (Part B) with results/2026-10-29-ring-place.md, results/2026-10-27-posture-fix.md, results/2026-10-28-plate-seat.md; the sample: plans/2026-11-03-rear-plate-drop.md

## Goal
On Isaac Sim 6.2 the UR5e with the side-mounted 2F-85 takes s0's 15 dishes one at a time from the counter spot and places each at its goal in the dag order, and every dish gets a row (gate, picked, carried, seated on the goal's wires, at-goal, cause), so the per-kind success rate on this hardware is a measured number.

## Decisions (from grilling)
- The premise is corrected: on this hardware not every dish is graspable and placeable (the closed study: plates have no measured counter pick, the rear bank is unreachable, the seated front plates used a welded hold; bowls 3 of 7 by perch-and-drop, the hand never fits at a seated pose; cups 8 of 8 settled mouth down, 1 of 8 at-goal). A dish the Kit-free gates reject is a row with its cause; Isaac runs the rest. Rejected a feasible-only run and reopening the mount (closed 2026-10-30).
- Spawn: one dish at a time at the study's counter spot (0.20, -0.12) in its pick orientation (plate face up, bowl mouth down, cup mouth up). The placed dishes stay where the robot left them; a failed dish is parked off the scene and the next dish is still attempted, even when its dag prerequisite failed. Rejected the sample's stacked start (unstacking is another study) and the study's convention of teleporting the earlier dishes to their goals.
- Moves, the study's best per kind: bowl = foot-ring hold, carried mouth down, lowered until the rack takes it, let go (D-ring); cup = the measured 45 mm mouth-side pinch from the counter (10 of 10), let go 100 mm above the goal, the mouth-down release gated Kit-free with that hold (the study's D-side-m2 cups were spawned in the hand: its rolled hold has no counter pick, 0 of 12 yaws at every mount, found by the runner 2026-11-04 and decided by the user); plate = rim pinch from the counter, lowered along the tine axis (I4) without the welded hold. Rejected the build's release for every kind (0 of 4 plates in the study).
- Carry: the pads alone hold the dish from the pick to the release (a new `--attach none`); no PhysX joint and no pin. The study's Part B rows fixed the dish to the wrist after the pick (the 2026-10-29 assumption); that assumption is dropped for this round, so a dish that slips in the carry is a failed row with the cause. Rejected keeping the joint (the user wants the real pipeline).
- Seated = the dish rests on the goal pose's rack wires, settled; the at-goal tolerance (AT_GOAL) is a second column. s0 only, dag order, mount posture_v1, the other rack pushed in per dish as in the study. Media: a Warp replay clip and the release and landed stills per attempted dish; a results note, no page this round.
- Cap 2 h from T0 (box RUNNING) to STOPPED: kill at T0 + 108 min, stop at T0 + 116 min. The 6.2 robot smoke runs first (the grip probe: pad contacts, under 5 mm slip over 3 s); a failed smoke ends the window with its cause.

## Change
- code/execution/frigidaire/frigidaire_placing_loads.py: `--sample <file>` writes data/results/robot/samples_v1/<sample>/load.json from the four-block sample (goal.poses, goal.slots, dag.order and edges; all kinds, the pick orientation per kind).
- code/execution/frigidaire/frigidaire_robot_episode.py: `--place-goal` on a mixed-kind load: per dish the kind's move (D-ring, D-side-m2, the I4 descent after a real rim-pinch pick), `--attach none` (the pads alone, a slip past 5 mm of the in-hand pose = dropped in carry), `--dishes` one id, `--state <json>` restoring the previous dish's settled poses with the failed dishes parked, the row appended to rows.jsonl (gate, picked, carried, seated, at_goal, cause, clip paths).
- Kit-free gate before the window (Mac or box): per dish G0 to G2 at the spot and at the release pose on the FCL arm model (frigidaire_robot_reach.py's checks on the load) into the load's rows as the gate column.
- A driver under data/logs/robot_s0/ runs the 15 Kit processes in dag order on one core pair and frigidaire_robot_replay.py `--load-dir` Kit-free on another as rows land. Docs: code/frigidaire/docs/robot.md (a dated section), the execution README rows.

## Run
Mac pytest before and after the edits. One window, inside the container: 1. `code/util/run_kit.sh code/execution/frigidaire/frigidaire_grip_probe.py --headless` (the smoke; stop on FAIL). 2. The driver: `code/util/run_kit.sh code/execution/frigidaire/frigidaire_robot_episode.py --headless --place-goal data/results/robot/samples_v1/<s0>/load.json --dishes <id> --state <prev.json> --run-id s0_<id>` for the 15 dishes in dag order, the replays beside. 3. Results, media and logs copied back; `brev stop`; STOPPED confirmed.

## Works when
data/results/robot/samples_v1/<s0>/rows.jsonl holds 15 rows, one per dish, each with gate, picked, carried, seated, at_goal and a cause for every failure; a clip and the release and landed stills exist for every attempted dish; results/2026-11-04-grasp-place-s0.md gives the per-kind table (picked k of n, seated k of n, at-goal k of n), the window times, files changed, deviations and the commit subject.

## Assumed / skipped
- Assumed: the robot asset and its pad contacts work on 6.2 (the smoke decides; the 4.5 fix was SetInstanceable(False) before sim.reset); up to 2 pick tries per dish (the episode's re-grasp rule), then a failed row; the plate's rim pinch is the grasp gate's measured pinch (held 3 of 10).
- Skipped this round: s10, the stacked start, the harness RTX video, a page, new tests, any hardware change.
