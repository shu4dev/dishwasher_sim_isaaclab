# Vertical insertion and drop height: the dependency graph under a realistic placement
Slug: 2026-10-15-insertion-drop-dependency | From: results/2026-10-15-sequential-goals-dependency-graphs.md (follow-up, no lit review)

## Goal
The sequential sampler, the swap tests and the library place a dish by lowering it from above the rack and dropping it from H above its goal; the Dependency graphs page shows, for one load per height, the goal, graph, clips, checker and library at H = 0, 20, 50 and 100 mm and which edges persist.

## Decisions (from grilling, 2026-10-15)
- The doubt: the teleport (hover 0 mm in nearly every drop) puts a dish inside a pocket no placement from above could reach and never tests a fall. Motion = vertical insertion + drop: at its goal orientation the dish starts with its lowest point 20 mm above its rack's rim wire and descends along the rack's up axis in 5 mm steps; every step must pass the bench checker's 1 mm allowance against the placed dishes and the appliance, else the move is blocked (no release); it is released at rest H above its goal and settles 150 + 60 ticks. Rejected: a plain drop from H (no path check), a drop from the rim (no lowering), the farm box top as the start (longer sweep, same geometry), placed dishes only in the path check.
- H = 20, 50 and 100 mm, each a full run (goal, graph, library, clips), beside 0 mm rerun under the path check so that H is the only variable. Rejected: one height.
- Scope: the dependency kit only (`--edges swaps`); the HOTEC bench's move and goal certificate stay. Rejected: bench too (every HOTEC result would need a rerun).
- Sampler landing: within DEV_MAX_M (8 cm) of the packed TARGET, not of the release pose, contact < 2 mm, no placed dish disturbed (build_sequence's rule); the built poses are the goal, S rescored on them. Rejected: the at-goal tolerance (most packings would fail).
- Pilot r07_00012 at every height; where it does not build, the next packings of `data/logs/dot_rules/sequential_fresh_order.txt` run in parallel until one builds (every failure kept with its reason). The 20 random orders, the per-edge violations, the margin reruns and the clips are as in the parent plan.
- Page: the same artifact (https://claude.ai/artifact/AkiJZLtFZnFtXYUxSEHJmm) gains a height selector for the video, graph, checker and library, plus one table of edges across heights (present, absent, or a different load). Rejected: one page per height.

## Change
- `code/planner/frigidaire/frigidaire_bench.py`: `insertion_release(checker, members, dishes, appliance, up, height_m, rim_z)` beside `clear_release`: the start pose from the rim, the 5 mm descent check, the release pose at H; a blocked path returns (None, None, blockers, step).
- `frigidaire_bench_kit.py` `build_sequence`: a `release=` switch (clear | insertion H); the own-load landing measured from the target.
- `frigidaire_dependency_kit.py`: `--release insertion --height-mm H`, recorded per drop (path blocked at which step, by what); records under `dependency/sequential_h<H>/`; `--merge-library` unchanged.
- `frigidaire_dependency_clips.py`: the descent and the fall are part of the recorded trajectory (recording starts at the start pose).
- `frigidaire_dependency_page.py`: `--records h0=... h20=... h50=... h100=...`, the selector and the cross-height table.

## Run
For h in 0 20 50 100: `code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load r07_00012 --edges swaps --release insertion --height-mm <h> --library 10 --out-dir data/results/dot_rules/dependency/sequential_h<h> --max-wall-seconds 14400 --headless` (fallback loads and library orders as parallel sessions, `taskset -c j,j+16`), then clips, page, publish to the same URL.

## Works when
Each of the four heights has a record with goal_source sequential_build and a witness landing 24 of 24 on some load, every edge has a reason and a clip, 26 library orders have outcomes, and the page's selector switches all four with the cross-height table filled.

## Assumed / skipped
- Assumed: the start height uses the dish's lowest collision point against its own rack's rim wire centre (geometry.py rim_height), capped by RACK_BOX; release at rest, goal orientation, no lateral search; a blocked path is reason "block" with the step and blocker recorded; about 45 min of box time per height, heights staggered over the 14 cores, one window of about 2 h.
- Skipped this round: the bench, three-body tests, more loads, a knock during the descent (the path check is geometric), Isaac-camera renders.
