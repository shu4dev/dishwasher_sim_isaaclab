# Sequential goals, swap-tested dependency graphs, per-edge clips and an order checker
Slug: 2026-10-15-sequential-goals-dependency-graphs | From: results/2026-10-14-newj-curve-and-leaning-pairs.md (follow-up, no lit review)

## Goal
For r02_00048 the goal is built one dish at a time by the benchmark move, every edge of its graph comes from a failed order swap in a state a build passes through and has a clip of that failure, and a new page lets the user reorder the dishes, see which edges break, and what Isaac did for 20 random orders.

## Decisions (from grilling, 2026-10-15)
- The doubt: a jointly settled goal is a state the one-dish move need not reach, so the v20 test asked the wrong question. Sampler = restricted packing + `build_sequence` (lowest first, learns up to 8 builds): the built poses are the goal, the last build the witness, its learned constraints seed the graph. Rejected: goal filtering (drops the leaning loads), the farm loads (low S), a random order with deferral (new code).
- Edge test: for each candidate pair (collision pieces within 20 mm at the goal, the v20 rule) the witness order is swapped once, in the partial load present when the earlier dish went in: the later dish is placed first, then the earlier. A failed swap writes one edge with its reason: need (the later dish falls without the earlier), block (the earlier has no clear release once the later is in), knock (its landing moves the later). Final contacts add a support label. Rejected: the v20 full-context L/R test (states no build visits), support + learned only (no drop evidence).
- Margins kept: a swap within 20 % of any threshold reruns under 3 perturbed releases (3 mm / 3 deg), majority verdict (v20 code).
- Evidence: the graph is right when orders obeying it land all 24 and orders breaking one edge fail at that dish. Library per load, each order built in Isaac: the witness, one violating order per edge (the witness with that one swap: the later dish moved to just before the earlier), 10 random obeying orders, 10 random single-violation orders (a random obeying order with one random edge reversed the same way, so each predicts a failure at one named dish). Rejected: stability alone; agreement with a reference method.
- Animation: the witness build and one clip per edge, from tick poses recorded during the settles, Warp ray-traced like the stills (the box's RTX renderer crashes; corallab is unreachable). Rejected: build video only; Isaac cameras.
- Page: a new artifact "Dependency graphs" (Dot Score Rules stays trimmed): build video, graph, per-edge clips, a drag-to-order checker that flags every violated edge with reason and clip; an order in the library shows its Isaac outcome, any other is marked untested. Rejected: a section on the trimmed page.
- The graph serves three purposes at once: buildable orders, physical support, planner input. Study only: the HOTEC bench's goal certificate is untouched. Pilot: r02_00048 (the mutual-support pair); the easy loads follow once it works.

## Change
- `code/planner/frigidaire/frigidaire_dependency_kit.py`: `--edges swaps` = the `--goal built` path as the sampler (witness order recorded), swap tests through `drop()` in partial contexts (`context()` parks the later dishes), the library through `build_sequence` with a fixed order (no learning), tick poses recorded every 4 ticks to `<out>/_kit/<id>/traj/`; record `dependency/sequential/<id>.json`: goal, witness, edges (reason, support, margin), tests, library outcomes.
- `code/planner/frigidaire/frigidaire_dependency_clips.py` (new): frames from the trajectories with `frigidaire_dot_rules_render.scene/render`, the dropped and affected dishes tinted, the rest grey; mp4 (ffmpeg if the container has it, else GIF).
- `code/planner/frigidaire/frigidaire_dependency_page.py` (new): one HTML with the graph and library JSON embedded, media under `media/`; the checker in plain JS; published as a new artifact.

## Run
`code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load r02_00048 --edges swaps --library 10 --out-dir data/results/dot_rules/dependency/sequential --max-wall-seconds 14400 --headless` (one core, `taskset -c j,j+16` inside the container; the library orders as parallel sessions, `--order-index k`), then the clips and page scripts, then publish.

## Works when
`sequential/r02_00048.json` has goal_source sequential_build, a witness that lands 24 of 24, every edge with a reason and a clip, 20 library orders with outcomes, and the page's checker flags a hand-made violating order with the right edge.

## Assumed / skipped
- Assumed: if r02_00048 does not build in 8 builds the record says so and r02_00021, then a fresh packing, follow; a violating order stops two dishes after the predicted failure; clips 640 x 480, both-racks view only, about 4 s per dish; tolerances as in `frigidaire_bench.py` (AT_GOAL, DEV_MAX_M 0.08, the clear-hover release); the v20 graph of r02_00048 shown beside the new one without a claim; about 3 h of wall time with 14 sessions.
- Skipped this round: the other 10 loads, the random farm, the bench change, three-body tests, a planner ablation, Isaac-camera renders.
