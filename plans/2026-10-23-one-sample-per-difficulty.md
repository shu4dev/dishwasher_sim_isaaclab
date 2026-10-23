# One complete valid sample per difficulty on the Dependency graphs page
Slug: 2026-10-23-one-sample-per-difficulty | From: results/2026-10-22-bowl-plate-edges.md (deliverable, no lit review)

## Goal
The page https://claude.ai/artifact/AkiJZLtFZnFtXYUxSEHJmm shows, for easy, medium and hard, one valid sample in the
user's sense: a random initial configuration (counter stacks and rack drops, Isaac-settled), the goal configuration
(the built load), its score, and its dependency graph, with the witness video. Today the valid pairs hold the goal,
the witness and the graph only (`--from build --to certify`), so the initial configuration, the score and the
clips are produced here, and the page gains the initial-configuration figure.

## Decisions (2026-10-23 01:05 ET, the session's; user: "pick 1 valid sample in each difficulty and add it")
- Samples (the richest graphs among the valid pairs): easy = `bp_easy` seed 1 (7 bowls; 2 support, 3 access edges),
  medium = `bp_medium` seed 14 (15 dishes; 1 support, 21 access), hard = `rest_hard` seed 2 (23 dishes; 1 support,
  25 access). r05_00027 (24 dishes) stays on the page as the fourth, labelled as the earlier hand-roster sample.
- Initial configuration: `sample.py --from initial --to initial --input <pair>` on each pair (the bench's `--start` for
  the pair's tier and seed: n dishes in random counter stacks, the rest dropped into the racks, settled; the same
  roster ids as the goal). Stored in the pair file with its settle and reproduction checks; `gate_initial` stays SKIP
  (not connected) and the page says so.
- Score: the pair's `score` stage (`frigidaire_dependency_score.py`, GPU) on the box, then the kind-ceiling pass on
  the Mac (`frigidaire_dot_rules_relative.py --dependency-dir <the three goal dirs>`), so each sample shows S, surface
  reached and the per-kind means exactly as r05_00027 does.
- Clips: `frigidaire_dependency_clips.py --record <goal record> --out data/media/dot_rules/dependency_samples_v8/<key>`
  for the three (four labelled load views, graph PNGs for direct / reduction / closure, the witness video).
- Initial-configuration figure: a rendered still of the start (counter stacks and both racks, dishes labelled with
  their ids, the same Warp renderer the load views use, with the counter slab); if the renderer cannot place the
  counter, a top view of the racks plus a labelled list of the counter stacks, and the note says which.
- Page: `frigidaire_dependency_page.py` gains `--initial KEY=PNG` (or reads `initial.png` from the clips dir) and a
  difficulty label per sample (easy 7 bowls / medium 8 plates + 7 bowls / hard 8 plates + 7 bowls + 8 cups), the
  tuple shown as initial, goal (four views + S), witness video, graph; the yield section keeps its current run.
  Published to the same URL as Version 18, files map = the new media only (the r05 media stay).
- Box: after the side-zone round has stopped the box (never two drivers on it): one window for the three initial
  stages (~5 min each), three score stages (~4 min each) and three clips (~3.5 min each), about 40 minutes.

## Change
- `code/planner/frigidaire/frigidaire_dependency_page.py`: the initial figure, the difficulty label, the tuple order.
- A renderer entry for the start (in `frigidaire_dependency_clips.py` or `frigidaire_dot_rules_render.py`, whichever
  already draws the rack loads) that takes the pair's `initial` poses and the counter.
- No sampler change.

## Run
Box (one window): the three `--from initial --to initial`, the three `--from score --to score`, the three clips and
the three start renders; copy back; `brev stop`; confirm STOPPED. Mac: the relative pass, the page build into
`data/media/dot_rules/dependency_page_v8/`, publish (read the live page first).

## Works when
The page (Version 18) shows four samples with a difficulty label each, and for the three new ones the initial still,
the four load views with S, the witness video and the three graph variants; `results/2026-10-23-one-sample-per-difficulty.md`
holds the three samples' numbers (initial: dishes on the counter, settle result; S and per-kind means; edges; witness
time), the box window, files changed and a suggested commit message; the box is STOPPED.

## Assumed / skipped
Assumed: the pair file accepts the `initial` stage after `certify` (the ledger appends; otherwise the stage runs into a
copy and the note says so). Skipped: `gate_initial`, more samples per tier, any sampler change, the yield table.
