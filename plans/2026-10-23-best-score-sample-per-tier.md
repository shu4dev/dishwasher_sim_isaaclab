# The page sample per tier is the valid load with the highest score, not a random pick
Slug: 2026-10-23-best-score-sample-per-tier | From: results/2026-10-23-one-sample-per-difficulty.md (deliverable, no lit review)

## Goal
Score every distinct valid goal load, pick per tier the one with the highest S, complete it (initial configuration,
clips) and put it on the Dependency graphs page in place of the seed-by-convenience picks. Hard has three valid
loads, so its pick is informational; easy (17) and medium (7) are the point.

## Decisions (2026-10-23 04:30 ET, the session's; user: "choose the one with the highest score")
- Candidates: the distinct valid packings across the rest, bowl-plate and side-zone rounds (`data/results/sampler/
  {rest,bp,sz}_<tier>/`, pairs whose `goal.certify.valid` is true), deduplicated by the packed entries: easy 17,
  medium 7, hard 3 (listed in the note). Identical packings from two rounds count once; the earlier round's pair is
  the canonical one.
- Score: the pair's `score` stage on the box (`frigidaire_dependency_score.py`, GPU) for every candidate that lacks a
  `.scores.json`, then the kind-ceiling pass on the Mac (`frigidaire_dot_rules_relative.py` with `seed_relative` for
  new goal dirs, `--dependency-dir` over all candidate goal dirs). S = the built state's kind-ceiling S (a pooling
  dish scores 0); tie-break: surface reached, then the lower seed.
- Pick per tier: the highest S. For a winner that is not yet complete: the `initial` stage (`--from initial --to
  initial`, redraw on a settle failure as before), the `graph` stage, and the clips with the initial still. If the
  winner is the current page sample, it stays.
- Page: Version 19, the same layout as Version 18 (initial, goal with S, witness, graphs, difficulty label), the
  winners replacing easy seed 1 and medium seed 14 (and hard seed 2 if beaten); each sample's caption says it is the
  highest-S valid load of its tier and how many candidates there were. The yield section unchanged.
- Box: one window, 4 slots: 27 score stages (about 25 minutes), then up to 3 initial stages and 3 clips (about 20
  minutes); hard cap 75 minutes; `brev stop` right after; confirm STOPPED. The Mac sleep trap: drive the score batch
  from a box-side script.

## Change
- No sampler change. A small Kit-free picker (in `order_check.py` or a new `code/src/sampler/pick.py`) that lists the
  candidates with S, surface reached, pooling count and edges, and writes `data/results/sampler/picks.json`.
- `frigidaire_dependency_page.py`: the caption line (highest-S of n candidates); nothing else.

## Run
Box: score batch; copy back; Mac: relative pass, picker; box: initial + clips for the winners; copy back; `brev stop`;
Mac: page build into `data/media/dot_rules/dependency_page_v9/`; I publish.

## Works when
`results/2026-10-23-best-score-sample-per-tier.md` holds the full candidate table per tier (seed, round, S, surface
reached, pooling dishes, edges) sorted by S, the three winners with their initial-stage result, the page dir and its
files map, the box window and a suggested commit message; the page shows the winners; the box is STOPPED.

## Assumed / skipped
Assumed: S of the built state is the ranking quantity (the page's S). Skipped: completing the non-winning pairs,
`gate_initial`, any sampler change.
