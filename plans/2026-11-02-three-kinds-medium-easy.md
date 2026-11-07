# One sample pair of all three kinds at medium count and easy counter slack
Slug: 2026-11-02-three-kinds-medium-easy | From: results/2026-11-02-fit-score-prove.md (the 6.2 pipeline as it stands; two small edits for the stills)

## Goal
One valid pair of plate 5, bowl 5, cup 5 at counter allowance 3 (levels kinds hard, count medium, allowance easy) through graph on 6.2, with its initial and goal stills, stored in its own folder `data/results/bench/samples` with a README, where `--from run --input <pair>.json` can reuse it.

## Decisions (from grilling)
- Roster plate 5, bowl 5, cup 5 (15 dishes, the medium roster's total; packs on 4 of 4 Mac seeds at level 0, 0 DAG cuts). Rejected plate 6, bowl 5, cup 5 (16) and plate 4, bowl 4, cup 4 (12): the user wants one sample, the balanced split at the medium total.
- No algorithms: the pair is the deliverable, saved for reuse. Rejected the default four (about 25 min of known track B gate failures plus 16 min of track A per pass) and track A only.
- Two chains in parallel, seeds 0.. and 10.., up to 3 attempts each on their own core pairs. Rejected one chain of 4 attempts (medium passed 1 of 4 seeds last window, a failed seed costs 15 to 22 min) and a single seed.
- Box cap 2 h from box up to STOPPED: the Mac driver stops the box at T0 + 108 min whatever the state. Rejected 1.5 h (one pass, no failed seed) and 3 h.
- Stills: `--algorithms none` makes the run stage render initial.png and goal.png and run no episode (the kit's `--run` renders both before its algorithm loop). Rejected `--to graph` (no picture of the sample) and one track A episode for the stills.
- Where: `data/results/bench/samples`, a folder of its own (`--out`; media under `data/media/bench/samples`, its own small page) with a README naming the pair, its labels, the command that made it and the reuse command. Rejected `data/results/bench/v3`: the user keeps samples apart from the benchmark runs. `data/experiments/` does not exist on the Mac, so no index entry.

## Change
- `code/src/bench/episodes.py` `stage_run`: `algorithms == ["none"]` gives an empty algorithm list, no own loads, the kit called with `--algorithms none`, the note "stills only"; the ledger entry passes on the kit log alone.
- `code/planner/frigidaire/frigidaire_bench_kit.py` `--run`: `--algorithms none` renders the two stills and skips the episode loop.
- `data/results/bench/samples/README.md` (the runner writes it on the box and the driver copies it back): pair, labels, S_ref, the two commands. The docs' bench.py line gains the `none` value (CLAUDE.md Commands, `code/frigidaire/docs/hotec_bench.md`).

## Run
Mac first: `.venv-kitfree/bin/python -m pytest code/tests/ code/frigidaire/tests/ -q` (237 passed, the 11 known failures). Then one box window, the scripts of `data/logs/fit/` reused under `data/logs/sample3/` (start_box, push of the two changed files, pre with the md5 check, chain with two rosters, driver copying back `samples` results, media and logs): T0 = box RUNNING, kill at T0 + 100 min, CAP = T0 + 108 min, RTX warm-up on cores 3/7, each chain judged from its log.
```
code/util/run_py.sh code/planner/bench.py --kinds plate,bowl,cup --count plate=5,bowl=5,cup=5 --counter-allowance 3 --seed 0 --n-samples 1 --max-attempts 3 --max-hours 0.9 --to run --algorithms none --out data/results/bench/samples
code/util/run_py.sh code/planner/bench.py --kinds plate,bowl,cup --count plate=5,bowl=5,cup=5 --counter-allowance 3 --seed 10 --n-samples 1 --max-attempts 3 --max-hours 0.9 --to run --algorithms none --out data/results/bench/samples
```
(cores 0/4 and 1/5, OMP 2, nice 10; the last chain to end writes the page over `samples`.)

## Works when
`data/results/bench/samples/frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s<seed>.json` has gate_goal, certify, score, graph and run PASS with an S_ref and levels {kinds hard, count medium, allowance easy}; `data/media/bench/samples/<pair>/initial.png` and `goal.png` exist and are not black; the README names the pair; all of it is on the Mac; `brev ls` says STOPPED within the cap; Mac pytest unchanged; results/2026-11-02-three-kinds-medium-easy.md written with S fit, S ascent, S_ref, the seeds spent and the window times.

## Assumed / skipped
- Assumed: the fit-score-prove code and caches already on the box (pre's md5 check, no rebake); a second passing chain is a bonus pair, kept; `--on-counter` drawn from the seed.
- Skipped this round: episodes, videos, track B's own loads, more seeds, the pages.
