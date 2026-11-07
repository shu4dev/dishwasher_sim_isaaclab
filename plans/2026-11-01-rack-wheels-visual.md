# The lower rack's wheels become visual only, then the easy roster through report on 6.2
Slug: 2026-11-01-rack-wheels-visual | From: results/2026-11-01-merged-benchmark.md

## Goal
The lower rack retracts without the hinge-seam lurch (empty machine, both orders, peak under the gate's 0.10 m/s), and one 7-bowl pair runs from load through report on Isaac Sim 6.2.

## Decisions (from grilling)
- The 8 lower-rack wheels get `collision=False` in the twin generator (the hubs already have it): the slide joint carries the rack and the asset calls them non-rotating roller geometry. Rejected a runtime FilteredPairsAPI patch (every twin consumer would carry it), a seam rebuild (faceted convex-hull wheels could still catch) and any cap or drive change (the lurch reaches the drive's 0.15 m/s limit, so a cap at the measurement would check nothing).
- Lower rack only: the upper rack peaked at 0.080 m/s in every gate and rolls in a closed cabinet track. Rejected doing both racks.
- The window runs the fix check and then the easy roster through report. Rejected check-only (a second window for 4 episodes) and all three rosters (medium and hard builds left dishes unplaced last time; that is the next round).
- The two 6.2 port fixes stay: the ground slab's pairs filtered against the appliance (the welded cabinet drifted 22 mm without it) and the 90 s kill of a Kit process hung in `app.close()` after its `[RESULT]` line.
- Plan grill: one window with a 2 h cap approved (rejected 90 min: one Kit retry would end it); the retraction check is a repo script, `frigidaire_retract_check.py` (rejected leaving the probe in data/logs and folding it into the baseline script); a fresh pair from load into `v2` (rejected rerunning the v1 start: mixed provenance); the hub upload follows the check and is yours (rejected skipping the hub).

## Change
- `data/assets/frigidaire_fdpc4221as/source/geometry.py`: the `Wheel_{side}_{i}` cylinders take `collision=False` (line 459); the source README gets the last-run line. Box, in the container: `build_frigidaire.py --out-dir data/outputs/twin_export`, copy `usd/lower_rack.usdc` and `usd/fdpc4221as.usdc` in (only those two change; `parameters.json` byte-equal), `export_components.py` (the 8 wheel solids flip to `collision: false`), twin pytest `-m pytest data/assets/frigidaire_fdpc4221as/source -q`. The old two USDs and components.json go to `data/_trash_20261101/wheels_colliding/` with a MANIFEST, on the Mac and the box. The hub upload of the changed files is yours, after the check.
- `code/initialization/frigidaire/frigidaire_retract_check.py` (new, from `data/logs/bench/probes/retract_diag.py`): restores the baseline, retracts each rack with the gate's ramp in both orders, prints each rack's peak speed and its stalls (lag over 15 mm), and `[RESULT] PASS` only when every peak is under 0.10 m/s and no stall occurred.
- Baseline: re-record at the flush travel on the new USD (`frigidaire_initial_state_validate.py --out-dir data/results/benchmark/frigidaire_hotec/baseline`); the 2026-11-01 record stays beside it as `result_wheels_colliding.json`.
- Docs: CLAUDE.md "Validation status", `code/frigidaire/docs/geometry.md` and `hotec_bench.md`: the lower-rack wheels are visual since this round and the rack-speed "flake" was the seam; `docs/repo_map.txt` loses the ◌ on run and report once they run.

## Run
Box, one window, self-halt armed at T0 + 2 h, every step judged from its log:
```
code/util/run_py.sh data/assets/frigidaire_fdpc4221as/source/build_frigidaire.py --out-dir data/outputs/twin_export   # then copy the two USDs in, export_components.py, twin pytest
code/util/run_kit.sh code/initialization/frigidaire/frigidaire_initial_state_validate.py --headless --out-dir data/results/benchmark/frigidaire_hotec/baseline
code/util/run_kit.sh code/initialization/frigidaire/frigidaire_retract_check.py --headless
code/util/run_py.sh code/planner/bench.py --kinds bowl --count bowl=7 --counter-allowance 3 --seed 0 --n-samples 1 --max-attempts 6 --to report --algorithms greedy_offline,rrt_connect,first_fit,mcts --out data/results/bench/v2
```

## Works when
`frigidaire_retract_check.py` prints PASS with both lower-rack peaks under 0.10 m/s and no stall; one easy pair in `data/results/bench/v2/` has gate_goal, certify, score and graph PASS with an S_ref; its four episodes have records, stills and videos, and `data/media/bench/v2/index.html` lists solved, moves and S / S_ref; twin pytest green and the Mac's Kit-free pytest unchanged (237 passed, the 11 Warp, PIL and packed-model failures); results/2026-11-01-rack-wheels-visual.md written.

## Assumed / skipped
- Assumed: a fresh pair from load on the new twin (`v2`), not the v1 starts, which settled with colliding wheels; v1 and its 13 pairs stay as history; the probe scripts stay in `data/logs/bench/`; the gate's lower-first retry stays; the box gets the twin by rsync, the hub after your upload.
- Skipped this round: the upper rack's rollers, the medium and hard builds (dishes left unplaced: the next round), more seeds, the pages.
