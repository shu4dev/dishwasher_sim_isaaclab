# One benchmark: the Frigidaire twin, the HOTEC set, the sampler's load as the ground truth
Slug: 2026-11-01-merged-benchmark | From: code/frigidaire/docs/hotec_bench.md, research/2026-11-01-general-assets.md, results/2026-11-01-stack-upgrade.md

## Goal
One benchmark and one CLI (`code/planner/bench.py`) on Isaac Sim 6.2: messy start, the sampler's score-greedy load as the goal and ground truth, three-level tier labels, the bench's fault rule, four algorithms; the Bosch and ArtVIP twins, the YCB props and the slot pipeline leave the repo; three pairs run end to end on the box.

## Decisions (from grilling)
- The merged benchmark is the main benchmark for algorithms. Machine: the Frigidaire twin only; dishes: HOTEC only. The Bosch and ArtVIP twins and the YCB props are removed (unused, merge cost); rejected keeping them as later rounds. Block 1 (gen_instances, tiers cells, compare_algorithms, capacity, compat) retires to git history; rejected a legacy pipeline.
- Start = the bench's: n dishes in messy counter stacks (n uniform in [N/4, 3N/4] from the seed), the rest dropped at random into the racks, settled, repro gate; counter cap = n + slack.
- Goal = always the sampler's: greedy by score, random dish order per seed, each dish at the feasible site and tilt with the highest S gain, joint gate after each drop (deferral and the lean-side bank rule stay); S_ref = its fair score. Rejected best-of-N packings and ascent.
- Truth = the goal load and S_ref. Track A reports solved within the budget and raw moves, no gap; compat's optimum and the lower bound go. Rejected the lower-bound gap.
- Tiers = three parameters with three levels: kinds 1 / 2 / all, count 1-8 / 9-16 / 17-24, allowance n+3 / n+2 / n+1. Explicit `--kinds`, `--count`, `--counter-allowance` define the roster; the levels are labels stored in the instance. Rejected drawing rosters from levels and fixed rosters per level.
- Faults: a nudge is counted, fatal only if the nudged dish was at goal or leaves the counter and racks. Algorithms: greedy_offline, rrt_connect (A); first-fit, move-level MCTS (B).
- Code home: `code/src/sampler` becomes `code/src/bench` with `run` and `report` stages; bench `--start` and `--finalize` retire. The removal happens in this round (rejected its own round first). The removed asset folders stay on the hub as history with a README line (yours), local copies go to hold folders. corallab keeps one line in CLAUDE.md. Everything on 6.2: the ceilings are recomputed on Warp 1.17 and the flush-travel baseline is recorded first. First run: today's rosters 7 bowls / 8 plates + 7 bowls / 8 + 7 + 8 cups, seed 0, four algorithms, one window with a 4 h cap; a roster with no valid pair after 6 seeds is reported unsampled (gate, certify or build) and the others still run.
- Plan grill: candidates are ranked at about 1/16 of the score stage's rays and the chosen load scored once at full budget for S_ref (rejected full rays everywhere and new-dish-only casts); the report stage writes an HTML page (tables, stills, videos, no paragraphs) plus the results note; `rearrange.py` is pruned now (slot oracle seam, placement at-goal check, Bosch knobs out), the bench's episodes must still replay (rejected trimming imports only).

## Change
- Delete `code/src/dishsim/{capacity,compat,instance_gen,rack_layout,slotting,machine,scene,tiers}.py`, `code/initialization/bosch/`, `code/planner/bosch/`, `code/tests/test_{compat,config_hash_frozen,instance_gen}.py`, `docs/{bosch800_asset,bosch800_source_data,joint_report,success_criteria,extending,overview}.md`. Keep `rrt`, `collision_world`, `geometry`, `media`, `app`, `quats`, `transforms`, `checks`; prune `rearrange.py` to the episode driver the bench uses (`run_episode`, `Instance`, `Move`, `Greedy`, the nudge rule) and delete `placement.py`; trim `config.py` to what `collision_world.py` and `media.py` still read.
- `code/src/bench/` (the sampler renamed, tests renamed with it): `build.py` = score-greedy insertion (random order per seed; per dish every candidate site and allowed tilt, S gain by a Warp cast of the partial load at a reduced ray budget, joint gate as today); `difficulty.py` = level labels from the explicit flags; new stages `run` (the four algorithms' episodes on the pair through `frigidaire_bench_kit.py --run`, cameras on) and `report` (the bench page over pairs). Loader, manifests and the other stages unchanged.
- `code/planner/bench.py`: the sampler's flags plus `--algorithms`; `frigidaire_bench.py` keeps `--run`, `--analyze` and the page functions, loses `--start`/`--finalize`; `frigidaire_bench_kit.py --run` reads a pair.
- Docs: CLAUDE.md "What this is", commands and validation rewritten for the one benchmark (corallab one line; hold folders listed); README.md; `docs/architecture.md`, `docs/environment.md` pruned; `docs/repo_map.txt` redrawn; `data/assets/README.md` gets the history line (upload yours).
- Box, in order: the empty-machine baseline at the flush travel (`data/results/benchmark/frigidaire_hotec/baseline/result.json`, the record the start kit's `--baseline` compares against; recorded by the same kit), the ceilings (`frigidaire_dot_rules_grid.py` inward and outward, `--out data/results/dot_rules/ceiling_62`), then the three pairs and their twelve episodes.

## Run
Mac first: `python3 -m pytest code/tests/ code/frigidaire/tests/ -q` and `python3 code/planner/bench.py --help`. Box, one window, self-halt at T0, each gate judged from its log:
```
code/util/run_py.sh code/planner/bench.py --kinds bowl --count bowl=7 --counter-allowance 3 --seed 0 --to report --algorithms greedy_offline,rrt_connect,first_fit,mcts --out data/results/bench/v1
code/util/run_py.sh code/planner/bench.py --kinds plate,bowl --count plate=8,bowl=7 --counter-allowance 2 --seed 0 --to report --algorithms ... --out data/results/bench/v1
code/util/run_py.sh code/planner/bench.py --kinds plate,bowl,cup --count plate=8,bowl=7,cup=8 --counter-allowance 1 --seed 0 --to report --algorithms ... --out data/results/bench/v1
```

## Works when
Three valid pairs (gate and certify PASS) carry an S_ref from the score-greedy build and the labels easy/easy/easy, medium/medium/medium, hard/hard/hard; twelve episodes recorded on 6.2 with stills and videos; the report lists solved, moves and S / S_ref per roster and algorithm; pytest green on the Mac after the removal; no `bosch800`, `artvip` or `bosch_props` reference left under code/; results/2026-11-01-merged-benchmark.md written.

## Assumed / skipped
- Assumed: the hub deletion or history line is yours; the Dependency graphs and Dot Score Rules pages stay as 4.5 history; the robot block is untouched; the 27 pairs and the 4.5 bench record stay labelled 4.5.
- Skipped this round: renaming the `frigidaire_*` scripts, a mixed-level roster, more seeds, the pages, any second machine or dish set (none planned).
