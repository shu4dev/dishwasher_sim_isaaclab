# Run note: New+J explorer curve, relative S, and the leaning bowl pairs under the v20 move

From: `research/2026-10-14-newj-curve-and-leaning-pairs.md` (no plans/ file; brief from the main session). Parent run note:
`results/2026-10-14-pairwise-dependency-graphs.md` (Version 18/19). Run 2026-10-14, 18:41-20:25 UTC, Brev box (1x L4),
container `dishsim-isaac`. Short ids: p = plate, b = bowl, c = cup; "a > b" = a before b.

## What works

- Deliverable A: all 11 gated loads have `dependency/pairs_v20/<id>.json`, each with `[RESULT] PASS` and no traceback or
  `invalid pointer`: 5 ok (r06_00008, r07_00023, r09_00014, r11_00034, r11_00050), 4 landing (r00_00031, r04_00021,
  r05_00026, r13_00038), 1 cycle (r02_00021), 1 mutual_support (r02_00048, diagnostic build ok). The 4 perturbed records
  are in `pairs_v20_perturbed/`: r11_00050 ok, r07_00023 ok, r00_00031 cycle, r02_00048 mutual_support. Rescored
  (`<id>.scores.json`) and relative S (`<id>.relative.json`) for all 11.
- Deliverable B: `data/media/dot_rules/page_v20/index.html` is 1,238,114 B (v19: 7,029,273 B; the explorer payload is now
  New+J only), with 44 media and 11 data files, 15,288,872 B in all; 55 local src/href, none missing, none unreferenced.
  The explorer is New+J only, one curve per entity (dot, plate, bowl, cup; dot | object level), Show = score | relative to
  0°; the relative-S column and best-by-relative row, the fifth spread panel and the dependency-table columns are present.
  Not published (main session).
- Spearman rank correlation of relative S against absolute S under New+J, 252 farm loads: **0.916** (page: 0.92); 2 of
  the five best by New+J are among the five best by relative S. Best by relative S: w03_g2_l0003 (0.300), which is also
  best by Original. Farm relative S min / median / max 0.053 / 0.170 / 0.300. References floored at 0.05: 216 of 6,048
  farm dishes.

## Rules as run (decided during the run, recorded in the research file by the main session)

- Release = the lowest hover, in 1 mm steps up the rack's z axis from the goal pose (cap 60 mm), whose pose passes the
  bench checker's **1 mm penetration allowance** (`checker.pair()["valid"]`, the farm's preflight) against the placed dishes
  (at their re-settled poses) and the appliance (user, pop-up 18:55 UTC). The brief's "at least 1 mm from" was
  unreachable: measured Kit-free on the 11 gated loads, a 1 mm separation from the appliance is never reached for 5-8
  dishes per load (lower plates leaning on vertical tines keep 0 mm distance along the whole lift), and a 1 mm separation
  from dishes left 0-3 dishes per load with no release in an idealised build (pairs touching at the goal).
- The 3 mm / 3° perturbation is applied to the release pose found at the unperturbed target, as `--perturb-*` did to
  v18's target + 3 mm. Perturbing the goal first sent a tine-nestled bowl to a 48 mm release in the smoke check.
- `--max-wall-seconds 10800` instead of 7200 (a deadline loses the whole record). Margin and two-way reruns use seeds 1, 2, 3
  unperturbed and 11, 12, 13 in the perturbed sessions (`10 S + 1..3`), so the perturbed run never reuses the first
  run's draws.

## Commands (repo root)

PREFIX = `docker exec -w /workspace/dishsim -e OMP_NUM_THREADS=2 dishsim-isaac taskset -c <j>,<j+16> nice -n 10`, run as
`setsid nohup PREFIX ... > data/logs/dot_rules/<log>.log 2>&1 &`; KIT = `code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load <id> --gate data/results/dot_rules/dependency/restricted/_gate/<id> --edges pairs --max-wall-seconds 10800 --headless`.

| step | command | wall | result |
|---|---|---|---|
| smoke 1, 2 | 4-dish r02_00048 (b02 b04 b06 p02) via scratchpad wrappers; forced margin reruns, then forced mutual support + perturbed | 18:57-19:01, 19:01-19:03 | `[RESULT] PASS ... status landing`, `[RESULT] PASS ... status mutual_support ... builds 1 diagnostic`; outputs deleted |
| 11 loads, j 0-10 | `PREFIX KIT --out-dir data/results/dot_rules/dependency/pairs_v20` (log `pairs_v20_<id>`) | 19:03:57-19:46:27, 1975.8-2549.6 s each | one `[RESULT] PASS <id> status ...` each (table below) |
| perturbed, j 11-13 then 10 | `PREFIX KIT --perturb-mm 3 --perturb-deg 3 --perturb-seed 1 --out-dir data/results/dot_rules/dependency/pairs_v20_perturbed` (log `pairs_v20_perturbed_<id>`); r02_00048 launched 19:37:18 by a watcher on the slot r09_00014 freed | 19:03:57-20:22:38, 2182.2-2719.7 s each | r11 ok, r07 ok, r00 cycle, r02 mutual_support |
| relative S, farm + v18 | `code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_relative.py --loads-dir data/results/dot_rules/loads/N0262144 --dependency-dir data/results/dot_rules/dependency/pairs` (log `relative_v20`) | 1732 s | `[RESULT] PASS 269 files, 6864 dish references ...; farm loads 252: Spearman(S_rel, S_NJ) 0.916`; check line plate / bowl / cup 0.814 / 0.504 / 0.183 = the grid |
| rescore | `code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_score.py --dependency-dir data/results/dot_rules/dependency/pairs_v20 --N 262144 --theta 30` (logs `dependency_score_pairs_v20_{a,b}`) | 19:37-19:48, 20:04-20:18 | `scored 5 of 5`, `scored 6 of 6` |
| relative S, v20 | `... frigidaire_dot_rules_relative.py --dependency-dir data/results/dot_rules/dependency/pairs_v20` (log `relative_v20_pairs_v20`) | 152 s | `[RESULT] PASS 11 files, 672 dish references` |
| figures | `code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_figures.py --grid-dir data/results/dot_rules/grid/N0262144 --grid-obj-dir data/results/dot_rules/grid_obj/N0262144 --sweep data/results/dot_rules/grid/convergence_sweep.json --loads-dir data/results/dot_rules/loads/N0262144 --farm-dir data/results/random_loads/rl_20261009 --dependency-dir data/results/dot_rules/dependency/pairs_v20 --out data/media/dot_rules/figures_v20` | 25 s | `[RESULT] PASS`, 24 PNGs |
| page | `code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_page.py` + the figures' grid/sweep/loads/farm/dependency flags + `--rerun-dir .../pairs_v20_perturbed --compare-dir .../pairs --compare-rerun-dir .../pairs_perturbed --figures-dir data/media/dot_rules/figures_v20 --out data/media/dot_rules/page_v20` (logs `page_v20`, rebuilt as `page_v20b` with the "built: diagnostic" label) | 3 s | `[RESULT] PASS`, index.html 1.18 MB |

## Per load, Version 18 → Version 20 (tested edges, learned left out; reproduced = under the 3 mm / 3° release)

| load | tested edges (knock / need / block in v20) | extra constraints | two-way pairs | reproduced | status v20 (failing dish) | S New+J built / relative S settled |
|---|---|---|---|---|---|---|
| r07_00023 | 8 → 4 (0/0/4) | 4 → 0 | none → none | 4 of 8 → 4 of 4 | ok | 0.1596 / 0.437 |
| r11_00050 | 3 → 2 (0/0/2) | 0 → 0 | none → none | 3 of 3 → 2 of 2 | ok | 0.1574 / 0.433 |
| r00_00031 | 5 → 2 (0/1/1) | – → 3 | b06–b08 cycle → none | 4 of 5 → 2 of 2 (+2 new) | landing (b08) | – / 0.448 |
| r02_00048 | 5 → 6 (0/2/4) | – → 0 | b02–b04, p02–p08 cycles → b02–b04 mutual support | 4 of 5 → 6 of 6 | mutual_support | 0.1669 diagnostic / 0.456 |
| r06_00008 | 5 → 4 (0/1/3) | 0 → 0 | none → none | – | ok | 0.1578 / 0.421 |
| r04_00021 | 3 → 2 (0/1/1) | 0 → 0 | none → none | – | landing (b07) | – / 0.424 |
| r02_00021 | 6 → 5 (0/0/5) | – → – | b05–b08 cycle → cycle | – | cycle | – / 0.438 |
| r11_00034 | 2 → 1 (0/0/1) | 0 → 0 | none → none | – | ok | 0.1612 / 0.455 |
| r05_00026 | 4 → 2 (0/1/1) | – → 1 | b05–b06 cycle → none | – | landing (b05) | – / 0.449 |
| r13_00038 | 4 → 3 (0/2/1) | – → 1 | b02–b05 cycle → none | – | landing (b02) | – / 0.418 |
| r09_00014 | 1 → 1 (0/0/1) | 0 → 0 | none → none | – | ok | 0.1598 / 0.444 |

Totals: 46 → 32 tested edges (knock reasons 28 → 0: the goal-judged disturbance removes them; 8 need, 24 block). Extra
constraints: v18 0 on its five built loads and 4 on r07; v20 0 on all five ok loads and on r02_00048's diagnostic build.
Reproduced under the perturbed release: v18 15 of 21 → v20 14 of 14 (+2 new edges in r00), two-way pairs agree in 3 of 4
(r00's perturbed run gains a b02–b05 cycle). Built S under New+J moves by at most -0.0007 against v18's builds.

## The five v18 bowl cycles

- r00_00031 b06–b08: no longer two-way. b08 falls without b06 (19.2 mm / 24.8°, need b06 > b08); b06 alone stays within
  tolerance (9.3 mm / 4.1°) but then blocks b08's release (no clear release, b06 + p07): the build fails at b08 (landing).
- r02_00048 b02–b04: **mutual support** (each falls without the other: b02 dz 22.2 mm, b04 tilt 35.8°); the diagnostic
  build with the pair released together builds the load in 1 build (built S New+J 0.1669). Its plate cycle p02–p08 is gone.
- r02_00021 b05–b08: **cycle**: neither bowl falls without the other, but each has no clear release with the other in
  place (block both ways), unchanged in all 3 perturbed retests of each drop.
- r05_00026 b05–b06: no longer two-way: need b06 > b05 (b05 tilts 16.9° without b06); b06 alone relaxes 8.6 mm / 13.1 dz /
  9.8°, within tolerance, and then b05 has no clear release (LowerRack + b06): build fails at b05 (landing).
- r13_00038 b02–b05: no longer two-way: need b05 > b02 (b02 26.4 mm dz / 27.7° without b05); b05 alone stays within
  tolerance (6.9 mm / 7.7°) and then b02 has no clear release (b05 + p03): build fails at b02 (landing).

So none of the five builds one dish at a time: 1 mutual support, 1 cycle, 3 one-way-need pairs whose partner cannot be
released once the standing bowl is in.

## The landing loads (four, not five: r00, r04, r05, r13)

Every failed build step is a no-clear-release, never a landing outside tolerance or a knock. Learned constraints are
written into the graph as edges with reason "learned" (`extra_constraints`); none was among the tested edges.
- r00_00031: 3 builds; b08 at build 1 (b06), b05 at build 2 (b02 + p05), b08 at build 3 (b06 + p07); learned p01 > b05,
  p05 > b05, p07 > b08.
- r04_00021: 1 build; b07 (LowerRack + b01). b07 needs b01 (it slides 21.5 mm off its goal in the re-settle without
  b01), and b01 alone settles 3.3 mm / 8.6° off its goal (8.7° when placed in the build), enough to close b07's column;
  every 15 cm neighbour of b07 was already ordered, so nothing was learned (0 learned). v18 built this load: its 3 mm
  release into the neighbour worked.
- r05_00026: 2 builds, b05 both times (LowerRack + b06); learned p06 > b05.
- r13_00038: 2 builds, b02 both times (b05 + p03); learned p03 > b02.

## Explorer, per-dot captions, relative S

- Relative New+J at 14 cm up, 0 cm out, θ 30 (grid and the built page's explorer table): 30° plate / bowl / cup 0.867 /
  0.865 / 0.863, 60° 0.504 / 0.501 / 0.499 (dot 0.866 / 0.500); on edge 0.125 / 0.124 / 0.127. The cup's best Original
  tilt is 25° (0.4315 against 0.4195 at 0°), stated in the explorer prose.
- Per-dot captions, share of the 0° score kept on edge: Original plate 81 % / bowl 42 % / cup 20 % (0.808 / 0.419 /
  0.205), New+J 12.5 % / **12.4 %** / 12.7 % (the brief quoted 12.5-12.7 %; the bowl is 12.4 %).
- Relative above 1 (dot level, cells with a reference ≥ 0.05): dot 0.28 % (max 1.45), plate 0.04 % (1.006), bowl 0.02 %
  (1.02), cup 0.31 % (1.14); object level plate up to 1.60. The research file's "a few percent" is at most 0.31 % per
  entity; the axis grows (checked: 0-1.2 at the cup's maximum).

## Files changed

`code/planner/frigidaire/frigidaire_bench.py` (`RELEASE_RULE`, `clear_release`, `off_goal`), `frigidaire_bench_kit.py`
(`build_sequence`: the clear release, goal-judged disturbance for at-goal dishes, hover per step, `together=` joint units;
`BenchOracle`: goal moves at the clear hover, protected dishes judged against their goal, others keep the counted nudge),
`frigidaire_dependency_kit.py` (the v20 pair test), `frigidaire_dependency_score.py` (skips `.relative.json`),
`frigidaire_dot_rules_relative.py` (new), `frigidaire_dot_rules_figures.py`, `frigidaire_dot_rules_page.py` (explorer JS,
`--compare-dir`, `--compare-rerun-dir`). New outputs: `dependency/pairs_v20/`, `pairs_v20_perturbed/`, 252
`loads/N0262144/*.relative.json`, 17 `dependency/pairs/*.relative.json` (v18 records untouched), `media/dot_rules/{figures,page}_v20`.

## Deviations, odd points, missing

- The fallen-dish rule runs before the drop: each context re-settle that leaves a dish outside its tolerance parks it and
  re-settles again (≤ 4 rounds), so no drop is wasted in an invalid context; need edges come from round 1 of L(B). The
  disturbance margin covers the context dishes a drop moved by > 1 mm / 1°. A two-way retest reruns the pair's 4 drops.
- The clear hover is 0 mm in 625 of 635 released drops (10 above 0, max 8 mm); 51 of 686 drops had no clear release.
  In practice the move is "release at the goal", or no release at all when a relaxed neighbour is in the way.
- A no-clear-release verdict hangs on sub-millimetre differences in the re-settle: r00 L(b05) had none, yet all 3
  retests released at 0 mm and landed (the perturbation comes after the search, so only re-settle history differed), so
  b02–b05 was not a cycle; in the perturbed session 1 of 3 released, so it became one.
- The pair test cannot see "place together" when need is one-way: no edge records that the partner has no release once
  the standing dish is in (r00, r05, r13, and r04 on a pair v18 built).
- Explorer: the distance axis is 0-35 cm for all entities (the dot's mirrored side is dropped, since all entities share
  one chart). Relative S is a column of the farm table plus a "Best by New+J relative" row (the table has rules as columns).
- Republish: v20 drops `media/depgraph_{r03_00045,r05_00001,r05_00010,r05_00014,r10_00037,r11_00049}.png` (the
  gate-rejected loads), which need `null` in the files map. The other PNGs ship as full RGB because the small index.html
  keeps the total under the 10 MB quantize threshold.
- Missing: no HOTEC bench rerun (the episode change is code only; the 18 Kit-free bench tests pass); the explorer JS was
  run in QuickJS with a stub DOM, not in a browser.

Publish: pending (main session)
