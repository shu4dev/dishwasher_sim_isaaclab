# Run note: object-level rules, three-rule page, Isaac dependency graphs (smoke stage)

Plan: `plans/2026-10-13-object-level-rules-dependency-graphs.md`. Run 2026-10-14, 01:25-01:41 UTC, Brev box
(1x L4, 23,034 MiB free at start), container `dishsim-isaac` restarted (it had exited 137 four days earlier).
Scope: the plan's smallest run only (N = 1,024 grid, one Kit build of r05_00014), its rescore, and a smoke
figures + page build. Not run: the other 4 loads, the N = 262,144 grid, the publishes.

## What ran (repo root, wrappers docker-exec into dishsim-isaac)

| step | command | wall | result |
|---|---|---|---|
| Kit build | `code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load r05_00014 --headless` | 137 s (kit 135.5 s) | `[RESULT] PASS r05_00014 status cycle builds 3 support 0 learned 7 neighbour 0 waves none` |
| grid smoke (parallel with the build) | `code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_grid.py --N 1024 --dishes plate bowl cup --out data/results/dot_rules/grid_obj` | 826 s (level 821 s) | `[RESULT] PASS level N=1024 ...`, `[RESULT] PASS sweep done` |
| rescore | `code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_score.py --dependency-dir data/results/dot_rules/dependency/restricted --loads r05_00014 --N 262144` | 105 s | `[RESULT] PASS r05_00014 S_O packed 0.1917 built -, S_N packed 0.0859 built -, S_NJ packed 0.1574 built -`; `[RESULT] PASS scored 1 of 1 loads` |
| figures | `frigidaire_dot_rules_figures.py --grid-dir data/results/dot_rules/grid/N0262144 --grid-obj-dir data/results/dot_rules/grid_obj/N0001024 --sweep data/results/dot_rules/grid/convergence_sweep.json --loads-dir data/results/dot_rules/loads/N0262144 --farm-dir data/results/random_loads/rl_20261009 --dependency-dir data/results/dot_rules/dependency/restricted --out data/media/dot_rules/figures_v15_smoke` | 20 s | `[RESULT] PASS figures in .../figures_v15_smoke` (14 PNGs) |
| page | `frigidaire_dot_rules_page.py` (same five inputs) `--figures-dir data/media/dot_rules/figures_v15_smoke --out data/media/dot_rules/page_v15_smoke` | 7 s | `[RESULT] PASS page .../page_v15_smoke/index.html` |

Logs: `data/logs/dot_rules/{dependency_r05_00014,grid_obj_N1024,dependency_score_r05_00014,figures_v15_smoke,page_v15_smoke}.log`.
None has a traceback; the Kit log has no `free(): invalid pointer`, and Kit shut down cleanly.

## Grid smoke (N = 1,024, plate + bowl + cup)

- Level line: wall 821 s, rays 8.380e+09 at 1.412e+08/s, kappa 2.64, W* 740 s, binds 0, above1 0; theta 30
  |NG-NJ| max 5.041e-02, mean 5.399e-04. W* is now 90 % of the wall: 0 dots skipped, 30.2 M certified and
  19.2 M uncertified, against 7.8 M and 5.2 M (22.2 M skipped) in the 2026-10-09 N = 1,024 level (W* 211 s).
- Self-check lines: plate max |D_obj - D| = 0 scaled, O_obj differs from O by > 1 scaled in 1626 of 1968 cells;
  bowl 0, 1834 of 2096; cup 0, 1932 of 2144.
- Host json check (`python3 -I`): grid.json (7.97 MB) has plate, bowl, cup, plate_obj, bowl_obj, cup_obj, dot,
  dot_neg. Grid: 37 tilts x 9 heights x 8 distances = 2664 cells, 10 thetas. For plate, the cells with O_obj and O
  both >= 0 that differ by more than 1 number 1626 of 1968 (16,260 cell-theta entries), max |O_obj - O| 253,762
  scaled. Max |D_obj - D| = 0 over all 19,680 entries. Bowl max |O_obj - O| 159,497, cup 119,560, both with
  D diff 0. perdot_{plate,bowl,cup}.npz exist (cup 120 KB), and there are 111 partials (3 x 37).

## Dependency build r05_00014 (restricted, own-load landing rule)

- status `cycle`, reason "no order built the load in 3 build(s): last moved at plate_07 ['plate_03']; the learned
  order constraints form a cycle"; builds 3; learned 7, support 0, neighbour 0; n_waves None,
  n_waves_dependencies None, acyclic False (no `waves` key); failed_dish plate_07; kit.wall_seconds 135.5.
- Builds: (1) landing at plate_01, contact depth 11.9 mm (limit 2 mm; deviation 10.4 mm is fine); (2) landing at
  plate_07, contact 10.71 mm; (3) moved: plate_07 (contact 13.33 mm) knocked plate_03. The five plates
  placed first (02, 08, 03, 05, 04) land with 0 mm contact in every build, but plate_03 settles 23.3 mm off its
  packed pose, tilted 13.9 deg.
- Why it stops after 3 of 8 builds: build 2's landing failure taught (plate_03, plate_07) and (plate_04,
  plate_07), which the order already satisfied, so build 3 repeated build 2's order exactly. Physics then reported
  "moved plate_03", which taught (plate_07, plate_03), a 2-cycle. That is the shared gate
  `frigidaire_bench_kit.build_sequence`: on a landing failure it always puts the neighbours first. Learned edges:
  p03>p01, p03>p07, p04>p01, p04>p07, p07>p01, p07>p03, p08>p01.
- Rescore at N = 262,144: packed S O 0.191682, D 0.071005, DG 0.014654, DJ 0.090116, N 0.085910, NG 0.157364,
  NJ 0.157405; built null. Packed S_NJ equals the 2026-10-09 restricted score bit for bit (0.1574054096931643),
  so the all-sets W* change leaves dot scores unchanged.

## Figures and page (smoke, N = 1,024 grid)

- Page log: N 1,024 from grid_obj, dishes plate, bowl, cup; object level yes; 252 scored loads; sweep 7 levels;
  1 dependency graph; index.html 6,785,741 B (6.47 MiB); total 12.23 MB (limit 15 MB).
- index.html contains 'seg-level' (4x), 'plate_obj' (1x) and 'depgraph_r05_00014' (2x). The random-load section
  (8,692 chars between id="random-loads" and id="isaac-scenes") contains no 'D+G', 'D+J' or 'New+G'.
- page media/ (34 files): convergence, depgraph_r05_00014, dots_{plate,bowl,cup}, objlevel_slices,
  rand_{top5,bottom5}_{o,n,nj}, rand_record, rand_spread (PNG), plus 20 Isaac stills
  isaac_{rand,restr}_{top,bottom}{1..5}.jpg. data/ (1 file): depgraph_r05_00014.json (20 KiB).
- depgraph_r05_00014.png: correct for a cyclic failure, but not a layered DAG. All 24 dishes sit in one wave-0
  column with the 7 dashed learned arrows among plates 01/03/04/07/08, plate_07 ringed red, and two top views
  (lower rack 13 and upper rack 11 dishes at packed poses). Caption: "no waves (the order is cyclic); S under New+J
  packed 0.157". Cosmetic: the title says "status cycle" twice, and the top-view arrows overlap along the plate row.
- objlevel_slices.png: 3 x 3 (plate/bowl/cup x tilt/height/distance slices). Original grey, New orange, New+J
  blue; object level solid with markers, dot level dashed. The solid and dashed curves separate where expected
  (Original on the cup and the plate at 60-110 deg tilt; New+J past the 23.3 cm disc edge), and New's pair nearly
  coincides. Gaps mark skipped poses (cup below 8 cm). Looks right.
- dots_cup.png: 5 tilts (0-180) x 3 rules, the 500 dots in viridis on the cup silhouette, with set means from
  0.420 / 0.238 / 0.183 at tilt 0 to 0.006 / 0.000 / 0.000 at 180. Coloured, looks right.
- Not checked: the explorer's dot | object toggle in a browser (no browser here). Its markup and the _obj payload
  are present.

## Works when (plan)

- Smoke grid.json has cup and plate_obj, with O_obj != O in some cell and D_obj = D: met.
- r05_00014.json has status ok (or a labelled failure) with waves assigned: half met. It is a labelled failure
  (`cycle`, failing dish plate_07), but there are no waves, because the learned edges are cyclic.
- Page build prints `[RESULT] PASS`: met on the smoke data; toggle not exercised. Republish: not done (later stage).

## Problems and fixes

- No code or command fixes were needed. Nothing was rerun.
- The grid and the rescore shared the GPU for about 105 s, which raised W* for plate tilts 160-180 from about 8 s
  to 14-28 s per tilt. The results are unaffected.
- r05_00014 fails in physics: shingled packed plates reach 10-13 mm contact depth once their neighbours settle,
  and the gate's learning rule closes a 2-cycle after 3 builds. The other 4 restricted loads are likely to fail
  the same way.

## Still to run

- A decision on the dependency builds first: accept cyclic partial graphs without waves, or change the build
  (for example, try the opposite direction when a landing fails on contact depth, or build toward jointly settled
  poses). Then build the other 4 loads (r07_00023 r11_00050 r00_00031 r02_00048) in parallel, about 2-5 min each,
  and rescore them.
- The N = 262,144 grid with `--out data/results/dot_rules/grid_obj`. Projected at about 5.5 h, not 4.6 h. Rays
  take 9,650 s x 1.53 for the cup's 2,144 extra cells, about 14,700 s. W* takes 1,360 s x about 3.7 for
  all-sets dots, about 5,000 s.
- The final figures and page on grid_obj/N0262144, and the two publishes to F5vrhrNM4XnUk3ZXhJycY2.

## Stage 2: joint gate + builds (2026-10-14)

Run 02:03-02:44 UTC. Five parallel chains, one per load, ran 02:03:18-02:21:25 (r05 22 s, r00 8.4 min, r02 and r11
13.3-13.6 min, r07 17.8 min). The rescore followed, 02:21:55-02:43:31 (21.6 min). The steps follow the Decisions
entry "After the smoke (2026-10-14)": (a) the joint-gate manifest, Kit-free; (b) the top-5 joint gate (all 24 dishes
settled at once, 5 mm momentary peak, upper rack retracted first, containment); (c) the build toward the jointly
settled poses with the at-goal landing rule. Chain j (0-4, in table order) ran pinned inside the container to CPUs j
and j+16, one physical core. On the four gates still running, 61 of 61 Kit threads were on their pair. Each chain ran
at nice 10 with 2 OMP/BLAS threads. The N = 262,144 grid (container PID 726) ran alongside, untouched.

### Commands (repo root)

PREFIX = `docker exec -w /workspace/dishsim -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=2 -e MKL_NUM_THREADS=2 dishsim-isaac taskset -c <j>,<j+16> nice -n 10`

- (a) `PREFIX /workspace/dishsim/code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load <id> --manifest-out data/results/dot_rules/dependency/restricted/_gate/<id>/manifest.json`
- (b) `PREFIX /workspace/dishsim/code/util/run_kit.sh code/initialization/frigidaire/frigidaire_initial_state_validate.py --manifest data/results/dot_rules/dependency/restricted/_gate/<id>/manifest.json --out-dir data/results/dot_rules/dependency/restricted/_gate/<id> --max-wall-seconds 600 --peak-penetration-m 0.005 --headless --device cpu`
- (c) `PREFIX /workspace/dishsim/code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load <id> --gate data/results/dot_rules/dependency/restricted/_gate/<id> --force --headless`
- rescore: `code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_score.py --dependency-dir data/results/dot_rules/dependency/restricted --loads r05_00014 r07_00023 r11_00050 r00_00031 r02_00048 --N 262144 --force`

### Joint gate

| load | outcome | reason | wall s | max settle pen. mm (pair) | max cycle pen. mm (phase) |
|---|---|---|---|---|---|
| r05_00014 | penetration_failure | Excessive penetration during joint initial settling (early abort at 0.26 s simulated) | 20.4 | 5.572 (bowl_08 / plate_01) | - (no retraction ran) |
| r07_00023 | accepted | All joint loaded-state and retraction checks passed | 404.2 | 3.039 (LowerRack / bowl_01) | 0.0048 (LowerRack retraction) |
| r11_00050 | accepted | same | 400.4 | 2.249 (plate_04 / plate_06) | 0.0029 (LowerRack retraction) |
| r00_00031 | accepted | same | 401.9 | 0.444 (LowerRack / plate_05) | 0.0215 (UpperRack retraction) |
| r02_00048 | accepted | same | 403.2 | 4.216 (plate_03 / plate_05) | 0.0045 (LowerRack retraction) |

Gate dirs: `data/results/dot_rules/dependency/restricted/_gate/<id>/`. There was no closure_failure, simulation_error
or timeout, so no gate was rerun and no `_lower_first` directory exists.

### Builds toward the jointly settled poses, rescored at N = 262,144

| load | status | builds | support | learned | neighbour | n_waves | n_waves_dependencies | acyclic | failed_dish | kit s | S_NJ packed | S_NJ settled | S_NJ built |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| r05_00014 | gate | 0 | 0 | 0 | 0 | - | - | - | - | - (no Kit boot) | 0.1574 | - | - |
| r07_00023 | ok | 5 | 1 | 6 | 28 | 8 | 3 | yes | - | 660.5 | 0.1571 | 0.1597 | 0.1598 |
| r11_00050 | ok | 2 | 1 | 1 | 25 | 6 | 2 | yes | - | 414.1 | 0.1565 | 0.1580 | 0.1581 |
| r00_00031 | landing | 3 | 0 | 2 | 0 | 3 | - | yes | bowl_08 | 98.6 | 0.1561 | 0.1651 | - |
| r02_00048 | landing | 5 | 0 | 5 | 0 | 4 | - | yes | bowl_02 | 372.5 | 0.1553 | 0.1674 | - |

| load | S_O packed / settled / built | S_N packed / settled / built |
|---|---|---|
| r05_00014 | 0.1917 / - / - | 0.0859 / - / - |
| r07_00023 | 0.1887 / 0.1751 / 0.1753 | 0.0861 / 0.0816 / 0.0817 |
| r11_00050 | 0.1819 / 0.1732 / 0.1733 | 0.0870 / 0.0826 / 0.0827 |
| r00_00031 | 0.1868 / 0.1809 / - | 0.0889 / 0.0876 / - |
| r02_00048 | 0.1868 / 0.1783 / - | 0.0889 / 0.0870 / - |

Build sequences (deviations are from the dish's jointly settled pose; bowl tolerance 15 mm lateral, 20 mm dz, 15 deg):
- r07_00023: (1) landing at bowl_04 (23.6 mm, 20.4 mm dz, 34.6 deg); (2) plate_03 moved bowl_04 (contact 5.7 mm);
  (3) plate_01 moved plate_02 (contact 9.6 mm); (4) plate_08 moved bowl_06; (5) ok, all 24 placed.
- r11_00050: (1) plate_06 moved plate_01; (2) ok.
- r00_00031: (1) bowl_08 moved bowl_06 (contact 8.2 mm). In (2) and (3), bowl_08 lands 18.8 mm lateral, 9.9 mm dz and
  24.6 deg off its pose with 0 mm contact. After (2) every same-rack neighbour within 15 cm already went first, so
  `build_sequence` stopped after (3), at 3 of 8 builds. The status is the last build's `landing`, not `no_order`.
- r02_00048: (1) landing at bowl_06 (32.1 mm, 33.0 mm dz, 43.2 deg); (2) landing at bowl_04 (11.3 mm, 5.1 mm dz,
  36.0 deg); (3) plate_03 moved plate_05 (contact 8.9 mm). In (4) and (5), bowl_02 lands 9.8 mm lateral, 21.8 mm dz
  (limit 20) and 7.0 deg off with 0 mm contact. The same early stop came after 5 builds.
- r05_00014: bowl_08 against plate_01 reached 5.57 mm (limit 5 mm) in the first 0.26 s of the joint settle, and the
  gate aborted. Step (c) then wrote status `gate`, with no edges and no waves, without booting Kit.

Findings:
- Two of the five loads have full graphs: r07_00023 and r11_00050 (ok, acyclic, waves). Two have partial
  learned-edge graphs with the failing bowl marked: r00_00031 and r02_00048 (waves over the learned edges,
  n_waves_dependencies null). r05_00014 has no graph because it failed the gate.
- Built S matches settled S within 0.0002 under all three rules, as the at-goal landing rule implies.
- Against packed, settling lowers S_O by 3-7 % and S_N by 1.5-5 % on all four gated loads, and raises S_NJ by
  0.0015-0.0121. Under settled S_NJ the four rank r02 > r00 > r07 > r11 (packed: r07 > r11 > r00 > r02).
- r05_00014's packed scores are bit-identical to the smoke rescore (S_NJ 0.1574054096931643).

Problems and fixes:
- No command failed and nothing was rerun.
- A host-side `taskset` pins only the docker CLI, so each step ran as `docker exec ... taskset -c j,j+16 nice -n 10`
  around the wrapper inside the container, the pattern of frigidaire_random_load_farm.py and frigidaire_bench.py.
- `--force` overwrote the smoke's r05_00014.json. The smoke's r05_00014.json, its .scores.json and its Kit report are
  kept in `data/results/dot_rules/dependency/restricted/_smoke_20261014/`.
- An accepted gate took about 400 s on 2 vCPUs (the top-5 gates took about 160 s), within the 600 s cap.
- The rescore took 21.6 min, not about 10, because the gated loads also score settled poses and the built ones built
  poses. While it shared the GPU, grid tilts (plate t070-t095) slowed from about 2.2 to 3.3-3.6 min, about 8 min lost
  in all. The Kit chains did not slow the grid.

Logs: `data/logs/dot_rules/{manifest,gate,build}_<id>.log` and `dependency_score_5.log`, with no traceback and no
`free(): invalid pointer`. Outputs: `data/results/dot_rules/dependency/restricted/` (`<id>.json`, `<id>.scores.json`,
`_gate/<id>/`, `_kit/<id>/report.json`).

Grid at 02:44: plate t095 done, 20 of 111 tilt partials (`[02:41:04] N=262144 plate t095: 40 cells, rays 8.809e+09
in 146.6s (6.007e+07/s), W* 50.1s ...`).

Still to run: the figures and page from these results (`--dependency-dir data/results/dot_rules/dependency/restricted`),
the rest of the grid, and the two publishes.

## Stage 3: first-publish page (2026-10-14)

Run 02:47 UTC, Kit-free, about 23 s in all, beside the grid (container PID 726, untouched; 24 of 111 tilt partials at
02:50). Built on the existing dot-level grid `grid/N0262144` (plate and bowl only) without `--grid-obj-dir`, so the
object-level section is a note and the page has no cup. No fix, no rerun. Publish: not done in this stage.

| step | command | wall | result |
|---|---|---|---|
| figures | `code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_figures.py --grid-dir data/results/dot_rules/grid/N0262144 --sweep data/results/dot_rules/grid/convergence_sweep.json --loads-dir data/results/dot_rules/loads/N0262144 --farm-dir data/results/random_loads/rl_20261009 --dependency-dir data/results/dot_rules/dependency/restricted --out data/media/dot_rules/figures_v15a` | 21 s | `[RESULT] PASS figures in /workspace/dishsim/data/media/dot_rules/figures_v15a` (16 PNGs) |
| page | `frigidaire_dot_rules_page.py` (same five inputs) `--figures-dir data/media/dot_rules/figures_v15a --out data/media/dot_rules/page_v15a` | 2.5 s | `[RESULT] PASS page /workspace/dishsim/data/media/dot_rules/page_v15a/index.html` |

Page log: "N 262,144 from data/results/dot_rules/grid/N0262144; dishes plate, bowl; object level no (note); 252 scored
loads; sweep 7 levels; 5 dependency graphs; index.html 4.84 MB; total 14.37 MB". Logs
`data/logs/dot_rules/{figures,page}_v15a.log`, no traceback.

- Sizes: index.html 5,073,368 B; media/ 36 files, 9,812,091 B (16 PNGs, the five depgraph PNGs 193-304 KB, plus the
  20 Isaac JPGs); data/ 5 files, 179,018 B (depgraph_<id>.json, 9 KB for r05 to 56 KB for r07). All 41 local links
  resolve and no file is unreferenced. The PNGs stayed RGBA: figures + index came to 9.93 MiB, just under the page
  script's 10 MiB palette threshold, so they are about 3x the smoke's.
- grep -c in index.html: depgraph_r07_00023 2; joint gate 9 (10 occurrences); being computed 3; seg-level 2 (3
  occurrences, all in the explorer JS behind `if (byId("seg-level"))`; no `id="seg-level"` element, so no toggle). No
  D+G, D+J or New+G between id="random-loads" and id="isaac-scenes" (8,692 chars).
- depgraph r07_00023 and r11_00050: layered DAGs, 8 and 6 wave columns, support solid dark (bowl_05 > bowl_02;
  bowl_01 > bowl_05), learned dashed orange (6: four into bowl_04, plate_08 > bowl_06, plate_01 > plate_02; 1:
  plate_06 > plate_01), neighbour thin grey (28, 25); top views at the built poses (13 + 11, 12 + 12).
- depgraph r00_00031 and r02_00048: partial, learned edges only, a wave-0 column of 22 and 20 dishes ordered by rack,
  kind and id, the chains plate_07 > bowl_08 > bowl_06 (3 waves) and plate_03 > plate_05, bowl_01 > bowl_02 > bowl_04 >
  bowl_06 plus bowl_01 > bowl_04 (4 waves); the failing bowl (bowl_08, bowl_02) is ringed red in the graph and in the
  lower-rack view; top views at the jointly settled poses (12 + 12, 13 + 11).
- depgraph r05_00014: "joint gate rejected: excessive penetration during joint initial settling", one row per kind,
  no edges, top views at the packed poses (13 + 11).
- dots_plate: 5 tilts x 3 columns (Original, New, New+J), set means 0.998 / 0.938 / 0.814 at tilt 0 to
  0.012 / 0.000 / 0.000 at 180; dots_bowl the same layout; no dots_cup. rand_record: 4 panels, the record curves under
  Original (best 0.189, worst 0.038), New (0.131, 0.023), New+J (0.189, 0.031), and 5/(n+1) = 0.020 at n = 252.
- Captions: S values and Isaac times match the stage 2 tables after rounding. Odd points: (1) the four gated loads
  read "deepest contact during the rack retraction 0.0 mm (limit 5 mm)", which is right (0.003-0.022 mm) but omits
  the settle peak the limit binds on (r02 4.2, r07 3.0, r11 2.2, r00 0.4 mm); (2) r05_00014's caption gives no number
  for the rejection (5.57 mm, bowl_08 / plate_01) yet links an "edge list (JSON)" for a load with no edges; (3) the
  summary table lists only the two built loads; (4) the object-level note says "about 4.6 h", the plan now says 5.5 h.

Still to run: the grid to completion, then the figures and page with `--grid-obj-dir data/results/dot_rules/grid_obj/N0262144`
and the second publish.

## Stage 4: final page with the object-level grid (2026-10-14)

Run 06:39-06:41 UTC, Kit-free, about 30 s in all, after the N = 262,144 object-level grid had finished (container idle;
no process touched). The figures and the page read the dot-level grid `grid/N0262144` plus `--grid-obj-dir
data/results/dot_rules/grid_obj/N0262144` (plate, bowl, cup and `<dish>_obj`), the stage 2 dependency results, and the
sweep, loads and farm inputs of stage 3. No fix, no rerun. Publish: not done in this stage.

Grid: launched 01:49:09 UTC, done 06:33:42 UTC, wall 17,072 s = 4.74 h (projected 5.5 h), 111 tilt partials, log
`data/logs/dot_rules/grid_obj_N262144.log` with no traceback: `[RESULT] PASS level N=262144: wall 17072s, rays 2.145e+12
at 1.657e+08/s, kappa 2.64, W* 4093s, binds 0, above1 0; theta 30 |NG-NJ| max 4.915e-03 mean 1.461e-05`, then `[RESULT]
PASS sweep done`. Self-checks: max |D_obj - D| = 0 scaled for all three dishes; O_obj differs from O by > 1 scaled in
1633 of 1968 cells (plate), 1841 of 2096 (bowl), 1942 of 2144 (cup).

| step | command | wall | result |
|---|---|---|---|
| figures | `code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_figures.py --grid-dir data/results/dot_rules/grid/N0262144 --grid-obj-dir data/results/dot_rules/grid_obj/N0262144 --sweep data/results/dot_rules/grid/convergence_sweep.json --loads-dir data/results/dot_rules/loads/N0262144 --farm-dir data/results/random_loads/rl_20261009 --dependency-dir data/results/dot_rules/dependency/restricted --out data/media/dot_rules/figures_v15b` | 23 s | `[RESULT] PASS figures in /workspace/dishsim/data/media/dot_rules/figures_v15b` (18 PNGs) |
| page | `frigidaire_dot_rules_page.py` (same six inputs) `--figures-dir data/media/dot_rules/figures_v15b --out data/media/dot_rules/page_v15b` | 7 s | `[RESULT] PASS page /workspace/dishsim/data/media/dot_rules/page_v15b/index.html` |

Page log: "N 262,144 from data/results/dot_rules/grid_obj/N0262144; dishes plate, bowl, cup; object level yes; 252 scored
loads; sweep 7 levels; 5 dependency graphs; index.html 6.68 MB; total 12.89 MB" (MiB). Logs
`data/logs/dot_rules/{figures,page}_v15b.log`, no traceback, no warning.

- Figures: the 16 PNGs shared with figures_v15a are byte-identical to them (md5); new are dots_cup.png (378,134 B) and
  objlevel_slices.png (326,163 B).
- Sizes: index.html 7,008,803 B (limit 15,000,000); page directory 13,514,507 B; media/ 38 files, 6,335,868 B (18 PNGs
  1,860,515 B, 20 JPGs 4,475,353 B); data/ 4 files, 169,836 B. Figures + index came to 12.45 MiB, over the page script's
  10 MiB threshold, so it palettised all 18 PNGs (colour type 3, 6,041,035 -> 1,860,515 B, 3.25x); the log does not say so.
- media/ PNG (B): convergence 88,792; depgraph_r00_00031 73,401; depgraph_r02_00048 79,379; depgraph_r05_00014 60,536;
  depgraph_r07_00023 91,586; depgraph_r11_00050 86,511; dots_bowl 121,658; dots_cup 117,491; dots_plate 98,209;
  objlevel_slices 109,482; rand_bottom5_n 137,917; rand_bottom5_nj 139,254; rand_bottom5_o 139,106; rand_record 52,312;
  rand_spread 46,205; rand_top5_n 140,043; rand_top5_nj 137,488; rand_top5_o 141,145.
- media/ JPG (B): isaac_rand_bottom1-5 198,425 / 223,194 / 219,211 / 224,857 / 222,889; isaac_rand_top1-5 213,547 /
  212,166 / 190,227 / 223,751 / 223,823; isaac_restr_bottom1-5 236,245 / 230,490 / 227,554 / 235,605 / 233,847;
  isaac_restr_top1-5 234,196 / 232,112 / 232,633 / 230,193 / 230,388.
- data/ (B): depgraph_r00_00031.json 29,350; depgraph_r02_00048.json 35,612; depgraph_r07_00023.json 55,693;
  depgraph_r11_00050.json 49,181. No depgraph_r05_00014.json (gate-rejected). Against page_v15a: the 20 JPGs and 4 JSONs
  are byte-identical, the 16 shared PNGs differ (palette), and v15a's data/depgraph_r05_00014.json (9,182 B) is gone.
- Links: 42 local src/href (38 media/, 4 data/), all resolve, no file unreferenced; 1 external (Google Fonts).
- grep in index.html (lines / occurrences): `id="seg-level"` 1 / 1 (seg-level 3 / 4 with the JS); dots_cup.png 1;
  objlevel_slices.png 1; each of the five depgraph_r*.png 1; being computed 0; 4.6 h 0 (5.5 h 0); plate_obj, bowl_obj,
  cup_obj 1 each (the explorer's `window.GRID` keys, O, N and NJ only, 26,640 values each, read as `G[kind + "_obj"]`);
  D+G 6 / 8, D+J 6 / 8, New+G 8 / 9. Outside the appendix: D+G, D+J and New+G once each in the explorer's one-line
  definition of its four extra rules and twice each in the explorer JS rule table; New+G once in the header ("The
  appendix proves that New+G and New+J become the same rule as N grows", also in v15a). The rest are in the appendix.

Object-level table (14 cm up, over the disc centre, θ 30°, dot / object; equal to grid.json at 3 decimals):

| pose | Original | New | New+J |
|---|---|---|---|
| plate on edge (tilt 90°) | 0.806 / 0.754 | 0.336 / 0.335 | 0.102 / 0.102 |
| plate facing the arm (0°) | 0.998 / 0.998 | 0.938 / 0.939 | 0.814 / 0.814 |
| bowl facing the arm (0°) | 0.903 / 0.884 | 0.690 / 0.689 | 0.504 / 0.504 |
| cup facing the arm (0°) | 0.420 / 0.400 | 0.238 / 0.214 | 0.183 / 0.183 |
| cup on edge (90°) | 0.086 / 0.110 | 0.047 / 0.050 | 0.023 / 0.023 |

The prose under it: object minus dot over the 62,080 non-skipped cell-θ entries, Original median −0.004 (−0.250 to
+0.159), New 0.000 (−0.039 to +0.020), New+J 0.000 (−0.166 to +0.895). Header tile: 0 dots above 1 in all seven rules.

- objlevel_slices.png (1700 x 1300): 3 x 3, rows plate / bowl / cup, columns tilt (14 cm up, 0 cm out), height (tilt 0°,
  over the centre) and distance (tilt 0°, 14 cm up), θ 30°. Original grey, New orange, New+J blue; object solid with
  markers, dot dashed, disc edge 23.3 cm dotted. Original separates most: object below dot on the plate at 40-110°
  (0.754 vs 0.806 at 90°) and the bowl to 80°, above it on the bowl at 90-120° and the cup at 70-100°; the cup's dot
  level rises to 0.431 at 30° while its object level falls (0.352), and at 26 cm height it is 0.713 vs 0.624. New's
  pair nearly coincides (largest gap the cup at 0°, 0.238 vs 0.214). New+J splits only past the disc edge (plate 0.402
  vs 0.688 at 30 cm, 0.187 vs 0.509 at 35 cm) and is flat in height (0.814 / 0.504 / 0.183). Gaps: bowl 3 cm, cup 3-5
  cm. No empty panel, no NaN, nothing clipped.
- dots_cup.png (1100 x 1773): 5 tilts (0° facing the arm, 45°, 90° on edge, 135°, 180° mouth up) x 3 rules, the 500
  dots in viridis on the grey cup silhouette seen along world Y, disc bar 14 cm below, one 0-1 bar. Set means
  Original 0.420 / 0.403 / 0.086 / 0.015 / 0.006, New 0.238 / 0.160 / 0.047 / 0.001 / 0.000, New+J 0.183 / 0.129 / 0.023
  / 0.000 / 0.000. At 0° only the band by the rim is bright (yellow under Original, teal under New, a few yellow rim dots
  under New+J); the deep wall and floor stay dark, so the cup scores far below the plate (0.998 / 0.938 / 0.814) and the
  bowl (0.903 / 0.690 / 0.504). At 45° the lit dots move to the wall nearest the arm (New+J about 0.7-0.8 at the rim
  corner), at 90° only the rim column is lit, at 135° and 180° only a few yellow and hollow-ring rim dots under
  Original. No empty panel, nothing clipped.

Odd points:
- index.html is 7.01 MB, below the expected 8-11 MB: the explorer carries only O, N, NJ for the three `_obj`
  entities (no D, DG, DJ, NG, zG, zJ). Well under the limit.
- The page palettised its PNGs this time (v15a stayed RGBA), so all 16 PNGs shared with the published v15a change bytes,
  and figures_v15b keeps the RGBA originals. The page log does not mention it.
- On a republish to the same URL, v15a's data/depgraph_r05_00014.json is kept unless the files map sets it to null;
  v15b neither ships nor links it.
- New+J object level can sit far above the dot level past the disc edge: max +0.895 at plate tilt 20°, 17 cm, 35 cm
  out, θ 10° (dot 0.004, object 0.899); 1,316 cell-θ entries differ by more than 0.1 (plate 864, bowl 404, cup 48;
  Original 3,030; New 0). Likely cause: a dot with an empty J set adds a zero denominator, so the few dots whose cone
  still reaches the disc set the ratio (dot scores summing to about 2 of 500). The prose says "a blocked dot still adds
  its whole denominator", which holds for New, not for an empty-J dot under New+J; the +0.895 is shown but not explained.
- The dot | object toggle (`id="seg-level"`, single-dot button disabled at object level) and the `_obj` payload are
  present; at object level the D, D+G, D+J and New+G curves have no data, as the explorer prose says. Not exercised
  in a browser (no node or browser on the box).
- Stage 3's four odd points are fixed: captions give the settle peaks (r07 3.0, r11 2.2, r00 0.4, r02 4.2 mm), r05's
  gives 5.57 mm (bowl_08 into plate_01) with no JSON link, the summary table lists all five loads, no "4.6 h".

Wording fix (06:52 UTC). `objlevel_section` in `frigidaire_dot_rules_page.py` only: p1 "a blocked dot still adds its
whole denominator" -> "a dot whose lines are all blocked still adds its whole denominator"; p2 gains two sentences after
"blocked dots included." on the empty-J effect, computed in the function from the grid over the same `ok` mask as the
object-minus-dot sentence that follows them (signed object - dot > 0.1, all dishes, every pose and theta; the odd point's
1,316 / 3,030 above count |object - dot| > 0.1: signed they are 967 above + 349 below and 240 + 2,790). Rebuilt with the
stage's page command (same six inputs, `--figures-dir data/media/dot_rules/figures_v15b --out
data/media/dot_rules/page_v15b`), log `data/logs/dot_rules/page_v15b_fix.log`, 7 s, no traceback: `[RESULT] PASS page
data/media/dot_rules/page_v15b/index.html`. index.html 7,009,343 B (+540 B, exactly the two edits); "no such line" 1,
"whole denominator" 1, the old phrase 0; media/ 38 files (6,335,868 B) and data/ 4 files (169,836 B, no r05) unchanged;
42 local src/href, all resolve. New sentences: "Under New+J a dot has a weight only when at least one of its lines
reaches the arm disc; a dot with no such line adds nothing to either sum, where the dot level counts it as a zero. Past
the disc edge most dots are of that kind, so the object level rates only the dots that still see the disc: it exceeds the
dot level by more than 0.1 in 967 of the cell–angle entries (Original 240, New 0), at most by +0.895 at the plate at tilt
20°, 17 cm up, 35 cm out, jet angle 10° (0.004 at dot level, 0.899 at object level)."

Publish: done 06:58 UTC, https://claude.ai/artifact/F5vrhrNM4XnUk3ZXhJycY2 Version 16 (version id 1791960881-a2f8), 38 media + 4 data files, `data/depgraph_r05_00014.json` removed; the page is private until shared.
