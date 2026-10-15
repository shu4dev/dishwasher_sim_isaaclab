# Object-level rules, three-rule page, Isaac dependency graphs
Slug: 2026-10-13-object-level-rules-dependency-graphs | From: research/2026-10-13-object-level-rules-dependency-graphs.md

## Goal
The Dot Score Rules artifact shows object-level (ratio-of-sums) Original, New and New+J for plate, bowl and cup in the explorer, is trimmed to three rules outside the explorer and the appendix, and has an Isaac-built dependency graph (figure + JSON) for each of the 5 best restricted loads.

## Decisions (from grilling)
- Object level = Σ over a dish's 500 dots of the numerator ÷ Σ of the per-dot denominators (W* of every dot summed for the New family, so W* is computed for all dots, blocked ones included; the D family equals its dot mean). Rejected per-dish plots of load objects (not a new aggregation) and a best-posed-object reference (an orientation search, a new rule).
- Full grid + explorer "dot | object" toggle + cup entity at N = 262,144 (about 4.6 h). Rejected static slices as the form (the user wants the explorer) and smaller N (the page states one N).
- Dependency edges from the Isaac one-at-a-time build (`frigidaire_bench_kit.build_sequence`): support + learned + neighbour order, three line styles; waves (layered DAG columns) over all three classes, the count without neighbour edges in the caption. Rejected Kit-free FCL proxies (packed poses are not settled) and support-only.
- Loads: the 5 best of the existing 351 restricted loads (r05_00014 r07_00023 r11_00050 r00_00031 r02_00048), built toward their commanded poses with the own-load landing rule (settle deviation ≤ 8 cm, < 2 mm contact, no placed dish moved > 10 mm / 20°), no joint gate. Rejected fresh sampling, the 10 shown loads, all 351.
- `--sampler random` reads the 252 farm loads (settled rack-local poses, at-goal landing); parameter only, not run this round.
- A load no order builds in 8 builds keeps a partial graph of its learned edges with the failing dish marked and a caption saying so. Drawing: layered DAG by wave plus two top-view rack panels.
- Built poses rescored under the three rules at N = 262,144 (packed vs built S in the caption). JSON edge lists in the benchmark's goal-order format live in the repo and are published with the artifact, linked from each caption.
- After the smoke (2026-10-14): r05_00014 did not build toward its packed poses (shingled plates land 10-13 mm deep against the 2 mm limit; the learned order closed a 2-cycle after 3 builds), so each load first passes the top-5 joint gate (all 24 settled at once, 5 mm momentary peak, retraction, containment) and is built one dish at a time toward the jointly settled poses with the at-goal landing rule, the benchmark's goal path; a load failing the joint gate gets no graph. Rejected accepting the labelled failures, relaxing the contact limit, and switching the graphs to the random loads. The grid stays at N = 262,144 although it now projects to 5.5 h (measured 4.74 h).
- Page: new "Object level" section (table of key numbers dot vs object + one static three-slice figure read from the grid) and "Dependency graphs" section; everything except the explorer and the appendix trimmed to Original, New, New+J (rules key, cases table, per-dot figures incl. a cup figure, load tables and figures). Two publishes: after the builds (object section as a note), then after the grid.
- Empty-J dots (2026-10-14, after the grid): a dot with no line to the arm disc has no J weight, so at the object level it adds nothing to either New+J sum while the dot level counts it as a zero; past the disc edge object-level New+J rates only the dots that still see the disc (up to 0.899 against 0.004 at dot level; 1,316 cell–angle entries differ by more than 0.1). Kept as defined and stated on the page; the alternative (such a dot adds its New weight W*_F) needs a 4.7 h grid rerun and is the next round's call.

## Change
- `code/frigidaire/src/dishsim_frigidaire/dot_rules.py`: `wstar(..., all_sets=False)`: W* for every dot of a non-empty set, not only cv > 0 (dot scores unchanged).
- `code/planner/frigidaire/frigidaire_dot_rules_grid.py`: `--out`; per-cell object-level means for the 7 rules in the partials and in grid.json as `<dish>_obj`; dishes default plate bowl cup.
- new `code/planner/frigidaire/frigidaire_dependency_kit.py` (boot-first Kit): `--sampler restricted|random --loads ...`; entries → world targets through the baseline's extended rack frames; reuses `frigidaire_bench_kit._sequence_scene` / `build_sequence`; writes `data/results/dot_rules/dependency/<sampler>/<id>.json` (status, builds, learned, support, neighbour via `frigidaire_bench.neighbour_order`, goal_order, order, poses_world, built_entries, waves, acyclic); `[RESULT]` line; `release_sim_for_close()`. New Kit-free `frigidaire_dependency_score.py`: packed and built entries under the 7 rules → `<id>.scores.json`.
- `frigidaire_dot_rules_figures.py`: three-rule per-dot (plate, bowl, cup), load, record and spread figures; `fig_depgraph` (layered DAG + top views); `fig_objlevel` (three slices).
- `frigidaire_dot_rules_page.py`: three-rule rules, header and loads sections; explorer toggle, cup, one-line definition of the dropped rules; the two new sections (`--grid-obj-dir`, `--dependency-dir` optional); footer, media and JSON file lists, date 14 October 2026.

## Run
`docker start dishsim-isaac` first (exited 4 days ago). Smallest:
`code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_grid.py --N 1024 --dishes plate bowl cup --out data/results/dot_rules/grid_obj`
`code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --loads r05_00014 --headless`
Per load, after the smoke: write the gate manifest, `frigidaire_initial_state_validate.py --manifest ... --out-dir ... --peak-penetration-m 0.005 --headless`, then the build with `--gate <dir>`.
Then the other 4 loads in parallel (one physical core each), the N = 262,144 grid in the background (about 5.5 h), rescoring, figures, page, two publishes.

## Works when
The smoke grid.json has `cup` and `plate_obj` with O_obj ≠ O in some cell and D_obj = D; `r05_00014.json` has status ok (or a labelled failure) with waves assigned; the page build prints `[RESULT] PASS` and the toggle switches the curves; the artifact is republished.

## Assumed / skipped
- Assumed: the cases table keeps the Original and New columns; nodes coloured by kind; Kit workers on CPU, one physical core each; the explorer payload stays under the 16 MB page limit (zG/zJ dropped for `_obj` entities if needed).
- Skipped this round: the random-sampler run, statistics beyond 5 loads, an Isaac check of the neighbour edges, new tests.
