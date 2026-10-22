# Run note: plan-branch cleanup (2026-10-22, plan `plans/2026-10-21-plan-branch-cleanup.md` with its 2026-10-22 update)

- **Branch:** `worktree-agent-a1f214059350c52a4`, off `plan` at `1ebdc1d`. One commit per step: `5427da3` delete the four
  unused dot-rules scripts, `30f7975` dependency page, `7bdc3dc` dependency kit, `d4dcdbb` `build_sequence`, `9a50446`
  bench constants, `b353bcc` clips, `0565eaa` relative score, `cbfc2f9` Dot Score Rules page and figures, `109a75b` docs,
  then this note. The page commit comes before the kit commit so that no commit leaves the page importing a deleted
  function.
- **Works (Kit-free, on the Mac):** both pages rebuild from data. Dependency graphs is byte for byte Version 13, the
  index and all 9 media files. Dot Score Rules is Version 23 except the 10 lines whose numbers only the box's full grid
  holds (listed below); fed V23's numbers, those three sections match V23 too. The kit's fake-job scheduler check
  passes. The scheduler sends the same jobs at the same fake times, reaches the same stages and writes the same log as
  before the cut. The kind pass gives identical output on copies of the data. No deleted flag, function or script is
  referenced in code or docs (9 grep hits, all other scripts' names, explained below). Nothing ran on the box.
- **Not done:** the box smoke (the coordinator's next window, commands below).

## Files (lines before -> after)

| File | Before | After |
|---|---|---|
| `code/planner/frigidaire/frigidaire_dependency_kit.py` | 3065 | 1183 |
| `code/planner/frigidaire/frigidaire_bench_kit.py` | 1385 | 1267 |
| `code/planner/frigidaire/frigidaire_bench.py` | 2121 | 2090 |
| `code/planner/frigidaire/frigidaire_dependency_clips.py` | 480 | 382 |
| `code/planner/frigidaire/frigidaire_dependency_page.py` | 1264 | 617 |
| `code/planner/frigidaire/frigidaire_dot_rules_relative.py` | 551 | 242 |
| `code/planner/frigidaire/frigidaire_dot_rules_page.py` | 1661 | 702 |
| `code/planner/frigidaire/frigidaire_dot_rules_figures.py` | 1403 | 308 |
| `code/planner/frigidaire/frigidaire_dot_rules_bench.py` | 329 | deleted |
| `code/planner/frigidaire/frigidaire_dot_rules_isaac.py` | 50 | deleted |
| `code/planner/frigidaire/frigidaire_dot_rules_tail.py` | 111 | deleted |
| `code/planner/frigidaire/frigidaire_dot_rules_loads.py` | 174 | deleted |
| `code/planner/frigidaire/frigidaire_dependency_score.py` | 74 | 74 (kept: the loop's score job, see "missed") |
| `CLAUDE.md` | 323 | 320 |
| `code/frigidaire/docs/exposure.md` | 282 | 303 |
| `code/frigidaire/docs/dependency.md` | new | 127 |
| `README.md` | 131 | 131 (names no deleted script) |

15 files changed, +686 / -6,344 lines. The planner folder went from 24 scripts and 16,537 lines to 20 and 10,734.
Deleted: 4 scripts and 145 top-level definitions: 43 in the kit, 1 in the bench kit, 5 in the bench, 6 in the
dependency page, 13 in the relative score, 22 in the Dot Score Rules page and 55 in the figures module (lists in
`grep_deleted.py`'s output below). In `build_sequence`, the parameters `initial`, `together`, `stop`, `backtrack`,
`goal_table`, `contact_m` and the backtracking helpers are gone.

## What changed, per script

- **`frigidaire_dependency_kit.py`:** keeps only the sampler (the `--edges physical` branch of the old `pairs()`, now
  `sampler()`), `--manifest-out`, `--certify` and the `--sample` loop with `--resume-from`.
  - Removed flags: `--sampler`, `--farm-dir`, `--out`, `--edges`, `--goal`, `--release`, `--plate-order`, `--tolerance`,
    `--backtrack`, `--learn-rounds`, `--library`, `--order-index`, `--merge-library`, `--refine`, `--rejudge`, `--records`,
    `--retry-of`, `--rerun-backtracked`, `--perturb-mm`, `--perturb-deg`, `--perturb-seed`.
  - `--out-dir` is now required. Before, a physical run without it fell into the retired `pairs_resettled/` folder.
  - Insertion, lean-side, backtrack 0 and the bench tolerance are wired in.
  - The module docstring describes the live method and the exact commands the scheduler runs.
- **`frigidaire_bench_kit.py`, `build_sequence`:**
  - Removed: the backtracking (`back_track`, `moved_by_undo`, `BACKTRACKS_PER_DISH`), the joint `together=` units,
    `stop=` (library builds), `initial=` (the learned set seeded from dependency graphs), and the sampler's learning hook
    (the "swaps' extra candidates" added to `learned`), along with `goal_table` / `contact_m`. Those two were the
    bench defaults in every live call.
  - Kept: `fixed=` (`--certify` rebuilds the witness with it) and the clear release (see "missed").
- **`frigidaire_bench.py`:** removed `AT_GOAL_LIBRARY`, `CONTACT_LIBRARY_M` and `RELEASE_RULE`, together with its
  never-run "gap" branch, `clearance` and `RELEASE_CLEAR_M`. `release_blockers` is always the 1 mm allowance, as before.
- **`frigidaire_dependency_clips.py`:** physical records only. Removed: the swap-test witness, `load.jpg`, edge and
  conditional clips, `--only`, and `segment(ghost=)`. Kept: `--views` access stills.
- **`frigidaire_dependency_page.py`:** keeps only the physical page. Removed: the swap-test page, the library and
  checker data, the `judge_order` import from the kit, `--failures`, `--failed-clip`, `--screen`, `--failure-note`, and
  the "rescaled S" caption of the retired `--reference best`. The old template is reduced to its CSS (`STYLE`, byte for
  byte, since V13 embeds it).
- **`frigidaire_dot_rules_relative.py`:** keeps only the kind pass and `--summary`. Removed: the per-dish engine
  reference (facing), `--reference best`, `--reference`, `--force` and the numpy/bench import. The kind functions are
  unchanged, `main` excepted.
- **`frigidaire_dot_rules_page.py`:** builds the published trimmed page: the template's publish wrapper is dropped, the
  object-level, goal-load, Isaac, dependency and appendix sections are gone, and the lead, rules, explorer prose,
  extra CSS and footer are V23's.
  - New: `--grid-payload PAGE`, read with `read_payload`. On the Mac, without `--grid-dir` / `--grid-obj-dir`, the
    numbers that only the full grid holds print as `nan` or `–`.
  - The dish-pose count now comes from the payload. Every rule's array marks a skipped pose -1, and 62,080 matches.
  - No numpy import. The figures module imports numpy optionally, the pattern `relative.py` already used.
- **`frigidaire_dot_rules_figures.py`:** keeps only `dots_<dish>.png` and the readers the page uses. Removed:
  `--sweep`, `--loads-dir`, `--farm-dir`, `--dependency-dir`, `--resettled-dir`, `--only`, and the convergence,
  random-load, object-level and depgraph figures with their readers.
- **Docs:**
  - CLAUDE.md: "Dot Score Rules page" becomes "Pages and the fair score". It gives both URLs and versions, the score
    with its three ceilings and the pooling rule, the sampler and graph, and the rebuild and publish commands. The
    2026-09-29 cleanup section is now one line about the hold folders.
  - New `code/frigidaire/docs/dependency.md`: the method, the commands (box loop, per-load jobs, Mac score and page),
    results on record, landmines (session history, the overlap rule, the gate flake, `--resume-from`, the kind-score
    gap, rsvg) and history.
  - `exposure.md` gains "The fair score across kinds" beside revision 5.

## Checks (outputs)

**1. py_compile:** Python 3.15.0 on the Mac, every changed script:
```
py_compile OK frigidaire_dependency_kit.py
py_compile OK frigidaire_bench_kit.py
py_compile OK frigidaire_bench.py
py_compile OK frigidaire_dependency_clips.py
py_compile OK frigidaire_dependency_page.py
py_compile OK frigidaire_dot_rules_relative.py
py_compile OK frigidaire_dot_rules_page.py
py_compile OK frigidaire_dot_rules_figures.py
```
Two more checks:
- All 20 planner scripts parse under `/usr/bin/python3` 3.9.6 (`3.9.6: 20 of 20 parse`). That is stricter than the box's
  Kit 3.10.
- An ast scan finds no undefined name in any changed script. The unused names it lists predate this cut: `math` in the
  bench kit and `TILTS_PERDOT` in the figures module.

**2. Dependency graphs page:**
```
$ python3 code/planner/frigidaire/frigidaire_dependency_page.py --records r05_00027=data/results/dot_rules/dependency/samples_v5/r05_00027.json \
    --clips r05_00027=data/media/dot_rules/dependency_samples_v6/r05_00027 --state data/logs/dot_rules/samples_v6b/scheduler_state_merged.json --out <scratch>/dep_final
[RESULT] PASS page <scratch>/dep_final/index.html: 1 valid samples (r05_00027), 20 packings in the yield, 28526 B, 9 media files, 2453018 B
diff index.html vs published Version 13 (unwrapped): no differences
media: all 9 files byte-identical to the files published in Version 13
```
- The published file is the page plus the artifact's publish wrapper (`<!doctype html>...<body>\n` ... `\n</body></html>`).
  It was unwrapped before the diff, and the unwrapped text equals `data/media/dot_rules/dependency_page_kind/index.html`.
- `rsvg-convert` 2.58.4 is not deterministic run to run. Two earlier rebuilds, one by the pre-change code and one by
  the new code, gave two arcs PNGs that differed from the published ones in 6 of 5.9 M pixel bytes. The final rebuild
  matched byte for byte.

**3. Dot Score Rules page:**
```
$ python3 code/planner/frigidaire/frigidaire_dot_rules_page.py --grid-payload data/media/dot_rules/page_v22/index.html \
    --figures-dir data/media/dot_rules/page_v22/media --out <scratch>/dot_final
[page] N 262,144 from data/media/dot_rules/page_v22/index.html; dishes plate, bowl, cup; object level yes; no grid level: the header tiles, the rules' cases and the per-dot captions print nan or –; index.html 1.12 MB; total 2.18 MB
[RESULT] PASS page <scratch>/dot_final/index.html
```
Diff against the published Version 23, unwrapped: 10 of 577 lines differ, and only in these spans (rebuilt -> published):
```
line 173: 'nan' -> '0.806'; 'nan' -> '0.336'                     header tile: plate on edge, Original -> New
line 174: 'nan' -> '0.938'; 'nan' -> '0.998'                     header tile: plate facing the arm
line 175: 'nan' -> '0'; 'nan' -> '0.999994'                      header tile: dots above 1, largest New dot (62,080 matches)
line 200: 'nan' -> '0.928'                                       rules: N x DJ at 30 deg
line 207: '–' -> '5.6·10⁻⁶'                                      rules: case 1
line 208: 'nan' -> '0.50'                                        rules: case 2
line 209: 'nan' -> '1.000'                                       rules: ideal
line 260, 264, 268: '–' -> the per-dot means (Original, New, New+J at tilt 0, 90, 180), "New and New+J below 0.01" and
          "On edge the <dish> keeps X % ... and Y % under New+J" of the plate, bowl and cup
```
- **Not reproducible from the data on the Mac:** these numbers come from `grid/N0262144` (or `grid_obj/N0262144`):
  - the header tiles need `grid.json`'s O and N arrays and `convergence.json`;
  - the rules' cases need the dot entity's D, N and DJ at 14 cm;
  - the per-dot captions need `perdot_<dish>.npz`.
  
  None of these is on the Mac. The saved payload holds only NJ, zJ, the object-level NJ and the dot's nJ. Nothing was
  hand-patched. On the box, `--grid-dir` / `--grid-obj-dir` computes them with unchanged code.
- **Harness:** a scratch harness feeds header(), rules() and perdot() the numbers V23 shows (values that round to them)
  and gets back V23's text exactly:
  ```
  header: identical to V23 (1575 chars)
  rules: identical to V23 (4185 chars)
  perdot: identical to V23 (1997 chars)
  [RESULT] PASS header, rules and per-dot reproduce V23 given its numbers
  ```
- Everything else is identical: the head, the explorer with the ceilings plate 0.872, bowl 0.548, cup 0.213, the
  footer and its version lines, the CSS, the `window.GRID` payload (a JSON round trip of V22's, equal to V23's) and the
  explorer JS. The three PNGs are identical to the published ones.

**4. The kit's fake-job scheduler check:**
- Source: `data/logs/dot_rules/samples_v5/check_physical.py`, adapted in a scratch copy, since data/ is read-only.
- Adaptations:
  - `import_sample(src, tmp)` takes no tolerance;
  - the args Namespace drops `plate_order` / `backtrack` / `tolerance`;
  - the argv assertions become "the kit with `--out-dir` and no mode flag" (no `--sampler`, `--edges`, `--release`,
    `--plate-order`, `--backtrack`, `--tolerance`) and "the gate's command carries `--out-dir <run>` and
    `--peak-penetration-m 0.005`";
  - a `DUMP` env var writes each scenario's jobs, stages and log.
- The original check passed on the original kit (baseline).
- The adapted check on the new kit:
```
  ok   force sign calibrated to -1 on 8 dish-rack rows
  ok   supports: p2 on p1, b3 on p2; the sideways b3-c4 touch is none
  ok   no dish-rack rows: the centre rule ({'bowl_03': ['plate_02'], 'cup_04': ['bowl_03']})
  ok   drawn: p1>p2 (support and access), p2>b2 (access)
  ok   b1>b2 support vs b2>b1 access: a conflict, neither drawn
  ok   against the witness: c1>p3 (support), c2>c1 (access)
  ok   b1's either-plate support kept as a disjunction, not drawn
  ok   layers: p2 above p1, b2 above p2
  ok   free = in no drawn edge
  ok   import once, then kept
  ok   imported record: sampler fields, witness traj, scores
  (per scenario plain / lower / uncertified / deadline: todo, the imports' gates first, "main and certify: the kit with
   --out-dir and no mode flag", "gate: manifest of the built poses, then validate at 5 mm", and the scenario's own
   checks: three valid, clips for each, no score for the imports, all 18 new packings started; closure_failure ->
   lower first -> certify on it; a witness that did not rebuild: invalid, its clips still made; deadline at 1500 s:
   running packings killed (2); all ok)
[RESULT] PASS check_physical
```
Before against after, the old kit with the original check against the new kit with the adapted one. The old argv was
compared with its mode flags dropped, and the old log with "--sample --edges physical" read as "--sample":
```
plain: 30 jobs, 88 log lines; stages same, reasons same, jobs same, log same; state keys only in one: ['backtrack', 'plate_order', 'tolerance']
lower: 31 jobs, 91 log lines; stages same, reasons same, jobs same, log same; state keys only in one: ['backtrack', 'plate_order', 'tolerance']
uncertified: 32 jobs, 92 log lines; stages same, reasons same, jobs same, log same; state keys only in one: ['backtrack', 'plate_order', 'tolerance']
deadline: 28 jobs, 84 log lines; stages same, reasons same, jobs same, log same; state keys only in one: ['backtrack', 'plate_order', 'tolerance']
[RESULT] PASS scheduler identical before/after (mode flags aside)
```

**5. The kind-ceiling score:** the 2026-10-21 command, run on scratch copies of `loads/N0262144`,
`restricted/scores_N0262144`, the samples_v5 records and page_v22. The old code ran with `--reference kind`, the new
without:
```
[RESULT] PASS kind ceilings: 278 files, 280 states, 6720 dishes, ceilings at theta 30 plate 0.8718 / bowl 0.5481 / cup 0.2134 (...page_v22/index.html); 2306 pooling (scored 0), 0 clipped at 1; in 0 s
log lines old 17 new 17: same
278 .relative.json rewritten: 278 identical, 0 different (kind_utc, ceilings_source aside)
[RESULT] PASS kind pass identical before/after
```
The comparison normalizes three things: the wall time, the `[RESULT]` label (`--reference kind:` became `kind
ceilings:`), and the group path relative to ROOT. Both runs print PASS, not the PARTIAL of 2026-10-21: the Mac's
farm and restricted score records now carry their pooling lists.

**6. grep for deleted flags, functions and scripts in code and docs:** a scratch scanner (`grep_deleted.py`) checks
code (114 `.py` files) and docs (38 `.md` files outside plans/, results/, research/). In code it looks for every
`alias.name` and `from module import name` of the 145 deleted definitions, and for every import of a deleted script.
In code and docs it looks for every deleted flag on a line that names its script, or anywhere for a flag no kept
script defines, and in docs for every backticked deleted function name.
```
scanned 114 .py files and 38 .md files
code/planner/frigidaire/frigidaire_dependency_kit.py:966: flag --out (deleted from frigidaire_dependency_kit): clips = ["code/planner/frigidaire/frigidaire_dependency_clips.py", "--record", ..., "--out",
CLAUDE.md:231: flag --set (deleted from frigidaire_dot_rules_isaac): ... rebuild with `--set kind.name=value` (the
docs/hotec_wheatstraw_asset.md:62, 73, 167, 168, 384: flag --set (deleted from frigidaire_dot_rules_isaac): `--set kind.mass_kg` ...
code/util/README.md:30: `media` (deleted from frigidaire_dependency_page): - `dishsim`: ... `media`
code/frigidaire/docs/hotec_bench.md:221: `failure_of` (deleted from frigidaire_dependency_page): ... (`failure_of`: `pooling`)
[RESULT] HITS 9 reference(s) to deleted names, flags or scripts
```
All 9 hits are other scripts' names:
- `--out` is the clips script's own flag.
- `--set` is `hotec_gen.py`'s.
- `media` is the `dishsim.media` module.
- `failure_of` is `frigidaire_bench.failure_of` of the HOTEC bench.

A text grep for backtrack, together=, judge_order, rejudge, learn_round, perturb, AT_GOAL_LIBRARY, RELEASE_RULE,
sample_loop, record_folder, gate_failed, library_run, merge_library, `--library`, `--plate-order`, `--tolerance`,
`--release` and `--sampler` finds only these:
- the published page text "no backtracking";
- the log text "goal built one dish at a time";
- the `rejudged` record field that `--resume-from` reads (see dependency.md);
- dependency.md's history;
- the Bosch "perturbed" instances.

One brace-form mention is left on purpose: the V23 footer's `frigidaire_dot_rules_{bench,grid,figures,page}.py`. It is
kept verbatim in `FOOTER` for byte identity, with a comment to drop `bench,` at the next version.

## Box smoke for the coordinator (repo root on the box, this branch merged into `plan` and pulled)

```bash
code/util/run_py.sh -m pytest code/tests/ -q
code/util/run_py.sh -m pytest code/frigidaire/tests/ -q          # test_frigidaire_bench.py imports frigidaire_bench

# 1. the sampler loop, one packing (r07_00012, the pilot, first in sequential_fresh_order.txt): its main job took 5.9
#    and 8.3 min on 2026-10-16; should it build all 24, gate, certify, clips and score follow inside --max-wall-seconds.
#    Pass: a clean run, whatever the outcome (samples_v6b placed 23 of 24 there): data/logs/dot_rules/smoke_20261022/scheduler.log ends with
#    "[RESULT] STOP --sample: ..." or "[RESULT] PASS --sample: ..."; r07_00012_main.log has "[RESULT] PASS r07_00012",
#    no Traceback, no "free(): invalid pointer"; data/results/dot_rules/dependency/smoke_20261022/r07_00012.json exists.
setsid nohup python3 code/planner/frigidaire/frigidaire_dependency_kit.py --sample \
    --out-dir data/results/dot_rules/dependency/smoke_20261022 --target-valid 1 --max-packings 1 --max-wall-seconds 2400 \
    > data/logs/dot_rules/smoke_20261022.out 2>&1 &

# 2. --certify on a copy of the valid sample (build_sequence(fixed=), forces, access, confirmations, graph): 8.6 min on
#    2026-10-16. Pass: "[RESULT] PASS r05_00027 joint gate accepted (...), witness rebuilt 24 of 24 in a fresh Kit
#    process; ..." (2026-10-16: 1 hard support and 11 access edges). "[RESULT] PASS r05_00027 the witness did not rebuild
#    in a fresh Kit process: ..." is a PhysX outcome (sessions differ), not a code failure; a FAIL or a Traceback is.
mkdir -p data/results/dot_rules/dependency/smoke_20261022_certify/_gate
cp data/results/dot_rules/dependency/samples_v5/r05_00027.json data/results/dot_rules/dependency/smoke_20261022_certify/
cp -r data/results/dot_rules/dependency/samples_v5/_gate/r05_00027 data/results/dot_rules/dependency/smoke_20261022_certify/_gate/
code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --load r05_00027 \
    --out-dir data/results/dot_rules/dependency/smoke_20261022_certify --certify \
    --gate data/results/dot_rules/dependency/smoke_20261022_certify/_gate/r05_00027 --max-wall-seconds 2400 --headless

# 3 and 4 need the HOTEC bench's instances (made on corallab 2026-09-29: easy s0-s5, medium and hard s0-s2). The Mac's
#    data holds only frigidaire_hotec/baseline/; check `ls data/results/benchmark/frigidaire_hotec/instances/` on the box
#    first. Without them the clear release has only the Kit-free cover of test_frigidaire_bench.py.
# 3. one HOTEC bench episode (BenchOracle and its clear release; release_blockers lost a never-run switch).
#    Pass: "[RESULT] PASS easy_s0" and data/results/benchmark/frigidaire_hotec_smoke_20261022/episodes/goal/easy/
#    easy_s0__greedy_offline.json (its own success is the physics; PASS means the episode ran).
code/util/run_kit.sh code/planner/frigidaire/frigidaire_bench_kit.py --run \
    --instance data/results/benchmark/frigidaire_hotec/instances/easy/easy_s0.json --algorithms greedy_offline \
    --out data/results/benchmark/frigidaire_hotec_smoke_20261022 --headless

# 4. the HOTEC sequence gate (build_sequence's clear-release path, which this cut edited), on a COPY of an accepted
#    attempt (it rewrites sequence.json and may append bans.json): any instances/attempts/<tier>_s<seed>_a<k>/ holding
#    start.json, goal.json and gate/result.json; copied as root, since Kit-written folders are root-owned.
#    Pass: "[RESULT] PASS built in N build(s), ..." (or REROLL with a reason), no Traceback.
docker exec -w /workspace/dishsim dishsim-isaac cp -r data/results/benchmark/frigidaire_hotec/instances/attempts/<tier>_s<seed>_a<k> \
    data/results/benchmark/frigidaire_hotec_smoke_20261022_sequence
code/util/run_kit.sh code/planner/frigidaire/frigidaire_bench_kit.py --sequence data/results/benchmark/frigidaire_hotec_smoke_20261022_sequence --headless
```
Judge every Kit run by its log: `[RESULT] PASS`, no Traceback, no `free(): invalid pointer`.

## What the plan missed

1. **`frigidaire_dependency_score.py` is used.** The `--sample` loop queues it as the "score" job of every valid
   sample, and it writes `<id>.scores.json`. The page's caption, the kind pass and `import_sample` all read that file.
   The plan's reason, "the kit scores samples itself", is not true, so the script stays unchanged.
2. **The clear release is the HOTEC bench's live move.**
   - `BenchOracle._release` (every goal move of an episode) calls `B.clear_release` and `B.near_poses`.
   - `sequence_gate` / `sequence_plan` (`--sequence`, `--sequence-plan`) call `build_sequence` with its default
     `release="clear"`.
   
   Deleting `clear_release` and "the clear release" would break the bench, which the 2026-10-22 update keeps intact.
   Deleted instead: the kit's `--release clear` mode and the bench's never-run `RELEASE_RULE` "gap" switch (with
   `clearance`, `RELEASE_CLEAR_M`). `near_poses` and the `RELEASE_STEP/CAP/NEAR_M` constants stay. Side note:
   hotec_bench.md still says the sequence gate releases at "+ 3 mm". That has been the lowest clear hover since
   2026-10-14 (HOTEC docs were out of scope).
3. **`fixed=` stays in `build_sequence`.** `--certify` rebuilds the witness with it. Of the "fixed-order library builds",
   only `stop=` went.
4. **A new sample can no longer get a kind-ceiling score.** The kind pass only rescales the per-dish New+J saved in a
   `.relative.json`, and the retired per-dish engine reference pass was the only thing that wrote those files. All 280
   current states have one. A new sample's page caption will show the absolute S until the kind pass builds its per-dish
   list from the `.scores.json`: `score_load`'s `per_dish` carries id, kind, area and E.NJ. That is about 10 lines and
   was not added. Section 2's `score.py` is the place.
5. **The Dot Score Rules page cannot be built from the payload alone.** The plan lists "the grid payload, the per-dot
   PNGs and the three ceilings" as its inputs. The header tiles, the rules' cases and the per-dot captions need the box's
   `grid.json` (all seven rules), `convergence.json` and `perdot_<dish>.npz`. `--grid-payload` therefore prints nan
   there on the Mac, and the box path computes them.
   
   Two more facts about this page:
   - No script had generated Versions 21-23 before. V21 was a hand trim, and V22 and V23 were patches of the saved HTML.
   - The old script and the figures module imported numpy at module level, so neither could run on the Mac.
6. **The V23 footer names `frigidaire_dot_rules_bench.py`**, as `{bench,grid,figures,page}`, which this plan deletes.
   The text is kept for byte identity; drop `bench,` at the next version.
7. **Flags.** The plan named dead modes. I deleted every flag that no longer selects anything:
   - in the kit: `--sampler` (restricted only), `--edges` (physical only), `--release` (insertion only), `--plate-order`
     (lean-side only), `--tolerance` (bench only) and `--goal`;
   - in `relative.py`: `--reference` and `--force`.
   
   Effects:
   - Old command lines in plans/ and results/ (the records) no longer run as written. The `--sample` entry parses with
     `parse_known_args`, so the old loop command still starts the loop; a hand-run Kit job must drop the flags.
   - The scheduler state loses `plate_order`, `tolerance` and `backtrack`.
   - Records keep `"sampler": "restricted"`, `"tolerance": "bench"`, the insertion `release` dict and
     `plate_order: lean-side` as method labels, and the rule texts are unchanged.
8. **Record fields only the deleted modes wrote are gone.** No live reader uses them; I checked the page, clips,
   certify, manifest, loop, score and scheduler check.
   - The sampler record loses `resettle_initial`, `original`, `reposing.backtrack`, `reposing.backtracks` and
     `witness.learned`.
   - A failed sampler record loses the `--edges build` placeholders: `builds`, `learned`, `support`, `neighbour`,
     `goal_order`, `order`, `poses_world`, `built_entries`, `n_waves`, `n_waves_dependencies` and `acyclic`.
   - `build_sequence` steps lose `together`, and the sampler's build loses `backtracks`.
   - `_kit/<id>/states.json` (the swap states) is no longer written.
   - The `[RESULT]` lines lose "goal built " and "--edges physical".
9. **CLAUDE.md lost one live landmine.** The cleanup section also carried `package_frigidaire.py --check` (red pending
   the two-job evidence refresh) and the `hf upload` caveat. The latter is still covered by "evidence and results stay
   out of the dataset". The package-check note now lives only in the asset folder's `source/`, which I could not see
   from the Mac; re-add it if it still holds. Its "untracked" bullet was stale: `mcts.py`, `robot/` and
   `docs/scoring_note/` are tracked now.
10. **No index lists the new doc.** Neither README.md's docs table nor `code/frigidaire/README.md` mentions
    `dependency.md`, and `code/planner/frigidaire/README.md` lists none of the dependency and dot-rules scripts. The
    plan limits README edits, so I left them.
11. **The scheduler check's data has moved on.** The adapted check copies `samples_v3` / `samples_v3_r07` / `samples_v4`
    from the Mac's data, as before. The `samples_v6*` folders and `scheduler_state_merged.json` (the V13 yield) exist
    only as outputs of the earlier runs.

## Decided without the user

- **Flags (missed item 7):** a flag with one value left is deleted, not kept as a one-choice option.
- **`--out-dir` is required for the kit.**
- **`--grid-payload` without a grid level builds the page with nan / – for the box-only numbers.** The alternative was
  copying those sections from the saved page. It was rejected, because then the check would pass by construction (the
  coordinator's "instead of hand-patching").
- **The dependency page's "rescaled S" caption branch (`S_rel_best`) is deleted.** It belongs to the retired
  `--reference best`, and V13 shows the kind S.
- **The clips script now refuses a non-physical record with a message**, instead of running the deleted swaps path.
- **Commits were reordered once** (page before kit), and two docstring fixups were folded into their step commits.

Scratch files (session scratchpad `/private/tmp/claude-501/-Users-brianshu-workspace-dishwasher-sim-isaaclab/9ad01507-6abc-472e-acf3-ffb6c2319ffa/scratchpad/`):
- the adapted check: `check_physical_adapted.py`;
- the comparison tools: `compare_dumps.py`, `compare_relruns.py`, `dot_sections_harness.py`, `grep_deleted.py`;
- the published pages, unwrapped: `published_dep/page.html`, `published_dot/page.html`.

A worktree-local `data` symlink to the main checkout's data (gitignored) served the reads; nothing under data/ was
written.
