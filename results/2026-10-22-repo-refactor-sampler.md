# Repo refactor, section 2: the sampler package (first run, Frigidaire + HOTEC only)
Plan: plans/2026-10-21-repo-refactor.md, section 2, scoped by "Decisions on the open items and the scope of the first run".
Branch: `worktree-agent-a33d60e2da8f1b878`, cut from `plan` at eabee14. Agent time: about 2 h (the 90-minute budget ran over).

## What changed (21 files: 19 new, 2 modified)
- New package `code/src/sampler/` (11 modules): `__init__` (`rel`, `run`: one script through `code/util/run_kit.sh` or
  `run_py.sh`, judged by `[RESULT] PASS` and no traceback), `loader`, `slots`, `difficulty`, `initial`, `gate`, `build`,
  `goal`, `graph`, `score`, `sample`.
- `code/src/sampler/schema/{pair,machine,dish}.json` (3): JSON Schema draft-07. The check is a small stdlib validator
  in `loader.py` covering type, required, properties, items and enum. jsonschema is not in the Dockerfile or
  requirements, so it is not used. `machine.json` admits a `basket` container and `dish.json` a `cutlery` kind; both
  schemas say these are unimplemented.
- Manifest writers (2, stdlib): `code/src/sampler/writers/frigidaire_fdpc4221as.py` writes `machine.json`;
  `code/src/sampler/writers/hotec_wheatstraw.py` writes `{plate,bowl,cup}.json`. They belong in
  `data/assets/<asset>/source/` but are staged in the repo; see "What the plan missed", item 1.
- Tests (2): `code/tests/test_sampler.py` (stdlib: the writers regenerate valid manifests; the pair ledger and a
  `--variant` rerun that leaves the baseline alone; the difficulty presets) and `code/tests/test_sampler_slots.py`
  (numpy and pxr, skipped without them: the slot finder against the HOTEC families).
- Modified (2), each with a one-line shim at the old location:
  - `frigidaire_bench.py`: `TIERS`, `TIER_INDEX`, `roster`, `draw_n` and `cap_of` moved to `sampler/difficulty.py`.
    The bench now has `from sampler.difficulty import ...`, so `B.TIERS` is the same dict object as before.
  - `frigidaire_dependency_kit.py`: `physical_graph` moved to `sampler/graph.py`. The kit keeps a one-line lazy
    delegate, because `code/src` joins `sys.path` only after AppLauncher (boot-first is unchanged).

## Mapping lines: done vs not done
| New module | Done | Not done |
|---|---|---|
| `loader.py` | Manifest readers (`read_machine`, `read_dishes`) and the schema validator, both new. The `load` stage reads the manifests, caps the counts by the catalog and finds the sites. | Moving `dishsim_frigidaire/{loading, tableware CATALOG, geometry components+parameters, asset BODY_POSITIONS, random_poses, planner PlannerWorld}`. These are six numpy/pxr modules with a loading <-> geometry import cycle. With no numpy on the Mac, a merged module and its shims could not be import-tested. |
| `slots.py` | Done (new, stdlib). Four modes: between_tines, over_tines, centre_gaps, glass_channel. Baskets are skipped. | None. |
| `initial.py` | Stage `initial`: `frigidaire_bench_kit.py --start --tier --seed --attempt --out <work>` by subprocess, up to `--max-attempts` attempts; `start.json` is stored in the pair. | Moving `initial_state_runtime.py`, `initial_state_candidates.py` and the bench's counter-stack and drop code. |
| `build.py` | Stage `build`. It packs the roster in-process with `frigidaire_dot_rules_restricted.pack_random` (top-5 level-0 families, masks and bans) and then runs `frigidaire_dependency_kit.py --load <id> --restricted-dir <work>` by subprocess. | Moving `frigidaire_bench_kit.build_sequence` and `frigidaire_bench.{insertion_release, upright_pose, lean_side, rack_heights, ceiling cap}`. Variants `build.order/release/plates` are refused (FAIL) unless they are the defaults. |
| `gate.py` | Stage `gate_goal`: `--manifest-out` (run_py), then `frigidaire_initial_state_validate.py --peak-penetration-m 0.005` (run_kit), with the bench's lower-first retry on `closure_failure`. | Stage `gate_initial` records SKIP (see "What the plan missed", item 2). `load_validation.py` was not moved. `gate=settle-only` is refused. |
| `goal.py` | Stage `certify`: `frigidaire_dependency_kit.py --certify --gate <accepted dir>` by subprocess. It stores `valid`, the verdict, the witness order, the confirmations and the access hits. | Moving the kit's `--sample` loop (`physical_loop`) and `write_manifest`. The sampler loop replaces the scheduler for one pair at a time. |
| `graph.py` | `physical_graph` moved, with a delegate in the kit. Stage `graph` (Kit-free) redraws the graph from the stored certify record; `graph=none` stores none. | None. |
| `score.py` | Stage `score`: `frigidaire_dependency_score.py --loads <id>` (run_py, GPU) stores the `dot_rules.score_load` records (packed, settled, built). `score.reference=none` skips it. | Moving the `dot_rules.py` engine and the grid ceilings. The kind-ceiling fair score (`frigidaire_dot_rules_relative.py --reference kind`) is not connected, so the stored score is the raw per-rule S and W. |
| `difficulty.py` | Moved (see above), plus `resolve()`: a preset, then explicit flags override it. | Bosch `dishsim/tiers.py` through the frame flag (out of scope this run). |
| `schema/*.json` | Done. | None. |
| `sample.py` | Done. Every parameter: `--machine --dishes --kinds --count --on-counter --counter-allowance --difficulty --n-samples --max-attempts --max-hours --seed --out --from --to --input --variant`. The pair file `<out>/<machine>__<dishes>__<seed>[__<variant>].json` is rewritten and schema-checked after each stage. The `stages` ledger records stage, params, `git rev-parse HEAD`, UTC time, PASS/FAIL/SKIP and a note. The loop stops a pair at the first FAIL. | None. |
| Manifest writers in `source/` | Written and tested (the test regenerates both manifests in memory). | Not in `data/assets/<asset>/source/`, and `machine.json` / `{plate,bowl,cup}.json` are not written into `data/assets` (see "What the plan missed", item 1). |

## Smoke (Mac, on the worktree; no Kit, numpy or pytest here)
```
$ python3 -m py_compile code/src/sampler/{__init__,loader,slots,difficulty,graph,initial,gate,build,goal,score,sample}.py \
    code/src/sampler/writers/*.py code/tests/test_sampler*.py code/planner/frigidaire/frigidaire_{bench,dependency_kit}.py
PY_COMPILE_OK            (+ the three schema files parse as JSON)
$ python3 code/src/sampler/sample.py --help
usage: sample.py [-h] [--machine MACHINE] [--dishes DISHES] [--kinds KINDS] [--count COUNT] [--on-counter ON_COUNTER]
                 [--counter-allowance COUNTER_ALLOWANCE] [--difficulty {easy,medium,hard}] [--n-samples N_SAMPLES]
                 [--max-attempts MAX_ATTEMPTS] [--max-hours MAX_HOURS] [--seed SEED] [--out OUT]
                 [--from {load,...,graph}] [--to {load,...,graph}] [--input [INPUT ...]] [--variant KEY=VALUE]
$ python3 -m pytest code/tests/test_sampler.py code/tests/test_sampler_slots.py
No module named pytest      (the Mac's python3 is 3.15 with no pytest, and installing it is a download). Run instead with
                            a stdlib stand-in runner (fake pytest: skipif, raises, importorskip, tmp_path, monkeypatch):
code/tests/test_sampler.py::test_manifests_regenerate_and_validate PASSED
[load] PASS 3 kinds, 45 sites
code/tests/test_sampler.py::test_pair_ledger_and_variant_rerun PASSED
code/tests/test_sampler.py::test_difficulty_preset_and_overrides PASSED
code/tests/test_sampler_slots.py: SKIPPED (module) no numpy
== 3 passed, 0 failed, 1 skipped
```
Extra check: both edited scripts were imported with numpy stubbed by `unittest.mock`. `B.roster`, `TIERS`, `cap_of`
and `TIER_INDEX` resolve through the shim (`B.TIERS is sampler.difficulty.TIERS`), and the kit's `physical_graph`
delegate returns the expected edges and layers on a three-dish example.
The finder's sites from the Mac's data/assets: plates `lower_front_00..09` (x <= basket x0 - 20 mm, gap 10 at x .1444
dropped) and `lower_rear_00..08`; bowls `lower_{frontright,rear,mid,rearright}` and `upper_bowl_{00..04,07..11}`; cups
`glass_{left,right}_{0..5}`.

## Box commands for the coordinator (inside the container; `sample.py` is not a Kit script, so use `run_py.sh`, not `run_kit.sh`)
```
# 0. the manifests into the box's data/assets (then copy the writers to data/assets/<asset>/source/write_manifest.py and hf upload)
code/util/run_py.sh code/src/sampler/writers/frigidaire_fdpc4221as.py
code/util/run_py.sh code/src/sampler/writers/hotec_wheatstraw.py
# 1. suites (includes the slot-finder test, which needs numpy and pxr)
code/util/run_py.sh -m pytest code/tests/ code/frigidaire/tests/ -q
# 2. CLI and the Kit-free stage
code/util/run_py.sh code/src/sampler/sample.py --help
code/util/run_py.sh code/src/sampler/sample.py --to load --out data/results/sampler/smoke
# 3. Isaac smoke: one pair. Kit stages run through run_kit.sh in the container. Needs the bench baseline
#    (data/results/benchmark/frigidaire_hotec/baseline/result.json) and the top-5 level-0 masks cache.
code/util/run_py.sh code/src/sampler/sample.py --difficulty hard --n-samples 1 --max-attempts 1 --out data/results/sampler/smoke
# 4. a Kit-free rerun on the stored pair
code/util/run_py.sh code/src/sampler/sample.py --from graph --to graph --input data/results/sampler/smoke/frigidaire_fdpc4221as__hotec_wheatstraw__0.json --variant graph=none
```

## What the plan missed (or what this run found)
1. The worktree agent cannot write the shared checkout's `data/assets`, so the writers are staged in
   `code/src/sampler/writers/`, and `machine.json` / `{plate,bowl,cup}.json` were not written there. The loader reads
   them from `data/assets/<asset>/`, so step 0 above must run before `--to load`.
2. `gate_initial` is not connected. The bench's `start.json` holds world poses only (`initial_snapshot.poses`), while
   the joint gate's manifest needs rack-local entries in the extended racks. The start's own settle, reproduction and
   unstack checks are kept under `initial.settle`, and the stage records SKIP: the plan's "same loaded-rack gate as a
   goal" is still open.
3. The rosters do not match. The goal path (restricted packing and dependency kit) is wired to the top-5 run's 24-dish
   `ROSTER`; `build.py` overrides `T.ROSTER` in-process. The initial path (bench `--start`) takes `--tier/--seed` only,
   and its tiers hold 7 bowls (hard = 23 dishes). So `initial` runs only on a pure `--difficulty` preset (explicit
   `--count/--on-counter/--counter-allowance` FAIL there), and the goal of a hard pair has 23 dishes. Check on the box
   that nothing in the kit assumes 24 (its verdict text says "24 of 24").
4. Not in any asset: the jet angle per spray arm (`exposure.ARM_SOURCES` has centre and radius only; theta 30 is a scorer
   default), so `jet_angle_deg` is null. The food-contact samples and normals file per dish (derived at runtime by
   `register_hotec` from the USD mesh) and the pooling ring are also not written, so both are null.
5. The cup ladder (y0 -0.210 m, pitch 0.085 m, 6 positions), the 20 mm front keep-out margin for plates and the bowl
   zones exist only as code in the HOTEC families. The writers copy them, each with its source named in the manifest.
6. `T.level_masks(..., compute=False)` exits if the top-5 level-0 masks cache is missing on the box (`frigidaire_hotec_top5.py --prepare --level 0`).
7. `sampler` is a generic top-level name. `sample.py` and the scripts prepend `code/src`, but if Kit's python ships a
   module named `sampler`, a path that appends it would pick the wrong one. Check with
   `code/util/run_py.sh -c "import sys; sys.path.insert(0,'code/src'); import sampler; print(sampler.__file__)"`.
8. Next run (the moves not done): move whole modules with a `sys.modules[__name__] = sampler.<module>` alias shim, which
   keeps private names and monkeypatching working. `dishsim_frigidaire/__init__.py` prepends `code/src` so the shims
   import from tests that add only `code/frigidaire/src`. Run it on the box, where numpy can import-test the shims.
