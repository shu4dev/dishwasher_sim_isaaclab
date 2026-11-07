# Manifests as the single source on the Frigidaire side
Superseded 2026-11-01 by plans/2026-11-01-merged-benchmark.md: one machine and one dish set, no onboarding to serve.
Slug: 2026-11-01-frigidaire-manifests | From: research/2026-11-01-general-assets.md (runs after plans/2026-11-01-stack-upgrade.md)

## Goal
Every Frigidaire-twin and HOTEC fact the code uses comes from `data/assets/<asset>/machine.json` and `<kind>.json` (schema v2), no python constant is left, and three Kit-free outputs on stored records are byte-identical before and after.

## Decisions (from grilling)
- Declared modes: the manifest names the mode per container and dish kind; a new rack feature or dish family is a new mode in code, one round each. Rejected sampling instead of slots (a new benchmark definition, not a refactor) and stopping at the flags.
- Two cache-key schemes: manifest assets hash their manifest plus CoACD params; the Bosch keeps its frozen anchors and 14 caches. Rejected one scheme: it rebakes the Bosch for no result.
- Declared kinds now: each `<kind>.json` carries its rules as data (contact, pooling, insertion, chain); kind-free methods are a later score revision. Rejected kinds as code (nothing moves) and kind-free now (changes published numbers).
- Frigidaire side first, one schema for both stacks; the Bosch core follows in its own round. Rejected both at once (two proofs in one long round). Proof is Kit-free byte identity on stored records; Kit stages are not re-run here (6.2 moved their numbers anyway).
- Borrowed from the quick check (DishwasherBench, BEHAVIOR-1K): every measured number keeps a status tag (measured, nominal or estimated, as HOTEC `parameters.json` already does); rules are `{name: {param: value}}` blocks.
- Tiers live in `data/assets/frigidaire_fdpc4221as/tiers/hotec_wheatstraw.json`, learned per machine and dish set. Rejected a repo preset. The writers move to each asset's `source/write_manifest.py` and read only that asset's `parameters.json`, `components.json` and `usd/`; the code reads `machine.json` alone. Rejected deleting them (hand-kept manifests drift). Rejected a Kit gate as proof (10 min of box time for values the Kit scripts only read).

## Change (rows: the 2026-11-01 Frigidaire audit)
- `code/src/sampler/schema/machine.json` v2: add `spray_arms` [{name, centre_m, radius_m, serves}] (typed today in `exposure.py:52-60`), `ceiling_point_m`, `tub` {solids, closed_liner_inner_y_m}, `retraction` {order, speed_mps} (`initial_state_runtime.py:28, 509`), one `counter` (the code has 1.8 and 1.2 m).
- `code/src/sampler/schema/dish.json` v2: add `rules` {contact: lathe | cutlery, pooling: {rim_depth_m}, insertion: upright_lean | packed, chain: lean_side | none}, `rim_m`, `foot_m` (`robot/grasp.py:23-33` holds pre-v4 values), `at_goal_tol` (`frigidaire_bench.py:39-47`); keep `allowed_orientations`.
- `data/assets/frigidaire_fdpc4221as/tiers/hotec_wheatstraw.json`: TIERS (`difficulty.py:8-19`), counter stacks and pitch (`frigidaire_bench_kit.py:33-46`, `frigidaire_bench.py:39-47`).
- Readers switch to the manifests: `paths.py:11-16` takes the machine name; `exposure.py:52-60, 182`; `random_pose_runtime.py:23-32`; `initial_state_candidates.py:21-30`; `frigidaire_bench.py:35-47, 598`; `difficulty.py`; `dot_rules.py:631-632` and the dot_rules scripts (`grid.py:101`, `relative.py:52-87`, `figures.py:34-36`, `render.py:27-29`); `frigidaire_hotec_load.py:46-66` and `frigidaire_hotec_exposure_search.py:136-177` take the families' numbers from `allowed_orientations`.
- `code/src/sampler/writers/*.py` move to `data/assets/{frigidaire_fdpc4221as,hotec_wheatstraw}/source/write_manifest.py`, read only the asset's own files and emit v2; the AST-parsed repo constants go.
- Stale values move with the data: the README's rack travel, `grasp.py` bowl numbers, one counter width, the bench docstring's slacks, and the bench mask cache key becomes family digest plus manifest hash (`frigidaire_bench.py:125-137`).

## Run
Mac, Kit-free, once before and once after the change, then `diff`:
```
python3 code/src/sampler/sample.py --from load --to load --input data/results/sampler/bp_easy/frigidaire_fdpc4221as__hotec_wheatstraw__0.json --out /tmp/manifests_check
python3 code/planner/frigidaire/frigidaire_dot_rules_relative.py --grid-payload data/media/dot_rules/page_v22/index.html --grid-obj-dir data/results/dot_rules/ceiling_inward/N0262144 --dependency-dir /tmp/manifests_check/samples_v5 --summary
python3 -m pytest code/tests/test_sampler*.py code/frigidaire/tests/ -q
```

## Works when
The load stage's sites, `r05_00027.relative.json` (S and ceilings) and the pytest counts are identical before and after, and the audit's B-row literals (0.914, 0.233, 0.191, door_hinge, lower_slide, upper_slide, the TIERS counts) appear in code/ only inside the schema folder and the tests.

## Assumed / skipped
- Assumed: the v2 manifests belong to the HF dataset; the upload after the round is yours (`hf upload`).
- Assumed: the Kit scripts only read the same values; the runner checks them by import and `--help` on the Mac and does not boot Kit.
- Skipped this round: the Bosch core (next round, same schema), the kind-free score, cutlery and basket modes, tub and door occlusion, more than one arm per rack, the robot block.
