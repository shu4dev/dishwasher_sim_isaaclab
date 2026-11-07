# Seat the rear-bank plates: flip the drop bias, gate the landing
Slug: 2026-11-03-rear-plate-drop | From: plans/2026-11-02-fit-score-prove.md (the build stage); results/2026-10-22-plate-rest-pose-dag.md (the rest table that recorded the perch)

## Goal
Every lower_rear plate of a built goal stands in its gap within 20 mm and 16 deg of its packed pose on Isaac Sim 6.2, and the two samples in data/results/bench/samples carry such goals.

## Decisions (from grilling)
- The move: a rear-bank plate's upright release is biased 3 mm toward -x; the front bank keeps +x. Evidence: the 4.5 rest table (results/2026-10-22-plate-rest-pose-dag.md): dropped from +x all 9 rear slots perch at 83 deg on the tine tips (mode 'tines', the pose of every rear plate in every 6.2 goal: 13 of 13), from -x all 9 seat at 120-124 deg, the packed lean; the front seats from either side. Rejected releasing at the packed lean (rewires the insertion model, the rest table and the docs) and a rear y inset (nothing measured says the front's inset is why it seats).
- The landing: a plate must settle within AT_GOAL's dz 20 mm and tilt 16 deg of its commanded pose; lateral keeps DEV_MAX_M (seated plates slide 28-31 mm along the tines). A miss is a failed landing: re-pose, defer, then a loud build failure. Bowls and cups keep the 80 mm rule (their lifted poses drop by design). Rejected keeping the 80 mm rule alone: a perch elsewhere would still become a goal.
- Proof first: the rest table is re-measured on 6.2 into data/results/sampler/plate_rest_62 (both sides, 38 settles) and order.py reads it from then on (a per-stack artifact, like the 6.2 ceilings; the bowl table stays 4.5). Any rear slot perching from -x: the window ends after the measurement, no rebuild.
- Rebuild s0 and s10 in place from the build stage: start poses kept, goal, dag, score and stills redone, the ledger keeps both builds, the perched sample files copied to _work/<sample>/perched_20261103.json first. Rejected a samples_v2 folder and adding the hard v3 s0 (its episodes and videos would go stale).
- Cap 1 h from T0 (box RUNNING) to STOPPED: kill at T0 + 50 min, stop at T0 + 58 min whatever the state.

## Change
- code/planner/frigidaire/frigidaire_bench.py: `move_side(entry)` = lean_side(variant), negated for `lower_rear_*` slots (docstring cites the rest table); `move_local` uses it; the comment block above `insertion_release` says so.
- code/src/bench/order.py: `rest_entries` looks a plate up by (slot, move_side); REST_TABLE -> data/results/sampler/plate_rest_62/plate_rest.json.
- code/planner/frigidaire/frigidaire_dependency_kit.py: the physical build's landing lambda adds the plate dz / tilt rule; the `own` rule string, `landing_tail` and `release_info` (version 2026-11-03, move_pose text) say so.
- Docs: CLAUDE.md (Samples and graph: the bias per bank; Validation status: the perch before this round), code/frigidaire/docs/dependency.md (Move and Landing bullets), data/results/bench/samples/README.md (the new S_ref).

## Run
One box window, from the repo root inside the container. Mac pytest (`.venv-kitfree/bin/python -m pytest code/tests/ code/frigidaire/tests/ -q`) before and after the edits.
1. `code/util/run_kit.sh code/planner/frigidaire/frigidaire_plate_rest.py --headless --out data/results/sampler/plate_rest_62` on cores 0/4, the RTX warm-up (`kit_smoke.py --enable_cameras`) on 3/7 meanwhile. Go on only when every rear entry of side -1 and every front entry is mode 'gap'.
2. Per sample, inside the container: copy the sample file to `_work/<sample>/perched_20261103.json`, then `code/util/run_py.sh code/planner/bench.py --from build --to stills --input data/results/bench/samples/<sample>.json --out data/results/bench/samples` (s0 on cores 1/5, s10 on 2/6).
3. Results, media and logs copied back to the Mac; `brev stop`; STOPPED confirmed.

## Works when
plate_rest_62: 38 of 38 settles, every (rear, -1) and every front entry 'gap'. Both samples: the new build ledger entry PASS, then gate_goal, certify, score, graph and stills PASS; every rear plate within 20 mm / 16 deg of its entry in `_work/<sample>/goal/<sample>.json` (goal_delta); the goal stills show the plates standing in the rack; index.jsonl rewritten; the perched copies exist. results/2026-11-03-rear-plate-drop.md gives the rest-table modes, each plate's dz and tilt, S_ref before and after, the window times and the commit subject.

## Assumed / skipped
- Assumed: `--from build --input` on a stored sample works (never exercised; the start is never re-rolled: if the stage plumbing fails the runner fixes it, no `--from load`); the fit and ascent rerun from the stored start with the same seed, so the new goal may differ from the perched one beyond the plates.
- Skipped this round: the v3 goals and their episodes, videos and report (rebuilt later), the own-load (track B) landing rule, the bowl rest table on 6.2, new tests.
