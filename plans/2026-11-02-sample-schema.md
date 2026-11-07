# A sample is a dataset record: metadata, initial configuration, goal configuration, dependency DAG
Slug: 2026-11-02-sample-schema | From: plans/2026-11-01-merged-benchmark.md (the pair file), results/2026-11-02-three-kinds-medium-easy.md

## Goal
Every stored sample is one file `<out>/<machine>__<dishes>__<roster>__s<seed>.json` with exactly four top-level blocks, metadata, initial, goal and dag, that maps a start configuration to its goal configuration with the certified dependency DAG as heuristics; the algorithms' results live beside it in `<out>/runs/`, and a folder of samples reads as a dataset through `index.jsonl`.

## Decisions (from grilling)
- metadata = params (kinds, count, on_counter, counter_allowance, levels), machine, dishes, seed, variant, sample_id, the stage ledger, a maker summary (method, level, S fit, S ascent, sweeps), provenance (the _work record paths, settle and gate figures, certify verdict, score N / theta / ceilings file) and media (the two stills). Rejected params and ids only (the ledger and stills would be orphans) and keeping the maker's history and timings (they stay in _work).
- initial and goal each hold {dish: world pose} for every dish of the roster in the machine frame (metres, Z up, XYZW) plus the rack joints; initial keeps the counter block (size, top, start count, cap, stacks); goal copies the Isaac-built settled poses from the build record and keeps per dish its rack, slot, variant and rack-local commanded pose, plus score = S_ref, S_area and per-dish S. Rejected the bare {dish: pose} mapping (the episode driver needs the counter cap and the slots) and leaving the goal rack-local only.
- dag = nodes, edges {a, b, kind} from the certified graph only (support or covered insertion path), each dish's layer, and the order the goal was built in. Rejected edges only and carrying the certify evidence (provenance, in _work).
- Algorithm results leave the sample: `<out>/runs/<sample>.json` holds instance, algorithms, own loads, episode rows with videos and the run / video / report ledger entries. Rejected a sibling `.run.json` and the work folder alone.
- `index.jsonl` per folder, one line per sample (file, sample_id, params, levels, S_ref, dag edge count, last stage), rewritten whole whenever a sample is written. Rejected globbing.
- The stills become their own stage `stills` after graph (the kit's two renders into the media folder, paths into metadata.media); `--algorithms none` from today's round retires. Rejected keeping the stills inside the run stage: the run stage writes runs/, the stills belong to the sample.
- Convert v3 (6) and samples (2) in place, Kit-free from their _work records, the schema-1 file kept at `_work/<sample>/schema1.json`; v2 stays history. Rejected regenerating on the box and leaving schema 1 in place.
- Rename pair to sample across code/src/bench, bench.py, the schema and the docs; file names keep their pattern. Rejected sample in docs only.
- The refactorer runs it Kit-free on the Mac; the new writers meet Kit at the next benchmark run. Rejected a box window now.

## Change
- `code/src/bench/schema/sample.json` (replaces pair.json; schema_version 2, the four blocks required) and `schema/run.json`.
- `code/src/bench/sample.py`: the state has the four blocks; `sample_path`, `write` (sample, index.jsonl), `run_path`; STAGES gain `stills` between graph and run.
- Stage writers: `loader.py` (load facts into metadata.provenance), `initial.py`, `build.py`, `gate.py`, `goal.py`, `score.py`, `graph.py` write into initial, goal, dag and metadata; a new `stills.py` calls the kit's two renders; `episodes.py` and `video.py` read the sample and write `runs/<sample>.json`; `report.py` reads samples, index.jsonl and runs/.
- `code/src/bench/convert.py`: `python -m bench.convert <folder>...`: schema 1 plus _work records -> schema 2 sample, runs/ file where a run ran, index.jsonl.
- `code/planner/bench.py`: `--input` takes sample files, `--to stills`, no `none` value; `frigidaire_bench.instance_of_pair` -> `instance_of_sample`, reading the sample's blocks and the _work paths in metadata.provenance; `frigidaire_bench_kit.py --run` loses the `none` branch and gains a `--stills` mode (or the stills stage calls `--run --stills`).
- `code/tests/test_bench.py`: the ledger and variant test asserts the new layout; the level-label test unchanged. Docs: CLAUDE.md (What this is, Commands, Architecture traps), docs/architecture.md, docs/repo_map.txt (the SAMPLER section's sample line and the stills stage row), code/frigidaire/docs/hotec_bench.md.

## Run
Mac, Kit-free, from the repo root with `PYTHONPATH=code/src:code/frigidaire/src .venv-kitfree/bin/python`:
```
-m pytest code/tests/ code/frigidaire/tests/ -q
-m bench.convert data/results/bench/v3 data/results/bench/samples
-c "from bench import report; print(report.write_page('data/results/bench/v3', 'data/media/bench/v3'), report.write_page('data/results/bench/samples', 'data/media/bench/samples'))"
```

## Works when
Every converted file validates against schema/sample.json and has only metadata, initial, goal, dag at the top; initial.poses and goal.poses name every dish of the roster; dag.edges equal the certified graph's edges and dag.order the build order; `runs/<sample>.json` exists for the three v3 pairs that ran, with their episode rows and videos; `index.jsonl` lists each folder's samples; both pages rebuild with the same roster and episode rows as before; `instance_of_sample` on a converted sample returns the same instance as `instance_of_pair` did on its schema-1 copy; pytest 237 passed with the 11 known failures; results/2026-11-02-sample-schema.md written.

## Assumed / skipped
- Assumed: the kit's instance schema (`--run`, the video script) unchanged, fed by `instance_of_sample`; the media folder layout unchanged; the baseline, caches and bans untouched.
- Skipped this round: a box window, a dataset loader class, track B's own loads, the stale "score-greedy" goal-still subtitle (fix it in passing only if the stills mode touches that line).
