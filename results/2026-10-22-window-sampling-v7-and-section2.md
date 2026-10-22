# Run note: the 06:10 to 08:30 ET window of 2026-10-22 (section 2 refactor + samples_v7), box dishsim-l4
From: plans/2026-10-21-repo-refactor.md (section 2), results/2026-10-22-inward-ceilings-and-smokes.md (the sampling loop).
The user was away; every decision below is the session's.

## What ran, in parallel
1. **Section 2 refactor** (Mac, refactorer agent, worktree off eabee14): 19 minutes of agent time, commit 27d4f52
   `Add the sampler package: schemas, manifest writers, slot finder, staged CLI` (21 files, +1231/-56), fast-forwarded
   into `plan` at 06:36 ET and pushed. Its own note: results/2026-10-22-repo-refactor-sampler.md (done: the CLI with
   every parameter, the pair ledger, `--from/--to/--input/--variant`, three schemas with a stdlib validator, both
   manifest writers, the slot finder and its family test, the `difficulty` and `graph` moves with shims; not done: the
   mechanical moves into loader/initial/gate/build/score and the kind-ceiling score; `gate_initial` records SKIP).
   Before the run the plan's three open items were decided by the session (section 3 first: done; difficulty = a
   preset from the bench's table, explicit flags override; schema admits `basket` and `cutlery`, unimplemented) and the
   first run scoped to the Frigidaire + HOTEC path, "connect, do not delete" (commit eabee14).
2. **samples_v7** (box): the cut kit's `--sample` loop over the untried packings of `sequential_fresh_order.txt`
   (pilot r00_00064; 38 packings tried by rounds 1-6 skipped), THREE Kit slots on cores 0-2 (siblings 4-6) and the
   Kit-free slot on core 3, via `data/outputs/samples_v7/sched3.py` (the 8-vCPU port of the scheduler:
   `KIT_CORES=(0,1,2)`, `PY_CORES=(3,)`, `taskset j,j+16 -> j,j+4`), `--target-valid 4 --max-packings 40`, deadline
   12:05 UTC. Launched 10:16:03 UTC. RAM with three Kit sessions: 3-4 of 30 GB.
3. **Section-2 checks on the box** (`data/outputs/samples_v7/smoke_s2.sh`, core 3, nice 10, beside the loop): step 0
   wrote `data/assets/frigidaire_fdpc4221as/machine.json` (6,323 B) and `data/assets/hotec_wheatstraw/{plate,bowl,cup}.json`
   (both `[RESULT] PASS`); `import sampler` resolves to `code/src/sampler/__init__.py` (no Kit collision);
   `sample.py --help` exit 0; `sample.py --to load` `[load] PASS 3 kinds, 45 sites`, `[RESULT] PASS 1 of 1 pairs`.
   Then `pytest code/tests/ code/frigidaire/tests/ -q` and the one-pair Isaac smoke
   (`sample.py --difficulty hard --n-samples 1 --max-attempts 1 --max-hours <to 12:00 UTC>`), results below.

## Results
- **Suites on the merged code (box, 10:37 UTC):** `pytest code/tests/ code/frigidaire/tests/ -q`: 263 passed, 24 subtests
  passed, 64 s (the slot-finder test included, numpy and pxr present in the container).
- **Isaac smoke, seed 0 (10:37-10:43 UTC):** `[load] PASS 3 kinds, 45 sites`; `[initial] PASS attempt 0: 17 on the
  counter` (the bench's `--start`, hard tier); `[gate_initial] SKIP` (not connected, start.json holds world poses only);
  `[build] FAIL packing: no free pose for plate_08`; `[RESULT] FAIL 0 of 1 pairs, stages load..graph`. The pair file
  with its four-entry ledger is `data/results/sampler/smoke/frigidaire_fdpc4221as__hotec_wheatstraw__0.json`. The build
  failure is the refactorer's item 3: the restricted packer and the kit are wired to the top-5 run's 24-dish roster while
  the bench's hard tier gives 23 dishes (7 bowls), and `build.py` overrides the roster in-process. The retry with
  `--seed 1` (10:45-10:51 UTC) ends the same way, `[build] FAIL packing: no free pose for plate_08`, so the failure is
  structural (the roster), not the draw.
- **Goal path through the new CLI (10:52-11:00 UTC):** `sample.py --count plate=8,bowl=8,cup=8 --from build --to graph
  --seed 7` (the 24-dish roster, the 23-dish `initial` skipped) packs its own restricted load and runs the kit's build:
  `[build] FAIL build status resettle_failed` (the sampler placed fewer than 24, as in the loop), so gate_goal, certify,
  score and graph are still unexercised through the CLI; seeds 8 and 9 (11:00-11:01 UTC) failed in the restricted packer itself
  (`no free pose for bowl_08` with seed 9), so the packer with a fresh `T.ROSTER` override fails often; seeds 10 to 20
  ran next until the deadline (below).
- **samples_v7, final (10:16-11:46 UTC, 90 min, 3 Kit slots):** `[RESULT] STOP --sample: 0 valid of 40 attempted`.
  39 packings failed in the sampler (20-23 of 24 placed, one dish's landing, 28-52 attempts each), 1 invalid:
  r02_00017 built all 24 and passed the joint gate (accepted, 5.3 min) but its witness did not rebuild in a fresh Kit
  process (landing at bowl_02, 8 of 24 landed). Throughput: 40 packings in 90 min, about 6.5 min per main job per slot.
  The same failure mode and rate as rounds v5 and v6b (17-18 of 20 failed there): the valid yield of the current
  re-posing sampler is about 1 in 40 to 60 packings, so five valid samples need a sampler change, not more box hours.
- **Dependency graphs page Version 16** (11:50 UTC, index.html only, the nine r05_00027 media kept): the yield table now
  holds 60 packings (the 2026-10-16 run's 20 and samples_v7's 40): 1 valid, 57 sampler failures, 1 joint-gate failure,
  1 witness failure. r05_00027 stays the only (image, graph, score) tuple. Built by `finish_v7.sh` (scratchpad):
  rsync, `frigidaire_dot_rules_relative.py` over samples_v5 + v7, the merged state
  `data/logs/dot_rules/samples_v7/scheduler_state_merged.json`, `frigidaire_dependency_page.py` into
  `data/media/dot_rules/dependency_page_v7/`.
- **samples_v7b (11:48-12:08 UTC):** the next 12 untried packings from r10_00005, same scheduler:
  `[RESULT] STOP --sample: 0 valid of 10 attempted`: 7 failed in the sampler, 3 killed at the deadline, 2 never started.
  Page Version 17 (12:10 UTC): 70 packings in the yield, still 1 valid (r05_00027).
- **Goal path through the CLI, seeds 7 to 19 (10:52-11:52 UTC, spare core):** 12 attempts, none past `build`: 6 ended
  `build status resettle_failed` (the kit's sampler), 6 in the packer (`no free pose for <dish>`); `[RESULT] FAIL` each.
  So `sample.py` runs load, initial, the packer and the kit's build end to end; gate_goal, certify, score and graph have
  not run through it yet (they need a packing that builds).
- **Yield across rounds:** v5 1 of 20, v6b 1 of 20 (the same sample), v7 0 of 40, v7b 0 of 10: about 1 valid in 90
  packings of the re-posing sampler, every failure one dish's landing. Five valid samples need the sampler to handle
  the landing failures (r07_00012's bowl_04 case, the loop's 20-23 of 24), not more box hours.

## Box window (UTC)
`brev start` 10:11:13, RUNNING 10:13, ssh 10:13, samples_v7 10:16:03-11:46:49, section-2 checks 10:36-10:43, smoke
retry 10:45-10:51, goal-path seeds 10:52-11:52, samples_v7b 11:48:10-12:08:07, copy-back 12:09, `brev stop` 12:09:52.
About 2 h of L4 (~$2.35). RAM never above 5 of 30 GB with three Kit sessions and a fourth on the spare core; root disk
78 %. The self-stop cron was running from boot.

## Files
- Repo: `code/src/sampler/` (new package), `code/tests/test_sampler*.py`, shims in `frigidaire_bench.py` and
  `frigidaire_dependency_kit.py`, `plans/2026-10-21-repo-refactor.md` (decisions), this note and the refactorer's.
- Box only (gitignored): `data/results/dot_rules/dependency/samples_v7/`, `data/logs/dot_rules/samples_v7/`,
  `data/media/dot_rules/dependency_samples_v7/`, `data/logs/sampler_smoke/`, `data/results/sampler/smoke/`,
  `data/outputs/samples_v7/{sched3.py,run_v7.sh,smoke_s2.sh}`, the four manifests under `data/assets/` (not yet on the hub).
