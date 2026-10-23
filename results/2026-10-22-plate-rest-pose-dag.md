# Run note: lone-plate rest table, the 32-plate check and the three-tier rerun with the rest-pose DAG (box dishsim-l4, 2026-10-22)
From: plans/2026-10-22-plate-rest-pose-dag.md.

**Ran:**
- Settles: `code/util/run_kit.sh code/planner/frigidaire/frigidaire_plate_rest.py --headless --out data/results/sampler/plate_rest` (cores 0,4; `[RESULT] PASS 38 of 38 settles, 29 in the gap`).
- Check: `code/util/run_py.sh code/src/sampler/order_check.py --records data/results/dot_rules/dependency/samples_v5/{r05_00027,r00_00038,r07_00012}.json data/results/dot_rules/dependency/samples_v7/r02_00017.json --models R --out data/results/sampler/plate_rest_check`.
- Rerun, after the user relaxed the gate: `code/util/run_py.sh code/src/sampler/sample.py --difficulty <d> --from build --to certify --seed <0..19> --n-samples 1 --max-attempts 1 --out data/results/sampler/rest_<d>`, `build.order` dag-rest (the default), 4 slots (cores j,j+4, nice 10) by `data/outputs/sampler_rate/sched_rest.py`. Logs are in `data/logs/sampler_rest/`; the table is `data/outputs/sampler_rate/tally_rest.md` (from tally.py).

- **Verdict: "Works when" is met.** Medium built 1 of 20 (needed more than 0) and easy is 12 of 20 valid (needed at least 12). Hard gives the sampler's first valid 23-dish pairs: 2 of 20 (seeds 2 and 16). No draw in any tier had a cycle. The box is STOPPED.
- **38 settles.** Each row is one bank and lean side. Tilt = the plate axis from the rack's up (90 = upright); dx = the origin from the gap mid. The packer's plates all have lean side +1, so the +1 rows are the ones used.

| bank, side | tilt deg | dx, z mm | rest | exceptions |
|---|---|---|---|---|
| front +1 | 120.9-122.2 (gaps 01-07) | +23.3 to +23.5, 118.9-119.4 | gap 10 of 10 | 00: 123.2 deg; 08: 119.0 deg; 09: 117.3 deg (y -177, z 127) |
| front -1 | 122.1-122.3 (02-07) | +23.4/+23.5, 118.9-119.0 | gap 10 of 10 | leans to +x too; 00 119.6, 01 121.7, 08 115.9, 09 120.1 deg |
| rear +1 | 83.34-83.37 (all 9) | -31.4, 187.2 | tines 9 of 9 | none: 6.6 deg away from the lean side, like the loads' rear plates |
| rear -1 | 122.1-122.3 (02-07) | +23.4/+23.5, 118.9-119.0 | gap 9 of 9 | 00 120.2, 01 124.3, 08 116.7 deg |

  Contact was 0.00 mm everywhere. Drift over the last 60 ticks was at most 0.02 mm, except front_08 -1 (0.49 mm) and rear_00 -1 (0.21 mm).
- **32-plate check:** 24 of 32 within 3 deg and 10 mm (the plan asked for 27); the worst is r07_00012 plate_01 at 6.5 deg and 29.7 mm.
  - Bank medians match: front 122.2 deg table vs 122.0 deg settled (0.8 mm); rear 83.4 vs 83.4 deg (0.1 mm).
  - All 8 misses are in tilt, and 31 of 32 plates are within 10 mm.
  - 4 misses are the gap-0 front plates (3.3-5.3 deg): alone a plate there leans 123.2 deg, but in a load it rests on the plate in gap 1 at 118-120 deg.
  - The other 4 are r07_00012 plates 01, 03, 04 and 08. Of r07's five plates that were off the bank median, only plate_06 agrees now.
  - On r05_00027 the rest model gives exactly the 9 certified plate hits, with no cycle.
  - The R test passes (`1 passed`).
  - The user then relaxed the gate to "plate edges exact on r05 and positions within 10 mm", both met (see the plan's last section).
- **Per stage, k of 20 seeds.**
  - Before, easy and medium: the lean-side rate run.
  - Before, hard: the packed-pose DAG run, plus the v7 loop.
  - Cycle rows before: the packed-pose DAG run.

| stage | easy before / after | medium before / after | hard before / after |
|---|---|---|---|
| packer found a packing | 20 / 20 | 12 / 12 | 13 / 13 |
| packing had a cycle (any draw) | 8 / **0** | 12 / **0** | 13 / **0** |
| acyclic within 10 draws | 20 / 20 | 0 / **12** | 0 / **13** |
| build PASS | 19 / 20 | 0 / **1** | 0 / **3** |
| joint gate accepted | 16 / 16 | 0 / 1 | 0 / 3 |
| witness rebuilt (valid) | 12 / **12** | 0 / 0 | 0 (v7 loop 0 of 40) / **2** |

- **DAG stats.** All DAGs had no support edges (off by design) and were acyclic at the first draw.
  - Easy: no edges, 1 layer.
  - Medium: 5-22 access edges (4-17 between plates, 0-7 from a bowl into a plate), 3-8 layers.
  - Hard: 7-29 access edges (4-22 between plates, 2-8 from a bowl), 5-9 layers.
  - On all 24 built loads, the witness order is a topological order of the DAG.
  - Median seed time: easy 300 s, medium 168 s, hard 301 s (longest 1,062 s). All 60 seeds ran in 67 min.
- **Failed seeds:**
  - **Easy, 8.** Gate 4: s3 too much penetration while the loaded rack retracted; s9 and s13 settle failures; s18 outside the cabinet interior. Certify 4, each a landing in the witness rebuild: s2 bowl_06 (5 of 7 landed), s12 bowl_03 (6 of 7), s14 bowl_05 (3 of 7), s16 bowl_06 (6 of 7).
  - **Medium, 20.** Packer 8 (s1, 4, 6, 7, 8, 10, 15, 17), the same "no free pose" seeds as before. Build 11 (s0, 3, 5, 9, 11, 12, 13, 14, 16, 18, 19), placing 11-14 of 15. Certify 1: s2's witness rebuild stopped at plate_03 with 8 of 15 landed.
  - **Hard, 18.** Packer 7 (s0, 1, 4, 6, 8, 9, 17). Build 10 (s3, 5, 7, 10, 12, 13, 14, 15, 18, 19), placing 20-22 of 23. Certify 1: s11's rebuild stopped at bowl_04 with 16 of 23 landed.
- **What stops medium and hard now:** a plate's descent into its own slot is blocked by a bowl placed before it. This explains 19 of the 21 build failures (16 of them rear-bank plates, rear_00 to rear_07, under the rear bowls), and the re-pose did not clear it. The rest model leaves out exactly this edge, because bowls take no edges. The two other build failures:
  - Medium s16: bowl_07's start is blocked by bowl_04.
  - Hard s13: plate_05 landed with a 1.09 mm bowl_07/bowl_04 overlap.
- **Files changed (all uncommitted; counts against 6f39f0c):**
  - `code/planner/frigidaire/frigidaire_plate_rest.py`: new, 146 lines.
  - `code/src/sampler/order.py`: 97 to 118 lines (`REST_TABLE`, `plate_rest_poses`, `rest_entries`, `pose_model="rest"`).
  - `code/src/sampler/order_check.py`: 303 to 316 (model R, bank medians).
  - `code/tests/test_sampler_order.py`: 30 to 44 (the R case).
  - `code/src/sampler/build.py`: +38/-13 against 6f39f0c (`dag-rest` is the default, `dag` stays selectable; the order file carries `pose_model` and `strict`).
  - `code/planner/frigidaire/frigidaire_dependency_kit.py`: +52/-11 (`--order` reads `strict` and `pose_model`; `dag_accept(pose_model)`).
  - `frigidaire_bench_kit.py` and `goal.py` are unchanged this round.
  - Gitignored: `data/outputs/sampler_rate/{sched_rest.py,tally.py (access edges split into plate pairs and the rest),driver_rest.sh,tally_rest.md}`.
- **Deviations:**
  1. A lone plate has no packed target, so its settle deviation is measured from the move pose.
  2. The kit reads `pose_model` and `strict` from the order file instead of new flags.
  3. Job order: easy and medium interleaved by seed, then hard. All 60 seeds ran, so this changes no count.
  4. How the window ended: the Mac slept from 20:13 to 20:51 and again until 21:48, so the detached Mac end-driver stalled. The box's own self-stop shut it down after 30 idle minutes, and the outputs came back in a second, 8-minute window.
- **Box windows (UTC):**
  - **Check window:** RUNNING 19:26:29; `brev stop` 19:31:29; STOPPED by 19:38:46.
  - **Rerun window, start:** the first `brev start` (19:40:55) stayed STOPPED, so I killed it at 19:43:13. The second reached STARTING at 19:44:18 and RUNNING at 19:45:01. ssh worked at 19:45:16 (the self-stop was running, PID 632).
  - **Rerun window, run:** files copied and compiled at 19:45:30. Seeds ran 19:45:36 to ALL DONE at 20:52:32. The self-stop shut the OS down at 21:22:30, 97.5 min after RUNNING. The box was STOPPED by 21:48:39; a late `brev stop` then cycled it through STOPPING to STOPPED at 21:52:06.
  - **Fetch window:** a start at 21:52:27 stayed STOPPED, and the next (21:54:43) reached RUNNING at 21:56:33. Copy-back ran 21:56:49-21:56:55. The clone was restored (0 status lines, 6f39f0c) and `brev stop` sent at 21:56:55. STOPPED by 22:04:10 (`brev ls`: `dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`).
- **Missing:**
  - Edges into bowls: a plate before a lower-rack bowl that covers its descent. This needs a bowl landing pose, or at least the bowl's packed pose trusted for plate paths.
  - The gap-0 chain lean.
  - Why the gate and certify fail at the rate they do (8 of 20 easy, as before).

Suggested commit message: `Add plate rest table and rest-pose DAG build order`
