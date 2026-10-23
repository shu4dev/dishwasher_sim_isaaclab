# Run note: bowl-plate edges in the rest-pose DAG, the three-tier rerun (box dishsim-l4, 2026-10-22/23)
From: plans/2026-10-22-bowl-plate-edges.md.

**Ran:**
- Check: `code/util/run_py.sh code/src/sampler/order_check.py --records data/results/dot_rules/dependency/samples_v5/r05_00027.json --models R --out data/results/sampler/bp_r05_check`, then `code/util/run_py.sh -m pytest code/tests/test_sampler_order.py -q`.
- Rerun: `code/util/run_py.sh code/src/sampler/sample.py --difficulty <d> --from build --to certify --seed <s> --n-samples 1 --max-attempts 1 --out data/results/sampler/bp_<d>` (build.order dag-rest, the default).
  - Driven by `data/outputs/sampler_rate/sched_bp.py`: box-side, nohup, 4 slots on cores j,j+4 at nice 10, the three tiers interleaved by seed.
  - Logs: `data/logs/sampler_bp/`. Table: `data/outputs/sampler_rate/tally_bp.md`.
  - The detached Mac driver `driver_bp.sh`, under `caffeinate -dims`, copied the outputs back, restored the clone and stopped the box.

- **Verdict: "Works when" is met.**
  - Medium built 6 of 20 (needed more than 1). Hard built 4 of 20 (needed more than 3).
  - Easy: 11 of the 19 seeds that ran are valid. The 8 that failed are the rest-pose round's 8 failed seeds, with the same reasons. Seed 19, valid before, did not run.
  - Medium gets its first valid pairs: 5 of 20. Hard fell from 2 valid to 1.
  - The box is STOPPED.

- **Per stage, k of 20 seeds: baseline / rest-pose round / now.**
  - Now: seeds 0-18 only. The scheduler's start deadline (95 min after RUNNING, set to keep the 2-hour cap) passed while 4 seeds were still running, so seed 19 of every tier never started.
  - The rest-pose round on the same seeds 0-18: easy 19/0/19/19/15/11, medium 11/0/11/1/1/0, hard 12/0/12/3/3/2 (rows in table order).
  - Baseline = the rest note's "before" column.

| stage | easy | medium | hard |
|---|---|---|---|
| packer found a packing | 20 / 20 / 19 | 12 / 12 / 11 | 13 / 13 / 12 |
| packing had a cycle (any draw) | 8 / 0 / **0** | 12 / 0 / **8** | 13 / 0 / **10** |
| acyclic within 10 draws | 20 / 20 / 19 | 0 / 12 / 11 | 0 / 13 / 11 |
| build PASS | 19 / 20 / 19 | 0 / 1 / **6** | 0 / 3 / **4** |
| joint gate accepted | 16 / 16 / 15 | 0 / 1 / 6 | 0 / 3 / 4 |
| witness rebuilt (valid) | 12 / 12 / **11** | 0 / 0 / **5** | 0 / 2 / **1** |

- **Cycles and DAGs.**
  - Cycles: all 49 cyclic draws (26 medium, 23 hard) were bowl-plate 2-cycles, where a bowl and a plate each cover the other's descent.
  - Draws: medium needed 1-9 per packing, hard 1-10.
  - Hard s14 is the only seed with no acyclic draw: of its 10 draws, 4 were cyclic and 6 found no free pose.
  - Medium's 11 DAGs: 5-25 access edges (5-22 plate-plate, 0-7 bowl>plate, 0-1 plate>bowl), 3-9 layers. Three DAGs have a plate>bowl edge.
  - Hard's 11 DAGs: 4-25 edges (4-22, 0-7, 0-1), 4-9 layers. Three DAGs have a plate>bowl edge. No cup ever got an edge.
  - Easy has no plates, so it has no edges.
  - On all 29 built loads (19 easy, 6 medium, 4 hard), the witness order is a topological order of the DAG.
- **r05_00027 check.**
  - The rest model gives exactly the 9 certified plate hits and no bowl-plate edge at all, so no new edge points against the witness order. Witness breaks: none. Cycle: none.
  - The R test, which now also asserts every access edge against the witness order, passes.
  - The packed-model test still fails, with the same message the DAG-build-order note recorded (`At index 1 diff: ('bowl_05', 'bowl_07') != ('bowl_07', 'bowl_05')`). That code was not touched.
- **Failed seeds:**
  - **Easy, 8**, the same seeds and reasons as before:
    - Gate: s3 too much penetration while the loaded rack retracted; s9 and s13 settle failures; s18 outside the cabinet interior.
    - Certify, each a landing in the witness rebuild: s2 bowl_06 (5 of 7 landed), s12 bowl_03 (6 of 7), s14 bowl_05 (3 of 7), s16 bowl_06 (6 of 7).
  - **Medium, 14:**
    - Packer: 8, the same "no free pose" seeds as before (s1, 4, 6, 7, 8, 10, 15, 17).
    - Build: 5 (s0, 11, 12, 13, 16), placing 12-14 of 15.
    - Certify: 1, s3, whose rebuild stopped at bowl_06 with only 1 of 15 landed.
  - **Hard, 18:**
    - Packer: 7 (s0, 1, 4, 6, 8, 9, 17).
    - Cycle: 1 (s14).
    - Build: 7 (s2, 3, 5, 7, 13, 16, 18), placing 21-22 of 23. s2 and s16 were valid before, but cycles re-drew both packings.
    - Certify: 3, s10 bowl_04 (1 of 23 landed), s12 cup_03 (20 of 23), s15 cup_06 (18 of 23).
- **Blocked by a bowl: 0 of 12 build failures**, against 20 of 21 in the rest-pose round. The rule: an unplaced plate with any try whose descent a bowl or cup blocked. The rest note's 19 counted only the failed plate's own slot.
- **What stops builds now:** in every one of the 12 failures, a bowl never goes in.
  - In 11 of 12, an unplaced bowl's first try was blocked by another bowl. That is a bowl-bowl conflict, which gets no edge by design.
  - In 2, a bowl missed its own landing: medium s0 bowl_05 by 69 mm, and hard s5 bowl_06 by 86 mm.
  - The re-pose then found no pose, and the bowl went to the back of the order.
  - In 6 of 12 (medium s11 and s13; hard s2, s3, s16 and s18), plates the bowl had to precede (bowl>plate edges) went in meanwhile, then blocked its start. The deferral ignores the DAG.

- **Files changed (uncommitted; counts against the rest-pose round):**
  - `code/src/sampler/order.py`: 118 to 119 lines. The rest model keeps every access pair that has a plate in it; docstring updated.
  - `code/planner/frigidaire/frigidaire_dependency_kit.py`: `dag_accept` lost its bowl/cup shortcut, +2/-3 (+51/-11 against 6f39f0c).
  - `code/src/sampler/order_check.py`: 316 to 321 (`bowl_plate`, `bowl_plate_breaks`, printed per load).
  - `code/tests/test_sampler_order.py`: 44 to 46 (witness assert in the R test).
  - `build.py`, `goal.py` and `frigidaire_bench_kit.py` are unchanged this round.
  - Gitignored, in `data/outputs/sampler_rate/`: `sched_bp.py` (new, 68 lines), `driver_bp.sh` (new, 40), `tally.py` (150: bowl>plate and plate>bowl counted apart, plus the bowl-blocked flag), `tally_bp.md` and `driver_bp.log`.
- **Deviations:**
  1. **A kit change the plan did not expect.** `dag_accept` passed every bowl or cup in the rest model ("edges into plates only"). With edges into bowls, that shortcut would let a re-posed bowl cover a later plate's descent. That is what failed the rest round's medium s0: bowl_05, re-posed to lower_rearright, blocked 4 rear plates. So the shortcut is gone, at a cost:
     - Bowl/cup re-pose search took 2,432 s in medium and 5,030 s in hard (before: 92 and 65 s), with single calls up to 486 s.
     - 31 of 89 such calls in medium and hard came back empty (before: 2 of 68).
     - From this one run, the gain cannot be split between the new edges and this check.
  2. Seed 19 of each tier did not run (see the table note).
     - Seed times grew: medians were easy 312 s, medium 359 s and hard 555 s, and hard s10, s11, s15, s16 and s18 each took 1,107-1,655 s.
     - Plate re-pose calls fell from 94 to 3.
  3. A bug in my wait loop: its `pgrep` matched its own command line, so the check's ssh hung, and the box sat idle for about 5 minutes before the scheduler started.
- **Box window (UTC):**
  - **Start:** two `brev start`s stayed STOPPED and were killed (22:42:27 to 22:45:04, and 22:45:07 to 22:47:45). The third (22:47:52) reached STARTING at 22:48:08 and RUNNING at 22:49:19.
  - **ssh and checks:** ssh worked at 22:49:33 after `brev refresh`. The self-stop was running (PID 619) and the clone was clean at 6f39f0c. Files were copied at 22:49:46, and the check and test ran from 22:49:54.
  - **Seeds:** 22:55:29 to ALL DONE at 00:40:56.
  - **End:** copy-back 00:41:06-00:41:13. The clone was restored (0 status lines, 6f39f0c), and `brev stop` went out at 00:41:14, 112 min after RUNNING. STOPPED by 00:50:02 (`brev ls` at 00:50:06: `dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`).
- **Missing:**
  - Bowl-bowl conflicts, the new blocker.
  - A deferral that respects the DAG.
  - Why some witness rebuilds stop after 1 dish (medium s3, hard s10).
  - Seed 19 of each tier.

Suggested commit message: `Add bowl-plate edges to the rest-pose build DAG`
