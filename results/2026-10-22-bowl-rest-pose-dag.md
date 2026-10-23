# Run note: lone bowl and cup rest table, the bowl check and gate (box dishsim-l4, 2026-10-23)
From: plans/2026-10-22-bowl-rest-pose-dag.md. **Verdict: the gate failed (5 of 12 loads, the limit is 2), so the rerun did not start, as the plan says. The box is STOPPED.**

**Ran** (from `data/outputs/sampler_rate/window_br.sh`, box-side, under nohup):
- Settles: `code/util/run_kit.sh code/planner/frigidaire/frigidaire_plate_rest.py --headless --kinds bowl,cup --part j --parts 4 --out data/results/sampler/bowl_rest` on cores j,j+4 at nice 10. Each part printed `[RESULT] PASS 30 of 30 settles`. The `--merge` step printed `[RESULT] PASS 4 parts, 120 of 120 settles`.
- Check and gate: `code/util/run_py.sh code/src/sampler/order_check.py --models R --out data/results/sampler/br_check --records <21 valid records>`. It printed `[GATE] FAIL bowl gate failed on 5 of 12 loads (s0, s4, s5, s8, s11; stop when more than 2)` and `[RESULT] PASS 21 checks, 0 errors, 19 s`.
  - The 21 records: r05_00027, rest_easy s0/1/4/5/6/7/8/10/11/15/17/19, rest_hard s2/16, bp_medium s2/5/9/14/18 and bp_hard s11. bp_easy repeats rest_easy.
- Test: `code/util/run_py.sh -m pytest code/tests/test_sampler_order.py -q` gave `1 failed, 2 passed`. The new r05 bowl case and the R plate case pass. The failure is the untouched packed-model test, with the same message as in the earlier notes.

**Table** (`data/results/sampler/bowl_rest/bowl_rest.json`, 120 keys, 0 blocked starts): one key per (zone, orientation, y). Tilt is the dish axis measured from the rack's up: 180° is mouth down, 90° is on its side.

| zone | keys (masked candidates) | rest tilt, deg (median) | tipped onto the side (<100°) | settle deviation, mm (median) |
|---|---|---|---|---|
| lower front-right | 32 (604) | 88.7-179.1 (107.8) | 14 | 1.9-112.2 (22.9) |
| lower rear | 20 (969) | 134.8-172.8 (141.7) | 0 | 4.9-113.3 (25.8) |
| lower rear-right | 15 (212) | 88.7-154.7 (140.7) | 5 | 5.3-53.3 (23.5) |
| upper centre bowls | 29 (29) | 10.0-146.4 (134.7) | 10 | 2.5-67.6 (12.2) |
| cups, left and right | 24 (24) | 162.7-177.1 (167) | 0 | 9.3-25.5 (17.6) |

- Six lower-rack rows were still moving in the last 60 ticks. The worst is front-right Y135_y-0.1695, at 18.9 mm and 13.8°.
- One row ends at 6.0 mm contact.

**Check:** settled bowls and cups against their table entry, counted as k of n within 3° and 10 mm.

| record set | bowls | cups |
|---|---|---|
| r05_00027 | 4 of 8 | 5 of 8 |
| easy | 29 of 84 | none in the tier |
| medium | 17 of 35 | none in the tier |
| hard | 10 of 21 | 19 of 24 |

- By zone: upper bowls 52 of 58, cups 24 of 32, lower-rack bowls only 8 of 90 (front-right 4 of 37, rear 3 of 43, rear-right 1 of 10).
- In 20 of the 90 lower-rack bowls, the lone bowl tips onto its side while the loaded one stays near 140°. Example: X150_y-0.1390, where the lone bowl rests at 88.7° and the loaded bowls at 141-147°.

**Gate:** 7 of 12 pass, on r05_00027 plus the 11 valid easy loads that have bowl-bowl hits.
- r05_00027's 2 bowl hits come out exact and acyclic.
- Each of the 5 failures has a spurious reverse edge that closes a 2-cycle:
  - s0: the record has bowl_04>bowl_02; the rest model (R) adds bowl_02>bowl_04.
  - s4 and s5: each also misses 1 edge.
- The bowls involved are lower-rack bowls whose table pose is 35-66 mm and 17-58° away from their settled pose.
- Medium and hard, for information: hard s2, s11 and s16 pass; medium s5 and s14 fail the same way.
  - Medium s2's record has both bowl_05>bowl_07 and bowl_07>bowl_05. An acyclic model cannot reproduce that 2-cycle.
- Overall, R is exact on 13 of 21 records. Plate pairs are exact on all 21.

**Before and after:**
- Before (the bowl-plate round, seeds 0-18), as packer / acyclic / build / gate / valid: easy 19/19/19/15/11, medium 11/11/6/6/5, hard 12/11/4/4/1. Cycle row: easy 0, medium 8, hard 10.
- After: not run, because the gate stopped the round. So there are no failed seeds and no DAG stats.

**Files changed** (against f2f0246):
- `frigidaire_plate_rest.py`: 146 to 212 lines. Adds `--kinds bowl,cup`, `--part/--parts` and a Kit-free `--merge`.
- `sampler/order.py`: 119 to 145 lines. Adds `BOWL_TABLE`, `bowl_key` and `bowl_rest_poses`; `rest_entries` now covers bowls and cups; the move pose stays the packed pose; the rest model keeps every pair.
- `order_check.py`: 321 to 399 lines. Adds the bowl errors, the bowl gate, its table and the `[GATE]` line.
- `test_sampler_order.py`: 46 to 59 lines, for the r05 bowl case.
- Gitignored, under `data/outputs/sampler_rate/`: new `window_br.sh`, `sched_br.py` (never started) and `driver_br.sh`; `tally.py` now has the bowl-bowl label and a BOWL-BY-BOWL count.

**Deviations:**
1. For lower-rack bowls, x was collapsed as well as the lift. With x kept, the 1,785 masked candidates on a 5 mm grid would need far more than the planned 60-150 settles. Each key settled once at its median x, and `order.py` shifts that row by the entry's x minus the row's x.
2. The lift was removed before each move: the descent stops where the appliance stops it and releases 20 mm higher.
3. The settles ran as 4 parallel Kit parts and took 3.3 minutes.
4. The check's per-record files collided for s2, s5 and s11, which exist in two tiers. `table.md` is complete. After the run I renamed the files to `<tier>__<load_id>.<model>.json`; that change was not rerun.

**Box window (UTC):**
- `brev start` at 03:13:01 worked on the first try; RUNNING at 03:15:17; ssh worked after `brev refresh`. The self-stop was running (PID 688). The clone was pulled from 6f39f0c to f2f0246 and the files copied at 03:15:35.
- Settles 03:15:43 to 03:19:04, check 03:19:24, ALL DONE 03:19:39.
- The Mac driver copied the outputs back from 03:21:46 to 03:21:57 and restored the clone (0 status lines, f2f0246). `brev stop` went out at 03:21:59; STOPPED at 03:29:31 (`brev ls`: `dishsim-l4  STOPPED  COMPLETED  NOT READY  i0z5jl21z  g6.2xlarge  L4`); the window ran 16 min from start to STOPPED and 6.7 min RUNNING to stop.

**Missing:** the three-tier rerun, and a rest model for lower-rack bowls that holds inside a load. The lone-bowl assumption fails there.

Suggested commit message: `Add bowl and cup rest table and bowl-bowl DAG edges`
