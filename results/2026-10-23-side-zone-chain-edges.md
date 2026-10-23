# Run note: side-zone chain edges, upper bowls at the table, the rerun (box dishsim-l4, 2026-10-23, stopped early)
From: plans/2026-10-23-side-zone-chain-edges.md. **Verdict: worse than the bowl-plate round. Medium went from 5 to 3 valid and hard from 1 to 0, so neither bar is met. Why is below. The coordinator stopped the round during easy (the user changed priority) and the box was left RUNNING, as asked.**

**Ran:** check `code/util/run_py.sh code/src/sampler/order_check.py --models R --out data/results/sampler/sz_check --records <21 certified records>` (r05_00027, rest_easy 12, rest_hard 2, bp_medium 5, bp_hard 1; bp_easy repeats rest_easy byte for byte), the test, then `sample.py --difficulty <d> --from build --to certify --seed <s> --n-samples 1 --max-attempts 1 --out data/results/sampler/sz_<d>` for medium, then hard, then easy, from `data/outputs/sampler_rate/window_sz.sh` and `sched_sz.py` (box-side nohup, 4 slots on cores j,j+4 at nice 10). Logs are in `data/logs/sampler_sz/`, the table in `data/outputs/sampler_rate/tally_sz.md`.

**Deviation 1, the chain direction.** I used `SIDE_ZONE_CHAIN = -1` (the larger packed x goes in first), not the plan's +x. `access_hits` records a hit (a, b) as "a must go in before b". Read that way, a has the larger x in 11 of 13 unique same-zone side-zone hits (18 of 20 with the repeats), and the witness order respects all 20. The chain-check note read its b as the one going first. It is one constant to flip back.

**Witness check:** `[CHAIN] 8 of 20 side-zone chain edges against the witness order (7 of 15 records); side-zone hits 11 held, 2 reversed, 0 at equal x, of 13`. With +x it would be 12 of 20 against the witness (10 records), 2 hits held and 11 reversed. The chain edges the witness breaks are pairs with no hit, where either order worked. The upper-rack path edges equal the certified upper hits on all 4 records that have any. Test: `1 failed, 2 passed`. The new chain test (the witness breaks 0 chain edges on r05) passes. The failure is the old packed-model test, with the same message as in earlier notes.

| stage, seeds 0-19 (before = bowl-plate round, seeds 0-18) | medium after (0-18) / before | hard after (0-18) / before |
|---|---|---|
| packer found a packing | 12 (11) / 11 | 13 (12) / 12 |
| packing had a cycle in at least one draw | **10 (9)** / 8 | **13 (12)** / 10 |
| acyclic within 10 draws | 11 (10) / 11 | 11 (10) / 11 |
| build PASS | **4** (4) / 6 | **1** (1) / 4 |
| joint gate accepted | 3 (3) / 6 | 1 (1) / 4 |
| witness rebuilt (valid) | **3** (3) / 5 | **0** (0) / 1 |

- **Easy, partial:** 8 seeds finished (0-4, 6-8). They gave build 8, gate 8, valid 6 of 8. The bowl-plate round on the same seeds gave 8 / 7 / 6. The coordinator's stop killed seeds 5 and 9, which had built and were in the gate stage, and seeds 10 and 11, which were in build. Seeds 12-19 never started.
- **Valid pairs.** Medium: s12, s14, s18. Before it was s2, 5, 9, 14, 18. Hard: none; before it was s11.
- **Gate and certify losses.** Medium s11 failed the gate: outside the dishwasher. Hard s15 failed certify at cup_06 with 18 of 23 landed, the same as before.
- **Cycles.** There were 78 cyclic draws (49 before), 21 of them with a bowl-bowl link (a chain or upper edge). Medium s9 and hard s7 and s14 found no acyclic draw (before, only hard s14). The extra re-draws mean most seeds built a different packing from the bowl-plate round.
- **DAGs (final draws):**
  - Medium: 8-25 access edges (5-22 plate-plate, 0-9 bowl>plate, 0-1 plate>bowl, 0-3 side-zone chain, 0-2 upper bowl-bowl); 0-3 side-zone pairs at equal x, which get no edge; 3-9 layers.
  - Hard: 8-22 (4-15, 0-9, 0-2, 0-3, 0-3); 0-3 equal-x pairs; 4-8 layers.
  - Easy: 0-4 edges (chain 0-3, upper 0-2).
  - Every built load's witness order is a topological order of its DAG.

**Build failures, by the zone that blocked (medium 7, hard 10; before 5 and 7):**
- **Lower front-right, 7 (4 before): the chain causes them.**
  - 4 times the chain put the larger-x bowl first and it blocked the other bowl's descent: medium s0 (dx 100 mm, a Y105 bowl tilted along x), s3 (dx 5 mm), s16 (dx 5 mm), hard s18 (dx 10 mm).
  - 3 times two bowls sat at equal x, so there was no edge, and the seeded order put the blocker first: medium s13, s19, hard s12.
  - In 6 of the 7 the two bowls are within 10 mm in x and 10-71 mm apart in y: they sit side by side along y. The certified hits the chain holds are 15-105 mm apart in x.
- **Rear row, 6 (6 before; no bowl-bowl edges by design).**
  - Hard s3, s13 and s16: a front-right bowl was re-posed into the rear row and blocked a rear bowl.
  - Hard s2: two rear bowls at the same x.
  - Medium s2 and hard s5: landing misses of 84-86 mm.
- **Other zones, 4.**
  - Rear-right, hard s19: the only bowl in its zone missed its landing three times (90-101 mm), then an upper bowl blocked its start.
  - Upper, hard s11: bowl_04 missed by 28-40 mm, with a 16 mm overlap.
  - Plates, medium s5 and hard s10: a landing miss of 33-35 mm, then deferral behind plates the plate has to precede.
- **What this says.** Along x, the chain holds only where bowls are at least 15 mm apart in x. For bowls side by side along y the order is not set by x. Those pairs need their own rule (y or tilt) or no edge.

**Box window (UTC):**
- Start: `brev start` at 06:07:58 worked first try. STARTING at 06:08:13, RUNNING at 06:09:03. ssh after `brev refresh` at 06:09:17. The self-stop (PID 610) and the container were up, and the clone was clean at f2f0246.
- Setup: `git pull` was already up to date. Code was copied at 06:09:29. The bowl table on the box was identical (same sha256), so it was not copied. The check ran 06:09:39-06:10:01 (21 s), and the seeds started at 06:10:01.
- Stop: at 07:39 the coordinator stopped the round. I killed the Mac end driver, the scheduler (PID 3358) and the sample.py trees for easy seeds 5, 9, 10 and 11, all by PID, by 07:40:05. The tally was written and copy-back finished at 07:40:35.
- Hand-over: the clone was restored at 07:40:45 (0 status lines, f2f0246 = origin/plan), and nothing of mine is running. `brev ls` at 07:40:45 gave `dishsim-l4  RUNNING  COMPLETED  READY`. The box is **left RUNNING**: 92 min RUNNING so far, no `brev stop`. The @reboot self-stop is still armed and halts the box after 30 idle minutes.
- Seed times (median / max): medium 277 / 1,159 s, hard 547 / 1,667 s, easy 311 / 474 s.

**Files changed (uncommitted; counts against f2f0246):**
- `code/src/sampler/order.py`, 119 to 168 lines: `SIDE_ZONE_CHAIN`, `chain_edges`; the bowl table for upper bowls and cups only; lower-rack bowl pairs replaced by the chain.
- `order_check.py`, 321 to 435 lines (bowl round included): the chain count and the `[CHAIN]` line. The bowl gate now judges upper-rack pairs only.
- `test_sampler_order.py`, 46 to 68 lines: the r05 chain and witness-count test.
- `frigidaire_plate_rest.py`: as the bowl round left it.
- Gitignored, in `data/outputs/sampler_rate/`: `sched_sz.py`, `window_sz.sh` and `driver_sz.sh` (new); `tally.py` (chain/upper split, equal-x pairs, a ZONE per build failure, cycle links).

**Missing:** easy seeds 5 and 9-19, an order rule for side-by-side (same x) bowls, and the rear row's order test.

Suggested commit message: `Add bowl rest table and side-zone chain DAG edges`
