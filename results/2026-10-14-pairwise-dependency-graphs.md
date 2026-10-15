# Run note: pairwise dependency graphs (one edge type from ordered-pair drop tests)

Plan: `plans/2026-10-14-pairwise-dependency-graphs.md` (slug `2026-10-14-pairwise-dependency-graphs`), from
`plans/2026-10-13-object-level-rules-dependency-graphs.md` (run note `results/2026-10-13-object-level-rules-dependency-graphs.md`).
Run 2026-10-14 on the Brev box (1x L4), container `dishsim-isaac`. Short ids: p = plate, b = bowl, c = cup (p03 = plate_03).
"X last" is the drop L(X) into the full goal load; "X w/o Y" is X dropped with Y also removed, R(Y, X); "a > b" is an edge,
a before b.

## Stage 1: pair tests, certifying builds, rescore (2026-10-14)

Run 08:52:59-09:50:35 UTC. Four Kit sessions ran at once, one per gate-accepted load (j = 0, 1, 2, 3 for r11_00050,
r07_00023, r00_00031, r02_00048), each pinned inside the container to CPUs j and j+16 at nice 10 with 2 OMP threads,
launched with nohup at 08:52:59 and finished 09:26:26 (r00), 09:28:40 (r11), 09:29:13 (r02) and 09:35:20 (r07). Container
PIDs (Kit python / isaaclab.sh): r11 4978 / 4944, r07 4968 / 4927, r00 4958 / 4921, r02 4955 / 4915; host docker-exec
clients 627334-627337. Per load: the Kit-free candidate pairs (FCL, collision pieces within 20 mm at the jointly settled
poses, any rack), 24 last-drops plus 2 removal drops per pair, each a 150-tick context re-settle and a 210-tick drop from
3 mm above the target, the edges, then for an acyclic graph `build_sequence(initial=<the graph's edges>)`. r05_00014 (j 4)
wrote its gate record at 08:53:11 without booting Kit. The rescore ran from 09:36:17 (container PID 9784). No fix, no rerun.
From about 09:28 another session ran 8 more Kit sessions in the same container (`--out-dir .../pairs_more`, and r02_00048
`--goal built` into `pairs_resettled/`) on CPUs 4-12 and 20-28, plus a second rescore on `pairs_resettled` that shared the
GPU with mine; none writes to `pairs/`. The kit script on disk was edited at 09:20 (adds `--out-dir`, `--goal`); the four
sessions ran the version they loaded at 08:53.

### Commands (repo root)

PREFIX = `docker exec -w /workspace/dishsim -e OMP_NUM_THREADS=2 dishsim-isaac taskset -c <j>,<j+16> nice -n 10`, run as
`nohup ... > data/logs/dot_rules/pairs_<id>.log 2>&1 &`.

| step | command | wall | result |
|---|---|---|---|
| pairs r11_00050 (j 0) | `PREFIX code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load r11_00050 --gate data/results/dot_rules/dependency/restricted/_gate/r11_00050 --edges pairs --max-wall-seconds 7200 --headless` | 2140.7 s (drops 1817.9 s) | `[RESULT] PASS r11_00050 status ok pairs 18 drops 60 edges 3 (knock 3 need 0 block 2) cycles 0 builds 1 extra 0 waves 2 targets joint_gate` |
| pairs r07_00023 (j 1) | the same with `r07_00023` | 2540.9 s (drops 2133.7 s) | `[RESULT] PASS r07_00023 status ok pairs 23 drops 70 edges 8 (knock 2 need 2 block 5) cycles 0 builds 2 extra 4 waves 4 targets joint_gate` |
| pairs r00_00031 (j 2) | the same with `r00_00031` | 2006.5 s (drops 1988.8 s) | `[RESULT] PASS r00_00031 status cycle pairs 20 drops 64 edges 5 (knock 5 need 0 block 2) cycles 1 builds 0 extra 0 waves none targets joint_gate` |
| pairs r02_00048 (j 3) | the same with `r02_00048` | 2174.2 s (drops 2156.4 s) | `[RESULT] PASS r02_00048 status cycle pairs 23 drops 70 edges 5 (knock 2 need 3 block 1) cycles 2 builds 0 extra 0 waves none targets joint_gate` |
| gate record r05_00014 (j 4) | the same with `r05_00014` | < 1 s, no Kit boot | `[RESULT] PASS r05_00014 status gate joint gate penetration_failure: Excessive penetration during joint initial settling` |
| rescore | `code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_score.py --dependency-dir data/results/dot_rules/dependency/pairs --N 262144 --theta 30 > data/logs/dot_rules/dependency_score_pairs.log 2>&1` | 858 s | one `[RESULT] PASS <id> ...` per load, then `[RESULT] PASS scored 5 of 5 loads` |

Logs `data/logs/dot_rules/pairs_<id>.log` and `dependency_score_pairs.log`: no traceback, no `free(): invalid pointer`; every
Kit session shut down by itself. Outputs in `data/results/dot_rules/dependency/pairs/`: `<id>.json`, `<id>.scores.json`
(packed for all five, settled for the four gated loads, built for r11 and r07), `_kit/<id>/report.json`. A drop cost
30.3-31.1 s (360 ticks with the 23-dish context); boot plus builds took 323 s for r11 (1 build) and 407 s for r07 (2).

### Per load

| load | gate (settle peak) | pairs | drops | edges (knock / need / block) | cycles | waves | builds | extra | status | failing dish | Kit s (drops s) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| r11_00050 | accepted (2.25 mm) | 18 | 60 | 3 (3 / 0 / 2) | 0 | 2 | 1 | 0 | ok | - | 2140.7 (1817.9) |
| r07_00023 | accepted (3.04 mm) | 23 | 70 | 8 (2 / 2 / 5) | 0 | 4 | 2 | 4 | ok | - | 2540.9 (2133.7) |
| r00_00031 | accepted (0.44 mm) | 20 | 64 | 5 (5 / 0 / 2) | 1 | none | 0 | 0 | cycle | - | 2006.5 (1988.8) |
| r02_00048 | accepted (4.22 mm) | 23 | 70 | 5 (2 / 3 / 1) | 2 | none | 0 | 0 | cycle | - | 2174.2 (2156.4) |
| r05_00014 | penetration_failure (5.57 mm, b08 / p01, limit 5) | 0 | 0 | 0 | 0 | none | 0 | 0 | gate | - | - |

A reason count is the number of edges that carry it; an edge can carry two, so the counts can sum past the edges. Drops =
24 + 2 x pairs in every load. Status reasons, verbatim:
- r11: "built in 1 build(s), 3 order constraints (3 learned), max lateral 1.8 mm; 0 constraint(s) learned beyond the graph's 3 edges"
- r07: "built in 2 build(s), 12 order constraints (12 learned), max lateral 0.7 mm; 4 constraint(s) learned beyond the graph's 8 edges"
- r00: "no valid one-at-a-time order: two-way pairs bowl_06 <-> bowl_08"
- r02: "no valid one-at-a-time order: two-way pairs bowl_02 <-> bowl_04, plate_02 <-> plate_08"
- r05: "joint gate penetration_failure: Excessive penetration during joint initial settling" (no pairs, tests or edges).

### Candidate pairs (FCL distance between the collision pieces at the jointly settled poses, mm; 0 = touching)

- r11 (18): b01-b05 0, b01-b06 9.93, b02-b07 0.89, b03-p02 16.52, b03-p07 2.65, b05-b06 3.54, b07-b08 4.16, c01-c02 14.58,
  c02-c04 13.28, c03-c04 16.96, c05-c07 13.22, c05-c08 14.27, c06-c08 15.74, p01-p06 0, p02-p08 9.69, p03-p04 15.69,
  p04-p06 1.49, p05-p07 0.
- r07 (23): b01-b06 5.66, b02-b05 0, b02-p04 19.04, b03-b08 4.16, b04-p03 0, b04-p05 13.07, b04-p06 9.44, b04-p07 3.51,
  b06-p06 4.49, c01-c02 17.71, c01-c07 12.23, c03-c08 12.74, c04-c05 13.47, c04-c08 17.3, c06-c07 13.29, p01-p02 0,
  p01-p03 19.08, p03-p05 15.55, p03-p07 0, p04-p05 0, p04-p07 15.38, p05-p07 0, p06-p08 0.
- r00 (20): b01-b04 1.22, b01-b07 10.86, b02-b05 0, b02-p01 7.03, b02-p05 0, b03-b04 13.92, b05-p01 4.08, b05-p05 0.57,
  b06-b08 0, c01-c04 9.39, c01-c06 17.75, c02-c07 13.53, c02-c08 13.51, c03-c06 14.67, c05-c07 16.04, p01-p03 15.66,
  p02-p04 0, p02-p08 0, p04-p08 15.76, p07-p08 15.22.
- r02 (23): b01-p04 7.25, b02-b04 0, b04-b06 18.65, b05-p01 4.74, b06-p01 16.41, b06-p02 0, b06-p07 17.77, b06-p08 5.98,
  b07-b08 0, c01-c05 15.87, c02-c07 8.91, c03-c06 12.81, c04-c08 18.46, c05-c08 10.32, c06-c07 18.05, p01-p08 0,
  p02-p06 15.69, p02-p07 0, p02-p08 15.4, p03-p05 0, p03-p06 2.52, p03-p07 18.48, p06-p07 0.

### Edges, waves, builds

- r11: b01 > b05 knock+block, b01 > b06 knock, p06 > p01 knock+block. Waves 2: wave 1 b05 b06 p01, the other 21 in wave 0.
  Build 1 from the graph (ties lowest centre first): ok, 24 placed, max lateral 1.8 mm; 0 extra constraints.
- r07: b05 > b02 knock+block, p01 > p02 block, p03 > b04 knock, p05 > p03 need, p07 > b04 block, p07 > p03 need,
  p07 > p04 block, p07 > p05 block. Waves 4: wave 1 b02 p02 p04 p05, wave 2 p03, wave 3 b04, 18 in wave 0. Build 1:
  moved, p08 (9th; dz 14.2 mm, tilt 16.6 deg, contact 6.76 mm) knocked b05 b01 b06 p06, so 4 extra constraints p08 > b01,
  p08 > b05, p08 > b06, p08 > p06; build 2: ok, max lateral 0.7 mm.
- r00: b02 > b05 knock+block, b02 > p05 knock, b06 > b08 knock+block, b06 > p07 knock, b08 > b06 knock. Cycle b06 <-> b08.
  No waves, no build.
- r02: b02 > b04 knock, b04 > b02 knock+need, p02 > p08 need, p08 > p02 need, p03 > p05 block. Cycles b02 <-> b04 and
  p02 <-> p08. No waves, no build.

Last drops that did not land (contact depth is 0 in every landed drop and 4.5-9.6 mm in every failed drop that touched):
r11 b01 (30.1 mm lateral, 12.5 dz, 31.4 deg, contact 8.35; moved b05 31.0 mm / 19.6 deg and b06 17.6 / 24.4), p06
(contact 8.87; moved p01 12.2 / 3.6). r07 b05 (tilt 22.4, contact 8.12; moved b02 12.6 / 18.8), p07 (lateral 15.7,
contact 4.52), p01 (contact 8.87); p03 landed but moved b04 27.1 / 24.3. r00 b02 (dz 20.7, tilt 19.7, contact 7.56; moved
b05 14.7 / 27.4, p05 16.6 / 37.3), b08 (tilt 35.6, contact 8.11; moved b06 12.6 / 14.5), b06 (27.5 mm, dz 22.3, 31.1 deg,
contact 8.45; moved b08 23.6 / 30.5, p07 30.3 / 16.2). r02 b04 (dz 38.5, contact 7.5; moved b02 21.4 / 13.8), p03 (lateral
10.7, contact 7.48); b02 landed but moved b04 26.2 / 15.8. Drops not landed in all: r11 4 of 60, r07 7 of 70, r00 6 of 64,
r02 9 of 70.

### S (N = 262,144, theta 30 deg): packed / settled / built

| load | Original | New | New+J |
|---|---|---|---|
| r11_00050 | 0.1819 / 0.1732 / 0.1733 | 0.0870 / 0.0826 / 0.0827 | 0.1565 / 0.1580 / 0.1581 |
| r07_00023 | 0.1887 / 0.1751 / 0.1754 | 0.0861 / 0.0816 / 0.0817 | 0.1571 / 0.1597 / 0.1597 |
| r00_00031 | 0.1868 / 0.1809 / - | 0.0889 / 0.0876 / - | 0.1561 / 0.1651 / - |
| r02_00048 | 0.1868 / 0.1783 / - | 0.0889 / 0.0870 / - | 0.1553 / 0.1674 / - |
| r05_00014 | 0.1917 / - / - | 0.0859 / - / - | 0.1574 / - / - |

Packed and settled S equal the parent's stage 2 values to 6 decimals on all five loads (same entries, same gates). Built S
differs from the parent's builds by at most 0.00008: r11 O 0.173339 (was 0.173318), NJ 0.158112 (0.158117); r07 O 0.175374
(0.175312), NJ 0.159686 (0.159767).

### Context drift (context_drift_mm: largest distance of a context dish from its target after the 150-tick re-settle)

| load | max mm | median mm | tests > 2 mm |
|---|---|---|---|
| r11_00050 | 16.6 | 0.15 | 6 of 60 |
| r07_00023 | 31.4 | 0.6 | 9 of 70 |
| r00_00031 | 30.7 | 0.2 | 7 of 64 |
| r02_00048 | 34.5 | 3.7 | 48 of 70 |

Except r02's tests 27-70, every drift over 2 mm comes with a removed dish that a neighbour leans on; where a test
removes that neighbour too, the drift vanishes, which names it. r11: b05 moves 16.3-16.6 mm without b01 (0.1 with both
out), p01 2.7 without p06. r07: base 0.6 mm in most tests (0.2 when b06 is out, so b06 sits 0.6 mm off), b02 5.7 without
b05, p02 16.2-16.3 without p01, b04 2.8 without p03; 31.3-31.4 without p03 and p07 (likely b04 again: p07's drop then
moved it 21.0 mm). r00: b05 17.4-17.5 without b02, b08 30.7 without b06, b06 6.7 without b08. r02: b02 22.8-22.9 without
b04, b04 34.5 without b02, 3.7 without p03 and 4.5 without p02 (dish not named), 19.4-20.4 without p02 and p08 (b06
moves in both drops). r02 tests 1-26 sit at 0.1-0.2 mm otherwise (L(b08) 1.9), but tests 27-70 never drop below 3.7 mm
(3.7-4.6), cup-only removals included (see Odd points).

### Margins (each test ran once)

Listed: tests with lateral, dz or tilt within 20 % of the at-goal tolerance (bowl and cup 15 mm / 20 mm / 15 deg, plate
18 mm / 20 mm / 16 deg), or a recorded disturbance within 20 % of 10 mm or 20 deg. * = the verdict can flip (a landed
criterion within 20 % under its limit, or every failing criterion within 20 % over it). Only dishes past a gate are stored
as disturbed, so near misses under 10 mm / 20 deg cannot be listed. No contact depth is near the 2 mm limit.
- r11 (2, none can flip): b01 last, not landed: moved b05 31.0 mm / 19.6 deg. b01 w/o b06, not landed: moved b05
  10.9 mm / 41.1 deg (an R drop; R drops make no knock edge).
- r07 (3, none): b05 last, not landed (tilt 22.4, contact 8.12): moved b02 12.6 mm / 18.8 deg, the closest knock in the run
  (26 % over 10 mm; b05 > b02 also has block). p07 last, not landed on contact 4.52 mm: lateral 15.7 / 18. p07 w/o p03, not
  landed (contact 8.87): moved b04 21.0 mm / 21.4 deg.
- r00 (4, one): b02 last: dz 20.7 / 20, but tilt 19.7 / 15 and contact 7.56 fail too. b06 last: dz 22.3 / 20 and moved p07
  30.3 mm / 16.2 deg; lateral 27.5 and contact 8.45 fail too. *b05 w/o b02, landed: tilt 13.8 / 15 (a flip would add need
  b02 > b05 to an edge that exists). b02 w/o p05: dz 18.3 / 20, fails on lateral 22.7, tilt 33.3, contact 6.71.
- r02 (4, one): b04 last: lateral 14.7 / 15, fails on dz 38.5 and contact 7.5. *b02 w/o b04, not landed on dz alone
  (21.7 / 20, 0 mm contact, run in the 3.7 mm drift state): the need reason of b04 > b02; that edge keeps its knock, so the
  cycle stands either way. b04 w/o b06: moved b02 15.0 mm / 19.2 deg. p03 w/o p06: moved p05 31.7 mm / 18.5 deg.

### Against the parent's graphs (stage 2, built from zero)

- r07_00023: the parent needed 5 builds and learned 6 constraints (b06 > b04, p01 > p02, p03 > b04, p06 > b04, p08 > b04,
  p08 > b06) plus support b05 > b02, 28 neighbour edges, 8 waves. The pair graph's order needed 2 builds and 4 extra
  constraints (all p08 first: b01, b05, b06, p06; p08 > b06 is one of the parent's). Kept: b05 > b02 (now knock+block),
  p01 > p02 (block), p03 > b04 (knock). Not in the new graph: b06 > b04, p06 > b04, p08 > b04, p08 > b06. None reversed.
- r11_00050: the parent needed 2 builds and learned 1 constraint (p06 > p01) plus support b01 > b05, 25 neighbour edges, 6
  waves. The pair graph holds both (b01 > b05 and p06 > p01, knock+block each) plus b01 > b06; its order built the load at
  the first build with 0 extra constraints.
- r00_00031: the parent failed on landing at b08 after 3 builds (learned b08 > b06, p07 > b08); now the cycle b06 <-> b08, and
  b08 w/o b06 lands 18.8 mm lateral, 9.9 dz, 24.6 deg off, the parent's failing numbers exactly.
- r02_00048: the parent failed on landing at b02 after 5 builds (learned b01 > b02, b01 > b04, b02 > b04, b04 > b06,
  p03 > p05); now the cycles b02 <-> b04 and p02 <-> p08; p03 > p05 agrees (block). b02 w/o b04 lands 9.1 mm / dz 21.7 /
  7.0 deg off (parent 9.8 / 21.8 / 7.0).

### Works when (plan)

- r11_00050.json: pairs_tested 18 > 0, every edge carries a reason, waves assigned (2), the graph's build needed 1 build
  (before: 2): met.
- r00_00031 or r02_00048 shows a two-way pair with status cycle: met by both.
- Figures and page `[RESULT] PASS`, republish: not part of this stage.

### Odd points

- Context drift over 2 mm in 70 of 264 drops (max 34.5 mm); the plan assumed under 2 mm. Apart from r02 below it is the
  removal itself: a leaning neighbour falls before the drop, and the drop is then judged against the fallen context. The
  knock edges b01 > b05 (r11), b06 > b08 and b02 > b05 (r00) and both r02 bowl knocks come from such drops: the dropped
  dish meets a neighbour that fell 16-35 mm and pushes it, so "knock" there means the neighbour leans on it. The direction
  matches the support order, but the reason label is not a clean knock.
- r02_00048: from test 27 (b04 w/o b02) to test 70 the context drift never falls below 3.7 mm, cup-only removals
  included (c05 w/o c01 3.7), while tests 1-26 sit at 0.1-0.2 mm unless a dish with a leaning neighbour was out (L(b08)
  1.9). Something stopped returning to its target after test 26 (b01 w/o p04, landed 4.2 mm off); the record keeps only
  the maximum, so the dish (or a rack that slid on its rail) is not identified. L(p03) drifted the same 3.7 mm. The
  drops that set need b04 > b02, need p02 <-> p08 and block p03 > p05 ran in that state.
- Dishes that never land, even last: r00 b08 (2 drops: last, and w/o b06) and r02 b04 (3 drops: last, w/o b02, w/o b06).
  Their jointly settled poses are not reached by one drop in any tested context; each sits in its load's bowl cycle.
- Old support edges: no contradiction. r07 b05 > b02 and r11 b01 > b05 are both in the new graphs, same direction. Two
  edges reverse an order the parent's successful builds used (neighbour edges): r07 p07 > p04 (block; the parent placed p04
  before p07), r11 b01 > b06 (knock from the fallen-context drop above; the parent placed b06 before b01).
- r02 plate_02 <-> plate_08 is a two-way need between plates 15.4 mm apart that do not touch. It runs through b06 (0 mm
  from p02, 5.98 mm from p08): with both plates out the context drifts 19.4-20.4 mm (most likely b06), and either
  plate's drop then hits b06 (contact 5.3 / 6.5 mm) and moves it 12.8-15.6 mm. b06 itself lands last (0.2 mm drift), so
  an order with b06 after both plates may exist that the test cannot see: this cycle comes from the full-load context
  more than from the pair. The bowl cycles (r00 b06 <-> b08, r02 b02 <-> b04) are mutual leans: each bowl falls 6.7-34.5
  mm without the other.
- r07 build 1: p08 knocked four dishes (contact 6.76 mm) although both its pair drops (last, and w/o p06) landed with 0 mm
  contact; b01, b05 and b06 are not even candidates of p08. The full-load test does not see failures in a partial load,
  hence the 4 extra constraints.
- Disturbances in removal drops make no edge by design, some large: r00 b02 w/o p01 moved b05 32.7 mm / 48.7 deg; r02 p03
  w/o p06 moved p05 31.7 mm / 18.5 deg.

### Stage 1 figures and page (2026-10-14)

Built 09:58:30-09:59:07 UTC with the 09:15 scripts, then rebuilt 10:01:56-10:02:35 UTC into the same folders after the
scripts gained context-drift reporting (10:00:54); the rebuild is what is on disk. Commands as for v15b with
`--dependency-dir data/results/dot_rules/dependency/pairs`, figures `--out data/media/dot_rules/figures_v17` (21.8 s),
page `--figures-dir data/media/dot_rules/figures_v17 --out data/media/dot_rules/page_v17` (7.4 s); logs
`data/logs/dot_rules/{figures,page}_v17.log` (first build) and `{figures,page}_v17b.log`. Both `[RESULT] PASS`, no
traceback, no fix. figures_v17: 18 PNGs, the 13 non-depgraph ones byte-identical to figures_v15b, five new depgraph PNGs.
page_v17: index.html 7,012,463 bytes, 43 files, 13,553,076 bytes in all; media/ 38 files with page_v15b's names (only the
five depgraph PNGs changed), data/ depgraph_<id>.json for r00, r02, r07, r11 (byte copies of pairs/<id>.json), none for
r05; the 42 local src/href resolve and every shipped file is referenced. grep -c: drift 6, two-way 5, ordered-pair 7,
"only that order" 2, "Ren and Qureshi" 1, learned 0, "neighbour order" 0, each depgraph PNG 1. New table column
"drift > 2 mm": r07 9 of 70, r11 6 of 60, r00 7 of 64, r02 48 of 70 (the context-drift table above). Figure check (all five
read): titles match the table, no red ring (no failed build), graph arrows end at their nodes. Three visual points:
- The upper head of each two-way edge (bowl_08 in r00; plate_02 and bowl_02 in r02) sits under that dish's label and is
  barely visible, so the pair reads as a one-way arrow downward.
- Top views: a head ends at the later dish's centre under its label, so most are hidden (r11 p01, r00 p05 and p07, r02
  p05, the p03/p07/p05/p04 cluster in r07), and r11 b01 > b05 (stacked bowls about 1 cm apart) does not show.
- The page explains drift above 2 mm as a removed dish that held a neighbour up; r02's 48 include all of tests 27-70 (44,
  never below 3.7 mm), 12 of them cup-only removals, which that reading does not cover (see Odd points).

Stage 1 published 10:06 UTC: https://claude.ai/artifact/F5vrhrNM4XnUk3ZXhJycY2 Version 17 (version id 1791972366-4ae2), figures_v17 / page_v17 rebuilt with the drift captions, index.html 7,012,463 B, 38 media + 4 data files. Stage 2 figures, page and publish: pending

## Stage 2: caveats (2026-10-14)

Run 09:27:39-10:26:27 UTC. Three Kit batches ran in the same container while Stage 1's last session, rescore and page
were still finishing. Each session was pinned inside the container to CPUs j and j+16, at nice 10 with 2 OMP threads,
and launched with nohup into its own log. No fix, no rerun, no script edit.
- B, 09:27:39, j = 4-10: the seven loads that passed the joint gate after Stage 1 (r06_00008 r04_00021 r02_00021
  r11_00034 r05_00026 r13_00038 r09_00014) into `pairs_more/`. Finished 09:57:16-10:06:31. With the same flags, the five
  gate-rejected loads (r10_00037 r11_00049 r05_00010 r03_00045 r05_00001) ran one after another on j = 11,
  09:27:40-09:27:42. Each wrote a status-gate record without booting Kit.
- C, 09:30:05, j = 11 and 12: r00_00031 and r02_00048 with `--goal built` into `pairs_resettled/`. A 60 s file poll
  launched them as soon as `pairs/r00_00031.json` and `pairs/r02_00048.json` existed. Both re-settles failed, r00 at
  09:31:15 and r02 at 09:36:42.
- A, 09:36:02, j = 0-3: the four first-run loads again, same flags, into `pairs_rerun/`. A 60 s poll on the first-run
  container PIDs (4915 4921 4927 4944 4955 4958 4968 4978) launched them once all eight had exited. Finished
  10:09:30-10:18:07.
- Rescores at N = 262,144, theta 30: `pairs_resettled` 09:37:03-09:45:19, sharing the GPU with Stage 1's rescore, and
  `pairs_more` 10:06:51-10:26:27.

All 13 Kit sessions shut down by themselves. A drop cost 29.7-32.4 s, as in Stage 1. The kit script on disk was
edited at 10:23 to add `--perturb-*`. Every Stage 2 session had started by 09:36 and ran the 09:20 version: no record
carries a `perturbation` key. From 10:26 another session ran four `--perturb-mm 3 --perturb-deg 3 --perturb-seed 1`
sessions into `pairs_perturbed/` on CPUs 0-3 and 16-19. They are untouched and not reported here.

### Joint gate over the top 17 restricted loads

Rank is by packed S_NJ: ranks 1-12 at N = 262,144 (`restricted/scores_N0262144`), ranks 13-17 at N = 16,384. This
stage's N = 262,144 rescore keeps that order. Of the 17 loads, 11 were accepted and 6 rejected. The reasons come from
`restricted/_gate/<id>/result.json` (outcome, reason, maximum_settle_penetration_event, settled):
- penetration_failure, 5 loads: r05_00014, r11_00049, r05_00010, r03_00045 and r05_00001. Each gate aborted early,
  0.12-0.86 s into the joint settle, when one contact passed the 5 mm momentary peak (20-30 s wall). Three of the five
  peaks are dish against dish, two are dish into the LowerRack.
- settle_failure, 1 load: r10_00037. Penetration was fine (peak 0.93 mm, p03 / p08), but b01 kept rocking. In each of
  18 one-second observation windows its quaternion span was 3.01-3.12 deg against the 3.0 deg rest limit (b08 reached
  2.57-2.95 deg). The gate gave up at the 12 s settle allowance (145 s wall).

| rank | load | gate (settle peak) | pairs | drops | edges (knock / need / block) | cycles | waves | builds | extra | status | S New+J packed / settled / built |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | r05_00014 | penetration_failure (5.57 mm, b08 / p01, limit 5; abort 0.26 s in) | 0 | 0 | 0 | 0 | none | 0 | 0 | gate | 0.1574 / - / - |
| 2 | r07_00023 | accepted (3.04 mm) | 23 | 70 | 8 (2 / 2 / 5) | 0 | 4 | 2 | 4 | ok | 0.1571 / 0.1597 / 0.1597 |
| 3 | r11_00050 | accepted (2.25 mm) | 18 | 60 | 3 (3 / 0 / 2) | 0 | 2 | 1 | 0 | ok | 0.1565 / 0.1580 / 0.1581 |
| 4 | r00_00031 | accepted (0.44 mm) | 20 | 64 | 5 (5 / 0 / 2) | 1 | none | 0 | 0 | cycle | 0.1561 / 0.1651 / - |
| 5 | r02_00048 | accepted (4.22 mm) | 23 | 70 | 5 (2 / 3 / 1) | 2 | none | 0 | 0 | cycle | 0.1553 / 0.1674 / - |
| 6 | r10_00037 | settle_failure (b01 rocks 3.01-3.12 deg, limit 3; 18 restarts, gave up at 12.1 s) | 0 | 0 | 0 | 0 | none | 0 | 0 | gate | 0.1549 / - / - |
| 7 | r11_00049 | penetration_failure (9.36 mm, b04 / p08, limit 5; abort 0.24 s in) | 0 | 0 | 0 | 0 | none | 0 | 0 | gate | 0.1549 / - / - |
| 8 | r05_00010 | penetration_failure (5.18 mm, LowerRack / b04, limit 5; abort 0.86 s in) | 0 | 0 | 0 | 0 | none | 0 | 0 | gate | 0.1548 / - / - |
| 9 | r03_00045 | penetration_failure (6.43 mm, b02 / p04, limit 5; abort 0.17 s in) | 0 | 0 | 0 | 0 | none | 0 | 0 | gate | 0.1545 / - / - |
| 10 | r06_00008 | accepted (1.50 mm) | 19 | 62 | 5 (2 / 0 / 4) | 0 | 3 | 1 | 0 | ok | 0.1542 / 0.1583 / 0.1585 |
| 11 | r04_00021 | accepted (2.75 mm) | 21 | 66 | 3 (2 / 2 / 2) | 0 | 2 | 1 | 0 | ok | 0.1541 / 0.1567 / 0.1565 |
| 12 | r02_00021 | accepted (0.70 mm) | 19 | 62 | 6 (5 / 1 / 3) | 1 | none | 0 | 0 | cycle | 0.1540 / 0.1591 / - |
| 13 | r11_00034 | accepted (4.61 mm) | 17 | 58 | 2 (1 / 0 / 1) | 0 | 2 | 1 | 0 | ok | 0.1534 / 0.1612 / 0.1612 |
| 14 | r05_00026 | accepted (1.16 mm) | 18 | 60 | 4 (3 / 1 / 2) | 1 | none | 0 | 0 | cycle | 0.1525 / 0.1610 / - |
| 15 | r05_00001 | penetration_failure (6.22 mm, LowerRack / b02, limit 5; abort 0.12 s in) | 0 | 0 | 0 | 0 | none | 0 | 0 | gate | 0.1524 / - / - |
| 16 | r13_00038 | accepted (0.75 mm) | 17 | 58 | 4 (2 / 1 / 3) | 1 | none | 0 | 0 | cycle | 0.1524 / 0.1540 / - |
| 17 | r09_00014 | accepted (1.61 mm) | 16 | 56 | 1 (1 / 0 / 1) | 0 | 2 | 1 | 0 | ok | 0.1522 / 0.1598 / 0.1599 |

How to read the table:
- Ranks 1-5 are Stage 1's records (`pairs/`); ranks 6-17 are this stage's (`pairs_more/`).
- Drops = 24 + 2 x pairs.
- A reason count is the number of edges that carry that reason.

Of the 11 gated loads, 6 are ok. Their certifying builds succeeded on the first build with 0 extra constraints, except
r07 (2 builds, 4 extra). The other 5 are cycles, each through a bowl-bowl pair:
- r00 b06 <-> b08;
- r02_00048 b02 <-> b04, plus p02 <-> p08;
- r02_00021 b05 <-> b08;
- r05_00026 b05 <-> b06;
- r13 b02 <-> b05.

On ranks 1-12, packed S_NJ equals `restricted/scores_N0262144` exactly. On all 11 gated loads, settling raises S_NJ by
0.0015-0.0121, and built S_NJ stays within 0.0002 of settled. Ranked by built S_NJ, the certified loads go r11_00034
0.1612 > r09_00014 0.1599 > r07_00023 0.1597 > r06_00008 0.1585 > r11_00050 0.1581 > r04_00021 0.1565. That is, rank 13
beats rank 2 once both are built.

New graphs (`pairs_more/`; "X last" = X dropped into the full load, "X w/o Y" = X dropped with Y also removed):
- r06_00008.
  - Edges: b03 > b08 block, b04 > b07 block, b04 > p06 knock, b07 > p06 knock+block, p07 > p03 block.
  - Waves 3: wave 1 b07 b08 p03, wave 2 p06, 20 dishes in wave 0.
  - Build: 1, ok, max lateral 1.8 mm.
  - Last drops not landed: b03, b07, p07, b04. b04 last fails on contact alone (9.06 mm; lateral 3.4, dz 1.9, tilt 4.3
    are inside tolerance). It lands into b07, which fell about 84 mm once b04 was out, and moves p06 73.9 mm / 41.5 deg.
- r04_00021.
  - Edges: b01 > b07 need+block, p02 > p01 knock+need, p02 > p06 knock+block.
  - Waves 2: wave 1 b07 p01 p06.
  - Build: 1, ok, 4 order constraints (one support edge on top of the 3), max lateral 3.3 mm.
  - Last drops not landed: p02 (moves p06 14.7 / 19.7 and p01 14.6 / 12.9) and b01 (contact 8.63 mm alone).
- r02_00021.
  - Edges: b04 > b03 knock+block, b05 > b06 knock+block, b05 > b08 knock, b08 > b05 knock, b08 > b06 knock+need,
    p02 > p01 block.
  - Cycle: b05 <-> b08. No waves, no build.
  - Last drops not landed: b05 (lateral 42.5, moves b06 53.2 / 53.7), b04, p02, b08.
- r11_00034.
  - Edges: b07 > b04 knock, b07 > b05 block.
  - Waves 2: wave 1 b04 b05.
  - Build: 1, ok, max lateral 0.6 mm.
  - Last drop not landed: b07 only (moves b04 36.6 / 31.8).
- r05_00026.
  - Edges: b05 > b06 knock, b06 > b05 knock+block, p03 > p05 need, p04 > p08 knock+block.
  - Cycle: b05 <-> b06.
  - Last drops not landed: b05, b06, p04.
- r13_00038.
  - Edges: b02 > b05 knock, b05 > b02 knock+block, p02 > p01 block, p07 > b03 need+block.
  - Cycle: b02 <-> b05.
  - Last drops not landed: b02, p07, p02, b05. p07 last fails on contact alone (10.58 mm; 3.0 / 0.3 / 2.2), landing
    into b03, which fell 57.8 mm without p07 (0.0 mm drift once both are out).
- r09_00014.
  - Edge: b04 > b05 knock+block.
  - Waves 2.
  - Build: 1, ok, max lateral 0.6 mm.
  - Last drop not landed: b04 only (lateral 29.6, moves b05 30.4 / 53.2).

S (N = 262,144, theta 30 deg), packed / settled / built, for the seven new gated loads:

| load | Original | New | New+J |
|---|---|---|---|
| r06_00008 | 0.1843 / 0.1771 / 0.1774 | 0.0862 / 0.0843 / 0.0844 | 0.1542 / 0.1583 / 0.1585 |
| r04_00021 | 0.1801 / 0.1708 / 0.1711 | 0.0920 / 0.0867 / 0.0867 | 0.1541 / 0.1567 / 0.1565 |
| r02_00021 | 0.1872 / 0.1842 / - | 0.0845 / 0.0825 / - | 0.1540 / 0.1591 / - |
| r11_00034 | 0.1823 / 0.1805 / 0.1808 | 0.0842 / 0.0839 / 0.0840 | 0.1534 / 0.1612 / 0.1612 |
| r05_00026 | 0.1814 / 0.1813 / - | 0.0826 / 0.0840 / - | 0.1525 / 0.1610 / - |
| r13_00038 | 0.1682 / 0.1647 / - | 0.0792 / 0.0783 / - | 0.1524 / 0.1540 / - |
| r09_00014 | 0.1837 / 0.1813 / 0.1818 | 0.0786 / 0.0791 / 0.0794 | 0.1522 / 0.1598 / 0.1599 |

The five rejected loads have packed scores only (Original / New / New+J):

| load | Original | New | New+J |
|---|---|---|---|
| r10_00037 | 0.1948 | 0.0924 | 0.1549 |
| r11_00049 | 0.1821 | 0.0899 | 0.1549 |
| r05_00010 | 0.1871 | 0.0925 | 0.1548 |
| r03_00045 | 0.1808 | 0.0805 | 0.1545 |
| r05_00001 | 0.1745 | 0.0824 | 0.1524 |

### Reproducibility (`pairs_rerun/`: identical inputs, second run)

| load | status run 1 / 2 | edges run 1 / 2 | in both | only run 1 | only run 2 | two-way pairs agree | waves 1 / 2 | builds (extra) 1 / 2 | drops identical | drift max mm (drops > 2 mm) 1 / 2 | Kit s 1 / 2 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| r11_00050 | ok / ok | 3 / 3 | 3 | 0 | 0 | yes (none) | 2 / 2 | 1 (0) / 1 (0) | 60 of 60 | 16.6 (6) / 16.6 (6) | 2140.7 / 2162.6 |
| r07_00023 | ok / ok | 8 / 8 | 8 | 0 | 0 | yes (none) | 4 / 4 | 2 (4) / 2 (4) | 70 of 70 | 31.4 (9) / 31.4 (9) | 2540.9 / 2523.6 |
| r00_00031 | cycle / cycle | 5 / 5 | 5 | 0 | 0 | yes (b06 <-> b08) | none / none | 0 / 0 | 64 of 64 | 30.7 (7) / 30.7 (7) | 2006.5 / 2007.5 |
| r02_00048 | cycle / cycle | 5 / 5 | 5 | 0 | 0 | yes (b02 <-> b04, p02 <-> p08) | none / none | 0 / 0 | 70 of 70 | 34.5 (48) / 34.5 (48) | 2174.2 / 2170.5 |

All four reruns reproduce run 1 exactly. Every key of the record is equal except generated_utc and kit, including:
- every drop's landed, lateral, dz, tilt, contact, disturbed and drift;
- the candidate pairs;
- the build records;
- the built poses_world of r11 and r07, at full float precision.

The edge sets (a, b), reasons ignored, are the same in both runs:
- r11: b01 > b05, b01 > b06, p06 > p01.
- r07: b05 > b02, p01 > p02, p03 > b04, p05 > p03, p07 > b04, p07 > p03, p07 > p04, p07 > p05.
- r00: b02 > b05, b02 > p05, b06 > b08, b06 > p07, b08 > b06.
- r02: b02 > b04, b04 > b02, p02 > p08, p03 > p05, p08 > p02.

In every load all edges are in both runs and none is in one only. The reasons match too, and so do r07's build-1
failure (p08 moved b05 b01 b06 p06) and its 4 extra constraints. No edge differs, so there are no test values to set
against the thresholds. Only the wall time changes, by -17.3 to +21.9 s.

So Isaac 4.5 CPU PhysX at 120 Hz is deterministic for this scene and these inputs, even though the two runs had
different neighbours (run 2 shared the box with up to 11 other Kit sessions). A repeat measures no settle noise. Two
things therefore hold only in this exact state, not as measured margins:
- Stage 1's marginal verdicts: r00 b05 w/o b02 at tilt 13.8 / 15, and r02 b02 w/o b04 at dz 21.7 / 20.
- r02_00048's 3.7 mm drift from test 27 onward. It is deterministic carry-over in the scene, not noise.

### Re-settled goals (`pairs_resettled/`, `--goal built`)

Neither goal re-settled one dish at a time under the own-load rule:
- settle deviation <= 80 mm of the jointly settled pose + 3 mm;
- contact < 2 mm;
- no placed dish moved > 10 mm / 20 deg.

Both records therefore read `resettle_failed`. goal_delta is null, and no pair test ran, so there is no new graph: no
pairs, edges, cycles, waves, certifying builds or extra constraints.

| load | initial constraints | resettle builds: failing dish (position in the order), what it moved, contact | goal_delta max | new graph | status | Kit s (re-settle s) |
|---|---|---|---|---|---|---|
| r00_00031 | 3: b02 > b05, b02 > p05, b06 > p07 | 2: (1) b08 (3rd) moved b06, 8.19 mm; (2) b06 (3rd) moved b08, 8.49 mm | null | none | resettle_failed (b06) | 69.2 (55.5) |
| r02_00048 | 1: p03 > p05 | 4: (1) p01 (6th) moved b05 b06, 3.89 mm; (2) p08 (7th) moved b06, 4.91 mm; (3) b02 (13th) moved b04, 0 mm; (4) b04 (13th) moved b02, 9.01 mm | null | none | resettle_failed (b04) | 396.5 (382.9) |

- r00_00031. The initial constraints are the first run's edges minus the two-way pair.
  - Build 1 (order b06 b02 b08 ...): b06 placed 10.1 / 3.5 / 4.2 off (lateral mm / dz mm / tilt deg), close to Stage
    1's b06 w/o b08 (10.4 / 3.6 / 4.3). b08 then landed 24.8 / 14.4 / 16.7 off and moved b06.
  - Build 2 (order b02 b08 b06 ...): b08 placed 18.7 / 9.9 / 24.6 off, Stage 1's b08 w/o b06 (18.8 / 9.9 / 24.6) to
    0.1 mm. b06 then landed 13.3 / 16.5 / 20.5 off and moved b08.
  - The learned b08 > b06 and b06 > b08 close the cycle. build_sequence stops at 2 of 8 builds.
- r02_00048.
  - Builds 1-2 learned p01 > b05, p01 > b06 and p08 > b06.
  - Builds 3-4 stop on the bowl pair. In build 3, b02 dropped after b04 moves b04 (b02 8.3 / 4.2 / 7.0 off, 0
    contact). In build 4, b04 dropped after b02 moves b02 (b04 4.6 / 15.3 / 13.8 off, 9.01 mm contact). That closes the
    cycle at 4 of 8 builds.
  - p02 and p08 went in without trouble in builds 3-4: p08 first, 0.1 mm off; p02 0.6 / 1.3 mm off; 0 contact.
- No built goal exists, so goal_delta is null. As a stand-in, these placed dishes were over 5 mm or 2 deg from their
  jointly settled pose in the builds (the steps' lateral / dz / tilt):
  - r00: b06 10.1 / 3.5 / 4.2 in build 1; b08 18.7 / 9.9 / 24.6 in build 2.
  - r02 b06: 32.1 / 33.0 / 43.2 in build 1, 12.7 / 4.5 / 12.5 in build 2, 3.2-3.3 / 0.9 / 2.6 in builds 3-4.
  - r02 b04: 8.7-9.2 / 3.8-4.0 / 35.5-35.6 in builds 1-3.
  - r02 b01: 4.1-4.5 / 1.1 / 2.1-2.2 in builds 1-4.
  - r02 b02: 9.9 / 21.8 / 7.1 in build 4.

| load, run | Original | New | New+J |
|---|---|---|---|
| r00_00031 first run (packed / settled) | 0.1868 / 0.1809 | 0.0889 / 0.0876 | 0.1561 / 0.1651 |
| r00_00031 `--goal built` (packed / re-settled / built) | 0.1868 / - / - | 0.0889 / - / - | 0.1561 / - / - |
| r02_00048 first run (packed / settled) | 0.1868 / 0.1783 | 0.0889 / 0.0870 | 0.1553 / 0.1674 |
| r02_00048 `--goal built` (packed / re-settled / built) | 0.1868 / - / - | 0.0889 / - / - | 0.1553 / - / - |

The failed record keeps the joint gate's entries_settled. The scorer's "settled" side for it is therefore the jointly
settled goal again, equal to the first run's settled bit for bit, and not a re-settled one. There is no re-settled or
built goal to score.

### Commands (repo root)

PREFIX = `docker exec -w /workspace/dishsim -e OMP_NUM_THREADS=2 dishsim-isaac taskset -c <j>,<j+16> nice -n 10`, each Kit
step run as `setsid nohup PREFIX ... > data/logs/dot_rules/<log>.log 2>&1 &` (logs `pairs_more_<id>`,
`pairs_resettled_<id>`, `pairs_rerun_<id>`).

KIT = `code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load <id> --gate data/results/dot_rules/dependency/restricted/_gate/<id> --edges pairs --max-wall-seconds 7200 --headless`

| step | command | wall (drops / re-settle) | result |
|---|---|---|---|
| B r06_00008 (j 4) | `PREFIX KIT --out-dir data/results/dot_rules/dependency/pairs_more` | 2217.9 s (1897.0) | `[RESULT] PASS r06_00008 status ok pairs 19 drops 62 edges 5 (knock 2 need 0 block 4) cycles 0 builds 1 extra 0 waves 3 targets joint_gate` |
| B r04_00021 (j 5) | the same | 2331.3 s (2008.5) | `[RESULT] PASS r04_00021 status ok pairs 21 drops 66 edges 3 (knock 2 need 2 block 2) cycles 0 builds 1 extra 0 waves 2 targets joint_gate` |
| B r02_00021 (j 6) | the same | 2025.9 s (2006.8) | `[RESULT] PASS r02_00021 status cycle pairs 19 drops 62 edges 6 (knock 5 need 1 block 3) cycles 1 builds 0 extra 0 waves none targets joint_gate` |
| B r11_00034 (j 7) | the same | 2039.8 s (1723.9) | `[RESULT] PASS r11_00034 status ok pairs 17 drops 58 edges 2 (knock 1 need 0 block 1) cycles 0 builds 1 extra 0 waves 2 targets joint_gate` |
| B r05_00026 (j 8) | the same | 1828.8 s (1810.3) | `[RESULT] PASS r05_00026 status cycle pairs 18 drops 60 edges 4 (knock 3 need 1 block 2) cycles 1 builds 0 extra 0 waves none targets joint_gate` |
| B r13_00038 (j 9) | the same | 1776.2 s (1757.9) | `[RESULT] PASS r13_00038 status cycle pairs 17 drops 58 edges 4 (knock 2 need 1 block 3) cycles 1 builds 0 extra 0 waves none targets joint_gate` |
| B r09_00014 (j 10) | the same | 1991.1 s (1672.9) | `[RESULT] PASS r09_00014 status ok pairs 16 drops 56 edges 1 (knock 1 need 0 block 1) cycles 0 builds 1 extra 0 waves 2 targets joint_gate` |
| B gate records (j 11, in sequence) | the same for r10_00037 r11_00049 r05_00010 r03_00045 r05_00001 | < 1 s each, no Kit boot | `[RESULT] PASS r10_00037 status gate joint gate settle_failure: continuous observation failed after the settle allowance; cannot restart`; r11_00049, r05_00010, r03_00045, r05_00001: `[RESULT] PASS <id> status gate joint gate penetration_failure: Excessive penetration during joint initial settling` |
| C r00_00031 (j 11) | `PREFIX KIT --goal built` | 69.2 s (55.5) | `[RESULT] PASS r00_00031 goal built status resettle_failed the goal did not build one dish at a time: no order built the load in 2 build(s): last moved at bowl_06 ['bowl_08']` |
| C r02_00048 (j 12) | the same | 396.5 s (382.9) | `[RESULT] PASS r02_00048 goal built status resettle_failed the goal did not build one dish at a time: no order built the load in 4 build(s): last moved at bowl_04 ['bowl_02']` |
| A r11_00050 (j 0) | `PREFIX KIT --out-dir data/results/dot_rules/dependency/pairs_rerun` | 2162.6 s (1836.9) | `[RESULT] PASS r11_00050 status ok pairs 18 drops 60 edges 3 (knock 3 need 0 block 2) cycles 0 builds 1 extra 0 waves 2 targets joint_gate` |
| A r07_00023 (j 1) | the same | 2523.6 s (2134.0) | `[RESULT] PASS r07_00023 status ok pairs 23 drops 70 edges 8 (knock 2 need 2 block 5) cycles 0 builds 2 extra 4 waves 4 targets joint_gate` |
| A r00_00031 (j 2) | the same | 2007.5 s (1989.1) | `[RESULT] PASS r00_00031 status cycle pairs 20 drops 64 edges 5 (knock 5 need 0 block 2) cycles 1 builds 0 extra 0 waves none targets joint_gate` |
| A r02_00048 (j 3) | the same | 2170.5 s (2152.1) | `[RESULT] PASS r02_00048 status cycle pairs 23 drops 70 edges 5 (knock 2 need 3 block 1) cycles 2 builds 0 extra 0 waves none targets joint_gate` |
| rescore re-settled | `code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_score.py --dependency-dir data/results/dot_rules/dependency/pairs_resettled --N 262144 --theta 30 > data/logs/dot_rules/dependency_score_pairs_resettled.log 2>&1` | 496 s | `[RESULT] PASS <id> ...` for r00 and r02, then `[RESULT] PASS scored 2 of 2 loads` |
| rescore pairs_more | the same with `pairs_more` (log `dependency_score_pairs_more.log`) | 1176 s | one `[RESULT] PASS <id> ...` per load, then `[RESULT] PASS scored 12 of 12 loads` |

Logs are in `data/logs/dot_rules/`: `{pairs_more,pairs_resettled,pairs_rerun}_<id>.log` and
`dependency_score_{pairs_more,pairs_resettled}.log`. None has a traceback or `free(): invalid pointer`. Outputs are in
`data/results/dot_rules/dependency/{pairs_more,pairs_resettled,pairs_rerun}/`:
- `<id>.json` and `_kit/<id>/report.json`, in all three folders.
- `<id>.scores.json`, in pairs_more and pairs_resettled only. Packed is scored for all 14 loads. Settled is scored for
  the 7 gated pairs_more loads and for both resettled loads (the joint gate's poses, see above). Built is scored for
  r06_00008, r04_00021, r11_00034 and r09_00014.

### Odd points

- No settle noise. The reruns are identical to run 1 at full precision, so batch A cannot say which tests are
  marginal. A noise estimate needs perturbed inputs.
- Both re-settles failed on their bowl pair. Each bowl, dropped second, moves the other even in a partial load of 2-12
  placed dishes: r00 b08 and b06 at 8.19 and 8.49 mm contact, r02 b02 at 0 mm and b04 at 9.01 mm. So the two-way bowl
  pairs are not artifacts of the jointly settled poses. The plate pair p02 <-> p08 never stopped a build, which
  supports Stage 1's reading that it comes from the full-load context.
- The own-load re-settle rule checks the settle position (<= 80 mm), contact and disturbance, but not tilt. Before
  failing, r02's builds accepted b06 at 43.2 deg tilt (32.1 mm lateral, 33.0 dz) and b04 at 35.5-35.6 deg off their
  jointly settled poses. Had the goal built, goal_delta could have reached tens of mm and degrees.
- r13_00038's jointly settled load pools: b08 in the upper rack (pooling_count 1, feasible False under the dot rules).
  Its packed load is pooling-free. As a cycle load it has no build either.
- Context drift over 2 mm appears in 51 of 422 B drops (Stage 1: 70 of 264):

  | load | max drift | drops over 2 mm |
  |---|---|---|
  | r06_00008 | 83.9 mm | 12 of 62 |
  | r13_00038 | 57.8 mm | 8 of 58 |
  | r02_00021 | 31.1 mm | 17 of 62 |
  | r09_00014 | 16.7 mm | 1 of 56 |
  | r05_00026 | 16.1 mm | 5 of 60 |
  | r04_00021 | 15.1 mm | 5 of 66 |
  | r11_00034 | 6.9 mm | 3 of 58 |

  - The two largest have named fallers. In r06_00008, b07 falls about 84 mm in all five drops with b04 out and b07 in
    (5.3 mm once b07 is out too). This is the largest drift seen so far. In r13_00038, b03 falls 57.8 mm without p07
    (0.0 mm with both out).
  - In both cases the dish then dropped onto the fallen neighbour fails on contact depth alone, at 8.95-10.58 mm, with
    its pose inside tolerance.
  - No B load shows r02_00048's persistent offset. The median drift is 0.0-0.2 mm, except r11_00034 at 0.8 mm.
- Some dishes never land, even dropped last: r02_00021 b08, r05_00026 b05 and r13_00038 b02. Each is in its load's bowl
  cycle, like r00 b08 and r02_00048 b04 in Stage 1.
- r11_00034 b07 > b04 is a knock on a non-candidate (b04 is more than 20 mm from b07). This is allowed by design, and it
  is the only such edge in B.
- Disturbances in removal drops make no edge, by design. Some are large:
  - r06 p07 w/o p05 moved p03 58.6 mm / 20.8 deg.
  - r02_00021 b08 w/o b06 moved b05 39.5 / 22.5.
  - r02_00021 b08 w/o b01 moved b05 33.1 / 45.4 and b06 35.1 / 22.2.
  - r05_00026 p05 w/o p03 moved b07 33.2 / 32.1.
- The top-17 cut is not tight. Rank 18 at N = 16,384 is r06_00030 (0.15213) and was not rescored. r09_00014 at
  N = 262,144 is 0.152189, only 6e-5 above it, which is about the N = 16,384 spread.
- Both rescore logs print their header line twice, as Stage 1's `dependency_score_pairs.log` does. Harmless.

## Stage 3: perturbed release (2026-10-14)

Run 10:26:02-11:02:16 UTC. Four Kit sessions ran at once, one per Stage 1 load (j = 0, 1, 2, 3 for r11_00050,
r07_00023, r00_00031, r02_00048), each pinned inside the container to CPUs j and j+16 at nice 10 with 2 OMP threads and
launched with nohup at 10:26:02. Records were written at 10:59:30 (r00), 11:01:52 (r07), 11:01:57 (r11) and 11:02:16
(r02). Every Kit session shut down by itself; all four docker-exec clients were gone by 11:07. Container PIDs (Kit
python / isaaclab.sh): r11 12781 / 12729, r07 12786 / 12741, r00 12770 / 12735, r02 12771 / 12723; host docker-exec
clients 782201-782204. The sessions ran the kit script as edited at 10:23:01 (adds `--perturb-*`), unchanged since.
Every L and R drop commanded its dish at the target + 3 mm hover, yawed U(-3, 3) deg about the world Z axis through its
visual centroid, then shifted by a point uniform in the 3 mm disc of its rack plane (`default_rng((1, test number))`).
The context re-settle and the certifying build are not perturbed, and landing is judged against the unperturbed target
with Stage 1's tolerances. Candidate pairs and drop order equal Stage 1's (18 / 23 / 20 / 23 pairs, 60 / 70 / 64 / 70
drops), so test k is the same drop in both runs. A drop cost 30.4-31.1 s. No fix, no rerun, no script edit, no rescore.

### Commands (repo root)

PREFIX as in Stage 1, each run as `nohup PREFIX KIT > data/logs/dot_rules/pairs_perturbed_<id>.log 2>&1 &`.

KIT = `code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load <id> --gate data/results/dot_rules/dependency/restricted/_gate/<id> --edges pairs --perturb-mm 3 --perturb-deg 3 --perturb-seed 1 --out-dir data/results/dot_rules/dependency/pairs_perturbed --max-wall-seconds 7200 --headless`

| step | command | wall (drops) | result |
|---|---|---|---|
| r11_00050 (j 0) | `PREFIX KIT` | 2154.8 s (1831.7) | `[RESULT] PASS r11_00050 status ok pairs 18 drops 60 edges 5 (knock 4 need 1 block 2) cycles 0 builds 1 extra 0 waves 2 targets joint_gate perturb 3 mm 3 deg seed 1` |
| r07_00023 (j 1) | the same | 2149.7 s (2131.5) | `[RESULT] PASS r07_00023 status cycle pairs 23 drops 70 edges 5 (knock 2 need 2 block 2) cycles 1 builds 0 extra 0 waves none targets joint_gate perturb 3 mm 3 deg seed 1` |
| r00_00031 (j 2) | the same | 2007.8 s (1989.7) | `[RESULT] PASS r00_00031 status cycle pairs 20 drops 64 edges 5 (knock 4 need 1 block 2) cycles 1 builds 0 extra 0 waves none targets joint_gate perturb 3 mm 3 deg seed 1` |
| r02_00048 (j 3) | the same | 2173.4 s (2155.3) | `[RESULT] PASS r02_00048 status cycle pairs 23 drops 70 edges 5 (knock 2 need 2 block 1) cycles 1 builds 0 extra 0 waves none targets joint_gate perturb 3 mm 3 deg seed 1` |
| comparison | host `python3`, json only, over `pairs/<id>.json` and `pairs_perturbed/<id>.json` (a session scratchpad script, not kept in the repo) | < 1 s | the tables below |

Logs `data/logs/dot_rules/pairs_perturbed_<id>.log`: no traceback, no `free(): invalid pointer`. Outputs in
`data/results/dot_rules/dependency/pairs_perturbed/`: `<id>.json` (with `perturbation` {lateral_mm 3, yaw_deg 3, seed 1}
and per test dx_mm, dy_mm, yaw_deg) and `_kit/<id>/report.json`. No scores.json.

### Per load (first = Stage 1's `pairs/<id>.json`; edges as (a, b), reasons ignored)

| load | status first / perturbed | edges first / perturbed | in both | only first | only perturbed | two-way pairs first / perturbed | agree | waves | builds (extra) | drops not landed | changed drops (landed / disturbed set) | Kit s (drops s) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| r11_00050 | ok / ok | 3 / 5 | 3 | 0 | 2: b07 > b02, p06 > c02 | none / none | yes | 2 / 2 | 1 (0) / 1 (0) | 4 / 5 of 60 | 1 / 1 | 2154.8 (1831.7) |
| r07_00023 | ok / cycle | 8 / 5 | 4 | 4: p05 > p03, p07 > b04, p07 > p04, p07 > p05 | 1: p03 > p07 | none / p03 <-> p07 | no | 4 / none | 2 (4) / 0 | 7 / 5 of 70 | 2 / 7 | 2149.7 (2131.5) |
| r00_00031 | cycle / cycle | 5 / 5 | 4 | 1: b06 > p07 | 1: p04 > p08 | b06 <-> b08 / b06 <-> b08 | yes | none / none | 0 / 0 | 6 / 7 of 64 | 1 / 3 | 2007.8 (1989.7) |
| r02_00048 | cycle / cycle | 5 / 5 | 4 | 1: p08 > p02 | 1: b08 > b07 | b02 <-> b04, p02 <-> p08 / b02 <-> b04 | no | none / none | 0 / 0 | 9 / 8 of 70 | 1 / 6 | 2173.4 (2155.3) |

15 of 21 edges reproduced, 6 lost, 5 new, two-way pairs agree in 2 of 4.

- Reproduced: r11 b01 > b05, b01 > b06, p06 > p01; r07 b05 > b02, p01 > p02, p03 > b04, p07 > p03; r00 b02 > b05,
  b02 > p05, b06 > b08, b08 > b06; r02 b02 > b04, b04 > b02, p02 > p08, p03 > p05.
- By reason (the (a, b) that carry it): knock 9 of 12 reproduced, 3 lost, 3 new; need 3 of 5, 2 lost, 3 new; block 7
  of 10, 3 lost (all from r07's one drop L(p07)), 0 new. Three reproduced edges changed reasons: r07 b05 > b02
  knock+block -> block, r07 p01 > p02 block -> knock+block, r02 b04 > b02 knock+need -> need.
- r11 perturbed: build 1 ok from the 5 edges, 0 extra, max lateral 2.2 mm (first 1.8); wave 1 b02 b05 b06 c02 p01
  (first b05 b06 p01). r07 perturbed: "no valid one-at-a-time order: two-way pairs plate_03 <-> plate_07", no build
  (first: built in 2 builds with 4 extra). r02 perturbed: "no valid one-at-a-time order: two-way pairs bowl_02 <->
  bowl_04".
- The landed verdict flipped in 5 of 264 drops: r11 b02 w/o b07 (test 30) True -> False; r07 p07 last (22) and p03 w/o
  p05 (60) False -> True; r00 p08 w/o p04 (61) True -> False; r02 p02 w/o p08 (62) False -> True. The disturbed set
  changed in 17 drops, so 20 drops changed in all. The two Stage 1 verdicts marked as able to flip held: r00 b05 w/o
  b02 tilt 13.8 -> 9.6 / 15 (landed) and r02 b02 w/o b04 dz 21.7 -> 22.1 / 20 (not landed).

### Differing edges: the drops behind them, first -> perturbed

Limits: landed = lateral <= 15 mm (plate 18), dz <= 20 mm, tilt <= 15 deg (plate 16) and contact < 2 mm; disturbed =
moved > 10 mm or > 20 deg. Only disturbed dishes are stored, so "-" means under both. Triples are lateral mm / dz mm /
tilt deg. Offset = the perturbed drop's dx, dy in rack axes (radius) and yaw; the first run had none. In every drop
listed the context drift agrees within 0.6 mm between the runs.

- r11 b07 > b02, new (need). L(b02) (test 12) landed in both, 0.0 / 0.0 / 0.1 -> 0.1 / 0.1 / 2.0 (offset -2.45, 0.18
  (2.46) mm, -2.78 deg). b02 w/o b07 (30): landed 0.0 / 0.0 / 0.2 -> not landed 37.2 / 2.8 / 41.5, contact 0; offset
  2.21, 1.37 (2.60) mm, -2.15 deg; drift 0.1.
- r11 p06 > c02, new (knock). L(p06) (21) failed in both: 2.4 / 6.5 / 4.8, contact 8.87 -> 9.9 / 10.8 / 12.1, contact
  8.06. It moved p01 12.2 / 3.6 -> 15.2 / 10.0 and, perturbed only, c02 25.0 mm / 12.1 deg; offset -2.49, 1.03 (2.69)
  mm, 2.23 deg; drift 2.7. c02 is not a candidate of p06, so no R drop exists.
- r07 p07 > b04, p07 > p04, p07 > p05, lost (block each). L(p07) (22): not landed 15.7 / 5.3 / 2.6, contact 4.52 ->
  landed 0.2 / 0.0 / 0.0, contact 0; offset 1.28, -0.92 (1.58) mm, -1.37 deg; drift 0.6. p07 w/o b04 (39), w/o p04 (65)
  and w/o p05 (67) landed in both runs (lateral <= 0.5 mm).
- r07 p05 > p03, lost (need). L(p03) (12) landed in both (lateral 0.9 -> 1.5). p03 w/o p05 (60): not landed 10.3 / 5.7
  / 6.6, contact 5.63, moved b04 14.9 / 33.6 -> landed 1.2 / 0.4 / 0.1, contact 0, nothing moved; offset -0.25, -2.10
  (2.11) mm, -1.65 deg; drift 2.8.
- r07 p03 > p07, new (need). L(p07) (22) now lands (above). p07 w/o p03 (61) fails in both: 8.8 / 4.6 / 3.3, contact
  8.87, moved b04 21.0 / 21.4 -> 7.3 / 4.9 / 5.2, contact 6.78, moved b04 45.2 / 40.5 and p05 38.2 / 27.6; offset -0.54,
  0.90 (1.05) mm, 1.29 deg; drift 31.3 -> 31.4. With the reproduced p07 > p03 (p03 w/o p07, test 62, fails in both:
  contact 8.27 -> 13.08, drift 31.4 -> 31.6) it closes p03 <-> p07.
- r00 b06 > p07, lost (knock). L(b06) (15) failed in both: 27.5 / 22.3 / 31.1, contact 8.45 -> 5.4 / 18.7 / 19.3,
  contact 7.68. p07 moved 30.3 mm / 16.2 deg -> - (b08 23.6 / 30.5 -> 28.8 / 48.8); offset -1.98, -2.17 (2.94) mm,
  1.33 deg; drift 30.7 -> 30.6. p07 is not a candidate of b06.
- r00 p04 > p08, new (need). L(p08) (11) landed in both (lateral 0.2 -> 1.9). p08 w/o p04 (61): landed 0.2 / 0.0 / 0.1
  -> not landed 6.8 / 13.1 / 18.6, contact 6.58, moved p02 15.1 / 14.0; offset -0.54, 0.90 (1.05) mm, 1.29 deg; drift
  0.2. p04 and p08 are 15.76 mm apart; p08 touches p02.
- r02 p08 > p02, lost (need). L(p02) (16) landed in both. p02 w/o p08 (62): not landed 5.3 / 4.1 / 2.6, contact 6.47,
  moved b06 12.8 / 9.7 -> landed 0.2 / 0.0 / 0.1, contact 0, moved b06 11.8 / 18.0 (a disturbance in an R drop makes no
  edge); offset -0.73, -1.59 (1.75) mm, -1.04 deg; drift 20.4 -> 19.9. The other half stays: p02 > p08 (p08 w/o p02,
  test 61, fails in both: contact 5.29 -> 5.97; perturbed it moved b06 78.2 / 25.8, p07 31.0 / 20.9, p06 18.5 / 13.4,
  p03 13.1 / 10.6 and p05 11.0 / 7.0).
- r02 b08 > b07, new (knock). L(b08) (11) landed in both, 1.5 / 0.4 / 8.9 -> 0.9 / 0.4 / 2.3. b07 moved - -> 109.8 mm
  / 94.5 deg; offset -0.71, 0.00 (0.71) mm, 1.27 deg; drift 1.9.

### Applied offsets (perturbed run)

| load | drops | lateral > 2 mm | lateral median / max / min mm | abs(yaw) > 2 deg | abs(yaw) median / max deg |
|---|---|---|---|---|---|
| r11_00050 | 60 | 33 | 2.08 / 2.96 / 0.18 | 15 | 1.30 / 2.98 |
| r07_00023 | 70 | 37 | 2.06 / 2.96 / 0.18 | 16 | 1.28 / 2.98 |
| r00_00031 | 64 | 35 | 2.08 / 2.96 / 0.18 | 15 | 1.28 / 2.98 |
| r02_00048 | 70 | 37 | 2.06 / 2.96 / 0.18 | 16 | 1.28 / 2.98 |

A uniform 3 mm disc puts 56 % of drops over 2 mm (median radius 2.12 mm); here 53-55 %. The offsets depend only on the
seed and the test number, so all four loads drew one sequence: tests 1-60 got identical offsets in all four.

### Context drift, first / perturbed

| load | max mm | median mm | tests > 2 mm |
|---|---|---|---|
| r11_00050 | 16.6 / 16.6 | 0.15 / 0.1 | 6 / 6 of 60 |
| r07_00023 | 31.4 / 31.6 | 0.6 / 0.6 | 9 / 9 of 70 |
| r00_00031 | 30.7 / 30.6 | 0.2 / 0.2 | 7 / 9 of 64 |
| r02_00048 | 34.5 / 34.1 | 3.7 / 3.7 | 48 / 49 of 70 |

The context is never perturbed. Its drift changed by 0.1-0.5 mm in 63 of 264 tests and by more than 0.5 mm in 5: r00
test 5 (p01 last) 0.4 -> 5.4, r00 test 16 (c03 last) 0.3 -> 4.0, r02 test 2 (c06 last) 0.2 -> 3.7, r02 test 10 (p03
last) 3.7 -> 2.2 and r02 test 61 (p08 w/o p02) 19.4 -> 20.0. r02's lasting 3.7 mm state from test 27 to 70 reproduced.

### Odd points

- r07 turns from ok to cycle on one drop. L(p07) (test 22) failed in Stage 1 on contact (4.52 vs 2 mm, lateral 15.7 /
  18), which Stage 1's margin list counted as unable to flip. Here it lands cleanly with a 1.58 mm / -1.37 deg offset.
  That one verdict removes three block edges and turns p07 w/o p03's failure into the need edge p03 > p07, which closes
  p03 <-> p07. A contact failure is hit or miss: a 1-3 mm offset decides whether the dish meets a neighbour, and the
  20 % pose margins do not predict it.
- The plate two-way pairs changed and the bowl pairs held. r02 p02 <-> p08 vanished and r07 p03 <-> p07 appeared. Both
  run through a bowl that falls once both plates are out (r02 b06, drift 19.4-20.4 mm; r07 b04, 31.3-31.6 mm), so
  whether they read two-way hangs on single verdicts that flipped here (r02 p02 w/o p08 now lands, r07 L(p07) now
  lands). r00 b06 <-> b08 reproduced with every verdict behind it. r02 b02 <-> b04 reproduced on margins: b04 > b02 lost
  its knock and rests on need (b02 w/o b04 fails on dz alone, 22.1 / 20), and b02's knock on b04 shrank from 26.2 to
  11.8 mm, 18 % over 10 mm.
- Small offsets, large effects. r02 L(b08) at 0.71 mm / 1.27 deg knocked b07 109.8 mm / 94.5 deg (first: under 10 mm /
  20 deg). r00 p08 w/o p04 at 1.05 mm failed (tilt 18.6, contact 6.58). r11 b02 w/o b07 at 2.60 mm landed 37.2 mm /
  41.5 deg off (first 0.0 / 0.2). The 20 changed drops had offsets of 0.18-2.94 mm (median 1.69), the whole run 0.18-
  2.96 mm (median 2.06-2.08): the changes are not confined to the large offsets.
- Knock edges onto non-candidates come and go: r11 p06 > c02 (new) and r00 b06 > p07 (lost) both come from last-drops
  that fail in both runs. What a failed drop pushes varies with the release.
- The re-settle carries history. The context and its targets are identical, yet r00 tests 5 and 16 and r02 test 2
  drifted 3.5-5.0 mm more than in Stage 1, each right after a drop that ended in a different pose (r00 b02 last and b06
  last, failed in both runs; r02 p05 last, landed 1.4 mm off instead of 0.1). Stage 2's identical rerun had the same
  history and agreed bit for bit. None of these drops is behind a differing edge. r02's 3.7 mm state from test 27 on
  reproduced although test 26 (b01 w/o p04) landed 1.7 mm off instead of 4.2, so that landing does not set it.
- One seed: 15 of 21 is a single draw, and with the shared offset sequence the four loads are not independent samples.

### Stage 2-3 figures and page (2026-10-14)

pairs/ held only the five Stage 1 loads, so the 12 records and 12 scores of pairs_more/ (7 gate-accepted, 5 gate-rejected)
were copied in at 11:16 UTC (`docker exec -w /workspace/dishsim dishsim-isaac cp -p -n`, every copy byte-identical, _kit/
untouched): 17 records, 17 scores. Built 11:16:28-11:17:24 UTC (figures 29.4 s, page 10.2 s), then rebuilt 11:31:01-11:31:48
(28.8 s, 10.3 s) into the same folders after the scripts were fixed at 11:30:09 (two-way edge as a straight double arrow,
a two-reasons sentence, lead and footer); the rebuild is what is on disk. Commands as for v17 plus `--resettled-dir
data/results/dot_rules/dependency/pairs_resettled` and, for the page, `--rerun-dir
data/results/dot_rules/dependency/pairs_perturbed`; out `figures_v18` / `page_v18`; logs
`data/logs/dot_rules/{figures,page}_v18.log` (first build) and `{figures,page}_v18b.log`. All four `[RESULT] PASS`, no
traceback, no fix. The rebuild changed only the five cyclic depgraph PNGs and index.html; no file name changed.
figures_v18: 30 PNGs, the 13 non-depgraph ones byte-identical to figures_v17, 17 depgraph PNGs; no re-settled figure, the
log prints "r00_00031: the goal did not re-settle, no figure" and the same for r02_00048. page_v18: index.html 7,026,645
bytes, 62 files, 14,792,181 bytes; media/ 50 (v17's 38, 34 of them byte-identical, the four v17 depgraphs other than
r05_00014 redrawn, plus 12 depgraph PNGs), data/ 11 depgraph JSONs (v17's 4 unchanged plus the 7 new gate-accepted loads,
byte copies of pairs/<id>.json), none for the 6 gate-rejected; the 61 local src/href resolve and every shipped file is
referenced. grep -c: "two reasons" 1, "for the 11 that pass it" 1, "perturbed every release" 1, "15 of 21" 1, "must be
placed together" 1, "17 loads" 1, two-way 11, learned 0, each depgraph PNG 1. The 17 table rows match the records and
scores. Outside the dependency section only the lead and the 9 October note changed (17 loads, pairs dropped for the 11
that pass the gate).
- The first build drew each two-way pair as a bracket whose head sat on the start of a one-way edge from the same node, so
  r00 bowl_06 -> plate_07 and r02_00021 bowl_05 / bowl_08 -> bowl_06 looked double-headed. Rebuilt, each of those edges
  has one head and every two-way pair is a straight double arrow (all five cyclic graphs checked).
- The 12 added rows and figures follow load-id order after Best 1-5, not S rank.
- Read zoomed: r07, r06, r13 (titles match their rows, every head at its node, no red ring) and the rebuilt r00 and
  r02_00021. Top views of tight clusters stay hard to read: r02_00021 b05 / b08 / b06 nest with centres 10-23 mm apart and
  render as a knot of labels and heads; r07 p03/p07/p05/p04 and r06 b04/b07 -> p06 are cramped. r13's settled poses pool
  one dish (settled pooling_count 1); no caption says so.

Stages 2-3 published 11:35 UTC: https://claude.ai/artifact/F5vrhrNM4XnUk3ZXhJycY2 Version 18 (version id 1791977695-f008), figures_v18 / page_v18, index.html 7,026,645 B, 50 media + 11 data files (17 loads, perturbed-release comparison, re-settle results); the page is private until shared.

### Graphs under the Isaac stills (2026-10-14, Version 19)

User request after the 4-hour window: "add the dependency graph below under the corresponding Isaac Sim load image".
Change (Fable, a small direct fix, no runner): `isaac_section` of `frigidaire_dot_rules_page.py` now puts, under each
still of the five best restricted loads, the load's dependency graph (the same `media/depgraph_<id>.png` the dependency
section shows, so no new media), adds one caption sentence ("Below: its dependency graph from the ordered-pair test,
<dep_summary text>; the test, the edge list and the build are in Dependency graphs", linking `#dependency-graphs`), and a
lead sentence under the "Restricted: the five best" heading. The random loads and the five worst restricted loads have no
graph (none was tested), and the 12 graph loads of ranks 6-17 have no Isaac still, so they are unchanged. Page only, the
figures are v18's: built 14:05:17-14:05:27 UTC into `page_v19` with the v18 page command plus `--figures-dir
data/media/dot_rules/figures_v18 --out data/media/dot_rules/page_v19`, log `data/logs/dot_rules/page_v19.log`, `[RESULT]
PASS`, no traceback. index.html 7,029,273 bytes (+2,628 over v18), 50 media + 11 data files all byte-identical to v18
(61 of 61 compared), total 14,794,809 bytes; each of the five depgraph PNGs is referenced twice, the "Below:" sentence
appears 5 times, the lead once, 20 Isaac stills as before.

Published 14:06 UTC: https://claude.ai/artifact/F5vrhrNM4XnUk3ZXhJycY2 Version 19 (version id 1791986758-38ea), index.html only (the media and data files were kept from Version 18); the page is private until shared.
