# Hardware-aware sampler: no goal for this hand at posture_v2 in four runs (best 14 of 15, basket out)

Plan: plans/2026-11-05-hardware-aware-sampler.md. Four loops ran on the Mac, none producing an all-pass goal:

| Run | Basket | Plate cuts | Time (EST) | Result |
|---|---|---|---|---|
| 1 | in | per pose | 2026-11-05 23:26 to 01:16 (110 min) | stopped at 6 iterations, 4 plates failing |
| 2 | in | by slot | 01:21 to 02:00 (39 min) | no packing at level 0 or 1 in iteration 4 |
| 3 | out for the robot | by slot | 02:02 to 04:01 (119 min) | stopped at 6 iterations, 3 plates failing |
| 4 | out, resumed from run 3 | slot, bowl site, culprit families | 04:04 to 06:42 (158 min) | 14 of 15 in iteration 8, then no packing at level 0 or 1 |

So there was no box window. So there was no box window: the Isaac stages, the goal still and the re-gate did not run, and the box stayed STOPPED with 0 billed minutes.

Command (the loop, once):

    PYTHONPATH=code/src .venv-kitfree/bin/python code/planner/bench.py --kinds plate,bowl,cup --count plate=5,bowl=5,cup=5 \
        --counter-allowance 3 --seed 0 --to fit --hardware data/results/robot/mount/posture_v2.json \
        --start-from data/results/bench/samples/frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s0.json \
        --out data/results/bench/samples_hw

The sample `data/results/bench/samples_hw/frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s0__hw.json` validates against the schema. Its ledger has load, initial and gate_initial copied from s0 and fit FAIL. The iteration files, cuts and logs are in `_work/<id>/hwgate/` (iter_1..iter_6, bans.json) and `data/logs/bench/hw/loop.log`.

## Run 1: the loop with pose cuts

Every iteration packed at level 0 on the first solve (1487 poses, 223,694 conflicts, about 3 s per fit). Gate seconds include the deletion-filter calls. No failing dish had a carry RRT timeout, so the 180 s retry never ran.

| Iter | Direct | Failed (slot: cause; culprits -> cut) | Cuts | Gate s |
|---|---|---|---|---|
| 1 | 7/15 | plate_01 rear_07: seats on basket -> ban; plate_05 rear_05: seats on basket; plate_01 -> ban; plate_02 front_05: seats on bowl_05 -> pair bowl_05; plate_04 front_02: no descent start; 3 -> pair plate_02; plate_03 front_00: no descent start; 4 -> no-good; bowl_02 upper_00: start collides; bowl_04 -> ban; bowl_03 frontright: hand not above -> ban; cup_02 right_3: start collides -> ban | 5 ban, 2 pair, 1 no-good | 610 |
| 2 | 9/15 | plate_01 rear_08: basket -> ban; plate_05 rear_05: basket -> ban; plate_02 rear_00: no descent start -> pair plate_05; plate_03 front_00 -> pair plate_04; bowl_01 upper_00: start collides -> ban; cup_02 left_3: no IK -> pair cup_03 | 3 ban, 3 pair | 1142 |
| 3 | 8/15 | plate_05 front_08: basket -> ban; plate_02 front_06 -> pair plate_05; plate_04 front_02 -> pair plate_02; plate_03 front_00 -> pair plate_04; bowl_05 rear: no IK -> ban; bowl_01 upper_00 -> ban; cup_02 left_3 -> pair cup_03 | 3 ban, 4 pair | 2115 |
| 4 | 11/15 | plate_02 front_08: basket -> ban; plate_01 rear_04: seats on bowl_03 -> ban; plate_03 front_00 -> pair plate_04; cup_02 right_2: start collides -> ban | 3 ban, 1 pair | 1128 |
| 5 | 10/15 | bowl_05 frontright: hand not above -> ban; plate_05 front_08: basket -> ban; plate_02 front_05 -> pair plate_05; plate_04 front_02 -> pair plate_02; plate_03 front_00 -> pair plate_04 | 2 ban, 3 pair | 663 |
| 6 | 11/15 | plate_01 rear_08: basket -> ban; plate_04 front_05: seats on bowl_05 -> pair bowl_05; plate_05 rear_02: seats on bowl_01 -> ban; plate_02 rear_00: no descent start; 3 -> no-good | 2 ban, 1 pair, 1 no-good | 598 |

In total there were 18 bans, 14 pair no-goods and 2 configuration no-goods. bans.json holds 32 bans and 2 no-goods under the level-0 digest and posture_v2. Plates account for 24 of the 34 failing dish-iterations, bowls for 6 and cups for 4. Every failure in iteration 6 is a plate. The plate failures have three causes: the seat lands on the SilverwareBasket (rear_04..08 and front_08 with the basket in), the seat lands on a neighbouring bowl, or there is no descent start with the earlier plates in the scene.

Each cut is keyed by one pose, which is too fine for plates. A slot carries many leans and offsets, so after every cut the packer moves the plate back into the same slot at another pose key. plate_03 at lower_front_00 failed against plate_04 at lower_front_02 in iterations 1 to 5 under five different key pairs, then passed in iteration 6 at spin -30.

## Run 1, iteration 6's goal (not valid)

| Dish | s0 slot | iter 6 slot | Gated pose | Direct | Move | Choice | Spin |
|---|---|---|---|---|---|---|---|
| bowl_01 | upper_bowl_07 | lower_rear | packed | yes | D-ring-side | foot-side | 345 |
| bowl_02 | lower_frontright | upper_bowl_04 | packed | yes | D-ring | foot-out | 15 |
| bowl_03 | upper_bowl_03 | lower_frontright | packed | yes | D-ring | foot-out | 15 |
| bowl_04 | upper_bowl_11 | upper_bowl_11 | packed | yes | D-ring-side | foot-side | 45 |
| bowl_05 | upper_bowl_01 | lower_frontright | packed | yes | D-ring | foot-out | 60 |
| cup_01..05 | | glass_left_2, right_2, left_4, right_1, left_1 | rest table | 5 yes | D-side-down | side-down (11 mm) | |
| plate_01 | lower_rear_05 | lower_rear_08 | rest table | no | | | |
| plate_02 | lower_front_07 | lower_rear_00 | rest table | no | | | |
| plate_03 | lower_front_00 | lower_front_00 | rest table | yes | I4-wall20 | wall pinch | -30 |
| plate_04 | lower_rear_00 | lower_front_05 | rest table | no | | | |
| plate_05 | lower_front_04 | lower_rear_02 | rest table | no | | | |

Only bowl_04, cup_05 and plate_03 kept their s0 slot. S_ref: none, since no goal was built. s0's S_ref is 0.513.

## Run 2: plate cuts by slot

The coordinator's rule (2026-11-06) changes only the plate cuts.
- A plate whose pose is banned has every plate pose of its (rack, slot) banned: 78 family poses, all leans and offsets.
- A plate that the deletion filter shows failing only beside one plate c gets a cut over every pose pair of the two slots.
- A bowl culprit keeps the pose pair, and bowls and cups keep their pose cuts.

The cuts are stored as slot records in bans.json and expanded at each level. Run 2 reloaded run 1's bans.json (32 pose bans, 2 no-goods) and re-expressed run 1's plate cuts under the slot rule before its first fit. That gave 12 slot records: 6 slots and 6 slot pairs. Run 1's iteration folders and sample are kept in `_work/<id>/hwgate/run1/`; run 2 rewrote iter_1..iter_4 and the sample, whose ledger is again three copied start stages and fit FAIL. Log: `data/logs/bench/hw/loop2.log`.

| Iter | Direct | Failed (slot; culprits) | Cuts | Gate s |
|---|---|---|---|---|
| 1 | 11/15 | bowl_05 frontright (plate_05); plate_02 front_05 (bowl_05, plate_05); plate_04 front_03 (3); plate_03 front_01 (4) | pair bowl_05/plate_05; slot pair front_05/front_07; 2 no-goods | 275 |
| 2 | 12/15 | plate_04 front_03 (3); plate_03 front_01 (4); plate_01 rear_03 (plate_05, still fails without it) | slot pair front_03/front_05; no-good; slot rear_03 | 619 |
| 3 | 10/15 | plate_01 rear_06 (none); plate_02 front_06 (bowl_05); plate_04 front_03 (2); plate_03 front_01 (3); bowl_01 upper_bowl_04 (none) | slot rear_06; pair plate_02/bowl_05; slot pair front_03/front_06; no-good; ban bowl_01 | 1103 |
| 4 | none | no packing at level 0 or level 1 (both INFEASIBLE) | | 0 |

The final cut state has 35 pose bans and 6 configuration no-goods. It also has 17 slot records:
- 8 plate slots banned: lower_front_08 and lower_rear_02 to lower_rear_08.
- 9 slot pairs, all in the front bank except rear_00 with rear_05: front_00/02, 02/05, 02/06, 03/05, 03/06, 05/07, 05/08, 06/08.

That leaves the five plates 11 slots (front_00 to front_07, front_09, rear_00, rear_01) under those pairs, the collision and two-cycle conflicts, and the lower-rack bowls. CP-SAT finds no packing of all 15 dishes at either level.

The verdict "infeasible for this hand at this mount" holds under the slot rule. That rule generalizes each failing plate pose to its whole slot and each confirmed plate pair to the two whole slots. The evidence behind it is 3 to 5 failing plates per iteration in both runs:
- the seat lands on the SilverwareBasket in the rear bank and at front_08;
- the seat lands on a lower-rack bowl;
- there is no descent start with the neighbouring plates and the frontright bowls in the scene.

plate_03 at lower_front_01 failed in all three gated iterations with three or four culprits that no single removal cleared, so it only ever produced configuration no-goods.

## Run 3: the basket lifted out for the robot

Run 3 used the variant of the robot rounds, `--hardware-basket out`:
- The access order and the deletion filter run with --no-basket.
- The candidates record basket false.
- The sample is `..._s0__hwnb.json`, whose ledger is again three copied start stages and fit FAIL.
- The bans file is fresh, keyed `|posture_v2|nobasket`.
- provenance.hwgate records basket false with the rule that the basket is lifted out for the robot while the dish poses stay legal with it in, so the Isaac stages would run as the benchmark defines them.

The slot cuts are kept. Log: `data/logs/bench/hw/loop3.log`.

| Iter | Direct | Failed (slot; culprits) | Cuts | Gate s |
|---|---|---|---|---|
| 1 | 8/15 | plate_05 rear_05 (plate_01); plate_02 front_05 (bowl_05); plate_04 front_02 (3); plate_03 front_00 (4); bowl_02 upper_bowl_00 (bowl_04); bowl_03 frontright (none); cup_02 right_3 (none) | 2 slot pair, 1 pair, 3 ban, 1 no-good | 611 |
| 2 | 10/15 | plate_05 rear_05 (plate_01); plate_02 front_08 (bowl_05); plate_04 front_05 (2); bowl_01 upper_bowl_00 (2); cup_02 left_3 (cup_03) | 2 slot pair, 2 pair, 1 no-good | 655 |
| 3 | 9/15 | plate_05 rear_04 (plate_01); plate_02 rear_00 (3); plate_03 front_00 (3); bowl_03 frontright; bowl_01 upper_bowl_00; cup_02 left_3 | 3 slot pair, 1 pair, 1 ban, 1 no-good | 3176 |
| 4 | 10/15 | plate_05 rear_04 (plate_01); plate_02 rear_01 (3); plate_03 front_01 (2); bowl_01 upper_bowl_03 (none); cup_02 right_2 (none) | 2 slot pair, 2 ban, 1 no-good | 1119 |
| 5 | 11/15 | plate_02 front_08 (bowl_05); plate_01 rear_04 (plate_05); plate_04 front_06 (2); bowl_01 upper_bowl_00 (bowl_02) | 1 slot, 1 slot pair, 1 pair, 1 ban | 634 |
| 6 | 12/15 | plate_04 front_05 (bowl_05); plate_05 rear_02 (bowl_01, still fails without it); plate_02 rear_00 (3) | 1 pair, 1 slot, 1 no-good | 588 |

Across run 3 there were 10 slot-pair cuts, 2 slot bans (rear_02, rear_04), 6 pose pairs, 7 pose bans and 5 configuration no-goods. With the basket out the rear bank opens: only 2 rear slots were banned, against 7 with the basket in. The best iteration still left 3 plates failing.

The blockers move to the lower-rack bowls. bowl_05 at lower_frontright and bowl_01 in the lower rear are culprits of the front and rear plates in every iteration, and plate_05 failed beside plate_01 in four iterations under four slot pairs. bowl_01's upper_bowl_00 pose (descent start colliding beside bowl_02 and bowl_04) and cup_02 (start collides or no IK at four glass slots) were the non-plate repeats. The start side is unchanged with the basket out: 1 of 4 rack dishes and 11 of 11 counter dishes are pickable.

## Run 4: bowl culprits cut by site, resumed from run 3

The coordinator's last rule (2026-11-06) adds two plate cuts:
- A bowl culprit that the deletion filter confirms cuts the plate's slot against the bowl's site family, for example lower_frontright or the lower-rear bowl zone, as one clause.
- A plate with two or more culprits that no single removal clears cuts its slot against every culprit's family in one clause, besides the configuration no-good.

Run 4 reloaded run 3's nobasket bans and re-expressed run 3's two bowl-culprit pose pairs and three plate configuration no-goods as 2 site cuts and 3 family cuts before the first fit. It ran with `--hardware-basket out --hardware-max-iter 12`. Run 3's folders are in `_work/<id>/hwgate/run3/`. The __hwnb sample was rewritten; its ledger is the copied start stages and fit FAIL. Log: `data/logs/bench/hw/loop4.log`.

| Iter | Direct | Failed (slot) | Cuts | Gate s |
|---|---|---|---|---|
| 1 | 12/15 | bowl_03 upper_bowl_00; plate_04 front_03; plate_03 front_00 | 1 ban, 2 slot pair | 1117 |
| 2 | 10/15 | bowl_03 lower_rear; bowl_05 lower_rear; bowl_01 upper_bowl_01; plate_04 front_05; plate_03 front_01 | 3 ban, 2 slot pair | 1138 |
| 3 | 12/15 | plate_05 rear_00 (bowl_05); plate_04 front_03; plate_03 front_01 | 1 slot x site, 2 slot pair | 1114 |
| 4 | 12/15 | plate_05 rear_03; plate_02 rear_01; bowl_01 upper_bowl_04 | 2 slot, 1 ban | 1059 |
| 5 | 11/15 | plate_02 front_06; plate_04 front_04; plate_03 front_00; bowl_01 upper_bowl_01 | 1 slot, 2 family, 2 no-good, 1 pair | 2136 |
| 6 | 12/15 | plate_02 front_07 (bowl_05); plate_04 front_04; plate_03 front_00 | 1 slot x site, 2 slot pair | 1106 |
| 7 | 12/15 | bowl_05 upper_bowl_00; plate_04 front_04; plate_03 front_01 | 1 ban, 2 slot pair | 638 |
| 8 | 14/15 | plate_05 rear_06: its seat 1 mm up the tine axis on plate_01 (rear_08) and two tines, a perch under the seat rule | 1 slot pair (rear_06/rear_08) | 906 |
| 9 | none | no packing at level 0 or 1 (both INFEASIBLE) | | 0 |

Iteration 8's load was one plate short of a goal. Its cut closed the model: under the accumulated cuts, no packing of all 15 dishes remains at either level.

The binding constraints are all plate placements. Run 3 and run 4 together hold:
- 20 pose bans and 7 configuration no-goods;
- 5 slot bans: lower_front_06 and lower_rear_01 to lower_rear_04;
- 21 slot pairs, adjacent plates whose descent start or seat meets the neighbour;
- 4 slot-by-site cuts: front_05, front_07 and front_08 against the frontright bowls, rear_00 against the lower-rear bowls;
- 5 family clauses.

In the front bank, plate_03 at front_00 or front_01 failed beside plate_04 at front_03 or front_04 in seven of eight iterations. In the rear bank, every pair of neighbouring plates (rear_04 to rear_08) and rear_00 beside a lower-rear bowl got cut. Bowls and cups pass once their upper sites are sorted out, by ban and pair: from iteration 3 on, at most one bowl failed per iteration and no cup failed.

## The start side

initial.robot and provenance.hwgate_initial: 1 of 4 rack dishes is pickable at its start pose. cup_01 passes. plate_04 lies flat (no carry leg, no descent start), plate_05 has no IK at the seat, and bowl_04 is mouth up (hand not above). The counter dishes are 11 of 11 pickable by their kind's pick at the posture_v2 spot. That verdict comes from the loop's last gate rows, with the plates on the plank and the bowls and cups mouth down. The start's counter stacks are not re-presented. Under --start-from the stage records this and goes on, so s0's start would make the sample hardware-invalid at the start anyway.

## Mac checks

py_compile is clean on every changed file. pytest gives 237 passed, 11 failed (the known 11) and 24 subtests, the baseline.

## Files changed

- code/src/bench/build.py: stage_fit (Kit-free: plan_goal, or the loop under --hardware, then the loads files), stage_build (the Kit build of those files, goal.robot copied in), fit_solve extra_bans / key_cuts, fit_pack pose_key and relaxed=False.
- code/src/bench/hwgate.py (new): the loop, the candidate writer and initial_check; since run 2, the plates' slot cuts (slot_keys, slot_bans, add_slot_record).
- code/src/bench/sample.py (fit stage, copy_start, rel_root), schema/sample.json (stage enum fit), gate.py (gate_initial check under --hardware), code/planner/bench.py (--hardware, --hardware-basket in|out, --start-from; ids __hw / __hwnb). hwgate passes --no-basket through to every access-order call and to the start-side check when the basket is out.
- code/execution/frigidaire/frigidaire_access_order.py: --candidate, --direct-only, --gate-one / --scene, the candidate load dir (symlinks to <s0>__fifteen/partA_* and partA_cup_side_down5 of __fifteen_r3, presentation.json copied).
- In candidate mode, frigidaire_access_order.py now honours --no-basket (run 1 and run 2 forced the basket in).
- frigidaire_robot_reach.py: the no_guided row flag (the tine-guided release off), and the pass of D-ring-side and D-side-down now requires release_check.ok. frigidaire_placing_loads.py: build_releases returns {} with no names or no goal record, and --fifteen reads goal.robot.
- data/logs/bench/hw/: start_box.sh, window.sh, box_run.sh, box_watchdog.sh, pull.sh (written, not used).

## Deviations

- The "whole-load" no-good covers the gated configuration: the dish plus the earlier dishes of its rack in its scene. That is a subset of the load and a stronger cut.
- The 180 s narrow retry applies to plates only, because the gate reads _retry_s only in the plate carry. It never triggered in this run.
- release_check.ok is required only where the gate computes one (D-ring-side, D-side-down). D-ring and I4-wall20 judge their release inside ring_release and i4_release.
- The 5 mm cup hold was added as a second cup choice. No cup needed it.
- The Mac has no Warp, so the fit's tiebreak is the seeded noise and not isolated exposure.
- The candidate JSONs are minimal four-block tool inputs and are not schema samples.
- Without --start-from, a failing start rack dish fails gate_initial and the next seed is tried. This is a seed-level re-roll and untested here.
- Run 2 re-expressed run 1's plate pose cuts as slot cuts before its first fit, so nothing learned was lost and nothing was gated twice.
- Run 4 re-expressed run 3's bowl-culprit pose pairs and plate configuration no-goods by site and family. A family clause takes any pose of each distinct family, so two culprit bowls in one site collapse into one group.
- Run 4 raised HW_MAX_ITER to 12 for that run only, on the coordinator's instruction (the plan says 6). The loop ended at iteration 9 anyway, with no packing left.
- A plate that still fails after the deletion filter removes its one culprit gets the slot ban, the same as a plate with no culprit. Plates with two or more culprits that no single removal clears keep the configuration no-good.
- Not done: the docs paragraphs (robot.md, the bench README) and the fake-Kit chain check.

## Missing

No valid __hw or __hwnb sample, no goal still, no S_ref (s0: 0.513), no re-gate, and 0 billed minutes: the box was never started and stays STOPPED. With the basket in, the plates leave no packing under either cut granularity (pose cuts did not converge in 6 iterations, slot cuts closed the model in 4). With the basket out, run 3 reached 12 of 15 and run 4 reached 14 of 15. Run 4's last cut (plate_05 perched on plate_01 in the rear bank) left no packing. Five plates in the lower rack beside the lower-rack bowls do not fit this hand's direct releases at posture_v2 under these cuts. The candidates for the next round are a plate pre-filter run alone (empty-rack seat and descent per pose, which separates the basket and hardware failures from the scene failures), a different mount, or fewer plates.

Suggested commit: `Add hardware-aware fit loop and direct-only gate`
