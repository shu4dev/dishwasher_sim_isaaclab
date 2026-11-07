# optimal_arrangement

Block 3b OPTIMAL ARRANGEMENT of `docs/repo_map.txt` (its own load): the arrangement track B builds from the start alone,
as poses in the racks, judged certified by the joint gate and scored S / S_ref. The package is `optimal_arrangement`:
`certify.py` holds `own_load` (plan with `code/arrangement_optimization/frigidaire_bench.py --plan`, the joint Isaac gate
`own_gate`, the one-dish-at-a-time build `code/sampler/scripts/frigidaire_bench_kit.py --sequence-plan`, resequence, up
to `PLAN_ATTEMPTS`) and `final_score` (S_final of an episode's final load, the fair score over the roster, unracked = 0).
The run stage `code/paths/episodes.py` imports them; its rows land in `runs/<sample>.json` (`own_loads`, `S_final`,
`S_over_S_ref`). No scripts of its own.
