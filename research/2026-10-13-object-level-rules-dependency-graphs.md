# Object-level dot rules and Isaac dependency graphs for HOTEC loads
Slug: 2026-10-13-object-level-rules-dependency-graphs

## Question
Does aggregating the dot rules at the object level (ratio of sums over a dish's 500 dots, not the mean of per-dot ratios) change how Original, New and New+J rank plate, bowl and cup poses, and which placement dependencies does an Isaac one-at-a-time build reveal in the best restricted loads?

## What's known (max 5)
No literature review (user: "no lit review"); what is known comes from this repo's own results.
- Dot Score Rules page, 2026-10-09 (artifact F5vrhrNM4XnUk3ZXhJycY2): seven per-dot rules at N = 262,144; a dish score is the mean of its 500 per-dot ratios; plate on edge 0.806 under Original, 0.336 under New. Matters here: a mean of ratios lets one rim dot that faces one spray point count as fully exposed.
- Grid run at N = 262,144 (`data/results/dot_rules/grid/N0262144`): 3.1 h for plate and bowl, 1.36e12 rays at 1.4e8 rays/s; the partial checkpoints store dish means only. Matters here: a ratio of sums needs the per-dot sums, so the grid must be recomputed (plus the cup, never gridded).
- Restricted sampler (8 October rules, rebuilt 2026-10-09, `frigidaire_dot_rules_restricted.py`): 351 of 895 tries fit; loads are packed release poses (variants lifted up to 60 mm), never settled; ranked under New+J, best r05_00014 at S 0.157. Matters here: these are the main loads; their poses are commands, not rest states.
- HOTEC bench sequence gate (`code/frigidaire/docs/hotec_bench.md`, `frigidaire_bench_kit.build_sequence`): dense loads are order-dependent; one Isaac build at a time learns an order (a knocker goes before what it knocked, a non-lander after its neighbours, up to 8 builds), takes support edges from the final contacts and adds a neighbour order within 15 cm; medium goals carried 17 and hard 23 constraints. Matters here: the ground-truth dependency machinery exists and is tested.
- Random farm `rl_20261009`: 252 Isaac-settled 24-dish loads with contact pairs and support paths recorded. Matters here: the "true complete" sampler option can read them without Kit.

## Gap
Nobody has checked whether the object-level aggregation alone removes Original's edge-case flaw (does the dish's Σ c·v / Σ c already behave like New?), nor what the dependency structure of a high-exposure packed load is once Isaac builds it one dish at a time: how many edges, how many placement waves, whether it is buildable at all.

## Candidate ideas (max 3)
1. Ratio-of-sums object level for all seven rules on the full grid with the cup, shown by an explorer toggle. Might work because the engine already produces every per-dot sum. Might fail because it costs 4.6 h of GPU and grows the page payload. Cheapest test: the grid at N = 1,024 for the three dishes (minutes).
2. The same object-level curves as a static three-slice figure (tilt, height, distance; 54 cells per dish). Cheapest, but it loses the explorer the user wants.
3. Isaac dependency graphs via the bench's `build_sequence` on the 5 best restricted loads; support, learned and neighbour edges; layered DAG plus top views. Might fail because a packed load may not build in 8 builds (then a labelled partial graph). Cheapest test: one load, r05_00014.

## Picked
1 and 3, because the user wants the explorer (full grid, one N on the page) and Isaac ground truth rather than Kit-free proxies for the dependencies.

## To grill
- Whether the D family's object level, equal to the dot mean by construction, is stated or hidden.
- Whether the page stays under the 16 MB artifact limit with a cup and three object-level entities in the explorer payload.
- Whether a load that fails all 8 builds still gets called a dependency graph.

## Follow-up 2026-10-14: the paper's definition
The dependency graph of Ren and Qureshi, "Multi-Stage Monte Carlo Tree Search for Non-Monotone Object Rearrangement Planning in Narrow Confined Environments" (IROS 2024, arXiv 2305.17175), is built over the goal arrangement: nodes are objects, an edge from o_j to o_i means o_i at its goal would sit in o_j's placing tunnel (the arm's swept path from the front opening), so o_j goes first; the graph must be a DAG, its topological sort with a longitude tie-break (farther from the robot first) fixes one MCTS stage per object; a random order dropped success on 7-8 objects from 100 % to 57.5 %. The initial arrangement enters only in the stages. Our benchmark has no arm, so the relation is restated for its move (a 3 mm release and an Isaac settle) as an ordered-pair test: A before B when only that order lands both dishes; see plans/2026-10-14-pairwise-dependency-graphs.md.
