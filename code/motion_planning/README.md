# motion_planning

Block 2a MOTION PLANNING of `docs/repo_map.txt` (reach the goal): moves the dishes from the start to the goal, one
teleport move at a time; Isaac settles each. Track A: `greedy_offline` and `rrt_connect`. The package is
`motion_planning`.

| Path | What |
|---|---|
| `rearrange.py` | the episode driver: instances, `run_episode`, the greedy baseline, the fault knobs (Kit-free by the oracle/world seam) |
| `rrt.py` | arrangement-space RRT / RRT-Connect / RRT* planners (Kit-free): a state is a full arrangement, an edge one legal teleport |
| `planner.py` | the bench-side planning helpers for the Frigidaire twin: the FCL `PlannerWorld` (appliance mirror, counter slab, objects), the support graph and the greedy sequencer with the counter cap |
| `tests/` | the driver's and the RRT planners' toy-oracle checks, the planner world (Kit-free) |
| `docs/planner.md` | reference of the planner helpers |

New algorithms implement `reset(instance, world)` / `next_move(obs)` and join the kit's `EPISODES`
(`code/sampler/scripts/frigidaire_bench_kit.py`); the run stage (`code/paths/episodes.py`) runs them.

Planner helpers used by the HOTEC benchmark (FCL world, support gate, sequencer): [docs/planner.md](docs/planner.md);
the pool-search planner was retired 2026-09-29.
