# Dependency graphs by physical cause (Frigidaire FDPC4221AS, HOTEC set)

Reference for agents working on `code/planner/frigidaire/frigidaire_dependency_{kit,clips,page,score}.py`. Plans and
run notes: `plans/2026-10-16-physical-cause-graph.md` (the method), `plans/2026-10-16-labelled-views-overlap-rule.md`
(the overlap rule, the four-view load image, the graph variants), `plans/2026-10-21-kind-ceiling-score.md` (the score)
and their `results/` notes. Page: Dependency graphs, https://claude.ai/artifact/AkiJZLtFZnFtXYUxSEHJmm (Version 13,
2026-10-21). The score: the fair score in [exposure.md](exposure.md).

## The claim

A sample is a restricted packing of the 24 HOTEC dishes (8 plates, 8 bowls, 8 cups;
`data/results/dot_rules/restricted/loads/<id>.json`, `frigidaire_dot_rules_restricted.py`) that Isaac built one dish at
a time. It is valid when three things hold, and it claims nothing more:

1. **Physically possible and stable**: the built poses pass the bench's joint gate
   (`frigidaire_initial_state_validate.py`: all 24 settle at once with a peak penetration under 5 mm and a last-window
   median under 1 mm, both racks retract, every dish stays contained).
2. **One sequential order exists**: the witness order, replayed from empty racks in a fresh Kit process with the same
   move, lands all 24 within the bench's at-goal tolerance of the built poses.
3. **An arrow only for a physical cause** in the finished load. No arrow means no physical cause was found, not that any
   order works.

## The sampler (one Kit process per packing)

- **Move** (`frigidaire_bench.insertion_release`): a plate stands upright in its slot, 3 mm toward the side its packed
  pose leans to; a bowl or cup keeps its packed pose. It starts with its lowest collision point 20 mm above its rack's
  rim wire (its top under the tub ceiling) and goes down in 5 mm steps, each checked against the placed dishes and the
  appliance with the bench checker's 1 mm allowance; a placed dish in the way (or a blocked start) blocks the move, the
  rack alone stops it; it is let go 20 mm above the lowest clear step and settles 150 + 60 ticks at 120 Hz.
- **Order**: a seeded random permutation whose plate positions are refilled per bank in chain order, the slot furthest on
  the lean side first (an upright plate tips about 30 deg into a lean and overhangs the next 3-4 slots;
  `frigidaire_bench_kit.chain_order`); bowls and cups stay where the permutation put them.
- **Landing**: within `DEV_MAX_M` (80 mm) of its target, contact under 2 mm, no placed dish moved out of its tolerance,
  and the **overlap rule**: no pair among the placed dishes, nor any dish against the racks, basket, cabinet or door, past
  the joint gate's own 1 mm preflight (the same collision pieces, `InitialCollisionChecker.pair`).
- A dish that does not go in is undone and **re-posed** at up to 5 other free poses of its kind with a clear descent
  (`Reposer`; a plate along its own bank's chain only), else **deferred** (a plate after the plates still queued, a bowl
  or cup to the back); the packing fails when no dish left goes in. No backtracking.
- The record: `<run>/<id>.json` with procedure `physical`, the built poses (`entries_settled`, `targets_world`), the
  chosen poses and moves, the attempts, the witness order and its tick poses `_kit/<id>/traj/witness.json`; or status
  `resettle_failed`.

## The graph (`--certify`, one fresh Kit process)

- Rebuild the witness order from empty racks (`build_sequence(fixed=...)`): certified = 24 of 24 land.
- **Support**: every dish-dish contact at rest whose net contact force on b from a points up by more than 0.05 N (the
  force sign is calibrated on the dish-rack contacts; the higher-centre rule when that fails). Each supported dish is
  dropped by the move into the finished load without its supports (dishes covering its path and everything resting on
  it or on a removed support also out), with each support alone and, with several, with all of them and with only one
  removed. A hard support (an arrow): b fails without a alone and lands with it. A support only needed as one of several
  is a disjunction, kept in the record, not drawn.
- **Access** (geometric, no Isaac): b before a when a's final pose fails the 1 mm allowance against b's own insertion path.
- A pair with arcs both ways loses both (conflicts); an arc the witness order breaks is not drawn; a dish in no arrow is
  free; layers make the DAG drawing. The page draws each graph three ways (direct, its transitive reduction, its closure),
  the same order information.

## Commands

On the Brev box (recipe: CLAUDE.md, "Research workflow (repo)"), from the repo root. The loop is host python; it
`docker exec`s every job into `dishsim-isaac`, Kit jobs on cores 0-13 (taskset j, j+16), clips and the score on 14-15:

```bash
setsid nohup python3 code/planner/frigidaire/frigidaire_dependency_kit.py --sample \
    --out-dir data/results/dot_rules/dependency/<run> --target-valid 5 --max-packings 20 --max-wall-seconds 10800 \
    [--resume-from data/results/dot_rules/dependency/<earlier run>] > data/logs/dot_rules/<run>.out 2>&1 &
```

Per packing it runs exactly these (also by hand, one at a time):

```bash
code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --load <id> --out-dir <dir> --headless
code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_kit.py --load <id> --out-dir <dir> --manifest-out <dir>/_gate/<id>/manifest.json
code/util/run_kit.sh code/initialization/frigidaire/frigidaire_initial_state_validate.py --manifest <dir>/_gate/<id>/manifest.json --out-dir <dir>/_gate/<id> --max-wall-seconds 600 --peak-penetration-m 0.005 --headless --device cpu
code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --load <id> --out-dir <dir> --certify --gate <dir>/_gate/<id> --headless
code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_clips.py --record <dir>/<id>.json --out data/media/dot_rules/dependency_<run>/<id>
code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_score.py --dependency-dir <dir> --loads <id>
```

Outputs: records and `_kit/`, `_gate/` under `data/results/dot_rules/dependency/<run>/`; `scheduler.log`,
`scheduler_state.json` and one log per job under `data/logs/dot_rules/<run>/`; clips under
`data/media/dot_rules/dependency_<run>/<id>/`. On the Mac (Kit-free): the kind-ceiling score and the page.

```bash
python3 code/planner/frigidaire/frigidaire_dot_rules_relative.py --grid-payload data/media/dot_rules/page_v22/index.html \
    --dependency-dir data/results/dot_rules/dependency/samples_v5 --summary
python3 code/planner/frigidaire/frigidaire_dependency_page.py --records r05_00027=data/results/dot_rules/dependency/samples_v5/r05_00027.json \
    --clips r05_00027=data/media/dot_rules/dependency_samples_v6/r05_00027 \
    --state data/logs/dot_rules/samples_v6b/scheduler_state_merged.json --out data/media/dot_rules/dependency_page_kind
```

The second reproduces Version 13 byte for byte. Publish with the Artifact tool to the same URL, `root` = the page dir
and a files map of its `media/*` (a dropped file set to null).

## Results on record

`samples_v5` (2026-10-16): 1 valid of 20 packings, `r05_00027` (1 hard support, 11 access arrows, 15 free dishes, 4
layers; S 0.3551, surface reached 0.2776). 17 of the 18 new packings stopped in the sampler; `r07_00012` failed the
joint gate's preflight on its built poses, then with the dish-pair overlap rule (`samples_v6`) built all 24 and failed
the preflight on `plate_06`/`SilverwareBasket`, and with the dish-appliance pairs too (`samples_v6b`) its sampler placed
23 of 24; `r00_00038` failed the gate on the loaded lower rack's retraction speed both ways.

## Landmines

- **Session history changes PhysX outcomes**: the same order gave other results after other builds in the same Kit
  session (8 of 53 library orders; research note below). One load per Kit process, and `--certify` always runs in a
  fresh one.
- PhysX is not reproducible across sessions: poses are stored and reused, never re-derived from a seed.
- **The overlap rule**: without it a load the sampler built can fail the gate's 1 mm preflight before any settle
  (`r07_00012`: `bowl_03`/`bowl_06` 1.64 mm, then `plate_06`/`SilverwareBasket` 2.23 mm); the rule checks dish pairs
  and dish-appliance pairs.
- The gate's rack-speed check flakes (`closure_failure`): the loop reruns it once with the lower rack first.
- The sampler, not the graph, is the bottleneck: the restricted packings were made for exposure, not for insertion.
- `--resume-from` imports an earlier run's built samples; a record's `rejudged.from` (written by the retired swap-test
  re-judging) names its parent run's folder, whose packings are skipped too.
- The kind-ceiling score only rescales the per-dish New+J saved in a load's `.relative.json`, which the retired per-dish
  reference pass wrote; a new sample has its `.scores.json` (absolute S) but no `.relative.json`, so its page caption
  shows the absolute S until that file exists.
- The page's graph PNGs come from `rsvg-convert`: another librsvg version differs by a few pixels.
- Stop jobs by PID; the loop kills its own jobs by their `--load <id> ... --max-wall-seconds` command line.

## History

From 2026-10-13 to 2026-10-16 the study tried build-learned graphs, ordered-pair drop tests on jointly settled loads
(v18, v20, perturbed reruns, goals re-settled one dish at a time), then six rounds of a sequential sampler: swap tests
with an order library, conditional edges, a learning loop, backtracking by state restore and a doubled tolerance; none
gave a valid sample. Why: `research/2026-10-16-dependency-graph-six-rounds.md`. Their code is in git history before
2026-10-22, their run notes in `results/`.
