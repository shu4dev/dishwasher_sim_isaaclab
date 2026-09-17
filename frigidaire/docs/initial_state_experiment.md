# Joint initial-state packing experiment

This experiment builds loaded initial states from the 176 accepted single-dish
trials in `results/random_poses/frigidaire/complete_20260910_seed0`. Its output is
a jointly measured load with both racks extended and the door open. Every accepted
load must also survive retraction of both racks in Isaac Sim 4.5.0.

The current run and its technical report live under
`results/initial_states/frigidaire/packing_20260911_seed20260911/`. The original
single-dish records remain unchanged.

## Reproduce a new run

From the repository root, with the existing `dishsim-isaac` container running:

```bash
bash frigidaire/scripts/experiment/run_initial_states.sh \
  --out-dir results/initial_states/frigidaire/new_run_seed20260911 \
  --seed 20260911 --budget-seconds 7200
```

Use a new output directory. The wrapper exposes the USD libraries already bundled
with the pinned image; it does not install software. Physics runs in a fresh Kit
process per attempted arrangement. CPU physics runs at 120 Hz with CCD; rendering
uses Isaac cameras. Runs are judged from saved validation records and `[RESULT]`
markers, because the Isaac Lab launcher can return zero after script errors.

After stopping a controller at a completed attempt boundary, resume the same
original budget with:

```bash
bash frigidaire/scripts/experiment/run_initial_states.sh --resume \
  --out-dir results/initial_states/frigidaire/new_run_seed20260911 --phase capacity
```

The resume controller refuses concurrent ownership, recovers completed but
unindexed attempts, preserves prior source snapshots, and does not reset the clock.
Its randomized phase alternates uniform subsets of the highest validated candidate
set with graph greedy proposals; the saved accepted distribution remains nonuniform.

## Sampling and search

Each source final pose is expressed relative to that trial's measured rack frame,
then composed into one common empty-appliance baseline. The basket is shared.
Each of the 176 templates contributes its original pose and eight independent
variants. Translation is uniform in a ball of radius 10 mm. A rotation axis is
uniform on the sphere and its angle is uniform from 0 to 10 degrees. This angular
distribution is explicitly not uniform over a geodesic ball in SO(3).

All 1,584 candidates retain source IDs, seeds, dimensions, masses and perturbations.
Variants are reusable placement templates for independent dishes. One candidate
cannot occur twice in a state. Selecting a loaded state is a subset/combinations
problem over the finite catalog; it is not an enumeration of rotation permutations.

Authored FCL collision primitives reject initial penetrations above 1 mm against
the actual baseline and between all object pairs, including cross-rack pairs.
Broadphase AABBs only accelerate these detailed queries. A conflict graph supplies
seeded greedy proposals and maximum-cardinality MILP proposals. Each MILP has at
most 60 seconds. Exact assignment exclusions prevent repeated tests while allowing
supersets that might provide additional support.

The controller reserves up to 80 minutes for capacity search, 30 minutes for ten
randomized states, and 10 minutes for images/reporting. A physically validated load
that reaches the finite catalog's geometric upper bound can end capacity search
early. Missing greedy proposals do not establish exhaustion of the catalog.

Randomized targets are three states near 25%, four near 50%, and three near 75%
of the highest count, rounded upward. Every generated state is independently
validated; density reductions and retries are recorded. Search proposals and
accepted states are not uniform samples of all physically possible arrangements.

## Physics and evidence

Dish actors are independent dynamic rigid bodies. Initial velocities are zero.
Settling may move them farther than the sampling perturbation bounds. The saved
state uses their measured poses, with proposed and source poses preserved separately.

A five-second observation must contain uninterrupted passing trailing one-second
windows. An early failed observation may restart within the twelve-second settling
allowance; a later start cannot extend that allowance. Rest checks include all
visual mesh vertices, support paths to each assigned rack, contact penetration,
and rack endpoints. The seated basket may support lower-rack dishes. Contact
connectivity does not certify force balance or resistance to arbitrary perturbations.

Both loaded racks retract, trying the reverse order from a fresh baseline if needed.
The nominal rack command is 0.08 m/s; the measured limit is 0.10 m/s with an explicit
0.00001 m/s numerical measurement tolerance. Peak penetration must remain below
2 mm, rest-window median maximum penetration below 1 mm, and final whole-mesh
containment must pass both world and measured-cabinet frames within 1 mm. Rest
limits are 5 mm position span, 3 degrees orientation span and 0.03 m/s vertex speed.

`attempts/<id>/` holds the manifest, physics result, trace and runtime log.
`states/` holds accepted measured initial states. `candidates.json` and
`compatibility.json` preserve the finite search domain. `source_snapshot/` and
`source_revisions/` preserve executable provenance and the development corrections
used during this run. Sparse maximum-penetration events retain full-rate evidence
for peaks that may occur between the 10 Hz trace rows.

The report separates the highest validated count from any finite-catalog bound.
Neither describes a global real-world capacity: the experiment does not test loading
accessibility, door closure, cleaning performance, or physical dishwasher calibration.

## Render or regenerate the report

```bash
scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_initial_state_render.py \
  --state results/initial_states/frigidaire/new_run_seed20260911/states/highest.json \
  --out-dir outputs/initial_state_render_review --headless --enable_cameras

scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_initial_state_report.py \
  --out-dir results/initial_states/frigidaire/new_run_seed20260911
```

Rendering replays measured transforms without advancing physics and audits the
replayed transforms. Use a fresh render directory; render evidence is not overwritten.
