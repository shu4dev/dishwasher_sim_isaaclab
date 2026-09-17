# Random dish poses and rack closure

This experiment tests a single dinner plate, bowl, or mug in each Frigidaire rack.
The dish is released at a random position and full 3-D orientation, settles under
gravity, and rides the selected rack inward. The other rack stays retracted and
the door stays open. Dish dimensions, masses, and appliance geometry are unchanged.

The [technical sampling report](random_pose_sampling_technical_report.md) explains
the exact probability distribution, why this is sampling rather than permutation
enumeration, statistical uncertainty, and how to choose a future experiment.
It is also available as a [printable HTML report](../../outputs/sampling_technical_report/index.html)
and [PDF](../../outputs/sampling_technical_report/technical_report.pdf).

## Explore every recorded pose

Open the [interactive pose explorer](../../outputs/random_pose_viewer/index.html)
in a browser, or open the repository's `index.html` shortcut. The page works
offline: meshes, measurements and the renderer are embedded, with no server or
running Isaac session required. It contains all **600 sampled proposals** for
dinner plates, bowls and mugs on both racks: **176 accepted final placements**,
including **168 strictly contained** placements. A continuous space of possible
positions and rotations cannot be exhausted by these samples.

Filter by object, rack, outcome or trial ID, then select a gallery card to orbit
and zoom its 3-D scene. **Final** shows the last complete measured scene after a
rack-closure attempt; **Settled** shows the pre-retraction measurement; **Sampled**
shows the proposed world pose with the selected rack open. Missing stages remain
empty; an earlier stage appears only after explicit selection. Proposals rejected
by the initial collision check were not released into physics and appear as red
ghosts labeled “Never dropped” in the Sampled view. Numerical failures
remain labeled **Unresolved**; their displayed geometry is not a validated placement. Acceptance
and strict containment describe the trial's final placement, not its starting
pose. Playback advances through independent trials, not one object's trajectory.

The inspector shows world position in metres, XYZ Euler angles and the saved
XYZW quaternion. Download one pose or the filtered set as JSON, including stage
labels and original sampled rack-local coordinates. The displayed meshes are
reconstructed from the matching source geometry at saved physics transforms;
they are not new simulation results. Only the selected rack's exact reset
transform was saved, so the sampled view omits other component meshes. The gray
enclosure outline is the nominal containment boundary. For a failed trial whose
standalone endpoint was saved after the last complete scene snapshot, the export
preserves that later endpoint separately.

For example, `bowl_LowerRack_00010` is an `initial_collision` rejection. Its
settled and final poses are null, and its trace contains only ten empty-rack
reset observations. It was never dropped or accepted. The viewer previously
substituted its sampled proposal when Final was unavailable; that fallback has
been removed. A rejection does not constitute an example of a feasible placement.

Rebuild the data and standalone page without changing the finalized run:

```bash
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_random_pose_viewer_data.py \
  --run-dir results/random_poses/frigidaire/complete_20260910_seed0
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_random_pose_viewer.py
```

The builder uses the existing Three.js 0.160.1 files and MIT license under
`outputs/random_pose_viewer/vendor/`. Derived data and HTML stay under
`outputs/random_pose_viewer/`; the finalized reports and experiment archive
remain unchanged.

## Run

Use the existing Isaac Sim 4.5 / Isaac Lab container from the repository root:

```bash
scripts/run_kit.sh frigidaire/scripts/experiment/frigidaire_random_pose_experiment.py \
  --headless --device cpu --samples-per-cell 100 --seed 0 --max-wall-seconds 1800
```

The seed-0 pilot completed on 2026-09-10 used this explicit output directory:

```bash
scripts/run_kit.sh frigidaire/scripts/experiment/frigidaire_random_pose_experiment.py \
  --headless --device cpu --samples-per-cell 100 --seed 0 --max-wall-seconds 1800 \
  --out-dir results/random_poses/frigidaire/pilot_20260910_seed0
```

That directory contains the completed pilot; use a new empty directory for a
repeat. The pilot targeted 600 proposals and reached its 30-minute budget after
261 attempts. During any active run, `summary.json` is a checkpoint and does not
establish a final result.

The default USD is `build/frigidaire_collection/usd/fdpc4221as.usdc`. Output goes
to a new timestamped directory in `build/frigidaire_random_poses/`; an explicit
`--out-dir` must be empty. `--usd` selects a portable copy of the same current
asset collection. A different geometry revision fails validation instead of
silently reusing incompatible sampling bounds.

For a short runtime smoke check, use `--samples-per-cell 1`. This checks the
pipeline, not whether every dish/rack combination has an accepted pose. Use one
Kit process at a time. The default CPU physics matches the existing Frigidaire
evidence; Kit still needs the working NVIDIA runtime.

Host-only validation is available without Isaac, FCL, or SciPy:

```bash
python3 frigidaire/scripts/experiment/frigidaire_random_pose_experiment.py --preflight-only
```

Preflight validates the current asset hashes, geometry parameters, tableware
catalog and source-derived sampling domains. Its result explicitly says
`[RESULT] NOT_RUN`; it provides no evidence of physical closure.

## Continue a run after its time limit

`--resume-from` continues the original planned proposals in a **new, empty output
directory**. It preserves the prior run and carries forward its classified
records and traces. Keep the original seed, number of proposals per cell, assets
and physics settings:

```bash
scripts/run_kit.sh frigidaire/scripts/experiment/frigidaire_random_pose_experiment.py \
  --headless --device cpu --samples-per-cell 100 --seed 0 \
  --max-wall-seconds 3600 \
  --resume-from results/random_poses/frigidaire/pilot_20260910_seed0 \
  --out-dir results/random_poses/frigidaire/complete_20260910_seed0
```

This command started on **2026-09-10 at 20:56:22 UTC** and completed the original
600-proposal scan within its fresh 60-minute limit. The continuation took
**2689.675 seconds (44.8 minutes)**; cumulative wall time, including the prior
pilot, was **4489.689 seconds (74.8 minutes)**. The completed results are below.
The destination above is occupied; any later continuation must use another new
directory.

Before reusing evidence, the continuation checks prior counts, replay records
and traces; matching seed, sample domains and acceptance thresholds; exact asset
and physics-source hashes; and identical recorded runtime versions and settings.
The physics sources include the sampler, runtime, collision helpers, geometry,
tableware, validation and quaternion conversions. Changes to the command-line,
scheduling and reporting code are recorded separately and may be allowed.
Both empty-rack baselines run again in a fresh simulator. The interrupted
physical trajectory is not resumed.

Every prior sampled position, rotation and bounding-box center is regenerated
from its original per-cell random stream and compared with the saved values
at a component tolerance of `1e-12`. This includes the interrupted proposal.
The scheduler consumes the random draws for inherited records before skipping
their evaluations, so subsequent proposals retain their original seeded poses.

For this seed-0 continuation, the provenance audit **PASS** confirms
**260 unchanged inherited records** and **340 new evaluations**. The interrupted
`bowl_LowerRack_00043` was retried at its original sampled pose and accepted;
the 339 previously unattempted proposals were also evaluated. The combined
result covers the original **600 logical proposals**, with **601 cumulative
evaluation attempts** including the archived interrupted attempt.

Prior metadata, trial records, replay records and traces are copied with hashes
under `resume_history/pilot_20260910_seed0/` in the new output. The old unresolved
record is also retained in `unresolved_trials.jsonl` there. Its old trace stays
in history, allowing the retry to write a fresh trace under the same trial ID.
The main report counts each logical proposal once. Metadata separately records
inherited classifications, new evaluations, cumulative evaluation attempts and
both continuation and cumulative wall time. Do not add the prior pilot's counts
to the combined report a second time.

## Sampling and acceptance

Each of the six dish/rack combinations has its own reproducible random stream.
Trials are interleaved so a wall-time cutoff does not omit all later combinations.
The target of 100 per combination means **100 raw proposals**, including initial
collisions, rather than repeatedly sampling until 100 successes are found.

The sampler draws a uniform rotation and a uniform rotated-mesh bounding-box
center. Center X/Y bounds follow the selected rack's rim. Z extends from its
lowest generated floor-wire top to the upper rack's underside for lower-rack
trials, or the tub ceiling for upper-rack trials. These are recorded proposal
domains, not a claim to cover all feasible poses. The actor pose compensates
for the bowl's base origin and the mug's asymmetric handle. No slot patterns,
catalog rack recommendations, or uprightness requirements restrict proposals.

Before dish trials, each empty rack must physically extend and retract while
the basket remains seated. Every trial then resets the articulation, basket,
velocities and contact history to that rack's measured baseline. FCL checks
actual measured component transforms, including the extended rack and rotated
door. Initially intersecting poses are recorded without releasing the dish.

A collision-free dish has up to 12 simulated seconds to reach rest. The test
then retracts the rack using the existing drive forces and smooth motion capped
at 0.10 m/s, followed by another rest window. Physics runs at 120 Hz with CCD.
The first full one-second window satisfying the gates completes each hold.

A confirmed success requires:

- Rack endpoint error at most 5 mm, with the other rack retracted and door open.
- Dish support from its selected rack, or from the seated basket on the lower rack.
- Every dish mesh vertex inside the authored tub envelope, allowing 1 mm numerical
  tolerance. This includes handles and allows overhang beyond the rack rim.
- Rest-window position span below 5 mm, orientation span below 3°, and measured
  mesh-point speed below 0.03 m/s. Large changes from the original sampled pose are allowed.
- Valid contact evidence: peak penetration below 2 mm and median maximum penetration
  below 1 mm in the rest windows; the 2 mm peak limit also applies throughout
  retraction and its final hold. Initial impact transients are recorded separately
  from the closure check. The same closure validity rule applies to the empty baseline.

Sleeping contacts remain evidence only while both contacting surfaces have moved
at most one micrometre since the last observation. Basket stability is a scene
validity gate even for upper-rack trials. Numerical/contact failures are reported
as unresolved, not proof that a placement is impossible.

## Read the results

After the run finishes, run the following reporting pipeline. Replace
`YOUR_FINALIZED_RUN` with its output directory. These commands read the saved
physics evidence and add derived reports; they do not start Kit or rerun trials.
Keep the published prior pilot directory unchanged.

```bash
RANDOM_POSE_RUN_DIR=results/random_poses/frigidaire/YOUR_FINALIZED_RUN

scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_random_pose_report.py \
  --run-dir "$RANDOM_POSE_RUN_DIR"

scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_random_pose_geometry_audit.py \
  --run-dir "$RANDOM_POSE_RUN_DIR"

scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_random_pose_views.py \
  --run-dir "$RANDOM_POSE_RUN_DIR" \
  --out "$RANDOM_POSE_RUN_DIR/accepted_pose_examples.png"

scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_random_pose_package.py \
  --run-dir "$RANDOM_POSE_RUN_DIR"
```

The first command writes `analysis.json`, `analysis.md` and
`outcome_breakdown.png` into that run's directory.
The analyzer rejects an unfinished run or inconsistent summary/replay counts.
For accepted trials it checks recorded rest, support, endpoint, whole-mesh
containment and penetration gates, together with the final-hold trace samples.
Missing evidence produces an incomplete audit, rather than a pass. It reports
final-sample endpoint error separately from the maximum sampled trace-window
error. The analysis is an audit of saved evidence, not another simulation.

The independent geometry audit defaults to
`RUN_DIR/independent_geometry_audit.json`. It regenerates the complete dish
visual meshes, compares their final bounds with the saved bounds, and checks
containment in both world and measured Cabinet coordinates. This checks final
mesh geometry; it does not replay physics or independently audit collisions.

Packaging requires a finalized physical run, passing saved-evidence and geometry
audits for every accepted trial, matching source hashes, and the illustration's
PNG and provenance JSON. It writes a standalone `experiment_report.md`,
`strictly_contained_poses.json`, a verified `source_snapshot/`, copies of the
available reporting scripts and documentation under `reporting_tools/`, and
`checksums.sha256`. It then creates `RUN_DIR.zip` alongside the run directory,
verifying the archive contents against the checksums. Matching appliance assets
and the original Isaac runtime are still required to replay the experiment;
the package does not duplicate those assets.

The strictly contained subset includes only accepted poses whose regenerated
meshes lie inside the nominal envelope with zero tolerance in both coordinate
frames. This additional filter preserves the original verdicts and the complete
accepted replay file. A completed 600-proposal scan may be packaged with
numerically unresolved rows, which remain explicitly reported and excluded from
accepted subsets. A passing accepted-pose audit does not resolve those rows or
establish that every proposal has a physical answer.

The analysis distinguishes acceptance per raw proposal from acceptance among
proposals that entered dish physics. Initial collisions belong only in the raw
denominator; interrupted/error trials need a saved physics measurement or an
active-dish trace to establish entry. Unknown entry stages remain explicit.
The stacked figure includes every planned proposal, including untested ones.

`report.md` and `summary.csv` compare the six combinations. `summary.json` also
contains empty-rack baselines and completion status. The confirmed rate is
accepted/raw-attempted, with unresolved and unattempted counts shown separately.
A settling timeout means the configured time window was insufficient; it is
not a claim that the pose could never settle.

`trials.jsonl` is flushed and synced after each trial, with an atomic summary
checkpoint. It contains requested, settled and final poses, contact and motion
metrics, joint observations, and failure reasons. `accepted_poses.json` preserves
complete successful records for replay against the matching asset hashes.
`traces/` records measured rack, basket and dish trajectories at 10 Hz; contact
and validity checks still run at the full physics rate. Position/orientation
plots summarize sampled starting poses, colored by outcome (`--no-plots` disables them).

Coordinates use metres and XYZW quaternions. Sampled poses are rack-local;
measured poses are in world coordinates. `metadata.json` records input/source
hashes, sample domains, thresholds, seed and runtime settings.

The static 3D illustration shows the first accepted trial in each
dish/rack combination, with its exact trial ID and accepted/attempted counts.
Cells without accepted samples are labeled explicitly. It is required for the
complete package above, or can be generated separately as an optional view:

```bash
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_random_pose_views.py \
  --run-dir "$RANDOM_POSE_RUN_DIR" \
  --out /tmp/frigidaire_accepted_pose_examples.png
```

Run this after the experiment finishes. The script verifies the run's asset and
visual-source hashes, reconstructs generated dish meshes (including mug handles)
and racks at their saved final world poses, and writes the PNG plus a same-stem
JSON containing selected trial IDs, poses and input hashes. The shell and other
rack are omitted for visibility. This is a graphical reconstruction of saved
Isaac observations, not a new simulation or Isaac render; it changes no trial
verdicts. NumPy, SciPy and Matplotlib run through the existing Kit-free launcher.

The default 30-minute budget (or explicit `--max-wall-seconds` limit) starts
before application startup and is checked during initialization and every
simulation step. A continuation receives its own fresh limit. Startup/native operations and report
finalization can finish after the deadline; no new trial starts after it. An
interrupted active trial is unresolved. A failed empty-rack baseline blocks the
three affected cells without counting invented dish failures. Check the saved
status, `[RESULT]` marker and exceptions; the Isaac wrapper can exit zero after
a failure. A completed scan with numerical errors remains incomplete evidence.

Continuous poses have no finite total count. Several accepted drops can converge
to the same resting arrangement. These results measure this model and sampling
protocol, not unique configurations, real-appliance capacity, multi-dish loading,
or door closability.

## Tests and current execution status

```bash
PYTHONPATH=src:frigidaire/src python3 -m unittest discover \
  -s frigidaire/tests -p '*random_pose*.py' -v
```

The host suite checks sampling, whole-mesh containment, live collider transforms,
input integrity, support evidence, trial decisions through a simulated backend,
interruption accounting, durable reports, and the saved-evidence analyzer.
Host tests alone do not validate physical behavior.

Docker/NVIDIA access is working. The seed-0 scan completed on 2026-09-10 with
**600 distinct planned sample IDs, 100 per dish/rack combination**. It recorded
**176 accepted drops out of 600 raw proposals (29.3%)**, with **two numerically
unresolved trials and none untested**. The saved `complete` status means all
planned proposals were evaluated. The `[RESULT] INCOMPLETE` marker preserves
the distinction that two numerical outcomes remain unresolved.

| Dish | Rack | Accepted / attempted | Raw acceptance | Strictly contained | Unresolved |
| --- | --- | ---: | ---: | ---: | ---: |
| Dinner plate | Lower | 15 / 100 | 15% | 14 | 0 |
| Dinner plate | Upper | 10 / 100 | 10% | 10 | 0 |
| Bowl | Lower | 44 / 100 | 44% | 42 | 1 |
| Bowl | Upper | 32 / 100 | 32% | 32 | 1 |
| Mug | Lower | 46 / 100 | 46% | 41 | 0 |
| Mug | Upper | 29 / 100 | 29% | 29 | 0 |

The other outcomes were **395 initial collisions, six closure failures, six
losses of support, 11 dishes outside the envelope, four settling timeouts and
two numerical failures**. The unresolved samples were `bowl_UpperRack_00055`
and `bowl_LowerRack_00097`, with settled-window penetration peaks of approximately
6.1069 mm and 8.1731 mm, respectively, above the 2 mm gate. Rack retraction had
not started for either. These outcomes do not establish physical infeasibility
or rack-closure failure.

The saved physics-evidence and independent geometry audits both **PASS all 176
accepted trials**. **168 accepted poses meet zero-tolerance containment** in
both world and measured Cabinet coordinates; eight rely on the configured 1 mm
numerical tolerance. These accepted-only audit results do not resolve the two
numerical failures. The continuation provenance audit also **PASS** verifies the
260 unchanged inherited records and exact seeded retry. The scan required 340
new evaluations and 601 cumulative attempts, with 44.8 minutes of continuation
wall time and 74.8 minutes cumulative wall time.

Completed artifacts: [experiment report](../../results/random_poses/frigidaire/complete_20260910_seed0/experiment_report.md),
[accepted-pose examples](../../results/random_poses/frigidaire/complete_20260910_seed0/accepted_pose_examples.png),
[176 accepted replay records](../../results/random_poses/frigidaire/complete_20260910_seed0/accepted_poses.json),
and [168 strictly contained replay records](../../results/random_poses/frigidaire/complete_20260910_seed0/strictly_contained_poses.json).
The [summary](../../results/random_poses/frigidaire/complete_20260910_seed0/summary.json),
[saved-evidence analysis](../../results/random_poses/frigidaire/complete_20260910_seed0/analysis.md)
and [geometry audit](../../results/random_poses/frigidaire/complete_20260910_seed0/independent_geometry_audit.json)
preserve the detailed accounting and validation results. All rates describe
sampled successful drops rather than unique arrangements or all possible poses.

## Prior pilot and smoke history

The following prior results remain unchanged. The completed scan inherits the
pilot's classified records; do not add these counts to the completed results.

The seed-0 pilot completed on 2026-09-10 with
status `budget_exhausted` after **1800.014 seconds** of wall time. It recorded
**82 accepted drops out of 261 raw proposals (31.4%)**. Of 600 planned proposals,
**339 remained untested**; one attempted lower-rack bowl trial was interrupted
and remains unresolved. There were 260 classified proposals and no simulation
errors. These are sampled successful drops, not unique arrangements or a global
count of possible poses.

| Dish | Rack | Accepted / attempted | Raw acceptance | Untested | Unresolved |
| --- | --- | ---: | ---: | ---: | ---: |
| Dinner plate | Lower | 5 / 44 | 11.4% | 56 | 0 |
| Dinner plate | Upper | 5 / 44 | 11.4% | 56 | 0 |
| Bowl | Lower | 21 / 44 | 47.7% | 56 | 1 |
| Bowl | Upper | 14 / 43 | 32.6% | 57 | 0 |
| Mug | Lower | 24 / 43 | 55.8% | 57 | 0 |
| Mug | Upper | 13 / 43 | 30.2% | 57 | 0 |

The other outcomes were 166 initial collisions, three closure failures, three
losses of support, five dishes outside the interior envelope, one settling
timeout, and the interrupted trial. Untested and unresolved proposals are not
evidence of impossible placements.

The saved-evidence audit and independent final-geometry audit both **PASS all 82
accepted trials**. Regenerated mesh bounds agree with the saved bounds within
3.56 nanometres, including mug handles. **77 of 82** accepted dishes lie inside
the nominal envelope with zero containment margin; **five rely on the configured
1 mm tolerance**, with a maximum extension of 0.8315 mm beyond the nominal
envelope. All 82 satisfy that tolerance in both world and measured Cabinet
coordinates. The audits leave the experiment's raw records and thresholds unchanged.

Pilot artifacts: [experiment report](../../results/random_poses/frigidaire/pilot_20260910_seed0/experiment_report.md),
[accepted-pose examples](../../results/random_poses/frigidaire/pilot_20260910_seed0/accepted_pose_examples.png),
and [82 replayable accepted records](../../results/random_poses/frigidaire/pilot_20260910_seed0/accepted_poses.json).
The finalized [summary](../../results/random_poses/frigidaire/pilot_20260910_seed0/summary.json)
contains all counters and baseline observations. The
[detailed saved-evidence analysis](../../results/random_poses/frigidaire/pilot_20260910_seed0/analysis.md)
and [independent geometry audit](../../results/random_poses/frigidaire/pilot_20260910_seed0/independent_geometry_audit.json)
record their checks separately from the simulation's trial verdicts.

A separate real Isaac smoke run completed on 2026-09-10
at `results/random_poses/frigidaire/smoke_20260910_01/`: **3 accepted out of 6 raw
proposals**, with one proposal per dish/rack cell. Both empty-rack baselines
passed. The accepted drops were the bowl on each rack and the mug on the lower
rack; the other three proposals had initial collisions. There were no unresolved
trials. The run took 122.4 seconds of wall time, using CPU physics at 120 Hz with
CCD. Its metadata records Isaac Sim `4.5.0-rc.36+release.19112.f59b3005.gl` and
installed Isaac Lab package version `0.41.3`.

The smoke run validates the execution/reporting pipeline and supplies three
measured successful drops; six proposals are insufficient to characterize each
cell's acceptance rate. Its saved accepted evidence also passes the analysis
audit. **Do not combine smoke and pilot counts**: both use seed 0, so their first
six sampled poses are identical. The pilot counts above exclude the smoke run.
