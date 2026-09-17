# Frigidaire source

This folder contains the FDPC4221AS generator, loader, build/evaluation tools,
tests and documentation. The finished collection belongs at
`assets/models/frigidaire_fdpc4221as/`; supplied references and generated images
belong with that collection.

The current source combines the polished **52-tine upper rack**, **72-tine lower
rack**, and basket shifted **27.5 mm right** with the complete dishwasher. See
[geometry and dimensions](docs/geometry.md). Older loading results describe older
geometry and are preserved in the collection's `history/` directories.

## Current delivery status

The organized collection is staged at
[build/frigidaire_collection](../build/frigidaire_collection/README.md), with the
supplied references, regenerated rack diagrams, manufacturer dimension image,
and checksum-verified v1/v2 history. Source and host checks have run.

The current USD assembly and tableware are built in the stage's `usd/` directory,
and the Isaac runtime is accessible. The single-dish random-pose experiment
completed all **600 proposals**, with **176 accepted drops (29.3%)**, two
numerically unresolved trials and none untested. The saved physics-evidence and
geometry audits pass all 176 accepted records; 168 meet zero-tolerance containment.
See the [experiment report](../results/random_poses/frigidaire/complete_20260910_seed0/experiment_report.md)
and [protocol and results](docs/random_pose_experiment.md).
Assembly-wide physics/render evidence, the release archive and final installation
are separate delivery gates. The read-only package check reports the remaining
requirements explicitly; the random-pose experiment does not certify a full release.

## Source layout and API

- `src/dishsim_frigidaire/`: geometry, USD authoring/loading, optional dish-loading
  utilities, candidate data and canonical collection paths.
- `scripts/setup/`: stage references/history, build USD and package the collection.
- `scripts/evaluation/`: rack diagrams, geometry clearance, USD inspection,
  assembly evidence and retained loading diagnostics.
- `scripts/experiment/`: scripted/passive appliance demonstration.
- `tests/`: Frigidaire tests, discovered by the repository pytest configuration.
- `docs/`: current geometry notes, collection README template and historical notes.

The earlier [four-view lower-rack polish](docs/lower_rack_polish.md) is collected
here as well: `src/dishsim_frigidaire/lower_rack_asset.py`,
`scripts/setup/polish_lower_rack.py`, and
`scripts/evaluation/lower_rack_polish_evidence.py`. Its documentation describes
the existing prototype and installation target.

The root project installs both `dishsim` and `dishsim_frigidaire`; refresh an
existing editable installation with `scripts/run_py.sh -m pip install -e . --no-deps`.
The package keeps USD and Kit imports out of module initialization.

```python
from dishsim_frigidaire.asset import build, spawn, apply_mode, step_passive
from dishsim_frigidaire.paths import ASSET_DIR

# Inside an initialized Isaac application, before resetting the simulation:
appliance, basket = spawn("/World/Dishwasher", mode="scripted")
```

`ASSET_DIR` is the collection's `usd/` directory. `build(output_dir=..., component=...)`
and `spawn(..., usd_path=...)` retain their existing interfaces. The old
`dishsim.frigidaire_*` imports and scattered script locations have been replaced.
Root launchers and shared `dishsim.media` utilities remain shared with the repo.

## Reproduce and validate

Run these from the repository root. Host staging needs NumPy, Matplotlib, Pillow
and Poppler (`pdftoppm`); USD and simulation commands use the existing pinned
Isaac Sim 4.5 / Isaac Lab container. Each command can use an explicit staging
path. Keep one Kit job running at a time.

```bash
python3 frigidaire/scripts/setup/stage_frigidaire_collection.py \
  --out-dir build/frigidaire_collection

scripts/run_py.sh frigidaire/scripts/setup/build_frigidaire.py \
  --out-dir build/frigidaire_collection/usd

scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_collection_inspect.py \
  --collection-dir build/frigidaire_collection

scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_asset_evidence.py \
  --headless --device cpu --assembly-only --physics-only \
  --usd build/frigidaire_collection/usd/fdpc4221as.usdc \
  --out-dir build/frigidaire_collection/images/assembly

scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_asset_evidence.py \
  --headless --device cpu --assembly-only --enable_cameras --render-only \
  --usd build/frigidaire_collection/usd/fdpc4221as.usdc \
  --out-dir build/frigidaire_collection/images/assembly

scripts/run_py.sh -m pytest tests frigidaire/tests

python3 frigidaire/scripts/setup/package_frigidaire.py \
  --collection-dir build/frigidaire_collection --check
python3 frigidaire/scripts/setup/package_frigidaire.py \
  --collection-dir build/frigidaire_collection \
  --output build/frigidaire_fdpc4221as.zip
```

Staging copies inputs without changing historical bytes or deleting their
originals. It accepts `--reference-dir` for a separately supplied reference set;
otherwise it reuses the installed or staged `references/` directory. PNG/SVG
layouts and measurements are regenerated together. A full build writes both
polished racks at once, along with the cabinet, door, basket and empty example.
Loose fixtures/tableware are optional prototypes, with no accepted placements.

The package check requires matching current geometry, composition, physics,
render and contact reports, rack images, dimension provenance, references and
history. It does not accept old load evidence as current validation. Check the
`[RESULT]` lines and JSON reports: the Isaac wrapper can exit zero after a Python
error. All report gates must pass before publishing the collection.

After verification, install the staged collection at the canonical asset
destination on the external drive. Preserve the verified historical copies
before removing the old sibling releases/galleries. Do not merge an old bundle's
root into the new `usd/` directory; its `full_load*` files and capacity reports
belong in `history/`. The single asset archive contains images and references;
there is no separate current gallery product.

## Random dish pose experiment

The [random dish pose experiment](docs/random_pose_experiment.md) independently
samples a dinner plate, bowl and mug in both racks, allows settling, and measures
physical rack retraction. On 2026-09-10 the seed-0 scan completed **100 raw
proposals per combination, 600 total**, after continuing the 30-minute pilot.
It recorded **176 accepted drops (29.3%)**, two numerically unresolved bowl
trials and no untested proposals. Accepted lower/upper counts were 15/10 for
plates, 44/32 for bowls and 46/29 for mugs. These are sampled drops, not distinct
arrangements or a count of all possible poses.

Read the [experiment report](../results/random_poses/frigidaire/complete_20260910_seed0/experiment_report.md),
view [accepted-pose examples](../results/random_poses/frigidaire/complete_20260910_seed0/accepted_pose_examples.png),
or use the [accepted replay records](../results/random_poses/frigidaire/complete_20260910_seed0/accepted_poses.json).
The [saved-evidence analysis](../results/random_poses/frigidaire/complete_20260910_seed0/analysis.md)
and [geometry audit](../results/random_poses/frigidaire/complete_20260910_seed0/independent_geometry_audit.json)
both pass all 176 accepted records. Of these, **168 meet strict containment** in
both world and measured Cabinet coordinates; eight rely on the protocol's 1 mm
numerical tolerance. The two unresolved bowls had excessive settled contact
penetration before retraction began; they do not establish physical infeasibility.
Saved status `complete` means the scan finished, while `[RESULT] INCOMPLETE`
retains the unresolved numerical evidence.

The continuation took 44.8 minutes, with 74.8 minutes cumulative wall time. Its
provenance audit verifies 260 unchanged inherited records and 340 new evaluations.
There were 601 cumulative evaluation attempts because the pilot's interrupted
proposal was retried at its original pose and accepted. The
[prior pilot report](../results/random_poses/frigidaire/pilot_20260910_seed0/experiment_report.md)
remains unchanged as history: 82 accepted out of 261 proposals, 339 untested and
one interrupted. The separate smoke accepted 3/6 proposals. Neither prior result
should be added to the completed scan's counts.

The [experiment documentation](docs/random_pose_experiment.md#read-the-results)
contains the complete finalized-run reporting, geometry-audit, visualization and
verified packaging pipeline, along with launch commands, acceptance gates,
runtime versions and replay conventions.

## Tests without Isaac

Host preflight, experiment tests and saved-evidence analysis do not start Isaac.

The component update, upper/lower loading-pattern, lower clearance, assembly
evidence contract, packaging and staging unittest files can run with host NumPy
and Pillow. USD-dependent component tests skip when the runtime is absent.
The complete pytest suite and actual physics/render checks require the existing
runtime; host contract tests do not substitute for them.

Original development documentation is preserved in
[docs/history/development_notes.md](docs/history/development_notes.md). Its old
paths and loading certifications are historical context.

## Exposure scorer

`src/dishsim_frigidaire/exposure.py` and `scripts/evaluation/frigidaire_exposure_*.py` score
arrangements of the same objects by ray-cast exposure of food-contact surfaces to the spray
arms (Kit-free, Warp on the CPU). Reference: [docs/exposure.md](docs/exposure.md); quickstart:
[docs/exposure_quickstart.md](docs/exposure_quickstart.md). Results under
`results/exposure/frigidaire/`.
