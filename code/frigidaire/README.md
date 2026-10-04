# Frigidaire source

This folder contains the FDPC4221AS generator, loader, tests and documentation;
its build, evaluation and experiment scripts sit in the stage folders under `code/`
(see [code/README.md](../README.md)). The finished collection belongs at
`data/assets/models/frigidaire_fdpc4221as/`; supplied references and generated images
belong with that collection.

The current source combines the tape-measured **48-tine upper rack**, **64-tine lower
rack** and **1x4 basket** (320 x 95 x 130 mm body, 220 mm handle, seated rear-right)
with the complete dishwasher (revisions `upper_tines_4x13_v5`, `lower_tines_6x12_v4`,
`basket_1x4_320x95_v4`; racks re-measured 2026-09-28 with non-uniform column and row gaps, heights measured
outside, bottom to top). See [geometry and dimensions](docs/geometry.md).
The photo-fitted build is archived as `data/build/frigidaire_collection/history/v3`, the first tape build as `history/v4`
and the 2026-09-23 outside-height build as `history/v5`;
older loading results describe older geometry and are preserved in the collection's
`history/` directories.

## Current delivery status

The organized collection is staged at
[data/build/frigidaire_collection](../../data/build/frigidaire_collection/README.md), with the
supplied references, regenerated rack diagrams, manufacturer dimension image,
and checksum-verified v1 to v5 history. Source and host checks have run.

The current USD assembly and tableware are built in the stage's `usd/` directory,
and the Isaac runtime is accessible. The v3-era single-dish random-pose experiment
(600 proposals, 176 accepted drops) was retired on 2026-09-29 with the other v3-era experiments
(see "Retired experiments" below); its numbers were measured on the v3 photo-fitted racks,
archived on 2026-09-22 under `history/v3`, and are stale for the tape-measured racks.
Assembly-wide physics/render evidence, the release archive and final installation
are separate delivery gates. The read-only package check reports the remaining
requirements explicitly; the random-pose experiment does not certify a full release.

## Source layout and API

- `src/dishsim_frigidaire/`: geometry, USD authoring/loading, optional dish-loading
  utilities, candidate data and canonical collection paths.
- `code/initialization/frigidaire/`: stage references/history, build USD and package the
  collection, rack diagrams, geometry clearance, USD inspection, assembly and claims
  evidence, the scripted/passive appliance demonstration and the Isaac gate.
- `code/planner/frigidaire/`: the HOTEC benchmark (`frigidaire_bench*.py`), HOTEC loads and
  top-5 loads, the MCTS sweep, the benchmark results page and episode videos.
- `code/execution/frigidaire/`: the arm bring-up probes, the robot episode and its report.
- `code/util/frigidaire/`: the two render helpers the planner and execution scripts import.
- `tests/`: Frigidaire tests, discovered by the repository pytest configuration.
- `docs/`: current geometry notes, collection README template and historical notes.

The earlier [four-view lower-rack polish](docs/lower_rack_polish.md) is collected
here as well: `src/dishsim_frigidaire/lower_rack_asset.py`,
`code/initialization/frigidaire/polish_lower_rack.py`, and
`code/initialization/frigidaire/lower_rack_polish_evidence.py`. Its documentation describes
the existing prototype and installation target.

The root project installs both `dishsim` and `dishsim_frigidaire`; refresh an
existing editable installation with `code/util/run_py.sh -m pip install -e . --no-deps`.
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
python3 code/initialization/frigidaire/stage_frigidaire_collection.py \
  --out-dir data/build/frigidaire_collection

code/util/run_py.sh code/initialization/frigidaire/build_frigidaire.py \
  --out-dir data/build/frigidaire_collection/usd

code/util/run_py.sh code/initialization/frigidaire/frigidaire_collection_inspect.py \
  --collection-dir data/build/frigidaire_collection

code/util/run_kit.sh code/initialization/frigidaire/frigidaire_asset_evidence.py \
  --headless --device cpu --assembly-only --physics-only \
  --usd data/build/frigidaire_collection/usd/fdpc4221as.usdc \
  --out-dir data/build/frigidaire_collection/images/assembly

code/util/run_kit.sh code/initialization/frigidaire/frigidaire_asset_evidence.py \
  --headless --device cpu --assembly-only --enable_cameras --render-only \
  --usd data/build/frigidaire_collection/usd/fdpc4221as.usdc \
  --out-dir data/build/frigidaire_collection/images/assembly

code/util/run_py.sh -m pytest code/tests code/frigidaire/tests

python3 code/initialization/frigidaire/package_frigidaire.py \
  --collection-dir data/build/frigidaire_collection --check
python3 code/initialization/frigidaire/package_frigidaire.py \
  --collection-dir data/build/frigidaire_collection \
  --output data/build/frigidaire_fdpc4221as.zip
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

## Retired experiments (2026-09-29)

The v3-era experiments (single-dish random poses, multi-dish packing, organized counterparts, exposure
demos, the pool-search planner) ran on the v3 photo-fitted racks and were retired on 2026-09-29: their
scripts, tests and protocol docs are in git history (last present at commit 4455813) and their result
folders wait in `dishsim/_trash_20260929/` until 2026-10-29 (`MANIFEST.txt` inside; undo = `mv` back).
Their headline numbers (600 random-pose proposals with 176 accepted, packing loads up to 35 dishes,
7 of 11 organized inventories) are history and were never regenerated on the tape-measured racks.
Modules that keep v3-sounding names are live: `random_pose_{experiment,runtime,assets}.py`,
`initial_state_runtime.py`, `initial_state_candidates.py` (collision checker only), `organization.py`.

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

`src/dishsim_frigidaire/exposure.py` scores arrangements of the same objects by ray-cast exposure of
food-contact surfaces to the spray arms (Kit-free, Warp on the CPU); the HOTEC benchmark and the
top-5 loads use it. The demo/search/summary scripts and their v3-rack results were retired
2026-09-29. Reference: [docs/exposure.md](docs/exposure.md); quickstart:
[docs/exposure_quickstart.md](docs/exposure_quickstart.md). Live results (pre-2026-09-28 racks, history):
`data/results/exposure/frigidaire/{hotec,hotec_heightfix}`.

Planner helpers used by the HOTEC benchmark (FCL world, support gate, sequencer): [docs/planner.md](docs/planner.md);
the pool-search planner was retired 2026-09-29. Benchmark: [docs/hotec_bench.md](docs/hotec_bench.md);
geometry: [docs/geometry.md](docs/geometry.md); robot bring-up (in progress): [docs/robot.md](docs/robot.md).
