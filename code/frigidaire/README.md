# Frigidaire source

This folder contains the FDPC4221AS loader, tests and documentation; its evaluation and
experiment scripts sit in the stage folders under `code/` (see [code/README.md](../README.md)).
Since 2026-10-21 the twin ships in the HF asset dataset `shu4dev/dishsim-assets` (`data/assets` is a
copy of it: `hf download` to restore, `hf upload` to publish), as `data/assets/frigidaire_fdpc4221as/` (`usd/`, `parameters.json`, `components.json`), with the
generator and the build, staging, packaging, preview and evidence scripts and their tests in its
`source/`; the ten tableware prototypes ship in `data/assets/frigidaire_prototypes/`. Generated
images and validation reports stay out of the dataset, in
`data/media/frigidaire_fdpc4221as/{images,validation}`.

The current source combines the tape-measured **48-tine upper rack**, **64-tine lower
rack** and **1x4 basket** (320 x 95 x 130 mm body, 220 mm handle, seated rear-right)
with the complete dishwasher (revisions `upper_tines_4x13_v5`, `lower_tines_6x12_v4`,
`basket_1x4_320x95_v4`; racks re-measured 2026-09-28 with non-uniform column and row gaps, heights measured
outside, bottom to top). See [geometry and dimensions](docs/geometry.md).
The photo-fitted build was archived as `history/v3`, the first tape build as `history/v4`
and the 2026-09-23 outside-height build as `history/v5`, under the staging folder
`data/build/frigidaire_collection/` (corallab only, never in the HF archive; not kept in the dataset since
2026-10-21, their code is in this repo's git history); older loading results describe older geometry.

## Current delivery status

Since 2026-10-21 the current USD assembly ships in `data/assets/frigidaire_fdpc4221as/usd/` and
the tableware prototypes in `data/assets/frigidaire_prototypes/usd/`; no code reads the old
staging folder any more (`dishsim.legacy_data_path` maps paths recorded before the move to the
new folders). The v3-era single-dish random-pose experiment
(600 proposals, 176 accepted drops) was retired on 2026-09-29 with the other v3-era experiments
(see "Retired experiments" below); its numbers were measured on the v3 photo-fitted racks,
archived on 2026-09-22 under `history/v3`, and are stale for the tape-measured racks.
Assembly-wide physics/render evidence, the release archive and final installation
are separate delivery gates. The read-only package check (in the asset folder's `source/`)
reports the remaining requirements explicitly; the random-pose experiment does not certify a
full release.

## Source layout and API

- `src/dishsim_frigidaire/`: geometry as the experiments read it (`parameters.json`,
  `components.json`, tine helpers), USD loading, optional dish-loading utilities, candidate
  data and canonical asset paths (`paths.py`).
- `code/initialization/frigidaire/`: the scripted/passive appliance demonstration, the Isaac
  gate and the random-load farm. Staging, USD build, packaging, rack diagrams, geometry
  clearance, USD inspection and the assembly and claims evidence moved to
  `data/assets/frigidaire_fdpc4221as/source/` on 2026-10-21.
- `code/planner/frigidaire/`: the HOTEC benchmark (`frigidaire_bench*.py`), HOTEC loads and
  top-5 loads, the MCTS sweep, the benchmark results page and episode videos.
- `code/execution/frigidaire/`: the arm bring-up probes, the robot episode and its report.
- `code/util/frigidaire/`: the two render helpers the planner and execution scripts import.
- `tests/`: Frigidaire tests, discovered by the repository pytest configuration.
- `docs/`: current geometry notes and historical notes (the collection README template
  moved to `data/assets/frigidaire_fdpc4221as/source/`).

The earlier four-view lower-rack polish (of the standalone Bosch 800 asset) moved with its
module, both scripts and `lower_rack_polish.md` to `data/assets/bosch800/source/` on
2026-10-21. Its documentation describes the existing prototype and installation target.

The root project installs both `dishsim` and `dishsim_frigidaire`; refresh an
existing editable installation with `code/util/run_py.sh -m pip install -e . --no-deps`.
The package keeps USD and Kit imports out of module initialization.

```python
from dishsim_frigidaire.asset import spawn, apply_mode, step_passive
from dishsim_frigidaire.paths import ASSET_DIR

# Inside an initialized Isaac application, before resetting the simulation:
appliance, basket = spawn("/World/Dishwasher", mode="scripted")
```

`ASSET_DIR` is `data/assets/frigidaire_fdpc4221as/usd`. `spawn(..., usd_path=...)` retains its
existing interface; `build(output_dir=..., component=...)` moved with the USD authoring to
`data/assets/frigidaire_fdpc4221as/source/asset.py` on 2026-10-21. The old
`dishsim.frigidaire_*` imports and scattered script locations have been replaced.
Root launchers and shared `dishsim.media` utilities remain shared with the repo.

## Reproduce and validate

The staging, build, inspection, evidence and packaging scripts live in
`data/assets/frigidaire_fdpc4221as/source/` since 2026-10-21; its README gives each exact
command and the git SHA of its last run. Host staging needs NumPy, Matplotlib, Pillow
and Poppler (`pdftoppm`); USD and simulation commands use the existing pinned
Isaac Sim 4.5 / Isaac Lab container. Keep one Kit job running at a time. Images and
validation reports go to `data/media/frigidaire_fdpc4221as/` (`paths.IMAGE_DIR`,
`paths.VALIDATION_DIR`). The tests left in the repo run from the repository root:

```bash
code/util/run_py.sh -m pytest code/tests code/frigidaire/tests
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

After verification, publish the rebuilt twin (since 2026-10-21; needs `hf auth login` with write
access; the dataset's history on the hub keeps the previous build):

```bash
hf upload shu4dev/dishsim-assets data/assets . --repo-type dataset --commit-message "<what changed>"
```

Do not merge an old bundle's root into the new `usd/` directory; there is no separate current
gallery product.

## Retired experiments (2026-09-29)

The v3-era experiments (single-dish random poses, multi-dish packing, organized counterparts, exposure
demos, the pool-search planner) ran on the v3 photo-fitted racks and were retired on 2026-09-29: their
scripts, tests and protocol docs are in git history (last present at commit 4455813) and their result
folders wait in `dishsim/_trash_20260929/` until 2026-10-29 (`MANIFEST.txt` inside; undo = `mv` back).
Their headline numbers (600 random-pose proposals with 176 accepted, packing loads up to 35 dishes,
7 of 11 organized inventories) are history and were never regenerated on the tape-measured racks.
Modules that keep v3-sounding names are live: `random_pose_{experiment,runtime}.py`,
`initial_state_runtime.py`, `initial_state_candidates.py` (collision checker only), `organization.py`.
(`random_pose_assets.py` split on 2026-10-21: its input validation moved to the asset folder's
`source/`, its runtime half into `random_pose_runtime.py`.)

## Tests without Isaac

Host preflight, experiment tests and saved-evidence analysis do not start Isaac.

The upper/lower loading-pattern unittest files can run with host NumPy and Pillow;
the component update, lower clearance, assembly evidence contract, packaging and staging
tests moved with their scripts to `data/assets/frigidaire_fdpc4221as/source/` on 2026-10-21.
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
