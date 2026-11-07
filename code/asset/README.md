# asset

Block 0 ASSETS of `docs/repo_map.txt` (the scene): the runtime of the Frigidaire FDPC4221AS twin and the HOTEC dish set
that every block loads. The package is `asset` (`from asset.loading import visual_points`); its submodules load on
demand, and importing the package imports no NumPy, USD, PhysX, FCL or Isaac Lab. The twin's USDs and manifests ship in
`data/assets` (a copy of the HF dataset `shu4dev/dishsim-assets`), each asset folder with its generator in `source/`.

| Path | What |
|---|---|
| `twin.py` | FDPC4221AS Isaac Lab loading (spawn, gains, passive forces) and the asset's component table (`asset.py` until 2026-11-07) |
| `geometry.py` | the geometry as the experiments read it: the asset folder's `parameters.json` and `components.json` |
| `loading.py`, `cutlery_candidates.json` | deterministic, full-size mixed loading of the twin |
| `load_validation.py` | Kit-free integrity checks at the full-load simulation boundary |
| `organization.py` | explicit organization rules for the unchanged dish geometry |
| `tableware.py` | fixed-size everyday tableware for the mixed-load experiment |
| `paths.py` | canonical locations of the asset folder and its prototypes |
| `scripts/` | the demo and the HOTEC load script (below) |
| `tests/` | Kit-free tests of the loading patterns, tableware, organization and the random-pose assets |
| `docs/geometry.md` | rack, basket and appliance geometry (tape re-measured 2026-09-28) |

## Scripts

Defaults are defined in the script itself unless another file is named. Two defaults come from `code/asset/paths.py`:
`ASSET_DIR` = `data/assets/frigidaire_fdpc4221as/usd`, `IMAGE_DIR` = `data/media/frigidaire_fdpc4221as/images`.
Kit = run through `code/util/run_kit.sh ... --headless`; Kit-free = `code/util/run_py.sh` (or host `python3`
where noted).

| Script | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `frigidaire_demo.py` | Kit. Minimal loader: ordered opening and closing, or passive handle forces. | `--mode` (`scripted`, or `passive`), `--usd` (the current collection), `--x`, `--y` (0), `--yaw` (0, degrees) | the appliance USD | the `[RESULT]` line |
| `frigidaire_hotec_load.py` | Two stages. `--layout-only` (Kit-free): FCL rack layout of plates, bowls and cups, overflow to the counter. Kit: spawn, open the door, extend the racks, teleport, settle, stills and an orbit video. | layout: `--assets` (`data/assets/hotec_wheatstraw/usd`), `--bowls-upper` (4), `--plates` (`split`, or `front`), `--search`; Kit: `--layout` (required), `--sequential`, `--settle-timeout` (12 s), `--observation` (5 s), `--usd` (`data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc`) | assets, the layout | `--out-dir`: `layout.json`; `<tag>_evidence.json`, `<tag>_step_NN.png`, stills, video |

## Frigidaire source

This folder contains the FDPC4221AS loader, tests and documentation; its evaluation and
experiment scripts sit in the block folders under `code/` (see [code/README.md](../README.md)).
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
(see "Retired experiments" in [code/sampler/README.md](../sampler/README.md)); its numbers were measured on the v3 photo-fitted racks,
archived on 2026-09-22 under `history/v3`, and are stale for the tape-measured racks.
Assembly-wide physics/render evidence, the release archive and final installation
are separate delivery gates. The read-only package check (in the asset folder's `source/`)
reports the remaining requirements explicitly; the random-pose experiment does not certify a
full release.

## Source layout and API

- `code/asset/` (this package): geometry as the experiments read it (`parameters.json`,
  `components.json`, tine helpers), USD loading, optional dish-loading utilities, candidate
  data and canonical asset paths (`paths.py`).
- `code/asset/scripts/` and `code/sampler/scripts/`: the scripted/passive appliance demonstration, the Isaac
  gate and the random-load farm. Staging, USD build, packaging, rack diagrams, geometry
  clearance, USD inspection and the assembly and claims evidence moved to
  `data/assets/frigidaire_fdpc4221as/source/` on 2026-10-21.
- The HOTEC benchmark (`code/arrangement_optimization/frigidaire_bench.py`, `code/sampler/scripts/frigidaire_bench_kit.py`),
  HOTEC loads (`code/asset/scripts/`) and top-5 loads (`code/sampler/scorer/scripts/`), the MCTS sweep
  (`code/arrangement_optimization/scripts/`), the benchmark results page and episode videos (`code/paths/scripts/`).
- `code/sampler/robot/scripts/`: the arm bring-up probes, the robot episode and its report.
- `code/sampler/scripts/`: the two render helpers the sampler, paths and robot scripts import.
- `tests/`: Frigidaire tests, discovered by the repository pytest configuration.
- `docs/`: current geometry notes (the historical notes are in `code/sampler/docs/history/`; the collection
  README template moved to `data/assets/frigidaire_fdpc4221as/source/`).

The earlier four-view lower-rack polish (of the standalone Bosch 800 asset) left with that asset folder
(history on the hub, 2026-11-01).

The root project installs `dishsim` and the block packages (`asset`, `sampler`, ...); refresh an
existing editable installation with `code/util/run_py.sh -m pip install -e . --no-deps`.
The package keeps USD and Kit imports out of module initialization.

```python
from asset.twin import spawn, apply_mode, step_passive
from asset.paths import ASSET_DIR

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
code/util/run_py.sh -m pytest code/
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

## Tests without Isaac

Host preflight, experiment tests and saved-evidence analysis do not start Isaac.

The upper/lower loading-pattern unittest files can run with host NumPy and Pillow;
the component update, lower clearance, assembly evidence contract, packaging and staging
tests moved with their scripts to `data/assets/frigidaire_fdpc4221as/source/` on 2026-10-21.
The complete pytest suite and actual physics/render checks require the existing
runtime; host contract tests do not substitute for them.

Original development documentation is preserved in
[code/sampler/docs/history/development_notes.md](../sampler/docs/history/development_notes.md). Its old
paths and loading certifications are historical context.

Geometry: [docs/geometry.md](docs/geometry.md).
