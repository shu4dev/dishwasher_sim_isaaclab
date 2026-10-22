# initialization/frigidaire

The demo and the Isaac start-state gate of the Frigidaire FDPC4221AS twin, plus the random-load farm
(`frigidaire_random_load_kit.py`, one long-lived Kit worker building random Isaac-settled 24-dish loads, and
`frigidaire_random_load_farm.py`, its host-side launcher and harvester; usage in their docstrings). The 18 scripts
that built and certified the twin (build, stage, inspect, package, assembly evidence, rack previews, clearance, dimensions, tape check,
cutlery search, load planning, claims, full-load evidence and renders, render settings, loaded-scene smoke) moved on
2026-10-21 to the asset dataset's `data/assets/frigidaire_fdpc4221as/source/`, the
Bosch lower-rack polish pair (`polish_lower_rack.py`, `lower_rack_polish_evidence.py`) to
`data/assets/bosch800/source/`; each folder's README gives the exact commands. Defaults are defined in the script
itself unless another file is named. Two defaults come from `code/frigidaire/src/dishsim_frigidaire/paths.py`:
`ASSET_DIR` = `data/assets/frigidaire_fdpc4221as/usd`, `IMAGE_DIR` = `data/media/frigidaire_fdpc4221as/images`.
Kit = run through `code/util/run_kit.sh ... --headless`; Kit-free = `code/util/run_py.sh` (or host `python3`
where noted). Reference: `code/frigidaire/README.md`, `code/frigidaire/docs/geometry.md`.

| Script | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `frigidaire_demo.py` | Kit. Minimal loader: ordered opening and closing, or passive handle forces. | `--mode` (`scripted`, or `passive`), `--usd` (the current collection), `--x`, `--y` (0), `--yaw` (0, degrees) | the appliance USD | the `[RESULT]` line |
| `frigidaire_initial_state_validate.py` | Kit (defaults `--device cpu`, headless). One fresh Isaac process: a measured baseline (no `--manifest`) or the joint validation of a load manifest; the HOTEC bench and the top-5 search call it as their gate. | `--manifest`, `--out-dir` (required), `--order` (`upper_first`), `--max-wall-seconds` (240), `--peak-penetration-m` (`LIMITS` in `code/frigidaire/src/dishsim_frigidaire/random_poses.py`), `--usd` (`data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc`) | the manifest, the USD | `<out-dir>/result.json` |
