# Figure provenance

Every tracked figure is a curated copy of a script-generated file under the gitignored
`media/` tree. To regenerate: run the producing command, then copy the media file here under
the tracked name.

| figure | producing command | media source |
|---|---|---|
| `instance_goal_iso.png` | `scripts/run_kit.sh scripts/evaluation/instance_views.py --headless --enable_cameras --instance results/instances/bosch800/placement/perturbed_s0.json` (2026-08-30, corallab) | `media/instances/bosch800/placement/perturbed_s0_goal_iso.png` |
| `loaded_iso.png` | produced by the retired `capacity_fill.py` (git history): 29-item hand-authored full load, physically settled, racks closable | robot-era media (retired; not in any archive) |
| `object_library.png` | asset authoring pipeline (retired to git history with the public-asset release; regenerated 2026-08-10 during the YCB-mug migration) | robot-era media (retired; not in any archive) |
| `rack_geometry.png` | retired `preview_rack.py` (git history) | robot-era media (retired; not in any archive) |
| `slot_detection.png` | retired `derive_slots.py` (git history) | robot-era media (retired; not in any archive) |
| `bosch800_loaded_reveal.png` | retired `reveal_render.py` (git history), from recorded measured poses, max settle drift 1.1 mm | robot-era media (retired; not in any archive) |
| `hotec_profiles.png` | `scripts/run_py.sh -m dishsim.hotec_gen --out assets/models/hotec_wheatstraw/v1` (2026-09-21, corallab) | `media/hotec_wheatstraw/v1/hotec_profiles.png` |
| `hotec_exposure_best.png` | `scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_hotec_exposure_search.py --assets assets/models/hotec_wheatstraw/v2 --sweeps 3 --seed 0` then the Isaac run `--layout results/exposure/frigidaire/hotec/best_layout.json --out-dir results/hotec/frigidaire/v9 --tag hotec_v9` and `frigidaire_hotec_exposure_figure.py` (2026-09-22, corallab): the v9 settled still (1280x960) beside the exposure heat map of run v8 vs the best load (S 0.159 -> 0.212) | `media/hotec_wheatstraw/v9/hotec_loaded_fdpc4221as.png` + `media/exposure_hotec/heatmap.png` |
| `hotec_placement_steps.png` | `scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_hotec_load.py --layout results/exposure/frigidaire/hotec/best_layout_ordered.json --assets assets/models/hotec_wheatstraw/v2 --out-dir results/hotec/frigidaire/v10 --tag hotec_v10 --sequential --headless --enable_cameras --device cpu` (2026-09-22, corallab): the 24 per-step stills tiled 6 x 4 (step 11 marked unsettled) | `media/hotec_wheatstraw/v10/hotec_v10_step_NN.png` |
| `hotec_loaded_fdpc4221as.png` | `scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_hotec_load.py --layout results/hotec/frigidaire/v8/layout.json --assets assets/models/hotec_wheatstraw/v2 --out-dir results/hotec/frigidaire/v8 --tag hotec_v8 --headless --enable_cameras --device cpu` (full 24-piece load: `--search --plates front --bowls-upper 10`, v2 massed assets on the first tape build, now history/v4, 2026-09-22, corallab), settled still downscaled to 1280x960 (regenerated 2026-09-22; the v1 to v7 stills stay under `media/hotec_wheatstraw/v1/` to `v7/`) | `results/hotec/frigidaire/v8/hotec_v8_settled.png` (copy under `media/hotec_wheatstraw/v8/`) |
| `frigidaire_upper_rack_cm.png` | `python3 frigidaire/scripts/evaluation/frigidaire_rack_dimensions_cm.py` (Kit-free, regenerated 2026-09-23, corallab): upper rack plan, front section and side view with every dimension in cm, from the generated source geometry `upper_tines_4x13_v4`; teal = the user's tape values, orange = derived margins, grey = photo-fit estimates | `media/frigidaire_rack_dimensions/frigidaire_upper_rack_cm.png` |
| `frigidaire_lower_rack_cm.png` | same command: lower rack plan and side view plus the basket's long side, end and top views, every dimension in cm (`lower_tines_6x12_v3`, `basket_1x4_320x95_v4`) | `media/frigidaire_rack_dimensions/frigidaire_lower_rack_cm.png` |
