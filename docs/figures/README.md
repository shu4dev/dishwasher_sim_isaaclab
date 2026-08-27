# Figure provenance

Every tracked figure is a curated copy of a script-generated file under the gitignored
`media/` tree. To regenerate: run the producing command, then copy the media file here under
the tracked name.

| figure | producing command | media source |
|---|---|---|
| `loaded_iso.png` | produced by the retired `capacity_fill.py` (git history): 29-item hand-authored full load, physically settled, racks closable | `media/fill/loaded_iso.png` |
| `object_library.png` | asset authoring pipeline (retired to git history with the public-asset release; regenerated 2026-08-10 during the YCB-mug migration) | `media/assets/library_sheet.png` |
| `rack_geometry.png` | `$PY scripts/setup/preview_rack.py` | `media/collision_world/rack_gen_preview_E_shelf_1_04.png` |
| `slot_detection.png` | `$PY scripts/setup/derive_slots.py --object mug --scenario placement` | `media/goals/slot_detection.png` |
| `bosch800_loaded_top.png` | produced by the retired robot episode runner (git history, `run_task.py`, run `bosch_sanity_load2`); a physically-settled full-load top view — regenerate the equivalent via `reveal_render.py --plan` | `media/task/bosch_sanity_load2/ep002_final_top.png` |
| `bosch800_loaded_reveal.png` | produced by the episode-era `reveal_render.py` (git history) from recorded measured poses, max settle drift 1.1 mm; regenerate via `scripts/run_kit.sh scripts/evaluation/reveal_render.py --headless --enable_cameras --plan results/capacity/bosch800/side_winner/full_load_plan.json` | `media/task/bosch_sanity_load2/ep002_reveal_iso.png` |

`$PY` = `/workspace/isaaclab/env_isaaclab/bin/python` (the planning venv).
