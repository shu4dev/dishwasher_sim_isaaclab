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
