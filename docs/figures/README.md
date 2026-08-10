# Figure provenance

Every tracked figure is a curated copy of a script-generated file under the gitignored
`media/` tree. To regenerate: run the producing command, then copy the media file here under
the tracked name.

| figure | producing command | media source |
|---|---|---|
| `loaded_iso.png` | `scripts/run_kit.sh scripts/setup/capacity_fill.py --headless --enable_cameras` | `media/fill/loaded_iso.png` |
| `object_library.png` | asset authoring pipeline (retired to git history with the public-asset release; regenerated 2026-08-10 during the YCB-mug migration) | `media/assets/library_sheet.png` |
| `rack_geometry.png` | `$PY scripts/setup/preview_rack.py` | `media/collision_world/rack_gen_preview_E_shelf_1_04.png` |
| `slot_detection.png` | `scripts/run_kit.sh scripts/setup/goal_configs.py --headless --enable_cameras --object mug` | `media/goals/slot_detection.png` |
| `planner_tree.png` | `$PY scripts/evaluation/plan_visual.py --slot 6 --seed 0` | `media/trials/rrt_connect_viz_*.png` |
| `trial_replay.png` | `scripts/run_kit.sh scripts/evaluation/render_videos.py --headless --enable_cameras --trial <trial.npz>` | `media/trials/replay/*_final.png` |
| `grasp_force_vs_theta.png` | `scripts/run_kit.sh scripts/setup/calibrate_grasp.py --headless --enable_cameras` | `media/calibration/force_vs_theta.png` |

`$PY` = `/workspace/isaaclab/env_isaaclab/bin/python` (the planning venv).
