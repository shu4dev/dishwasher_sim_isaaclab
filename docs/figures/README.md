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
| `bosch800_reachable_capacity.png` | `$PY scripts/setup/plan_full_load.py --machine bosch800 --placement side_winner --fig_docs` (Kit-free reach map per loading state: filled = reachable slot, hollow = unreachable, ringed = assigned in the certified full load; see docs/success_criteria.md "Reachable capacity") | `results/capacity/bosch800/side_winner/reachability.png` |
| `bosch800_loaded_top.png` | `scripts/run_kit.sh scripts/experiment/run_task.py --headless --enable_cameras --machine bosch800 --placement side_winner --phases "third_out:fork=8;placement:plate=2,bowl=4" --seed 2 --run_id bosch_sanity_load2 --orbit` (final top still: 3 bowls + plate seated on the extended lower rack, forks stowed above) | `media/task/bosch_sanity_load2/ep002_final_top.png` |
| `bosch800_loaded_reveal.png` | `scripts/run_kit.sh scripts/evaluation/reveal_render.py --headless --enable_cameras --run bosch_sanity_load2 --ep ep002` — both loaded racks extended, every item at its RECORDED measured pose (placed forks from the phase-0 trajectory final frame, lower-rack items from phase-1's), settled; max drift 1.1 mm proves the tableau is physically self-consistent | `media/task/bosch_sanity_load2/ep002_reveal_iso.png` |

`$PY` = `/workspace/isaaclab/env_isaaclab/bin/python` (the planning venv).
