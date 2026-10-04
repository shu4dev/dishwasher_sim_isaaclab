# util/frigidaire

The two render helpers. Scripts in `planner/frigidaire/` and `execution/frigidaire/` import them by bare name
after putting `code/util/frigidaire` on `sys.path`: `frigidaire_bench_kit.py`, `frigidaire_hotec_load.py`,
`frigidaire_planner_video.py`, `frigidaire_robot_episode.py`. Keep the two files together: the render helper
imports the report helper from its own folder. Defaults are defined in the script itself.

| Script | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `frigidaire_initial_state_render.py` | Kit. Renders one measured accepted state with Isaac RTX, without advancing physics, plus a saved-versus-rendered transform audit. As a module: `CAMERA`, `WIDTH`, `HEIGHT`, `item_tints`, `caption_fonts`, `digest`. | `--state`, `--out-dir` (both required), `--usd` (`data/build/frigidaire_collection/usd/fdpc4221as.usdc`) | the state JSON, the USD | `<id>_initial.png` (1920x1440), `<id>_render_evidence.json` |
| `frigidaire_initial_state_report.py` | Kit-free. A factual report from saved multi-dish initial-state records; missing states stay explicit. As a module: `validate_state`, `pose_error`. | `--out-dir` (required, the run folder) | the run's records | `<out-dir>/experiment_report.md` |
