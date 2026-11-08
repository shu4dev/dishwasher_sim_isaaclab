# paths

Block 3a PATHS of `docs/repo_map.txt` (initial state to goal state): the path from the initial state to the goal state,
one teleport move at a time, judged solved within the 60 s budget, moves and faults; written to `runs/<sample>.json`,
the videos and the page. The package is `paths`: `episodes.py` (the run stage: own loads, episodes, S_final; track B's
own loads are `code/optimal_arrangement/certify.py`), `video.py` (the video stage) and `report.py` (the bench page over a
results folder). The CLI is `code/sampler/scripts/bench.py`; rerun these stages on a stored sample:

```bash
code/util/run_py.sh code/sampler/scripts/bench.py --from run --to report --input data/results/bench/v1/<sample>.json
```

Output: the episodes in `data/results/bench/v1/runs/`, media and the page (`index.html`) under `data/media/bench/v1/`.

## Scripts

Defaults are defined in the script itself unless another file is named. Kit = through `code/util/run_kit.sh`;
Kit-free = `code/util/run_py.sh`.

| Script | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `frigidaire_planner_video.py` | Kit. Stop-motion video of one recorded bench episode on the bench kit's scene (`make_backend`, dishes kinematic): the stored start, then each executed move's `poses_after` written, one tick, rendered, captioned (S_final and S_ref from the `.fair.json`); no physics. The video stage (`code/paths/video.py`) runs it. | `--instance`, `--episode`, `--out-dir` (all required), `--video-width` (1280; the stage passes 640), `--hold-frames` (18), `--fps` (24), `--renders-per-frame` (the kit's `STILL_RENDERS`, 48), `--warmup-renders` (32), `--usd` (`data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc`) | the run stage's `instance.json`, the episode record and its `.fair.json` | `<sample>__<track>__<algorithm>.mp4` (never overwritten), start and end PNGs, `*_video_evidence.json` |
| `frigidaire_bench_page.py` | Kit-free. The benchmark's results page as one folder to publish (sprites keep it under 255 files, 64 MB). | `--out` (records, default `data/results/benchmark/frigidaire_hotec`), `--media` (default `data/media/benchmark/frigidaire_hotec`) | records and media | `data/media/benchmark/frigidaire_hotec/page/`: `index.html`, `sheets/`, `analysis/`, `video/`, `files.json` |
