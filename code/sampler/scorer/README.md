# sampler/scorer

Block 1a SCORER of `docs/repo_map.txt` (score a load): the food-contact surface the spray arms reach, S and S_ref. The
package is `sampler.scorer`: `exposure.py` (the ray-cast exposure scorer, revision 5, Kit-free), `dot_rules.py` (the
large-N dot-rule engine, Warp + torch on CUDA) and `score.py` (the score stage and the fair score the build and the run
use, with the 6.2 ceilings file). Tests in `tests/` (`test_exposure.py` reads the settled states in
`tests/fixtures/settled_states/`), reference in `docs/` ([exposure.md](docs/exposure.md),
[exposure_quickstart.md](docs/exposure_quickstart.md)).

## Scripts

Defaults are defined in the script itself unless another file is named. Host = `python3` on the host; Kit = through
`code/util/run_kit.sh`; Kit-free = `code/util/run_py.sh`.

| Script | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `frigidaire_hotec_exposure_search.py` | Kit-free (Warp, CUDA when available). Highest-exposure full load of the 24 HOTEC pieces by coordinate ascent on the revision-5 score S; a sampled maximum, not a proof. | `--assets` (`data/assets/hotec_wheatstraw/usd`), `--start-layout` (`data/results/hotec/frigidaire/v8/layout.json`), `--sweeps` (3), `--top` (6), `--random` (2), `--seed` (0), `--samples` (500) and `--directions` (64) from `DEFAULTS` in `code/sampler/scorer/exposure.py`, `--isolated-samples` (200), `--isolated-directions` (32), `--insertion-gate` | the assets, the start layout | `--out` (`data/results/exposure/frigidaire/hotec`): `search.json`, `best_layout.json` |
| `frigidaire_hotec_exposure_figure.py` | Kit-free. Method image of two loads: per-sample exposure top-down per rack, plus per-piece bars. | `--start`, `--best`, `--out` (all required), `--assets` (`data/assets/hotec_wheatstraw/usd`), `--samples` (500), `--directions` (64) | two layouts | the PNG at `--out` |
| `frigidaire_hotec_top5.py` | Top-5 exposure loads of all 24 dishes, each through the joint Isaac gate, then scored on the settled poses and rendered. `--all` (host) runs every stage, resumable. | `--level` (0; `LEVELS` in the script relaxes the capacity probe), `--parts` (`PARTS` = 2), `--kit-jobs` (3), `--py-jobs` (2); gate peak `GATE_PEAK_PEN_M` (0.005 m) and dish tolerance `DISH_TOL_M` (0.0008 m) in the script | the HOTEC v2 assets, the bench's families | `data/results/hotec/frigidaire/top5_20260929_touch08/`, `data/media/hotec_wheatstraw/top5_20260929_touch08/`, logs `data/logs/hotec_top5/touch08/` |

Also in `scripts/` (usage in their docstrings): `frigidaire_dependency_score.py` (the score stage's scorer),
`frigidaire_dot_rules_grid.py`, `frigidaire_dot_rules_relative.py` (the ceilings), `frigidaire_dot_rules_render.py`,
`frigidaire_dot_rules_restricted.py`, `frigidaire_dot_rules_figures.py`, `frigidaire_dot_rules_page.py` (the Dot Score
Rules page) and `frigidaire_tilt_check_figure.py`.

## The exposure scorer

`code/sampler/scorer/exposure.py` scores arrangements of the same objects by ray-cast exposure of
food-contact surfaces to the spray arms (Kit-free, Warp on the CPU); the HOTEC benchmark and the
top-5 loads use it. The demo/search/summary scripts and their v3-rack results were retired
2026-09-29. Reference: [docs/exposure.md](docs/exposure.md); quickstart:
[docs/exposure_quickstart.md](docs/exposure_quickstart.md). Live results (pre-2026-09-28 racks, history):
`data/results/exposure/frigidaire/{hotec,hotec_heightfix}`.
