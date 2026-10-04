# util/tools

Bring-up, the install gate, and the asset archive. Defaults are defined in the script itself.

| Script | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `bootstrap.sh` | Host, idempotent. Builds the image if absent, `compose up -d`, waits for the entrypoint's installs, restores the archive, then the `kit_smoke.py` gate. | `--repo <id>` (`shu4dev/dishsim-assets`) | `code/util/docker/` | the restored data (see `restore_assets.py`) |
| `restore_assets.py` | Kit-free. Downloads the tarballs (or takes local ones), safe-extracts them into `data/`, verifies every file's sha256 against the tarball manifest, and checks every restored cache's `config_hash` against the current `code/src/dishsim/config.py`; then runs `pytest code/tests/`. | `--repo` (`shu4dev/dishsim-assets`), `--kinds` (`assets`; also `media`, `models`, `evidence`), `--local` (tarball paths instead of downloading), `--skip_tests` | the public HF dataset, `latest.json` | `data/assets/`, `data/media/`, `data/results/` |
| `archive_assets.py` | Kit-free. Builds manifest-stamped tarballs per kind and optionally uploads them, merging into the remote `latest.json`. `--status` is read-only and needs no token. Never `--upload` the documented kinds from this box: `assets` walks all of `data/results/`, `models` includes HOTEC. | `--kinds` (required unless `--status`), `--status`, `--upload`, `--no-build` (with `--tag`), `--tag` (today + HEAD short SHA), `--repo` (`DEFAULT_REPO` = `shu4dev/dishsim-assets`), `--fresh` | the data trees of each kind | `data/outputs/archive/dishsim_<kind>_<tag>.tar.gz` |
| `kit_smoke.py` | Kit. Install gate: `fcl`, `coacd`, `trimesh`, `imageio` import inside Kit, and a headless camera frame is non-black. | `--out_dir` (`data/media/smoke`) | none | `data/media/smoke/smoke.png` |
