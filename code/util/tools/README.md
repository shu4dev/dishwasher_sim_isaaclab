# util/tools

Bring-up and the install gate. Defaults are defined in the script itself. The asset archive and restore scripts
were removed on 2026-10-21: `data/assets` is a copy of the HF dataset `shu4dev/dishsim-assets`, restored with
`hf download shu4dev/dishsim-assets --repo-type dataset --local-dir data/assets` and published with
`hf upload shu4dev/dishsim-assets data/assets . --repo-type dataset --commit-message "<what changed>"` (needs
`hf auth login` with write access).

| Script | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `bootstrap.sh` | Host, idempotent. Builds the image if absent, `compose up -d`, waits for the entrypoint's installs, then the `kit_smoke.py` gate. Restores no assets (`hf download` them into `data/assets` first). | none | `code/util/docker/` | the image and the running container; the gate's log `/tmp/dishsim_kit_smoke.log` |
| `kit_smoke.py` | Kit. Install gate: `fcl`, `coacd`, `trimesh`, `imageio` import inside Kit, and a headless camera frame is non-black. | `--out_dir` (`data/media/smoke`) | none | `data/media/smoke/smoke.png` |
