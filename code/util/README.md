# util

What every stage uses: the two launchers, the container files, bring-up, the install gate, and the two render
helpers that planner and execution scripts import. The assets are not restored here: `data/assets` is a copy of
the HF dataset `shu4dev/dishsim-assets` (since 2026-10-21): `hf download` to restore, `hf upload` to publish.

- `run_kit.sh`: boots Kit. On the host it forwards itself into the container `${DISHSIM_NAME:-dishsim-isaac}` at
  the cwd mapped under `/workspace/dishsim`; inside it hands off to `/workspace/isaaclab/isaaclab.sh -p`.
- `run_py.sh`: Kit-free python the same way (`/isaac-sim/python.sh`), with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` and
  Kit's USD extension on the path, so `pxr` imports without Kit.
- Both forward `HF_TOKEN` only when it is set in the shell. They always run the code under `/workspace/dishsim`
  (the main checkout), not a git worktree.

Folders: [tools/](tools/README.md), [frigidaire/](frigidaire/README.md), [docker/](docker/README.md).

## Commands, in order

```bash
hf download shu4dev/dishsim-assets --repo-type dataset --local-dir data/assets
DISHSIM_GPU=<n> docker compose -f code/util/docker/compose.yaml up -d
code/util/tools/bootstrap.sh
code/util/run_py.sh -m pytest code/tests code/frigidaire/tests
code/util/run_kit.sh code/util/tools/kit_smoke.py --headless --enable_cameras
```

## Package modules

Shared helpers, with the stages that import them:

- `dishsim`: `config`, `quats`, `transforms` (initialization, planner), `checks` (initialization), `media`
  (initialization, planner, execution).
- `dishsim_frigidaire`: `paths` (initialization, planner).
