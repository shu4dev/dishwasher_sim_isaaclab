# util

What every stage uses: the two launchers, the container files, bring-up, archive and restore, the install gate,
and the two render helpers that planner and execution scripts import.

- `run_kit.sh`: boots Kit. On the host it forwards itself into the container `${DISHSIM_NAME:-dishsim-isaac}` at
  the cwd mapped under `/workspace/dishsim`; inside it hands off to `/workspace/isaaclab/isaaclab.sh -p`.
- `run_py.sh`: Kit-free python the same way (`/isaac-sim/python.sh`), with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` and
  Kit's USD extension on the path, so `pxr` imports without Kit.
- Both forward `HF_TOKEN` only when it is set in the shell. They always run the code under `/workspace/dishsim`
  (the main checkout), not a git worktree.

Folders: [tools/](tools/README.md), [frigidaire/](frigidaire/README.md), [docker/](docker/README.md).

## Commands, in order

```bash
DISHSIM_GPU=<n> docker compose -f code/util/docker/compose.yaml up -d
code/util/tools/bootstrap.sh
code/util/run_py.sh -m pytest code/tests code/frigidaire/tests
code/util/run_kit.sh code/util/tools/kit_smoke.py --headless --enable_cameras
code/util/run_py.sh code/util/tools/archive_assets.py --status
```

## Package modules

Shared helpers, with the stages that import them:

- `dishsim`: `config`, `quats`, `transforms` (initialization, planner), `checks` (initialization), `media`
  (initialization, planner, execution).
- `dishsim_frigidaire`: `paths`, `usd_bootstrap` (initialization, planner).
