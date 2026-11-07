# util/docker

The runtime: Isaac Sim 6.2.0 + Isaac Lab v3.0.0 in the long-lived container `dishsim-isaac` on the Brev box (since
2026-11-01). corallab's driver 535 caps it at Isaac Sim 4.5.0 + Isaac Lab v2.1.1, which these files built before (git tag
`isaac45-final`). Never change the pins in place. Full write-up: `docs/environment.md`.

| File | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `compose.yaml` | Starts the container (builds nothing): editable-installs the repo on every start, mounts the repo at `/workspace/dishsim` and the 2 TB drive at its host path, keeps Kit's caches on the 2 TB drive. Start: `DISHSIM_GPU=<n> docker compose -f code/util/docker/compose.yaml up -d`. | `DISHSIM_GPU` (1; pick the least-loaded GPU), `DISHSIM_NAME` (`dishsim-isaac`), `DISHSIM_REPO` (`../../..`, the repo root seen from this folder) | the image `dishsim-isaac:6.2.0` | the container's writable layer only; everything mutable goes to the 2 TB drive |
| `Dockerfile` | Build record of the image: Isaac Sim 6.2.0 base, Isaac Lab v3.0.0 (isaaclab, isaaclab_physx, isaaclab_newton, isaaclab_ov editable with `--no-deps`), the pinned planning deps from `requirements-planning.txt`, all in Kit's python 3.12 (no venv). Build from the repo root: `docker build -f code/util/docker/Dockerfile -t dishsim-isaac:6.2.0 .` | the pins in `requirements-planning.txt` | `requirements-planning.txt` (the only file the build COPYs) | the image |
