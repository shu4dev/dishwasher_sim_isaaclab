# util/docker

The runtime: Isaac Sim 4.5.0 + Isaac Lab v2.1.1 in the long-lived container `dishsim-isaac`. Never change the
pins; the host driver caps Isaac Sim at 4.5.0. Full write-up: `docs/environment.md`.

| File | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `compose.yaml` | Starts the container (builds nothing): editable-installs the repo on every start, mounts the repo at `/workspace/dishsim` and the 2 TB drive at its host path, keeps Kit's caches on the 2 TB drive. Start: `DISHSIM_GPU=<n> docker compose -f code/util/docker/compose.yaml up -d`. | `DISHSIM_GPU` (1; pick the least-loaded GPU), `DISHSIM_NAME` (`dishsim-isaac`), `DISHSIM_REPO` (`../../..`, the repo root seen from this folder) | the image `dishsim-isaac:4.5.0` | the container's writable layer only; everything mutable goes to the 2 TB drive |
| `Dockerfile` | Build record of the image: Isaac Sim 4.5.0 base, Isaac Lab v2.1.1 (`--install none`), the pinned planning deps from `requirements-planning.txt`, all in Kit's python (no venv). Build from the repo root: `docker build -f code/util/docker/Dockerfile -t dishsim-isaac:4.5.0 .` | the pins in `requirements-planning.txt` | `requirements-planning.txt` (the only file the build COPYs) | the image |
