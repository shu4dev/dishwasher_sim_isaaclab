#!/usr/bin/env bash
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
#
# One-command bring-up on the corallab machine: build the image, start the container, and
# restore the public archive (every collision cache, prop, machine USD and recorded result —
# the ~2.5 h of one-time Kit work nobody should repay). After this, experiments run
# immediately:
#
#   scripts/tools/bootstrap.sh                # default public archive
#   scripts/tools/bootstrap.sh --repo <id>    # a fork's archive
#
# The runtime environment itself lives in docker/ (Dockerfile + compose.yaml); all deps are
# baked into the image — this script no longer installs anything on the host. Idempotent:
# every step skips work it finds already done. Judge success from the final [RESULT] line
# (the restore validates every cache's config_hash).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
REPO_ARGS=("--repo" "shu4dev/dishsim-assets")
if [ "${1:-}" = "--repo" ]; then REPO_ARGS=("--repo" "$2"); fi

if ! docker image inspect dishsim-isaac:4.5.0 >/dev/null 2>&1; then
    echo "[INFO] building dishsim-isaac:4.5.0"
    docker build -f "$ROOT/docker/Dockerfile" -t dishsim-isaac:4.5.0 "$ROOT"
else
    echo "[INFO] image present"
fi

echo "[INFO] starting the runtime container"
docker compose -f "$ROOT/docker/compose.yaml" up -d

echo "[INFO] restoring the public archive (caches, props, machine USDs, results)"
"$ROOT/scripts/run_py.sh" scripts/tools/restore_assets.py "${REPO_ARGS[@]}"

echo "[INFO] bootstrap complete — try:"
echo "  scripts/run_kit.sh scripts/experiment/run_task.py --headless --enable_cameras \\"
echo "      --machine bosch800 --placement side_winner --scenario placement \\"
echo "      --spawn \"cup=1\" --seed 1 --run_id bringup"
