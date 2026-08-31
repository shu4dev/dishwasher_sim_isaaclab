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

# `up -d` returns before the entrypoint's editable installs finish (isaaclab core + pytest +
# the repo — the container self-heals them on every start). Racing it makes the restore's
# pytest gate fail with "No module named pytest" on a freshly (re)created container.
echo "[INFO] waiting for the container entrypoint (editable installs) ..."
for _ in $(seq 1 60); do
    if docker exec dishsim-isaac /isaac-sim/python.sh -c "import isaaclab, pytest, dishsim" \
        >/dev/null 2>&1; then break; fi
    sleep 5
done
docker exec dishsim-isaac /isaac-sim/python.sh -c "import isaaclab, pytest, dishsim" \
    || { echo "[RESULT] FAIL (container entrypoint did not become ready)"; exit 1; }

echo "[INFO] restoring the public archive (caches, props, machine USDs, results)"
"$ROOT/scripts/run_py.sh" scripts/tools/restore_assets.py "${REPO_ARGS[@]}"

echo "[INFO] install gate: kit_smoke (one headless Kit boot + camera render)"
# isaaclab.sh -p exits 0 even when the wrapped script crashes, so `set -e` gates nothing —
# the [RESULT] grep is the actual gate.
"$ROOT/scripts/run_kit.sh" scripts/setup/kit_smoke.py --headless --enable_cameras 2>&1 \
    | tee /tmp/dishsim_kit_smoke.log
grep -q '^\[RESULT\] PASS' /tmp/dishsim_kit_smoke.log \
    || { echo "[RESULT] FAIL (kit_smoke gate)"; exit 1; }

echo "[INFO] bootstrap complete — try:"
echo "  scripts/run_kit.sh scripts/setup/gen_instances.py --headless --mode perturbed --state placement --n 3 --seed 0"
echo "  scripts/run_kit.sh scripts/experiment/run_rearrange.py --headless --instances \"results/instances/bosch800/placement/*.json\" --algorithms greedy"
