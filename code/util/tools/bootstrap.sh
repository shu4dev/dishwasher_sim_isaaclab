#!/usr/bin/env bash
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
#
# One-command bring-up via compose: build the image, start the container and run the install
# gate (the Brev box recreates its container by hand, docs/environment.md). The assets are not
# restored here: data/assets is a copy of the HF dataset shu4dev/dishsim-assets (see
# data/assets/README.md), downloaded once before experiments:
#
#   hf download shu4dev/dishsim-assets --repo-type dataset --local-dir data/assets
#   code/util/tools/bootstrap.sh
#
# The runtime environment itself lives in code/util/docker/ (Dockerfile + compose.yaml); all deps are
# baked into the image — this script no longer installs anything on the host. Idempotent:
# every step skips work it finds already done. Judge success from the final [RESULT] line.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"

if ! docker image inspect dishsim-isaac:6.2.0 >/dev/null 2>&1; then
    echo "[INFO] building dishsim-isaac:6.2.0"
    docker build -f "$ROOT/code/util/docker/Dockerfile" -t dishsim-isaac:6.2.0 "$ROOT"
else
    echo "[INFO] image present"
fi

echo "[INFO] starting the runtime container"
docker compose -f "$ROOT/code/util/docker/compose.yaml" up -d

# `up -d` returns before the entrypoint's editable installs finish (isaaclab core + pytest +
# the repo — the container self-heals them on every start). Racing it makes the first
# pytest run fail with "No module named pytest" on a freshly (re)created container.
echo "[INFO] waiting for the container entrypoint (editable installs) ..."
for _ in $(seq 1 60); do
    if docker exec dishsim-isaac /isaac-sim/python.sh -c "import isaaclab, pytest, dishsim" \
        >/dev/null 2>&1; then break; fi
    sleep 5
done
docker exec dishsim-isaac /isaac-sim/python.sh -c "import isaaclab, pytest, dishsim" \
    || { echo "[RESULT] FAIL (container entrypoint did not become ready)"; exit 1; }

echo "[INFO] install gate: kit_smoke (one headless Kit boot + camera render)"
# Kit's exit code is not a verdict (run_kit.sh, CLAUDE.md), so `set -e` gates nothing —
# the [RESULT] grep is the actual gate.
"$ROOT/code/util/run_kit.sh" code/util/tools/kit_smoke.py --headless --enable_cameras 2>&1 \
    | tee /tmp/dishsim_kit_smoke.log
grep -q '^\[RESULT\] PASS' /tmp/dishsim_kit_smoke.log \
    || { echo "[RESULT] FAIL (kit_smoke gate)"; exit 1; }

echo "[INFO] bootstrap complete — try:"
echo "  code/util/run_py.sh code/sampler/scripts/bench.py --kinds bowl --count bowl=7 --counter-allowance 3 --seed 0 --to report --out data/results/bench/v1"
