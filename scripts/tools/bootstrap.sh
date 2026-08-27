#!/usr/bin/env bash
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
#
# One-command bring-up on a fresh instance: planning venv + pinned deps + editable install +
# archive restore (every collision cache, prop, machine USD and recorded result — the ~2.5 h
# of one-time Kit work nobody should repay). After this, experiments run immediately:
#
#   scripts/tools/bootstrap.sh                # default public archive
#   scripts/tools/bootstrap.sh --repo <id>    # a fork's archive
#
# Idempotent: every step skips work it finds already done. Judge success from the final
# [RESULT] line (the restore validates every cache's config_hash and runs the test suite).
set -euo pipefail

VENV=/workspace/isaaclab/env_isaaclab
KIT_PY=/isaac-sim/kit/python/bin/python3
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
REPO_ARGS=("--repo" "shu4dev/dishsim-assets")
if [ "${1:-}" = "--repo" ]; then REPO_ARGS=("--repo" "$2"); fi

if [ ! -x "$VENV/bin/python" ]; then
    echo "[INFO] creating planning venv at $VENV"
    "$KIT_PY" -m venv --system-site-packages "$VENV"
else
    echo "[INFO] venv present"
fi

echo "[INFO] installing pinned planning deps + archive tooling"
"$VENV/bin/pip" install -q -r "$ROOT/requirements-planning.txt"
"$VENV/bin/pip" install -q requests pyyaml filelock tqdm fsspec
"$VENV/bin/pip" install -q -e "$ROOT"

echo "[INFO] restoring the public archive (caches, props, machine USDs, results)"
"$VENV/bin/python" "$ROOT/scripts/tools/restore_assets.py" "${REPO_ARGS[@]}"

echo "[INFO] bootstrap complete — try:"
echo "  $VENV/bin/python scripts/setup/plan_full_load.py --machine bosch800 --placement side_winner"
echo "  scripts/run_kit.sh scripts/setup/capacity_fill.py --headless --enable_cameras"
