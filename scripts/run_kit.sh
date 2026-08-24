#!/usr/bin/env bash
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
#
# Kit launcher for the corallab docker era.
#
# On the host it forwards itself into the long-lived `dishsim-isaac` container (started via
# `docker compose -f docker/compose.yaml up -d`) at the cwd mapped under /workspace/dishsim.
# Inside the container it just hands off to Isaac Lab's wrapper — no venv exists in our own
# image, so `isaaclab.sh -p` resolves straight to Kit's python and no EXP_PATH shimming is
# needed (that landmine was a property of the old Brev venv layout).
#
# Usage (from the project root, host or container):
#   scripts/run_kit.sh scripts/setup/kit_smoke.py --headless --enable_cameras
set -e
if [ ! -d /isaac-sim ]; then
    ROOT="$(cd "$(dirname "$0")/.." && pwd)"
    REL="$(realpath --relative-to="$ROOT" "$PWD" 2>/dev/null || echo .)"
    case "$REL" in ..*) REL=. ;; esac
    exec docker exec -w "/workspace/dishsim/$REL" dishsim-isaac \
        /workspace/dishsim/scripts/run_kit.sh "$@"
fi
exec /workspace/isaaclab/isaaclab.sh -p "$@"
