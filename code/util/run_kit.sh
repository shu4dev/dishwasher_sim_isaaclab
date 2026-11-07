#!/usr/bin/env bash
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
#
# Kit launcher for the docker era (Isaac Sim 6.2.0 + Isaac Lab 3.0.0 image on the Brev box).
#
# On the host it forwards itself into the long-lived `dishsim-isaac` container at the cwd mapped
# under /workspace/dishsim. Inside the container it execs Kit's python directly: Isaac Lab 3.0
# deprecates isaaclab.sh, and no venv exists in our image, so no EXP_PATH shimming is needed (that
# landmine was a property of the old Brev venv layout). Scripts boot Kit through dishsim.app.
#
# Usage (from the project root, host or container):
#   code/util/run_kit.sh code/util/tools/kit_smoke.py --headless --enable_cameras
set -e
if [ ! -d /isaac-sim ]; then
    ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
    REL="$(realpath --relative-to="$ROOT" "$PWD" 2>/dev/null || echo .)"
    case "$REL" in ..*) REL=. ;; esac
    exec docker exec -w "/workspace/dishsim/$REL" ${HF_TOKEN:+-e HF_TOKEN} "${DISHSIM_NAME:-dishsim-isaac}" \
        /workspace/dishsim/code/util/run_kit.sh "$@"
fi
exec /isaac-sim/python.sh "$@"
