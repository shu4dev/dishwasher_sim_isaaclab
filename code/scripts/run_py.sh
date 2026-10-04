#!/usr/bin/env bash
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
#
# Kit-free python launcher (pytest, planners, tools) — same host→container forwarding as
# run_kit.sh, but execs Kit's python directly without booting Omniverse.
#
#   code/scripts/run_py.sh -m pytest code/tests/
#   code/scripts/run_py.sh code/scripts/tools/restore_assets.py --repo shu4dev/dishsim-assets
#
# PYTEST_DISABLE_PLUGIN_AUTOLOAD is baked in: hydra's pytest plugin (pulled in by Isaac's
# python) breaks collection outside Kit, and the variable only affects pytest runs.
set -e
if [ ! -d /isaac-sim ]; then
    ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
    REL="$(realpath --relative-to="$ROOT" "$PWD" 2>/dev/null || echo .)"
    case "$REL" in ..*) REL=. ;; esac
    # HF_TOKEN is forwarded only when set in this shell (archive_assets.py --upload);
    # it stays a process env var - never written to disk or the compose file.
    exec docker exec -w "/workspace/dishsim/$REL" -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
        ${HF_TOKEN:+-e HF_TOKEN} \
        "${DISHSIM_NAME:-dishsim-isaac}" /workspace/dishsim/code/scripts/run_py.sh "$@"
fi
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
# Kit-free `pxr` (USD python) for the Frigidaire FCL collision helpers (loading.collision_parts):
# Kit's own USD extension carries the modules and their .so files; both paths must be set
# before the interpreter starts (the loader reads LD_LIBRARY_PATH once).
USD_LIBS="$(ls -d /isaac-sim/extscache/omni.usd.libs-* 2>/dev/null | head -n 1)"
if [ -n "$USD_LIBS" ]; then
    export PYTHONPATH="${USD_LIBS}${PYTHONPATH:+:$PYTHONPATH}"
    export LD_LIBRARY_PATH="${USD_LIBS}/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
exec /isaac-sim/python.sh "$@"
