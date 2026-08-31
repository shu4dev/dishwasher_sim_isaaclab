#!/usr/bin/env bash
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
#
# Kit-free python launcher (pytest, planners, tools) — same host→container forwarding as
# run_kit.sh, but execs Kit's python directly without booting Omniverse.
#
#   scripts/run_py.sh -m pytest tests/
#   scripts/run_py.sh scripts/tools/restore_assets.py --repo shu4dev/dishsim-assets
#
# PYTEST_DISABLE_PLUGIN_AUTOLOAD is baked in: hydra's pytest plugin (pulled in by Isaac's
# python) breaks collection outside Kit, and the variable only affects pytest runs.
set -e
if [ ! -d /isaac-sim ]; then
    ROOT="$(cd "$(dirname "$0")/.." && pwd)"
    REL="$(realpath --relative-to="$ROOT" "$PWD" 2>/dev/null || echo .)"
    case "$REL" in ..*) REL=. ;; esac
    exec docker exec -w "/workspace/dishsim/$REL" -e PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
        dishsim-isaac /workspace/dishsim/scripts/run_py.sh "$@"
fi
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
exec /isaac-sim/python.sh "$@"
