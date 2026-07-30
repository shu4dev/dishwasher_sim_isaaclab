#!/usr/bin/env bash
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
#
# Kit launcher for the venv era.
#
# With /workspace/isaaclab/env_isaaclab present, `isaaclab.sh -p` resolves Python to the bare
# venv interpreter, which lacks the environment `_isaac_sim/python.sh` would have exported
# (EXP_PATH / CARB_APP_PATH / ISAAC_PATH, the kit LD_LIBRARY_PATH entries, and the kit
# PYTHONPATH). Without it, AppLauncher dies at boot with `KeyError: 'EXP_PATH'`. This shim
# exports that environment, then hands off to the wrapper unchanged.
#
# Usage (from the project root):
#   scripts/run_kit.sh scripts/05_kit_smoke.py --headless --enable_cameras
set -e
ISAAC_ROOT=/workspace/isaaclab/_isaac_sim
export CARB_APP_PATH="$ISAAC_ROOT/kit"
export ISAAC_PATH="$ISAAC_ROOT"
export EXP_PATH="$ISAAC_ROOT/apps"
# shellcheck disable=SC1091
source "$ISAAC_ROOT/setup_python_env.sh"
exec /workspace/isaaclab/isaaclab.sh -p "$@"
