#!/usr/bin/env bash
# Use the USD libraries already bundled with the pinned Isaac image; no install.
set -euo pipefail
if [[ ! -d /isaac-sim ]]; then
    exec docker exec -w /workspace/dishsim dishsim-isaac \
        bash /workspace/dishsim/frigidaire/scripts/experiment/run_initial_states.sh "$@"
fi
DISHSIM_USD_CANDIDATES=(/isaac-sim/extscache/omni.usd.libs-*/pxr)
if [[ ${#DISHSIM_USD_CANDIDATES[@]} != 1 || ! -d "${DISHSIM_USD_CANDIDATES[0]}" ]]; then
    echo 'Expected one bundled USD library in the pinned Isaac image.' >&2
    exit 1
fi
DISHSIM_USD_LIBRARY_ROOT="${DISHSIM_USD_CANDIDATES[0]%/pxr}"
export PYTHONPATH="$DISHSIM_USD_LIBRARY_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$DISHSIM_USD_LIBRARY_ROOT/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
DISHSIM_INITIAL_STATE_SCRIPT=frigidaire/scripts/experiment/frigidaire_initial_state_experiment.py
if [[ "${1:-}" == --resume ]]; then
    shift
    DISHSIM_INITIAL_STATE_SCRIPT=frigidaire/scripts/experiment/frigidaire_initial_state_resume.py
fi
exec /workspace/dishsim/scripts/run_py.sh "$DISHSIM_INITIAL_STATE_SCRIPT" "$@"
