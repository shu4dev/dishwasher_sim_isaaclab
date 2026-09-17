#!/usr/bin/env bash
set -euo pipefail
if [[ ! -d /isaac-sim ]]; then
    exec docker exec -w /workspace/dishsim dishsim-isaac bash /workspace/dishsim/frigidaire/scripts/experiment/run_organized_states.sh "$@"
fi
DISHSIM_ORG_USD=(/isaac-sim/extscache/omni.usd.libs-*/pxr)
if [[ ${#DISHSIM_ORG_USD[@]} != 1 || ! -d "${DISHSIM_ORG_USD[0]}" ]]; then
    echo 'Expected the bundled USD library in the pinned Isaac image.' >&2
    exit 1
fi
DISHSIM_ORG_USD_ROOT="${DISHSIM_ORG_USD[0]%/pxr}"
export PYTHONPATH="$DISHSIM_ORG_USD_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$DISHSIM_ORG_USD_ROOT/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec /workspace/dishsim/scripts/run_py.sh frigidaire/scripts/experiment/frigidaire_organized_experiment.py "$@"
