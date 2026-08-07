#!/usr/bin/env bash
# Round-6 per-class driver: plate/bowl demos in the placement_open state (both racks out —
# the stowed upper rack sits over the rear tine bank), fork with the handle-gate exemption.
cd "$(dirname "$0")/.."
PY=/workspace/isaaclab/env_isaaclab/bin/python
KIT=scripts/run_kit.sh
mkdir -p logs

gate() {
  if grep -q "\[RESULT\] PASS" "$1"; then echo "[GATE OK] $2"; return 0
  else echo "[GATE FAIL] $2 (see $1)"; grep -E "\[FAIL\]|Error|assert" "$1" | head -3; return 1; fi
}

declare -A SLOTS=( [plate]="3,5" [bowl]="0,1" [fork]="0,1" )
declare -A SCN=( [plate]="placement_open" [bowl]="placement_open" [fork]="placement" )

for obj in plate bowl fork; do
  echo "===================== $obj (${SCN[$obj]}) ====================="
  ok=1
  $KIT scripts/12_extract_geometry.py --headless --object $obj --scenario "${SCN[$obj]}" \
    > logs/12_$obj.log 2>&1
  gate logs/12_$obj.log "$obj extraction" || { echo "[CLASS FAIL] $obj"; continue; }
  $PY scripts/13_decompose_meshes.py --object $obj --scenario "${SCN[$obj]}" > logs/13_$obj.log 2>&1
  gate logs/13_$obj.log "$obj decomposition" || { echo "[CLASS FAIL] $obj"; continue; }
  $KIT scripts/15_goal_configs.py --headless --enable_cameras --object $obj \
    --scenario "${SCN[$obj]}" > logs/15_$obj.log 2>&1
  gate logs/15_$obj.log "$obj goal configs" || { echo "[CLASS FAIL] $obj"; continue; }
  $KIT scripts/20_plan_and_place.py --headless --enable_cameras --object $obj \
    --scenario "${SCN[$obj]}" --slots "${SLOTS[$obj]}" --seeds 0 > logs/20_$obj.log 2>&1
  gate logs/20_$obj.log "$obj demo trials" || { echo "[CLASS FAIL] $obj"; continue; }
  grep "success=" logs/20_$obj.log
  echo "[CLASS OK] $obj"
done
echo "[DRIVER DONE]"
