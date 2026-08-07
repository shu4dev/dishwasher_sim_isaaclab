#!/usr/bin/env bash
# Per-class demo driver (Task 7): calibrate -> freeze -> extract -> decompose -> goals -> demo.
# Continues past a failing class (recorded); gates on log content, never exit codes.
cd "$(dirname "$0")/.."
PY=/workspace/isaaclab/env_isaaclab/bin/python
KIT=scripts/run_kit.sh
mkdir -p logs

gate() {  # gate <log> <tag>
  if grep -q "\[RESULT\] PASS" "$1"; then echo "[GATE OK] $2"; return 0
  else echo "[GATE FAIL] $2 (see $1)"; tail -5 "$1"; return 1; fi
}

# ---- per-class loop --------------------------------------------------------------------------
declare -A SLOTS=( [plate]="3,5" [bowl]="0,1" [cup]="2,7" [fork]="0,1" [tumbler]="2,7" )
declare -A TMAX=( [plate]="0.80" [bowl]="0.80" [cup]="0.30" [fork]="0.80" [tumbler]="0.34" )
declare -A TFORCE=( [plate]="5.0" [bowl]="5.0" [cup]="5.0" [fork]="3.0" [tumbler]="5.0" )
# thin-rim classes need a fine staircase: 0.01 rad steps slam a 5-6 mm rim from zero contact
# straight past the 30 N abort in one plateau
declare -A DTHETA=( [plate]="0.004" [bowl]="0.008" [cup]="0.008" [fork]="0.004" [tumbler]="0.01" )

for obj in plate bowl cup fork tumbler; do
  echo "===================== $obj ====================="
  frozen="$($PY -c "from dishsim import config; print(config.OBJECTS['$obj'].grasp.aperture_rad is not None)")"
  if [ "$frozen" = "True" ]; then
    echo "[INFO] $obj already calibrated — skipping scripts/11"
  else
    $KIT scripts/11_calibrate_grasp.py --headless --enable_cameras --object $obj \
      --rim_z "$($PY -c "from dishsim import config; print(config.OBJECTS['$obj'].grasp.rim_tcp_z_m)")" \
      --theta_max "${TMAX[$obj]}" --dtheta "${DTHETA[$obj]}" --target_force "${TFORCE[$obj]}" \
      > logs/11_$obj.log 2>&1
    gate logs/11_$obj.log "$obj calibration" || { echo "[SKIP] $obj (calibration)"; continue; }
    $PY scripts/freeze_calibration.py --object $obj >> logs/11_$obj.log 2>&1 \
      || { echo "[SKIP] $obj (freeze)"; continue; }
    grep "\[OK\] froze" logs/11_$obj.log
  fi

  $KIT scripts/12_extract_geometry.py --headless --object $obj --scenario placement \
    > logs/12_$obj.log 2>&1
  gate logs/12_$obj.log "$obj extraction" || { echo "[SKIP] $obj (extract)"; continue; }
  $PY scripts/13_decompose_meshes.py --object $obj --scenario placement > logs/13_$obj.log 2>&1
  gate logs/13_$obj.log "$obj decomposition" || { echo "[SKIP] $obj (decompose)"; continue; }
  $KIT scripts/15_goal_configs.py --headless --enable_cameras --object $obj \
    > logs/15_$obj.log 2>&1
  gate logs/15_$obj.log "$obj goal configs" || { echo "[SKIP] $obj (goals)"; continue; }
  $KIT scripts/20_plan_and_place.py --headless --enable_cameras --object $obj \
    --scenario placement --slots "${SLOTS[$obj]}" --seeds 0 > logs/20_$obj.log 2>&1
  gate logs/20_$obj.log "$obj demo trials" || { echo "[CLASS FAIL] $obj demo"; continue; }
  grep "success=" logs/20_$obj.log
done
echo "[DRIVER DONE]"
