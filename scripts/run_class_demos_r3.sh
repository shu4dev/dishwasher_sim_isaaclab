#!/usr/bin/env bash
# Round-3 per-class driver: plate/bowl/cup/fork with graded-staircase + grasp-point fixes
# (tumbler completed in round 2). Gates on log content, never exit codes.
cd "$(dirname "$0")/.."
PY=/workspace/isaaclab/env_isaaclab/bin/python
KIT=scripts/run_kit.sh
mkdir -p logs

gate() {
  if grep -q "\[RESULT\] PASS" "$1"; then echo "[GATE OK] $2"; return 0
  else echo "[GATE FAIL] $2 (see $1)"; grep "\[FAIL\]" "$1" | head -3; return 1; fi
}

declare -A SLOTS=( [plate]="3,5" [bowl]="0,1" [cup]="2,7" [fork]="0,1" )
declare -A TMAX=( [plate]="0.80" [bowl]="0.80" [cup]="0.30" [fork]="0.80" )
# thin stiff rims on a rigid weld spike from 0 to ~30 N inside one coarse plateau: grade the
# curve with fine steps + a higher spike allowance; softer targets stop the effort-limited
# drive from stalling deep past touch (the cup hit 20 N steady and shoved the weld 3.5 mm)
declare -A TFORCE=( [plate]="4.0" [bowl]="4.0" [cup]="3.0" [fork]="3.0" )
declare -A DTHETA=( [plate]="0.002" [bowl]="0.002" [cup]="0.008" [fork]="0.004" )
declare -A ABORT=( [plate]="60" [bowl]="60" [cup]="30" [fork]="60" )

for obj in plate bowl cup fork; do
  echo "===================== $obj ====================="
  frozen="$($PY -c "from dishsim import config; print(config.OBJECTS['$obj'].grasp.aperture_rad is not None)")"
  if [ "$frozen" = "True" ]; then
    echo "[INFO] $obj already calibrated — skipping scripts/11"
  else
    $KIT scripts/11_calibrate_grasp.py --headless --enable_cameras --object $obj \
      --rim_z "$($PY -c "from dishsim import config; print(config.OBJECTS['$obj'].grasp.rim_tcp_z_m)")" \
      --theta_max "${TMAX[$obj]}" --dtheta "${DTHETA[$obj]}" \
      --target_force "${TFORCE[$obj]}" --pad_force_abort "${ABORT[$obj]}" \
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
