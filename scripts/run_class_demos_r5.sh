#!/usr/bin/env bash
# Round-5 per-class driver: apertures frozen (data-derived), weld-carried families start
# welded; fork re-runs goals with the arch-offset drop points + per-piece cluster.
cd "$(dirname "$0")/.."
PY=/workspace/isaaclab/env_isaaclab/bin/python
KIT=scripts/run_kit.sh
mkdir -p logs

gate() {
  if grep -q "\[RESULT\] PASS" "$1"; then echo "[GATE OK] $2"; return 0
  else echo "[GATE FAIL] $2 (see $1)"; grep -E "\[FAIL\]|Error|assert" "$1" | head -3; return 1; fi
}

declare -A SLOTS=( [plate]="3,5" [bowl]="0,1" [cup]="2,7" [fork]="0,1" )
declare -A STAGES=( [plate]="15 20" [bowl]="15 20" [cup]="12 13 15 20" [fork]="12 13 15 20" )

for obj in plate bowl cup fork; do
  echo "===================== $obj ====================="
  ok=1
  for st in ${STAGES[$obj]}; do
    case $st in
      12) $KIT scripts/12_extract_geometry.py --headless --object $obj --scenario placement \
            > logs/12_$obj.log 2>&1
          gate logs/12_$obj.log "$obj extraction" || { ok=0; break; } ;;
      13) $PY scripts/13_decompose_meshes.py --object $obj --scenario placement \
            > logs/13_$obj.log 2>&1
          gate logs/13_$obj.log "$obj decomposition" || { ok=0; break; } ;;
      15) $KIT scripts/15_goal_configs.py --headless --enable_cameras --object $obj \
            > logs/15_$obj.log 2>&1
          gate logs/15_$obj.log "$obj goal configs" || { ok=0; break; } ;;
      20) $KIT scripts/20_plan_and_place.py --headless --enable_cameras --object $obj \
            --scenario placement --slots "${SLOTS[$obj]}" --seeds 0 > logs/20_$obj.log 2>&1
          gate logs/20_$obj.log "$obj demo trials" || { ok=0; break; }
          grep "success=" logs/20_$obj.log ;;
    esac
  done
  [ $ok -eq 1 ] && echo "[CLASS OK] $obj" || echo "[CLASS FAIL] $obj"
done
echo "[DRIVER DONE]"
