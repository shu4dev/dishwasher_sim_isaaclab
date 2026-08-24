#!/usr/bin/env bash
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
#
# Mirror the 6.0-bucket UR5e + Robotiq 2F-85 into assets/robots/ (the path
# dishsim.robots.UR5E_USD_PATH prefers) and make it readable by Isaac Sim 4.5.
#
# Why the 6.0 asset on a 4.5 runtime: the 4.5 bucket's ur5e.usd is arm-only — the
# pre-assembled Gripper=Robotiq_2f_85 variant (whose joint names, prim paths and calibrated
# apertures every measured number in this repo derives from) exists only in the 6.0 asset.
# All of its layers are crate 0.8.0 (readable by 4.5's USD 22.11) except two 0.9.0 layers,
# which this script converts to version-agnostic text usda via a THROWAWAY usd-core venv in
# the container (never into Kit's site-packages). Idempotent; run from the project root on
# the host:
#
#   scripts/setup/mirror_robot_usd.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
B="https://omniverse-content-production.s3-us-west-2.amazonaws.com"
DEST="$ROOT/assets/robots"

if [ -f "$DEST/Assets/Isaac/6.0/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd" ] \
   && head -1 "$DEST/Assets/Isaac/6.0/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd" 2>/dev/null | grep -q "#usda"; then
    echo "[INFO] robot mirror present and converted"
    exit 0
fi

echo "[INFO] mirroring 6.0 UR5e + Robotiq 2F-85 (~10 MB)"
for prefix in "Assets/Isaac/6.0/Isaac/Robots/UniversalRobots/ur5e/" "Assets/Isaac/6.0/Isaac/Robots/Robotiq/2F-85/"; do
    curl -s "$B/?list-type=2&prefix=$prefix&max-keys=300" \
        | grep -o "<Key>[^<]*</Key>" | sed 's/<\/*Key>//g' | grep -v ".thumbs" \
        | while read -r k; do
            mkdir -p "$DEST/$(dirname "$k")"
            curl -sf "$B/$k" -o "$DEST/$k"
          done
done

echo "[INFO] converting crate-0.9 layers to text usda (throwaway usd-core venv, in-container)"
docker exec dishsim-isaac bash -c '
/isaac-sim/kit/python/bin/python3 -m venv /tmp/usdtool 2>/dev/null || true
/tmp/usdtool/bin/pip install -q usd-core==24.05
/tmp/usdtool/bin/python - <<EOF
from pxr import Sdf
import shutil
for p in ("/workspace/dishsim/assets/robots/Assets/Isaac/6.0/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd",
          "/workspace/dishsim/assets/robots/Assets/Isaac/6.0/Isaac/Robots/UniversalRobots/ur5e/configuration/ur5e_robot_schema.usd"):
    with open(p, "rb") as f:
        if f.read(8) != b"PXR-USDC":
            print("already text:", p); continue
    l = Sdf.Layer.FindOrOpen(p); assert l, p
    tmp = p + ".usda_tmp"
    assert l.Export(tmp, args={"format": "usda"})
    shutil.move(tmp, p)
    print("converted:", p)
EOF'
echo "[RESULT] PASS"
