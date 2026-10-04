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
# which this script converts to version-agnostic text usda via a THROWAWAY usd-core venv on
# the HOST (never into Kit's site-packages; inside the container usd-core cannot load its own
# plugins next to Kit's python, measured 2026-09-29). Idempotent; run from the project root
# on the host:
#
#   USD_CORE_VERSION=<x.y.z> frigidaire/scripts/setup/mirror_robot_usd.sh
#
# Plan 2026-09-29-easy-s0, decision D11: the usd-core version is PINNED by the caller (the
# version that produced the mirrored layers on 2026-09-29 was not recorded and cannot be
# re-derived without a re-download, which D11 forbids; the script refuses to guess), the venv
# lives on the 2 TB drive (TMPDIR), the version used is written to assets/robots/CONVERSION.json,
# and assets/robots/MANIFEST.sha256 (robot_asset_manifest.py) records every file's sha256 --
# regenerate it after any conversion. "Diff against the asset in use" = the Kit-side prim-stack
# dump of /World/Robot/Gripper (frigidaire_grip_colliders.py pattern), not a byte diff.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
B="https://omniverse-content-production.s3-us-west-2.amazonaws.com"
DEST="$ROOT/assets/robots"
export TMPDIR="${TMPDIR:-/media/corallab-s1/2tbhdd/brianshu/dishsim/tmp}"   # the root disk gains nothing
mkdir -p "$TMPDIR"

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

if [ -z "${USD_CORE_VERSION:-}" ]; then
    echo "[RESULT] FAIL set USD_CORE_VERSION=<x.y.z> (D11: the conversion runs only with a pinned usd-core)"
    exit 1
fi
echo "[INFO] converting crate-0.9 layers to text usda (throwaway usd-core==$USD_CORE_VERSION venv on the HOST under $TMPDIR, deleted after)"
# 2026-09-29: inside the container usd-core (24.05, 24.11, 25.05) failed to load its own usd plugin
# ("libusd_kind-*.so: cannot open shared object file") next to Kit's python; the host's python3 works.
TOOL="$(mktemp -d)/usdtool"
python3 -m venv "$TOOL" >/dev/null
"$TOOL/bin/pip" install -q "usd-core==$USD_CORE_VERSION"
"$TOOL/bin/python" - "$DEST" "$USD_CORE_VERSION" <<'EOF'
import json, shutil, sys
from datetime import datetime, timezone
from pxr import Sdf
root = sys.argv[1] + "/Assets/Isaac/6.0/Isaac/Robots/"
done = []
for p in (root + "Robotiq/2F-85/Robotiq_2F_85_edit.usd", root + "UniversalRobots/ur5e/configuration/ur5e_robot_schema.usd"):
    with open(p, "rb") as f:
        if f.read(8) != b"PXR-USDC":
            print("already text:", p); continue
    layer = Sdf.Layer.FindOrOpen(p); assert layer, p
    tmp = p + ".usda_tmp"
    assert layer.Export(tmp, args={"format": "usda"})
    shutil.move(tmp, p)
    done.append(p[len(sys.argv[1]) + 1:])
    print("converted:", p)
with open(sys.argv[1] + "/CONVERSION.json", "w") as f:
    json.dump({"usd_core": sys.argv[2], "converted": done, "utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
               "note": "then run scripts/run_py.sh frigidaire/scripts/setup/robot_asset_manifest.py"}, f, indent=1)
EOF
rm -rf "$(dirname "$TOOL")"
echo "[INFO] now regenerate the manifest: scripts/run_py.sh frigidaire/scripts/setup/robot_asset_manifest.py"
echo "[RESULT] PASS"
