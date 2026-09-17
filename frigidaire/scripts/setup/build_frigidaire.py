#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Build the standalone FDPC4221AS without booting Isaac Sim.

    scripts/run_py.sh frigidaire/scripts/setup/build_frigidaire.py
"""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR

from dishsim_frigidaire.usd_bootstrap import ensure_usd
from dishsim_frigidaire.asset import build

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--out-dir",type=Path,default=ASSET_DIR)
parser.add_argument("--component",choices=["UpperRack", "LowerRack"],
                    help="Update one rack in an existing bundle (LowerRack also moves the basket mounting position)")
args=parser.parse_args()
ensure_usd()
report=build(args.out_dir,component=args.component)
print(json.dumps(report,indent=2),flush=True)
print("[RESULT] PASS: Frigidaire USD authoring (physics validation is separate)",flush=True)
