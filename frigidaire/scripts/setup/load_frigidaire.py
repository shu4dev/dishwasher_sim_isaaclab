#!/usr/bin/env python3
"""Plan a fixed-size Frigidaire mixed load using the authored contact geometry."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR
from dishsim_frigidaire.usd_bootstrap import ensure_usd
ensure_usd()
from dishsim_frigidaire.loading import plan_full_load

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--asset-dir", type=Path, default=ASSET_DIR)
parser.add_argument("--out", type=Path)
parser.add_argument("--ban-file", type=Path, help="JSON list of physics-rejected candidate keys")
args = parser.parse_args()
banned = json.loads(args.ban_file.read_text()) if args.ban_file else []
report = plan_full_load(args.asset_dir, args.out or args.asset_dir / "full_load_manifest.json", banned)
print(json.dumps({"counts": report["counts"], "total": len(report["objects"]),
                  "candidates": report["packing"]["candidate_count"],
                  "status": report["packing"]["status"]}, indent=2), flush=True)
