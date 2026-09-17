"""Generate one rack-capacity claim layout (FCL-checked) as a full-load manifest.

Kit-free (Isaac's bundled USD via ensure_usd). Output feeds
``frigidaire_full_load_evidence.py --manifest``.

    scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_claim_layouts.py --variant A
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR, REPO_ROOT  # noqa: E402
from dishsim_frigidaire.usd_bootstrap import ensure_usd  # noqa: E402

ensure_usd()
from dishsim_frigidaire import claims  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--asset-dir", type=Path, default=ASSET_DIR)
parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "build/frigidaire_collection/validation/claims")
parser.add_argument("--variant", choices=claims.VARIANTS, required=True)
parser.add_argument("--label", default=None, help="attempt name for the output files (default: variant)")
parser.add_argument("--ban-file", type=Path, default=None,
                    help="JSON list of candidate keys a physics run rejected; they take their next free variant")
args = parser.parse_args()

banned = json.loads(args.ban_file.read_text()) if args.ban_file else ()
manifest = claims.generate(args.asset_dir, args.variant, banned=banned)
label = args.label or args.variant
manifest_path, report_path = claims.write(manifest, args.out_dir, label)
report = claims.geometry_report(manifest)
print(f"[INFO] wrote {manifest_path} and {report_path}")
print(f"[INFO] by_rack {report['by_rack']}")
for item in report["unplaced"]:
    print(f"[FAIL] unplaced {item['item']} ({item['kind']}, {item['tried']} variants tried)")
print(f"[RESULT] {report['result']}: claim {label} geometry — {report['object_count']} objects, "
      f"upper bowls {report['upper_bowl_fill']}")
sys.exit(0 if report["result"] == "PASS" else 1)
