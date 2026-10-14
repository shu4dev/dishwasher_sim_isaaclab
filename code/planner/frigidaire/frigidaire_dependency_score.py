#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Score dependency-graph loads under the seven dot rules: packed, jointly settled and Isaac-built poses (Kit-free, GPU).

Reads <dependency-dir>/<load_id>.json (frigidaire_dependency_kit.py) and writes <load_id>.scores.json beside it:
{load_id, N, theta_deg, packed, settled, built}, each a dot_rules.score_load record (S and W per rule, per_dish,
pooling, feasible) of the load's entries (packed), its entries_settled (the joint gate's settled poses; null without
an accepted gate) and its built_entries (null when the load did not build).

    code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_score.py --N 262144
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(Path(__file__).resolve().parent)]
import frigidaire_bench as B  # noqa: E402

SHOWN = ("O", "N", "NJ")          # the page's three rules, in the [RESULT] line


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dependency-dir", type=Path, default=ROOT / "data/results/dot_rules/dependency/restricted")
    ap.add_argument("--loads", nargs="*", default=None, help="load ids (default: every <id>.json in the directory)")
    ap.add_argument("--N", type=int, default=262144)
    ap.add_argument("--theta", type=float, default=30.)
    ap.add_argument("--force", action="store_true", help="rescore loads that already have a .scores.json")
    args = ap.parse_args()
    d = args.dependency_dir
    ids = args.loads or sorted(p.name[:-len(".json")] for p in d.glob("*.json") if not p.name.endswith(".scores.json"))
    todo = [i for i in ids if args.force or not (d / f"{i}.scores.json").is_file()]
    print(f"{len(ids)} loads in {d}, {len(todo)} to score at N {args.N}, theta {args.theta}", flush=True)
    failed = []
    if todo:
        B.HX().register_hotec(B.ASSETS)
        from dishsim_frigidaire import dot_rules as DR
        eng = DR.DotRuleEngine(args.N)
    for lid in todo:
        try:
            dep = json.loads((d / f"{lid}.json").read_text())
            packed = DR.score_load(eng, dep["entries"], args.theta, f"{lid}_packed")
            settled = (DR.score_load(eng, dep["entries_settled"], args.theta, f"{lid}_settled")
                       if dep.get("entries_settled") else None)
            built = DR.score_load(eng, dep["built_entries"], args.theta, f"{lid}_built") if dep.get("built_entries") else None
            rec = {"load_id": lid, "N": args.N, "theta_deg": args.theta, "packed": packed, "settled": settled,
                   "built": built}
            path = d / f"{lid}.scores.json"
            tmp = path.with_name(path.name + ".tmp")
            tmp.write_text(json.dumps(rec, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
            os.replace(tmp, path)
            sides = (("packed", packed), ("settled", settled), ("built", built))
            line = ", ".join(f"S_{k} " + " ".join(f"{side} " + (f"{r['S'][k]:.4f}" if r else "-") for side, r in sides)
                             for k in SHOWN)
            print(f"[RESULT] PASS {lid} {line}", flush=True)
        except Exception as exc:                               # noqa: BLE001
            traceback.print_exc()
            failed.append(lid)
            print(f"[RESULT] FAIL {lid} {exc!r}", flush=True)
    print(f"[RESULT] {'FAIL' if failed else 'PASS'} scored {len(todo) - len(failed)} of {len(todo)} loads"
          + (f", failed {failed}" if failed else ""), flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
