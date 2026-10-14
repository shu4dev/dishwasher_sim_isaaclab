#!/usr/bin/env python3
"""Isaac layouts for the 5 best and 5 worst loads under New+J (frigidaire_hotec_load.py renders them).

    code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_isaac.py --set rand \
        --scores "data/results/dot_rules/loads/N0262144/*.json" --out data/media/dot_rules/isaac
"""
import argparse
import glob
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location("top5", ROOT / "code/planner/frigidaire/frigidaire_hotec_top5.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)


def s_nj(d):
    return d["S"]["NJ"] if "S" in d else d["S_NJ"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", required=True)
    ap.add_argument("--scores", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--n-total", type=int, default=None, help="loads ranked (when --scores holds only the rescored extremes)")
    ap.add_argument("--label", default=None)
    a = ap.parse_args()
    recs = [json.loads(Path(p).read_text()) for p in sorted(glob.glob(a.scores)) if not p.endswith("summary.csv")]
    recs.sort(key=s_nj, reverse=True)
    picks = [("top", i + 1, r) for i, r in enumerate(recs[:5])] + [("bottom", i + 1, r) for i, r in enumerate(recs[::-1][:5])]
    out = Path(a.out)
    rows = []
    for side, rank, r in picks:
        tag = f"{a.set}_{side}{rank}"
        ents = [{**e, "slot": e.get("slot", "random"), "variant": e.get("variant", "random")} for e in r["entries"]]
        lay = T.layout_of(ents)
        lay["priority"] = f"{a.set} {side} {rank} by New+J: {r['load_id']} S_NJ {s_nj(r):.4f}"
        T.atomic_write(out / tag / "layout.json", json.dumps(lay, indent=1) + "\n")
        rows.append({"tag": tag, "side": side, "rank": rank, "load_id": r["load_id"], "S_NJ": s_nj(r),
                     "n": a.n_total or len(recs), "label": a.label, "N": r.get("N")})
        print(tag, r["load_id"], f"{s_nj(r):.4f}", flush=True)
    T.atomic_write(out / f"{a.set}_picks.json", json.dumps(rows, indent=1) + "\n")
    print(f"[RESULT] PASS {len(rows)} layouts of {len(recs)} loads", flush=True)


if __name__ == "__main__":
    main()
