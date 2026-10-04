#!/usr/bin/env python3
"""Kit-free S-versus-budget sweep of the track-B MCTS planner (frigidaire_bench.plan_open "mcts").

    code/util/run_py.sh code/planner/frigidaire/frigidaire_mcts_sweep.py --instances medium_s0 hard_s1 --budgets 180 600

One JSON per (instance, budget) under data/results/benchmark/frigidaire_hotec/mcts_budget_sweep/: planned S (full
resolution), first-fit's planned S on the same instance, the search statistics, wall and CPU time. Plans are
not written and nothing is certified in Isaac: this measures what more search buys on paper. The 60 s point
is the benchmark's own plan (plans/open/<tier>/<id>__mcts.a0.json).
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--instances", nargs="+", required=True)
    ap.add_argument("--budgets", nargs="+", type=float, default=[180., 600.])
    args = ap.parse_args()
    spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "code/planner/frigidaire/frigidaire_bench.py")
    B = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(B)
    out = B.OUT / "mcts_budget_sweep"
    out.mkdir(parents=True, exist_ok=True)
    fam, parts, points = B.families()
    masks = B.family_masks(fam, parts, points)
    scorer = B.Scorer()
    for iid in args.instances:
        inst = json.loads((B.OUT / "instances" / iid.split("_")[0] / f"{iid}.json").read_text())
        ff = B.plan_open(inst, "baseline", scorer, fam, masks, parts, points, seed=0)
        s_ff = scorer.score(ff["goal"])["score"] if ff["goal"] else None
        for budget in args.budgets:
            path = out / f"{iid}__{int(budget)}s.json"
            if path.exists():
                print(f"[INFO] {path.name} exists", flush=True)
                continue
            seed = int(hashlib.sha256(f"0|{iid}|mcts".encode()).hexdigest()[:8], 16)     # the a0 plan's seed
            t0, c0 = time.monotonic(), time.process_time()
            plan = B.plan_open(inst, "mcts", scorer, fam, masks, parts, points, budget_s=budget, seed=seed)
            rec = {"instance": iid, "budget_s": budget, "S_planned": plan["S_planned"], "S_first_fit": s_ff,
                   "sequenced": plan["sequenced"], "moves": len(plan["moves"]), "wall_s": time.monotonic() - t0,
                   "cpu_s": time.process_time() - c0,
                   "search": {k: v for k, v in (plan.get("search") or {}).items() if k != "history"},
                   "history": (plan.get("search") or {}).get("history")}
            path.write_text(json.dumps(rec, indent=1) + "\n")
            print(f"[INFO] {iid} {budget:.0f} s: S {plan['S_planned']} (first-fit {s_ff}), "
                  f"{rec['search'].get('simulations')} simulations", flush=True)
    print("[RESULT] PASS mcts sweep", flush=True)


if __name__ == "__main__":
    main()
