#!/usr/bin/env python3
"""Per tier the valid goal load with the highest S (plans/2026-10-23-best-score-sample-per-tier.md; Kit-free, stdlib).

Candidates: every pair <root>/{rest,bp,sz}_<tier>/<machine>__<dishes>__<seed>.json whose goal.certify.valid is true,
deduplicated by the goal record's entries_chosen (ids and positions): a packing found in several rounds counts once, as
the earliest round's pair (ROUNDS order). Per candidate: S and surface reached (S_area) of the built state (the "settled"
state of <load_id>.relative.json beside the goal record, frigidaire_dot_rules_relative.py; None until scored), its
pooling dishes, the goal record's support and access edges. Sorted by S, then surface reached, then the lower seed; the
first is the pick. Writes <out> (default <root>/picks.json) and prints a table per tier.

    python3 code/src/sampler/pick.py
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ROUNDS = ("rest", "bp", "sz")                   # oldest first: a duplicate keeps the earliest round's pair
TIERS = ("easy", "medium", "hard")


def rel(p):
    try:
        return str(Path(p).resolve().relative_to(ROOT))
    except ValueError:
        return str(p)


def candidates(root, tier):
    """[row] of the distinct valid packings of one tier (module docstring), unsorted; rows carry 'duplicates'."""
    seen = {}
    for rnd in ROUNDS:
        pairs = sorted((root / f"{rnd}_{tier}").glob("*__*__*.json"), key=lambda p: int(p.stem.rsplit("__", 1)[1]))
        for path in pairs:
            pair = json.loads(path.read_text())
            goal = pair.get("goal") or {}
            if not (goal.get("certify") or {}).get("valid"):
                continue
            rec_path = ROOT / goal["record"]
            rec = json.loads(rec_path.read_text())
            key = tuple(sorted((e["id"], tuple(e["position"])) for e in rec["entries_chosen"]))
            if key in seen:
                seen[key]["duplicates"].append(f"{rnd}_{tier} seed {pair['seed']}")
                continue
            r = json.loads((rec_path.parent / f"{goal['load_id']}.relative.json").read_text()) \
                if (rec_path.parent / f"{goal['load_id']}.relative.json").is_file() else {}
            st = (r.get("states") or {}).get("settled") or {}
            done = {x["stage"] for x in pair["stages"] if x["result"] == "PASS"}
            n = rec["graph"]["counts"]
            seen[key] = {"tier": tier, "round": rnd, "seed": pair["seed"], "pair": rel(path), "record": goal["record"],
                         "load_id": goal["load_id"], "dishes": len(rec["entries_chosen"]),
                         "scored": (rec_path.parent / f"{goal['load_id']}.scores.json").is_file(),
                         "S": st.get("S"), "S_area": st.get("S_area"), "S_by_kind": st.get("S_by_kind"),
                         "n_pooling": st.get("n_pooling"), "pooling": st.get("pooling"),
                         "support": n["support"], "access": n["access"],
                         "complete": {"initial", "score", "graph"} <= done, "duplicates": []}
    return list(seen.values())


def ranked(rows):
    """S high to low (unscored last), then surface reached, then the lower seed, then the earlier round."""
    return sorted(rows, key=lambda c: (c["S"] is None, -(c["S"] or 0.), -(c["S_area"] or 0.), c["seed"],
                                       ROUNDS.index(c["round"])))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", type=Path, default=ROOT / "data/results/sampler")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    out = {"utc": datetime.now(timezone.utc).isoformat(timespec="seconds"), "rounds": list(ROUNDS),
           "rule": "highest S of the built state (kind ceilings, a pooling dish 0); tie: surface reached, lower seed",
           "tiers": {}}
    unscored = []
    f = lambda v: "-" if v is None else f"{v:.4f}"                 # noqa: E731
    for tier in TIERS:
        rows = ranked(candidates(a.root, tier))
        out["tiers"][tier] = {"n": len(rows), "pick": rows[0] if rows else None, "candidates": rows}
        unscored += [c["pair"] for c in rows if c["S"] is None]
        print(f"\n{tier}: {len(rows)} candidates\n| # | seed | round | S | surface reached | pooling | edges (support + access) "
              f"| complete | also in |\n|---|---|---|---|---|---|---|---|---|")
        for k, c in enumerate(rows, 1):
            print(f"| {k} | {c['seed']} | {c['round']} | {f(c['S'])} | {f(c['S_area'])} | {c['n_pooling']} | "
                  f"{c['support']} + {c['access']} | {'yes' if c['complete'] else 'no'} | "
                  f"{', '.join(c['duplicates']) or '-'} |")
    path = a.out or a.root / "picks.json"
    path.write_text(json.dumps(out, indent=1) + "\n")
    picks = ", ".join(f"{t} {p['round']}_{t} seed {p['seed']} S {f(p['S'])} of {v['n']}"
                      for t, v in out["tiers"].items() if (p := v["pick"]))
    print(f"\n[RESULT] {'PARTIAL' if unscored else 'PASS'} picks {rel(path)}: {picks}"
          + (f"; unscored {len(unscored)}: {' '.join(unscored)}" if unscored else ""))


if __name__ == "__main__":
    main()
