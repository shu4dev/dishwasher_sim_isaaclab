#!/usr/bin/env python3
"""Part B goal loads (plans/2026-10-26-placing-goal.md): three single-kind subsets of certified sampler goals ->
data/results/robot/placing_v1/<load>/load.json; ``--summarize`` writes each load's summary.json from its dishes.jsonl
and subset_settle.json. Stdlib only (runs on the Mac).

    python3 code/execution/frigidaire/frigidaire_placing_loads.py [--summarize]

Goal pose = the goal record's ``targets_world`` (the sequentially built, settled poses; base frame, metres, Z-up,
XYZW). Order = the certify witness order restricted to the subset; edges = the graph's arrows within the subset.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "data/results/robot/placing_v1"
LOADS = {"easy_bowls": ("rest_easy", 5, "bowl"), "easy_plates": ("bp_medium", 5, "plate"), "easy_cups": ("rest_hard", 2, "cup")}
GRASP = {"bowl": "rim pinch", "plate": "idealized rim hold", "cup": "wall pinch"}
STEPS = ("R1", "R2", "S1", "S2", "S3", "S4", "S5", "S6")


def build(name, tier, seed, kind):
    pair_rel = f"data/results/sampler/{tier}/frigidaire_fdpc4221as__hotec_wheatstraw__{seed}.json"
    pair = json.loads((ROOT / pair_rel).read_text())
    rec = json.loads((ROOT / pair["goal"]["record"]).read_text())
    order = [d for d in pair["goal"]["certify"]["order"] if d.startswith(kind + "_")]
    assert order == [d for d in rec["witness"]["order"] if d.startswith(kind + "_")], "witness orders disagree"
    meta = {e["id"]: e for e in rec["entries_settled"]}
    dishes = []
    for k, d in enumerate(order):
        p = rec["targets_world"][d]
        dishes.append({"name": d, "kind": kind, "grasp": GRASP[kind], "rack": meta[d]["rack"], "slot": meta[d]["slot"],
                       "variant": meta[d]["variant"], "order_index": k,
                       "goal_pose": {"pos": p["position_m"], "quat_xyzw": p["quaternion_xyzw"]}})
    sub, edges = set(order), []
    for e in rec["graph"]["edges"]:
        if e["a"] in sub and e["b"] in sub and [e["a"], e["b"]] not in edges:
            edges.append([e["a"], e["b"]])
    load = {"load": name, "source_pair": pair_rel, "goal_record": pair["goal"]["record"], "kind": kind,
            "pose_convention": "base frame (the twin's world), metres, Z-up, quaternion XYZW; targets_world of the goal record",
            "frames": rec["frames"], "dishes": dishes, "order": order, "edges": edges}
    (OUT / name).mkdir(parents=True, exist_ok=True)
    (OUT / name / "load.json").write_text(json.dumps(load, indent=1) + "\n")
    print(f"{name}: {len(dishes)} {kind}s, order {order}, edges {edges}, racks {sorted({d['rack'] for d in dishes})}")


def summarize(name):
    d = OUT / name
    rows = {}
    if (d / "dishes.jsonl").exists():
        for line in (d / "dishes.jsonl").read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                rows[(r["dish"], r["variant"])] = r              # a rerun's row replaces the earlier one
    load = json.loads((d / "load.json").read_text())
    counts = lambda v: {s: sum(r["failed_at"] == s for (_, vv), r in rows.items() if vv == v) for s in STEPS
                        if any(r["failed_at"] == s for (_, vv), r in rows.items() if vv == v)}
    ss = json.loads((d / "subset_settle.json").read_text()) if (d / "subset_settle.json").exists() else None
    out = {"load": name, "n_dishes": len(load["dishes"]),
           "placed_W": sum(r["failed_at"] is None for (_, v), r in rows.items() if v == "W"),
           "placed_R": sum(r["failed_at"] is None for (_, v), r in rows.items() if v == "R"),
           "placed_R_ideal": sum(r["failed_at"] is None for (_, v), r in rows.items() if v == "R-ideal"),
           "placed_S": sum(r["failed_at"] is None for (_, v), r in rows.items() if v == "S"),
           "rows_W": sum(v == "W" for _, v in rows), "rows_R": sum(v == "R" for _, v in rows),
           "rows_R_ideal": sum(v == "R-ideal" for _, v in rows), "rows_S": sum(v == "S" for _, v in rows),
           "failed_at_counts": counts("W"), "failed_at_counts_R": counts("R"), "failed_at_counts_R_ideal": counts("R-ideal"),
           "failed_at_counts_S": counts("S"),
           "subset_settle": None if ss is None else {k: ss[k] for k in ("ok", "max_drift_mm", "max_drift_deg")}}
    (d / "summary.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--summarize", action="store_true")
    a = ap.parse_args()
    for name, spec in LOADS.items():
        summarize(name) if a.summarize else build(name, *spec)


if __name__ == "__main__":
    main()
