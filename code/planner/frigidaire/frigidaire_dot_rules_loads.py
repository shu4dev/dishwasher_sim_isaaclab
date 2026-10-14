#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Score settled 24-dish loads under the seven dot rules at one theta (Kit-free, GPU).

Per-rack arms (lower arm for LowerRack and basket-supported dishes, middle arm for UpperRack; G and J use the
dish's own arm disc), revision-5 occluders (every dish, both racks, the basket), settled poses.

    code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_loads.py --watch --N 262144 --theta 30 --until-utc 11:40
    code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_loads.py --in "data/results/random_loads/rl_20261009/loads/*/result.json" --N 65536

Input per load: a random-load result.json (racks + initial_snapshot), a top-5 settled layout (objects[].rack_local_pose)
or a racked_entries JSON ({"entries": [...]}). Output: data/results/dot_rules/loads/N{N:07d}/<load_id>.json and a
row in summary.csv; loads that already have an output are skipped.
"""
from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(Path(__file__).resolve().parent)]
import frigidaire_bench as B  # noqa: E402

RUN = ROOT / "data/results/random_loads/rl_20261009"
OUT = ROOT / "data/results/dot_rules/loads"
RULES = ("O", "D", "DG", "DJ", "N", "NG", "NJ")


def log(*a):
    print(f"[{dt.datetime.utcnow():%H:%M:%S}]", *a, flush=True)


def kind_of(oid, kind=None):
    k = kind or oid.rsplit("_", 1)[0]
    return k[6:] if k.startswith("hotec_") else k


def entries_of(data):
    """Scorer entries (rack-local, racks in) from any supported load record."""
    from dishsim_frigidaire.random_poses import relative_pose
    if "entries" in data:
        return [dict(e) for e in data["entries"]]
    if "initial_snapshot" in data and "racks" in data:          # random-load / gate result
        poses = data["initial_snapshot"]["poses"]
        kinds = {o["object_id"]: o.get("kind") for o in data.get("objects", [])}
        out = []
        for oid, rack in data["racks"].items():
            world = poses.get(oid) or next(o["pose_world"] for o in data["objects"] if o["object_id"] == oid)
            frame = poses[rack]
            p, q = relative_pose(world["position_m"], world["quaternion_xyzw"], frame["position_m"], frame["quaternion_xyzw"])
            out.append({"id": oid, "kind": kind_of(oid, kinds.get(oid)), "rack": rack, "position": [float(v) for v in p],
                        "quaternion_xyzw": [float(v) for v in q]})
        return out
    if "objects" in data:                                        # top-5 settled layout
        return [{"id": o["object_id"], "kind": kind_of(o["object_id"], o.get("kind")), "rack": o["rack"],
                 "position": o["rack_local_pose"]["position_m"], "quaternion_xyzw": o["rack_local_pose"]["quaternion_xyzw"]}
                for o in data["objects"]]
    raise ValueError("unknown load record")


def write_out(out_dir, rec):
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{rec['load_id']}.json"
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(rec, indent=1) + "\n")
    os.replace(tmp, path)
    csv = out_dir / "summary.csv"
    new = not csv.is_file()
    with open(csv, "a") as f:
        if new:
            f.write("load_id,N,theta_deg," + ",".join(f"S_{k}" for k in RULES) + "," + ",".join(f"W_{k}" for k in RULES)
                    + ",feasible,pooling_count,n_lower,n_upper,rays,seconds,scored_utc,source\n")
        racks = [d["rack"] for d in rec["per_dish"]]
        f.write(",".join(str(v) for v in [rec["load_id"], rec["N"], rec["theta_deg"]] + [rec["S"][k] for k in RULES]
                         + [rec["W"][k] for k in RULES] + [int(rec["feasible"]), rec["pooling_count"],
                                                           racks.count("LowerRack"), racks.count("UpperRack"),
                                                           int(rec["rays"]), round(rec["seconds"]["total"], 3),
                                                           rec["scored_utc"], rec["source"]]) + "\n")


def score_one(DR, eng, path, load_id, theta, out_dir, extra=None):
    data = json.loads(Path(path).read_text())
    if data.get("outcome") not in (None, "accepted"):
        log(f"skip {load_id}: outcome {data.get('outcome')}")
        return None
    entries = entries_of(data)
    rec = DR.score_load(eng, entries, theta, load_id)
    rec.update({"source": str(Path(path).resolve().relative_to(ROOT)) if str(path).startswith(str(ROOT)) else str(path),
                "scored_utc": dt.datetime.utcnow().isoformat() + "Z", "entries": entries, **(extra or {})})
    write_out(out_dir, rec)
    log(f"[RESULT] PASS {load_id}: S_O {rec['S']['O']:.4f} S_N {rec['S']['N']:.4f} S_NG {rec['S']['NG']:.4f} "
        f"S_NJ {rec['S']['NJ']:.4f}, pooling {rec['pooling_count']}, rays {rec['rays']:.3e}, "
        f"{rec['seconds']['total']:.1f}s (rays {rec['seconds']['rays']:.1f}, W* {rec['seconds']['wstar']:.1f}), "
        f"binds {rec['wstar']['binds']}, above1 {sum(rec['n_above_1'].values())}")
    return rec


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="inp", default=None, help="glob of load records (batch mode)")
    ap.add_argument("--watch", action="store_true", help="follow accepted.jsonl of the random-load run")
    ap.add_argument("--run", default=str(RUN))
    ap.add_argument("--poll", type=float, default=30.)
    ap.add_argument("--until-utc", default=None)
    ap.add_argument("--N", type=int, required=True)
    ap.add_argument("--theta", type=float, default=30.)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    out_dir = Path(args.out) if args.out else OUT / f"N{args.N:07d}"
    B.HX().register_hotec(B.ASSETS)
    from dishsim_frigidaire import dot_rules as DR
    eng = DR.DotRuleEngine(args.N)
    until = None
    if args.until_utc:
        h, m = (int(v) for v in args.until_utc.split(":"))
        until = dt.datetime.now(dt.timezone.utc).replace(hour=h, minute=m, second=0, microsecond=0).timestamp()
    if args.inp:
        for p in sorted(glob.glob(args.inp)):
            lid = Path(p).parent.name if Path(p).name == "result.json" else Path(p).stem
            if (out_dir / f"{lid}.json").is_file():
                continue
            score_one(DR, eng, p, lid, args.theta, out_dir)
    if not args.watch:
        log("[RESULT] PASS batch done")
        return
    run = Path(args.run)
    acc = run / "accepted.jsonl"
    n_done, times, failed = 0, [], {}
    log(f"watching {acc} until {args.until_utc}, N={args.N}, theta={args.theta}")
    while True:
        lines = acc.read_text().splitlines() if acc.is_file() else []
        todo = []
        for line in lines:
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not (out_dir / f"{r['load_id']}.json").is_file() and failed.get(r["load_id"], 0) < 2:
                todo.append(r)
        for r in todo:
            if until and time.time() > until:
                break
            p = ROOT / r["path"] if not os.path.isabs(r["path"]) else Path(r["path"])
            if not p.is_file():
                continue
            t0 = time.time()
            try:
                rec = score_one(DR, eng, p, r["load_id"], args.theta, out_dir,
                                {"accepted": {k: r.get(k) for k in ("finished_utc", "wall_s", "order")}})
            except Exception as exc:                              # keep watching; report the load (2 tries)
                failed[r["load_id"]] = failed.get(r["load_id"], 0) + 1
                log(f"[RESULT] FAIL {r['load_id']} (try {failed[r['load_id']]}): {type(exc).__name__}: {exc}")
                continue
            if rec is not None:
                n_done += 1
                times.append(time.time() - t0)
                backlog = len(todo) - todo.index(r) - 1
                log(f"scored {n_done} this session, mean {sum(times) / len(times):.1f}s/load, backlog {backlog} "
                    f"(~{backlog * sum(times) / len(times) / 60:.1f} min)")
        if until and time.time() > until:
            log("[RESULT] PASS watch stopped at --until-utc")
            return
        time.sleep(args.poll)


if __name__ == "__main__":
    main()
