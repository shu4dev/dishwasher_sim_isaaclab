# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Run the exposure planner and the first-fit baseline on planner instances; collect the table; draw the method figure.

Kit-free (Warp on CUDA when available). Modes, in the order they are used:
  --prepare-pool                       geometric pool cache -> results/planner/frigidaire/pool_geometric.json
  --certify --instances GLOB           references (search best over the pool, organized state) and a
                                       sequenceability certificate written INTO each instance file
  --instances GLOB [--algorithms a,b]  episodes -> episodes/<inst>_<algo>.json and, when solved,
                                       episodes/<inst>_<algo>.manifest.json for the final Isaac gate
  --collect                            gates/<inst>_<algo>/result.json -> summary.csv, summary.md
  --figure INST                        method_<inst>.png: score over moves (planner vs baseline, references)
                                       and before/after exposure maps
"""
import argparse
import csv
import glob
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from dishsim import rearrange  # noqa: E402
from dishsim_frigidaire.paths import REPO_ROOT  # noqa: E402
from dishsim_frigidaire import exposure as E  # noqa: E402
from dishsim_frigidaire import planner as P  # noqa: E402

OUT = REPO_ROOT / "results/planner/frigidaire"
COLUMNS = ["instance", "seed", "n", "bowls", "mugs", "plates", "cutlery", "cutlery_start", "cap", "algorithm", "solved", "abort", "planned_feasible", "S", "W",
           "S_ref", "gap", "S_organized", "moves", "buffer_moves", "moves_lower_bound", "kept", "infeasible_cmds",
           "counter_full", "planning_s", "gate_outcome", "gate_s", "S_settled", "dS", "settle_disp_mm"]


def parse():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--prepare-pool", action="store_true")
    p.add_argument("--certify", action="store_true")
    p.add_argument("--instances", default=None, help="glob of planner instance files")
    p.add_argument("--algorithms", default="planner,baseline")
    p.add_argument("--budget-s", type=float, default=60.)
    p.add_argument("--collect", action="store_true")
    p.add_argument("--figure", default=None, help="instance id for method_<inst>.png")
    p.add_argument("--device", default=E.DEVICE)
    p.add_argument("--pool", type=Path, default=P.POOL_CACHE, help="geometric pool cache (v2 = bowls, mugs, plates, cutlery)")
    p.add_argument("--ban", nargs="*", default=[], help="candidate ids a gate rejected (the claims precedent): dropped from the pool before planning")
    p.add_argument("--out", type=Path, default=OUT)
    return p.parse_args()


def episode_seed(instance_id, name):
    return int(hashlib.sha256(f"{instance_id}:{name}".encode()).hexdigest()[:8], 16)


def run_instance(inst, name, pool, budget_s, device, out):
    world = P.PlannerWorld(inst)
    rinst = P.to_rearrange_instance(inst)
    oracle = P.GeometricOracle(inst, rinst, world, device)
    seed = episode_seed(inst["instance_id"], name)
    algo = P.ALGORITHMS[name](seed=seed, budget_s=budget_s, pool=pool, device=device)
    started = time.time()
    rec = rearrange.run_episode(rinst, algo, world, oracle, budget=None, algorithm_name=name, counter_cap=inst["counter"]["cap"])
    planned = P.score_objects(oracle.inside(), oracle.basket, device) if rec["solved"] else None
    goals = {it["item_id"]: {k: it["target"].get(k) for k in ("rack", "rack_local_pose", "candidate_id", "candidate_index", "keep")}
             for it in rinst.items if it["target"].get("T_base_obj") is not None}
    refs = inst.get("references", {})
    s_ref = (refs.get("reference") or {}).get("score")
    counter_start = sum(o["start"] == "Counter" for o in inst["objects"])
    kept = sum(bool(g.get("keep")) for g in goals.values())
    lower_bound = counter_start + (len(inst["objects"]) - counter_start - kept) if goals else None
    record = {**rec, "seed": seed, "goals": goals, "planned": planned, "banned": pool.get("banned", []),
              "reference_score": s_ref, "organized_score": (refs.get("organized") or {}).get("score"),
              "gap": (s_ref - planned["score"]) if (planned and s_ref is not None) else None,
              "moves_lower_bound": lower_bound, "kept": kept, "score_trace": oracle.trace,
              "initial_inside_score": P.score_objects(P.GeometricOracle(inst, rinst, world, device).inside(), oracle.basket, device)["score"],
              "wall_s": round(time.time() - started, 1)}
    (out / "episodes").mkdir(parents=True, exist_ok=True)
    stem = out / "episodes" / f"{inst['instance_id']}_{name}"
    Path(f"{stem}.json").write_text(json.dumps(record, indent=1, default=lambda v: v.tolist() if hasattr(v, "tolist") else str(v)) + "\n")
    if rec["solved"]:
        Path(f"{stem}.manifest.json").write_text(json.dumps(P.gate_manifest(inst, rinst), indent=1) + "\n")
    print(f"[OK] {inst['instance_id']} {name}: solved {rec['solved']} abort {rec['abort']} moves {rec['moves_used']} "
          f"(buffer {rec['buffer_moves']}, kept {kept}) S {planned['score'] if planned else float('nan'):.3f} "
          f"ref {s_ref if s_ref is not None else float('nan'):.3f} planning {rec['planning_time_total_s']:.1f}s")
    return record


def collect(out):
    rows = []
    for path in sorted((out / "episodes").glob("*.json")):
        if path.name.endswith(".manifest.json"):
            continue
        r = json.loads(path.read_text())
        inst = json.loads((out / "instances" / f"{r['instance']}.json").read_text())
        gate = out / "gates" / path.stem / "result.json"
        g = {"gate_outcome": "", "gate_s": "", "S_settled": "", "dS": "", "settle_disp_mm": ""}
        if gate.exists():
            result = json.loads(gate.read_text())
            g["gate_outcome"], g["gate_s"] = result.get("outcome"), round(result.get("wall_seconds", 0), 1)
            if result.get("outcome") == "accepted" and Path(f"{path.with_suffix('')}.manifest.json").exists():
                manifest = json.loads(Path(f"{path.with_suffix('')}.manifest.json").read_text())
                s = P.settled_score(result, manifest)
                g.update(S_settled=round(s["score"], 4), dS=round(abs(s["score"] - r["planned"]["score"]), 5),
                         settle_disp_mm=round(1e3 * s["max_settle_displacement_m"], 2))
        planned = r.get("planned") or {}
        refs = inst.get("references", {})                       # the instance's references are authoritative
        if (refs.get("reference") or {}).get("score") is not None:
            r["reference_score"] = refs["reference"]["score"]
            r["organized_score"] = (refs.get("organized") or {}).get("score")
            r["gap"] = r["reference_score"] - planned["score"] if planned else None
        rows.append({"instance": r["instance"], "seed": inst.get("seed"), "n": inst["n_objects"],
                     "bowls": inst["inventory"].get("bowl", 0), "mugs": inst["inventory"].get("mug", 0),
                     "plates": inst["inventory"].get("dinner_plate", 0),
                     "cutlery": sum(inst["inventory"].get(k, 0) for k in P.CUTLERY_KINDS), "cutlery_start": inst.get("cutlery_start", ""),
                     "cap": inst["counter"]["cap"], "algorithm": r["algorithm"], "solved": r["solved"], "abort": r["abort"] or "",
                     "planned_feasible": planned.get("feasible", ""), "S": round(planned.get("score", float("nan")), 4),
                     "W": round(planned.get("worst", float("nan")), 4),
                     "S_ref": round(r["reference_score"], 4) if r.get("reference_score") is not None else "",
                     "gap": round(r["gap"], 4) if r.get("gap") is not None else "",
                     "S_organized": round(r["organized_score"], 4) if r.get("organized_score") is not None else "",
                     "moves": r["moves_used"], "buffer_moves": r["buffer_moves"], "moves_lower_bound": r.get("moves_lower_bound", ""),
                     "kept": r.get("kept", 0), "infeasible_cmds": r["infeasible_commands"], "counter_full": r["counter_full_refusals"],
                     "planning_s": round(r["planning_time_total_s"], 1), **g})
    rows.sort(key=lambda x: (x["n"], x["seed"], x["algorithm"]))
    with (out / "summary.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        w.writerows(rows)
    lines = ["# Planner results (revision-5 exposure objective)", "",
             "| " + " | ".join(COLUMNS) + " |", "|" + "---|" * len(COLUMNS)]
    lines += ["| " + " | ".join(str(r[c]) for c in COLUMNS) + " |" for r in rows]
    by = {}
    for r in rows:
        by.setdefault(r["algorithm"], []).append(r)
    lines.append("")
    for name, rs in by.items():
        solved = [r for r in rs if r["solved"]]
        gates = [r for r in rs if r["gate_outcome"] == "accepted"]
        gaps = [r["gap"] for r in solved if r["gap"] != ""]
        lines.append(f"- **{name}**: solved {len(solved)}/{len(rs)}; gates accepted {len(gates)}/{len([r for r in rs if r['gate_outcome']])}; "
                     f"mean S {np.mean([r['S'] for r in solved]):.3f}; mean gap {np.mean(gaps) if gaps else float('nan'):.3f}; "
                     f"mean moves {np.mean([r['moves'] for r in solved]) if solved else float('nan'):.1f}; "
                     f"max |dS| {max([r['dS'] for r in gates if r['dS'] != ''], default=float('nan'))}")
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines[-len(by):]))
    print(f"[OK] wrote {out / 'summary.csv'} ({len(rows)} rows)")


def figure(out, instance_id, device):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from frigidaire_exposure_search import draw_top
    inst = json.loads((out / "instances" / f"{instance_id}.json").read_text())
    episodes = {name: json.loads(p.read_text()) for name in ("planner", "baseline")
                for p in [out / "episodes" / f"{instance_id}_{name}.json"] if p.exists()}
    basket = P.basket_in(inst["initial_snapshot"])
    fig, axes = plt.subplots(1, 3, figsize=(18, 6.2), gridspec_kw={"width_ratios": [1.25, 1, 1]}, constrained_layout=True)
    ax = axes[0]
    for name, color in (("planner", "tab:green"), ("baseline", "0.5")):
        if name not in episodes:
            continue
        r = episodes[name]
        ys = [r["initial_inside_score"]] + [m.get("score_after", np.nan) for m in r["moves"] if "score_after" in m]
        ax.plot(range(len(ys)), ys, marker="o", ms=3, color=color, label=f"{name}: {r['moves_used']} moves, S {r['planned']['score']:.3f}" if r["planned"] else name)
        for k, m in enumerate(r["moves"], 1):
            if m.get("kind") == "buffer":
                ax.plot(k, m.get("score_after", np.nan), marker="s", ms=6, color=color, mfc="white")
    r0 = next(iter(episodes.values()))
    if r0.get("reference_score") is not None:
        ax.axhline(r0["reference_score"], color="tab:orange", ls="--", label=f"search best (pool) {r0['reference_score']:.3f}")
    if r0.get("organized_score") is not None:
        ax.axhline(r0["organized_score"], color="tab:blue", ls=":", label=f"organized state {r0['organized_score']:.3f}")
    ax.set_xlabel("move"); ax.set_ylabel("exposure S of the objects inside"); ax.set_ylim(0, max(.45, ax.get_ylim()[1]))
    ax.set_title(f"{instance_id}: {inst['n_objects']} objects, {inst['counter']['start_count']} start on the counter, cap {inst['counter']['cap']}\n"
                 "score of the objects inside after each move (hollow square = parked on the counter)", fontsize=10)
    ax.legend(fontsize=8, loc="lower right")
    world = P.PlannerWorld(inst)
    rinst = P.to_rearrange_instance(inst)
    before = P.GeometricOracle(inst, rinst, world, device).inside()
    def result_of(objects, name):
        posed = [{"id": o["id"], "kind": o["kind"], "rack": o["rack"], **P.racks_in_pose(o["rack"], o["rack_local_pose"], basket)} for o in objects]
        return E.score_arrangement(E.Arrangement(name, "", "", posed, basket), device=device, baselines=False)
    rb = result_of(before, "before")
    sc = draw_top(axes[1], rb, f"start: {len(before)} inside, {inst['n_objects'] - len(before)} on the counter; S {rb['score']:.3f}\n"
                                "(red circle = holds water)")
    if "planner" in episodes and episodes["planner"]["planned"]:
        goals = episodes["planner"]["goals"]
        after = [{"id": i, "kind": next(o["kind"] for o in inst["objects"] if o["object_id"] == i), "rack": g["rack"], "rack_local_pose": g["rack_local_pose"]}
                 for i, g in goals.items()]
        ra = result_of(after, "after")
        sc = draw_top(axes[2], ra, f"planner's final load: all {len(after)} inside\nS {ra['score']:.3f}, W {ra['worst']:.3f}, "
                                    f"{sum(g.get('keep') for g in goals.values())} kept in place")
    fig.colorbar(sc, ax=axes[1:], shrink=.8, label="exposure")
    fig.suptitle("Arrangement planner over the revision-5 exposure objective: score along the move sequence, "
                 "and food-contact exposure maps before and after (racks in, top view)", fontsize=11)
    path = out / f"method_{instance_id}.png"
    fig.savefig(path, dpi=150); plt.close(fig)
    print(f"[OK] wrote {path}")


def main():
    args = parse()
    args.out.mkdir(parents=True, exist_ok=True)
    if args.prepare_pool:
        t = time.time()
        pool = P.build_geometric_pool()
        args.pool.parent.mkdir(parents=True, exist_ok=True)
        args.pool.write_text(json.dumps(pool) + "\n")
        print(f"[OK] pool: {len(pool['candidates'])} candidates {pool['counts_by_kind']}, {len(pool['conflict_pairs'])} edges "
              f"({pool['conflict_reasons']}), plates {pool['plates']}, cross-kind overlaps {pool['cross_kind_overlaps']} "
              f"in {time.time() - t:.1f}s -> {args.pool}")
    if args.instances:
        pool = P.load_pool(args.pool)
        if args.ban:
            keep = [i for i, c in enumerate(pool["candidates"]) if c["candidate_id"] not in set(args.ban)]
            remap = {old: new for new, old in enumerate(keep)}
            pool = {**pool, "candidates": [pool["candidates"][i] for i in keep],
                    "conflict_pairs": [[remap[a], remap[b]] for a, b in pool["conflict_pairs"] if a in remap and b in remap],
                    "banned": list(args.ban)}
            print(f"[OK] banned {len(args.ban)} candidates: {len(pool['candidates'])} remain")
        paths = sorted(glob.glob(str(args.instances)))
        if not paths:
            raise SystemExit(f"no instances match {args.instances}")
        for path in paths:
            inst = P.load_instance(path)
            if args.certify:
                c = P.certify(inst, pool, budget_s=args.budget_s, device=args.device)
                inst["references"] = {"reference": c["reference"], "organized": c["organized"], "certificate": c["certificate"],
                                      "pool_source": pool["source"], "seconds": c["seconds"]}
                Path(path).write_text(json.dumps(inst, indent=1) + "\n")
                print(f"[OK] {inst['instance_id']}: reference S {c['reference'].get('score', float('nan')):.3f} "
                      f"({c['reference'].get('proposals')} proposals), organized "
                      f"{(c['organized'] or {}).get('score', float('nan')):.3f}, sequenceable {c['certificate']['sequenceable']} "
                      f"({c['certificate']['moves']} moves), {c['seconds']} s")
            else:
                for name in args.algorithms.split(","):
                    run_instance(inst, name, pool, args.budget_s, args.device, args.out)
    if args.collect:
        collect(args.out)
    if args.figure:
        figure(args.out, args.figure, args.device)
    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
