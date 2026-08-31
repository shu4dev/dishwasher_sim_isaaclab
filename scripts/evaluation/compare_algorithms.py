# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Aggregate benchmark episode records into the comparison table — Kit-free.

Success rate is over COMPLETED episodes (init-mismatch is a harness reproduction artifact,
excluded from the denominator and reported in its own column).

Groups records by (cell, algorithm) — the cell comes from ``instance_meta["cell"]``
(authoritative; the per-cell directory layout is only for globbing), legacy flat records
aggregate as cell ``"legacy"``. Success rate counts EVERY record; the shape metrics
(moves, gap, planning time) aggregate over SOLVED episodes only — relocation distance is
retired for now (records still log raw ``travel_m``); the optimality gap
additionally requires the instance's cap-aware optimum to be proven
(``optimum_status == "solved"``). ``init-mismatch`` records are stubs with no metric
fields — tolerated and counted, never aggregated.

Run:
    scripts/run_py.sh scripts/evaluation/compare_algorithms.py
"""

import argparse
import csv
import glob
import json
import os
import statistics
from collections import defaultdict

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# preset-first ordering for the tables (hardcoded so this script needs no dishsim import)
_CELL_ORDER = ["easy", "medium", "hard", "med_count5", "med_count15", "med_types_bowls",
               "med_types_mix", "med_cap6", "med_cap1", "med_cap0", "med_disp_third",
               "med_disp_allcyc", "legacy"]

parser = argparse.ArgumentParser(description="Aggregate rearrangement episode records.")
parser.add_argument("--records", type=str, nargs="*",
                    default=["results/rearrange/bosch800/placement/*/*.json",
                             "results/rearrange/bosch800/placement/*.json"],
                    help="Record globs (relative to the project root unless absolute).")
parser.add_argument("--out", type=str, default=os.path.join("results", "compare"))
args = parser.parse_args()


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return round(statistics.fmean(xs), 3) if xs else None


def main() -> int:
    paths = sorted({p for g in args.records
                    for p in glob.glob(g if os.path.isabs(g)
                                       else os.path.join(PROJECT_ROOT, g))})
    if not paths:
        print(f"[FAIL] no records match {args.records}")
        return 1
    groups: dict = defaultdict(list)
    for p in paths:
        rec = json.load(open(p))
        cell = (rec.get("instance_meta") or {}).get("cell") or "legacy"
        groups[(cell, rec.get("algorithm", "?"))].append(rec)

    rows = []
    for (cell, algo), recs in groups.items():
        n = len(recs)
        init_mism = sum(r.get("abort") == "init-mismatch" for r in recs)
        completed = [r for r in recs if r.get("abort") != "init-mismatch"]
        solved = [r for r in completed if r.get("solved")]
        # gap only where the cap-aware optimum is PROVEN
        gapped = [r for r in solved
                  if (r.get("instance_meta") or {}).get("optimum_status") == "solved"]
        aborts = defaultdict(int)
        for r in completed:
            if r.get("abort"):
                aborts[r["abort"]] += 1
        plan_ms = [t * 1e3 for r in completed for t in r.get("planning_time_s", [])]
        meta0 = (solved or completed or recs)[0].get("instance_meta") or {}
        rows.append({
            "cell": cell, "algorithm": algo,
            "n": n, "n_solved": len(solved), "init_mismatch": init_mism,
            "success": round(len(solved) / len(completed), 3) if completed else None,
            "fraction_at_goal": _mean([r.get("fraction_at_goal") for r in completed]),
            "gap": _mean([r["moves_used"] - r["instance_meta"]["optimum"] for r in gapped]),
            "opt_ratio": _mean([r["moves_used"] / r["instance_meta"]["optimum"]
                                for r in gapped if r["instance_meta"]["optimum"]]),
            "no_opt": len(solved) - len(gapped),
            "moves": _mean([r.get("moves_used") for r in solved]),
            "moves_eff": _mean([r.get("moves_used", 0) - r.get("failed_settles", 0)
                                for r in solved]),
            "plan_total_s": _mean([r.get("planning_time_total_s") for r in solved]),
            "plan_ms_med": round(statistics.median(plan_ms), 3) if plan_ms else None,
            "infeasible": _mean([r.get("infeasible_commands") for r in completed]),
            "counter_full": _mean([r.get("counter_full_refusals") for r in completed]),
            "failed_settles": _mean([r.get("failed_settles") for r in completed]),
            "world_queries": _mean([r.get("world_queries") for r in completed]),
            "aborts": ",".join(f"{k}:{v}" for k, v in sorted(aborts.items())) or "-",
            "n_items": meta0.get("classes") and sum(meta0["classes"].values())
                       or (completed or recs)[0].get("n_items"),
            "counter_cap": meta0.get("counter_cap"),
            "displace_frac": meta0.get("displace_frac"),
        })

    def order(row):
        c = row["cell"]
        return (_CELL_ORDER.index(c) if c in _CELL_ORDER else len(_CELL_ORDER),
                c, row["algorithm"])

    rows.sort(key=order)
    out_dir = os.path.join(PROJECT_ROOT, args.out)
    os.makedirs(out_dir, exist_ok=True)
    cols = list(rows[0].keys())
    csv_path = os.path.join(out_dir, "summary.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    md_path = os.path.join(out_dir, "summary.md")
    with open(md_path, "w") as f:
        f.write("# Benchmark comparison\n\n"
                "Measurement names: success rate = `success` · planning time = "
                "`plan_total_s`/`plan_ms_med` · number of steps = `moves`. (Relocation "
                "distance is retired for now; records still log raw `travel_m`.)\n\n"
                "Success rate over ALL completed episodes; steps/gap/time over SOLVED only; gap "
                "only where the cap-aware optimum is proven. `moves_eff` excludes "
                "failed-settle retries. A null effect on the cap-ablation cells at medium "
                "displacement is expected signal (where the knob starts to bind). NEGATIVE "
                "gaps are real and documented: the optimum is a geometric-relaxation bound "
                "(finite lattice, commanded poses), while physics occasionally settles a "
                "neighbour into tolerance for free — a planner under physics can beat it.\n\n")
        f.write("| " + " | ".join(cols) + " |\n")
        f.write("|" + "---|" * len(cols) + "\n")
        for r in rows:
            f.write("| " + " | ".join("" if r[c] is None else str(r[c]) for c in cols) + " |\n")
    print(f"[INFO] {len(paths)} records -> {len(rows)} (cell, algorithm) groups")
    print(f"[INFO] wrote {os.path.relpath(csv_path, PROJECT_ROOT)}")
    print(f"[INFO] wrote {os.path.relpath(md_path, PROJECT_ROOT)}")
    print("[RESULT] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
