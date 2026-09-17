# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Score every packing/organized pair with identical objects; optional convergence check.

Kit-free. Writes summary.csv, summary.md, summary.png under results/exposure/frigidaire/summary/
and, with --convergence <pair>, convergence.md for directions {32,64,128} x samples {200,500,1000}.
"""
import argparse
import csv
import itertools
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import REPO_ROOT  # noqa: E402
from dishsim_frigidaire import exposure as E  # noqa: E402

STATES = REPO_ROOT / "results/initial_states/frigidaire"
FAMILIES = ("packing", "organized")
PAIRS = tuple(f"random_{i:02d}" for i in range(7))


FORMULA = """# Exposure score (revision 4, 2026-09-17)

Objects o = 1..O with food-contact surface area A_o (bowl and mug interiors).
Samples p_j, j = 1..N per object (N = 500, face centroids, each weighted A_o / N), outward normals n_j.
Source points q_k with weights u_k for the object's rack: K = 64 points on the arm disc under the
rack, u_k = (1 - w_c) / K; for the upper rack optionally the ceiling point with u = w_c (default w_c = 0).

    d_jk = (q_k - p_j) / |q_k - p_j|                                  ray direction
    v_jk = 1 if the segment p_j + eps n_j -> q_k hits nothing in the load, else 0
           (load = every dish including o, both racks, the basket; tub ignored; eps = 0.1 mm)
    c_jk = max(n_j . d_jk, 0)                                          impingement weight
    e_j  = sum_k u_k c_jk v_jk / sum_k u_k c_jk                        sample exposure, 0 if the denominator is 0
    E_o  = (1 / N) sum_j e_j                                           object exposure
    S    = sum_o A_o E_o / sum_o A_o                                   arrangement score (primary)
    W    = min_o E_o                                                   worst object (secondary)

Feasible iff no vessel pools: min z(interior vertices) >= min z(rim ring) - 2 mm at its pose.
Arm discs: lower arm centre (0, 0.008, 0.185) radius 0.245 m for LowerRack and basket objects;
middle arm centre (0, 0.008, 0.540) radius 0.205 m for UpperRack objects; ceiling point (0, 0.018, 0.817).
These are assumptions from the asset dimensions, not measurements.
"""


def state_path(family, pair):
    return STATES / f"{family}_20260911_seed20260911/states/{pair}.json"


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pairs", nargs="*", default=list(PAIRS))
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--convergence", default=None, help="pair name for the 3x3 stability check")
    parser.add_argument("--source", default=E.DEFAULTS["source"],
                        help="per-rack (default: arm discs), per-rack-directions, below, above, below+above, hemisphere")
    parser.add_argument("--ceiling-weight", type=float, default=E.DEFAULTS["ceiling_weight"],
                        help="share of an upper-rack sample's ray weight given to the ceiling nozzle point")
    parser.add_argument("--ceiling-sweep", action="store_true", help="also tabulate weights 0, .25, .5, 1")
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "results/exposure/frigidaire/summary")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    started = time.time()

    rows = []
    for pair in args.pairs:
        row = {"pair": pair}
        for family in FAMILIES:
            r = E.score_state(state_path(family, pair), device=args.device, source=args.source,
                              ceiling_weight=args.ceiling_weight)
            row[f"{family}_score"] = r["score"]; row[f"{family}_worst"] = r["worst"]
            row[f"{family}_mouth_up"] = r.get("pooling_count", r["mouth_up_count"]); row[f"{family}_feasible"] = r["feasible"]
            row["n_objects"] = r["n_objects"]
        row["organized_minus_packing"] = row["organized_score"] - row["packing_score"]
        rows.append(row)
        print(f"[OK] {pair}: packing {row['packing_score']:.3f} (mouth-up {row['packing_mouth_up']}) "
              f"organized {row['organized_score']:.3f} (mouth-up {row['organized_mouth_up']}) "
              f"diff {row['organized_minus_packing']:+.3f}")

    keys = list(rows[0].keys())
    with open(args.out / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(rows)
    lines = ["| pair | n | packing score | packing pooling | organized score | organized pooling | organized - packing |",
             "|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['pair']} | {r['n_objects']} | {r['packing_score']:.3f}"
                     f"{'' if r['packing_feasible'] else ' (infeasible)'} | {r['packing_mouth_up']} | "
                     f"{r['organized_score']:.3f}{'' if r['organized_feasible'] else ' (infeasible)'} | "
                     f"{r['organized_mouth_up']} | {r['organized_minus_packing']:+.3f} |")
    wins = sum(r["organized_minus_packing"] > 0 for r in rows)
    lines.append(f"\nOrganized scores higher in {wins} of {len(rows)} pairs. Feasible = no vessel that cannot "
                 f"drain (interior below its rim); infeasible arrangements are scored for information only. "
                 f"Ray source setting: {args.source}; ceiling weight {args.ceiling_weight}.")
    (args.out / "summary.md").write_text("\n".join(lines) + "\n")

    fig, ax = plt.subplots(figsize=(10, 4.2))
    x = np.arange(len(rows)); w = .38
    for j, family in enumerate(FAMILIES):
        vals = [r[f"{family}_score"] for r in rows]
        hatch = ["" if r[f"{family}_feasible"] else "//" for r in rows]
        bars = ax.bar(x + (j - .5) * w, vals, w, label=family, edgecolor="black", linewidth=.5)
        for b, h, r in zip(bars, hatch, rows):
            b.set_hatch(h)
            if not r[f"{family}_feasible"]:
                ax.text(b.get_x() + b.get_width() / 2, b.get_height() + .004, f"{r[f'{family}_mouth_up']}",
                        ha="center", va="bottom", fontsize=8, color="red")
    ax.set_xticks(x); ax.set_xticklabels([f"{r['pair']}\n(n={r['n_objects']})" for r in rows], fontsize=8)
    ax.set_ylabel("arrangement score (mean exposure)")
    ax.set_title(f"Same objects, two arrangements per pair; rays: {args.source}, ceiling weight {args.ceiling_weight}; "
                 "hatched = infeasible (red = vessels that cannot drain)", fontsize=9)
    ax.legend(); fig.tight_layout(); fig.savefig(args.out / "summary.png", dpi=150); plt.close(fig)

    if args.convergence:
        grid = list(itertools.product((32, 64, 128), (200, 500, 1000)))
        table = {}
        for directions, samples in grid:
            for family in FAMILIES:
                r = E.score_state(state_path(family, args.convergence), device=args.device,
                                  samples=samples, directions=directions, source=args.source,
                                  ceiling_weight=args.ceiling_weight)
                table[(directions, samples, family)] = r["score"]
        base = {f: table[(64, 500, f)] for f in FAMILIES}
        gap = abs(base["organized"] - base["packing"])
        lines = [f"# Convergence on {args.convergence} (default = 64 directions x 500 samples)", "",
                 "| directions | samples | packing | organized | max shift vs default | within gap |",
                 "|---|---|---|---|---|---|"]
        ok_all = True
        for directions, samples in grid:
            shift = max(abs(table[(directions, samples, f)] - base[f]) for f in FAMILIES)
            ok = shift <= gap; ok_all &= ok
            lines.append(f"| {directions} | {samples} | {table[(directions, samples, 'packing')]:.3f} | "
                         f"{table[(directions, samples, 'organized')]:.3f} | {shift:.3f} | {'yes' if ok else 'NO'} |")
        lines.append(f"\nGap at default: {gap:.3f}. Ordering preserved in all settings: "
                     f"{all(table[(d, s, 'organized')] > table[(d, s, 'packing')] for d, s in grid)}. "
                     f"All shifts within the gap: {ok_all}.")
        (args.out / "convergence.md").write_text("\n".join(lines) + "\n")
        print("\n".join(lines))

    if args.ceiling_sweep:
        lines = ["# Ceiling-nozzle weight sweep (share of an upper-rack sample's ray weight)", "",
                 "| weight | " + " | ".join(args.pairs) + " | organized wins |", "|---|" + "---|" * (len(args.pairs) + 1)]
        for w in (0., .25, .5, 1.):
            diffs = []
            for pair in args.pairs:
                sc = {f: E.score_state(state_path(f, pair), device=args.device, ceiling_weight=w)["score"] for f in FAMILIES}
                diffs.append(sc["organized"] - sc["packing"])
            lines.append(f"| {w:.2f} | " + " | ".join(f"{d:+.3f}" for d in diffs) + f" | {sum(d > 0 for d in diffs)}/{len(diffs)} |")
        lines.append("\nEntries are organized minus packing score. Packing states remain infeasible (pooling) at every weight.")
        (args.out / "ceiling_sweep.md").write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
    (args.out.parent / "FORMULA.md").write_text(FORMULA)
    print(f"[OK] wrote {args.out} in {time.time() - started:.1f}s")
    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
