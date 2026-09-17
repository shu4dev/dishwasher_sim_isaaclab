# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Sample other arrangements of the same objects and score them against the settled ones.

Kit-free. Reuses the organized run's saved candidate pool and compatibility graph. Two
samplers: random greedy (many diverse FCL-feasible sets) and MILP alternatives (each
solve excludes every previous solution). Samples are geometric proposals, NOT settled.
"""
import argparse
import json
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import REPO_ROOT  # noqa: E402
from dishsim_frigidaire import exposure as E  # noqa: E402

RUN = REPO_ROOT / "results/initial_states/frigidaire/organized_20260911_seed20260911"
STATES = REPO_ROOT / "results/initial_states/frigidaire"


def arrangement_from(catalog, indices, basket, name):
    objects = []
    for i in indices:
        c = catalog["candidates"][i]
        p, q = E.compose_pose(E.BODY_POSITIONS[c["rack"]], E.IDENTITY,
                              c["rack_local_pose"]["position_m"], c["rack_local_pose"]["quaternion_xyzw"])
        objects.append({"id": c["candidate_id"], "kind": c["kind"], "rack": c["rack"],
                        "position_m": p, "quaternion_xyzw": q, "candidate_index": int(i)})
    return E.Arrangement(name, "", "", objects, basket)


def random_greedy(catalog, allowed, adjacency, inventory, rng, attempts=50):
    for _ in range(attempts):
        chosen, counts = [], Counter()
        for i in rng.permutation(allowed):
            kind = catalog["candidates"][i]["kind"]
            if counts[kind] >= inventory.get(kind, 0) or adjacency[i] & set(chosen):
                continue
            chosen.append(int(i)); counts[kind] += 1
            if counts == inventory:
                return sorted(chosen)
    return None


def draw_top(ax, result, title):
    from matplotlib.patches import Circle
    s = result["samples"]
    order = np.argsort(s["points"][:, 2])
    sc = ax.scatter(s["points"][order, 0], s["points"][order, 1], c=s["exposure"][order], cmap="viridis",
                    vmin=0, vmax=1, s=3)
    for o in result["objects"]:
        if o.get("pools"):
            ax.add_patch(Circle(o["position_m"][:2], .05, fill=False, color="red", lw=1.2))
    ax.set_xlim(-.3, .3); ax.set_ylim(-.3, .3); ax.set_aspect("equal"); ax.set_title(title, fontsize=9)
    return sc


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from dishsim_frigidaire.organized_candidates import solve_inventory

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pair", default="random_06")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--random-seeds", type=int, default=100)
    parser.add_argument("--milp-rounds", type=int, default=10)
    parser.add_argument("--milp-limit", type=float, default=20.)
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "results/exposure/frigidaire/search")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    started = time.time()

    catalog = json.loads((RUN / "candidates_screened.json").read_text())
    graph = json.loads((RUN / "compatibility_screened.json").read_text())
    allowed = list(graph["allowed_indices"])
    adjacency = defaultdict(set)
    for a, b in graph["conflict_pairs"]:
        adjacency[a].add(b); adjacency[b].add(a)

    organized = E.load_state(STATES / f"organized_20260911_seed20260911/states/{args.pair}.json")
    inventory = Counter(o["kind"] for o in organized.objects)
    reference = {f: E.score_state(STATES / f"{f}_20260911_seed20260911/states/{args.pair}.json", device=args.device)
                 for f in ("packing", "organized")}
    print(f"[OK] inventory {dict(inventory)}; settled organized {reference['organized']['score']:.3f}, "
          f"packing {reference['packing']['score']:.3f}")

    samples, seen = [], set()
    rng = np.random.default_rng(20260917)
    for seed in range(args.random_seeds):
        chosen = random_greedy(catalog, allowed, adjacency, inventory, rng)
        if chosen is None or tuple(chosen) in seen:
            continue
        seen.add(tuple(chosen))
        samples.append(("random", seed, chosen))
    excluded = []
    for round_ in range(args.milp_rounds):
        sol = solve_inventory(catalog, graph, dict(inventory), seed=round_, time_limit_s=args.milp_limit,
                              excluded_sets=excluded)
        chosen = sol.get("selected_indices")
        if chosen is None:
            print(f"[WARN] milp round {round_}: {sol.get('geometric_feasibility')}"); break
        excluded.append(frozenset(chosen))
        if tuple(chosen) not in seen:
            seen.add(tuple(chosen)); samples.append(("milp", round_, chosen))
    print(f"[OK] {len(samples)} distinct feasible proposals ({sum(s[0] == 'random' for s in samples)} random, "
          f"{sum(s[0] == 'milp' for s in samples)} milp)")

    scored = []
    for method, seed, chosen in samples:
        r = E.score_arrangement(arrangement_from(catalog, chosen, organized.basket, f"{method}_{seed}"), device=args.device)
        scored.append({"method": method, "seed": seed, "indices": chosen, "score": r["score"], "worst": r["worst"],
                       "feasible": r["feasible"], "pooling_count": r["pooling_count"]})
    scored.sort(key=lambda r: -r["score"])
    scores = np.array([r["score"] for r in scored])
    (args.out / "samples.json").write_text(json.dumps({"pair": args.pair, "inventory": dict(inventory),
        "reference": {f: E.strip_samples(reference[f]) for f in reference}, "samples": scored}, indent=1) + "\n")

    org, pack = reference["organized"]["score"], reference["packing"]["score"]
    better = int((scores > org).sum())
    lines = [f"# Alternative arrangements for {args.pair} ({dict(inventory)})", "",
             f"{len(scored)} distinct FCL-feasible proposals from the organized candidate pool "
             f"({RUN.name}, {len(allowed)} candidates), scored with the revision-4 exposure.", "",
             "| | score |", "|---|---|",
             f"| settled organized state | {org:.3f} |", f"| settled packing state (infeasible) | {pack:.3f} |",
             f"| proposals: min | {scores.min():.3f} |", f"| proposals: median | {np.median(scores):.3f} |",
             f"| proposals: max | {scores.max():.3f} |", "",
             f"{better} of {len(scored)} proposals score above the settled organized state; "
             f"best is {scored[0]['method']} seed {scored[0]['seed']} at {scored[0]['score']:.3f} "
             f"(worst object {scored[0]['worst']:.3f}, feasible={scored[0]['feasible']}).", "",
             "Caveat: proposals are geometric (FCL clearance, no nesting, organized policy) and NOT physically "
             "settled; settle the best with frigidaire_full_load_evidence.py before quoting it."]
    (args.out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines[4:12]))

    fig, ax = plt.subplots(figsize=(8, 3.6))
    ax.hist(scores, bins=20, color="0.6", edgecolor="black")
    ax.axvline(org, color="tab:orange", lw=2, label=f"settled organized {org:.3f}")
    ax.axvline(pack, color="tab:blue", lw=2, ls="--", label=f"settled packing {pack:.3f} (infeasible)")
    ax.set_xlabel("arrangement score"); ax.set_ylabel("proposals"); ax.legend(fontsize=8)
    ax.set_title(f"{len(scored)} alternative arrangements of the same {sum(inventory.values())} objects", fontsize=10)
    fig.tight_layout(); fig.savefig(args.out / "hist.png", dpi=150); plt.close(fig)

    best = E.score_arrangement(arrangement_from(catalog, scored[0]["indices"], organized.basket, "best"), device=args.device)
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.4))
    draw_top(axes[0], reference["organized"], f"settled organized {args.pair}: score {org:.3f}")
    sc = draw_top(axes[1], best, f"best proposal ({scored[0]['method']} seed {scored[0]['seed']}): score {best['score']:.3f}, unsettled")
    fig.colorbar(sc, ax=axes, shrink=.8, label="exposure"); fig.savefig(args.out / "best.png", dpi=150); plt.close(fig)
    print(f"[OK] wrote {args.out} in {time.time() - started:.1f}s")
    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
