#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Method image for the HOTEC exposure search: per-sample exposure of the start load (run v8) and
the best load, top-down per rack, plus per-piece bars. Kit-free; re-scores both loads with samples kept.

    code/scripts/run_py.sh code/frigidaire/scripts/evaluation/frigidaire_hotec_exposure_figure.py \\
        --start data/results/hotec/frigidaire/v8/layout.json --best data/results/exposure/frigidaire/hotec/best_layout.json \\
        --out data/results/exposure/frigidaire/hotec/heatmap.png
"""
import argparse
import importlib.util
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
_spec = importlib.util.spec_from_file_location("hes", Path(__file__).with_name("frigidaire_hotec_exposure_search.py"))
HES = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(HES)
E = HES.E


def scored(layout, name, device, samples, directions):
    entries = HES.load_entries(layout)
    basket = (np.asarray(HES.BODY_POSITIONS["SilverwareBasket"], dtype=float), np.asarray(E.IDENTITY, dtype=float))
    return E.score_arrangement(E.Arrangement(name, "", "", HES.world_objects(entries), basket), device=device,
                               samples=samples, directions=directions, baselines=False)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--assets", type=Path, default=ROOT / "data/assets/models/hotec_wheatstraw/v2")
    p.add_argument("--start", type=Path, required=True)
    p.add_argument("--best", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--device", default=E.DEVICE)
    p.add_argument("--samples", type=int, default=E.DEFAULTS["samples_per_object"])
    p.add_argument("--directions", type=int, default=E.DEFAULTS["directions"])
    a = p.parse_args(argv)
    HES.register_hotec(a.assets)
    loads = [("run v8 (first-free planner)", json.loads(a.start.read_text())), ("best (exposure search)", json.loads(a.best.read_text()))]
    results = [scored(L, n, a.device, a.samples, a.directions) for n, L in loads]
    fig, axes = plt.subplots(2, 3, figsize=(16, 9), facecolor="white", gridspec_kw={"width_ratios": [1, 1, 1.15]})
    for row, ((name, _), res) in enumerate(zip(loads, results)):
        s = res["samples"]; owner = s["owner"]; racks = [o["rack"] for o in res["objects"]]
        for col, rack in enumerate(("LowerRack", "UpperRack")):
            ax = axes[row, col]
            mask = np.isin(owner, [i for i, r in enumerate(racks) if r == rack])
            pts, ex = s["points"][mask], s["exposure"][mask]
            order = np.argsort(ex)
            sc = ax.scatter(pts[order, 0] * 1000, pts[order, 1] * 1000, c=ex[order], s=2, cmap="viridis", vmin=0, vmax=1, rasterized=True)
            for wire in E.rack_wires(rack):
                w = wire + np.asarray(HES.BODY_POSITIONS[rack])
                ax.plot(w[:, 0] * 1000, w[:, 1] * 1000, color="#b8c0c8", lw=.4, zorder=0)
            ax.set_aspect("equal"); ax.set_xlim(-300, 300); ax.set_ylim(-300, 300)
            ax.set_title(f"{name}: {rack}, S {res['score']:.3f}", fontsize=10)
            ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm]" if col == 0 else "")
        ax = axes[row, 2]
        objs = sorted(res["objects"], key=lambda o: o["id"])
        ax.barh([o["id"] for o in objs], [o["exposure"] for o in objs],
                color=["#4c72b0" if o["kind"].endswith("plate") else "#dd8452" if o["kind"].endswith("bowl") else "#55a868" for o in objs])
        ax.set_xlim(0, 1); ax.set_xlabel("food-contact exposure (area-weighted mean over the piece)")
        ax.invert_yaxis(); ax.tick_params(axis="y", labelsize=7)
        ax.set_title(f"per piece; worst {res['worst']:.3f}", fontsize=10)
    fig.colorbar(sc, ax=axes[:, :2], shrink=.6, label="per-sample exposure (share of spray-arm rays that reach the surface)")
    fig.suptitle("HOTEC set in the FDPC4221AS: rev-5 spray-arm exposure of the food-contact surfaces, before and after the search", fontsize=12)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=130); plt.close(fig)
    print(f"[RESULT] PASS {a.out}: S {results[0]['score']:.4f} -> {results[1]['score']:.4f}")


if __name__ == "__main__":
    raise SystemExit(main())
