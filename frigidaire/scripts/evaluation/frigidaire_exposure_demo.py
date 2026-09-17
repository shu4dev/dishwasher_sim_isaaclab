# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Score one packing/organized pair with the minimal AO exposure metric and draw it.

Kit-free. Writes sanity.png, pair.png, bars.png, turntable.gif, rays.gif (one spray arm
rotating under one bowl, rays green when open and red up to the hit point), scores.json and,
when Isaac stills of both states are given or found, evidence.png.
"""
import argparse
import json
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
CMAP, VMIN, VMAX = "viridis", 0., 1.


def parse():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pair", default="random_06")
    p.add_argument("--device", default="cpu")
    p.add_argument("--samples", type=int, default=E.DEFAULTS["samples_per_object"])
    p.add_argument("--directions", type=int, default=E.DEFAULTS["directions"])
    p.add_argument("--frames", type=int, default=72)
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--isaac-packing", type=Path, default=None, help="Isaac still of the packing state")
    p.add_argument("--isaac-organized", type=Path, default=None, help="Isaac still of the organized state")
    return p.parse_args()


def find_isaac(args, out):
    """Explicit paths, else the known render locations for this pair."""
    guesses = {"packing": args.isaac_packing or out / "isaac" / f"{args.pair}_initial.png",
               "organized": args.isaac_organized or (STATES / "organized_20260911_seed20260911/renders"
                                                     / f"{args.pair}_after.png")}
    return {k: v for k, v in guesses.items() if v is not None and Path(v).exists()}


def wire_segments(arrangement):
    """3D line segments of both racks and the basket in the racks-in pose."""
    segments = []
    for name in ("LowerRack", "UpperRack"):
        offset = np.asarray(E.BODY_POSITIONS[name])
        for path in E.rack_wires(name):
            p = path + offset
            segments.extend(np.stack((p[:-1], p[1:]), axis=1))
    pos, quat = arrangement.basket
    rot = E.quaternion_matrix_xyzw(quat)
    for path in E.rack_wires("SilverwareBasket"):
        p = path @ rot.T + np.asarray(pos)
        segments.extend(np.stack((p[:-1], p[1:]), axis=1))
    return np.asarray(segments)


def draw_3d(ax, result, segments, title, elev=32, azim=-50):
    from mpl_toolkits.mplot3d.art3d import Line3DCollection
    s = result["samples"]
    ax.add_collection3d(Line3DCollection(segments, colors="0.75", linewidths=.25, alpha=.6))
    sc = ax.scatter(s["points"][:, 0], s["points"][:, 1], s["points"][:, 2], c=s["exposure"],
                    cmap=CMAP, vmin=VMIN, vmax=VMAX, s=3, depthshade=False)
    for obj in result["objects"]:
        if obj.get("pools"):
            p = obj["position_m"]
            ax.plot([p[0]], [p[1]], [p[2] + .09], marker="v", color="red", markersize=7)
    ax.set_xlim(-.3, .3); ax.set_ylim(-.3, .3); ax.set_zlim(.15, .75)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.set_title(title, fontsize=10)
    return sc


def draw_top(ax, result, segments, title):
    from matplotlib.collections import LineCollection
    s = result["samples"]
    ax.add_collection(LineCollection(segments[:, :, :2], colors="0.8", linewidths=.25))
    order = np.argsort(s["points"][:, 2])
    sc = ax.scatter(s["points"][order, 0], s["points"][order, 1], c=s["exposure"][order],
                    cmap=CMAP, vmin=VMIN, vmax=VMAX, s=3)
    for obj in result["objects"]:
        if obj.get("pools"):
            ax.add_patch(__import__("matplotlib.patches", fromlist=["Circle"]).Circle(
                obj["position_m"][:2], .05, fill=False, color="red", lw=1.2))
    ax.set_xlim(-.3, .3); ax.set_ylim(-.3, .3); ax.set_aspect("equal")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m), front is -y"); ax.set_title(title, fontsize=10)
    return sc


def rays_gif(result, arrangement, segments, out, frames=72, device="cpu"):
    """Animate one spray arm under one bowl; rays green when open, red up to the hit point."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from mpl_toolkits.mplot3d.art3d import Line3DCollection
    objs = result["objects"]
    idx = next((i for i, o in enumerate(objs) if o["rack"] == "UpperRack" and o["kind"] == "bowl"), 0)
    s = result["samples"]; mask = s["owner"] == idx
    pts, nrm = s["points"][mask][::8], s["normals"][mask][::8]
    others = s["points"][~mask]
    soup = E.occluder_soup(arrangement)
    arm = E.ARM_SOURCES[E.source_for(objs[idx]["rack"], "per-rack")]
    center, radius = np.asarray(arm["center"]), arm["radius"]
    radii = np.linspace(.04, radius, 4)
    starts = pts + 1e-4 * nrm
    fig = plt.figure(figsize=(9, 7)); ax = fig.add_subplot(111, projection="3d")

    def frame(k):
        ax.cla()
        th = 2 * np.pi * k / frames
        u = np.array([np.cos(th), np.sin(th), 0.])
        q = np.array([center + r * u for r in radii] + [center - r * u for r in radii])
        d = q[None] - starts[:, None]; dist = np.linalg.norm(d, axis=2); dirs = d / dist[..., None]
        o = np.repeat(starts, len(q), 0); dv = dirs.reshape(-1, 3)
        hit, t = E.cast(soup, o, dv, device, dist.reshape(-1))
        segs = np.stack((o, o + dv * t[:, None]), axis=1)
        ax.add_collection3d(Line3DCollection(segments, colors="0.85", linewidths=.2))
        ax.scatter(others[:, 0], others[:, 1], others[:, 2], s=1, c="0.6")
        if (~hit).any():
            ax.add_collection3d(Line3DCollection(segs[~hit], colors="green", linewidths=.5, alpha=.7))
        if hit.any():
            ax.add_collection3d(Line3DCollection(segs[hit], colors="red", linewidths=.5, alpha=.7))
        tip = np.stack((center - radius * u, center + radius * u))
        ax.plot(tip[:, 0], tip[:, 1], tip[:, 2], color="black", lw=4)
        ax.scatter(q[:, 0], q[:, 1], q[:, 2], c="black", s=12)
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c="blue", s=8)
        ax.set_xlim(-.3, .3); ax.set_ylim(-.3, .3); ax.set_zlim(center[2] - .05, center[2] + .3)
        ax.set_box_aspect((1, 1, .6)); ax.view_init(elev=18, azim=-60)
        ax.set_title(f"{objs[idx]['id']}: arm at {np.degrees(th):5.1f} deg, open rays {(~hit).mean():.2f}; "
                     f"object exposure {objs[idx]['exposure']:.3f}", fontsize=9)
        return []
    FuncAnimation(fig, frame, frames=frames, blit=False).save(out / "rays.gif", writer=PillowWriter(fps=10), dpi=80)
    plt.close(fig)


def label(result):
    status = ("feasible" if result.get("feasible")
              else f"INFEASIBLE (pooling {result.get('pooling_count', 0)}/{result['n_objects']})")
    return f"{result['name']}\nscore {result['score']:.3f}, worst {result['worst']:.3f}, {status}"


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    args = parse()
    out = args.out or REPO_ROOT / "results/exposure/frigidaire" / f"demo_{args.pair}"
    out.mkdir(parents=True, exist_ok=True)
    started = time.time()

    # 1. sanity pair
    sanity = E.sanity_pair(args.device, args.samples, args.directions)
    fig = plt.figure(figsize=(9, 4.2))
    for i, key in enumerate(("alone", "covered")):
        ax = fig.add_subplot(1, 2, i + 1, projection="3d")
        s = sanity[key]["samples"]
        sc = ax.scatter(s["points"][:, 0], s["points"][:, 1], s["points"][:, 2], c=s["exposure"],
                        cmap=CMAP, vmin=VMIN, vmax=VMAX, s=5, depthshade=False)
        if key == "covered":
            t = np.linspace(0, 2 * np.pi, 80)
            z = sanity[key]["objects"][1]["position_m"][2] + .010   # plate top face
            ax.plot(.13 * np.cos(t), .13 * np.sin(t), z, color="0.4", lw=1.5)
        ax.set_xlim(-.14, .14); ax.set_ylim(-.13, .15); ax.set_zlim(.15, .30); ax.set_box_aspect((1, 1, .5))
        ax.view_init(elev=28, azim=-55)
        ax.set_title(f"bowl {key}: mean exposure {sanity[key]['exposure']:.3f}", fontsize=10)
    fig.colorbar(sc, ax=fig.axes, shrink=.7, label="exposure (share of open rays)")
    fig.suptitle("Sanity: same bowl mouth-down on the lower rack floor, alone vs. a plate 15 mm below its rim\n"
                 "(lower arm disc 30 mm below the rim)", fontsize=10)
    fig.savefig(out / "sanity.png", dpi=150); plt.close(fig)

    # 2. the pair
    results, arrangements = {}, {}
    for family in FAMILIES:
        path = STATES / f"{family}_20260911_seed20260911/states/{args.pair}.json"
        arr = E.load_state(path)
        results[family] = E.score_state(path, device=args.device, samples=args.samples, directions=args.directions)
        results[family]["name"] = f"{family} {args.pair}"
        arrangements[family] = arr
        print("[OK] " + label(results[family]).replace("\n", ": "))
    segments = {f: wire_segments(arrangements[f]) for f in FAMILIES}

    fig = plt.figure(figsize=(12, 10.5))
    for i, family in enumerate(FAMILIES):
        sc = draw_top(fig.add_subplot(2, 2, i + 1), results[family], segments[family], label(results[family]))
        draw_3d(fig.add_subplot(2, 2, i + 3, projection="3d"), results[family], segments[family], "oblique")
    fig.colorbar(sc, ax=fig.axes, shrink=.6, label="exposure (share of open rays)")
    fig.suptitle(f"Exposure of food-contact surfaces, same {results['packing']['n_objects']} objects; "
                 "sources = arm discs under each rack, cosine-weighted; red = cannot drain", fontsize=11)
    fig.savefig(out / "pair.png", dpi=150); plt.close(fig)

    # 2b. Isaac stills above the exposure maps, when both exist
    isaac = find_isaac(args, out)
    if len(isaac) == 2:
        fig = plt.figure(figsize=(12, 11), constrained_layout=True)
        for i, family in enumerate(FAMILIES):
            ax = fig.add_subplot(2, 2, i + 1)
            ax.imshow(plt.imread(isaac[family])); ax.axis("off")
            ax.set_title(f"{family} {args.pair}: Isaac RTX, settled state (racks out)", fontsize=10)
            sc = draw_top(fig.add_subplot(2, 2, i + 3), results[family], segments[family], label(results[family]))
        fig.colorbar(sc, ax=fig.axes[2:], shrink=.8, label="exposure (share of open rays)")
        fig.suptitle("Same objects, two arrangements: Isaac render (top) and exposure map, racks in (bottom); "
                     "water from the arm discs under each rack; red = cannot drain", fontsize=11)
        fig.savefig(out / "evidence.png", dpi=150); plt.close(fig)
        print(f"[OK] evidence.png from {isaac['packing']} and {isaac['organized']}")
    else:
        print(f"[WARN] evidence.png skipped; Isaac stills found: {list(isaac)}")

    # 3. per-object bars, aligned by object id
    ids = [o["id"] for o in results["organized"]["objects"]]
    fig, ax = plt.subplots(figsize=(11, 4))
    x = np.arange(len(ids)); w = .4
    for j, family in enumerate(FAMILIES):
        by_id = {o["id"]: o for o in results[family]["objects"]}
        vals = [by_id[i]["exposure"] for i in ids]
        bars = ax.bar(x + (j - .5) * w, vals, w, label=f"{family} (score {results[family]['score']:.3f})")
        for b, i in zip(bars, ids):
            if by_id[i].get("pools"):
                ax.plot(b.get_x() + b.get_width() / 2, b.get_height() + .01, "rv", ms=5)
    ax.set_xticks(x); ax.set_xticklabels(ids, rotation=75, fontsize=7)
    ax.set_ylabel("mean exposure"); ax.set_ylim(0, 1); ax.legend()
    ax.set_title("Per-object exposure, same objects in both arrangements; red = cannot drain")
    fig.tight_layout(); fig.savefig(out / "bars.png", dpi=150); plt.close(fig)

    # 4. turntable GIF
    fig = plt.figure(figsize=(11, 5.2))
    axes = [fig.add_subplot(1, 2, i + 1, projection="3d") for i in range(2)]
    for ax, family in zip(axes, FAMILIES):
        draw_3d(ax, results[family], segments[family], label(results[family]))
    fig.suptitle("Exposure turntable, arm-disc sources (red = cannot drain)", fontsize=11)

    def update(k):
        for ax in axes:
            ax.view_init(elev=32, azim=-50 + 360. * k / args.frames)
        return axes
    FuncAnimation(fig, update, frames=args.frames, blit=False).save(
        out / "turntable.gif", writer=PillowWriter(fps=12), dpi=80)
    plt.close(fig)

    # 5. ray-motion video: one arm rotating under one bowl of the organized state
    rays_gif(results["organized"], arrangements["organized"], segments["organized"], out, args.frames)

    payload = {"pair": args.pair, "source": E.DEFAULTS["source"], "sanity": {k: {"exposure": v["exposure"]} for k, v in sanity.items()},
               "results": {f: E.strip_samples(results[f]) for f in FAMILIES},
               "seconds": round(time.time() - started, 1)}
    (out / "scores.json").write_text(json.dumps(payload, indent=2) + "\n")
    print(f"[OK] sanity alone {sanity['alone']['exposure']:.3f} covered {sanity['covered']['exposure']:.3f}")
    print(f"[OK] wrote {out} in {payload['seconds']}s")
    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
