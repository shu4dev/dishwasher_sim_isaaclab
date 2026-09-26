# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Demo scene for the revision-5 exposure scorer: one settled load plus a plate and cutlery.

Kit-free. Takes the settled organized state of one pair (bowls and mugs), adds one dinner
plate standing on edge in the lower-rack tine slot with the most clearance, and a fork, a
knife and a tablespoon head-down in the basket (poses from cutlery_candidates.json: per kind
the candidate with the highest isolated exposure among the first --candidates entries of its
compartment). Writes <out>/<pair>_plus.json, a state-like file that load_state and
frigidaire_exposure_scene_render.py accept, <out>/<pair>_plus_scores.json and
<out>/<pair>_plus_method.png: every food-contact sample coloured by exposure, the two arm
discs, one ray fan per added kind (green open, red blocked) and per-object bars. The added
objects are posed geometrically, not settled.
"""
import argparse
import json
import math
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from dishsim_frigidaire.paths import REPO_ROOT  # noqa: E402
from dishsim_frigidaire import exposure as E  # noqa: E402
from dishsim_frigidaire.loading import rotation, quaternion_xyzw  # noqa: E402
from dishsim_frigidaire.random_poses import compose_pose  # noqa: E402
from frigidaire_exposure_demo import wire_segments, draw_3d  # noqa: E402

STATES = REPO_ROOT / "results/initial_states/frigidaire"
CANDIDATES = REPO_ROOT / "frigidaire/src/dishsim_frigidaire/cutlery_candidates.json"
PLATE_LEAN_DEG = -4          # claims.lower_plate_variants: the v2-accepted lean
PLATE_FLOOR = .006           # claims.LOWER_PLATE_FLOOR, rack-local
ADDED = ("dinner_plate", "fork", "knife", "tablespoon")
SHORT = {"LowerRack": "L", "UpperRack": "U", "SilverwareBasket": "B"}


def parse():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pair", default="random_06")
    p.add_argument("--device", default=E.DEVICE)
    p.add_argument("--samples", type=int, default=E.DEFAULTS["samples_per_object"])
    p.add_argument("--directions", type=int, default=E.DEFAULTS["directions"])
    p.add_argument("--candidates", type=int, default=40, help="cutlery candidates tried per kind")
    p.add_argument("--out", type=Path, default=REPO_ROOT / "results/exposure/frigidaire/scene")
    return p.parse_args()


def plate_slot(arrangement, points):
    """Rack-local pose of a dinner plate on edge in the lower-rack slot with the most clearance."""
    from scipy.spatial import cKDTree
    from dishsim_frigidaire.geometry import PARAMETERS, lower_tine_positions
    obstacles = [E.posed(t, o["position_m"], o["quaternion_xyzw"]).reshape(-1, 3)
                 for o in arrangement.objects if o["rack"] == "LowerRack" for t in E.dish_visuals(o["kind"])]
    obstacles.append(E.posed(E.component_triangles("SilverwareBasket"), *arrangement.basket).reshape(-1, 3))
    tree = cKDTree(np.concatenate(obstacles))
    xs, ys = lower_tine_positions()
    columns = np.concatenate((xs, (xs[:-1] + xs[1:]) / 2))
    origin = np.asarray(E.BODY_POSITIONS["LowerRack"])
    upright = rotation("X", math.radians(90 - PLATE_LEAN_DEG))
    rack = PARAMETERS["lower_rack"]          # rim centreline half-sizes plus 3 mm of wire and slack
    half_x = rack["wire_width"] / 2 - rack["rim_diameter"] / 2 + .003
    half_y = rack["wire_depth"] / 2 - rack["rim_diameter"] / 2 + .003
    best = None
    for orient, tag in ((upright, "faces_y"), (rotation("Z", math.pi / 2) @ upright, "faces_x")):
        local = points @ orient.T
        z = PLATE_FLOOR + .002 - local[:, 2].min()
        for x in columns:
            for y in ys:
                world = local + origin + [x, y, z]
                if np.abs(world[:, 0] - origin[0]).max() > half_x or np.abs(world[:, 1] - origin[1]).max() > half_y:
                    continue                                       # outside the rack footprint
                clearance = float(tree.query(world)[0].min())
                if best is None or clearance > best[0]:
                    best = (clearance, [float(x), float(y), float(z)], [float(v) for v in quaternion_xyzw(orient)], tag)
    return best


def cutlery_pose(kind, arrangement, sources, device, n):
    """Best-exposed head-down basket candidate among the first n of the kind's own compartment."""
    patterns = json.loads(CANDIDATES.read_text())["patterns"][kind]
    compartment = patterns[0]["slot"].split("_")[1]
    best = None
    for c in [c for c in patterns if c["slot"].split("_")[1] == compartment][:n]:
        p, q = compose_pose(arrangement.basket[0], arrangement.basket[1], c["position"], c["quaternion_xyzw"])
        e = E.isolated_baseline(kind, p, q, device, sources=sources)
        if best is None or e > best[0]:
            best = (e, c)
    return best


def added_objects(state, arrangement, args):
    """The four added objects as state entries (rack-local pose + racks-out world pose)."""
    from dishsim_frigidaire.tableware import tableware_geometry
    snap = state["initial_snapshot"]["poses"]
    points = np.asarray(tableware_geometry("dinner_plate")["visuals"][0][0])
    clearance, position, quat, tag = plate_slot(arrangement, points)
    entries = [("dinner_plate", "LowerRack", position, quat,
                {"slot_clearance_m": clearance, "orientation": tag, "lean_deg": PLATE_LEAN_DEG})]
    print(f"[OK] plate on edge ({tag}) at rack-local {np.round(position, 3).tolist()}, clearance {clearance * 1e3:.1f} mm")
    sources = E.rack_sources("SilverwareBasket", args.directions)[:2]
    for kind in ("fork", "knife", "tablespoon"):
        e, c = cutlery_pose(kind, arrangement, sources, args.device, args.candidates)
        entries.append((kind, "SilverwareBasket", c["position"], c["quaternion_xyzw"],
                        {"slot": c["slot"], "variant": c["variant"], "isolated_exposure": e}))
        print(f"[OK] {kind} head-down in the basket, slot {c['slot']} {c['variant']}, alone {e:.3f}")
    objects = []
    for kind, rack, position, quat, meta in entries:
        frame = snap[rack]
        wp, wq = compose_pose(frame["position_m"], frame["quaternion_xyzw"], position, quat)
        objects.append({"object_id": f"{kind}_{rack}_demo", "kind": kind, "rack": rack,
                        "rack_local_pose": {"position_m": [float(v) for v in position],
                                            "quaternion_xyzw": [float(v) for v in quat]},
                        "pose_world": {"position_m": wp.tolist(), "quaternion_xyzw": wq.tolist()},
                        "provenance": "geometric demo pose, not settled", **meta})
    return objects


def ray_fan(result, obj_index, soup, device):
    """Rays from one representative sample of an object to every source point of its rack."""
    s = result["samples"]
    mask = np.where(s["owner"] == obj_index)[0]
    j = mask[np.argmin(np.abs(s["exposure"][mask] - result["objects"][obj_index]["exposure"]))]   # a typical sample
    start = s["points"][j] + E.DEFAULTS["origin_offset_m"] * s["normals"][j]
    q = E.rack_sources(result["objects"][obj_index]["rack"])[0]
    d = q - start
    dist = np.linalg.norm(d, axis=1)
    dirs = d / dist[:, None]
    facing = dirs @ s["normals"][j] > 0                                          # rays that carry weight
    hit, t = E.cast(soup, np.repeat(start[None], len(q), 0), dirs, device, dist)
    segs = np.stack((np.repeat(start[None], len(q), 0), start + dirs * t[:, None]), axis=1)
    return segs[facing & ~hit], segs[facing & hit], start


def method_figure(result, arrangement, out, pair, device):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Line3DCollection
    objs = result["objects"]
    fig = plt.figure(figsize=(17, 8))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    sc = draw_3d(ax, result, wire_segments(arrangement), "", elev=24, azim=-58)
    t = np.linspace(0, 2 * np.pi, 90)
    for name, arm in E.ARM_SOURCES.items():
        c, r = np.asarray(arm["center"]), arm["radius"]
        ax.plot(c[0] + r * np.cos(t), c[1] + r * np.sin(t), c[2], color="black", lw=1.2)
        q = E.disk_points(c, r, E.DEFAULTS["directions"])
        ax.scatter(q[:, 0], q[:, 1], q[:, 2], c="black", s=3)
    soup = E.occluder_soup(arrangement)
    fans = {}
    for kind in ADDED:
        i = next(k for k, o in enumerate(objs) if o["kind"] == kind)
        open_, blocked, start = ray_fan(result, i, soup, device)
        fans[kind] = (len(open_), len(blocked))
        if len(open_):
            ax.add_collection3d(Line3DCollection(open_, colors="green", linewidths=.6, alpha=.8))
        if len(blocked):
            ax.add_collection3d(Line3DCollection(blocked, colors="red", linewidths=.6, alpha=.8))
        ax.text(start[0], start[1], start[2] + .03, kind.replace("_", " "), fontsize=9, weight="bold",
                bbox={"boxstyle": "round,pad=0.2", "fc": "white", "ec": "none", "alpha": .75})
    ax.set_title("Samples coloured by exposure; black discs = the spray-arm sources;\n"
                 "ray fans from one sample per added kind: green reaches the disc, red is blocked", fontsize=10)
    fig.colorbar(sc, ax=ax, shrink=.5, pad=.06, label="exposure (weighted share of open rays)")

    ax2 = fig.add_subplot(1, 2, 2)
    order = sorted(range(len(objs)), key=lambda k: (objs[k]["kind"] not in ADDED, objs[k]["rack"], objs[k]["kind"], objs[k]["id"]))
    vals = [objs[k]["exposure"] for k in order]
    colors = ["tab:orange" if objs[k]["kind"] in ADDED else "0.6" for k in order]
    bars = ax2.bar(np.arange(len(order)), vals, color=colors, edgecolor="black", linewidth=.4)
    for b, k in zip(bars, order):
        if objs[k].get("pools"):
            ax2.plot(b.get_x() + b.get_width() / 2, b.get_height() + .01, "rv", ms=6)
    ax2.set_xticks(np.arange(len(order)))
    ax2.set_xticklabels([f"{objs[k]['kind']} ({SHORT[objs[k]['rack']]})" for k in order], rotation=75, fontsize=7)
    ax2.set_ylim(0, 1); ax2.set_ylabel("mean exposure of the food-contact surface")
    ax2.set_title(f"Per object (orange = added kinds); score {result['score']:.3f}, worst {result['worst']:.3f}, "
                  f"{'feasible' if result['feasible'] else 'INFEASIBLE'} (pooling {result['pooling_count']})", fontsize=10)
    fig.suptitle(f"Exposure scorer revision 5: {pair} organized load + a plate on edge + fork, knife and tablespoon "
                 f"in the basket ({result['n_objects']} objects, racks in)\nFood contact = vessel interiors, plate top, "
                 "spoon bowl, fork tines, knife blade; handles never. Sources = the lower and middle arm discs.", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, .93))
    fig.savefig(out / f"{pair}_plus_method.png", dpi=150); plt.close(fig)
    return fans


def main():
    import matplotlib
    matplotlib.use("Agg")
    args = parse()
    args.out.mkdir(parents=True, exist_ok=True)
    started = time.time()
    source = STATES / f"organized_20260911_seed20260911/states/{args.pair}.json"
    state = json.loads(source.read_text())
    arrangement = E.load_state(source)
    added = added_objects(state, arrangement, args)
    poses = dict(state["initial_snapshot"]["poses"], **{o["object_id"]: o["pose_world"] for o in added})
    objects = [{k: o[k] for k in ("object_id", "kind", "rack", "rack_local_pose", "pose_world")} for o in state["objects"]] + added
    scene = {"schema_version": 1, "purpose": "exposure_demo", "state_id": f"{args.pair}_plus",
             "source_state": str(source.relative_to(REPO_ROOT)), "source_state_sha256": arrangement.sha256,
             "status": "geometric demo: source objects settled, added objects posed (not settled)",
             "demo_added": [o["object_id"] for o in added], "objects": objects,
             "counts": {"total": len(objects), "by_kind": dict(Counter(o["kind"] for o in objects)),
                        "by_rack": dict(Counter(o["rack"] for o in objects))},
             "initial_snapshot": {"joints": state["initial_snapshot"]["joints"], "poses": poses}}
    scene_path = args.out / f"{args.pair}_plus.json"
    scene_path.write_text(json.dumps(scene, indent=1) + "\n")

    result = E.score_state(scene_path, device=args.device, samples=args.samples, directions=args.directions, baselines=False)
    fans = method_figure(result, E.load_state(scene_path), args.out, args.pair, args.device)
    for o in result["objects"]:
        if o["kind"] in ADDED:
            print(f"[OK] {o['id']}: exposure {o['exposure']:.3f}, area {o['area_m2'] * 1e4:.1f} cm2, pools {o['pools']}, "
                  f"fan open/blocked {fans[o['kind']]}")
    payload = {"pair": args.pair, "scene": scene_path.name, "added": added, "result": E.strip_samples(result),
               "ray_fans": fans, "seconds": round(time.time() - started, 1)}
    (args.out / f"{args.pair}_plus_scores.json").write_text(json.dumps(payload, indent=1) + "\n")
    print(f"[OK] score {result['score']:.3f} worst {result['worst']:.3f} feasible {result['feasible']} "
          f"({result['n_objects']} objects, device {result['parameters']['device']})")
    print(f"[OK] wrote {args.out} in {payload['seconds']}s")
    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
