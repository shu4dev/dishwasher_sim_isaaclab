# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Venv side (no Kit): measure where on the countertop plane the robot can actually pick.

The countertop is a config constant, but "how big should it be" is not a free choice — the far
half of the current slab already sits outside the UR5e's reach (its far corners are 1.099 m from
the robot base against a ~0.85 m reach), so growing it in +x adds surface the robot can never
use. This script measures the reachable set instead of guessing it, per the numeric-provenance
rule: the derived rectangle printed here is what goes into ``config.COUNTERTOP_*`` and
``config.TASK["spawn_rect_w"]``.

Method — the same feasibility idiom the runner uses for a real pick
(``scripts/experiment/run_trials.py:597-612``), evaluated on a grid:

    stand the object at (x, y) on the countertop plane
      -> T_grasp_tcp = T_base_obj @ inv(T_tcp_obj)          (config.grasp_transform, pure)
      -> hover  = T_grasp_tcp + PICK_HOVER_M along base z
      -> ik_wrist3_all(...) at BOTH the hover and the grasp pose
      -> keep solutions that survive world.in_collision

A cell counts as reachable for a class when at least ``--min_yaws`` of the sampled yaws yield a
collision-free IK solution at the hover *and* at the grasp pose. Requiring both matters: the
hover alone is what the runner checks today, and a pose can hover fine yet have no valid arm
configuration 10 cm lower.

The collision world used is the placement-state cache with nothing carried (``pick_world`` in
the runner). That is the state a pick happens in. The countertop slab itself lives inside the
``E_body_5`` static mesh, so this map is measured against the CURRENT slab — valid for sizing
because the objects stand *above* the surface and the growth direction is away from the machine.
Re-run after the cache rebuild to confirm the new slab did not eat into the reachable set.

Run with:
    /workspace/isaaclab/env_isaaclab/bin/python scripts/setup/reach_map.py
    /workspace/isaaclab/env_isaaclab/bin/python scripts/setup/reach_map.py --step 0.01 --yaws 8
"""

import argparse
import json
import os
import sys

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402
from dishsim.fill_plan import _axis_bottom_offset, _stand_R  # noqa: E402
from dishsim.collision_world import CollisionWorld  # noqa: E402
from dishsim.transforms import T_inv, make_T  # noqa: E402
from dishsim.ur5e_kin import ik_wrist3_all  # noqa: E402

parser = argparse.ArgumentParser(description="Measure the reachable region of the countertop plane.")
parser.add_argument("--scenario", type=str, default="placement",
                    help="Machine state the pick happens in (default: placement).")
parser.add_argument("--step", type=float, default=0.02, help="Grid pitch [m].")
parser.add_argument("--yaws", type=int, default=4, help="Yaw samples per cell.")
parser.add_argument("--min_yaws", type=int, default=1,
                    help="Cell counts as reachable when this many yaws succeed.")
parser.add_argument("--classes", type=str, default=None,
                    help="Comma list of object classes (default: every robot_demo class).")
parser.add_argument("--margin_m", type=float, default=0.02,
                    help="Inset applied to the derived spawn rectangle [m].")
parser.add_argument("--rect_x_min", type=float, default=0.65,
                    help="Derived rectangle may not extend below this world x [m]. The worktop "
                         "must not overhang the door opening (open-door tip at x 0.271, and the "
                         "robot loads the racks through that space), so the counter starts at "
                         "the machine's mid-depth. The full grid is still mapped and plotted.")
parser.add_argument("--out_dir", type=str, default=os.path.join(PROJECT_ROOT, "media", "reach"))
args = parser.parse_args()


def main() -> int:
    config.apply_scenario(args.scenario)
    cache_dir = config.scenario_cache_dir(args.scenario)
    if not os.path.exists(os.path.join(cache_dir, "scene_state.json")):
        print(f"[FAIL] no collision cache at {cache_dir} — run extract_geometry/decompose_meshes first")
        return 1

    world = CollisionWorld(cache_dir=cache_dir, self_check=True, object_attached=False)
    T_w3_tcp = np.array(world.manifest["t_wrist3_tcp"])
    T_tcp_w3 = T_inv(T_w3_tcp)

    names = ([s.strip() for s in args.classes.split(",")] if args.classes
             else [k for k, s in config.OBJECTS.items() if s.robot_demo])
    print(f"[INFO] scenario={args.scenario} cache={os.path.relpath(cache_dir, PROJECT_ROOT)}")
    print(f"[INFO] classes: {names}")

    # grid over the countertop plane, generously beyond the current slab so growth room shows up
    cx, cy, cz = config.COUNTERTOP_CENTER_W
    top_z_w = cz + config.COUNTERTOP_SIZE[2] / 2.0
    xs = np.arange(0.50, 1.10 + 1e-9, args.step)
    ys = np.arange(-0.60, 0.45 + 1e-9, args.step)
    yaws = np.linspace(0.0, 360.0, args.yaws, endpoint=False)
    # full world -> base re-expression (the base may be yawed)
    T_base_w = T_inv(make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W))
    q_seed = np.array(config.HOME_Q)

    masks: dict[str, np.ndarray] = {}
    for name in names:
        spec = config.OBJECTS[name]
        T_tcp_obj = make_T(*config.grasp_transform(spec))
        T_obj_tcp = T_inv(T_tcp_obj)
        mask = np.zeros((len(xs), len(ys)), dtype=bool)
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                n_ok = 0
                for yaw in yaws:
                    R = _stand_R(spec, float(yaw))  # world-frame stand rotation + yaw
                    T_w_obj = np.eye(4)
                    T_w_obj[:3, :3] = R
                    T_w_obj[:3, 3] = (x, y, top_z_w + _axis_bottom_offset(spec, R)[2])
                    T_base_obj = T_base_w @ T_w_obj  # rotation AND translation re-expressed
                    T_grasp = T_base_obj @ T_obj_tcp
                    T_hover = T_grasp.copy()
                    T_hover[2, 3] += config.PICK_HOVER_M
                    if _feasible(world, T_hover, T_grasp, T_tcp_w3, q_seed):
                        n_ok += 1
                        if n_ok >= args.min_yaws:
                            break
                mask[i, j] = n_ok >= args.min_yaws
        masks[name] = mask
        print(f"[INFO] {name:<10} reachable cells: {int(mask.sum()):5d} / {mask.size} "
              f"({100.0 * mask.sum() / mask.size:.1f}%)")

    stack = np.stack(list(masks.values()))
    inter, union = stack.all(axis=0), stack.any(axis=0)
    print(f"[INFO] {'ANY class':<10} reachable cells: {int(union.sum()):5d} / {union.size} "
          f"({100.0 * union.sum() / union.size:.1f}%)")
    print(f"[INFO] {'ALL classes':<10} reachable cells: {int(inter.sum()):5d} / {inter.size} "
          f"({100.0 * inter.sum() / inter.size:.1f}%)")

    # The spawn rectangle is derived from the UNION, not the intersection. Classes disagree
    # about where they are pickable (plate's edge_pinch has a hole in the mid-y band that mug
    # does not), and the layout generator reach-checks every item individually and resamples
    # rejects — so an all-class rectangle would needlessly shrink the usable surface to a
    # sliver. The per-class acceptance inside the derived rectangle is reported below; it is
    # what tells you whether rejection sampling will converge.
    allowed = union & (xs[:, None] >= args.rect_x_min)
    i0, i1, j0, j1 = largest_rectangle(allowed)
    if i1 < i0 or j1 < j0:
        print(f"[FAIL] no cell with x >= {args.rect_x_min} is reachable for any class")
        return 1
    rect = {
        "x_min": float(xs[i0] + args.margin_m), "x_max": float(xs[i1] - args.margin_m),
        "y_min": float(ys[j0] + args.margin_m), "y_max": float(ys[j1] - args.margin_m),
    }
    area = (rect["x_max"] - rect["x_min"]) * (rect["y_max"] - rect["y_min"])
    print(f"[INFO] spawn rectangle (union, x >= {args.rect_x_min}, inset {args.margin_m * 1000:.0f} mm): "
          f"x [{rect['x_min']:.3f}, {rect['x_max']:.3f}] y [{rect['y_min']:.3f}, {rect['y_max']:.3f}] "
          f"-> {area:.4f} m^2")
    inside = (xs[:, None] >= rect["x_min"]) & (xs[:, None] <= rect["x_max"]) \
        & (ys[None, :] >= rect["y_min"]) & (ys[None, :] <= rect["y_max"])
    accept = {k: float(np.mean(v[inside])) for k, v in masks.items()}
    for k, v in sorted(accept.items(), key=lambda kv: kv[1]):
        print(f"[INFO]   acceptance inside rect: {k:<10} {100.0 * v:5.1f}%")
    worst = min(accept.values())
    if worst < 0.10:
        print(f"[WARN] worst-class acceptance {100.0 * worst:.1f}% — rejection sampling may struggle")

    # the countertop slab must cover the spawn rectangle with clearance on every side
    slab_lo = (min(rect["x_min"] - args.margin_m, 0.65), rect["y_min"] - args.margin_m)
    slab_hi = (max(rect["x_max"] + args.margin_m, 1.045), rect["y_max"] + args.margin_m)
    print(f"[INFO] implied countertop: x [{slab_lo[0]:.3f}, {slab_hi[0]:.3f}] "
          f"y [{slab_lo[1]:.3f}, {slab_hi[1]:.3f}]")
    print(f"[INFO]   COUNTERTOP_SIZE = ({slab_hi[0] - slab_lo[0]:.3f}, {slab_hi[1] - slab_lo[1]:.3f}, 0.020)")
    print(f"[INFO]   COUNTERTOP_CENTER_W = ({(slab_lo[0] + slab_hi[0]) / 2:.4f}, "
          f"{(slab_lo[1] + slab_hi[1]) / 2:.4f}, {config.COUNTERTOP_CENTER_W[2]:.3f})")

    os.makedirs(args.out_dir, exist_ok=True)
    report = {
        "scenario": args.scenario, "step_m": args.step, "yaws": args.yaws,
        "min_yaws": args.min_yaws, "margin_m": args.margin_m,
        "countertop_top_z_w": float(top_z_w),
        "per_class_cells": {k: int(v.sum()) for k, v in masks.items()},
        "acceptance_in_rect": accept,
        "n_cells": int(inter.size), "all_class_cells": int(inter.sum()), "any_class_cells": int(union.sum()),
        "spawn_rect_w": rect, "spawn_rect_area_m2": float(area),
        "grid": {"x_min": float(xs[0]), "x_max": float(xs[-1]),
                 "y_min": float(ys[0]), "y_max": float(ys[-1])},
    }
    with open(os.path.join(args.out_dir, "reach_map.json"), "w") as f:
        json.dump(report, f, indent=2)
    _plot(xs, ys, masks, union, rect, args.out_dir)
    print(f"[INFO] wrote {os.path.relpath(args.out_dir, PROJECT_ROOT)}/reach_map.{{json,png}}")
    print("[RESULT] PASS")
    return 0


def _feasible(world, T_hover, T_grasp, T_tcp_w3, q_seed) -> bool:
    """A yaw is feasible when one arm configuration clears BOTH the hover and the grasp pose."""
    hover_sols = [q for q in ik_wrist3_all(T_hover @ T_tcp_w3, q_seed=q_seed) if not world.in_collision(q)]
    if not hover_sols:
        return False
    return any(not world.in_collision(q) for q in ik_wrist3_all(T_grasp @ T_tcp_w3, q_seed=hover_sols[0]))


def largest_rectangle(mask: np.ndarray) -> tuple[int, int, int, int]:
    """Largest all-True axis-aligned rectangle in a boolean grid.

    Returns:
        ``(i0, i1, j0, j1)`` inclusive index bounds, or ``(0, -1, 0, -1)`` when empty.
    """
    n_rows, n_cols = mask.shape
    heights = np.zeros(n_cols, dtype=int)
    best = (0, (0, -1, 0, -1))
    for i in range(n_rows):
        heights = np.where(mask[i], heights + 1, 0)
        stack: list[tuple[int, int]] = []
        for j in range(n_cols + 1):
            h = heights[j] if j < n_cols else 0
            start = j
            while stack and stack[-1][1] >= h:
                sj, sh = stack.pop()
                area = sh * (j - sj)
                if area > best[0] and sh > 0:
                    best = (area, (i - sh + 1, i, sj, j - 1))
                start = sj
            stack.append((start, h))
    return best[1]


def _plot(xs, ys, masks, union, rect, out_dir: str) -> None:
    import matplotlib  # noqa: PLC0415

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415
    from matplotlib.patches import Rectangle  # noqa: PLC0415

    panels = list(masks.items()) + [("ANY class", union)]
    n = len(panels)
    ncol = min(4, n)
    nrow = int(np.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(3.2 * ncol, 3.0 * nrow), squeeze=False)
    cur = config.COUNTERTOP_CENTER_W, config.COUNTERTOP_SIZE
    for k, (title, m) in enumerate(panels):
        ax = axes[k // ncol][k % ncol]
        ax.pcolormesh(ys, xs, m.astype(float), cmap="Greens", vmin=0.0, vmax=1.4, shading="nearest")
        ax.add_patch(Rectangle((cur[0][1] - cur[1][1] / 2, cur[0][0] - cur[1][0] / 2),
                               cur[1][1], cur[1][0], fill=False, ec="0.35", lw=1.2, ls="--"))
        if title == "ANY class":
            ax.add_patch(Rectangle((rect["y_min"], rect["x_min"]),
                                   rect["y_max"] - rect["y_min"], rect["x_max"] - rect["x_min"],
                                   fill=False, ec="crimson", lw=1.8))
        ax.plot(0.0, 0.0, "k+", ms=9)  # robot base, world origin in x/y
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("world y [m]", fontsize=8)
        ax.set_ylabel("world x [m]", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.set_aspect("equal")
    for k in range(n, nrow * ncol):
        axes[k // ncol][k % ncol].axis("off")
    fig.suptitle("Countertop-plane pickability (dashed = current slab, red = derived spawn rect)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "reach_map.png"), dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
