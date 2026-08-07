# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Venv side (no Kit): render the RRT-Connect planning visual for one trial query.

Re-runs the exact planning query of trial ``trial_<slot>_<seed>_<rep>`` (same goal-subset RNG
and OMPL seed as ``scripts/experiment/run_trials.py``) with ``PlanDebug`` instrumentation, then
renders two figures:

- ``media/trials/rrt_connect_viz_<slot>_<seed>_<rep>.png`` — paper-style single 3-D view:
  translucent obstacle hulls, both RRT-Connect trees as FK-mapped branches (blue = start
  tree, orange = goal tree), the solution path, labeled start/goal points. Slot 9 / seed 2
  gives the densest successful tree; slot 2 / seed 0 connects almost immediately.
- ``media/trials/plan_visual_<slot>_<seed>_<rep>.png`` — 4-panel diagnostic:

- C-space (shoulder_pan x shoulder_lift): both RRT-Connect trees, invalid samples, raw +
  simplified solution, goal states, plan stats;
- workspace top-down: rack wires, slot cells, tree-vertex TCP scatter, solution TCP trace;
- workspace 3-D: statics vertex clouds + TCP polyline;
- executed joint profiles after constant-speed time parameterization.

The figure is rendered even on a planner timeout (the run then FAILs, but the tree shows why).

Run with:
    /workspace/isaaclab/env_isaaclab/bin/python scripts/evaluation/plan_visual.py --slot 2 --seed 0

Rack-state scenarios (cache built by 12/13/15 with the same --scenario) render to
``media/trials/<scenario>/``:
    /workspace/isaaclab/env_isaaclab/bin/python scripts/evaluation/plan_visual.py --scenario both_out
"""

import argparse
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

import trimesh  # noqa: E402

from dishsim import config, geometry  # noqa: E402
from dishsim import planners as dplanners  # noqa: E402
from dishsim import trajectory as dtraj  # noqa: E402
from dishsim.collision_world import CollisionWorld  # noqa: E402
from dishsim.placement import SlotFrame  # noqa: E402
from dishsim.plan_debug_io import load_plan_debug  # noqa: E402
from dishsim.transforms import make_T  # noqa: E402
from dishsim.ur5e_kin import fk_wrist3  # noqa: E402

parser = argparse.ArgumentParser(description="Render the planning visual for one trial query.")
parser.add_argument("--planner", type=str, default=None,
                    help="Planner for --replan (default: config.PLANNER; see dishsim.planners).")
parser.add_argument("--plan_debug", type=str, default=None,
                    help="Saved plan artifact (.npz). Default: the matching trial in the latest run.")
parser.add_argument("--run", type=str, default=None, help="Run id holding the plan artifact.")
parser.add_argument("--replan", action="store_true",
                    help="Plan a fresh query here instead of reading a recorded one.")
parser.add_argument("--slot", type=int, default=2, help="Slot id (default matches trial_02_00_0).")
parser.add_argument("--seed", type=int, default=0, help="Trial RNG seed.")
parser.add_argument("--repeat", type=int, default=0, help="Trial repeat index.")
parser.add_argument("--budget_s", type=float, default=config.PLAN_TIME_BUDGET_S)
parser.add_argument("--out_dir", type=str, default=None,
                    help="Media dir (default: media/trials or media/trials/<scenario>).")
parser.add_argument("--scenario", type=str, default="both_out",
                    help="Rack-state scenario (see config.SCENARIOS).")
args = parser.parse_args()

config.apply_scenario(args.scenario)
if args.out_dir is None:
    args.out_dir = config.scenario_media_dir("trials")
RESULTS_DIR = (os.path.join(PROJECT_ROOT, "results") if args.scenario == "both_out"
               else os.path.join(PROJECT_ROOT, "results", args.scenario))
# planning happens in the shared post-rack-action placement state (see scripts/experiment/run_trials.py): the cache,
# its hash, and the goal sets all belong to that state, whatever the trial's initial scenario
config.apply_scenario(config.PLACEMENT_STATE)
CACHE = config.scenario_cache_dir()


def trial_start_q() -> np.ndarray:
    """The place-plan start config: the trial's recorded post-pick carry config (q_carry),
    falling back to home for legacy records."""
    path = os.path.join(RESULTS_DIR, f"trial_{args.slot:02d}_{args.seed:02d}_{args.repeat}.json")
    try:
        with open(path) as f:
            rec = json.load(f)
        if rec.get("q_carry"):
            return np.array(rec["q_carry"], dtype=float)
        print(f"[WARN] {path} has no q_carry — using HOME_Q")
    except OSError:
        print(f"[WARN] no trial record at {path} — using HOME_Q")
    return np.array(config.HOME_Q)


START_Q = trial_start_q()

# house palette (dataviz reference); series hues are consistent across
# panels: blue = start tree, orange = goal tree, aqua = solution path, muted gray = context.
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e5e4e0"
MUTED = "#898781"
C_START = "#2a78d6"
C_GOAL = "#eb6834"
C_PATH = "#1baf7a"
SERIES6 = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

T_W3_TCP = make_T(config.T_WRIST3_TCP_POS, config.T_WRIST3_TCP_QUAT)

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"[{'OK' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}")
    if not ok:
        FAILURES.append(name)


def finish() -> None:
    print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL: ' + ', '.join(FAILURES)}")
    raise SystemExit(0 if not FAILURES else 1)


def style_axes(ax) -> None:
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=8)
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def tcp_xyz(q_arr: np.ndarray) -> np.ndarray:
    """TCP positions [m] in the robot-base frame for joint configs, shape [N, 3]."""
    return np.array([(fk_wrist3(q) @ T_W3_TCP)[:3, 3] for q in np.atleast_2d(q_arr)])


def pick(n_total: int, n_max: int, seed: int) -> np.ndarray:
    """Deterministic subsample indices (all indices when n_total <= n_max)."""
    if n_total <= n_max:
        return np.arange(n_total)
    return np.random.default_rng(seed).choice(n_total, n_max, replace=False)


def statics_cloud(manifest: dict, name: str, n_max: int) -> np.ndarray:
    """Subsampled vertices [m] of a cached static body, transformed to the robot-base frame."""
    entry = manifest["statics"][name]
    mesh = trimesh.load(os.path.join(CACHE, entry["mesh"]), force="mesh")
    verts = mesh.vertices[pick(len(mesh.vertices), n_max, seed=7)]
    t = np.array(entry["T_base_body"])
    return (t @ np.hstack([verts, np.ones((len(verts), 1))]).T).T[:, :3]


def panel_cspace(ax, dbg: dplanners.PlanDebug, res: dplanners.PlanResult, sub: np.ndarray,
                 exec_time_s: float | None) -> None:
    """Panel A: the search in a (q0, q1) projection of the 6-D C-space."""
    style_axes(ax)
    ax.xaxis.grid(True, color=GRID, linewidth=0.8)
    if dbg.checked_q is not None and dbg.n_checks:
        val = dbg.checked_q[dbg.checked_valid]
        val = val[pick(len(val), 6000, seed=1)]
        ax.scatter(val[:, 0], val[:, 1], s=2, c=MUTED, alpha=0.10, linewidths=0,
                   label="checked valid")
        if (~dbg.checked_valid).any():
            inv = dbg.checked_q[~dbg.checked_valid]
            inv = inv[pick(len(inv), 5000, seed=2)]
            ax.scatter(inv[:, 0], inv[:, 1], s=7, c=MUTED, alpha=0.55, marker="x",
                       linewidths=0.7, label="checked invalid")
    if dbg.tree_q is not None and dbg.tree_edges is not None and len(dbg.tree_edges):
        e = dbg.tree_edges
        same = dbg.tree_tag[e[:, 0]] == dbg.tree_tag[e[:, 1]]
        bridge, e_draw = e[~same], e[same]
        e_draw = e_draw[pick(len(e_draw), 4000, seed=3)]
        for tag, color, label in ((1, C_START, "start tree"), (2, C_GOAL, "goal tree")):
            m = dbg.tree_tag[e_draw[:, 0]] == tag
            if m.any():
                segs = np.stack([dbg.tree_q[e_draw[m, 0]][:, :2], dbg.tree_q[e_draw[m, 1]][:, :2]], axis=1)
                ax.add_collection(LineCollection(segs, colors=color, linewidths=0.7, alpha=0.55,
                                                 label=label))
        for a, b in bridge:  # the RRT-Connect bridge: always drawn, never subsampled away
            pa, pb = dbg.tree_q[a][:2], dbg.tree_q[b][:2]
            ax.plot([pa[0], pb[0]], [pa[1], pb[1]], color=INK, lw=1.6, zorder=5)
            ax.annotate("connect", ((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2), fontsize=7,
                        color=INK, xytext=(4, 4), textcoords="offset points")
    ax.scatter(sub[:, 0], sub[:, 1], s=18, facecolors="none", edgecolors=INK2, linewidths=0.6,
               label="goal states")
    if dbg.raw_path_q is not None:
        ax.plot(dbg.raw_path_q[:, 0], dbg.raw_path_q[:, 1], color=C_PATH, lw=1.2, ls="--",
                alpha=0.8, label="raw path")
    if res.path_q is not None:
        ax.plot(res.path_q[:, 0], res.path_q[:, 1], color=C_PATH, lw=2.5, marker="o",
                markersize=4, label="simplified path", zorder=6)
        ax.scatter(*res.path_q[-1, :2], s=60, c=C_PATH, zorder=7)
    ax.scatter(*START_Q[:2], s=140, marker="*", c=INK, zorder=7, label="start (carry)")

    v1 = int((dbg.tree_tag == 1).sum()) if dbg.tree_tag is not None else 0
    v2 = int((dbg.tree_tag == 2).sum()) if dbg.tree_tag is not None else 0
    n_edges = len(dbg.tree_edges) if dbg.tree_edges is not None else 0
    n_valid = int(dbg.checked_valid.sum()) if dbg.checked_valid is not None else 0
    n_raw = len(dbg.raw_path_q) if dbg.raw_path_q is not None else 0
    n_simp = len(res.path_q) if res.path_q is not None else 0
    stats = [
        f"status: {res.status}   plan: {res.plan_time_s:.2f} s (budget {args.budget_s:.0f} s)",
        f"path: {res.path_len_rad:.2f} rad, {n_raw} → {n_simp} waypoints",
        f"checks: {dbg.n_checks} ({n_valid} valid / {dbg.n_checks - n_valid} invalid)",
        f"tree: {v1} start + {v2} goal vertices, {n_edges} edges",
        f"goals: {len(sub)} offered, #{res.goal_index} reached",
    ]
    if exec_time_s is not None:
        stats.append(f"exec: {exec_time_s:.1f} s @ {config.EXEC_JOINT_SPEED_RAD_S} rad/s cap")
    ax.text(0.02, 0.98, "\n".join(stats), transform=ax.transAxes, va="top", fontsize=7.5,
            color=INK, family="monospace",
            bbox={"facecolor": SURFACE, "edgecolor": GRID, "boxstyle": "round,pad=0.4"})
    ax.set_xlabel(f"q0 {JOINT_NAMES[0]} [rad]", color=INK2)
    ax.set_ylabel(f"q1 {JOINT_NAMES[1]} [rad]", color=INK2)
    ax.set_title("C-space search (2 of 6 joints shown)", color=INK, fontsize=11)
    ax.legend(loc="lower right", fontsize=7, labelcolor=INK2, framealpha=0.9,
              facecolor=SURFACE, edgecolor=GRID)


def panel_topdown(ax, manifest: dict, slots: dict, dbg: dplanners.PlanDebug,
                  res: dplanners.PlanResult, sub: np.ndarray) -> None:
    """Panel B: search coverage and TCP path over the rack, top-down in the base frame."""
    style_axes(ax)
    ax.xaxis.grid(True, color=GRID, linewidth=0.8)
    rack = statics_cloud(manifest, "E_shelf_1_04", 20000)
    ax.scatter(rack[:, 0], rack[:, 1], s=1.5, c=MUTED, alpha=0.65, linewidths=0, label="rack wires")
    for s in slots.values():
        cx, cy, half = s.T_base_slot[0, 3], s.T_base_slot[1, 3], s.width_m / 2.0
        chosen = s.slot_id == args.slot
        ax.add_patch(plt.Rectangle((cx - half, cy - half), 2 * half, 2 * half,
                                   fill=chosen, facecolor=C_START if chosen else "none",
                                   alpha=0.15 if chosen else 1.0,
                                   edgecolor=C_START if chosen else GRID,
                                   lw=1.4 if chosen else 0.7))
        ax.annotate(str(s.slot_id), (cx - half, cy + half), ha="left", va="top",
                    fontsize=8 if chosen else 6, color=INK if chosen else INK2,
                    xytext=(1, -1), textcoords="offset points")
    theta = np.linspace(0, 2 * np.pi, 128)
    ax.plot(config.UR5E_REACH_M * np.cos(theta), config.UR5E_REACH_M * np.sin(theta),
            ls="--", lw=0.8, color=INK2, label=f"{config.UR5E_REACH_M} m reach")
    ax.scatter([0], [0], marker="*", s=140, c=INK, zorder=7)
    ax.annotate("base", (0, 0), fontsize=7, color=INK2, xytext=(5, -9), textcoords="offset points")
    if dbg.tree_q is not None:
        idx = pick(len(dbg.tree_q), 3000, seed=4)
        tp = tcp_xyz(dbg.tree_q[idx])
        for tag, color in ((1, C_START), (2, C_GOAL)):
            m = dbg.tree_tag[idx] == tag
            ax.scatter(tp[m, 0], tp[m, 1], s=8, c=color, alpha=0.55, linewidths=0, zorder=5)
    goal_tp = tcp_xyz(sub)
    ax.scatter(goal_tp[:, 0], goal_tp[:, 1], s=10, facecolors="none", edgecolors=INK2,
               linewidths=0.5, label="goal TCPs")
    if res.path_q is not None:
        path_tp = tcp_xyz(res.path_q)
        ax.plot(path_tp[:, 0], path_tp[:, 1], color=C_PATH, lw=2.5, label="TCP path", zorder=6)
        ax.scatter(*path_tp[0, :2], marker="*", s=90, c=INK, zorder=7)
        ax.scatter(*path_tp[-1, :2], s=40, c=C_PATH, zorder=7)
    ax.set_aspect("equal")
    ax.set_xlim(-0.15, 1.02)
    ax.set_ylim(-0.62, 0.55)
    ax.set_xlabel("x [m] (robot-base frame)", color=INK2)
    ax.set_ylabel("y [m]", color=INK2)
    ax.set_title("Workspace top-down: tree TCPs (blue/orange) + path", color=INK, fontsize=11)
    ax.legend(loc="lower left", fontsize=7, labelcolor=INK2, framealpha=0.9,
              facecolor=SURFACE, edgecolor=GRID)


def panel_3d(ax, manifest: dict, res: dplanners.PlanResult, dbg: dplanners.PlanDebug,
             sub: np.ndarray, dense: np.ndarray | None) -> None:
    """Panel C: statics vertex clouds + TCP polyline in 3-D."""
    rack = statics_cloud(manifest, "E_shelf_1_04", 8000)
    goal_tp = tcp_xyz(sub)
    frame_pts = [rack, goal_tp]
    tp = None
    if dense is not None:
        tp = tcp_xyz(dense[:: max(1, len(dense) // 150)])
        frame_pts.append(tp)
    framed = np.vstack(frame_pts)
    mn, mx = framed.min(axis=0) - 0.10, framed.max(axis=0) + 0.10
    for name, n_max in (("E_shelf_1_04", 8000), ("E_body_5", 6000), ("E_door_4", 4000)):
        c = statics_cloud(manifest, name, n_max) if name != "E_shelf_1_04" else rack
        c = c[((c >= mn) & (c <= mx)).all(axis=1)]  # keep the frame on the action, not the cabinet
        if len(c):
            ax.scatter(c[:, 0], c[:, 1], c[:, 2], s=0.6, c=MUTED, alpha=0.45, linewidths=0)
    ax.scatter(goal_tp[:, 0], goal_tp[:, 1], goal_tp[:, 2], s=5, c=C_GOAL, alpha=0.7, linewidths=0)
    if dbg.raw_path_q is not None:
        raw_tp = tcp_xyz(dbg.raw_path_q)
        ax.plot(raw_tp[:, 0], raw_tp[:, 1], raw_tp[:, 2], color=C_PATH, lw=0.9, ls="--", alpha=0.7)
    if tp is not None:
        ax.plot(tp[:, 0], tp[:, 1], tp[:, 2], color=C_PATH, lw=2.5)
        ax.scatter(*tp[0], marker="*", s=90, c=INK)
        ax.scatter(*tp[-1], s=40, c=C_PATH)
    ax.set_xlim(mn[0], mx[0])
    ax.set_ylim(mn[1], mx[1])
    ax.set_zlim(mn[2], mx[2])
    ax.set_box_aspect(mx - mn)
    ax.view_init(elev=24, azim=-55)
    ax.set_axis_off()
    ax.set_title("TCP path through the scene (home ★ → slot; orange = goal TCPs)",
                 color=INK, fontsize=11, y=0.98)


def panel_profiles(ax, res: dplanners.PlanResult, dense: np.ndarray | None) -> None:
    """Panel D: executed joint targets after constant-speed time parameterization."""
    style_axes(ax)
    if dense is None:
        ax.text(0.5, 0.5, "no solution — nothing to execute", transform=ax.transAxes,
                ha="center", color=INK2)
        return
    t = np.arange(len(dense)) * config.SIM_DT
    step = config.EXEC_JOINT_SPEED_RAD_S * config.SIM_DT
    tw = [0.0]
    for a, b in zip(res.path_q[:-1], res.path_q[1:]):  # waypoint arrival times (same rule as
        n = max(1, int(np.ceil(np.max(np.abs(b - a)) / step)))  # dtraj.time_parameterize)
        tw.append(tw[-1] + n * config.SIM_DT)
    for x in tw:
        ax.axvline(x, color=GRID, lw=0.8, zorder=0)
    ypos = dense[-1].astype(float).copy()  # de-collide the direct end-labels
    min_gap = (dense.max() - dense.min()) * 0.05
    order = np.argsort(ypos)
    for a, b in zip(order[:-1], order[1:]):
        if ypos[b] - ypos[a] < min_gap:
            ypos[b] = ypos[a] + min_gap
    for j in range(6):
        ax.plot(t, dense[:, j], color=SERIES6[j], lw=1.6)
        ax.annotate(JOINT_NAMES[j], (t[-1], ypos[j]), xytext=(5, 0),
                    textcoords="offset points", fontsize=7.5, color=SERIES6[j], va="center")
    ax.set_xlim(0, t[-1] * 1.18)
    ax.set_xlabel("time [s]", color=INK2)
    ax.set_ylabel("joint position [rad]", color=INK2)
    ax.set_title(f"Executed joint profiles ({config.EXEC_JOINT_SPEED_RAD_S} rad/s cap, "
                 "gridlines = waypoints)", color=INK, fontsize=11)


def panel_tree3d(out_png: str, manifest: dict, dbg: dplanners.PlanDebug, res: dplanners.PlanResult,
                 sub: np.ndarray, dense: np.ndarray | None) -> None:
    """Paper-style single 3-D view: translucent obstacles, both trees at the TCP, path, labels.

    Trees are planned in 6-D joint space; each edge is interpolated in joint space and mapped
    through FK, so branches render as the curves the TCP would actually sweep.
    """
    from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

    from dishsim.geometry import coacd_dir_for

    rack = statics_cloud(manifest, "E_shelf_1_04", 4000)
    start_tp = tcp_xyz(START_Q)[0]
    goal_tp = tcp_xyz(sub)
    pts = [rack, goal_tp, start_tp[None, :]]
    tp_path = None
    if dense is not None:
        tp_path = tcp_xyz(dense[:: max(1, len(dense) // 150)])
        pts.append(tp_path)
    mn = np.vstack(pts).min(axis=0) - 0.10
    mx = np.vstack(pts).max(axis=0) + 0.10

    fig = plt.figure(figsize=(9, 8), facecolor=SURFACE)
    ax = fig.add_subplot(projection="3d")
    # obstacles: CoACD hulls, muted translucent (pink like the reference figure fails the
    # palette validator against the orange goal tree — normal-vision ΔE 12.9 < 15)
    for name, alpha in (("E_shelf_1_04", 0.14), ("E_body_5", 0.05), ("E_door_4", 0.05)):
        entry = manifest["statics"][name]
        t = np.array(entry["T_base_body"])
        pdir = coacd_dir_for(name, entry["mesh"], CACHE)
        for fname in sorted(os.listdir(pdir)):
            hull = trimesh.load(os.path.join(pdir, fname), force="mesh").convex_hull
            v = (t @ np.hstack([hull.vertices, np.ones((len(hull.vertices), 1))]).T).T[:, :3]
            c = v.mean(axis=0)
            if ((c < mn) | (c > mx)).any():
                continue
            ax.add_collection3d(Poly3DCollection(v[hull.faces], facecolor=MUTED, alpha=alpha,
                                                 edgecolor="none"))

    if dbg.tree_q is not None and dbg.tree_edges is not None and len(dbg.tree_edges):
        fracs = np.linspace(0.0, 1.0, 6)[:, None]
        for tag, color, label in ((1, C_START, "start tree"), (2, C_GOAL, "goal tree")):
            edges = dbg.tree_edges[dbg.tree_tag[dbg.tree_edges[:, 0]] == tag]
            segs = []
            for a, b in edges:
                qs = dbg.tree_q[a] * (1 - fracs) + dbg.tree_q[b] * fracs
                tp = tcp_xyz(qs)
                segs.extend(np.stack([tp[:-1], tp[1:]], axis=1))
            if segs:
                segs = np.array(segs)  # mplot3d doesn't clip collections — drop out-of-frame bits
                mid = segs.mean(axis=1)
                segs = segs[((mid >= mn) & (mid <= mx)).all(axis=1)]
                ax.add_collection3d(Line3DCollection(segs, colors=color,
                                                     linewidths=1.0, alpha=0.75, label=label))

    ax.scatter(goal_tp[:, 0], goal_tp[:, 1], goal_tp[:, 2], s=4, c=C_GOAL, alpha=0.6,
               linewidths=0, label="goal TCPs")
    if tp_path is not None:
        ax.plot(tp_path[:, 0], tp_path[:, 1], tp_path[:, 2], color=C_PATH, lw=3,
                label="solution path", zorder=10)
        ax.scatter(*tp_path[-1], s=45, c=C_PATH, edgecolor=INK, linewidths=0.8, zorder=11)
    ax.scatter(*start_tp, s=45, c=C_START, edgecolor=INK, linewidths=0.8, zorder=11)
    bbox = {"facecolor": SURFACE, "edgecolor": INK, "boxstyle": "round,pad=0.3", "alpha": 0.9}
    ax.text(*(start_tp + [0.0, 0.0, 0.07]), "Start point (home)", fontsize=8, color=INK,
            bbox=bbox, ha="center", zorder=12)
    if tp_path is not None:
        ax.text(*(tp_path[-1] + [0.0, 0.0, 0.09]), f"Goal point (slot {args.slot})", fontsize=8,
                color=INK, bbox=bbox, ha="center", zorder=12)

    ax.set_xlim(mn[0], mx[0])
    ax.set_ylim(mn[1], mx[1])
    ax.set_zlim(mn[2], mx[2])
    ax.set_box_aspect(mx - mn)
    ax.view_init(elev=25, azim=-55)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color((1, 1, 1, 0))
    ax.set_xlabel("x [m]", color=INK2, fontsize=8)
    ax.set_ylabel("y [m]", color=INK2, fontsize=8)
    ax.set_zlabel("z [m]", color=INK2, fontsize=8)
    ax.tick_params(colors=INK2, labelsize=7)
    ax.set_title(f"RRT-Connect — search trees and solution path (slot {args.slot}, "
                 f"seed {args.seed}, scenario {config.SCENARIO_NAME})", color=INK, fontsize=12,
                 pad=12)
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9, facecolor=SURFACE, edgecolor=GRID,
              labelcolor=INK2, markerscale=2.5)
    fig.text(0.5, 0.02, "planned in 6-D joint space; trees drawn at the TCP via forward "
             "kinematics", ha="center", fontsize=8, color=INK2)
    fig.savefig(out_png, dpi=140, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)


def default_plan_debug_path() -> str | None:
    """The recorded query for this trial in the requested run (default: the latest run)."""
    from dishsim import metrics as dmetrics  # noqa: PLC0415

    results = os.path.join(PROJECT_ROOT, "results")
    run_dir = (os.path.join(results, "experiments", args.run) if args.run
               else dmetrics.latest_run(results))
    if not run_dir:
        return None
    tag = f"trial_{args.slot:02d}_{args.seed:02d}_{args.repeat}"
    return os.path.join(run_dir, "plans", f"{tag}.npz")


def main() -> None:
    os.makedirs(args.out_dir, exist_ok=True)

    with open(os.path.join(CACHE, "slots", "slots.json")) as f:
        slots_doc = json.load(f)
    with open(os.path.join(CACHE, "slots", "goal_sets.json")) as f:
        goals_doc = json.load(f)
    for doc, fname in ((slots_doc, "slots.json"), (goals_doc, "goal_sets.json")):
        if "scenario" in doc:  # legacy files carry no stamp; stamped files must match exactly
            check(f"{fname} stamp matches scenario", doc["scenario"] == config.SCENARIO_NAME
                  and doc["config_hash"] == geometry.config_hash(),
                  f"{doc['scenario']} vs {config.SCENARIO_NAME}")
            if FAILURES:
                finish()
    slots = {s["slot_id"]: SlotFrame.from_json(s) for s in slots_doc["slots"]}
    goal_sets = {g["slot_id"]: np.array(g["configs"]) for g in goals_doc["goal_sets"]}

    goals = goal_sets.get(args.slot, np.zeros((0, 6)))
    feasible = sorted(s for s, g in goal_sets.items() if len(g))
    check("goal set non-empty", len(goals) > 0, f"slot {args.slot} has {len(goals)} configs "
          f"(feasible slots: {feasible})")
    if not len(goals):
        finish()

    # Prefer the query the experiment recorded (scripts/experiment/run_trials.py --save_plan_debug).
    # Re-planning here would mean duplicating the runner's seed arithmetic, goal-subset RNG and
    # collision-world settings — which drifted apart once already (the visual used a different
    # merged_cluster than the runner), so the artifact is the source of truth.
    dbg = dplanners.PlanDebug()
    artifact = args.plan_debug or default_plan_debug_path()
    if artifact and os.path.exists(artifact):
        arrays, meta = load_plan_debug(artifact)
        for field in ("checked_q", "checked_valid", "tree_q", "tree_tag", "tree_edges", "raw_path_q"):
            setattr(dbg, field, arrays.get(field))
        dbg.n_checks = int(meta.get("n_checks", 0))
        dbg.planner_data_ok = bool(meta.get("planner_data_ok", False))
        dbg.planner_data_error = meta.get("planner_data_error", "")
        sub = arrays["goal_qs"]
        start_q = arrays["start_q"]
        path_q = arrays["path_q"] if len(arrays["path_q"]) else None
        res = dplanners.PlanResult(path_q, meta["plan_time_s"], meta["path_len_rad"],
                                   meta["status"], meta["goal_index"])
        world = CollisionWorld(cache_dir=CACHE, self_check=True,
                               merged_cluster=bool(meta.get("merged_cluster", True)))
        print(f"[INFO] read the recorded query from {os.path.relpath(artifact, PROJECT_ROOT)} "
              f"(planner {meta.get('planner', {}).get('name')})")
    elif args.replan:
        world = CollisionWorld(cache_dir=CACHE, self_check=True)
        rng = np.random.default_rng(args.seed * 1000 + args.repeat)
        sub = goals[rng.choice(len(goals), min(config.GOALS_PER_PLAN, len(goals)), replace=False)]
        start_q = START_Q
        planner_name = args.planner or config.PLANNER
        planner_params = {**config.PLANNER_PARAMS.get(planner_name, {}), "budget_s": args.budget_s}
        planner = dplanners.make_planner(planner_name, **planner_params)
        res = planner.plan(world, start_q, sub, seed=args.seed * 7 + args.repeat + 1, debug=dbg)
        print(f"[INFO] --replan: fresh {planner_name} query (not the trial's recorded one)")
    else:
        check("plan-debug artifact present", False,
              f"no saved query at {artifact} — rerun the experiment with --save_plan_debug, "
              "or pass --replan to plan a fresh query here")
        finish()
    dense = dtraj.time_parameterize(res.path_q) if res.path_q is not None else None
    exec_time_s = len(dense) * config.SIM_DT if dense is not None else None

    n_edges = len(dbg.tree_edges) if dbg.tree_edges is not None else 0
    n_verts = len(dbg.tree_q) if dbg.tree_q is not None else 0
    print(f"[INFO] plan {res.status} in {res.plan_time_s:.2f}s, len {res.path_len_rad:.2f} rad, "
          f"{dbg.n_checks} validity checks, tree {n_verts} vertices / {n_edges} edges")

    fig = plt.figure(figsize=(15, 10.5), facecolor=SURFACE)
    gs = fig.add_gridspec(2, 2, hspace=0.26, wspace=0.18, left=0.06, right=0.97, top=0.90,
                          bottom=0.06)
    panel_cspace(fig.add_subplot(gs[0, 0]), dbg, res, sub, exec_time_s)
    panel_topdown(fig.add_subplot(gs[0, 1]), world.manifest, slots, dbg, res, sub)
    panel_3d(fig.add_subplot(gs[1, 0], projection="3d"), world.manifest, res, dbg, sub, dense)
    panel_profiles(fig.add_subplot(gs[1, 1]), res, dense)
    fig.suptitle(f"RRT-Connect, 6-D joint space — slot {args.slot}, seed {args.seed}, "
                 f"repeat {args.repeat}, scenario {config.SCENARIO_NAME} (run_trials planner)",
                 color=INK, fontsize=14)
    out_png = os.path.join(args.out_dir, f"plan_visual_{args.slot:02d}_{args.seed:02d}_{args.repeat}.png")
    fig.savefig(out_png, dpi=120, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)
    print(f"[INFO] figure: {out_png}")

    tree_png = os.path.join(args.out_dir,
                            f"rrt_connect_viz_{args.slot:02d}_{args.seed:02d}_{args.repeat}.png")
    panel_tree3d(tree_png, world.manifest, dbg, res, sub, dense)
    print(f"[INFO] tree view: {tree_png}")

    check("planner solved within budget", res.status == "solved",
          f"{res.status} after {res.plan_time_s:.2f}s")
    if dense is not None:
        col = world.in_collision_batch(dense[::5])
        check("executed path re-validates collision-free", not col.any(),
              f"{len(col)} configs checked")
    check("debug capture non-empty", dbg.n_checks > 0, f"{dbg.n_checks} checks")
    check("planner tree extracted", dbg.planner_data_ok, dbg.planner_data_error or f"{n_verts} vertices")
    check("figure written", os.path.isfile(out_png), out_png)
    check("tree view written", os.path.isfile(tree_png), tree_png)
    finish()


if __name__ == "__main__":
    main()
