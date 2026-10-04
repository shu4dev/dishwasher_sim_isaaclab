#!/usr/bin/env python3
"""Kit-free mount search for the UR5e: which pedestal pose reaches every grasp of an instance's moves?

    python3 code/execution/frigidaire/frigidaire_robot_mount.py --instance data/results/benchmark/frigidaire_hotec/instances/easy/easy_s0.json

For every candidate base pose (a grid beside the open door, base x axis facing the dishes) and every dish, the
top-down grasps (``robot/grasp.py``) at its START pose and at its GOAL pose (the settled goal, as a proxy for the
place pose) are tested with the analytic IK (``robot/kin.py``). Score = dishes with a reachable start grasp +
dishes with a reachable goal grasp (at least one candidate each), ties broken by the mean joint margin. Writes
data/results/robot/mount/<instance>.json. Collisions of the arm are NOT checked here (the Isaac episode judges them).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "code/frigidaire/src"))
from dishsim_frigidaire.robot import grasp as G, kin, ur5e  # noqa: E402
from dishsim_frigidaire.robot.rig import T_of  # noqa: E402

XS, YS, ZS = (.35, .45, .55, .65), (-.45, -.55, -.65, -.75, -.85, -.95), (.45, .60, .75, .90)
FORBIDDEN = {"x": (-.36, .36), "y": (-.95, -.12)}      # the open door / extended racks footprint (+ base radius)


def pose_T(p):
    return T_of(p["position_m"], p["quaternion_xyzw"])


ANY_BRANCH = False


def reachable(T_wb, T_tcp, T_w3_tcp):
    """Elbow-up IK solutions (shoulder lift in (-pi, 0), elbow in (0, pi): the episode's branch rule)."""
    sols = kin.ik_wrist3_all(np.linalg.inv(T_wb) @ T_tcp @ np.linalg.inv(T_w3_tcp))
    out = []
    for q in sols:
        lift, elbow = (q[1] + np.pi) % (2 * np.pi) - np.pi, (q[2] + np.pi) % (2 * np.pi) - np.pi
        if np.all(q >= kin.JOINT_LIMITS[:, 0] + .02) and np.all(q <= kin.JOINT_LIMITS[:, 1] - .02) and (ANY_BRANCH or (-np.pi < lift < 0. and 0. < elbow < np.pi)):
            out.append(q)
    return out


def free_reach(world, T_wb, T_tcp, T_w3_tcp, **kw):
    world.T_wb = T_wb
    return [q for q in reachable(T_wb, T_tcp, T_w3_tcp) if world.free(q, **kw)]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--instance", type=Path, required=True)
    ap.add_argument("--grid", choices=("floor", "counter"), default="floor",
                    help="counter = the base on the worktop (z 0.914), behind its front edge")
    ap.add_argument("--any-branch", action="store_true", help="every IK branch (the episode then plans with RRT)")
    args = ap.parse_args()
    global XS, YS, ZS, ANY_BRANCH
    ANY_BRANCH = args.any_branch
    if args.grid == "counter":
        XS, YS, ZS = (.30, .45, .60, .75), (-.25, -.15, -.05, .05, .15), (.914,)
    import importlib.util
    spec = importlib.util.spec_from_file_location("fb", ROOT / "code/planner/frigidaire/frigidaire_bench.py")
    B = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(B)
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.initial_state_candidates import COMPONENT_NAMES
    from dishsim_frigidaire.random_pose_runtime import EXTENSION
    from dishsim_frigidaire.robot import collide as C
    inst = json.loads(args.instance.read_text())
    T_w3_tcp = T_of(ur5e.T_WRIST3_TCP_POS, ur5e.T_WRIST3_TCP_QUAT_XYZW)
    kinds = {o["object_id"]: o["kind"] for o in inst["objects"]}
    starts = {o["object_id"]: pose_T(o["pose_world"]) for o in inst["objects"]}
    goals = {oid: pose_T(g["settled_pose_world"]) for oid, g in inst["goal"]["objects"].items()}
    goal_rack = {oid: g["rack"] for oid, g in inst["goal"]["objects"].items()}
    frames_out = {n: inst["initial_snapshot"]["poses"][n] for n in COMPONENT_NAMES}
    frames_in = json.loads(json.dumps(frames_out))                 # phase L: the upper rack pushed in (slide along +y)
    frames_in["UpperRack"]["position_m"][1] -= EXTENSION["UpperRack"]
    checker = B.bench_checker()
    hulls, X = C.load_model(ROOT / "data/results/robot/arm_model.json")
    world = C.ArmWorld(checker, P.slab_body(checker, B.COUNTER), hulls, X, np.eye(4), T_w3_tcp)
    start_bodies = {oid: checker._body(kinds[oid], o, oid) for oid, o in
                    ((o["object_id"], o["pose_world"]) for o in inst["objects"])}
    target = np.mean([T[:3, 3] for T in (*starts.values(), *goals.values())], axis=0)
    rows = []
    for x in XS:
        for y in YS:
            if args.grid == "floor" and FORBIDDEN["x"][0] < x < FORBIDDEN["x"][1] and FORBIDDEN["y"][0] < y < FORBIDDEN["y"][1]:
                continue
            for z in ZS:
                yaw = np.arctan2(target[1] - y, target[0] - x)          # base +x toward the work centre
                T_wb = np.eye(4)
                T_wb[:3, :3] = [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
                T_wb[:3, 3] = (x, y, z)
                ok_s, ok_g, per = 0, 0, {}
                for oid in starts:
                    checker.update_components(frames_in)                # picks happen in phase L (or later, counter only)
                    world.set_dishes(start_bodies)
                    rs = [g for g in G.tilted_grasps(starts[oid])
                          if free_reach(world, T_wb, g["T_tcp"], T_w3_tcp, ignore=(oid,))
                          and free_reach(world, T_wb, G.hover_along(g["T_tcp"], .10), T_w3_tcp)]
                    checker.update_components(frames_in if goal_rack.get(oid) == "LowerRack" else frames_out)
                    world.set_dishes({})
                    rg = [g for g in G.tilted_grasps(goals[oid])
                          if free_reach(world, T_wb, g["T_tcp"], T_w3_tcp)
                          and free_reach(world, T_wb, G.hover_along(g["T_tcp"], .10), T_w3_tcp)] if oid in goals else []
                    ok_s += bool(rs)
                    ok_g += bool(rg)
                    per[oid] = {"start": len(rs), "goal": len(rg), "start_kind": G.bowl_class(starts[oid]),
                                "goal_kind": G.bowl_class(goals[oid]) if oid in goals else None}
                rows.append({"base": [x, y, z], "yaw_rad": round(float(yaw), 4), "score": ok_s + ok_g, "start_ok": ok_s,
                             "goal_ok": ok_g, "margin": 0., "per_dish": per})
                print(f"   {[x, y, z]}: start {ok_s}/{len(starts)} goal {ok_g}/{len(goals)}", flush=True)
    rows.sort(key=lambda r: (-r["score"], -sum(v["start"] + v["goal"] for v in r["per_dish"].values())))
    out = ROOT / "data/results/robot/mount" / f"{inst['instance_id']}{'' if args.grid == 'floor' else '_counter'}{'_any' if args.any_branch else ''}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"instance": inst["instance_id"], "n_dishes": len(starts), "collision_checked": True,
                               "best": rows[0], "top": rows[:8]}, indent=1) + "\n")
    b = rows[0]
    print(f"[RESULT] {'PASS' if b['score'] == 2 * len(starts) else 'PARTIAL'} mount {b['base']} yaw {b['yaw_rad']}: "
          f"{b['start_ok']}/{len(starts)} starts, {b['goal_ok']}/{len(goals)} goals collision-free", flush=True)
    for r in rows[:5]:
        print(f"   best {r['base']} score {r['score']} (start {r['start_ok']}, goal {r['goal_ok']})")
    print("   per dish (best):", {k: (v['start'], v['goal'], v['start_kind'], v['goal_kind']) for k, v in b["per_dish"].items()})


if __name__ == "__main__":
    main()
