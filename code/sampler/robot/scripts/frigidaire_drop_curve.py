#!/usr/bin/env python3
"""The drop-height curve (plans/2026-10-29-sampler-release.md): each dish of a robot placing load, ALONE (no robot), dropped
in Isaac from the sampler's release orientation at several heights above the rack stop; per drop, seated on the goal's wires.

Boot-first; one load per Kit process. The scene is the benchmark's extended baseline (both racks out = the goal records'
frames), every dish of the load parked; per dish the OTHER rack is pushed in (the robot rounds' rack state: an extended upper
rack covers the lower rack and caps its drops near 100 mm). Per dish (the load's witness order) and gap g in --gap-mm, per
repeat r: the dishes before it at their goal poses (moved with their rack's current frame; teleported, 30 ticks; after a drop
the dish is parked, 5 ticks, and the earlier dishes are teleported back only when one is off its goal by > 0.5 mm / 0.5 deg), the dish's
MOVE POSE (frigidaire_bench.move_local: a plate upright in
its slot, 3 mm toward its packed lean's side; a bowl or cup at its goal pose), its insertion descent (frigidaire_bench.
insertion_release: from 20 mm above the rim in 5 mm steps until the appliance alone stops it; geometric, the bench checker's
1 mm allowance), the release lift = the lowest clear step + g exactly (not capped at the descent start; a lift that collides
is recorded 'blocked' and not dropped), a seeded jitter (up to --jitter lateral in the plane normal to the rack's up axis and
about the dish's own axis; re-drawn, seeded, up to 10 times while the jittered pose collides), the dish teleported there at rest and 150 + 60 ticks of settle. Judged on the settled pose:
seated on the goal's wires (plates: frigidaire_robot_episode.seated_test, bowls: bowl_seated_test from kitfree_v1/insert_bowl/
goal_supports.json; cups: the centroid within 2 mm and the rotation within 5 deg of the goal pose), mouth (bowls, cups:
the dish axis z), the offsets from the goal (frigidaire_bench.goal_distance), the drift over the last 60 ticks, the earlier
dishes moved.

    code/util/run_kit.sh code/sampler/robot/scripts/frigidaire_drop_curve.py --headless \\
        --load data/results/robot/placing_v1/easy_plates/load.json --gap-mm 20,40,60,80,100,150 --repeats 5

Writes data/results/robot/drop_curve/<load>/drops.jsonl (one row per drop), summary.csv (per dish and gap) and highest.json
(per dish: the highest gap with a seated repeat, the highest with all repeats seated, the at-goal count per gap).

--goals (plans/2026-10-30-bowl-drop100.md, as round 20 had it): the refreshed goals (goals_v4.json, the load's frames) replace
the load's goal poses (the earlier dishes of the witness order sit there, the seated test judges against them) and each drop
starts at the dish's refreshed goal + the gap along its rack's up axis (no insertion descent: the release h above the goal, as
the D-ring-h100 robot rows let go); --bowl-supports names the bowl seated test's goal_supports json (goal_supports_v4.json).

    code/util/run_kit.sh code/sampler/robot/scripts/frigidaire_drop_curve.py --headless \\
        --load data/results/robot/placing_v1/easy_bowls/load.json --goals data/results/robot/placing_v1/easy_bowls/goals_v4.json \\
        --bowl-supports data/results/robot/kitfree_v1/insert_bowl/goal_supports_v4.json --gap-mm 100 --repeats 5 \\
        --dishes bowl_06 --out-root data/results/robot/bowl_drop100/curve_bowl_06"""
from __future__ import annotations

import argparse
import csv
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[4]
CUP_POS_M, CUP_ROT_DEG = .002, 5.               # cups: "seated" = the goal pose within 2 mm / 5 deg
PRE_TICKS = 30                                  # the earlier dishes teleported to their goals, then this many ticks
POST_TICKS = 5                                  # after a drop the dish is parked, then this many ticks; the earlier dishes are
REUSE_MM, REUSE_DEG = .5, .5                    # re-teleported (PRE_TICKS) only when one is off its goal by more than this
TRAJ_EVERY_TICKS, DT_S = 6, 1. / 120.           # --perch-roll: the dish trajectory at 20 Hz (physics at 120 Hz)
ROLL_DEG = 2.                                   # --perch-roll: the final lean this far below / above the perch tilt = rolled
GATE_K = 4                                      # --perch-roll: a bowl is gated in when its best tilt seats (mouth down) >= 4
DISTURB_MM, DISTURB_DEG = 10., 20.              # an earlier dish moved more than this = moved (the benchmark's rule)
REFRESH_LIFT_M = .002                           # --refresh-goals: each dish released 2 mm above its recorded goal (rack's up axis)
REFRESH_FLAG_MM, REFRESH_FLAG_DEG = 15., 15.    # ... a settled goal farther than this from the recorded one is flagged


def seed_of(*parts):
    return int(hashlib.sha256("|".join(str(p) for p in parts).encode()).hexdigest()[:8], 16)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--load", type=Path, required=True, help="a placing_v1/<load>/load.json")
    ap.add_argument("--gap-mm", default="20,40,60,80,100,150")
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--jitter", default="2mm,2deg", help="lateral and about-axis jitter bounds, seeded per repeat")
    ap.add_argument("--dishes", default="", help="only these dishes (comma list)")
    ap.add_argument("--out-root", type=Path, default=ROOT / "data/results/robot/drop_curve")
    ap.add_argument("--baseline", type=Path, default=ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json")
    ap.add_argument("--usd", type=Path, default=ROOT / "data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc")
    ap.add_argument("--max-wall-seconds", type=float, default=9000.)
    ap.add_argument("--goals", type=Path, default=None,
                    help="plans/2026-10-30-bowl-drop100.md: refreshed goals (goals_v4.json, the load's frames) replacing the "
                         "load's goal poses; each drop starts at the refreshed goal + the gap along the rack's up axis")
    ap.add_argument("--bowl-supports", type=Path, default=None,
                    help="the bowl seated test's goal_supports json (default frigidaire_robot_episode.BOWL_SUPPORTS_JSON)")
    ap.add_argument("--perch-roll", default="",
                    help="bowls (plans/2026-10-29-perch-roll.md): instead of --gap-mm, per tilt (deg from straight down, comma "
                         "list, e.g. 66,75,82,90; tilts below the bowl's goal lean are dropped) the round-11 over-lean pose at "
                         "spin 0 (frigidaire_robot_episode.overlean_tcp: turned about the lip's rim tangent, the centroid at the "
                         "goal x, y) lowered alone from 350 mm to its first contact (perch_descent) and released THERE; per drop "
                         "also mouth down, at goal (frigidaire_bench.AT_GOAL), the roll vs the static-roll prediction and the "
                         "20 Hz dish trajectory; writes best_tilt.json (the best tilt per bowl and the 4/5 gate)")
    ap.add_argument("--refresh-goals", action="store_true",
                    help="plans/2026-10-29-ring-place.md: instead of drops, each dish of --dishes (the load's witness order) ALONE "
                         "released 2 mm above its recorded goal pose along its rack's up axis (its rack out, the other pushed in; "
                         "the earlier dishes of this run at their refreshed goals, every other dish parked), 150 + 60 settle "
                         "ticks; the settled pose (in the goal record's frames) is the refreshed goal; writes <load dir>/"
                         "goals_v4_<--refresh-tag>.json with the drift per dish (flagged above 15 mm or 15 deg)")
    ap.add_argument("--refresh-tag", default="all", help="--refresh-goals: the output's tag (one per parallel process)")
    ap.add_argument("--rejudge", action="store_true",
                    help="Kit-free: re-judge an existing drops.jsonl with the current seated tests (round 14: the bowl test's "
                         "physical wire ids, the goals resting on a dish or on tines only, the rear plates wedged between their "
                         "tines) and rewrite drops.jsonl (the as-run verdict kept as seated_as_run), summary.csv, highest.json")
    if "--rejudge" in sys.argv:
        args = ap.parse_args()
        load = json.loads(args.load.read_text())
        return rejudge(args, load, args.out_root / load["load"])
    sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util")]
    from dishsim.app import add_args, launch   # Kit-free; boots Kit below
    add_args(ap)
    ap.set_defaults(device="cpu", headless=True)
    args = ap.parse_args()
    load = json.loads(args.load.read_text())
    L = load["load"]
    if args.perch_roll and args.out_root == ROOT / "data/results/robot/drop_curve":
        args.out_root = ROOT / "data/results/robot/perch_roll"
    out = args.out_root / L
    if args.refresh_goals:                          # its own folder per process (report, the Kit scene)
        out = (ROOT / "data/results/robot/goal_refresh" if args.out_root == ROOT / "data/results/robot/drop_curve"
               else args.out_root) / f"{L}_{args.refresh_tag}"
        if (out / "report.json").exists():
            print(f"[RESULT] FAIL goal refresh {L}: refusing to overwrite {out}", flush=True)
            return
    if (out / "drops.jsonl").exists():
        print(f"[RESULT] FAIL drop curve {L}: refusing to overwrite {out / 'drops.jsonl'}", flush=True)
        return
    out.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    report = {"result": "FAIL", "load": L, "started_utc": datetime.now(timezone.utc).isoformat()}
    app = launch(args)
    args.app = app
    try:
        run(args, load, out, report, started)
    except Exception as exc:                                   # noqa: BLE001
        traceback.print_exc()
        report.update(result="FAIL", reason=f"exception {exc!r}")
    finally:
        report.update(wall_seconds=round(time.monotonic() - started, 1), finished_utc=datetime.now(timezone.utc).isoformat())
        (out / "report.json").write_text(json.dumps(report, indent=1, default=str) + "\n")
        print(f"[RESULT] {report['result']} drop curve {L}: {report.get('summary') or report.get('reason')}", flush=True)
        from dishsim.media import release_sim_for_close
        release_sim_for_close()
        app.close()


def run(args, load, out, report, started):
    import numpy as np
    sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util"), str(ROOT / "code/sampler/scripts"),
                    str(ROOT / "code/sampler/robot/scripts")]
    import arrangement_optimization.frigidaire_bench as B
    import frigidaire_bench_kit as K
    import frigidaire_robot_episode as EP
    import fcl
    from types import SimpleNamespace
    from motion_planning import planner as P
    from asset.loading import visual_points
    from sampler.robot.rig import T_of
    from sampler.initial_state_candidates import COMPONENT_NAMES
    from sampler.random_pose_runtime import EXTENSION, RACK_JOINT
    L = load["load"]
    gaps = [float(g) / 1e3 for g in args.gap_mm.split(",")]
    jl, ja = (float(x.rstrip("mdeg")) for x in args.jitter.split(","))
    jl /= 1e3
    dishes = sorted(load["dishes"], key=lambda d: d["order_index"])
    names = [d["name"] for d in dishes]
    todo = [n for n in names if not args.dishes or n in set(args.dishes.split(","))]
    meta = {d["name"]: d for d in dishes}
    goal = {d["name"]: {"position_m": [float(v) for v in d["goal_pose"]["pos"]],
                        "quaternion_xyzw": [float(v) for v in d["goal_pose"]["quat_xyzw"]]} for d in dishes}
    if args.goals is not None:                     # plans/2026-10-30-bowl-drop100.md: the refreshed goals (the load's frames)
        gv = json.loads(args.goals.read_text())
        fr_mm = max((float(np.linalg.norm(np.subtract(gv["frames"][r]["position_m"], load["frames"][r]["position_m"]))) * 1e3
                     for r in ("LowerRack", "UpperRack") if "frames" in gv), default=0.)
        if fr_mm > 1.:
            raise RuntimeError(f"--goals {args.goals}: its frames differ from the load's by {fr_mm:.1f} mm")
        for n_, g_ in gv["bowls"].items():
            if n_ in goal and g_.get("goal_pose"):
                goal[n_] = {"position_m": [float(v) for v in g_["goal_pose"]["pos"]],
                            "quaternion_xyzw": [float(v) for v in g_["goal_pose"]["quat_xyzw"]]}
        report["goals"] = {"file": str(args.goals), "bowls": sorted(gv["bowls"]), "frames_vs_load_mm": round(fr_mm, 3),
                           "flagged": gv.get("flagged")}
    if args.bowl_supports is not None:
        EP.BOWL_SUPPORTS_JSON = args.bowl_supports if args.bowl_supports.is_absolute() else ROOT / args.bowl_supports
        report["bowl_supports"] = str(EP.BOWL_SUPPORTS_JSON)
    report["baseline"] = str(args.baseline)
    base = json.loads(args.baseline.read_text())
    snapshot = base.get("snapshot", base)
    entries = [{"id": d["name"], "kind": d["kind"], "rack": d["rack"]} for d in dishes]
    backend = K._sequence_scene(args, B, entries, snapshot, out / "_kit_scene", report)
    if backend is None:
        raise RuntimeError("the extended baseline did not hold")
    centroids = B.kind_centroids({k: visual_points(B.ASSETS / f"{k}.usda") for k in B.KINDS})
    checker = B.release_checker()
    judge = SimpleNamespace(checker=B.bench_checker(), fcl=fcl)          # the seated tests' FCL world
    labels = EP.rack_labels()
    park = {n: {"position_m": [3. + .4 * j, 2., .3], "quaternion_xyzw": [0., 0., 0., 1.]} for j, n in enumerate(names)}
    now = backend.poses()
    frames = {r: now[r] for r in ("LowerRack", "UpperRack")}
    report["frames_vs_load_mm"] = {r: round(float(np.linalg.norm(np.subtract(frames[r]["position_m"],
                                                                             load["frames"][r]["position_m"]))) * 1e3, 2)
                                   for r in frames}

    F_goal = {r: P.pose_T(load["frames"][r]) for r in ("LowerRack", "UpperRack")}
    moves = json.loads((ROOT / load["goal_record"]).read_text())["moves_world"]    # the sampler's move poses (its frames)
    tilts = [float(t) for t in args.perch_roll.split(",") if t.strip()]
    if tilts:
        import frigidaire_robot_reach as RC
        M_bowl = np.asarray(RC.measured_M("bowl")[0], dtype=float)                 # the bowl gate's median rim pinch (TCP -> dish)
        com = np.asarray(EP.PERCH_COM_DISH_M, dtype=float)
        report["perch_roll"] = {"tilts": tilts, "hold": "frigidaire_robot_reach.measured_M('bowl')", "dropped_tilts": {}}

    def perch_cases(n, d, now, ctx):
        """The perch-roll cases of one bowl: per tilt the over-lean pose at spin 0 lowered alone (perch_descent, the components
        and the earlier dishes as they are now) to its first contact; the static roll (the uniform-density centre of mass minus
        the contact, along the goal axis's horizontal direction: + = mouth down)."""
        judge.checker.update_components({nm: now[nm] for nm in COMPONENT_NAMES})
        r_ = d["rack"]
        T_g = P.pose_T(now[r_]) @ np.linalg.inv(F_goal[r_]) @ P.pose_T(goal[n])
        lean_g = float(np.degrees(np.arccos(np.clip(-T_g[2, 2], -1., 1.))))
        tg = list(judge.checker.components.items()) + [(c, judge.checker._body(meta[c]["kind"], now[c], c)) for c in ctx]
        hd, ez = EP.goal_hor(T_g, np), np.array([0., 0., 1.])
        out_ = []
        for th in tilts:
            if th < lean_g - 1e-6:
                report["perch_roll"]["dropped_tilts"].setdefault(n, []).append(th)
                continue
            at = lambda h, th=th: EP.overlean_tcp(M_bowl, T_g, centroids["bowl"], h, 0., th, np)[1]   # noqa: E731
            rr = EP.perch_descent(judge, "bowl", at, n, tg, np)
            if rr is None:
                out_.append({"key": th, "why": "no first contact from 350 mm down to -300 mm"})
                continue
            h, tn, cp, start = rr
            T_p = at(h)
            info = {"tilt_deg": th, "goal_lean_deg": round(lean_g, 1), "perch_mm": round(h * 1e3, 1), "perch_body": tn,
                    "perch_start_mm": round(start * 1e3, 1)}
            if cp is not None:
                off = (T_p[:3, :3] @ com + T_p[:3, 3]) - np.asarray(cp)
                cm = round(float(off @ hd) * 1e3, 1)
                info.update(contact_point=[round(float(v), 5) for v in cp], perch_part=EP.part_name(judge, tn, cp, np),
                            com_mouth_mm=cm, com_lateral_mm=round(float(off @ np.cross(ez, hd)) * 1e3, 1),
                            roll_pred="mouth-down" if cm > 0 else "mouth-up")
            out_.append({"key": th, "pose": EP._pose_of(T_p), "info": info})
        return out_

    def move_world(d, now):
        """The sampler's move pose (the goal record's moves_world: a plate upright in its slot 3 mm toward its packed lean's
        side, a bowl or cup at its PACKED pose), moved with the dish's rack from the record's frame to its frame now."""
        r = d["rack"]
        return EP._pose_of(P.pose_T(now[r]) @ np.linalg.inv(F_goal[r]) @ P.pose_T(moves[d["name"]]))

    def set_racks(rack):
        """The dish's rack out, the other pushed in (the robot rounds' rule)."""
        other = "UpperRack" if rack == "LowerRack" else "LowerRack"
        want = {RACK_JOINT[rack]: EXTENSION[rack], RACK_JOINT[other]: 0.}
        js = backend.joints()
        move = {j: v for j, v in want.items() if abs(js[j] - v) > .005}
        if move:
            backend.ramp(move, f"drop_curve_racks_{rack}")

    def scene_for(i):
        """The dishes before ``i`` at their goals moved with their rack's current frame, the rest parked, PRE_TICKS ticks."""
        now = backend.poses()
        for j, n in enumerate(names):
            if j < i:
                r = meta[n]["rack"]
                pose = EP._pose_of(P.pose_T(now[r]) @ np.linalg.inv(F_goal[r]) @ P.pose_T(goal[n]))
            else:
                pose = park[n]
            backend.set_rigid_pose(backend.objects[n], pose)
        for _ in range(PRE_TICKS):
            backend.tick()
        return backend.poses()

    def jittered(pose, up, rng):
        T = P.pose_T(pose)
        r, a = jl * math.sqrt(rng.uniform()), rng.uniform(0., 2 * math.pi)
        u = np.asarray(up, dtype=float)
        e1 = np.cross(u, [1., 0., 0.]) if abs(u[0]) < .9 else np.cross(u, [0., 1., 0.])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(u, e1)
        d = r * (math.cos(a) * e1 + math.sin(a) * e2)
        th = math.radians(rng.uniform(-ja, ja))
        Rz = np.eye(4)
        Rz[:2, :2] = [[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]]
        T2 = T @ Rz
        T2[:3, 3] += d
        return EP._pose_of(T2), {"lateral_mm": [round(float(x) * 1e3, 2) for x in d], "spin_deg": round(math.degrees(th), 2)}

    if args.refresh_goals:                         # plans/2026-10-29-ring-place.md
        return refresh(args, load, out, report, started, backend, B, K, EP, P, np, checker, centroids, names, todo, meta, goal,
                       F_goal, park, set_racks)
    rows, t_loop = [], time.monotonic()
    f_rows = open(out / "drops.jsonl", "a")
    reps_all = list(range(args.repeats))           # pass 1 = the first 3 repeats of every dish, pass 2 = the rest (the wall
    passes = [reps_all[:3], reps_all[3:]] if args.repeats > 3 else [reps_all]   # cap cuts pass 2 first)
    cache, cut = {}, None
    for p_i, reps in enumerate(passes):
      for i, n in enumerate(names):                # noqa: E111
        if n not in todo or not reps or cut:
            continue
        d, kind = meta[n], meta[n]["kind"]
        set_racks(d["rack"])
        now = scene_for(i)
        ctx = names[:i]
        app_b = B.appliance_bodies(checker, now)
        up, rim_z, top_z = B.rack_heights(now, d["rack"])
        if n not in cache and args.goals is not None:   # from the refreshed goal (its rack's frame now) + the gap
            mv = EP._pose_of(P.pose_T(now[d["rack"]]) @ np.linalg.inv(F_goal[d["rack"]]) @ P.pose_T(goal[n]))
            m = {"lowest": 0.}
            cache[n] = (mv, m, {"load": L, "dish": n, "kind": kind, "rack": d["rack"], "slot": d.get("slot"),
                                "variant": d.get("variant"), "order_index": i, "move_pose": mv,
                                "release_from": "the refreshed goal (goals_v4) + the gap along the rack's up axis"})
            print(f"[DROP] {L} {n} ({i + 1}/{len(names)}): from its refreshed goal + gap, the earlier dishes {ctx} at "
                  f"their refreshed goals", flush=True)
        if n not in cache:                         # the descent once per dish (geometric; the earlier dishes at their goals)
            bodies = [checker._body(meta[c]["kind"], now[c], c) for c in ctx]
            mv = move_world(d, now)
            m = B.insertion_release(checker, [(kind, mv, n)], bodies, app_b, up, rim_z, top_z)
            cache[n] = (mv, m, {"load": L, "dish": n, "kind": kind, "rack": d["rack"], "slot": d.get("slot"),
                                "variant": d.get("variant"), "order_index": i, "move_pose": mv, **B.path_info(m)})
            print(f"[DROP] {L} {n} ({i + 1}/{len(names)}): descent {B.path_info(m)}", flush=True)
        mv, m, base_rec = cache[n]
        T_goal = P.pose_T(goal[n])
        if tilts:                                  # the perch-roll cases (once per dish; the scene is the same every pass)
            if ("perch", n) not in cache:
                cache[("perch", n)] = perch_cases(n, d, now, ctx)
                print(f"[DROP] {L} {n}: perches " + "; ".join(f"{c_['key']:.0f} deg "
                      + (f"{c_['info']['perch_mm']} mm on {c_['info']['perch_body']} roll {c_['info'].get('roll_pred')}"
                         if "info" in c_ else c_["why"]) for c_ in cache[("perch", n)]), flush=True)
            cases = cache[("perch", n)]
        else:
            cases = [{"key": g} for g in gaps]
        dirty = False                              # scene_for(i) just ran

        def off_goal(now_):
            """The largest offset (mm, deg) of the earlier dishes from their goals in their rack's frame now."""
            w = (0., 0.)
            for c in ctx:
                rc = meta[c]["rack"]
                Tg = P.pose_T(now_[rc]) @ np.linalg.inv(F_goal[rc]) @ P.pose_T(goal[c])
                Tc = P.pose_T(now_[c])
                dg = float(np.degrees(np.arccos(np.clip((np.trace(Tg[:3, :3].T @ Tc[:3, :3]) - 1) / 2, -1, 1))))
                w = (max(w[0], float(np.linalg.norm(Tc[:3, 3] - Tg[:3, 3])) * 1e3), max(w[1], dg))
            return w
        for case in cases:
            g = case["key"]                        # the gap (m), or the tilt (deg) with --perch-roll
            tag = round(g, 2) if tilts else round(g * 1e3)
            for r in reps:
                if time.monotonic() - started > args.max_wall_seconds - 90:
                    cut = f"wall budget {args.max_wall_seconds:.0f} s: stopped in pass {p_i + 1} at {n} {'tilt' if tilts else 'gap'} {tag} repeat {r}"
                    break
                t0 = time.monotonic()
                rec = {**base_rec, **({"tilt_deg": g, **case.get("info", {})} if tilts else {"gap_mm": round(g * 1e3)}),
                       "repeat": r}
                if tilts and "pose" not in case:
                    rec.update(dropped=False, why=case["why"])
                    rows.append(rec)
                    f_rows.write(json.dumps(rec, default=str) + "\n")
                    f_rows.flush()
                    continue
                if not tilts and m["lowest"] is None:
                    rec.update(dropped=False, why=f"the descent is blocked ({m['blocked']}: {m['blockers']})")
                    rows.append(rec)
                    f_rows.write(json.dumps(rec, default=str) + "\n")
                    f_rows.flush()
                    continue
                now = scene_for(i) if dirty else backend.poses()
                pre = {c: P.pose_T(now[c]) for c in ctx}
                bodies = [checker._body(meta[c]["kind"], now[c], c) for c in ctx]
                lift_pose = case["pose"] if tilts else B.lifted(mv, m["lowest"] + g, up)
                hit, k_try = None, 0
                for k_try in range(10):                 # the jitter re-drawn (seeded) while the jittered pose collides
                    rng = np.random.default_rng(seed_of(L, n, tag, r, k_try))
                    rel_pose, jit = jittered(lift_pose, up, rng)
                    hit = B.release_blockers(checker, checker._body(kind, rel_pose, n), bodies, app_b)
                    if not hit:
                        break
                rec.update(release_pose=rel_pose, jitter=jit, jitter_tries=k_try + 1, seed=seed_of(L, n, tag, r, k_try),
                           scene_reset=bool(dirty), earlier_off_goal=[round(x, 2) for x in off_goal(now)])
                if not tilts:
                    rec["release_lift_mm"] = round((m["lowest"] + g) * 1e3, 1)
                if hit:
                    rec.update(dropped=False, why=f"the release pose collides with {hit}")
                    rows.append(rec)
                    f_rows.write(json.dumps(rec, default=str) + "\n")
                    f_rows.flush()
                    continue
                backend.set_rigid_pose(backend.objects[n], rel_pose)
                hist, traj = [], []
                if tilts:                               # the 20 Hz dish trajectory: the release pose, then every 6 ticks
                    traj.append([0.] + [round(float(v), 6) for v in rel_pose["position_m"] + rel_pose["quaternion_xyzw"]])
                for k in range(K.SETTLE_TICKS + K.WINDOW_TICKS):
                    backend.tick()
                    if k == K.SETTLE_TICKS - 1 or k == K.SETTLE_TICKS + K.WINDOW_TICKS - 1:
                        hist.append(P.pose_T(backend.poses()[n]))
                    if tilts and (k + 1) % TRAJ_EVERY_TICKS == 0:
                        p_ = backend.poses()[n]
                        traj.append([round((k + 1) * DT_S, 4)] + [round(float(v), 6) for v in
                                                                   p_["position_m"] + p_["quaternion_xyzw"]])
                now2 = backend.poses()
                T = P.pose_T(now2[n])
                dist = B.goal_distance(centroids[kind], T, T_goal)
                drift_mm = float(np.linalg.norm(hist[1][:3, 3] - hist[0][:3, 3])) * 1e3
                drift_deg = float(np.degrees(np.arccos(np.clip((np.trace(hist[0][:3, :3].T @ hist[1][:3, :3]) - 1) / 2, -1, 1))))
                moved = []
                for c in ctx:
                    Tc = P.pose_T(now2[c])
                    mm_ = float(np.linalg.norm(Tc[:3, 3] - pre[c][:3, 3])) * 1e3
                    dg = float(np.degrees(np.arccos(np.clip((np.trace(pre[c][:3, :3].T @ Tc[:3, :3]) - 1) / 2, -1, 1))))
                    if mm_ > DISTURB_MM or dg > DISTURB_DEG:
                        moved.append([c, round(mm_, 1), round(dg, 1)])
                judge.checker.update_components({nm: now2[nm] for nm in COMPONENT_NAMES})
                mouth = None if kind == "plate" else ("down" if T[2, 2] < 0 else "up")
                if kind == "plate":
                    F_now = P.pose_T(now2["LowerRack"])
                    st = EP.seated_test(judge, kind, n, T, F_now @ np.linalg.inv(F_goal["LowerRack"]) @ T_goal, d["slot"], labels,
                                        np, centroid=centroids[kind])
                    seated = st["seated"]
                elif kind == "bowl":
                    F_r = P.pose_T(now2[d["rack"]])
                    st = EP.bowl_seated_test(judge, kind, n, T, n, np, others={c: now2[c] for c in ctx},
                                             T_goal=F_r @ np.linalg.inv(F_goal[d["rack"]]) @ T_goal, centroid=centroids[kind])
                    seated = bool(st["seated_wires"] and mouth == "down")
                else:
                    pos_mm = math.hypot(dist[0], dist[1]) * 1e3
                    st = {"centroid_mm": round(pos_mm, 2), "rot_deg": round(dist[2], 2),
                          "rule": f"centroid within {CUP_POS_M * 1e3:g} mm and rotation within {CUP_ROT_DEG:g} deg of the goal"}
                    seated = bool(pos_mm <= CUP_POS_M * 1e3 and dist[2] <= CUP_ROT_DEG)
                deep = float(max((v for k_, v in backend.latest_contact["depths_m"].items() if n in k_.split("|")), default=0.))
                if tilts:                               # mouth down, at goal, the roll vs the static prediction, the trajectory
                    lean_f = float(np.degrees(np.arccos(np.clip(-T[2, 2], -1., 1.))))
                    roll_m = ("mouth-down" if lean_f < g - ROLL_DEG else "mouth-up" if lean_f > g + ROLL_DEG else "stays")
                    rec.update(at_goal=bool(B.within_goal(kind, dist)), lean_final_deg=round(lean_f, 1), roll_measured=roll_m,
                               roll_agrees=None if roll_m == "stays" or "roll_pred" not in case.get("info", {}) else
                               bool(roll_m == case["info"]["roll_pred"]), traj=traj,
                               traj_rule=f"[t s, x, y, z, qx, qy, qz, qw] of the dish at the release and every "
                                         f"{TRAJ_EVERY_TICKS} ticks ({1. / (TRAJ_EVERY_TICKS * DT_S):.0f} Hz), world frame")
                rec.update(dropped=True, seated=bool(seated), seated_detail=st, mouth=mouth, axis_z=round(float(T[2, 2]), 3),
                           at_goal=bool(B.within_goal(kind, dist)),
                           error={"lateral_mm": round(dist[0] * 1e3, 1), "dz_mm": round(dist[1] * 1e3, 1),
                                  "tilt_deg": round(dist[2], 1)},
                           drift_mm=round(drift_mm, 2), drift_deg=round(drift_deg, 2),
                           settled=bool(drift_mm <= 5. and drift_deg <= 3.), contact_mm=round(deep * 1e3, 2),
                           earlier_moved=moved, final_pose=now2[n], wall_s=round(time.monotonic() - t0, 1))
                rows.append(rec)
                f_rows.write(json.dumps(rec, default=str) + "\n")
                f_rows.flush()
                print(f"[DROP] {L} {n} {'tilt %.0f' % g if tilts else 'gap %.0f' % (g * 1e3)} r{r}: "
                      f"{'SEATED' if seated else 'not seated'} mouth {mouth} err {rec['error']} drift {rec['drift_mm']} mm "
                      f"moved {moved}" + (f" roll {rec['roll_measured']} (pred {case['info'].get('roll_pred')})" if tilts else "")
                      + f" ({rec['wall_s']} s)", flush=True)
                backend.set_rigid_pose(backend.objects[n], park[n])
                for _ in range(POST_TICKS):
                    backend.tick()
                w_ = off_goal(backend.poses())
                dirty = w_[0] > REUSE_MM or w_[1] > REUSE_DEG
            if cut:
                break
    report["cut"] = cut
    f_rows.close()
    if tilts:
        bt = write_perch_summaries(out, rows, L, names, todo, meta, np)
        n_drop = sum(bool(x.get("dropped")) for x in rows)
        report.update(result="PASS", drops=n_drop, seated=sum(bool(x.get("seated")) for x in rows), rows=len(rows),
                      loop_seconds=round(time.monotonic() - t_loop, 1), gated=bt["gated"],
                      summary=f"{n_drop} drops ({len(rows)} rows) of {len(todo)} bowls x {len(tilts)} tilts x {args.repeats} "
                              f"repeats{'; ' + cut if cut else ''}; best tilt per bowl " + ", ".join(
                                  f"{k} {v['best_tilt']} ({v['seated_down']}/{v['drops']})" for k, v in bt["bowls"].items())
                              + f"; gated {bt['gated']}")
        return
    per, highest = write_summaries(out, rows, L, names, todo, meta, np)
    n_drop = sum(bool(x.get("dropped")) for x in rows)
    n_seat = sum(bool(x.get("seated")) for x in rows)
    reps_done = {dn: min((len(per.get((dn, round(g * 1e3)), [])) for g in gaps), default=0) for dn in todo}
    report.update(result="PASS", drops=n_drop, seated=n_seat, rows=len(rows), loop_seconds=round(time.monotonic() - t_loop, 1),
                  repeats_done=reps_done,
                  summary=f"{n_drop} drops ({len(rows)} rows) of {len(todo)} dishes x {len(gaps)} gaps x {args.repeats} repeats "
                          f"(rows per gap: min {min(reps_done.values(), default=0)}, max {max(reps_done.values(), default=0)}"
                          f"{'; ' + cut if cut else ''}), seated {n_seat}; highest any-seated per dish "
                          + ", ".join(f"{k} {v['highest_any_seated_mm']}" for k, v in highest.items())
                          + "; seated / at goal per dish and gap " + ", ".join(
                              f"{k} " + " ".join(f"{g} mm seated {c} at goal {v['at_goal_by_gap'][g]}"
                                                 for g, c in v["seated_by_gap"].items())
                              for k, v in highest.items()))


def refresh(args, load, out, report, started, backend, B, K, EP, P, np, checker, centroids, names, todo, meta, goal, F_goal,
            park, set_racks):
    """--refresh-goals: per dish of ``todo`` in the witness order, alone (the earlier refreshed dishes of this run at their
    refreshed goals, the others parked): released REFRESH_LIFT_M above its recorded goal along its rack's up axis (teleported at
    rest), 150 + 60 ticks; the settled pose mapped back to the goal record's rack frames is the refreshed goal."""
    import hashlib as hl
    L = load["load"]
    inv = np.linalg.inv
    ref, rows_ = {}, {}
    t_loop = time.monotonic()
    for i, n in enumerate(names):
        if n not in todo:
            continue
        d, kind, r = meta[n], meta[n]["kind"], meta[n]["rack"]
        set_racks(r)
        now = backend.poses()
        for c in names:
            if c == n:
                continue
            if c in ref:
                rc = meta[c]["rack"]
                backend.set_rigid_pose(backend.objects[c], EP._pose_of(P.pose_T(now[rc]) @ inv(F_goal[rc]) @ ref[c]))
            else:
                backend.set_rigid_pose(backend.objects[c], park[c])
        for _ in range(PRE_TICKS):
            backend.tick()
        now = backend.poses()
        F_r = P.pose_T(now[r])
        up = F_r[:3, 2] / np.linalg.norm(F_r[:3, 2])
        T_g = F_r @ inv(F_goal[r]) @ P.pose_T(goal[n])
        T_rel = T_g.copy()
        T_rel[:3, 3] += REFRESH_LIFT_M * up
        bodies = [checker._body(meta[c]["kind"], now[c], c) for c in ref]
        hit = B.release_blockers(checker, checker._body(kind, EP._pose_of(T_rel), n), bodies, B.appliance_bodies(checker, now))
        backend.set_rigid_pose(backend.objects[n], EP._pose_of(T_rel))
        hist = []
        for k in range(K.SETTLE_TICKS + K.WINDOW_TICKS):
            backend.tick()
            if k == K.SETTLE_TICKS - 1 or k == K.SETTLE_TICKS + K.WINDOW_TICKS - 1:
                hist.append(P.pose_T(backend.poses()[n]))
        now2 = backend.poses()
        T = P.pose_T(now2[n])
        T_rec = F_goal[r] @ inv(P.pose_T(now2[r])) @ T
        rot = lambda A, Bm: float(np.degrees(np.arccos(np.clip((np.trace(A[:3, :3].T @ Bm[:3, :3]) - 1) / 2, -1., 1.))))  # noqa: E731
        drift_mm = float(np.linalg.norm(T[:3, 3] - T_g[:3, 3])) * 1e3
        drift_deg = rot(T_g, T)
        dist = B.goal_distance(centroids[kind], T, T_g)
        w_mm = float(np.linalg.norm(hist[1][:3, 3] - hist[0][:3, 3])) * 1e3
        w_deg = rot(hist[0], hist[1])
        partners = sorted({p_ for pr in backend.latest_contact.get("pairs", []) for p_ in pr if n in pr and p_ != n})
        ref[n] = T_rec
        q = EP._pose_of(T_rec)
        rows_[n] = {"order_index": i, "rack": r, "slot": d.get("slot"),
                    "goal_pose": {"pos": [float(v) for v in q["position_m"]], "quat_xyzw": [float(v) for v in q["quaternion_xyzw"]]},
                    "recorded_goal_pose": {"pos": [float(v) for v in goal[n]["position_m"]],
                                           "quat_xyzw": [float(v) for v in goal[n]["quaternion_xyzw"]]},
                    "drift_mm": round(drift_mm, 2), "drift_deg": round(drift_deg, 2),
                    "flag": bool(drift_mm > REFRESH_FLAG_MM or drift_deg > REFRESH_FLAG_DEG),
                    "error": {"lateral_mm": round(dist[0] * 1e3, 1), "dz_mm": round(dist[1] * 1e3, 1), "tilt_deg": round(dist[2], 1)},
                    "settle_window": {"drift_mm": round(w_mm, 3), "drift_deg": round(w_deg, 3),
                                      "settled": bool(w_mm <= 5. and w_deg <= 3.)},
                    "axis_z": round(float(T[2, 2]), 3), "rests_on": partners,
                    "release_collides_kitfree": hit or None, "final_world": now2[n]}
        print(f"[REFRESH] {L} {n} ({i + 1}/{len(names)}, {r}): drift {drift_mm:.1f} mm / {drift_deg:.1f} deg "
              f"{'FLAG' if rows_[n]['flag'] else 'ok'} err {rows_[n]['error']} window {rows_[n]['settle_window']} on {partners} "
              f"release collides {hit or None}", flush=True)
    asset = ROOT / "data/assets/hotec_wheatstraw/usd/bowl.usda"
    res = {"plan": "plans/2026-10-29-ring-place.md", "load": L, "tag": args.refresh_tag, "dishes": list(rows_),
           "rule": f"each dish alone, its rack out and the other pushed in, the earlier dishes of this run at their refreshed goals "
                   f"and the rest parked: released {REFRESH_LIFT_M * 1e3:.0f} mm above its recorded goal pose along its rack's up "
                   f"axis (teleported at rest), {K.SETTLE_TICKS} + {K.WINDOW_TICKS} ticks; the settled pose in the goal record's "
                   f"rack frames is the refreshed goal; drift = the settled pose vs the recorded goal (flag above "
                   f"{REFRESH_FLAG_MM:.0f} mm or {REFRESH_FLAG_DEG:.0f} deg)",
           "asset_bowl_md5": hl.md5(asset.read_bytes()).hexdigest() if asset.is_file() else None,
           "bowls": rows_, "seconds": round(time.monotonic() - t_loop, 1)}
    dst = args.load.parent / f"goals_v4_{args.refresh_tag}.json"
    dst.write_text(json.dumps(res, indent=1, default=str) + "\n")
    report.update(result="PASS", summary=f"{len(rows_)} goals refreshed -> {dst.name}: " + ", ".join(
        f"{k} {v['drift_mm']} mm/{v['drift_deg']} deg{' FLAG' if v['flag'] else ''}" for k, v in rows_.items()))
    return


def write_summaries(out, rows, L, names, todo, meta, np):
    """summary.csv (per dish and gap) and highest.json (per dish: the highest gap with a seated repeat, the highest with all
    repeats seated). (per, highest)"""
    per = {}
    for x in rows:
        per.setdefault((x["dish"], x["gap_mm"]), []).append(x)
    with open(out / "summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["load", "dish", "kind", "slot", "gap_mm", "drops", "seated", "mouth_down", "settled", "mean_lateral_mm",
                    "mean_dz_mm", "mean_tilt_deg", "earlier_moved_drops", "not_dropped"])
        for (dn, g), xs in sorted(per.items(), key=lambda kv: (names.index(kv[0][0]), kv[0][1])):
            dr = [x for x in xs if x.get("dropped")]
            mean = lambda k: round(float(np.mean([x["error"][k] for x in dr])), 1) if dr else ""        # noqa: E731
            w.writerow([L, dn, meta[dn]["kind"], meta[dn].get("slot"), g, len(dr), sum(x["seated"] for x in dr),
                        sum(x.get("mouth") == "down" for x in dr), sum(x["settled"] for x in dr), mean("lateral_mm"),
                        mean("dz_mm"), mean("tilt_deg"), sum(bool(x["earlier_moved"]) for x in dr), len(xs) - len(dr)])
    highest = {}
    for dn in todo:
        gs = sorted({g for (x, g) in per if x == dn})
        any_ = [g for g in gs if any(y.get("seated") for y in per[(dn, g)])]
        all_ = [g for g in gs if per[(dn, g)] and all(y.get("seated") for y in per[(dn, g)])]
        highest[dn] = {"kind": meta[dn]["kind"], "slot": meta[dn].get("slot"),
                       "highest_any_seated_mm": max(any_) if any_ else None,
                       "highest_all_seated_mm": max(all_) if all_ else None,
                       "at_goal_by_gap": {str(g): f"{sum(bool(y.get('at_goal')) for y in per[(dn, g)])}/{len(per[(dn, g)])}"
                                          for g in gs},
                       "seated_by_gap": {str(g): f"{sum(bool(y.get('seated')) for y in per[(dn, g)])}/{len(per[(dn, g)])}" for g in gs}}
    (out / "highest.json").write_text(json.dumps(highest, indent=1) + "\n")
    return per, highest


def write_perch_summaries(out, rows, L, names, todo, meta, np):
    """--perch-roll: summary.csv (per bowl and tilt), highest.json (per bowl: the counts per tilt and the best tilt) and
    best_tilt.json (the tilt table for frigidaire_robot_episode.py --tilt-table: the best tilt per bowl = the most drops seated
    on the goal's wires and mouth down, ties the lowest perch; gate = that count >= GATE_K). The best-tilt table."""
    per = {}
    for x in rows:
        per.setdefault((x["dish"], x["tilt_deg"]), []).append(x)
    with open(out / "summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["load", "dish", "tilt_deg", "goal_lean_deg", "perch_mm", "perch_body", "roll_pred", "drops", "seated_and_down",
                    "mouth_down", "at_goal", "roll_agrees", "settled", "mean_lateral_mm", "mean_dz_mm", "mean_tilt_deg",
                    "earlier_moved_drops", "not_dropped"])
        for (dn, t), xs in sorted(per.items(), key=lambda kv: (names.index(kv[0][0]), kv[0][1])):
            dr = [x for x in xs if x.get("dropped")]
            mean = lambda k: round(float(np.mean([x["error"][k] for x in dr])), 1) if dr else ""        # noqa: E731
            x0 = xs[0]
            w.writerow([L, dn, t, x0.get("goal_lean_deg"), x0.get("perch_mm"), x0.get("perch_body"), x0.get("roll_pred"), len(dr),
                        sum(bool(x["seated"]) for x in dr), sum(x.get("mouth") == "down" for x in dr),
                        sum(bool(x.get("at_goal")) for x in dr), sum(x.get("roll_agrees") is True for x in dr),
                        sum(bool(x["settled"]) for x in dr), mean("lateral_mm"), mean("dz_mm"), mean("tilt_deg"),
                        sum(bool(x["earlier_moved"]) for x in dr), len(xs) - len(dr)])
    highest, bowls = {}, {}
    for dn in todo:
        ts = sorted({t for (x, t) in per if x == dn})
        cells = {}
        for t in ts:
            dr = [x for x in per[(dn, t)] if x.get("dropped")]
            x0 = per[(dn, t)][0]
            cells[str(t)] = {"drops": len(dr), "seated_and_down": sum(bool(x["seated"]) for x in dr),
                             "mouth_down": sum(x.get("mouth") == "down" for x in dr),
                             "at_goal": sum(bool(x.get("at_goal")) for x in dr), "perch_mm": x0.get("perch_mm"),
                             "roll_pred": x0.get("roll_pred"), "roll_agrees": sum(x.get("roll_agrees") is True for x in dr),
                             "roll_measured": dict(Counter(x.get("roll_measured") for x in dr))}
        ok = [(t, c) for t, c in cells.items() if c["drops"]]
        best = max(ok, key=lambda tc: (tc[1]["seated_and_down"], -(tc[1]["perch_mm"] if tc[1]["perch_mm"] is not None
                                                                  else 1e9))) if ok else None
        highest[dn] = {"kind": meta[dn]["kind"], "slot": meta[dn].get("slot"), "tilts": cells,
                       "best_tilt": None if best is None else float(best[0])}
        bowls[dn] = {"best_tilt": None if best is None else float(best[0]),
                     "seated_down": 0 if best is None else best[1]["seated_and_down"],
                     "mouth_down": 0 if best is None else best[1]["mouth_down"],
                     "drops": 0 if best is None else best[1]["drops"],
                     "perch_mm": None if best is None else best[1]["perch_mm"],
                     "gate": bool(best is not None and best[1]["seated_and_down"] >= GATE_K)}
    (out / "highest.json").write_text(json.dumps(highest, indent=1) + "\n")
    bt = {"load": L, "rule": f"best tilt = the most drops seated on the goal's wires AND mouth down, ties the lowest perch; "
                             f"gate = that count >= {GATE_K} (of 5)", "bowls": bowls,
          "gated": [dn for dn, v in bowls.items() if v["gate"]]}
    (out / "best_tilt.json").write_text(json.dumps(bt, indent=1) + "\n")
    return bt

def rejudge(args, load, out):
    """--rejudge (Kit-free): every dropped row of out/drops.jsonl judged again from its recorded final pose with the current
    seated tests; the racks as the run had them (the dish's rack extended = the goal records' frames, the other pushed in:
    frigidaire_robot_reach.frames_for), the earlier dishes at their goal poses (the bowl test's dish support)."""
    import numpy as np
    sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util"), str(ROOT / "code/sampler/scripts"),
                    str(ROOT / "code/sampler/robot/scripts")]
    import fcl
    from types import SimpleNamespace
    import arrangement_optimization.frigidaire_bench as B
    import frigidaire_robot_episode as EP
    import frigidaire_robot_reach as RC
    from motion_planning import planner as P
    from sampler.initial_state_candidates import COMPONENT_NAMES
    L = load["load"]
    dishes = sorted(load["dishes"], key=lambda d: d["order_index"])
    names = [d["name"] for d in dishes]
    meta = {d["name"]: d for d in dishes}
    goal = {d["name"]: {"position_m": [float(v) for v in d["goal_pose"]["pos"]],
                        "quaternion_xyzw": [float(v) for v in d["goal_pose"]["quat_xyzw"]]} for d in dishes}
    base = json.loads(args.baseline.read_text())
    base = base.get("snapshot", base)["poses"]
    judge = SimpleNamespace(checker=B.bench_checker(), fcl=fcl)
    labels = EP.rack_labels()
    state = {r: {**{nm: base[nm] for nm in COMPONENT_NAMES}, **RC.frames_for(B, r)} for r in ("LowerRack", "UpperRack")}
    F_goal = {r: P.pose_T(load["frames"][r]) for r in ("LowerRack", "UpperRack")}
    from asset.loading import visual_points
    centroids = B.kind_centroids({k: visual_points(B.ASSETS / f"{k}.usda") for k in B.KINDS})
    rows = [json.loads(x) for x in (out / "drops.jsonl").read_text().splitlines() if x.strip()]
    changed, cur = 0, None
    for rec in rows:
        if not rec.get("dropped"):
            continue
        n, kind, rack = rec["dish"], rec["kind"], rec["rack"]
        if cur != rack:
            judge.checker.update_components(state[rack])
            cur = rack
        T = P.pose_T(rec["final_pose"])
        mouth = None if kind == "plate" else ("down" if T[2, 2] < 0 else "up")
        F = {r: P.pose_T(state[rack][r]) for r in ("LowerRack", "UpperRack")}
        if kind == "plate":
            st = EP.seated_test(judge, kind, n, T, F["LowerRack"] @ np.linalg.inv(F_goal["LowerRack"]) @ P.pose_T(goal[n]),
                                meta[n]["slot"], labels, np, centroid=centroids[kind])
            seated = st["seated"]
        elif kind == "bowl":
            others = {c: EP._pose_of(F[meta[c]["rack"]] @ np.linalg.inv(F_goal[meta[c]["rack"]]) @ P.pose_T(goal[c]))
                      for c in names[:names.index(n)]}
            st = EP.bowl_seated_test(judge, kind, n, T, n, np, others=others,
                                     T_goal=F[rack] @ np.linalg.inv(F_goal[rack]) @ P.pose_T(goal[n]), centroid=centroids[kind])
            seated = bool(st["seated_wires"] and mouth == "down")
        else:
            continue                                   # cups: the pose test, unchanged
        if "seated_as_run" not in rec:
            rec["seated_as_run"] = rec.get("seated")
        changed += bool(seated) != bool(rec["seated_as_run"])
        rec.update(seated=bool(seated), seated_detail=st, rejudged="round 14 seated tests (Kit-free, --rejudge)")
    (out / "drops.jsonl").write_text("".join(json.dumps(x, default=str) + "\n" for x in rows))
    todo = [n for n in names if any(x["dish"] == n for x in rows)]
    per, highest = write_summaries(out, rows, L, names, todo, meta, np)
    rp = out / "report.json"
    report = json.loads(rp.read_text()) if rp.is_file() else {}
    report["rejudged"] = {"changed": changed, "seated": sum(bool(x.get("seated")) for x in rows),
                          "seated_as_run": sum(bool(x.get("seated_as_run", x.get("seated"))) for x in rows),
                          "rule": rejudge.__doc__.split("\n")[0]}
    rp.write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(f"[RESULT] PASS drop curve rejudge {L}: {changed} verdicts changed, seated {report['rejudged']['seated']} "
          f"(as run {report['rejudged']['seated_as_run']}); highest any-seated per dish "
          + ", ".join(f"{k} {v['highest_any_seated_mm']}" for k, v in highest.items()), flush=True)


if __name__ == "__main__":
    main()
