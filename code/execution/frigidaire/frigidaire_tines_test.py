#!/usr/bin/env python3
"""Robot-free tines test: does the rack alone tilt a level bowl at the right-side lower sites?

    code/util/run_kit.sh code/execution/frigidaire/frigidaire_tines_test.py --headless --n 5 \
        --out data/results/robot/exhaustive_w1/tines

Plan plans/2026-10-26-hardware-exhaustive.md, window 1. No robot in the scene. Per site of --reach (the limits round's
reach.json), per trial: the racks set as in the robot trials (a lower-rack site has the upper rack pushed in), the bowl
teleported mouth-up and level to the site's clear-height pose plus the release offset (flags.release_above_m) with a
seeded +-5 mm x, y perturbation, then the benchmark's per-move settle (frigidaire_bench_kit SETTLE_TICKS with the
drift window, as the episode's bench_settle) and the final tilt; a further rest to 5 s gives the rest tilt.
Verdict per site: the bowl tilts past TILT_FAIL_DEG on its own (the rack causes the failure) or stays level (the robot).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
SITES = ("lower_frontright@x+0.06", "lower_frontright@x+0.08", "lower_frontright@x+0.13", "lower_rearright@x-0.02")
TILT_FAIL_DEG = 15.
REST_TICKS = 600                       # 5 s at 120 Hz from the teleport (the rest tilt)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--sites", default=",".join(SITES))
    ap.add_argument("--reach", type=Path, default=ROOT / "data/results/robot/limits_v1/reach.json")
    ap.add_argument("--out", type=Path, default=ROOT / "data/results/robot/exhaustive_w1/tines")
    ap.add_argument("--usd", type=Path, default=ROOT / "data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc")
    sys.path[:0] = [str(ROOT / "code/src")]
    from dishsim.app import add_args, launch   # Kit-free; boots Kit below
    add_args(ap)
    args = ap.parse_args()
    args.out = args.out if args.out.is_absolute() else ROOT / args.out
    if args.out.exists():
        raise SystemExit(f"[RESULT] FAIL refusing to overwrite {args.out}")
    args.out.mkdir(parents=True)
    app = launch(args)
    sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/planner/frigidaire"),
                    str(ROOT / "code/execution/frigidaire")]
    import importlib.util
    import numpy as np
    spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "code/planner/frigidaire/frigidaire_bench.py")
    B = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(B)
    import frigidaire_bench_kit as K
    from dishsim import rearrange as R
    from dishsim_frigidaire.initial_state_runtime import IsaacInitialStateBackend
    from dishsim_frigidaire.random_poses import LIMITS, source_geometry_domains
    from dishsim_frigidaire.random_pose_runtime import EXTENSION, RACK_JOINT
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.robot import flags as F
    from dishsim_frigidaire.robot.rig import T_of
    import frigidaire_robot_episode as EP             # limits_instance only (stdlib at module level)

    inst = EP.limits_instance("bowl")
    oid, kind = "bowl_01", "bowl"
    release_above = F.resolve("headline").release_above_m
    centroid = B.kind_centroids({k: visual_points(B.ASSETS / f"{k}.usda") for k in B.KINDS})[kind]
    entries = [{"object_id": o["object_id"], "kind": o["kind"], "rack": o.get("rack", "LowerRack"), "pose_world": o["pose_world"]}
               for o in inst["objects"]]

    class Backend(IsaacInitialStateBackend):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.assignments = {}                     # the bowl starts on the counter (as the episode's counter dishes)

    slab = ("/World/Counter", tuple(B.COUNTER["size_m"]), tuple(B.COUNTER["center_m"]))
    backend = Backend(args.usd, args.out / "_kit", device="cpu", domains=source_geometry_domains(),
                      deadline=time.monotonic() + 3600., app=app, candidates=entries, extra_statics=[slab],
                      tableware=B.tableware())
    backend.restore(inst["baseline"])
    for _ in range(12):
        backend.tick()
    backend.set_rigid_pose(backend.objects[oid], inst["initial_snapshot"]["poses"][oid])
    backend.loaded = True
    backend.assignments = {}
    for _ in range(K.RESET_TICKS):
        backend.tick()

    def set_racks(rack):
        want = {r: EXTENSION[r] for r in ("LowerRack", "UpperRack")}
        want["UpperRack" if rack == "LowerRack" else "LowerRack"] = 0.
        js = backend.joints()
        move = {RACK_JOINT[r]: v for r, v in want.items() if abs(js[RACK_JOINT[r]] - v) > .005}
        if move:
            backend.ramp(move, f"tines_racks_{rack}")

    def T_now():
        p = backend.poses()[oid]
        return T_of(p["position_m"], p["quaternion_xyzw"])

    tilt = lambda T: round(float(np.degrees(np.arccos(np.clip(T[2, 2], -1., 1.)))), 1)
    reach = json.loads(args.reach.read_text())
    named = {s["site"]: s for s in reach["sites"][kind]}
    rng = np.random.default_rng(args.seed)
    rows = []
    t_run = time.monotonic()
    for name in [x for x in args.sites.split(",") if x]:
        site = named[name]
        lv = site["world"]["level"]
        for t in range(args.n):
            set_racks(site["rack"])
            d = rng.uniform(-.005, .005, 2)
            p = [lv["position_m"][0] + d[0], lv["position_m"][1] + d[1], lv["position_m"][2] + release_above]
            T_goal = T_of(lv["position_m"], [0., 0., 0., 1.])
            backend.set_rigid_pose(backend.objects[oid], {"position_m": p, "quaternion_xyzw": [0., 0., 0., 1.]})
            hist = []
            for s in range(K.SETTLE_TICKS):           # the benchmark's per-move settle (episode bench_settle)
                backend.tick()
                if s >= K.SETTLE_TICKS - K.WINDOW_TICKS:
                    hist.append(T_now())
            T = hist[-1]
            drift_p = float(np.linalg.norm(hist[-1][:3, 3] - hist[0][:3, 3]))
            drift_deg = R.rot_angle_deg(hist[0], hist[-1])
            dev_p = float(np.linalg.norm(T[:3, 3] - T_goal[:3, 3]))
            deep = max((v for k_, v in backend.latest_contact["depths_m"].items() if oid in k_.split("|")), default=0.)
            settled = bool(drift_p <= R.STABLE_POS_M and drift_deg <= R.STABLE_ROT_DEG and dev_p <= K.DEV_MAX_M
                           and deep < LIMITS["peak_penetration_m"])
            dist = B.goal_distance(centroid, T, T_goal)
            for _ in range(REST_TICKS - K.SETTLE_TICKS):
                backend.tick()
            T_rest = T_now()
            row = {"site": name, "rack": site["rack"], "trial": t, "perturb_mm": [round(float(v) * 1e3, 2) for v in d],
                   "teleport_z_m": round(p[2], 4), "final_tilt_deg": tilt(T), "rest_tilt_deg": tilt(T_rest),
                   "tilted": tilt(T) > TILT_FAIL_DEG,
                   "settle": {"drift_mm": round(drift_p * 1e3, 2), "drift_deg": round(drift_deg, 2),
                              "settle_dev_mm": round(dev_p * 1e3, 1), "contact_depth_mm": round(deep * 1e3, 2), "passed": settled},
                   "at_target": bool(B.within_goal(kind, dist)),
                   "distance": [round(dist[0] * 1e3, 1), round(dist[1] * 1e3, 1), round(dist[2], 1)],
                   "final_pose": backend.poses()[oid]}
            rows.append(row)
            with open(args.out / "trials.jsonl", "a") as f:
                f.write(json.dumps(row) + "\n")
            print(f"[INFO] {name} t{t}: tilt {row['final_tilt_deg']} (rest {row['rest_tilt_deg']}) settle {settled} "
                  f"at target {row['at_target']} drop {round((p[2] - T[2, 3]) * 1e3, 1)} mm", flush=True)
    summary = {}
    for name in dict.fromkeys(r["site"] for r in rows):
        rs = [r for r in rows if r["site"] == name]
        n_tilt = sum(r["tilted"] for r in rs)
        summary[name] = {"n": len(rs), "tilted_past_15": n_tilt, "median_tilt_deg": float(np.median([r["final_tilt_deg"] for r in rs])),
                         "median_rest_tilt_deg": float(np.median([r["rest_tilt_deg"] for r in rs])),
                         "settle_and_at_target": sum(r["settle"]["passed"] and r["at_target"] for r in rs),
                         "verdict": "rack tilts it" if n_tilt * 2 > len(rs) else "stays level (robot causes the failure)"}
    (args.out / "tines.json").write_text(json.dumps({"sites": summary, "rule": f"tilt > {TILT_FAIL_DEG} deg after the "
                                                     "benchmark settle", "release_above_m": release_above,
                                                     "seconds_wall": round(time.monotonic() - t_run, 1)}, indent=1) + "\n")
    print(f"[RESULT] PASS tines test {len(rows)} trials: {summary}", flush=True)
    try:
        from dishsim.media import release_sim_for_close
        release_sim_for_close()
    finally:
        app.close()


if __name__ == "__main__":
    main()
