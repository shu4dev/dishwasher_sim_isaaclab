#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Where a lone plate rests after the insertion move (boot-first; plans/2026-10-22-plate-rest-pose-dag.md).

One plate alone in the Frigidaire twin (the benchmark baseline's extended, empty racks) is dropped by the sampler's
insertion move exactly as build_sequence does it (frigidaire_bench.upright_pose(slot, side) with the 3 mm lean bias,
lowered from 20 mm above the rim in 5 mm steps against the appliance, released 20 mm above the stop, settled 150 + 60
ticks at 120 Hz, parked 60 ticks before), into every lower-rack plate gap (the plate families' front and rear gaps) from
each lean side (-1, +1), all in one Kit session. Writes <out>/plate_rest.json: per (bank, gap, side) the rest pose
rack-local in the baseline's LowerRack (the frame of the records' entries_settled: position, quaternion XYZW), tilt (the
plate axis from the rack's up, 90 = upright), the lowest visual point over the plate floor (mode 'tines' above 30 mm,
else 'gap'), the settle deviation (rest origin from the move pose origin), the drift over the last 60 ticks, the contact
depth and the move's path; with the code SHA and the move's constants.
    code/util/run_kit.sh code/planner/frigidaire/frigidaire_plate_rest.py --headless --out data/results/sampler/plate_rest
--kinds bowl,cup (plans/2026-10-22-bowl-rest-pose-dag.md): one bowl or cup alone, the same move and settle, once per
sampler.order.bowl_key of the packer's level-0 bowl and cup families under the level-0 masks (frigidaire_hotec_top5: the
lifts collapsed, and in the lower rack the x too): the key's candidate at the median of its x values (its lowest lift),
moved with that lift removed (the lift only sets the release height). Writes <out>/bowl_rest.json: per key the rest pose
(rack-local in the baseline's extended rack of the dish), x_m (the candidate's x: order.py moves a lower-rack bowl's row
along x by the entry's x minus x_m), tilt (the dish axis from the rack's up), low_mm (the lowest visual point, rack-local
z), the settle deviation, drift, contact and the move's path. --part K --parts N settles every N-th key from K into
bowl_rest.partK.json; --merge (Kit-free, code/util/run_py.sh) joins the parts into bowl_rest.json.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[3]
PARK_TICKS, TINES_ABOVE_M = 60, .030
DISHES = {"plate": ("plate_01", "LowerRack"), "bowl": ("bowl_01", "LowerRack"), "cup": ("cup_01", "UpperRack")}


def code_sha():
    """HEAD's commit read from .git (git itself fails in the container)."""
    head = (ROOT / ".git/HEAD").read_text().strip()
    if not head.startswith("ref: "):
        return head
    ref = head[5:]
    if (ROOT / ".git" / ref).is_file():
        return (ROOT / ".git" / ref).read_text().strip()
    packed = ROOT / ".git/packed-refs"
    return next((ln.split()[0] for ln in (packed.read_text().splitlines() if packed.is_file() else [])
                 if ln.endswith(" " + ref)), None)


def plate_jobs(B, X):
    """(row, rack-local move pose) per lower-rack plate gap and lean side: the plate upright in its slot."""
    teeth, _ = X.lower_tine_positions()
    mids = (teeth[:-1] + teeth[1:]) / 2
    for bank, gaps in (("front", X.HL._FRONT_USABLE), ("rear", X.geometry.lower_plate_gaps("rear"))):
        for gap in gaps:
            for side in (-1, 1):
                slot = f"lower_{bank}_{gap:02d}"
                yield ({"kind": "plate", "rack": "LowerRack", "bank": bank, "gap": int(gap), "slot": slot, "side": side,
                        "gap_mid_x_m": float(mids[gap])}, B.upright_pose(slot, side))


def bowl_jobs(kinds):
    """(row, rack-local move pose) per bowl_key of the masked level-0 bowl and cup families (module docstring)."""
    import importlib.util
    from sampler.order import bowl_key
    spec = importlib.util.spec_from_file_location("top5_rest", ROOT / "code/planner/frigidaire/frigidaire_hotec_top5.py")
    T = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(T)
    fam, parts, points = T.level_families(0)
    masks = T.level_masks(fam, parts, points, 0)
    groups = {}
    for kind in kinds:
        for (c, _), ok in zip(fam[kind], masks[kind]):
            if ok:
                groups.setdefault(bowl_key(c), []).append(c)
    lift = lambda c: float(c["variant"].rsplit("_lift", 1)[1]) if "_lift" in c["variant"] else 0.     # noqa: E731
    for key, cs in sorted(groups.items()):
        xs = sorted({round(float(c["position"][0]), 4) for c in cs})
        c = min((c for c in cs if round(float(c["position"][0]), 4) == xs[(len(xs) - 1) // 2]), key=lift)
        x, y, z = (float(v) for v in c["position"])
        yield ({"kind": c["kind"], "rack": c["rack"], "key": key, "slot": c["slot"], "variant": c["variant"], "x_m": x,
                "lift_mm": round(lift(c) * 1e3, 1), "candidates": len(cs), "x_mm": [round(xs[0] * 1e3, 1), round(xs[-1] * 1e3, 1)]},
               {"position_m": [x, y, z - lift(c)], "quaternion_xyzw": [float(v) for v in c["quaternion_xyzw"]]})


def settle_all(args, report):
    import numpy as np
    import frigidaire_bench as B
    import frigidaire_bench_kit as K
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import LOWER_PLATE_FLOOR

    kinds = args.kinds.split(",")
    base = json.loads(Path(args.baseline).read_text())
    snapshot = base.get("snapshot", base)
    X, checker = B.HX(), B.release_checker()
    jobs = list(plate_jobs(B, X) if kinds == ["plate"] else bowl_jobs(kinds))[args.part::args.parts]
    print(f"{len(jobs)} settles (part {args.part} of {args.parts})", flush=True)
    dishes = {k: DISHES[k] for k in kinds}
    backend = K._sequence_scene(args, B, [{"id": i, "kind": k, "rack": r} for k, (i, r) in dishes.items()], snapshot,
                                args.out / f"_kit{args.part}", report)
    if backend is None:
        raise RuntimeError("the extended baseline did not hold")
    pts = {k: X.visual_points(B.ASSETS / f"{k}.usda") for k in kinds}
    rows = []
    for row, local_move in jobs:
        kind, rack = row["kind"], snapshot["poses"][row["rack"]]
        pid, name = dishes[kind][0], row.get("key") or f"{row['slot']} side {row.get('side', 0):+d}"
        for n, (i, _) in enumerate(dishes.values()):          # every dish parked apart
            backend.set_rigid_pose(backend.objects[i], {"position_m": [3., 2. + .5 * n, .3], "quaternion_xyzw": [0., 0., 0., 1.]})
        for _ in range(PARK_TICKS):
            backend.tick()
        live = backend.poses()
        move = P.world_from_local(rack, local_move)
        up, rim_z, top_z = B.rack_heights(live, row["rack"])
        m = B.insertion_release(checker, [(kind, move, pid)], [], B.appliance_bodies(checker, live), up, rim_z, top_z)
        row.update(B.path_info(m))
        rows.append(row)
        if m["poses"] is None:
            print(f"{name}: blocked {m['blocked']}", flush=True)
            continue
        backend.set_rigid_pose(backend.objects[pid], m["poses"][0])
        for t in range(K.SETTLE_TICKS + K.WINDOW_TICKS):
            frames, _ = backend.tick()
            if t == K.SETTLE_TICKS - 1:
                window = frames[pid]
        now = backend.poses()[pid]
        local = P.local_from_world(rack, now)
        R = B.quat_matrix(local["quaternion_xyzw"])
        low = float((pts[kind] @ R.T)[:, 2].min()) + float(local["position_m"][2])
        drift = K.pose_distance(window, now)
        deep = max((d for k, d in backend.latest_contact["depths_m"].items() if pid in k.split("|")), default=0.)
        row.update(position=[float(v) for v in local["position_m"]],
                   quaternion_xyzw=[float(v) for v in local["quaternion_xyzw"]],
                   tilt_deg=round(math.degrees(math.acos(float(np.clip(R[2, 2], -1., 1.)))), 2),
                   settle_dev_mm=round(math.dist(now["position_m"], move["position_m"]) * 1e3, 1),
                   drift_mm=round(drift[0] * 1e3, 2), drift_deg=round(drift[1], 2), contact_mm=round(deep * 1e3, 2))
        if kind == "plate":
            low -= LOWER_PLATE_FLOOR
            row.update(dx_gap_mm=round((float(local["position_m"][0]) - row["gap_mid_x_m"]) * 1e3, 1),
                       low_mm=round(low * 1e3, 1), mode="tines" if low > TINES_ABOVE_M else "gap")
        else:
            row.update(low_mm=round(low * 1e3, 1), d_mm=[round((a - b) * 1e3, 1) for a, b in
                                                        zip(local["position_m"], local_move["position_m"])])
        print(f"{name}: tilt {row['tilt_deg']} deg, xyz {' '.join(f'{v * 1e3:.1f}' for v in row['position'])} mm, low "
              f"{row['low_mm']} mm, dev {row['settle_dev_mm']} mm, drift {row['drift_mm']} mm / {row['drift_deg']} deg, "
              f"contact {row['contact_mm']} mm, stop {row['stop']} at {row['lowest_lift_mm']} mm", flush=True)
    out = {"code_sha": code_sha(), "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
           "baseline": str(args.baseline), "kinds": kinds, "part": [args.part, args.parts],
           "frame": "rack-local in the baseline's extended rack of the dish (the records' entries_settled frame)",
           "move": {"step_mm": B.INSERT_STEP_M * 1e3, "above_rim_mm": B.INSERT_ABOVE_RIM_M * 1e3,
                    "gap_mm": B.INSERT_GAP_M * 1e3, "lean_bias_mm": B.INSERT_LEAN_BIAS_M * 1e3,
                    "settle_ticks": K.SETTLE_TICKS, "window_ticks": K.WINDOW_TICKS, "hz": 120, "park_ticks": PARK_TICKS},
           "entries": rows}
    args.out.mkdir(parents=True, exist_ok=True)
    name = "plate_rest" if kinds == ["plate"] else "bowl_rest"
    (args.out / f"{name}{f'.part{args.part}' if args.parts > 1 else ''}.json").write_text(json.dumps(out, indent=1) + "\n")
    rested = sum("position" in r for r in rows)
    report.update(result="PASS" if rested == len(rows) else "FAIL",
                  reason=f"{rested} of {len(rows)} settles" + (f", {sum(r.get('mode') == 'gap' for r in rows)} in the gap"
                                                               if kinds == ["plate"] else ""))


def merge(out):
    """bowl_rest.json from the parts' tables (standard library)."""
    parts = sorted(Path(out).glob("bowl_rest.part*.json"))
    tables = [json.loads(p.read_text()) for p in parts]
    rows = [r for t in tables for r in t["entries"]]
    (Path(out) / "bowl_rest.json").write_text(json.dumps({**tables[0], "part": None, "parts": [p.name for p in parts],
                                                          "entries": sorted(rows, key=lambda r: r["key"])}, indent=1) + "\n")
    rested = sum("position" in r for r in rows)
    print(f"[RESULT] {'PASS' if rested == len(rows) else 'FAIL'} {len(parts)} parts, {rested} of {len(rows)} settles")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, required=True, help="writes <out>/plate_rest.json")
    parser.add_argument("--baseline", type=Path, default=ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json")
    parser.add_argument("--usd", type=Path, default=ROOT / "data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc")
    parser.add_argument("--max-wall-seconds", type=float, default=1800.)
    parser.add_argument("--kinds", default="plate", help="plate (the plate table) or bowl,cup (the bowl table)")
    parser.add_argument("--part", type=int, default=0)
    parser.add_argument("--parts", type=int, default=1)
    parser.add_argument("--merge", action="store_true", help="Kit-free: join the bowl table's parts")
    if "--merge" in sys.argv:
        return merge(parser.parse_known_args()[0].out)
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device="cpu", headless=True)
    args = parser.parse_args()
    report, app = {"result": "FAIL", "reason": None}, None
    try:
        app = AppLauncher(args).app
        args.app = app
        sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/planner/frigidaire")]
        settle_all(args, report)
    except Exception as exc:                                   # noqa: BLE001
        traceback.print_exc()
        report.update(result="FAIL", reason=f"exception {exc!r}")
    finally:
        print(f"[RESULT] {report['result']} {report['reason']}", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()


if __name__ == "__main__":
    main()
