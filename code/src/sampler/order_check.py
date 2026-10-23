#!/usr/bin/env python3
"""Which pose model gives the pre-build access DAG (plans/2026-10-22-dag-pose-model.md): per valid sampler record
(entries_chosen, entries_settled, access.hits, witness.order) and pose model, sampler/order.py's dag_order at the model's
poses against the record's certified access hits. A model passes a load when its access edges equal the hits exactly and
its DAG (access + support) is acyclic. Models, each a full set of rack-local entries:
    P  packed: entries_chosen (what the build was given)
    L  lowest clear step: each dish alone, the insertion move's descent (frigidaire_bench.insertion_release: from 20 mm
       above the rim in 5 mm steps, the 1 mm allowance) against the appliance only, at the lowest clear step; plates from
       upright_pose with the 3 mm lean bias (move_local), bowls and cups from the packed pose less the variant's lift
    S  settled: entries_settled
    M  mixed: plates from S, bowls and cups from L
    C  control, not a model: the certify's own computation (access_hits at certify.poses_world with moves_world)
    F  foot pivot to contact (plans/2026-10-22-plate-pose-model.md): each plate as in L (upright, lowered against the
       appliance alone), then rotated about the rack-y axis through its lowest visual point toward its lean side in 1 deg
       steps to the last step within the 1 mm allowance against the appliance, at most PIVOT_MAX_DEG; bowls and cups at S
    A  the same pivot by the packed variant's lean angle (lean-28 = 28 deg); bowls and cups at S
    R  rest table (plans/2026-10-22-plate-rest-pose-dag.md): each plate at the lone-plate rest pose of its slot and lean
       side (order.REST_TABLE), bowls and cups packed; the DAG is order.py's rest model (plate pairs and, since
       plans/2026-10-22-bowl-plate-edges.md, bowl or cup against plate both ways; no support), so R's plain hit count
       misses bowl-bowl hits by design: judge it by the plate pairs, its bowl-plate edges by the witness order (none may
       point against it: bowl_plate_breaks)
Every model but C also gets each plate's pose error against entries_settled (rack-local: tilt = the plate axis from the
rack's up, position = the pose origin, mode 'tines' when its lowest visual point sits more than TINES_ABOVE_M over the
plate floor, else 'gap'). Access edges are compared only on valid records (the hits), also restricted to plate pairs.
Kit-free; needs numpy, fcl and pxr (the container):
    code/util/run_py.sh code/src/sampler/order_check.py --records R1 R2 ... --models P,L,S,M,C --out DIR
Writes DIR/<load_id>.<model>.json and DIR/table.md."""
import argparse
import json
import math
from multiprocessing import Pool
import os
from pathlib import Path
import sys
import time
import traceback

import numpy as np

os.environ.setdefault("OMP_NUM_THREADS", "1")
ROOT = Path(__file__).resolve().parents[3]
if __name__ == "__main__":                       # as a script: code/src instead of the script dir
    sys.path[:1] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
from sampler import order  # noqa: E402

PIVOT_MAX_DEG, TINES_ABOVE_M, TILT_MAX_DEG, POS_MAX_MM = 60, .030, 3., 10.     # the plan's 60 deg cap and pass limits


def plate_points():
    B, _, _, _ = order._mods()
    return B.HX().visual_points(B.ASSETS / "plate.usda")


def pivot(entries, frames, model):
    """Model F or A of the plate ``entries`` (module docstring) and {id: {lift_mm, stop, blocked, angle_deg, contact}}."""
    B, _, _, P = order._mods()
    from dishsim_frigidaire.loading import quaternion_xyzw, rotation
    checker = B.release_checker()
    appliance = B.appliance_bodies(checker, frames)
    pts = plate_points()
    low, info = lowest_clear(entries, frames)
    out = []
    for e in low:
        R0, p0, side = B.quat_matrix(e["quaternion_xyzw"]), np.asarray(e["position"], dtype=float), B.lean_side(e["variant"])
        w = pts @ R0.T + p0
        foot = w[w[:, 2].argmin()]                          # the axis: through this point, along the rack's y

        def posed(a):
            Ry = rotation("Y", math.radians(side * a))
            return {"position_m": (foot + Ry @ (p0 - foot)).tolist(), "quaternion_xyzw": quaternion_xyzw(Ry @ R0)}
        a, hit = 0, []
        if model == "A":
            a = abs(float(e["variant"].split("lean", 1)[1].split("_", 1)[0]))
        else:
            while a < PIVOT_MAX_DEG:
                body = checker._body("plate", P.world_from_local(frames[e["rack"]], posed(a + 1)), e["id"])
                hit = B.release_blockers(checker, body, [], appliance)
                if hit:
                    break
                a += 1
        pose = posed(a)
        out.append({**e, "position": pose["position_m"], "quaternion_xyzw": pose["quaternion_xyzw"]})
        info[e["id"]].update(angle_deg=a, contact=hit)
    return out, info


def plate_errors(entries, settled):
    """{plate id: slot, lean, tilt [model, settled] deg, tilt_err deg (between the plate axes), d_mm (model - settled,
    rack-local), mm, low_mm [model, settled] (lowest visual point over the plate floor), mode [model, settled]}."""
    from dishsim_frigidaire.loading import LOWER_PLATE_FLOOR
    B, _, _, _ = order._mods()
    pts, out = plate_points(), {}
    for e in entries:
        if e["kind"] != "plate":
            continue
        s = settled[e["id"]]
        R = [B.quat_matrix(x["quaternion_xyzw"]) for x in (e, s)]
        low = [float((pts @ r.T)[:, 2].min() + x["position"][2] - LOWER_PLATE_FLOOR) for r, x in zip(R, (e, s))]
        d = (np.asarray(e["position"], dtype=float) - np.asarray(s["position"], dtype=float)) * 1e3
        out[e["id"]] = {"slot": e["slot"], "lean": e["variant"].split("lean", 1)[1].split("_", 1)[0],
                        "tilt": [round(math.degrees(math.acos(np.clip(r[2, 2], -1., 1.))), 1) for r in R],
                        "tilt_err": round(math.degrees(math.acos(np.clip(R[0][:, 2] @ R[1][:, 2], -1., 1.))), 1),
                        "d_mm": [round(float(v), 1) for v in d], "mm": round(float(np.linalg.norm(d)), 1),
                        "low_mm": [round(v * 1e3, 1) for v in low],
                        "mode": ["tines" if v > TINES_ABOVE_M else "gap" for v in low]}
    return out


def cyclic(edges):
    """The nodes on or behind a cycle of ``edges`` (what Kahn's algorithm cannot order); [] when acyclic."""
    nodes, done = {n for e in edges for n in e}, set()
    need = {n: {a for a, b in edges if b == n} for n in nodes}
    while ready := {n for n in nodes - done if need[n] <= done}:
        done |= ready
    return sorted(nodes - done)


def lowest_clear(entries, frames):
    """Model L of ``entries`` (module docstring) and {id: {lift_mm above the move pose, stop, blocked}}."""
    B, _, _, P = order._mods()
    checker = B.release_checker()
    appliance = B.appliance_bodies(checker, frames)
    out, info = [], {}
    for e in entries:
        if e["kind"] == "plate":
            local = B.upright_pose(e["slot"], B.lean_side(e["variant"]))
        else:
            lift = float(e["variant"].rsplit("_lift", 1)[1]) if "_lift" in (e.get("variant") or "") else 0.
            local = {"position_m": [*e["position"][:2], e["position"][2] - lift], "quaternion_xyzw": e["quaternion_xyzw"]}
        up, rim_z, top_z = B.rack_heights(frames, e["rack"])
        m = B.insertion_release(checker, [(e["kind"], P.world_from_local(frames[e["rack"]], local), e["id"])], [],
                                appliance, up, rim_z, top_z)
        h = m["start"] if m["lowest"] is None else m["lowest"]       # a blocked start stays at the start (flagged)
        x, y, z = local["position_m"]                                 # the rack's local z is its up axis
        out.append({**e, "position": [x, y, z + h], "quaternion_xyzw": [float(v) for v in local["quaternion_xyzw"]]})
        info[e["id"]] = {"lift_mm": round(h * 1e3, 1), "stop": m["stop"], "blocked": m["blocked"]}
    return out, info


def check(job):
    """One (record path, model): the model's DAG against the record's hits (or the error)."""
    path, model = job
    rec = json.loads(Path(path).read_text())
    hits = sorted({(h["a"], h["b"]) for h in rec["access"]["hits"]}) if rec.get("valid") else None
    out = {"load_id": rec["load_id"], "record": path, "model": model, "hits": hits}
    t0 = time.monotonic()
    try:
        B, _, DK, P = order._mods()
        frames = json.loads(order.BASELINE.read_text())["snapshot"]["poses"]
        chosen, settled = rec["entries_chosen"], {e["id"]: e for e in rec["entries_settled"]}
        cert = (rec.get("certify") or {}).get("poses_world")
        if model == "C":
            got, _ = DK.access_hits(B, B.release_checker(), {e["id"]: e["kind"] for e in chosen},
                                    {e["id"]: e["rack"] for e in chosen}, rec["moves_world"], {**frames, **cert},
                                    [e["id"] for e in chosen])
            dag = {"access": sorted({(h["a"], h["b"]) for h in got}), "support": [], "cycle": None, "order": None,
                   "layers": None}
        else:
            entries = chosen if model == "P" else [settled[e["id"]] for e in chosen]
            if model in "LM":
                entries, info = lowest_clear(chosen, frames)
                if model == "M":
                    entries = [settled[e["id"]] if e["kind"] == "plate" else e for e in entries]
                out["lowest_clear"] = {e["id"]: info[e["id"]] for e in entries if model == "L" or e["kind"] != "plate"}
            if model in "FA":
                plates, out["pivot"] = pivot([e for e in chosen if e["kind"] == "plate"], frames, model)
                plates = {e["id"]: e for e in plates}
                entries = [plates.get(e["id"], e) for e in entries]
            if model == "R":
                entries = order.rest_entries(chosen)
            if model == "S" and hits is not None:
                out["settled_vs_certify_mm"] = round(1e3 * max(math.dist(P.world_from_local(
                    frames[e["rack"]], {"position_m": e["position"], "quaternion_xyzw": e["quaternion_xyzw"]})["position_m"],
                    cert[e["id"]]["position_m"]) for e in entries), 1)
            out["plates"] = plate_errors(entries, settled)
            if hits is None:                     # not a valid record: pose errors only (hits are judged on valid loads)
                return {**out, "seconds": round(time.monotonic() - t0, 1)}
            dag = order.dag_order(chosen, 0, pose_model="rest") if model == "R" else order.dag_order(entries, 0)
    except Exception:
        return {**out, "error": traceback.format_exc()}
    acc, sup = [tuple(x) for x in dag["access"]], [tuple(x) for x in dag["support"]]
    pos = {i: n for n, i in enumerate(rec["witness"]["order"])}
    out.update(access=acc, support=sup, missed=sorted(set(hits) - set(acc)), spurious=sorted(set(acc) - set(hits)),
               cycle=dag["cycle"], order=dag["order"], layers=dag["layers"],
               witness_breaks=[e for e in acc + sup if pos[e[0]] > pos[e[1]]], seconds=round(time.monotonic() - t0, 1))
    out["pass"] = not out["missed"] and not out["spurious"] and out["cycle"] is None
    kind = {e["id"]: e["kind"] for e in rec["entries_chosen"]}
    pp = lambda es: sorted((a, b) for a, b in es if kind[a] == kind[b] == "plate")          # noqa: E731
    out.update(plate_hits=pp(hits), plate_missed=pp(out["missed"]), plate_spurious=pp(out["spurious"]),
               plate_cycle=cyclic(pp(acc + sup)))
    out["plate_pass"] = not out["plate_missed"] and not out["plate_spurious"] and not out["plate_cycle"]
    bp = [e for e in acc if (kind[e[0]] == "plate") != (kind[e[1]] == "plate")]     # a bowl or cup against a plate
    out.update(bowl_plate=bp, bowl_plate_breaks=[e for e in bp if pos[e[0]] > pos[e[1]]])
    return out


def short(load_id):
    return "s" + load_id.rsplit("__", 1)[1] if "__" in load_id else load_id


def edges(es):
    return " ".join(f"{a}>{b}" for a, b in es) or "-"


def cell(r):
    if "error" in r:
        return "error"
    if "pass" not in r:
        return "poses only"
    bits = ["pass" if r["pass"] else f"miss {len(r['missed'])}, spur {len(r['spurious'])}"]
    return ", ".join(bits + (["cycle " + ">".join(r["cycle"])] if r["cycle"] else []))


def table(records, models, rows):
    plates = plate_table(records, models, rows)
    records = [p for p in records if rows[(p, models[0])]["hits"] is not None]      # the valid records: hits
    first = {p: rows[(p, models[0])] for p in records}
    lines = ["| load | hits | " + " | ".join(models) + " |", "|---|---|" + "---|" * len(models)]
    lines += [f"| {short(first[p]['load_id'])} | {len(first[p]['hits'])} | "
              + " | ".join(cell(rows[(p, m)]) for m in models) + " |" for p in records]
    lines.append(f"| k of {len(records)} | | " + " | ".join(str(sum(bool(rows[(p, m)].get("pass")) for p in records))
                                                          for m in models) + " |")
    lines += ["", "Per load and model (a>b = a before b): missed and spurious access edges, cycle, support edges, the "
                  "edges the witness order breaks."]
    for p in records:
        for m in models:
            r = rows[(p, m)]
            lines.append(f"- {short(r['load_id'])} {m}: " + (r["error"].strip().splitlines()[-1] if "error" in r else
                         f"missed {edges(r['missed'])}; spurious {edges(r['spurious'])}; cycle {r['cycle']}; support "
                         f"{edges(r['support'])}; witness breaks {edges(r['witness_breaks'])}; bowl-plate "
                         f"{edges(r['bowl_plate'])} (witness breaks {edges(r['bowl_plate_breaks'])})"))
    return "\n".join(lines + plates) + "\n"


def plate_table(records, models, rows):
    """The plate sections (plans/2026-10-22-plate-pose-model.md): plate pairs on the valid records, every plate's pose
    error per model, the verdict per model."""
    ms = [m for m in models if m != "C" and any("plates" in rows[(p, m)] for p in records)]
    if not ms:
        return []
    valid = [p for p in records if rows[(p, ms[0])]["hits"] is not None]
    lines = ["", "Plate pairs on the valid records (a>b = a before b; plate cycle = plates no order of the plate-pair "
                 "access and support edges places):", "| load | plate hits | " + " | ".join(ms) + " |",
             "|---|---|" + "---|" * len(ms)]
    for p in valid:
        lines.append(f"| {short(rows[(p, ms[0])]['load_id'])} | {len(rows[(p, ms[0])]['plate_hits'])} | " + " | ".join(
            "error" if "plate_pass" not in rows[(p, m)] else "pass" if rows[(p, m)]["plate_pass"] else
            f"miss {len(rows[(p, m)]['plate_missed'])}, spur {len(rows[(p, m)]['plate_spurious'])}"
            + (f", cycle {len(rows[(p, m)]['plate_cycle'])}" if rows[(p, m)]["plate_cycle"] else "") for m in ms) + " |")
        lines += [f"- {short(rows[(p, m)]['load_id'])} {m}: missed {edges(rows[(p, m)].get('plate_missed', []))}; spurious "
                  f"{edges(rows[(p, m)].get('plate_spurious', []))}; plate cycle {rows[(p, m)].get('plate_cycle') or '-'}"
                  for m in ms]
    lines += ["", f"Per plate (rack-local): tilt = the plate axis from the rack's up [deg], model / settled; err = the angle "
                  f"between the two axes; mode by the lowest visual point (tines when more than {TINES_ABOVE_M * 1e3:.0f} mm "
                  f"over the plate floor; low = that height, model / settled); d = model - settled pose origin [mm]; F and A "
                  f"add the descent's stop lift, the pivot angle and what ended it.",
              "| load | plate | slot | lean | model | tilt | err | mode | low mm | dx dy dz | mm | stop, pivot |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for p in records:
        ids = sorted({i for m in ms for i in rows[(p, m)].get("plates", {})})
        for i in ids:
            for m in ms:
                e, v = rows[(p, m)].get("plates", {}).get(i), rows[(p, m)].get("pivot", {}).get(i)
                if e is None:
                    lines.append(f"| {short(rows[(p, m)]['load_id'])} | {i} | | | {m} | error | | | | | | |")
                    continue
                piv = "" if v is None else (f"{v['lift_mm']} mm, {v['angle_deg']:g} deg"
                                            + (f" ({' '.join(v['contact'])})" if v.get("contact") else ""))
                lines.append(f"| {short(rows[(p, m)]['load_id'])} | {i} | {e['slot']} | {e['lean']} | {m} | "
                             f"{e['tilt'][0]} / {e['tilt'][1]} | {e['tilt_err']} | {e['mode'][0]} / {e['mode'][1]} | "
                             f"{e['low_mm'][0]} / {e['low_mm'][1]} | {' '.join(f'{d:g}' for d in e['d_mm'])} | {e['mm']} | "
                             f"{piv} |")
    lines += ["", f"Verdict (pass = the plate pairs reproduced exactly with no plate cycle on every valid record, and every "
                  f"plate within {TILT_MAX_DEG:g} deg and {POS_MAX_MM:g} mm of its settled pose on every record):"]
    for m in ms:
        ps = [(rows[(p, m)]["load_id"], i, e) for p in records for i, e in rows[(p, m)].get("plates", {}).items()]
        errors = sum("error" in rows[(p, m)] for p in records)
        ok = [x for x in ps if x[2]["tilt_err"] <= TILT_MAX_DEG and x[2]["mm"] <= POS_MAX_MM]
        pairs = all(rows[(p, m)].get("plate_pass") for p in valid)
        worst_t = max(ps, key=lambda x: x[2]["tilt_err"], default=None)
        worst_d = max(ps, key=lambda x: x[2]["mm"], default=None)
        lines.append(f"- {m}: plate pairs {'pass' if pairs else 'FAIL'} on {len(valid)} valid record(s); {len(ok)} of "
                     f"{len(ps)} plates within the limits" + (f"; worst tilt {worst_t[2]['tilt_err']} deg ({worst_t[0]} "
                     f"{worst_t[1]}), worst position {worst_d[2]['mm']} mm ({worst_d[0]} {worst_d[1]})" if ps else "")
                     + (f"; {errors} errors" if errors else "")
                     + f" -> {'PASS' if pairs and len(ok) == len(ps) and not errors and ps else 'FAIL'}")
    for bank in ("front", "rear") if "R" in ms else ():          # the rest table against each bank's settled plates
        es = [e for p in records for e in rows[(p, "R")].get("plates", {}).values() if f"_{bank}_" in e["slot"]]
        med = lambda f: round(float(np.median([f(e) for e in es])), 1)          # noqa: E731
        if es:
            lines.append(f"- R {bank} bank ({len(es)} plates), medians: tilt table {med(lambda e: e['tilt'][0])} / settled "
                         f"{med(lambda e: e['tilt'][1])} deg, err {med(lambda e: e['tilt_err'])} deg; table - settled d "
                         f"{med(lambda e: e['d_mm'][0])} {med(lambda e: e['d_mm'][1])} {med(lambda e: e['d_mm'][2])} mm, "
                         f"|d| {med(lambda e: e['mm'])} mm")
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--records", nargs="+", required=True, help="sampler records (hits judged on the valid ones)")
    ap.add_argument("--models", default="P,L,S,M", help="comma-separated, of P L S M C F A R")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    models = args.models.split(",")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rows, t0 = {}, time.monotonic()
    with Pool(min(8, os.cpu_count() or 1)) as pool:
        for r in pool.imap_unordered(check, [(p, m) for p in args.records for m in models]):
            (out / f"{r['load_id']}.{r['model']}.json").write_text(json.dumps(r, indent=1) + "\n")
            rows[(r["record"], r["model"])] = r
            print(f"{short(r['load_id'])} {r['model']}: {cell(r)}" + ("" if "error" in r else f" ({r['seconds']} s)"),
                  flush=True)
    (out / "table.md").write_text(table(args.records, models, rows))
    print((out / "table.md").read_text())
    errors = sum("error" in r for r in rows.values())
    print(f"[RESULT] {'FAIL' if errors else 'PASS'} {len(rows)} checks, {errors} errors, {time.monotonic() - t0:.0f} s")


if __name__ == "__main__":
    main()
