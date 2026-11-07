"""The hardware-aware goal (plans/2026-11-05-hardware-aware-sampler.md): the fit's packing gated by the robot's Kit-free gate
(code/execution/frigidaire/frigidaire_access_order.py --candidate --direct-only, by subprocess: it forks its pool), each failure
turned into a cut by its collision set, and the packer re-solved, until every dish passes a direct release (HW_MAX_ITER).

Per iteration: the CP-SAT fit (build.fit_pack, level 0 then the level-1 families under the same cuts, never the relaxed solve),
its DAG order, a four-block candidate (goal.poses at the rest model's predicted landing, order.rest_entries; the packed pose where
no rest table exists) under <work>/hwgate/iter_<k>/, the access order in the dag order with the earlier dishes of each rack in the
scene. A failing dish: a plate whose carry RRT found no path is re-gated once at the 180 s narrow retry; then no culprit -> a ban
on its pose (frigidaire_bench.cand_key); culprits -> the deletion filter (the dish gated with one culprit removed, at most
FILTER_CALLS calls): passes -> a pair no-good (a two-key ban), one culprit that still fails -> a ban, else a no-good over the
gated configuration (the dish and its scene). Bans and no-goods persist in <work>/hwgate/bans.json keyed by the level-0 family
digest and the mount name. The start side (initial_check): the start's rack dishes gated by the same admissibility test at their
start poses (the other start dishes of their rack in the scene), the counter dishes by the pick of their kind's hold at the
mount's counter spot as the robot pipeline presents them (the start's stacks are not re-presented).
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from . import ROOT, rel

AO = ROOT / "code/execution/frigidaire/frigidaire_access_order.py"
HW_MAX_ITER = 6
FILTER_CALLS = 3                 # deletion-filter gate calls per dish
NARROW_RETRY_S = 180.            # a carry RRT timeout's one re-gate (collide.NARROW_RETRY_S)
WORKERS = 6
DIRECT_MOVES = ("D-ring", "D-ring-side", "D-side-down", "I4-wall20")
RULE = ("lazy loop (the user's Decisions, 2026-11-05; plates cut by slot since 2026-11-06: a plate pose ban becomes a ban "
        "of every plate pose of its slot, a confirmed plate culprit a cut over every pose pair of the two slots, a confirmed "
        "bowl culprit a cut over the plate's slot x the bowl's site family, two or more culprits no removal clears one clause "
        "over the plate's slot x each culprit's family besides the configuration no-good): CP-SAT packs (level 0, then level 1 under the same cuts; no relaxed "
        "solve, no score ascent), the access order gates the load in its dag order with the earlier dishes of each rack in "
        "the scene at the rest model's poses, direct releases only; a failure is cut by its collision set: no culprit -> a "
        "pose ban, a culprit the deletion filter (<= 3 calls) confirms -> a pair no-good, one culprit that still fails -> a "
        "ban, else a no-good over the gated configuration; a plate's carry RRT timeout is re-gated once at 180 s first; at "
        "most 6 iterations")


def access_order(args, out_json, log_path):
    """frigidaire_access_order.py ``args`` by subprocess (this python, code/src on PYTHONPATH) -> its order.json."""
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(p for p in (str(ROOT / "code/src"), env.get("PYTHONPATH")) if p)
    Path(log_path).parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "a") as f:
        subprocess.run([sys.executable, str(AO), *args], cwd=ROOT, env=env, stdout=f, stderr=subprocess.STDOUT)
    if not Path(out_json).is_file():
        tail = Path(log_path).read_text().splitlines()[-15:]
        raise RuntimeError(f"access order wrote no {rel(out_json)}: " + " | ".join(tail))
    return json.loads(Path(out_json).read_text())


def gate_one(cand, pairs, out_dir, mount, log_path=None, retry_s=0., basket=True):
    """--gate-one for [(dish, scene)] on the candidate ``cand`` -> the run's order.json."""
    out = Path(out_dir) / "order.json"
    n_jobs = sum(5 if n.startswith("plate") else 2 for n, _ in pairs)
    args = ["--candidate", rel(cand), "--mount", rel(mount), "--workers", str(max(1, min(WORKERS, n_jobs))), "--out", rel(out)]
    for n, sc in pairs:
        args += ["--gate-one", n, "--scene", ",".join(sc)]
    if retry_s:
        args += ["--narrow-retry-s", str(retry_s)]
    if not basket:
        args.append("--no-basket")
    return access_order(args, out, log_path or Path(out_dir) / "gate.log")


def candidate(sid, entries, dag, mount, basket=True):
    """The four-block candidate of a packing: goal.poses at the rest model's poses in the baseline's racks, goal.slots the packed
    entries, dag.order and edges from order.dag_order."""
    from . import order as O
    _, _, _, P = O._mods()
    base = json.loads(O.BASELINE.read_text())["snapshot"]
    rest = {e["id"]: e for e in O.rest_entries(entries)}
    poses, model = {}, {}
    for e in entries:
        r = rest[e["id"]]
        poses[e["id"]] = P.world_from_local(base["poses"][e["rack"]], {"position_m": list(r["position"]),
                                                                       "quaternion_xyzw": list(r["quaternion_xyzw"])})
        model[e["id"]] = "packed" if r is e else "rest table"
    slots = {e["id"]: {"rack": e["rack"], "slot": e.get("slot"), "variant": e.get("variant"),
                       "rack_local_pose": {"position_m": list(e["position"]), "quaternion_xyzw": list(e["quaternion_xyzw"])}}
             for e in entries}
    edges = ([{"a": a, "b": b, "kind": "access"} for a, b in dag["access"]]
             + [{"a": a, "b": b, "kind": "support"} for a, b in dag["support"]])
    return {"metadata": {"sample_id": sid, "provenance": {"mount": rel(mount), "basket": bool(basket), "goal_record": None}},
            "initial": None,
            "goal": {"poses": poses, "rack_joints": base["joints"], "slots": slots, "score": None, "pose_model": model},
            "dag": {"nodes": list(dag["order"]), "edges": edges, "layers": dag["layers"], "order": list(dag["order"])}}


def slot_keys(R, level, rack, slot, kind="plate"):
    """The pose keys (frigidaire_bench.cand_key) of every ``kind`` candidate of (rack, slot) in the level's families: all its
    leans and offsets."""
    from . import build as BD
    B = R.T.B
    return sorted({B.cand_key(f[0]) for f in BD.packer(R, level)[0][kind] if f[0]["rack"] == rack and f[0]["slot"] == slot})


def slot_bans(R, level, records):
    """The two-key and one-key bans of the slot records at ``level``: a "slot" record bans every plate pose of its slot, a
    "slot_pair" record every pose pair of its two slots."""
    out = set()
    for r in records:
        if r["type"] in ("slot_site", "family_nogood"):
            continue
        a = slot_keys(R, level, r["rack"], r["slot"])
        if r["type"] == "slot":
            out |= {frozenset([x]) for x in a}
        else:
            b = slot_keys(R, level, r["rack_c"], r["slot_c"])
            out |= {frozenset([x, y]) for x in a for y in b if x != y}
    return out


def family_cuts(R, level, records):
    """The family no-goods of the slot records at ``level`` as key groups (build.fit_solve key_cuts): a "slot_site" record (a
    plate slot x a bowl site family) and a "family_nogood" record (a plate slot x each culprit's family, one clause)."""
    out = []
    for r in records:
        if r["type"] == "slot_site":
            out.append([slot_keys(R, level, r["rack"], r["slot"]), slot_keys(R, level, r["rack_c"], r["slot_c"], "bowl")])
        elif r["type"] == "family_nogood":
            out.append([slot_keys(R, level, g["rack"], g["slot"], g["kind"]) for g in r["groups"]])
    return out


def add_slot_record(rec, r):
    """Append a slot record unless the same cut is there (a slot pair in either order)."""
    if r["type"] == "family_nogood":
        g = lambda q: sorted((x["kind"], x["rack"], x["slot"]) for x in q["groups"])
        same = lambda q: q["type"] == r["type"] and g(q) == g(r)
    else:
        same = lambda q: q["type"] == r["type"] and (
            (q["rack"], q["slot"]) == (r["rack"], r["slot"]) if r["type"] == "slot" else
            {(q["rack"], q["slot"]), (q["rack_c"], q["slot_c"])} == {(r["rack"], r["slot"]), (r["rack_c"], r["slot_c"])})
    if not any(same(q) for q in rec["slots"]):
        rec["slots"].append(r)
        return True
    return False


def robot_of(od):
    """goal.robot of an all-pass order.json: per dish its move, hold, choice, spin and descent start."""
    out = {}
    for n, v in od["dishes"].items():
        out[n] = {k: v.get(k) for k in ("move", "hold", "choice", "spin_deg", "start_mm")}
        for k in ("side_below_top_m", "hold_gate_run", "narrow_retry_s"):
            if v.get(k) is not None:
                out[n][k] = v[k]
    return out


BASKET_OUT_RULE = ("the basket is lifted out for the robot; the dish poses stay legal with it in, so the Isaac stages run as the "
                   "benchmark defines them")


def family_record(v, od, culprits, k, level, R):
    """A plate's slot against the families of its culprits (plates: their slot; bowls: their site), one clause."""
    groups = [{"kind": "plate", "rack": v["rack"], "slot": v["slot"]}]
    groups += [{"kind": m.split("_")[0], "rack": od["dishes"][m]["rack"], "slot": od["dishes"][m]["slot"]} for m in culprits]
    return {"type": "family_nogood", "groups": groups, "dish": v["dish"], "culprits": list(culprits), "iteration": k,
            "level": level, "sizes": [len(slot_keys(R, level, g["rack"], g["slot"], g["kind"])) for g in groups]}


def hw_goal(count, seed, R, mount, work, log=print, levels=(0, 1), basket=True, max_iter=HW_MAX_ITER):
    """(entries or None, dag or None, info): the hardware-aware fit of the module docstring."""
    from . import build as BD
    from . import order as O
    B = R.T.B
    t0, mount, work = time.monotonic(), Path(mount), Path(work)
    hw = work / "hwgate"
    hw.mkdir(parents=True, exist_ok=True)
    bans_file = hw / "bans.json"
    store = json.loads(bans_file.read_text()) if bans_file.is_file() else {}
    rec = store.setdefault(f"{B.family_digest(BD.packer(R, 0)[0])}|{mount.stem}" + ("" if basket else "|nobasket"),
                           {"bans": [], "load_nogoods": []})
    rec.setdefault("slots", [])                           # the plates' slot cuts (2026-11-06, the coordinator's rule)
    iters, last = [], None
    info = {"method": "fit-hwgate", "seed": seed, "level": None, "S_first_fit": None, "S_ascent": None, "sweeps": None,
            "hwgate": {"mount": rel(mount), "basket": bool(basket), "iterations": iters, "bans_file": rel(bans_file),
                       "direct_moves": list(DIRECT_MOVES), "rule": RULE if basket else f"{RULE}; {BASKET_OUT_RULE}"}}
    nb = [] if basket else ["--no-basket"]
    info["hwgate"]["max_iter"] = int(max_iter)
    for k in range(1, int(max_iter) + 1):
        entries, fits = None, []
        for level in levels:
            bans = {frozenset(b) for b in rec["bans"]} | slot_bans(R, level, rec["slots"])
            entries, fit = BD.fit_pack(count, seed, R, log, level=level, extra_bans=bans,
                                       key_cuts=rec["load_nogoods"] + family_cuts(R, level, rec["slots"]), relaxed=False)
            fits.append({x: fit.get(x) for x in ("level", "poses", "conflicts", "cuts", "solves", "status", "seconds")})
            if entries is not None:
                break
        it = {"k": k, "level": fits[-1]["level"] if entries is not None else None, "fits": fits, "failed": {}, "cuts": [],
              "gate_seconds": 0., "bans_in": len(rec["bans"]), "nogoods_in": len(rec["load_nogoods"]),
              "slot_cuts_in": len(rec["slots"])}
        iters.append(it)
        if entries is None:
            info.update(seconds=round(time.monotonic() - t0, 1),
                        stop=f"infeasible for this hand at this mount: no packing at levels {list(levels)} under "
                             f"{len(rec['bans'])} bans and {len(rec['load_nogoods'])} no-goods (iteration {k})")
            log(f"  hwgate: {info['stop']}")
            return None, None, info
        key_of = fit["pose_key"]
        dag = O.dag_order(entries, seed)
        it_dir = hw / f"iter_{k}"
        it_dir.mkdir(parents=True, exist_ok=True)
        cand = candidate(work.name, entries, dag, mount, basket)
        cand_path = it_dir / "candidate.json"
        cand_path.write_text(json.dumps(cand, indent=1) + "\n")
        g0 = time.monotonic()
        od = access_order(["--candidate", rel(cand_path), "--direct-only", "--mount", rel(mount), "--workers", str(WORKERS),
                           "--out", rel(it_dir / "order.json"), *nb], it_dir / "order.json", it_dir / "gate.log")
        last = od
        failed = list(od["failed"])
        retry = [n for n in failed if n.startswith("plate") and od["dishes"][n].get("carry_timeout")]
        if retry:                                             # a carry RRT timeout is retractable: one re-gate at 180 s
            ro = gate_one(cand_path, [(n, od["dishes"][n]["scene"]) for n in retry], it_dir / "retry", mount,
                          retry_s=NARROW_RETRY_S, basket=basket)
            it["retry"] = {n: ro["dishes"][n]["direct_ok"] for n in retry}
            for n in retry:
                if ro["dishes"][n]["direct_ok"]:
                    od["dishes"][n] = dict(ro["dishes"][n], narrow_retry_s=NARROW_RETRY_S)
                    failed.remove(n)
        for n in failed:
            v = od["dishes"][n]
            it["failed"][n] = {"why": str(v["direct_why"])[:300], "culprits": v["culprits"], "scene": v["scene"],
                               "cand_key": key_of[n], "slot": v["slot"]}
        decided = {n: ("ban", None) for n in failed if not od["dishes"][n]["culprits"]}
        calls = {n: 0 for n in failed}
        for call in range(FILTER_CALLS):                      # the deletion filter, one culprit removed per call
            todo = [(n, od["dishes"][n]["culprits"][call]) for n in failed
                    if n not in decided and call < len(od["dishes"][n]["culprits"])]
            if not todo:
                break
            fo = gate_one(cand_path, [(n, [m for m in od["dishes"][n]["scene"] if m != c]) for n, c in todo],
                          it_dir / f"filter_{call + 1}", mount, basket=basket)
            for n, c in todo:
                calls[n] += 1
                if fo["dishes"][n]["direct_ok"]:
                    decided[n] = ("pair", c)
                elif len(od["dishes"][n]["culprits"]) == 1:
                    decided[n] = ("ban", None)
        for n in failed:
            typ, c = decided.get(n, ("load", None))
            v = od["dishes"][n]
            if n.startswith("plate") and typ == "pair" and c.startswith("bowl"):   # a bowl culprit: the plate slot x its site
                r = {"type": "slot_site", "rack": v["rack"], "slot": v["slot"], "rack_c": od["dishes"][c]["rack"],
                     "slot_c": od["dishes"][c]["slot"], "dish": n, "culprit": c, "iteration": k, "level": it["level"]}
                new = add_slot_record(rec, r)
                it["cuts"].append({"type": "slot_site", "dish": n, "culprit": c, "slot": v["slot"], "slot_c": r["slot_c"],
                                   "cand_key": [key_of[n], key_of[c]], "filter_calls": calls[n], "new": new})
                continue
            if n.startswith("plate") and typ == "load" and v["culprits"]:   # the plate slot x every culprit's family
                new = add_slot_record(rec, family_record(dict(v, dish=n), od, v["culprits"], k, it["level"], R))
                it["cuts"].append({"type": "family_nogood", "dish": n, "culprit": None, "slot": v["slot"],
                                   "culprits": v["culprits"], "filter_calls": calls[n], "new": new})
                # (the configuration no-good below is kept too)
            if n.startswith("plate") and (typ == "ban" or (typ == "pair" and c.startswith("plate"))):
                r = ({"type": "slot", "rack": v["rack"], "slot": v["slot"]} if typ == "ban" else
                     {"type": "slot_pair", "rack": v["rack"], "slot": v["slot"], "rack_c": od["dishes"][c]["rack"],
                      "slot_c": od["dishes"][c]["slot"]})
                r.update(dish=n, culprit=c, iteration=k, level=it["level"],
                         keys=slot_keys(R, it["level"], r["rack"], r["slot"]),
                         **({"keys_c": slot_keys(R, it["level"], r["rack_c"], r["slot_c"])} if c else {}))
                new = add_slot_record(rec, r)
                it["cuts"].append({"type": r["type"], "dish": n, "culprit": c, "slot": v["slot"],
                                   "slot_c": r.get("slot_c"), "cand_key": [key_of[n]] + ([key_of[c]] if c else []),
                                   "filter_calls": calls[n], "new": new})
                continue
            keys = ([key_of[n]] if typ == "ban" else sorted([key_of[n], key_of[c]]) if typ == "pair"
                    else sorted({key_of[n], *[key_of[m] for m in v["scene"]]}))
            (rec["load_nogoods"] if typ == "load" else rec["bans"]).append(keys)
            it["cuts"].append({"type": typ, "dish": n, "culprit": c, "cand_key": keys, "filter_calls": calls[n]})
        bans_file.write_text(json.dumps(store, indent=1) + "\n")
        it["gate_seconds"] = round(time.monotonic() - g0, 1)
        log(f"  hwgate iteration {k}: level {it['level']}, {len(od['dishes']) - len(failed)}/{len(od['dishes'])} direct, failed "
            f"{ {n: v['culprits'] for n, v in it['failed'].items()} }, cuts {[(c['type'], c['dish'], c['culprit']) for c in it['cuts']]}, "
            f"gate {it['gate_seconds']} s")
        if not failed:
            fit["dag"] = {"access": len(dag["access"]), "support": len(dag["support"]), "layers": dag["layers"]}
            info.update(level=it["level"], fit=fit, robot=robot_of(od), order_file=rel(it_dir / "order.json"),
                        seconds=round(time.monotonic() - t0, 1))
            info["hwgate"]["direct"] = f"{len(od['dishes'])}/{len(od['dishes'])} at iteration {k}"
            return entries, dag, info
    info.update(seconds=round(time.monotonic() - t0, 1), stop=f"the loop stopped at {HW_MAX_ITER} iterations with "
                f"{len(iters[-1]['failed'])} dishes failing")
    log(f"  hwgate: {info['stop']}")
    return None, None, info


def initial_check(state, params, od=None, log=print):
    """(ok, note): initial.robot and metadata.provenance.hwgate_initial (module docstring); ok = every rack dish pickable."""
    init, prov = state["initial"], state["metadata"]["provenance"]
    work, mount = Path(params["work"]), ROOT / params["hardware"]
    basket = params.get("hardware_basket", "in") != "out"
    counter = [d for st in init["counter"]["stacks"] for d in st["ids"]]
    start = json.loads((ROOT / prov["initial_record"]).read_text())
    racks = {o["object_id"]: o.get("rack") for o in start["objects"]}
    in_rack = [n for n in init["poses"] if n not in counter]
    robot, d_ = {}, work / "hwgate" / "initial"
    if in_rack:
        d_.mkdir(parents=True, exist_ok=True)
        cand = {"metadata": {"sample_id": f"{work.name}__start", "provenance": {"mount": rel(mount), "basket": basket,
                                                                               "goal_record": None}},
                "initial": None,
                "goal": {"poses": {n: init["poses"][n] for n in in_rack}, "rack_joints": init["rack_joints"],
                         "slots": {n: {"rack": racks[n], "slot": "start", "variant": "start"} for n in in_rack}, "score": None},
                "dag": {"nodes": in_rack, "edges": [], "layers": {}, "order": in_rack}}
        (d_ / "candidate.json").write_text(json.dumps(cand, indent=1) + "\n")
        res = gate_one(d_ / "candidate.json", [(n, [m for m in in_rack if m != n and racks[m] == racks[n]]) for n in in_rack],
                       d_, mount, basket=basket)
        for n in in_rack:
            v = res["dishes"][n]
            robot[n] = {"where": racks[n], "pickable": bool(v["direct_ok"]), "hold": v["hold"], "move": v["move"],
                        "why": v["direct_why"] or "passes the goal's admissibility test at its start pose (the other start "
                                                  "dishes of its rack in the scene)"}
    picks = {}
    for v in ((od or {}).get("dishes") or {}).values():   # the counter: the pick of each kind's hold at the mount's spot
        picks.setdefault((v["kind"], v["hold"]), []).append(bool(v.get("pick_ok")))
    for n in counter:
        kind = n.split("_")[0]
        got = [(h, any(ok)) for (k_, h), ok in picks.items() if k_ == kind]
        if not got:
            robot[n] = {"where": "counter", "pickable": None, "hold": None, "why": "no pick verdict of its kind yet"}
            continue
        h, ok = max(got, key=lambda x: x[1])
        robot[n] = {"where": "counter", "pickable": ok, "hold": h,
                    "why": f"the {h} hold's pick at the counter spot as the robot pipeline presents a {kind} (the start's "
                           f"counter stack is not re-presented)"}
    init["robot"] = robot
    bad = sorted(n for n, v in robot.items() if v["where"] != "counter" and not v["pickable"])
    prov["hwgate_initial"] = {"counts": {"rack": len(in_rack), "rack_pickable": len(in_rack) - len(bad),
                                         "counter": len(counter),
                                         "counter_pickable": sum(bool(robot[n]["pickable"]) for n in counter)},
                              "rack_failed": bad, "dir": rel(d_), "mount": rel(mount), "basket": basket,
                              "rule": "rack dishes: the goal's admissibility test (access order --gate-one, direct-only) at the "
                                      "start pose with the other start dishes of its rack in the scene; counter dishes: the pick "
                                      "of the kind's hold at the mount's counter spot as presented (stacks not re-presented)"}
    note = (f"hwgate_initial: rack {len(in_rack) - len(bad)}/{len(in_rack)} pickable"
            + (f" (failed {bad})" if bad else "") + f", counter {prov['hwgate_initial']['counts']['counter_pickable']}"
            f"/{len(counter)}")
    log(f"  {note}")
    return not bad, note
