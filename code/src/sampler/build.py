"""build stage: a restricted packing of the roster (frigidaire_dot_rules_restricted.pack_random over the top-5 run's
level-0 families and masks; Kit-free, numpy, in-process), its build order from the packing's access and support DAG
(order.py; a cyclic packing is drawn again, seed * 1000 + k, k < DRAWS), then the sampler build in Isaac
(frigidaire_dependency_kit.py --load --order, by subprocess: the insertion move in that order, the joint gate's overlap
rule). build.order "dag-rest" (the default, plans/2026-10-22-plate-rest-pose-dag.md): order.py's rest pose model, the kit
keeps re-pose and deferral; "dag" (plans/2026-10-22-dag-build-order.md): the packed poses, a strict build (no deferral, a
re-pose only on a physical miss). The order file carries pose_model and strict to the kit.
"""
import importlib.util
import json
from pathlib import Path

from . import ROOT, rel, run
from .order import dag_order

DEP_KIT = "code/planner/frigidaire/frigidaire_dependency_kit.py"
DEFAULTS = {"build.order": "dag-rest", "build.release": "insertion", "build.plates": "upright"}
ORDERS = {"dag-rest": ("rest", False), "dag": ("packed", True)}     # build.order -> (order.py pose model, strict build)
DRAWS = 10


def roster(count):
    return [(f"{kind}_{k:02d}", kind) for kind, n in count.items() for k in range(1, n + 1)]


def pack(count, seed):
    """(entries or None, failing dish id, order): one restricted random packing of the roster, all dishes racked."""
    spec = importlib.util.spec_from_file_location("restricted", ROOT / "code/planner/frigidaire/frigidaire_dot_rules_restricted.py")
    R = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(R)
    R.T.ROSTER = roster(count)                       # pack_random draws its order from T.ROSTER (the 24 by default)
    fam, parts, points = R.T.level_families(0)
    masks = R.T.level_masks(fam, parts, points, 0)
    return R.pack_random(fam, masks, parts, points, R.T.all_bans(), seed)


def stage_build(state, params):
    off = {k: v for k, v in params.items() if k in DEFAULTS and v != DEFAULTS[k] and not (k == "build.order" and v in ORDERS)}
    if off:
        return state, "FAIL", f"variants {off} not connected (the kit's sampler rules are fixed)"
    work = Path(params["work"]) / "goal"
    load_id = Path(params["work"]).name
    model, strict = ORDERS[params.get("build.order", DEFAULTS["build.order"])]
    cycles = []
    for k in range(DRAWS):
        seed = state["seed"] if k == 0 else state["seed"] * 1000 + k
        entries, failed, order = pack(state["params"]["count"], seed)
        if entries is None and k == 0:
            return state, "FAIL", f"packing: no free pose for {failed}"
        if entries is None:                                   # a re-draw with no free pose: the next draw
            continue
        dag = dag_order(entries, seed, pose_model=model)
        if dag["cycle"] is None:
            break
        cycles.append({"seed": seed, "cycle": dag["cycle"]})
    info = {"draws": k + 1, "seed": seed, "cycles": cycles, "pose_model": model, "strict": strict,
            **{x: dag[x] for x in ("access", "support", "layers", "order")}}
    if dag["cycle"] is not None:
        state["goal"] = {"load_id": load_id, "dag": info}
        return state, "FAIL", f"packing had a cycle: {len(cycles)} cyclic of {DRAWS} draws, none acyclic"
    loads = work / "loads"
    loads.mkdir(parents=True, exist_ok=True)
    (loads / f"{load_id}.json").write_text(json.dumps({"load_id": load_id, "seed": seed, "order": order,
                                                        "entries": entries}, indent=1) + "\n")
    order_file = loads / f"{load_id}.order.json"
    order_file.write_text(json.dumps(info, indent=1) + "\n")
    ok, _ = run("kit", [DEP_KIT, "--load", load_id, "--restricted-dir", rel(loads), "--order", rel(order_file),
                        "--out-dir", rel(work), "--headless"], work / "logs" / "build.log")
    record = work / f"{load_id}.json"
    dep = json.loads(record.read_text()) if record.is_file() else {}
    state["goal"] = {"load_id": load_id, "record": rel(record), "packed": entries, "entries": dep.get("entries_settled"),
                     "order": (dep.get("witness") or {}).get("order"), "gate": None, "certify": None, "score": None,
                     "graph": None, "dag": info}
    return state, "PASS" if ok and dep.get("status") == "ok" else "FAIL", (
        f"build status {dep.get('status')}; dag {len(info['access'])} access + {len(info['support'])} support edges, "
        f"{info['layers']} layers, draw {k + 1}")
