"""build stage: a restricted packing of the roster (frigidaire_dot_rules_restricted.pack_random over the top-5 run's
level-0 families and masks; Kit-free, numpy, in-process), then the sampler build in Isaac (frigidaire_dependency_kit.py
--load, by subprocess: the insertion move, lean-side plate order, re-posing, deferral, the joint gate's overlap rule).
The kit's rules are fixed this round: only the default variants run.
"""
import importlib.util
import json
from pathlib import Path

from . import ROOT, rel, run

DEP_KIT = "code/planner/frigidaire/frigidaire_dependency_kit.py"
DEFAULTS = {"build.order": "lean-side", "build.release": "insertion", "build.plates": "upright"}


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
    off = {k: v for k, v in params.items() if k in DEFAULTS and v != DEFAULTS[k]}
    if off:
        return state, "FAIL", f"variants {off} not connected (the kit's sampler rules are fixed)"
    work = Path(params["work"]) / "goal"
    load_id = Path(params["work"]).name
    entries, failed, order = pack(state["params"]["count"], state["seed"])
    if entries is None:
        return state, "FAIL", f"packing: no free pose for {failed}"
    loads = work / "loads"
    loads.mkdir(parents=True, exist_ok=True)
    (loads / f"{load_id}.json").write_text(json.dumps({"load_id": load_id, "seed": state["seed"], "order": order,
                                                        "entries": entries}, indent=1) + "\n")
    ok, _ = run("kit", [DEP_KIT, "--load", load_id, "--restricted-dir", rel(loads), "--out-dir", rel(work), "--headless"],
                work / "logs" / "build.log")
    record = work / f"{load_id}.json"
    dep = json.loads(record.read_text()) if record.is_file() else {}
    state["goal"] = {"load_id": load_id, "record": rel(record), "packed": entries, "entries": dep.get("entries_settled"),
                     "order": (dep.get("witness") or {}).get("order"), "gate": None, "certify": None, "score": None,
                     "graph": None}
    return state, "PASS" if ok and dep.get("status") == "ok" else "FAIL", f"build status {dep.get('status')}"
