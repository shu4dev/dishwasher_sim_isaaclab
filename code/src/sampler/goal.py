"""certify stage: frigidaire_dependency_kit.py --certify --gate (Kit, one fresh process, by subprocess): the witness
order rebuilt from empty racks, support from resting contacts, access from the path check, confirmation drops; the
record's graph inputs are stored for graph.py.
"""
import json
from pathlib import Path

from . import rel, run

DEP_KIT = "code/planner/frigidaire/frigidaire_dependency_kit.py"


def stage_certify(state, params):
    goal = state["goal"]
    if not (goal.get("gate") or {}).get("dir"):
        return state, "FAIL", "no accepted joint gate"
    work = Path(params["work"]) / "goal"
    run("kit", [DEP_KIT, "--load", goal["load_id"], "--restricted-dir", rel(work / "loads"), "--out-dir",
                goal["record"].rsplit("/", 1)[0], "--certify", "--gate", goal["gate"]["dir"], "--headless"],
        work / "logs" / "certify.log")
    dep = json.loads((work / f"{goal['load_id']}.json").read_text())
    order = (dep.get("witness") or {}).get("order")
    goal["certify"] = {"valid": bool(dep.get("valid")), "verdict": dep.get("verdict"), "order": order,
                       "confirmations": dep.get("confirmations") or [], "hits": (dep.get("access") or {}).get("hits") or []}
    goal["order"] = order
    return state, "PASS" if dep.get("valid") else "FAIL", (dep.get("verdict") or {}).get("reason")
