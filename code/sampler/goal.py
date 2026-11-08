"""certify stage: frigidaire_dependency_kit.py --certify --gate (Kit, one fresh process, by subprocess): the witness
order rebuilt from empty racks, support from resting contacts, access from the path check, confirmation drops. The
verdict goes to metadata.provenance.certify; the evidence (order, confirmations, access hits) stays in the build record
in _work, where graph.py reads it.
"""
import json
from pathlib import Path

from . import rel, run

DEP_KIT = "code/sampler/scripts/frigidaire_dependency_kit.py"


def stage_certify(state, params):
    prov = state["metadata"]["provenance"]
    if not (prov.get("gate_goal") or {}).get("dir"):
        return state, "FAIL", "no accepted joint gate"
    work = Path(params["work"]) / "goal"
    run("kit", [DEP_KIT, "--load", prov["load_id"], "--restricted-dir", rel(work / "loads"), "--out-dir",
                prov["goal_record"].rsplit("/", 1)[0], "--certify", "--gate", prov["gate_goal"]["dir"], "--headless"],
        work / "logs" / "certify.log")
    dep = json.loads((work / f"{prov['load_id']}.json").read_text())
    prov["certify"] = {"valid": bool(dep.get("valid")), "verdict": dep.get("verdict")}
    return state, "PASS" if dep.get("valid") else "FAIL", (dep.get("verdict") or {}).get("reason")
