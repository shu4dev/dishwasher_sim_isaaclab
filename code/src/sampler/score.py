"""score stage (Kit-free, GPU): frigidaire_dependency_score.py on the goal's record (dot_rules.score_load of its packed,
jointly settled and built poses; S and W per rule, per dish). The kind-ceiling fair score (frigidaire_dot_rules_relative.py
--reference kind) is not connected this round: score.reference=kind stores the raw records only.
"""
import json
from pathlib import Path

from . import run

SCORER = "code/planner/frigidaire/frigidaire_dependency_score.py"


def stage_score(state, params):
    if params.get("score.reference", "kind") == "none":
        state["goal"]["score"] = None
        return state, "SKIP", "score.reference=none"
    goal, work = state["goal"], Path(params["work"]) / "goal"
    ok, _ = run("py", [SCORER, "--dependency-dir", goal["record"].rsplit("/", 1)[0], "--loads", goal["load_id"]],
                work / "logs" / "score.log")
    path = work / f"{goal['load_id']}.scores.json"
    if not (ok and path.is_file()):
        return state, "FAIL", "scorer failed"
    goal["score"] = {"file": str(path), **json.loads(path.read_text())}
    return state, "PASS", f"scored ({params.get('score.reference', 'kind')} reference not applied: raw records)"
