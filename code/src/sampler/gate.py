"""Gates: the joint Isaac gate of a loaded rack state (frigidaire_initial_state_validate.py, Kit, by subprocess: settle,
retract both racks, containment; a rack-speed flake, closure_failure, is retried lower-first once, as the bench does).
"""
import json
from pathlib import Path

from . import rel, run

VALIDATE = "code/initialization/frigidaire/frigidaire_initial_state_validate.py"
DEP_KIT = "code/planner/frigidaire/frigidaire_dependency_kit.py"
PEAK_PENETRATION_M = "0.005"     # frigidaire_dependency_kit.py: the samples' gate (--peak-penetration-m 0.005)


def joint_gate(manifest, gate_dir, log_dir):
    """(outcome, accepted gate dir or None) of the joint gate of ``manifest``."""
    argv = [VALIDATE, "--manifest", rel(manifest), "--out-dir", rel(gate_dir), "--max-wall-seconds", "600",
            "--peak-penetration-m", PEAK_PENETRATION_M]
    outcome = None
    for order, out in ((None, Path(gate_dir)), ("lower_first", Path(f"{gate_dir}_lower_first"))):
        extra = ["--order", order] if order else []
        if order:
            argv[argv.index("--out-dir") + 1] = rel(out)
        run("kit", argv + extra + ["--headless", "--device", "cpu"], Path(log_dir) / f"gate_{order or 'upper_first'}.log")
        result = out / "result.json"
        outcome = json.loads(result.read_text()).get("outcome") if result.is_file() else None
        if outcome == "accepted":
            return outcome, out
        if outcome != "closure_failure":
            break
    return outcome, None


def stage_gate_initial(state, params):
    """Not connected this round: the bench's start record holds world poses (initial.poses), and the joint gate's manifest
    needs rack-local entries in the extended racks; the start's own settle, reproduction and unstack checks are kept
    under initial.settle."""
    state["initial"]["gate"] = None
    return state, "SKIP", "joint gate of the initial's rack dishes not connected (start.json has world poses only)"


def stage_gate_goal(state, params):
    if params.get("gate", "full") != "full":
        return state, "FAIL", f"gate={params['gate']} not connected (the joint gate only)"
    goal, work = state["goal"], Path(params["work"]) / "goal"
    load_id = goal["load_id"]
    manifest = work / "_gate" / load_id / "manifest.json"
    ok, _ = run("py", [DEP_KIT, "--load", load_id, "--out-dir", rel(work), "--manifest-out", rel(manifest)],
                work / "logs" / "manifest.log")
    if not ok:
        return state, "FAIL", "manifest-out failed"
    outcome, accepted = joint_gate(manifest, work / "_gate" / load_id, work / "logs")
    goal["gate"] = {"outcome": outcome, "dir": rel(accepted) if accepted else None, "manifest": rel(manifest)}
    return state, "PASS" if accepted else "FAIL", f"joint gate {outcome}"
