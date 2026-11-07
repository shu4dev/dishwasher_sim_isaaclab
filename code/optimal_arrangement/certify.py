"""Track B's own loads and the fair score of an episode's final load, out of the run stage (code/paths/episodes.py,
which imports them back; plans/2026-11-01-merged-benchmark.md, Decisions): own_load plans (frigidaire_bench.py
--plan), gates (own_gate, the joint Isaac gate), builds (frigidaire_bench_kit.py --sequence-plan) and resequences one
own load; final_score is S_final of an episode's final load (the fair score over the roster, unracked = 0).
"""
import json
from pathlib import Path
import shutil

from sampler import rel, run
from sampler.scorer import score as SC

KIT = "code/sampler/scripts/frigidaire_bench_kit.py"
BENCH = "code/arrangement_optimization/frigidaire_bench.py"
VALIDATE = "code/sampler/scripts/frigidaire_initial_state_validate.py"


def own_gate(manifest, gate_dir, logs, tag):
    """The bench's joint gate of an own load (frigidaire_bench.gate_unit's argv, the gate's own limits); a rack-speed
    flake (closure_failure) is retried lower rack first once and the accepted run takes the gate's place."""
    gate_dir = Path(gate_dir)
    argv = [VALIDATE, "--manifest", rel(manifest), "--out-dir", rel(gate_dir), "--max-wall-seconds", "600"]
    result = gate_dir / "result.json"
    if not result.is_file():
        run("kit", argv + ["--headless", "--device", "cpu"], Path(logs) / f"{tag}_gate.log")
    outcome = json.loads(result.read_text()).get("outcome") if result.is_file() else None
    if outcome == "closure_failure":
        alt = Path(f"{gate_dir}_lower_first")
        argv[argv.index("--out-dir") + 1] = rel(alt)
        run("kit", argv + ["--order", "lower_first", "--headless", "--device", "cpu"], Path(logs) / f"{tag}_gate_lower_first.log")
        r2 = alt / "result.json"
        if r2.is_file() and json.loads(r2.read_text()).get("outcome") == "accepted":
            shutil.move(str(gate_dir), f"{gate_dir}_upper_first")
            shutil.copytree(str(alt), str(gate_dir))
            outcome = "accepted"
    return outcome == "accepted", outcome


def own_load(B, inst_path, inst, name, rundir, logs):
    """Plan -> gate -> sequence build -> resequence, up to PLAN_ATTEMPTS; the final plan file or the last failure."""
    final = rundir / "plans" / "open" / inst["tier"] / f"{inst['instance_id']}__{name}.json"
    trail = []
    for attempt in range(B.PLAN_ATTEMPTS):
        if final.is_file():
            break
        last = attempt == B.PLAN_ATTEMPTS - 1
        p = B.plan_attempt_path(inst, name, attempt, rundir)
        stem = p.name[:-len(".json")]
        ok, _ = run("py", [BENCH, "--plan", rel(inst_path), "--algorithm", name, "--attempt", attempt, "--out", rel(rundir)],
                    logs / f"plan_{stem}.log")
        manifest, gate_dir = p.with_name(stem + ".gate_manifest.json"), p.with_name(stem + ".gate")
        gated, outcome = own_gate(manifest, gate_dir, logs, stem) if manifest.is_file() else (False, "no load")
        built = False
        if gated:
            built, _ = run("kit", [KIT, "--sequence-plan", rel(p), "--instance", rel(inst_path), "--gate", rel(gate_dir),
                                   "--out", rel(rundir), "--headless"], logs / f"sequence_{stem}.log")
        if not built and not p.with_name(stem + ".bans.json").is_file() and p.is_file():
            run("py", [BENCH, "--ban-plan", rel(p)], logs / f"ban_{stem}.log")
        reseq, _ = run("py", [BENCH, "--resequence", rel(p), "--instance", rel(inst_path), "--out", rel(rundir),
                              *(["--final"] if last else [])], logs / f"resequence_{stem}.log")
        trail.append({"attempt": attempt, "plan": ok, "gate": outcome, "built": built, "resequenced": reseq})
        if reseq:
            break
    plan = json.loads(final.read_text()) if final.is_file() else {}
    return {"certified": bool(plan.get("certified")), "attempts": trail, "plan": rel(final) if final.is_file() else None,
            "reason": (plan.get("certification") or {}).get("reason")}


def final_score(B, DR, eng, ceil, inst, rec, points):
    """(S_final, n racked, pooling ids) of an episode's final load: the fair score over the roster, unracked = 0."""
    P = B._planner()
    final, frames = rec.get("final_poses") or {}, rec.get("final_components") or {}
    kinds = {o["object_id"]: o["kind"] for o in inst["objects"]}
    entries = []
    for oid, kind in kinds.items():
        pose = final.get(oid)
        rack = B.racked_in(points[kind], pose, frames) if pose and frames else None
        if rack is None:
            continue
        local = P.local_from_world(frames[rack], pose)
        entries.append({"id": oid, "kind": kind, "rack": rack, "position": list(local["position_m"]),
                        "quaternion_xyzw": list(local["quaternion_xyzw"])})
    if not entries:
        return 0., 0, []
    r = DR.score_load(eng, entries, SC.THETA_DEG, f"{inst['instance_id']}_{rec.get('algorithm')}")
    f = SC.fair(*SC.record_state(r), ceil)
    return f["S"] * len(entries) / len(kinds), len(entries), r["pooling"]
