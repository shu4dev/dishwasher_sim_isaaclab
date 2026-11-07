"""run stage (module episodes: bench.run is the subprocess helper): the four algorithms' episodes on the sample
(plans/2026-11-01-merged-benchmark.md, Decisions), written to <out>/runs/<sample>.json (plans/2026-11-02-sample-schema.md).

1. The sample becomes a bench instance (frigidaire_bench.instance_of_sample) at <work>/run/instance.json.
2. Track B's own loads (first_fit, mcts; code/frigidaire/docs/hotec_bench.md, "own loads"): plan (frigidaire_bench.py
   --plan, Kit-free, 60 s) -> the joint Isaac gate (frigidaire_initial_state_validate.py, lower rack first once on a
   rack-speed flake) -> the one-dish-at-a-time Isaac build (frigidaire_bench_kit.py --sequence-plan) -> --resequence;
   a failed build bans what it blamed (--ban-plan) and re-plans, up to frigidaire_bench.PLAN_ATTEMPTS.
3. The episodes (frigidaire_bench_kit.py --run --sample, cameras on): track A greedy_offline and rrt_connect reach the
   sample's goal; track B replays its certified plans. Fault rule: a nudge is counted, fatal only when the nudged dish
   was at its goal or leaves the counter and the racks (BenchOracle). Stills to the sample's media folder (the kit
   renders the initial and goal stills again before the episodes; the stills stage owns metadata.media).
4. S_final of each episode's final load (Kit-free, GPU): the fair score at the full budget (score.fair, the 6.2
   ceilings) over the roster, an unracked dish scoring 0; S / S_ref.
5. No videos here: the video stage (bench/video.py) renders them; the rows keep video null.
Solved: track A = every dish at its goal within the planning budget and the end check (racks retract, contained);
track B = every dish racked, the end check, and no dish pooling.
"""
import json
from pathlib import Path
import shutil

from . import ROOT, rel, run
from . import score as SC
from .sample import SCHEMA_VERSION, read_run, write_run

KIT = "code/planner/frigidaire/frigidaire_bench_kit.py"
BENCH = "code/planner/frigidaire/frigidaire_bench.py"
VALIDATE = "code/initialization/frigidaire/frigidaire_initial_state_validate.py"
TRACK = {"greedy_offline": "goal", "rrt_connect": "goal", "first_fit": "open", "mcts": "open"}


def bench_module():
    import importlib.util
    import sys
    for p in (ROOT / "code/frigidaire/src", ROOT / "code/planner/frigidaire"):
        if str(p) not in sys.path:
            sys.path.append(str(p))
    spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "code/planner/frigidaire/frigidaire_bench.py")
    B = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(B)
    return B


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


def stage_run(state, params):
    goal = state.get("goal") or {}
    if not ((state["metadata"]["provenance"].get("certify") or {}).get("valid")
            and (goal.get("score") or {}).get("S_ref") is not None):
        return state, "FAIL", "no certified, scored goal (run certify and score first)"
    algos = [a for a in params.get("algorithms") or TRACK if a in TRACK]
    sample_path = Path(params["sample"])
    pid = sample_path.stem
    rundir = Path(params["work"]) / "run"
    logs = rundir / "logs"
    media = Path(params.get("media") or ROOT / "data/media/bench/v1") / pid
    rundir.mkdir(parents=True, exist_ok=True)
    B = bench_module()
    inst = B.instance_of_sample(state, pid)
    inst_path = rundir / "instance.json"
    inst_path.write_text(json.dumps(inst, indent=1) + "\n")
    own = {name: own_load(B, inst_path, inst, name, rundir, logs) for name in algos if TRACK[name] == "open"}
    kit_ok, _ = run("kit", [KIT, "--run", "--sample", rel(sample_path), "--algorithms", *algos,
                            "--out", rel(rundir), "--media", rel(media), "--headless", "--enable_cameras"], logs / "episodes.log")
    eps = {a: rundir / "episodes" / TRACK[a] / inst["tier"] / f"{pid}__{a}.json" for a in algos}
    have = {a: p for a, p in eps.items() if p.is_file()}
    rows = []
    if have:
        B.HX().register_hotec(B.ASSETS)
        from dishsim_frigidaire import dot_rules as DR
        from dishsim_frigidaire.loading import visual_points
        eng = DR.DotRuleEngine(SC.N_FULL)
        ceil = SC.ceilings(params.get("ceilings", SC.CEILINGS))
        points = {kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS}
    for a in algos:
        row = {"algorithm": a, "track": TRACK[a], "record": rel(eps[a]) if a in have else None}
        if a in have:
            rec = json.loads(have[a].read_text())
            S, racked, pooling = final_score(B, DR, eng, ceil, inst, rec, points)
            end, abort = (rec.get("end_check") or {}).get("outcome"), rec.get("abort")
            solved = bool(rec.get("success")) and (TRACK[a] == "goal" or not pooling)
            failure = (None if solved else abort if abort not in (None, "give-up") else end if end not in (None, "accepted")
                       else "pooling" if TRACK[a] == "open" and pooling else abort or "not solved")
            row.update(solved=solved, moves=rec.get("moves_used"), abort=abort, end_check=end, failure=failure,
                       at_goal=f"{rec.get('at_goal_final')}/{rec.get('n_items')}", nudges=rec.get("nudges"),
                       failed_settles=rec.get("failed_settles"), refusals=rec.get("infeasible_commands"),
                       planning_s=rec.get("planning_time_total_s"), S_final=S, racked=racked, pooling=pooling,
                       S_over_S_ref=S / goal["score"]["S_ref"] if goal["score"]["S_ref"] else None,
                       still=rel(media / f"finished__{TRACK[a]}__{a}.png"))
            (have[a].with_name(have[a].stem + ".fair.json")).write_text(json.dumps(
                {"S_final": S, "racked": racked, "pooling": pooling, "S_ref": goal["score"]["S_ref"],
                 "ceilings": rel(params.get("ceilings", SC.CEILINGS)), "N": SC.N_FULL}, indent=1) + "\n")
            row["video"] = None
        rows.append(row)
    write_run(sample_path, {"schema_version": SCHEMA_VERSION, "sample_id": pid, "instance": rel(inst_path),
                            "dir": rel(rundir), "media": rel(media), "algorithms": algos, "own_loads": own,
                            "episodes": rows, "kit_log": rel(logs / "episodes.log"),
                            "stages": (read_run(sample_path) or {}).get("stages") or []})
    missing = [a for a in algos if a not in have]
    note = (", ".join(f"{r['algorithm']} {'solved' if r.get('solved') else r.get('failure') or 'not solved'}"
                      f" {r.get('moves')} moves S/S_ref {r['S_over_S_ref']:.3f}" if r.get("record") else f"{r['algorithm']} no record"
                      for r in rows))
    ok = kit_ok and not missing
    return state, "PASS" if ok else "FAIL", note + ("" if ok else f"; episodes missing {missing}" if missing else "; kit log not PASS")
