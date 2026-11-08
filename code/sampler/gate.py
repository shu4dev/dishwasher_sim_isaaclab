"""Gates: the joint Isaac gate of a loaded rack state (frigidaire_initial_state_validate.py, Kit, by subprocess: settle,
retract both racks, containment; a rack-speed flake, closure_failure, is retried lower-first once, as the bench does).
gate_initial runs no Kit: it judges the start generator's own checks in the stored start record.
"""
import json
from pathlib import Path

from . import rel, run

VALIDATE = "code/sampler/scripts/frigidaire_initial_state_validate.py"
DEP_KIT = "code/sampler/scripts/frigidaire_dependency_kit.py"
PEAK_PENETRATION_M = "0.005"     # frigidaire_dependency_kit.py: the samples' gate (--peak-penetration-m 0.005)
REPRO_MAX_MM, REPRO_MAX_DEG = 10., 15.   # the bench's reproduction tolerance (docs/hotec_bench.md, Start generation, 4)


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
    """What covers the initial today (plans/2026-10-31-caveat-fixes.md, Decisions, gate_initial), judged from the start
    record the initial stage stored (metadata.provenance.settle, start.json's validation;
    code/sampler/docs/hotec_bench.md, Start generation): the start's settle passed (item 2: 1 s windows over 5 s, peak
    and median contact depth under 2 mm), its reproduction is within REPRO_MAX_MM / REPRO_MAX_DEG (item 4) and its
    unstack rehearsal found a clearing order (item 5).
    PASS when all three hold, FAIL otherwise or when the record is missing; never SKIP. No Kit: the fresh-process
    re-settle of the stored poses is the next sampler round's."""
    initial, prov = state.get("initial"), state["metadata"]["provenance"]
    settle = prov.get("settle") or {}
    missing = [k for k in ("start", "reproduction", "unstack") if not settle.get(k)]
    if initial is None or missing:
        if initial is not None:
            prov["gate_initial"] = None
        return state, "FAIL", "no stored start record: " + (
            "no initial (run the initial stage first)" if initial is None else f"provenance.settle lacks {', '.join(missing)}")
    start, repro, unstack = settle["start"], settle["reproduction"], settle["unstack"]
    peak, mm, deg = start.get("maximum_settle_penetration_m"), repro.get("max_pos_mm"), repro.get("max_rot_deg")
    prov["gate_initial"] = gate = {"source": "start generator (same Kit process)",
                                   "settled": bool((start.get("settled") or {}).get("passed")),
                                   "settle_peak_mm": None if peak is None else round(peak * 1e3, 3),
                                   "reproduction_mm": mm, "reproduction_deg": deg,
                                   "unstack_steps": len(unstack.get("order") or [])}
    why = [w for w, bad in (("settle not passed", not gate["settled"]),
                            (f"reproduction over {REPRO_MAX_MM:g} mm / {REPRO_MAX_DEG:g} deg",
                             mm is None or deg is None or mm > REPRO_MAX_MM or deg > REPRO_MAX_DEG),
                            ("no clearing order", not gate["unstack_steps"])) if bad]
    peak_txt = "n/a" if gate["settle_peak_mm"] is None else f"{gate['settle_peak_mm']} mm"
    note = f"settle {peak_txt}, reproduction {mm} mm / {deg} deg, unstack {gate['unstack_steps']} steps"
    if why:
        return state, "FAIL", f"not covered by the start generator: {note} ({'; '.join(why)})"
    if params.get("hardware"):                           # plans/2026-11-05-hardware-aware-sampler.md: the start's rack dishes
        from . import hwgate                             # must be pickable (a failure spends the seed: the next seed re-rolls)
        ok, hnote = hwgate.initial_check(state, params)
        note = f"{note}; {hnote}"
        if not ok:
            return state, "FAIL", f"hardware-invalid at the start: {note}"
    return state, "PASS", f"covered by the start generator: {note}"


def stage_gate_goal(state, params):
    if params.get("gate", "full") != "full":
        return state, "FAIL", f"gate={params['gate']} not connected (the joint gate only)"
    prov, work = state["metadata"]["provenance"], Path(params["work"]) / "goal"
    load_id = prov["load_id"]
    manifest = work / "_gate" / load_id / "manifest.json"
    ok, _ = run("py", [DEP_KIT, "--load", load_id, "--out-dir", rel(work), "--manifest-out", rel(manifest)],
                work / "logs" / "manifest.log")
    if not ok:
        return state, "FAIL", "manifest-out failed"
    outcome, accepted = joint_gate(manifest, work / "_gate" / load_id, work / "logs")
    prov["gate_goal"] = {"outcome": outcome, "dir": rel(accepted) if accepted else None, "manifest": rel(manifest)}
    return state, "PASS" if accepted else "FAIL", f"joint gate {outcome}"
