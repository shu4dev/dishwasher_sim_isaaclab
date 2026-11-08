"""Schema 1 pair files -> schema 2 samples, in place and Kit-free (plans/2026-11-02-sample-schema.md):

    PYTHONPATH=code:code/util python3 -m sampler.convert data/results/bench/v3 data/results/bench/samples

For each <folder>/*.json of schema_version 1: the original is kept at <folder>/_work/<sample>/schema1.json; the sample
(metadata, initial, goal, dag) is built from it and its _work records (the start record, the build record) with the
stages' own block builders (sampler.sample); <folder>/runs/<sample>.json is written when its run block has episode rows,
with the run, video and report ledger entries; then index.jsonl. A file of schema 2 is left as it is.
"""
import json
import shutil
import sys
from pathlib import Path

from . import ROOT
from . import sample as SA


def convert(old, sample_id):
    """(sample, runs record or None) of one schema-1 pair."""
    init, goal, run = old.get("initial") or {}, old.get("goal") or {}, old.get("run") or {}
    rows = run.get("episodes") or []
    start = json.loads((ROOT / init["source"]).read_text()) if init.get("source") else None
    dep = json.loads((ROOT / goal["record"]).read_text()) if goal.get("record") else {}
    cert, graph, score = goal.get("certify"), goal.get("graph"), goal.get("score")
    prov = {"load": old["load"]} if old.get("load") else {}
    if start is not None:
        prov.update(initial_record=init["source"], settle=init.get("settle"), gate_initial=init.get("gate"))
    prov.update({k: v for k, v in (("load_id", goal.get("load_id")), ("goal_record", goal.get("record")),
                                   ("gate_goal", goal.get("gate"))) if v is not None})
    if cert is not None:
        prov["certify"] = {"valid": cert["valid"], "verdict": cert["verdict"], **({"counts": graph["counts"]} if graph else {})}
    if score is not None:
        prov["score"] = {"file": score["file"], "N": score["N"], "theta_deg": score["theta_deg"],
                         "ceilings": score["ceilings"]["file"]}
    moved = [s for s in old["stages"] if rows and s["stage"] in SA.RUN_STAGES]
    meta = {"schema_version": SA.SCHEMA_VERSION, "sample_id": sample_id, "machine": old["machine"],
            "dishes": old["dishes"], "seed": old["seed"], "variant": old.get("variant"), "params": old["params"],
            "stages": [s for s in old["stages"] if s not in moved],
            "maker": goal.get("maker") and {k: goal["maker"].get(k) for k in SA.MAKER_KEYS},
            "provenance": prov, "media": run.get("stills") or None}
    new_goal = SA.goal_of(dep, start) if dep.get("targets_world") else None
    if new_goal is not None and score is not None:
        new_goal["score"] = {k: score[k] for k in ("S_ref", "S_area", "per_dish")}
    new = {"metadata": meta, "initial": start and SA.initial_of(start), "goal": new_goal,
           "dag": graph and SA.dag_of(graph, goal["order"])}
    runs = None
    if rows:
        runs = {"schema_version": SA.SCHEMA_VERSION, "sample_id": sample_id,
                **{k: run.get(k) for k in ("instance", "dir", "media", "algorithms", "own_loads", "episodes", "kit_log")},
                "stages": moved}
    return new, runs


def main(argv=None):
    folders = [Path(f) for f in (sys.argv[1:] if argv is None else argv)]
    if not folders:
        raise SystemExit("usage: python -m sampler.convert <folder>...")
    done = 0
    for folder in folders:
        for p in sorted(folder.glob("*.json")):
            old = json.loads(p.read_text())
            if old.get("schema_version") != 1:
                print(f"[skip] {p.name}: not schema 1", flush=True)
                continue
            keep = folder / "_work" / p.stem / "schema1.json"
            if not keep.exists():
                keep.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(p, keep)
            new, runs = convert(old, p.stem)
            if runs:
                SA.write_run(p, runs)
            SA.write(p, new)
            done += 1
            g, dag = new["goal"] or {}, new["dag"] or {}
            print(f"[convert] {p.name}: {len(new['metadata']['stages'])} ledger entries kept, "
                  f"{len((runs or {}).get('stages') or [])} moved to runs/; initial {len((new['initial'] or {}).get('poses') or {})}"
                  f" poses, goal {len(g.get('poses') or {})} poses, S_ref {(g.get('score') or {}).get('S_ref')}, dag "
                  f"{len(dag.get('edges') or []) if dag else None} edges; runs file {bool(runs)}"
                  f"{', ' + str(len(runs['episodes'])) + ' episode rows' if runs else ''}; media "
                  f"{bool(new['metadata']['media'])}; original -> {keep.relative_to(folder)}", flush=True)
        SA.write_index(folder)
    print(f"[RESULT] PASS {done} file(s) converted in {len(folders)} folder(s)", flush=True)


if __name__ == "__main__":
    main()
