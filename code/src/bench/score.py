"""score stage (Kit-free, GPU) and the fair score the build and the run use (code/frigidaire/docs/exposure.md, fair score).

A dish scores min(1, NJ / ceiling[its arm][its kind]) (NJ = the dish's object New+J at theta 30, dot_rules.score_load's
per-dish mean), 0 when it pools (interior more than 2 mm below its rim); a load's S is the plain mean over its dishes,
S_area the area-weighted mean. The ceilings are the Isaac Sim 6.2 / Warp 1.17 grids' (plans/2026-11-01-merged-benchmark.md):
data/results/dot_rules/ceiling_62/ceilings.json, written by `PYTHONPATH=code/src python3 -m bench.score --write-ceilings` from the
outward and inward grids under data/results/dot_rules/ceiling_62/ (frigidaire_dot_rules_relative.ceilings: the largest
<kind>_obj New+J over every cell inside the arm's disc and every tilt). The 4.5 ceilings stay readable as history in
every 4.5 .relative.json (ceilings_kind) and in their own grids; nothing here falls back to them.

The stage scores the goal's record with frigidaire_dependency_score.py (N 262144, all rules: packed, settled = the built
goal) and stores goal.score = S_ref (the fair S of the built goal), S_area and the per-dish rows; the scores file, N, theta
and the ceilings file go to metadata.provenance.score.
"""
import json
from pathlib import Path

from . import ROOT, rel, run

SCORER = "code/planner/frigidaire/frigidaire_dependency_score.py"
CEILING_DIR = ROOT / "data/results/dot_rules/ceiling_62"
CEILINGS = CEILING_DIR / "ceilings.json"
GRIDS = {"outward": CEILING_DIR / "outward" / "N0262144", "inward": CEILING_DIR / "inward" / "N0262144"}
THETA_DEG, N_FULL = 30., 262144


def ceilings(path=CEILINGS, wait_s=0.):
    """{arm: {kind: provenance with 'value'}} of the 6.2 ceilings file; SystemExit when it is missing (never a default)
    after waiting up to ``wait_s`` for it to appear (a box run computes it beside the first stages)."""
    import time
    path = Path(path)
    deadline = time.monotonic() + float(wait_s or 0.)
    while not path.is_file() and time.monotonic() < deadline:
        time.sleep(20.)
    if not path.is_file():
        raise SystemExit(f"[RESULT] FAIL no 6.2 ceilings at {path} (run the ceiling grids, then -m bench.score --write-ceilings)")
    data = json.loads(path.read_text())
    return data["ceilings"]


def write_ceilings(grids=GRIDS, out=CEILINGS, theta=THETA_DEG):
    """Compute the six ceilings from the 6.2 grids present (frigidaire_dot_rules_relative.ceilings) and write ``out``;
    ``provisional`` is true while a grid of ``grids`` is still missing (the box computes inward first, then outward)."""
    import datetime as dt
    import importlib.util
    spec = importlib.util.spec_from_file_location("relative", ROOT / "code/planner/frigidaire/frigidaire_dot_rules_relative.py")
    R = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(R)
    present = {k: d for k, d in grids.items() if (Path(d) / "grid.json").is_file()}
    if not present:
        raise SystemExit(f"[RESULT] FAIL no 6.2 grid under {CEILING_DIR}")
    loaded = [(R.rel(Path(d) / "grid.json"), json.loads((Path(d) / "grid.json").read_text())) for d in present.values()]
    ceil = R.ceilings(loaded, theta, R.arm_radii())
    meta = [{"grid": name, "warp": (g.get("meta") or {}).get("warp"), "generated_utc": (g.get("meta") or {}).get("generated_utc"),
             "tilts_deg": [min(g["tilts"]), max(g["tilts"])], "N": g["n_points"]} for name, g in loaded]
    import os
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    tmp = Path(out).with_name(Path(out).name + ".tmp")
    tmp.write_text(json.dumps({"theta_deg": theta, "provisional": len(present) < len(grids),
                               "missing": sorted(set(grids) - set(present)), "rule": R.kind_text(ceil), "grids": meta,
                               "ceilings": ceil, "written_utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")},
                              indent=1) + "\n")
    os.replace(tmp, out)                      # atomic: a build waiting on the file never reads half of it
    return ceil


def fair(per_dish, pooling, ceil):
    """The fair score of one state: per dish S_kind (0 when it pools) and ratio (unclipped); S, S_area, S_by_kind.
    ``per_dish`` items carry id, kind (plain or hotec_), arm, area_m2 and NJ."""
    pset = set(pooling or ())
    rows = []
    for p in per_dish:
        kind = p["kind"].replace("hotec_", "")
        ratio = p["NJ"] / ceil[p["arm"]][kind]["value"]
        rows.append({"id": p["id"], "kind": kind, "arm": p["arm"], "area_m2": p["area_m2"], "NJ": p["NJ"],
                     "pools": p["id"] in pset, "ratio_kind": ratio, "S_kind": 0. if p["id"] in pset else min(1., ratio)})
    area = sum(r["area_m2"] for r in rows) or 1.
    kinds = sorted({r["kind"] for r in rows})
    return {"S": sum(r["S_kind"] for r in rows) / max(1, len(rows)),
            "S_area": sum(r["area_m2"] * r["S_kind"] for r in rows) / area,
            "S_by_kind": {k: sum(r["S_kind"] for r in rows if r["kind"] == k) / sum(r["kind"] == k for r in rows) for k in kinds},
            "n_pooling": len(pset), "n_clipped": sum(r["ratio_kind"] > 1 and not r["pools"] for r in rows), "per_dish": rows}


def record_state(rec):
    """per_dish (id, kind, arm, area_m2, NJ) and pooling of a dot_rules.score_load record."""
    return [{**{k: d[k] for k in ("id", "kind", "arm", "area_m2")}, "NJ": d["E"]["NJ"]} for d in rec["per_dish"]], rec["pooling"]


def stage_score(state, params):
    if params.get("score.reference", "kind") == "none":
        state["goal"]["score"] = None
        return state, "SKIP", "score.reference=none"
    ceil = ceilings(params.get("ceilings", CEILINGS))
    goal, prov, work = state["goal"], state["metadata"]["provenance"], Path(params["work"]) / "goal"
    ok, _ = run("py", [SCORER, "--dependency-dir", prov["goal_record"].rsplit("/", 1)[0], "--loads", prov["load_id"],
                       "--N", N_FULL, "--theta", THETA_DEG, "--force"],        # --force: never an earlier build's scores
                work / "logs" / "score.log")
    path = work / f"{prov['load_id']}.scores.json"
    if not (ok and path.is_file()):
        return state, "FAIL", "scorer failed"
    rec = json.loads(path.read_text())
    if not rec.get("settled"):
        return state, "FAIL", "no built goal to score (entries_settled missing)"
    built = fair(*record_state(rec["settled"]), ceil)
    packed = fair(*record_state(rec["packed"]), ceil)
    goal["score"] = {"S_ref": built["S"], "S_area": built["S_area"], "per_dish": built["per_dish"]}
    prov["score"] = {"file": rel(path), "N": rec["N"], "theta_deg": rec["theta_deg"],
                     "ceilings": rel(params.get("ceilings", CEILINGS))}
    return state, "PASS", (f"S_ref {built['S']:.4f} (fair, 6.2 ceilings; S_area {built['S_area']:.4f}, packed "
                           f"{packed['S']:.4f}, {built['n_pooling']} pooling, {built['n_clipped']} clipped)")


def cli(argv=None):
    """--write-ceilings: the ceilings file from the 6.2 grids present (standard library only). In the container, where
    the editable install predates the bench package: code/util/run_py.sh -c "import sys; sys.path.insert(0, 'code/src');
    from bench import score; score.cli(['--write-ceilings'])"."""
    import argparse
    ap = argparse.ArgumentParser(description="Write the 6.2 ceilings file from the grids (standard library only).")
    ap.add_argument("--write-ceilings", action="store_true")
    a = ap.parse_args(argv)
    if a.write_ceilings:
        c = write_ceilings()
        prov = json.loads(CEILINGS.read_text())
        print(f"[RESULT] PASS ceilings{' (provisional, missing ' + ','.join(prov['missing']) + ')' if prov['provisional'] else ''} "
              + "; ".join(f"{arm} " + ", ".join(f"{k} {v['value']:.4f}" for k, v in kinds.items()) for arm, kinds in c.items())
              + f" -> {CEILINGS}", flush=True)


if __name__ == "__main__":
    cli()
